#include <queue>
#include <vector>
#include <algorithm>
#include "openings_detector/openings_detector_node.hpp"
#include <visualization_msgs/msg/marker.hpp>

OpeningsDetectorNode::OpeningsDetectorNode()
: Node("openings_detector_node")
{
  declare_parameter("min_opening_width", 10);
  min_opening_width_ = get_parameter("min_opening_width").as_int();

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 1,
    std::bind(&OpeningsDetectorNode::mapCallback, this, std::placeholders::_1));

  goal_pub_    = create_publisher<geometry_msgs::msg::PoseStamped>("/openings_detector/goal", 1);
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/openings_detector/markers", 10);

  RCLCPP_INFO(get_logger(), "OpeningsDetectorNode started (min_opening_width=%d cells)",
    min_opening_width_);
}

void OpeningsDetectorNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  const int   W   = static_cast<int>(msg->info.width);
  const int   H   = static_cast<int>(msg->info.height);
  const float res = msg->info.resolution;
  const float ox  = msg->info.origin.position.x;
  const float oy  = msg->info.origin.position.y;

  // Step 1: find frontier cells — free (0) cells that touch at least one unknown (-1) cell
  const int dx4[4] = { 1, -1,  0,  0};
  const int dy4[4] = { 0,  0,  1, -1};

  std::vector<bool> is_frontier(W * H, false);
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      if (msg->data[y * W + x] != 0) continue;
      for (int d = 0; d < 4; ++d) {
        int nx = x + dx4[d];
        int ny = y + dy4[d];
        if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
        if (msg->data[ny * W + nx] == -1) {
          is_frontier[y * W + x] = true;
          break;
        }
      }
    }
  }

  // Step 2: group frontier cells into connected components via BFS (4-connectivity)
  struct Opening {
    float cx, cy;
    size_t size;
  };

  std::vector<bool> visited(W * H, false);
  std::vector<Opening> openings;

  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      int start = y * W + x;
      if (!is_frontier[start] || visited[start]) continue;

      std::queue<int> q;
      q.push(start);
      visited[start] = true;

      float sum_wx = 0.0f, sum_wy = 0.0f;
      size_t count = 0;

      while (!q.empty()) {
        int cur = q.front(); q.pop();
        int cx  = cur % W;
        int cy  = cur / W;

        sum_wx += ox + (cx + 0.5f) * res;
        sum_wy += oy + (cy + 0.5f) * res;
        ++count;

        for (int d = 0; d < 4; ++d) {
          int nx   = cx + dx4[d];
          int ny   = cy + dy4[d];
          if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
          int nidx = ny * W + nx;
          if (!is_frontier[nidx] || visited[nidx]) continue;
          visited[nidx] = true;
          q.push(nidx);
        }
      }

      if (static_cast<int>(count) >= min_opening_width_) {
        openings.push_back({sum_wx / count, sum_wy / count, count});
      }
    }
  }

  // Step 3: publish visualization markers for all openings
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker delete_marker;
  delete_marker.header.frame_id = "map";
  delete_marker.header.stamp    = msg->header.stamp;
  delete_marker.action          = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_marker);

  if (openings.empty()) {
    markers_pub_->publish(marker_array);
    RCLCPP_DEBUG(get_logger(), "No openings detected");
    return;
  }

  // Step 4: select opening with lowest centroid Y (westernmost in map frame)
  auto best_it = std::min_element(openings.begin(), openings.end(),
    [](const Opening & a, const Opening & b) { return a.cy < b.cy; });

  // Step 5: publish the selected opening as goal (in free space at frontier centroid)
  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp    = msg->header.stamp;
  goal.header.frame_id = "map";
  goal.pose.position.x = best_it->cx;
  goal.pose.position.y = best_it->cy;
  goal.pose.position.z = 0.0;
  goal.pose.orientation.w = 1.0;
  goal_pub_->publish(goal);

  for (size_t i = 0; i < openings.size(); ++i) {
    bool selected = (openings.data() + i == &(*best_it));

    visualization_msgs::msg::Marker m;
    m.header.frame_id    = "map";
    m.header.stamp       = msg->header.stamp;
    m.ns                 = "openings";
    m.id                 = static_cast<int>(i);
    m.type               = visualization_msgs::msg::Marker::SPHERE;
    m.action             = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x    = openings[i].cx;
    m.pose.position.y    = openings[i].cy;
    m.pose.position.z    = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.4;
    // orange = selected, green = other
    if (selected) {
      m.color.r = 1.0f; m.color.g = 0.5f; m.color.b = 0.0f; m.color.a = 1.0f;
    } else {
      m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.0f; m.color.a = 0.8f;
    }
    m.lifetime = rclcpp::Duration::from_seconds(2.0);
    marker_array.markers.push_back(m);
  }

  markers_pub_->publish(marker_array);

  RCLCPP_DEBUG(get_logger(), "Detected %zu opening(s), selected westernmost at (%.2f, %.2f)",
    openings.size(), best_it->cx, best_it->cy);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpeningsDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
