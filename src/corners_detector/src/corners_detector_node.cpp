#include <cmath>
#include <vector>
#include "corners_detector/corners_detector_node.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace
{

struct Point2D { double x, y; };
using Segment = std::vector<Point2D>;

Point2D polarToCartesian(double r, double angle)
{
  return {r * std::cos(angle), r * std::sin(angle)};
}

// PCA-based principal direction of a point cluster
std::pair<double, double> fitLineDir(const Segment & seg)
{
  double cx = 0.0, cy = 0.0;
  for (const auto & p : seg) { cx += p.x; cy += p.y; }
  cx /= seg.size(); cy /= seg.size();

  double sxx = 0.0, sxy = 0.0, syy = 0.0;
  for (const auto & p : seg) {
    double dx = p.x - cx, dy = p.y - cy;
    sxx += dx * dx; sxy += dx * dy; syy += dy * dy;
  }

  // Angle of the principal eigenvector of the 2x2 covariance matrix
  double th = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
  return {std::cos(th), std::sin(th)};
}

}  // namespace

CornersDetectorNode::CornersDetectorNode()
: Node("corners_detector_node")
{
  declare_parameter("gap_threshold", 0.3);
  declare_parameter("min_segment_points", 3);
  declare_parameter("corner_angle_deg", 20.0);

  gap_threshold_       = get_parameter("gap_threshold").as_double();
  min_segment_points_  = get_parameter("min_segment_points").as_int();
  corner_angle_rad_    = get_parameter("corner_angle_deg").as_double() * M_PI / 180.0;

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/simple_drone/scan", 10,
    std::bind(&CornersDetectorNode::scanCallback, this, std::placeholders::_1));

  count_pub_   = create_publisher<std_msgs::msg::Int32>("/corners_detector/count", 10);
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/corners_detector/markers", 10);

  RCLCPP_INFO(get_logger(), "CornersDetectorNode started (gap=%.2f m, min_pts=%d, angle=%.1f deg)",
    gap_threshold_, min_segment_points_, corner_angle_rad_ * 180.0 / M_PI);
}

void CornersDetectorNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // 1. Convert polar → Cartesian, discard invalid returns
  std::vector<Point2D> pts;
  pts.reserve(msg->ranges.size());
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    float r = msg->ranges[i];
    if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) {
      continue;
    }
    double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
    pts.push_back(polarToCartesian(r, angle));
  }

  if (pts.size() < 2) {
    return;
  }

  // 2. Split into contiguous segments separated by gaps
  std::vector<Segment> segments;
  Segment current;
  current.push_back(pts[0]);

  for (size_t i = 1; i < pts.size(); ++i) {
    double dx = pts[i].x - pts[i - 1].x;
    double dy = pts[i].y - pts[i - 1].y;
    if (std::sqrt(dx * dx + dy * dy) > gap_threshold_) {
      if (static_cast<int>(current.size()) >= min_segment_points_) {
        segments.push_back(std::move(current));
      }
      current.clear();
    }
    current.push_back(pts[i]);
  }
  if (static_cast<int>(current.size()) >= min_segment_points_) {
    segments.push_back(std::move(current));
  }

  // 3. Find corners at boundaries between adjacent segments
  std::vector<Point2D> corners;

  for (size_t i = 0; i + 1 < segments.size(); ++i) {
    auto [dx1, dy1] = fitLineDir(segments[i]);
    auto [dx2, dy2] = fitLineDir(segments[i + 1]);

    // Angle between the two lines (always in [0, pi/2])
    double dot   = std::clamp(std::abs(dx1 * dx2 + dy1 * dy2), 0.0, 1.0);
    double angle = std::acos(dot);

    if (angle > corner_angle_rad_) {
      const auto & p1 = segments[i].back();
      const auto & p2 = segments[i + 1].front();
      corners.push_back({(p1.x + p2.x) * 0.5, (p1.y + p2.y) * 0.5});
    }
  }

  // 4. Transform corners from the scan frame into the map frame
  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_buffer_->lookupTransform(
      "map", msg->header.frame_id, msg->header.stamp,
      rclcpp::Duration::from_seconds(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Cannot transform %s → map: %s", msg->header.frame_id.c_str(), ex.what());
    return;
  }

  std::vector<Point2D> map_corners;
  map_corners.reserve(corners.size());
  for (const auto & c : corners) {
    geometry_msgs::msg::PointStamped in_pt, out_pt;
    in_pt.header   = msg->header;
    in_pt.point.x  = c.x;
    in_pt.point.y  = c.y;
    in_pt.point.z  = 0.0;
    tf2::doTransform(in_pt, out_pt, tf_stamped);
    map_corners.push_back({out_pt.point.x, out_pt.point.y});
  }

  // 5. Publish corner count
  std_msgs::msg::Int32 count_msg;
  count_msg.data = static_cast<int>(map_corners.size());
  count_pub_->publish(count_msg);

  // 6. Publish RViz markers (in map frame)
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker delete_marker;
  delete_marker.header.frame_id = "map";
  delete_marker.header.stamp    = msg->header.stamp;
  delete_marker.action          = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_marker);

  for (size_t i = 0; i < map_corners.size(); ++i) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id    = "map";
    m.header.stamp       = msg->header.stamp;
    m.ns                 = "corners";
    m.id                 = static_cast<int>(i);
    m.type               = visualization_msgs::msg::Marker::SPHERE;
    m.action             = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x    = map_corners[i].x;
    m.pose.position.y    = map_corners[i].y;
    m.pose.position.z    = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.3;
    m.color.r = 0.0f; m.color.g = 0.0f; m.color.b = 1.0f; m.color.a = 1.0f;
    m.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker_array.markers.push_back(m);
  }

  markers_pub_->publish(marker_array);

  RCLCPP_DEBUG(get_logger(), "Detected %zu corner(s) from %zu segments",
    map_corners.size(), segments.size());
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CornersDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
