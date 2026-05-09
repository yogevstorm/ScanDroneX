#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <random>
#include <vector>

class ScanLayer
{
public:
  // Samples num_candidates random unknown cells and returns the one farthest
  // from last_goal. Falls back to a purely random pick when has_last_goal is
  // false or only one unknown cell remains. Returns false if no unknown cells exist.
  bool FindRandomGoal(const nav_msgs::msg::OccupancyGrid & map,
                      geometry_msgs::msg::PoseStamped & goal,
                      bool has_last_goal,
                      const geometry_msgs::msg::PoseStamped & last_goal,
                      int num_candidates);

  // Finds the `attempt`-th closest unknown cell to map corner `corner_index`
  // (0 = closest, 1 = next closest, …), allowing callers to spiral outward on retry.
  // Corners: 0=bottom-left, 1=bottom-right, 2=top-right, 3=top-left.
  // Returns false if fewer than (attempt+1) unknown cells exist in the map.
  bool FindCornerGoal(const nav_msgs::msg::OccupancyGrid & map,
                      geometry_msgs::msg::PoseStamped & goal,
                      int corner_index,
                      int attempt = 0);

private:
  std::mt19937 m_rng{std::random_device{}()};
};
