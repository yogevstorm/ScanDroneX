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

private:
  std::mt19937 m_rng{std::random_device{}()};
};
