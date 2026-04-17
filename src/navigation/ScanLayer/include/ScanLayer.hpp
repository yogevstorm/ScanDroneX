#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <random>
#include <vector>

class ScanLayer
{
public:
  // Picks a random unknown cell (value == -1) from the occupancy grid and
  // returns its world-frame pose. Returns false if no unknown cells exist.
  bool FindRandomGoal(const nav_msgs::msg::OccupancyGrid & map,
                      geometry_msgs::msg::PoseStamped & goal);

private:
  std::mt19937 m_rng{std::random_device{}()};
};
