#pragma once
#include "navigation_msgs/msg/world_point.hpp"


class Node
{
  //A node class for A* Pathfinding
  public:
    int parent_ind = -1;
    int ind = 0;
    navigation_msgs::msg::WorldPoint wpoint;
    std::vector<int> mpoint;
    float g = 0.0;
    float h = 0.0;
    float f = 0.0;
    float segment_len = 0.0;
    int segments_num = 0;
    
};


