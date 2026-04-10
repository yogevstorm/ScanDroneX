#pragma once
#include <vector>
#include <cmath>
#include <memory>
#include <map>

class LaneNode
{
  //A node class for A* Pathfinding
  public:
    int parent_ind = -1;
    int ind = 0;
    std::tuple<float, float> cluster;
    int lane_idx = 0;
    float g = 0.0;
    float h = 0.0;
    float f = 0.0;
};


