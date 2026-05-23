cmake_minimum_required(VERSION 3.8)
project(navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(navigation_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  visualization_msgs
  nav_msgs
  geometry_msgs
  navigation_msgs
  OpenCV
  Eigen3
  tf2_ros
)

include_directories(
  Utils/include
  MissionLayer/include
  Localization/include
  ObstacleMap/include
  BehaviorLayer/include
  LocalPlannerLayer/include
  MotionPlannerLayer/include
  ControlLayer/include
  ScanLayer/include
  ${EIGEN3_INCLUDE_DIR}
)

set(utils_lib "Utils")
add_library(${utils_lib} SHARED
  Utils/src/ControlUtils.cpp
  Utils/src/DistMap.cpp
  Utils/src/Pid.cpp
)

set(obstacle_map_lib "ObstacleMap")
add_library(${obstacle_map_lib} SHARED
  ObstacleMap/src/ObstacleMap.cpp
)

set(mission_layer_lib "MissionLayer")
add_library(${mission_layer_lib} SHARED
  MissionLayer/src/MissionPath.cpp
  MissionLayer/src/Astar.cpp
)

set(behavior_layer_lib "BehaviorLayer")
add_library(${behavior_layer_lib} SHARED
  BehaviorLayer/src/BehaviorPlanner.cpp
)

set(local_planner_layer_lib "LocalPlannerLayer")
add_library(${local_planner_layer_lib} SHARED
  LocalPlannerLayer/src/LocalPlanner.cpp
  LocalPlannerLayer/src/HybridAstar.cpp
)

set(motion_planner_layer_lib "MotionPlannerLayer")
add_library(${motion_planner_layer_lib} SHARED
  MotionPlannerLayer/src/MotionPlanner.cpp
)

set(control_layer_lib "ControlLayer")
add_library(${control_layer_lib} SHARED
  ControlLayer/src/Control.cpp
)

set(scan_layer_lib "ScanLayer")
add_library(${scan_layer_lib} SHARED
  ScanLayer/src/ScanLayer.cpp
)



# Tell CMake that utils_lib depends on rclcpp (and std_msgs)
ament_target_dependencies(${utils_lib} ${dependencies})
ament_target_dependencies(${obstacle_map_lib} ${dependencies})
target_link_libraries(${obstacle_map_lib} ${utils_lib})
ament_target_dependencies(${mission_layer_lib} ${dependencies})
ament_target_dependencies(${behavior_layer_lib} ${dependencies})
ament_target_dependencies(${local_planner_layer_lib} ${dependencies})
ament_target_dependencies(${motion_planner_layer_lib} ${dependencies})
ament_target_dependencies(${control_layer_lib} ${dependencies})
target_link_libraries(${control_layer_lib} ${utils_lib})

ament_target_dependencies(${scan_layer_lib} ${dependencies})

add_executable(ObstacleMapNode ObstacleMap/src/ObstacleMapNode.cpp ${utils_lib} ${obstacle_map_lib})
ament_target_dependencies(ObstacleMapNode  ${dependencies})
target_link_libraries(ObstacleMapNode ${utils_lib} ${obstacle_map_lib})

add_executable(Localization Localization/src/Localization.cpp ${utils_lib})
ament_target_dependencies(Localization  ${dependencies})
target_link_libraries(Localization ${utils_lib})

add_executable(MissionPathNode MissionLayer/src/MissionPathNode.cpp ${utils_lib} ${mission_layer_lib})
ament_target_dependencies(MissionPathNode  ${dependencies})
target_link_libraries(MissionPathNode ${utils_lib} ${mission_layer_lib})

add_executable(BehaviorNode BehaviorLayer/src/BehaviorNode.cpp ${utils_lib} ${behavior_layer_lib})
ament_target_dependencies(BehaviorNode  ${dependencies})
target_link_libraries(BehaviorNode ${utils_lib} ${behavior_layer_lib})

add_executable(LocalPlannerNode LocalPlannerLayer/src/LocalPlannerNode.cpp ${utils_lib} ${local_planner_layer_lib})
ament_target_dependencies(LocalPlannerNode  ${dependencies})
target_link_libraries(LocalPlannerNode ${utils_lib} ${local_planner_layer_lib})

add_executable(MotionPlannerNode MotionPlannerLayer/src/MotionPlannerNode.cpp ${utils_lib} ${motion_planner_layer_lib})
ament_target_dependencies(MotionPlannerNode  ${dependencies})
target_link_libraries(MotionPlannerNode ${utils_lib} ${motion_planner_layer_lib})

add_executable(ControlNode ControlLayer/src/ControlNode.cpp ${utils_lib} ${control_layer_lib})
ament_target_dependencies(ControlNode  ${dependencies})
target_link_libraries(ControlNode ${utils_lib} ${control_layer_lib})

add_executable(ScanNode ScanLayer/src/ScanNode.cpp)
ament_target_dependencies(ScanNode ${dependencies})




install(TARGETS
  Localization
  ObstacleMapNode
  MissionPathNode
  BehaviorNode
  LocalPlannerNode
  MotionPlannerNode
  ControlNode
  ScanNode
  DESTINATION  lib/${PROJECT_NAME})

install(DIRECTORY launch config
DESTINATION share/${PROJECT_NAME})

ament_package()
