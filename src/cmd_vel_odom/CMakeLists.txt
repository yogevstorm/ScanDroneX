cmake_minimum_required(VERSION 3.8)
project(cmd_vel_odom)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(cmd_vel_odom_node
  src/main.cpp
  src/cmd_vel_odom_node.cpp
)

target_include_directories(cmd_vel_odom_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(cmd_vel_odom_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

add_executable(odom_disagreement_node
  src/odom_disagreement_main.cpp
  src/odom_disagreement_node.cpp
)

target_include_directories(odom_disagreement_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(odom_disagreement_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
)

add_executable(odom_switch_node
  src/odom_switch_main.cpp
  src/odom_switch_node.cpp
)

target_include_directories(odom_switch_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(odom_switch_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2_ros
)

install(TARGETS cmd_vel_odom_node odom_disagreement_node odom_switch_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
