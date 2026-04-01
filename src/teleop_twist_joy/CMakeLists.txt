cmake_minimum_required(VERSION 3.5)
project(teleop_twist_joy)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

# Shared library (component)
add_library(${PROJECT_NAME} SHARED
  src/teleop_twist_joy.cpp
)
target_compile_definitions(${PROJECT_NAME} PRIVATE TELEOP_TWIST_JOY_BUILDING_DLL)
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  rclcpp
  rclcpp_components
  rcutils
  sensor_msgs
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "teleop_twist_joy::TeleopTwistJoy"
  EXECUTABLE teleop_node
)

install(TARGETS ${PROJECT_NAME} teleop_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  geometry_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
)

ament_package()
