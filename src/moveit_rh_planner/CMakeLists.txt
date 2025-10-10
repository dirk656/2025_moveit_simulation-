cmake_minimum_required(VERSION 3.16)
project(moveit_rh_planner)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(class_loader REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Include directories
include_directories(include)

# Create library
add_library(${PROJECT_NAME}
  src/rh_planner_manager.cpp
  src/rh_planner.cpp
)

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  pluginlib
  rclcpp
  rclcpp_components
  tf2_eigen
  class_loader
  geometry_msgs
  sensor_msgs
  trajectory_msgs
)

# Install targets
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install plugin description
install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  pluginlib
  rclcpp
  rclcpp_components
  tf2_eigen
  class_loader
  geometry_msgs
  sensor_msgs
  trajectory_msgs
)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
pluginlib_export_plugin_description_file(moveit_rh_planner plugin_description.xml)
ament_package()