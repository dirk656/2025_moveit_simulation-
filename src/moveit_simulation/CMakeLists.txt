cmake_minimum_required(VERSION 3.8)
project(moveit_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置包含目录
include_directories(include)

# 添加可执行文件
add_executable(moveit_test src/test.cpp)

add_executable(planner_node src/planner.cpp
src/rh_planner.cpp
)

add_executable(rh_moveit_integration src/rh_moveit_integration.cpp
src/rh_planner.cpp
)

add_executable(comparison_node src/comparison_node.cpp
src/rh_planner.cpp
)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# 依赖关系
ament_target_dependencies(moveit_test
  "rclcpp"
  "moveit_core"
  "trajectory_msgs"
  "sensor_msgs"
  "moveit_ros_planning_interface"
)

ament_target_dependencies(planner_node
  "rclcpp"
  "moveit_core"
  "trajectory_msgs"
  "sensor_msgs"
  "moveit_ros_planning_interface"
  "geometry_msgs"
)

ament_target_dependencies(rh_moveit_integration
  "rclcpp"
  "moveit_core"
  "trajectory_msgs"
  "sensor_msgs"
  "moveit_ros_planning_interface"
  "geometry_msgs"
  "std_srvs"
)

ament_target_dependencies(comparison_node
  "rclcpp"
  "moveit_core"
  "trajectory_msgs"
  "sensor_msgs"
  "moveit_ros_planning_interface"
  "geometry_msgs"
  "std_srvs"
  "std_msgs"
  "visualization_msgs"
)


# 安装目标
install(TARGETS
  moveit_test
  planner_node
  rh_moveit_integration
  comparison_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装Python脚本
install(PROGRAMS
  scripts/velocity_plotter.py
  scripts/trigger_comparison.py
  scripts/test_comparison_system.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
