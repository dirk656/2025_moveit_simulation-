#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  // 创建节点
  auto node = std::make_shared<rclcpp::Node>("simple_moveit_test");
  auto logger = node->get_logger();
  
  RCLCPP_INFO(logger, "Starting MoveIt test program");
  
  try {
    // 连接到已经运行的MoveIt系统
    auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "panda_arm");
    
    RCLCPP_INFO(logger, "MoveGroupInterface created successfully");
    RCLCPP_INFO(logger, "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
    RCLCPP_INFO(logger, "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
    
    // 设置目标姿态
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.28;
    target_pose.position.y = -0.2;
    target_pose.position.z = 0.5;
    
    move_group_interface.setPoseTarget(target_pose);
    RCLCPP_INFO(logger, "Target pose set");
    
    // 创建规划
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success) {
      RCLCPP_INFO(logger, "Planning successful!");
      // 执行规划
      move_group_interface.execute(plan);
      RCLCPP_INFO(logger, "Execution completed!");
    } else {
      RCLCPP_ERROR(logger, "Planning failed!");
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Exception occurred: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}
