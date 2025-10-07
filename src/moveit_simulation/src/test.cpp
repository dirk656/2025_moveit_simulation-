#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <geometry_msgs/msg/pose.hpp>


int main(int argc ,char**argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("moveit_test_node");
    auto logger = rclcpp::get_logger("moveit_test");
    

    
    auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "panda_arm");

auto const target_pose = []{
  geometry_msgs::msg::Pose msg;
  msg.orientation.w = 1.0;
  msg.position.x = 0.28;
  msg.position.y = -0.2;
  msg.position.z = 0.5;
  return msg;
}();
move_group_interface.setPoseTarget(target_pose);

// 创建到目标姿态的规划
moveit::planning_interface::MoveGroupInterface::Plan plan;
auto const success = static_cast<bool>(move_group_interface.plan(plan));

// 执行规划
if(success) {
  move_group_interface.execute(plan);
  RCLCPP_INFO(logger,"planning success");
} else {
  RCLCPP_ERROR(logger, "规划失败!");
}
rclcpp::spin(node);
rclcpp::shutdown();
return 0;
}
