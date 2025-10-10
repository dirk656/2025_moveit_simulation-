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

    // 设置使用RH规划器
    move_group_interface.setPlannerId("RH");
    
    // 设置规划器参数
    move_group_interface.setPlanningTime(5.0);
    move_group_interface.setNumPlanningAttempts(3);
    move_group_interface.setMaxVelocityScalingFactor(0.1);
    move_group_interface.setMaxAccelerationScalingFactor(0.1);

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
  RCLCPP_INFO(logger,"RH规划器规划成功!");
} else {
  RCLCPP_ERROR(logger, "RH规划器规划失败!");
}
rclcpp::spin(node);
rclcpp::shutdown();
return 0;
}
