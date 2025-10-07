#include "moveit_simulation/rh_planner.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <memory>
#include <vector>
#include <string>

class RHMoveItIntegration : public rclcpp::Node {
public:
    RHMoveItIntegration() : Node("rh_moveit_integration") {
        // 初始化规划器
        planner_ = std::make_unique<receding_horizon_planner::RecedingHorizonPlanner>();
        
        // 创建MoveGroup接口
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            std::shared_ptr<rclcpp::Node>(this, [](auto*) {}), "panda_arm");
            
        // 设置规划器参数
        setupPlannerParameters();
        
        // 创建轨迹发布器
        trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);
            
        // 创建服务
        plan_service_ = this->create_service<std_srvs::srv::Trigger>(
            "~/plan_with_rh",
            std::bind(&RHMoveItIntegration::handlePlanRequest, this,
                     std::placeholders::_1, std::placeholders::_2));
                     
        // 创建目标姿态订阅器
        target_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "~/target_pose", 10,
            std::bind(&RHMoveItIntegration::handleTargetPose, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Receding Horizon MoveIt Integration Node started");
    }

private:
    void setupPlannerParameters() {
        // 设置Panda机械臂的关节限制
        std::vector<double> max_velocities = {
            2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100  // rad/s
        };
        
        std::vector<double> max_accelerations = {
            15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0  // rad/s²
        };
        
        planner_->setRobotParameters(max_velocities, max_accelerations);
        RCLCPP_INFO(this->get_logger(), "Planner parameters configured for Panda arm");
    }
    
    void handlePlanRequest(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        
        RCLCPP_INFO(this->get_logger(), "Received planning request");
        
        try {
            // 检查是否有目标姿态
            if (!target_pose_received_) {
                response->success = false;
                response->message = "No target pose received. Please send a target pose first.";
                RCLCPP_ERROR(this->get_logger(), "No target pose available");
                return;
            }
            
            // 获取当前关节状态
            auto current_state = move_group_interface_->getCurrentState();
            std::vector<double> current_joint_positions;
            current_state->copyJointGroupPositions("panda_arm", current_joint_positions);
            
            // 使用存储的目标姿态进行逆运动学计算
            bool found_ik = current_state->setFromIK(
                move_group_interface_->getRobotModel()->getJointModelGroup("panda_arm"),
                target_pose_);
                
            if (!found_ik) {
                response->success = false;
                response->message = "Failed to compute inverse kinematics for target pose";
                RCLCPP_ERROR(this->get_logger(), "IK computation failed for target pose");
                return;
            }
            
            // 获取目标关节位置
            std::vector<double> target_joint_positions;
            current_state->copyJointGroupPositions("panda_arm", target_joint_positions);
            
            // 生成路径点（简单的直线插值）
            std::vector<std::vector<double>> path_points = generatePathPoints(
                current_joint_positions, target_joint_positions, 10);
                
            // 使用滚动时域规划器生成轨迹
            std::vector<double> initial_velocity(7, 0.0);  // 从静止开始
            auto trajectory_points = planner_->generateTrajectory(path_points, initial_velocity);
            
            // 转换为ROS轨迹消息
            auto ros_trajectory = convertToROSTrajectory(trajectory_points);
            
            // 发布轨迹
            trajectory_pub_->publish(ros_trajectory);
            
            // 获取统计信息
            auto stats = planner_->getPlanningStats();
            
            response->success = true;
            response->message = "Trajectory generated successfully with " + 
                               std::to_string(trajectory_points.size()) + 
                               " points, calculation time: " + 
                               std::to_string(stats.total_calculation_time) + " ms";
                               
            RCLCPP_INFO(this->get_logger(), "Trajectory generation completed: %s", 
                       response->message.c_str());
                       
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Planning failed: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "Planning failed: %s", e.what());
        }
    }
    
    void handleTargetPose(const geometry_msgs::msg::Pose::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received target pose: [%.3f, %.3f, %.3f]", 
                   msg->position.x, msg->position.y, msg->position.z);
                   
        // 存储目标姿态
        target_pose_ = *msg;
        target_pose_received_ = true;
        
        // 设置目标姿态到MoveGroup
        move_group_interface_->setPoseTarget(*msg);
    }
    
    std::vector<std::vector<double>> generatePathPoints(
        const std::vector<double>& start,
        const std::vector<double>& end,
        int num_points) {
        
        std::vector<std::vector<double>> path_points;
        
        for (int i = 0; i <= num_points; ++i) {
            double t = static_cast<double>(i) / num_points;
            std::vector<double> point;
            
            for (size_t j = 0; j < start.size(); ++j) {
                point.push_back(start[j] + t * (end[j] - start[j]));
            }
            
            path_points.push_back(point);
        }
        
        return path_points;
    }
    
    trajectory_msgs::msg::JointTrajectory convertToROSTrajectory(
        const std::vector<receding_horizon_planner::TrajectoryPoint>& points) {
        
        trajectory_msgs::msg::JointTrajectory trajectory;
        trajectory.joint_names = {
            "panda_joint1", "panda_joint2", "panda_joint3",
            "panda_joint4", "panda_joint5", "panda_joint6", 
            "panda_joint7"
        };
        
        for (const auto& point : points) {
            trajectory_msgs::msg::JointTrajectoryPoint ros_point;
            ros_point.positions = point.positions;
            ros_point.velocities = point.velocities;
            ros_point.accelerations = point.accelerations;
            ros_point.time_from_start.sec = static_cast<int32_t>(point.time_from_start);
            ros_point.time_from_start.nanosec = static_cast<int32_t>(
                (point.time_from_start - static_cast<int>(point.time_from_start)) * 1e9);
                
            trajectory.points.push_back(ros_point);
        }
        
        return trajectory;
    }
    
    std::unique_ptr<receding_horizon_planner::RecedingHorizonPlanner> planner_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_service_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_pose_sub_;
    geometry_msgs::msg::Pose target_pose_;
    bool target_pose_received_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RHMoveItIntegration>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
