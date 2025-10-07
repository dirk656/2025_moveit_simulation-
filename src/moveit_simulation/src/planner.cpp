#include "moveit_simulation/rh_planner.hpp"
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <iostream>

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode() : Node("rh_planner_node") {
        planner_ = std::make_unique<receding_horizon_planner::RecedingHorizonPlanner>();
        
        trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);
            
        // 定时器示例：定期发布测试轨迹
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&PlannerNode::publishTestTrajectory, this));
            
        RCLCPP_INFO(this->get_logger(), "Receding Horizon Planner Node started");
    }

private:
    void publishTestTrajectory() {
        // 测试路径点
        std::vector<std::vector<double>> path_points = {
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1},
            {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3},
            {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5}
        };
        
        std::vector<double> initial_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        
        try {
            auto trajectory_points = planner_->generateTrajectory(path_points, initial_velocity);
            auto ros_trajectory = convertToROSTrajectory(trajectory_points);
            
            trajectory_pub_->publish(ros_trajectory);
            
            auto stats = planner_->getPlanningStats();
            RCLCPP_INFO(this->get_logger(), 
                       "Generated trajectory with %zu points, calculation time: %.3f ms",
                       trajectory_points.size(), stats.total_calculation_time);
                       
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Trajectory generation failed: %s", e.what());
        }
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
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}