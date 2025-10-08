#include "moveit_simulation/rh_planner.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <memory>
#include <vector>
#include <string>
#include <chrono>

class ComparisonNode : public rclcpp::Node {
public:
    ComparisonNode() : Node("comparison_node") {
        // 初始化规划器
        rh_planner_ = std::make_unique<receding_horizon_planner::RecedingHorizonPlanner>();
        
        // 创建MoveGroup接口
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            std::shared_ptr<rclcpp::Node>(this, [](auto*) {}), "panda_arm");
            
        // 设置规划器参数
        setupPlannerParameters();
        
        // 创建轨迹发布器
        rh_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/rh_joint_trajectory", 10);
            
        traditional_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/traditional_joint_trajectory", 10);
            
        // 创建速度数据发布器（用于rqt绘图）
        velocity_data_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/velocity_comparison_data", 10);
            
        // 创建服务
        compare_service_ = this->create_service<std_srvs::srv::Trigger>(
            "~/compare_planners",
            std::bind(&ComparisonNode::handleCompareRequest, this,
                     std::placeholders::_1, std::placeholders::_2));
                     
        // 创建定时器用于发布速度数据
        data_publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ComparisonNode::publishVelocityData, this));
            
        RCLCPP_INFO(this->get_logger(), "Planner Comparison Node started");
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
        
        rh_planner_->setRobotParameters(max_velocities, max_accelerations);
        RCLCPP_INFO(this->get_logger(), "Planner parameters configured for Panda arm");
    }
    
    void handleCompareRequest(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        
        RCLCPP_INFO(this->get_logger(), "Received planner comparison request");
        
        try {
            // 定义测试路径
            std::vector<std::vector<double>> path_points = {
                {0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.0},  // 起始位置
                {0.5, 0.5, 0.5, -1.0, 0.5, 1.0, 0.5},  // 中间点1
                {1.0, 1.0, 1.0, -0.5, 1.0, 0.5, 1.0},  // 中间点2
                {1.5, 1.5, 1.5, 0.0, 1.5, 0.0, 1.5}   // 目标位置
            };
            
            std::vector<double> initial_velocity(7, 0.0);
            
            // 测试滚动时域规划器
            auto rh_start_time = std::chrono::high_resolution_clock::now();
            auto rh_trajectory = rh_planner_->generateTrajectory(path_points, initial_velocity);
            auto rh_end_time = std::chrono::high_resolution_clock::now();
            auto rh_duration = std::chrono::duration_cast<std::chrono::microseconds>(
                rh_end_time - rh_start_time);
                
            // 测试传统规划器
            auto traditional_start_time = std::chrono::high_resolution_clock::now();
            auto traditional_trajectory = generateTraditionalTrajectory(path_points);
            auto traditional_end_time = std::chrono::high_resolution_clock::now();
            auto traditional_duration = std::chrono::duration_cast<std::chrono::microseconds>(
                traditional_end_time - traditional_start_time);
            
            // 存储轨迹数据用于显示
            rh_trajectory_ = rh_trajectory;
            traditional_trajectory_ = traditional_trajectory;
            
            // 发布轨迹
            auto rh_ros_trajectory = convertToROSTrajectory(rh_trajectory);
            auto traditional_ros_trajectory = convertToROSTrajectory(traditional_trajectory);
            
            rh_trajectory_pub_->publish(rh_ros_trajectory);
            traditional_trajectory_pub_->publish(traditional_ros_trajectory);
            
            // 计算统计信息
            double rh_total_time = rh_trajectory.empty() ? 0.0 : rh_trajectory.back().time_from_start;
            double traditional_total_time = traditional_trajectory.empty() ? 0.0 : traditional_trajectory.back().time_from_start;
            
            // 计算最大速度
            double rh_max_speed = calculateMaxSpeed(rh_trajectory);
            double traditional_max_speed = calculateMaxSpeed(traditional_trajectory);
            
            // 准备响应消息
            std::stringstream result;
            result << "Comparison Results:\n";
            result << "Receding Horizon Planner:\n";
            result << "  - Planning Time: " << (rh_duration.count() / 1000.0) << " ms\n";
            result << "  - Trajectory Points: " << rh_trajectory.size() << "\n";
            result << "  - Total Execution Time: " << rh_total_time << " s\n";
            result << "  - Max Speed: " << rh_max_speed << " rad/s\n";
            
            result << "Traditional Planner:\n";
            result << "  - Planning Time: " << (traditional_duration.count() / 1000.0) << " ms\n";
            result << "  - Trajectory Points: " << traditional_trajectory.size() << "\n";
            result << "  - Total Execution Time: " << traditional_total_time << " s\n";
            result << "  - Max Speed: " << traditional_max_speed << " rad/s\n";
            
            result << "Speedup: " << (traditional_duration.count() / (double)rh_duration.count()) << "x\n";
            
            response->success = true;
            response->message = result.str();
            
            RCLCPP_INFO(this->get_logger(), "Planner comparison completed");
            
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Comparison failed: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "Comparison failed: %s", e.what());
        }
    }
    
    std::vector<receding_horizon_planner::TrajectoryPoint> generateTraditionalTrajectory(
        const std::vector<std::vector<double>>& path_points) {
        
        std::vector<receding_horizon_planner::TrajectoryPoint> trajectory;
        
        if (path_points.size() < 2) {
            return trajectory;
        }
        
        // 简单的线性插值轨迹生成（传统方法）
        double total_time = 5.0; // 固定总时间
        double dt = 0.01; // 10ms时间步长
        size_t num_segments = path_points.size() - 1;
        double segment_time = total_time / num_segments;
        
        double current_time = 0.0;
        
        for (size_t seg_idx = 0; seg_idx < num_segments; ++seg_idx) {
            const auto& start_point = path_points[seg_idx];
            const auto& end_point = path_points[seg_idx + 1];
            
            size_t num_points_in_segment = static_cast<size_t>(segment_time / dt);
            
            for (size_t i = 0; i < num_points_in_segment; ++i) {
                double t = static_cast<double>(i) / num_points_in_segment;
                
                receding_horizon_planner::TrajectoryPoint point(7);
                point.time_from_start = current_time + t * segment_time;
                
                // 线性位置插值
                for (size_t j = 0; j < 7; ++j) {
                    point.positions[j] = start_point[j] + t * (end_point[j] - start_point[j]);
                }
                
                // 恒定速度（基于距离和时间）
                for (size_t j = 0; j < 7; ++j) {
                    double distance = end_point[j] - start_point[j];
                    point.velocities[j] = distance / segment_time;
                }
                
                // 零加速度
                std::fill(point.accelerations.begin(), point.accelerations.end(), 0.0);
                
                trajectory.push_back(point);
            }
            
            current_time += segment_time;
        }
        
        // 添加最后一个点
        if (!path_points.empty()) {
            receding_horizon_planner::TrajectoryPoint final_point(7);
            final_point.time_from_start = current_time;
            final_point.positions = path_points.back();
            std::fill(final_point.velocities.begin(), final_point.velocities.end(), 0.0);
            std::fill(final_point.accelerations.begin(), final_point.accelerations.end(), 0.0);
            trajectory.push_back(final_point);
        }
        
        return trajectory;
    }
    
    void publishVelocityData() {
        if (rh_trajectory_.empty() || traditional_trajectory_.empty()) {
            return;
        }
        
        std_msgs::msg::Float64MultiArray velocity_data;
        
        // 发布最后关节的速度数据用于rqt绘图
        size_t last_joint_idx = 6; // panda_joint7
        
        // 添加时间戳
        velocity_data.data.push_back(this->now().seconds());
        
        // 添加滚动时域规划器的速度
        if (!rh_trajectory_.empty()) {
            velocity_data.data.push_back(rh_trajectory_.back().velocities[last_joint_idx]);
        } else {
            velocity_data.data.push_back(0.0);
        }
        
        // 添加传统规划器的速度
        if (!traditional_trajectory_.empty()) {
            velocity_data.data.push_back(traditional_trajectory_.back().velocities[last_joint_idx]);
        } else {
            velocity_data.data.push_back(0.0);
        }
        
        velocity_data_pub_->publish(velocity_data);
    }
    
    double calculateMaxSpeed(const std::vector<receding_horizon_planner::TrajectoryPoint>& trajectory) {
        double max_speed = 0.0;
        
        for (const auto& point : trajectory) {
            for (double speed : point.velocities) {
                if (std::abs(speed) > max_speed) {
                    max_speed = std::abs(speed);
                }
            }
        }
        
        return max_speed;
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
    
    std::unique_ptr<receding_horizon_planner::RecedingHorizonPlanner> rh_planner_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr rh_trajectory_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traditional_trajectory_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_data_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr compare_service_;
    rclcpp::TimerBase::SharedPtr data_publish_timer_;
    
    std::vector<receding_horizon_planner::TrajectoryPoint> rh_trajectory_;
    std::vector<receding_horizon_planner::TrajectoryPoint> traditional_trajectory_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ComparisonNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
