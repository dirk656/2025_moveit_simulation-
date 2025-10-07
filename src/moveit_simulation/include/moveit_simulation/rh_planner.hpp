#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <memory>

namespace receding_horizon_planner {

// TVP参数结构体
struct TVPParameters {
    double wm = 0.0;  // 最大速度
    double a = 0.0;   // 加速度
    double t1 = 0.0;  // 加速结束时间
    double t2 = 0.0;  // 减速开始时间
    double tf = 0.0;  // 总时间
    
    TVPParameters() = default;
    TVPParameters(double wm_val, double a_val, double t1_val, double t2_val, double tf_val)
        : wm(wm_val), a(a_val), t1(t1_val), t2(t2_val), tf(tf_val) {}
};

// 轨迹点结构体
struct TrajectoryPoint {
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> accelerations;
    double time_from_start;
    
    TrajectoryPoint(size_t num_joints) 
        : positions(num_joints, 0.0), velocities(num_joints, 0.0), 
          accelerations(num_joints, 0.0), time_from_start(0.0) {}
};

class RecedingHorizonPlanner {
public:
    RecedingHorizonPlanner();
    
    // 设置机器人参数
    void setRobotParameters(const std::vector<double>& max_velocities, 
                           const std::vector<double>& max_accelerations);
    
    // 主规划接口
    std::vector<TrajectoryPoint> generateTrajectory(
        const std::vector<std::vector<double>>& path_points,
        const std::vector<double>& initial_velocity);
    
    // 动态路径更新
    void updatePath(const std::vector<std::vector<double>>& new_path);
    
    // 获取计算统计信息
    struct PlanningStats {
        double total_calculation_time = 0.0;
        size_t segments_calculated = 0;
        double average_calculation_time = 0.0;
    };
    
    PlanningStats getPlanningStats() const { return stats_; }

private:
    // 核心算法模块
    std::vector<std::vector<double>> computeDirectionSwitchPoints(
        const std::vector<std::vector<double>>& path_points);
    
    TVPParameters timeMinimalTVP(
        double q0, double w0, double qg, double a_max, double w_max) const;
    
    TVPParameters adjustTVPForSyncTime(
        double q0, double w0, double q_via, double t_via, 
        double a_max, double w_max) const;
    
    double determineSyncTime(
        const std::vector<double>& t_via_min,
        const std::vector<int>& motion_phases) const;
    
    // TVP轨迹计算
    void calculateTVPTrajectory(
        std::vector<TrajectoryPoint>& trajectory,
        const std::vector<double>& q_start,
        const std::vector<double>& q_end,
        const std::vector<double>& start_vel,
        double segment_time,
        double time_offset);
    
    // 计算到达via point的时间
    double calculateViaTimeForJoint(
        const TVPParameters& tvp, double q0, double w0, double q_via) const;
    
    // 机器人参数
    std::vector<double> max_velocities_;
    std::vector<double> max_accelerations_;
    size_t num_joints_;
    
    // 当前状态
    std::vector<double> current_velocity_;
    std::vector<std::vector<double>> current_path_;
    
    // 统计信息
    PlanningStats stats_;
    
    // 常量
    static constexpr double CONTROL_CYCLE_TIME = 0.001; // 1ms控制周期
    static constexpr double EPSILON = 1e-6;
};

} // namespace receding_horizon_planner