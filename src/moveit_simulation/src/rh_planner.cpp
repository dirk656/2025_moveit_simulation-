#include "moveit_simulation/rh_planner.hpp"
#include <chrono>

namespace receding_horizon_planner {

RecedingHorizonPlanner::RecedingHorizonPlanner() 
    : num_joints_(7) { // 默认7关节，如Franka Panda
    
    // 设置默认参数（Franka Panda）
    max_velocities_ = {0.1000, 0.1000, 0.1000, 0.1000, 0.1250, 0.1250, 0.1250};
    max_accelerations_ = {0.3750, 0.1875, 0.2500, 0.3125, 0.3750, 0.5000, 0.5000};
    current_velocity_ = std::vector<double>(num_joints_, 0.0);
}

void RecedingHorizonPlanner::setRobotParameters(
    const std::vector<double>& max_velocities, 
    const std::vector<double>& max_accelerations) {
    
    if (max_velocities.size() != num_joints_ || max_accelerations.size() != num_joints_) {
        throw std::invalid_argument("Parameter size mismatch with number of joints");
    }
    
    max_velocities_ = max_velocities;
    max_accelerations_ = max_accelerations;
}

std::vector<std::vector<double>> 
RecedingHorizonPlanner::computeDirectionSwitchPoints(
    const std::vector<std::vector<double>>& path_points) {
    
    std::vector<std::vector<double>> switch_points(num_joints_);
    
    for (size_t j = 0; j < num_joints_; ++j) {
        for (size_t i = 1; i < path_points.size() - 1; ++i) {
            double prev_diff = path_points[i][j] - path_points[i-1][j];
            double next_diff = path_points[i+1][j] - path_points[i][j];
            
            // 检查方向变化
            if (prev_diff * next_diff < -EPSILON) {
                switch_points[j].push_back(path_points[i][j]);
            }
        }
    }
    
    return switch_points;
}

TVPParameters RecedingHorizonPlanner::timeMinimalTVP(
    double q0, double w0, double qg, double a_max, double w_max) const {
    
    TVPParameters params;
    double distance = qg - q0;
    double abs_distance = std::abs(distance);
    int direction = (distance >= 0) ? 1 : -1;
    
    // 考虑运动方向
    w0 *= direction;
    a_max *= direction;
    
    // 计算三角型速度曲线所需的最大速度
    double w_tri = std::sqrt(w0 * w0 / 2.0 + a_max * abs_distance);
    
    if (w_tri < w_max) {
        // 三角型速度曲线
        params.wm = w_tri;
        params.tf = (2.0 * params.wm - w0) / a_max;
    } else {
        // 梯形速度曲线
        params.wm = w_max;
        double term1 = abs_distance / params.wm;
        double term2 = (params.wm - w0 + (w0 * w0) / (2.0 * params.wm)) / a_max;
        params.tf = term1 + term2;
    }
    
    params.a = a_max;
    params.t1 = (params.wm - w0) / a_max;
    params.t2 = params.tf - params.wm / a_max;
    
    // 确保时间非负
    params.t1 = std::max(0.0, params.t1);
    params.t2 = std::max(0.0, params.t2);
    params.tf = std::max(0.0, params.tf);
    
    return params;
}

double RecedingHorizonPlanner::calculateViaTimeForJoint(
    const TVPParameters& tvp, double q0, double w0, double q_via) const {
    
    double distance = q_via - q0;
    double current_pos = q0;
    double current_vel = w0;
    double t = 0.0;
    double dt = 0.001; // 1ms时间步长
    
    while (t <= tvp.tf + EPSILON) {
        // 根据TVP相位计算当前速度和位置
        if (t <= tvp.t1) {
            // 加速阶段
            current_vel = w0 + tvp.a * t;
            current_pos = q0 + w0 * t + 0.5 * tvp.a * t * t;
        } else if (t <= tvp.t2) {
            // 匀速阶段
            current_vel = tvp.wm;
            current_pos = q0 + w0 * tvp.t1 + 0.5 * tvp.a * tvp.t1 * tvp.t1 + 
                         tvp.wm * (t - tvp.t1);
        } else {
            // 减速阶段
            double t_dec = t - tvp.t2;
            current_vel = tvp.wm - tvp.a * t_dec;
            current_pos = q0 + w0 * tvp.t1 + 0.5 * tvp.a * tvp.t1 * tvp.t1 + 
                         tvp.wm * (tvp.t2 - tvp.t1) + 
                         tvp.wm * t_dec - 0.5 * tvp.a * t_dec * t_dec;
        }
        
        // 检查是否到达目标位置
        if (std::abs(current_pos - q_via) < EPSILON) {
            return t;
        }
        
        // 检查是否越过目标位置
        if ((distance >= 0 && current_pos >= q_via) || 
            (distance < 0 && current_pos <= q_via)) {
            return t;
        }
        
        t += dt;
    }
    
    return tvp.tf; // 返回总时间作为保守估计
}

double RecedingHorizonPlanner::determineSyncTime(
    const std::vector<double>& t_via_min,
    const std::vector<int>& motion_phases) const {
    
    if (t_via_min.empty()) return 0.0;
    
    std::vector<double> t_min_allowed;
    
    for (size_t j = 0; j < t_via_min.size(); ++j) {
        if (motion_phases[j] > 0) {
            // 加速或匀速阶段：可行时间 >= t_via_min
            t_min_allowed.push_back(t_via_min[j]);
        } else {
            // 减速阶段：可行时间 <= t_via_min，这里取一个保守值
            t_min_allowed.push_back(0.0);
        }
    }
    
    // 选择所有关节的最大最小允许时间作为同步时间
    return *std::max_element(t_min_allowed.begin(), t_min_allowed.end());
}

TVPParameters RecedingHorizonPlanner::adjustTVPForSyncTime(
    double q0, double w0, double q_via, double t_via, 
    double a_max, double w_max) const {
    
    double distance = q_via - q0;
    double abs_distance = std::abs(distance);
    int direction = (distance >= 0) ? 1 : -1;
    
    w0 *= direction;
    a_max *= direction;
    
    // 根据论文中的四种情况
    if (abs_distance <= 0.5 * w0 * t_via) {
        // Situation 1 & 2
        if (abs_distance < (w0 * w0) / (2.0 * std::abs(a_max))) {
            // Situation 1: 无法在约束下准确停止
            return TVPParameters(w0, a_max, 0.0, 0.0, w0 / std::abs(a_max));
        } else {
            // Situation 2: 可以在约束下停止
            double tf = 2.0 * abs_distance / w0;
            double a = w0 / tf;
            return TVPParameters(w0, a, 0.0, 0.0, tf);
        }
    } else if (abs_distance <= w0 * t_via) {
        // Situation 3: 仅减速阶段，在t_via时通过
        double a = 2.0 * (w0 * t_via - abs_distance) / (t_via * t_via);
        return TVPParameters(w0, a, 0.0, 0.0, t_via);
    } else {
        // Situation 4: 需要加速阶段
        double a = a_max;
        double t1 = t_via - std::sqrt(t_via * t_via - 2.0 * (abs_distance - w0 * t_via) / std::abs(a));
        double wm = w0 + a * t1;
        return TVPParameters(wm, a, t1, 0.0, t_via);
    }
}

void RecedingHorizonPlanner::calculateTVPTrajectory(
    std::vector<TrajectoryPoint>& trajectory,
    const std::vector<double>& q_start,
    const std::vector<double>& q_end,
    const std::vector<double>& start_vel,
    double segment_time,
    double time_offset) {
    
    size_t num_points = static_cast<size_t>(segment_time / CONTROL_CYCLE_TIME) + 1;
    
    for (size_t i = 0; i < num_points; ++i) {
        double t = i * CONTROL_CYCLE_TIME;
        TrajectoryPoint point(num_joints_);
        point.time_from_start = time_offset + t;
        
        for (size_t j = 0; j < num_joints_; ++j) {
            TVPParameters tvp = timeMinimalTVP(
                q_start[j], start_vel[j], q_end[j], 
                max_accelerations_[j], max_velocities_[j]);
            
            // 根据TVP计算当前位置、速度、加速度
            if (t <= tvp.t1) {
                // 加速阶段
                point.velocities[j] = start_vel[j] + tvp.a * t;
                point.positions[j] = q_start[j] + start_vel[j] * t + 0.5 * tvp.a * t * t;
                point.accelerations[j] = tvp.a;
            } else if (t <= tvp.t2) {
                // 匀速阶段
                point.velocities[j] = tvp.wm;
                point.positions[j] = q_start[j] + start_vel[j] * tvp.t1 + 
                                   0.5 * tvp.a * tvp.t1 * tvp.t1 + 
                                   tvp.wm * (t - tvp.t1);
                point.accelerations[j] = 0.0;
            } else if (t <= tvp.tf) {
                // 减速阶段
                double t_dec = t - tvp.t2;
                point.velocities[j] = tvp.wm - tvp.a * t_dec;
                point.positions[j] = q_start[j] + start_vel[j] * tvp.t1 + 
                                   0.5 * tvp.a * tvp.t1 * tvp.t1 + 
                                   tvp.wm * (tvp.t2 - tvp.t1) + 
                                   tvp.wm * t_dec - 0.5 * tvp.a * t_dec * t_dec;
                point.accelerations[j] = -tvp.a;
            } else {
                // 运动结束
                point.velocities[j] = 0.0;
                point.positions[j] = q_end[j];
                point.accelerations[j] = 0.0;
            }
        }
        
        trajectory.push_back(point);
    }
}

std::vector<TrajectoryPoint> RecedingHorizonPlanner::generateTrajectory(
    const std::vector<std::vector<double>>& path_points,
    const std::vector<double>& initial_velocity) {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    if (path_points.size() < 2) {
        throw std::invalid_argument("Path must contain at least 2 points");
    }
    
    if (initial_velocity.size() != num_joints_) {
        throw std::invalid_argument("Initial velocity size mismatch");
    }
    
    std::vector<TrajectoryPoint> trajectory;
    current_velocity_ = initial_velocity;
    current_path_ = path_points;
    
    auto switch_points = computeDirectionSwitchPoints(path_points);
    
    double current_time = 0.0;
    
    // 滚动时域规划：逐个路径段规划
    for (size_t seg_idx = 0; seg_idx < path_points.size() - 1; ++seg_idx) {
        const auto& q_current = path_points[seg_idx];
        const auto& q_next = path_points[seg_idx + 1];
        
        // Step 1: 为每个关节计算时间最优TVP到下一个方向切换点
        std::vector<double> t_via_min(num_joints_);
        std::vector<int> motion_phases(num_joints_);
        std::vector<TVPParameters> joint_tvps(num_joints_);
        
        for (size_t j = 0; j < num_joints_; ++j) {
            // 确定下一个方向切换点或终点
            double q_target = path_points.back()[j]; // 默认使用路径终点
            for (const auto& sp : switch_points[j]) {
                if ((sp > q_current[j] && sp <= q_next[j]) || 
                    (sp < q_current[j] && sp >= q_next[j])) {
                    q_target = sp;
                    break;
                }
            }
            
            joint_tvps[j] = timeMinimalTVP(
                q_current[j], current_velocity_[j], q_target,
                max_accelerations_[j], max_velocities_[j]);
            
            t_via_min[j] = calculateViaTimeForJoint(
                joint_tvps[j], q_current[j], current_velocity_[j], q_next[j]);
            
            motion_phases[j] = (joint_tvps[j].t2 > EPSILON) ? 1 : 0;
        }
        
        // Step 2: 确定同步时间
        double t_sync = determineSyncTime(t_via_min, motion_phases);
        
        // Step 3: 为当前路径段生成轨迹
        calculateTVPTrajectory(
            trajectory, q_current, q_next, current_velocity_, 
            t_sync, current_time);
        
        // 更新当前时间和速度用于下一个路径段
        current_time += t_sync;
        for (size_t j = 0; j < num_joints_; ++j) {
            // 使用调整后的TVP计算段末速度
            TVPParameters adjusted_tvp = adjustTVPForSyncTime(
                q_current[j], current_velocity_[j], q_next[j],
                t_sync, max_accelerations_[j], max_velocities_[j]);
            
            if (t_sync <= adjusted_tvp.t1) {
                current_velocity_[j] = current_velocity_[j] + adjusted_tvp.a * t_sync;
            } else if (t_sync <= adjusted_tvp.t2) {
                current_velocity_[j] = adjusted_tvp.wm;
            } else {
                double t_dec = t_sync - adjusted_tvp.t2;
                current_velocity_[j] = adjusted_tvp.wm - adjusted_tvp.a * t_dec;
            }
        }
        
        stats_.segments_calculated++;
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    stats_.total_calculation_time = duration.count() / 1000.0; // 转换为毫秒
    stats_.average_calculation_time = stats_.total_calculation_time / stats_.segments_calculated;
    
    return trajectory;
}

void RecedingHorizonPlanner::updatePath(const std::vector<std::vector<double>>& new_path) {
    if (new_path.empty()) {
        throw std::invalid_argument("New path cannot be empty");
    }
    
    if (new_path[0].size() != num_joints_) {
        throw std::invalid_argument("New path joint count mismatch");
    }
    
    current_path_ = new_path;
    // 重置统计信息
    stats_ = PlanningStats();
}

} // namespace receding_horizon_planner