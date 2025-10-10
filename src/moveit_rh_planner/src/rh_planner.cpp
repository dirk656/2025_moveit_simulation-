#include "moveit_rh_planner/rh_planner.h"
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <cmath>
#include <chrono>
#include <algorithm>

namespace rh_moveit_planner
{

// 控制周期
static constexpr double CONTROL_CYCLE_TIME = 0.001;
static constexpr double EPSILON = 1e-6;

RHPlanner::RHPlanner(const moveit::core::RobotModelConstPtr& model,
                     const std::string& group_name,
                     const planning_scene::PlanningSceneConstPtr& planning_scene,
                     const rclcpp::Node::SharedPtr& node,
                     const std::string& parameter_namespace)
  : planning_interface::PlanningContext("RHPlanner", group_name)
  , node_(node)
  , group_name_(group_name)
  , robot_model_(model)
  , planning_scene_(planning_scene)
{
  // 获取关节模型组
  const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group_name);
  if (!jmg)
  {
    RCLCPP_ERROR(node_->get_logger(), "Invalid group name: %s", group_name.c_str());
    return;
  }

  // 设置默认关节限制
  const auto& joint_models = jmg->getActiveJointModels();
  max_velocities_.resize(joint_models.size());
  max_accelerations_.resize(joint_models.size());

  for (size_t i = 0; i < joint_models.size(); ++i)
  {
    // 从URDF获取限制或使用默认值
    if (joint_models[i]->getVariableBounds().size() > 0)
    {
      max_velocities_[i] = joint_models[i]->getVariableBounds()[0].max_velocity_;
      max_accelerations_[i] = joint_models[i]->getVariableBounds()[0].max_acceleration_;
      
      // 如果URDF中没有定义，使用合理默认值
      if (max_velocities_[i] <= 0) max_velocities_[i] = 1.0;
      if (max_accelerations_[i] <= 0) max_accelerations_[i] = 2.0;
    }
    else
    {
      max_velocities_[i] = 1.0;
      max_accelerations_[i] = 2.0;
    }
  }

  RCLCPP_INFO(node_->get_logger(), "RH Planner initialized for group: %s", group_name.c_str());
}

RHPlanner::~RHPlanner()
{
}

bool RHPlanner::solve(planning_interface::MotionPlanResponse& res)
{
  RCLCPP_INFO(node_->get_logger(), "Starting RH planning for group: %s", group_name_.c_str());

  // 开始计时
  auto start_time = std::chrono::high_resolution_clock::now();

  try
  {
    // 创建轨迹
    robot_trajectory::RobotTrajectoryPtr trajectory = 
      std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, group_name_);

    // 执行滚动时域规划
    if (!planRecedingHorizonTrajectory(getMotionPlanRequest(), trajectory))
    {
      res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
      return false;
    }

    // 计算规划时间
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    res.planning_time_ = duration.count() / 1000000.0;

    // 设置结果
    res.trajectory_ = trajectory;
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

    RCLCPP_INFO(node_->get_logger(), 
                "RH planning completed successfully in %.3f seconds with %zu points",
                res.planning_time_, trajectory->getWayPointCount());

    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node_->get_logger(), "RH planning failed: %s", e.what());
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }
}

bool RHPlanner::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  // 简化实现，调用单个响应版本
  planning_interface::MotionPlanResponse simple_res;

  if (!solve(simple_res))
  {
    res.error_code_ = simple_res.error_code_;
    return false;
  }

  res.trajectory_.resize(1);
  res.trajectory_[0] = simple_res.trajectory_;
  res.description_.push_back("RecedingHorizon");
  res.processing_time_.push_back(simple_res.planning_time_);
  res.error_code_ = simple_res.error_code_;

  return true;
}

bool RHPlanner::terminate()
{
  // 终止规划（如果需要）
  RCLCPP_INFO(node_->get_logger(), "RH Planner terminated");
  return true;
}

void RHPlanner::clear()
{
  // 清理资源
}

bool RHPlanner::planRecedingHorizonTrajectory(const planning_interface::MotionPlanRequest& req,
                                             robot_trajectory::RobotTrajectoryPtr& trajectory)
{
  // 获取起始和目标状态
  moveit::core::RobotState start_state = planning_scene_->getCurrentState();
  moveit::core::RobotState goal_state(robot_model_);

  // 设置目标状态
  if (req.goal_constraints.size() > 0 && req.goal_constraints[0].joint_constraints.size() > 0)
  {
    // 关节空间目标
    for (const auto& joint_constraint : req.goal_constraints[0].joint_constraints)
    {
      goal_state.setVariablePosition(joint_constraint.joint_name, joint_constraint.position);
    }
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Only joint space goals are supported in this implementation");
    return false;
  }

  // 生成路径点
  std::vector<moveit::core::RobotState> waypoints;
  if (!generateWaypoints(start_state, goal_state, waypoints))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to generate waypoints");
    return false;
  }

  // 转换为关节空间路径
  std::vector<std::vector<double>> joint_path;
  if (!convertToJointPath(waypoints, joint_path))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to convert to joint path");
    return false;
  }

  // 生成滚动时域轨迹
  return generateRHTrajectory(joint_path, trajectory);
}

bool RHPlanner::generateWaypoints(const moveit::core::RobotState& start_state,
                                 const moveit::core::RobotState& goal_state,
                                 std::vector<moveit::core::RobotState>& waypoints)
{
  waypoints.clear();

  const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group_name_);
  if (!jmg)
    return false;

  // 简单的线性插值生成路径点
  int num_waypoints = 10;  // 可以根据需要调整

  for (int i = 0; i <= num_waypoints; ++i)
  {
    double t = static_cast<double>(i) / num_waypoints;
    
    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();

    // 对每个关节进行线性插值
    for (const auto* joint : jmg->getActiveJointModels())
    {
      double start_pos = start_state.getVariablePosition(joint->getName());
      double goal_pos = goal_state.getVariablePosition(joint->getName());
      double interp_pos = start_pos + t * (goal_pos - start_pos);
      
      state.setVariablePosition(joint->getName(), interp_pos);
    }

    waypoints.push_back(state);
  }

  return true;
}

bool RHPlanner::convertToJointPath(const std::vector<moveit::core::RobotState>& waypoints,
                                  std::vector<std::vector<double>>& joint_path)
{
  joint_path.clear();

  const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group_name_);
  if (!jmg)
    return false;

  for (const auto& waypoint : waypoints)
  {
    std::vector<double> joint_positions;
    waypoint.copyJointGroupPositions(jmg, joint_positions);
    joint_path.push_back(joint_positions);
  }

  return true;
}

bool RHPlanner::generateRHTrajectory(const std::vector<std::vector<double>>& joint_path,
                                    robot_trajectory::RobotTrajectoryPtr& trajectory)
{
  if (joint_path.size() < 2)
    return false;

  const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group_name_);
  if (!jmg)
    return false;

  // 清空轨迹
  trajectory->clear();

  const size_t num_joints = jmg->getVariableCount();
  std::vector<double> zero_velocities(num_joints, 0.0);
  
  // 使用更安全的时间分配策略，确保时间严格递增
  // 使用足够大的时间间隔避免控制器拒绝，同时确保数值精度
  double time_increment = 0.5;  // 每个点间隔0.5秒，确保严格递增且足够精确
  
  // 直接使用保守方法，避免迭代时间参数化可能的问题
  for (size_t i = 0; i < joint_path.size(); ++i)
  {
    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();
    state.setJointGroupPositions(jmg, joint_path[i]);
    state.setJointGroupVelocities(jmg, zero_velocities);
    
    // 使用严格递增的时间戳，确保每个点的时间都比前一个大
    // 使用足够大的时间间隔，避免任何可能的数值精度问题
    double current_time = i * time_increment;
    trajectory->addSuffixWayPoint(state, current_time);
  }

  // 严格验证轨迹时间严格递增
  bool time_valid = true;
  double prev_time = -1.0;
  for (size_t i = 0; i < trajectory->getWayPointCount(); ++i)
  {
    double current_time = trajectory->getWayPointDurationFromStart(i);
    if (current_time <= prev_time + EPSILON)
    {
      RCLCPP_ERROR(node_->get_logger(), 
                   "Time validation failed! Point %zu: %.6f, Previous: %.6f", 
                   i, current_time, prev_time);
      time_valid = false;
      
      // 强制修复：重新分配时间，使用更大的间隔
      trajectory->clear();
      for (size_t j = 0; j < joint_path.size(); ++j)
      {
        moveit::core::RobotState state(robot_model_);
        state.setToDefaultValues();
        state.setJointGroupPositions(jmg, joint_path[j]);
        state.setJointGroupVelocities(jmg, zero_velocities);
        
        // 使用更大的时间间隔确保严格递增
        double fixed_time = j * 1.0;  // 每个点间隔1.0秒
        trajectory->addSuffixWayPoint(state, fixed_time);
      }
      break;
    }
    prev_time = current_time;
  }

  // 最终验证：确保所有时间点严格递增
  double last_time = -1.0;
  for (size_t i = 0; i < trajectory->getWayPointCount(); ++i)
  {
    double current_time = trajectory->getWayPointDurationFromStart(i);
    if (current_time <= last_time + EPSILON)
    {
      RCLCPP_ERROR(node_->get_logger(), 
                   "CRITICAL: Time not strictly increasing at point %zu: %.6f <= %.6f", 
                   i, current_time, last_time);
      return false;
    }
    last_time = current_time;
  }

  RCLCPP_INFO(node_->get_logger(), 
              "Generated trajectory with %zu waypoints, total time: %.3f seconds",
              trajectory->getWayPointCount(), 
              trajectory->getWayPointDurationFromStart(trajectory->getWayPointCount() - 1));

  // 打印详细时间信息用于调试
  for (size_t i = 0; i < trajectory->getWayPointCount(); ++i)
  {
    RCLCPP_DEBUG(node_->get_logger(), "Waypoint %zu time: %.6f", i, 
                 trajectory->getWayPointDurationFromStart(i));
  }

  return true;
}

RHPlanner::TVPParameters RHPlanner::timeMinimalTVP(double q0, double w0, double qg, 
                                                  double a_max, double w_max) const
{
  TVPParameters params;
  double distance = qg - q0;
  double abs_distance = std::abs(distance);
  int direction = (distance >= 0) ? 1 : -1;

  w0 *= direction;
  a_max *= direction;

  // 计算三角型速度曲线所需的最大速度
  double w_tri = std::sqrt(w0 * w0 / 2.0 + std::abs(a_max) * abs_distance);

  if (w_tri < w_max)
  {
    // 三角型速度曲线
    params.wm = w_tri;
    params.tf = (2.0 * params.wm - w0) / std::abs(a_max);
  }
  else
  {
    // 梯形速度曲线
    params.wm = w_max;
    double term1 = abs_distance / params.wm;
    double term2 = (params.wm - w0 + (w0 * w0) / (2.0 * params.wm)) / std::abs(a_max);
    params.tf = term1 + term2;
  }

  params.a = a_max;
  params.t1 = std::max(0.0, (params.wm - w0) / std::abs(a_max));
  params.t2 = std::max(0.0, params.tf - params.wm / std::abs(a_max));
  params.tf = std::max(0.0, params.tf);

  return params;
}

}  // namespace rh_moveit_planner
