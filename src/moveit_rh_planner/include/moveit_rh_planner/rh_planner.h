#pragma once

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>
#include <rclcpp/rclcpp.hpp>

#include "moveit_rh_planner/visual_ctl.h"

#include <vector>
#include <string>
#include <memory>

namespace rh_moveit_planner
{

class RHPlanner : public planning_interface::PlanningContext
{
public:
  RH_MOVEIT_PLANNER_PUBLIC
  RHPlanner(const moveit::core::RobotModelConstPtr& model,
            const std::string& group_name,
            const planning_scene::PlanningSceneConstPtr& planning_scene,
            const rclcpp::Node::SharedPtr& node,
            const std::string& parameter_namespace);

  RH_MOVEIT_PLANNER_PUBLIC
  virtual ~RHPlanner();

  RH_MOVEIT_PLANNER_PUBLIC
  bool solve(planning_interface::MotionPlanResponse& res) override;

  RH_MOVEIT_PLANNER_PUBLIC
  bool solve(planning_interface::MotionPlanDetailedResponse& res) override;

  RH_MOVEIT_PLANNER_PUBLIC
  bool terminate() override;

  RH_MOVEIT_PLANNER_PUBLIC
  void clear() override;

private:
  // 滚动时域规划算法实现
  bool planRecedingHorizonTrajectory(const planning_interface::MotionPlanRequest& req,
                                    robot_trajectory::RobotTrajectoryPtr& trajectory);

  // 生成路径点
  bool generateWaypoints(const moveit::core::RobotState& start_state,
                        const moveit::core::RobotState& goal_state,
                        std::vector<moveit::core::RobotState>& waypoints);

  // 转换为关节空间路径
  bool convertToJointPath(const std::vector<moveit::core::RobotState>& waypoints,
                         std::vector<std::vector<double>>& joint_path);

  // 滚动时域轨迹生成
  bool generateRHTrajectory(const std::vector<std::vector<double>>& joint_path,
                           robot_trajectory::RobotTrajectoryPtr& trajectory);

  // TVP参数结构
  struct TVPParameters {
    double wm = 0.0;  // 最大速度
    double a = 0.0;   // 加速度
    double t1 = 0.0;  // 加速结束时间
    double t2 = 0.0;  // 减速开始时间
    double tf = 0.0;  // 总时间
  };

  // 时间最优TVP计算
  TVPParameters timeMinimalTVP(double q0, double w0, double qg, 
                              double a_max, double w_max) const;

  // 机器人参数
  std::vector<double> max_velocities_;
  std::vector<double> max_accelerations_;

  rclcpp::Node::SharedPtr node_;
  std::string group_name_;
  moveit::core::RobotModelConstPtr robot_model_;
  planning_scene::PlanningSceneConstPtr planning_scene_;
};

}
