#include "moveit_rh_planner/rh_planner_manager.h"
#include <pluginlib/class_list_macros.hpp>

namespace rh_moveit_planner
{

RHPlannerManager::RHPlannerManager()
{
}

RHPlannerManager::~RHPlannerManager()
{
}

bool RHPlannerManager::initialize(const moveit::core::RobotModelConstPtr& model, 
                                 const rclcpp::Node::SharedPtr& node,
                                 const std::string& parameter_namespace)
{
  robot_model_ = model;
  node_ = node;
  parameter_namespace_ = parameter_namespace;

  RCLCPP_INFO(node_->get_logger(), "Receding Horizon Planner Manager initialized");

  return true;
}

void RHPlannerManager::getPlanningAlgorithms(std::vector<std::string>& algs) const
{
  algs.clear();
  algs.push_back("RHPlanner");
  algs.push_back("RecedingHorizon");
}

planning_interface::PlanningContextPtr RHPlannerManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req,
    moveit_msgs::msg::MoveItErrorCodes& error_code) const
{
  error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

  // 创建规划器上下文
  return std::make_shared<RHPlanner>(robot_model_, req.group_name, 
                                   planning_scene, node_, parameter_namespace_);
}

bool RHPlannerManager::canServiceRequest(const moveit_msgs::msg::MotionPlanRequest& req) const
{
  // 检查是否支持该请求类型
  return req.planner_id.find("RH") != std::string::npos || 
         req.planner_id.find("RecedingHorizon") != std::string::npos;
}

void RHPlannerManager::setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pconfig)
{
  // 配置规划器参数
  for (const auto& config : pconfig)
  {
    if (config.first.find("RH") != std::string::npos || 
        config.first.find("RecedingHorizon") != std::string::npos)
    {
      RCLCPP_INFO(node_->get_logger(), "Configuring RH Planner: %s", config.first.c_str());
    }
  }
}

}  // namespace rh_moveit_planner

// 注册插件
PLUGINLIB_EXPORT_CLASS(rh_moveit_planner::RHPlannerManager, planning_interface::PlannerManager)
