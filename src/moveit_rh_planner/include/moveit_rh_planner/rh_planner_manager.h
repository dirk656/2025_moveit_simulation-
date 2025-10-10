#pragma once

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>
#include <rclcpp/rclcpp.hpp>

#include "moveit_rh_planner/visual_ctl.h"
#include "moveit_rh_planner/rh_planner.h"

namespace rh_moveit_planner
{

class RHPlannerManager : public planning_interface::PlannerManager
{
public:
  RH_MOVEIT_PLANNER_PUBLIC
  RHPlannerManager();

  RH_MOVEIT_PLANNER_PUBLIC
  virtual ~RHPlannerManager();

  RH_MOVEIT_PLANNER_PUBLIC
  bool initialize(const moveit::core::RobotModelConstPtr& model, 
                  const rclcpp::Node::SharedPtr& node,
                  const std::string& parameter_namespace) override;

  RH_MOVEIT_PLANNER_PUBLIC
  std::string getDescription() const override { return "Receding Horizon Planner"; }

  RH_MOVEIT_PLANNER_PUBLIC
  void getPlanningAlgorithms(std::vector<std::string>& algs) const override;

  RH_MOVEIT_PLANNER_PUBLIC
  planning_interface::PlanningContextPtr getPlanningContext(
      const planning_scene::PlanningSceneConstPtr& planning_scene,
      const planning_interface::MotionPlanRequest& req,
      moveit_msgs::msg::MoveItErrorCodes& error_code) const override;

  RH_MOVEIT_PLANNER_PUBLIC
  bool canServiceRequest(const moveit_msgs::msg::MotionPlanRequest& req) const override;

  RH_MOVEIT_PLANNER_PUBLIC
  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pconfig) override;

protected:
  moveit::core::RobotModelConstPtr robot_model_;
  rclcpp::Node::SharedPtr node_;
  std::string parameter_namespace_;
};

}  // namespace rh_moveit_planner
