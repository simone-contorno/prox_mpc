// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


#include "prox_mpc_controller/prox_mpc_controller.hpp"

#include <memory>
#include <string>

#include <pluginlib/class_list_macros.hpp>

namespace prox_mpc_controller
{

void ProxMpcController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  auto node = parent.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // The solver is constructed here; concrete model selection, horizons, weights,
  // and constraints are declared as parameters and wired in as the control law
  // is implemented.
  mpc_ = std::make_shared<prox_mpc::MPC>();

  RCLCPP_INFO(
    logger_, "Configured ProxMpcController '%s' (control law not yet implemented).",
    plugin_name_.c_str());
}

void ProxMpcController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up ProxMpcController '%s'.", plugin_name_.c_str());
  mpc_.reset();
  costmap_ros_.reset();
  tf_.reset();
}

void ProxMpcController::activate()
{
  RCLCPP_INFO(logger_, "Activating ProxMpcController '%s'.", plugin_name_.c_str());
}

void ProxMpcController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating ProxMpcController '%s'.", plugin_name_.c_str());
}

void ProxMpcController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

geometry_msgs::msg::TwistStamped ProxMpcController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)velocity;
  (void)goal_checker;

  // TODO(prox_mpc): build the reference trajectory from global_plan_, map the
  // local costmap into obstacle constraints, set the pose, solve the MPC, and
  // return the first control. Until then this returns a zero command.
  RCLCPP_WARN_THROTTLE(
    logger_, *clock_, 5000,
    "ProxMpcController::computeVelocityCommands is not implemented yet; "
    "commanding zero velocity.");

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = pose.header.frame_id;
  cmd.header.stamp = clock_->now();
  return cmd;
}

void ProxMpcController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
}

}  // namespace prox_mpc_controller

PLUGINLIB_EXPORT_CLASS(prox_mpc_controller::ProxMpcController, nav2_core::Controller)
