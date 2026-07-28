// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


#include "prox_mpc_benchmark/timing_controller_wrapper.hpp"

#include <chrono>
#include <memory>
#include <string>

#include <nav2_core/controller_exceptions.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace prox_mpc_benchmark
{

void TimingControllerWrapper::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  // Nav2 reconfigures a plugin on the same node after a cleanup
  // (deactivate -> cleanup -> configure), and the parameter survives the
  // teardown; re-declaring it would throw ParameterAlreadyDeclaredException and
  // abort the second bringup.
  const std::string param_name = name + ".wrapped_plugin";
  if (!node->has_parameter(param_name)) {
    node->declare_parameter(param_name, rclcpp::ParameterValue(std::string("")));
  }
  const std::string wrapped_type = node->get_parameter(param_name).as_string();
  if (wrapped_type.empty()) {
    throw nav2_core::ControllerException(
            "TimingControllerWrapper: '" + name + ".wrapped_plugin' must name a controller plugin");
  }

  try {
    wrapped_ = loader_.createSharedInstance(wrapped_type);
  } catch (const pluginlib::PluginlibException & ex) {
    throw nav2_core::ControllerException(
            std::string("TimingControllerWrapper: failed to load wrapped controller '") +
            wrapped_type + "': " + ex.what());
  }

  // Per-cycle compute time, only-when-subscribed so it is free in production.
  time_pub_ = node->create_publisher<std_msgs::msg::Float64>(
    name + "/compute_time_ms", rclcpp::QoS(10).reliable());

  // The wrapped controller is configured with the same name, so it reads its own
  // parameters from <name>.* exactly as it would unwrapped.
  wrapped_->configure(parent, name, tf, costmap_ros);
  RCLCPP_INFO(
    logger_, "TimingControllerWrapper '%s' wrapping '%s'.", name.c_str(), wrapped_type.c_str());
}

void TimingControllerWrapper::cleanup()
{
  if (wrapped_) {wrapped_->cleanup();}
  wrapped_.reset();
  time_pub_.reset();
}

void TimingControllerWrapper::activate()
{
  if (time_pub_) {time_pub_->on_activate();}
  if (wrapped_) {wrapped_->activate();}
}

void TimingControllerWrapper::deactivate()
{
  if (time_pub_) {time_pub_->on_deactivate();}
  if (wrapped_) {wrapped_->deactivate();}
}

void TimingControllerWrapper::setPlan(const nav_msgs::msg::Path & path)
{
  wrapped_->setPlan(path);
}

geometry_msgs::msg::TwistStamped TimingControllerWrapper::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  const auto t0 = std::chrono::steady_clock::now();
  geometry_msgs::msg::TwistStamped cmd =
    wrapped_->computeVelocityCommands(pose, velocity, goal_checker);
  const auto t1 = std::chrono::steady_clock::now();

  if (time_pub_ && time_pub_->get_subscription_count() > 0) {
    std_msgs::msg::Float64 m;
    m.data = std::chrono::duration<double, std::milli>(t1 - t0).count();
    time_pub_->publish(m);
  }
  return cmd;
}

void TimingControllerWrapper::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  wrapped_->setSpeedLimit(speed_limit, percentage);
}

bool TimingControllerWrapper::cancel()
{
  return wrapped_ ? wrapped_->cancel() : true;
}

void TimingControllerWrapper::reset()
{
  if (wrapped_) {wrapped_->reset();}
}

}  // namespace prox_mpc_benchmark

PLUGINLIB_EXPORT_CLASS(prox_mpc_benchmark::TimingControllerWrapper, nav2_core::Controller)
