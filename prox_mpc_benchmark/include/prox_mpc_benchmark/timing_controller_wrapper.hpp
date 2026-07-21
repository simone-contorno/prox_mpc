// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// nav2_core::Controller decorator that wall-clock times the wrapped controller.
//
// It loads a real controller plugin named by the <name>.wrapped_plugin parameter,
// forwards every nav2_core::Controller call to it, and times each
// computeVelocityCommands, publishing the per-cycle compute time [ms] on
// <name>/compute_time_ms (only-when-subscribed). Because the same decorator
// measures every controller under test identically, the cross-controller compute
// comparison is apples-to-apples (the benchmark real-time comparison). It adds no
// control behaviour of its own and is benchmark-only.

#ifndef PROX_MPC_BENCHMARK__TIMING_CONTROLLER_WRAPPER_HPP_
#define PROX_MPC_BENCHMARK__TIMING_CONTROLLER_WRAPPER_HPP_

#include <memory>
#include <string>

#include <nav2_core/controller.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <std_msgs/msg/float64.hpp>

namespace prox_mpc_benchmark
{

class TimingControllerWrapper : public nav2_core::Controller
{
public:
  TimingControllerWrapper() = default;
  ~TimingControllerWrapper() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  bool cancel() override;
  void reset() override;

private:
  // The base-class export package and type the controllers register against.
  // Declared before `wrapped_`: members destruct in reverse declaration order,
  // so `wrapped_` releases its plugin instance before this loader unloads the
  // library. Reversing the two would unload the .so out from under a live
  // instance.
  pluginlib::ClassLoader<nav2_core::Controller> loader_{
    "nav2_core", "nav2_core::Controller"};
  std::shared_ptr<nav2_core::Controller> wrapped_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr time_pub_;
  rclcpp::Logger logger_{rclcpp::get_logger("TimingControllerWrapper")};
};

}  // namespace prox_mpc_benchmark

#endif  // PROX_MPC_BENCHMARK__TIMING_CONTROLLER_WRAPPER_HPP_
