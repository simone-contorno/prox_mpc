// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


#ifndef PROX_MPC_CONTROLLER__PROX_MPC_CONTROLLER_HPP_
#define PROX_MPC_CONTROLLER__PROX_MPC_CONTROLLER_HPP_

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>

#include <prox_mpc/mpc.hpp>

namespace prox_mpc_controller
{

/// Nav2 controller plugin that wraps the ProxMPC SQP/QP core (prox_mpc::MPC).
///
/// SKELETON: this class satisfies the nav2_core::Controller interface and wires
/// up the lifecycle, costmap, TF, and a prox_mpc::MPC instance, but the control
/// law in computeVelocityCommands() is not implemented yet. It is the surface on
/// which the MPC integration (build the reference from the plan, map the costmap
/// to obstacle constraints, solve, and publish the first control) is developed.
class ProxMpcController : public nav2_core::Controller
{
public:
  ProxMpcController() = default;
  ~ProxMpcController() override = default;

  /// Read parameters and cache the lifecycle node, TF buffer, and costmap.
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /// Release owned resources.
  void cleanup() override;

  /// Activate publishers and runtime state.
  void activate() override;

  /// Deactivate publishers and runtime state.
  void deactivate() override;

  /// Store the global plan to track.
  void setPlan(const nav_msgs::msg::Path & path) override;

  /// Compute the next command. NOT IMPLEMENTED: returns a zero command.
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /// Constrain the maximum speed (absolute [m/s] or percentage of the maximum).
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("ProxMpcController")};
  rclcpp::Clock::SharedPtr clock_;

  nav_msgs::msg::Path global_plan_;

  /// The ProxMPC core solver this plugin drives.
  std::shared_ptr<prox_mpc::MPC> mpc_;

  /// Speed limit set by Nav2 (absolute [m/s]); 0 means no limit.
  double speed_limit_{0.0};
  bool speed_limit_is_percentage_{false};
};

}  // namespace prox_mpc_controller

#endif  // PROX_MPC_CONTROLLER__PROX_MPC_CONTROLLER_HPP_
