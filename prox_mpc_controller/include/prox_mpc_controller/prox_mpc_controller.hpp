// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


#ifndef PROX_MPC_CONTROLLER__PROX_MPC_CONTROLLER_HPP_
#define PROX_MPC_CONTROLLER__PROX_MPC_CONTROLLER_HPP_

#include <cstddef>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>

#include <prox_mpc/mpc.hpp>
#include <prox_mpc/model.hpp>

namespace prox_mpc_controller
{

/// Nav2 controller plugin that wraps the ProxMPC SQP/QP core (prox_mpc::MPC).
///
/// The plugin owns the ROS integration: it loads and configures a prox_mpc::Model
/// plugin, sizes the MPC, builds the state/control reference from the global plan,
/// reduces the local costmap to obstacle triples, solves one SQP cycle per
/// control step, maps the first optimal control to a body Twist, and decelerates
/// within the robot's limits when a solve fails. The engine math (the SQP loop,
/// the QP, and the obstacle half-planes) lives in prox_mpc_core and is unchanged.
class ProxMpcController : public nav2_core::Controller
{
public:
  ProxMpcController() = default;
  ~ProxMpcController() override = default;

  /// Read parameters, load and configure the model, and size the MPC.
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /// Release owned resources.
  void cleanup() override;

  /// Reset runtime state for a new task.
  void activate() override;

  /// Stop processing.
  void deactivate() override;

  /// Store the global plan to track.
  void setPlan(const nav_msgs::msg::Path & path) override;

  /// Compute the next command by solving one SQP cycle of the wrapped MPC.
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /// Constrain the maximum speed (absolute [m/s] or percentage of the maximum).
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  /// Request a graceful stop; returns true only once the robot has decelerated.
  bool cancel() override;

  /// Clear runtime state between tasks (keeps owned handles intact).
  void reset() override;

protected:
  /// Reduce the local costmap to at most max_obstacles_ (o_x, o_y, d_safe) triples
  /// per predicted node, centered on the reference trajectory positions.
  void reduceCostmap(const MatrixXd & reference, MatrixXd & obs);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("ProxMpcController")};
  rclcpp::Clock::SharedPtr clock_;

  nav_msgs::msg::Path global_plan_;

  /// The ProxMPC core solver this plugin drives, and the model it solves.
  std::shared_ptr<prox_mpc::MPC> mpc_;
  std::shared_ptr<pluginlib::ClassLoader<prox_mpc::Model>> model_loader_;
  std::shared_ptr<prox_mpc::Model> model_;

  /// Cached dimensions and horizon settings.
  std::size_t n_{0};
  std::size_t m_{0};
  std::size_t np_{0};
  std::size_t nc_{0};
  double dt_{0.1};
  double desired_linear_vel_{1.0};

  /// Control-law parameters.
  int max_solver_failures_{3};
  int max_obstacles_{1};
  double safety_margin_{0.1};
  double robot_radius_{0.5};
  double cbf_gamma_{1.0};
  int costmap_cost_threshold_{200};
  double obstacle_cluster_radius_{0.3};

  /// Speed bounds: v_max_ is the model's original upper bound on the speed
  /// channel; max_linear_vel_ is the currently applied limit.
  double v_max_{0.0};
  double max_linear_vel_{0.0};

  /// Deceleration limits read from the model's du bounds.
  double a_dec_lin_{0.5};
  double a_dec_ang_{0.5};

  /// Speed limit cached before the model is loaded (re-applied in configure()).
  double speed_limit_{0.0};
  bool speed_limit_is_percentage_{false};

  /// Runtime state, reset between tasks.
  int failure_count_{0};
  double steering_state_{0.0};
  double last_cmd_v_{0.0};
  double last_cmd_w_{0.0};
  bool cancelling_{false};
  std::size_t plan_index_{0};
};

}  // namespace prox_mpc_controller

#endif  // PROX_MPC_CONTROLLER__PROX_MPC_CONTROLLER_HPP_
