// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


#ifndef PROX_MPC_CONTROLLER__PROX_MPC_CONTROLLER_HPP_
#define PROX_MPC_CONTROLLER__PROX_MPC_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <prox_mpc_msgs/msg/obstacle_array.hpp>
#include <prox_mpc_msgs/msg/solver_diagnostics.hpp>

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
  /// The request is cached and applied on the control thread, so it takes effect
  /// from the next control cycle.
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  /// Request a graceful stop; returns true only once the robot has decelerated.
  bool cancel() override;

  /// Clear runtime state between tasks (keeps owned handles intact).
  void reset() override;

protected:
  /// Read the model's speed bound (upper bound of `u[0]`) and its per-channel
  /// deceleration limits (lower bounds of `du[0]` and `du[1]`) into v_max_,
  /// max_linear_vel_, a_dec_lin_ and a_dec_ang_. A model that declares none of a
  /// required bound cannot be driven safely - a zero deceleration limit leaves
  /// the brake ramp stuck at the current velocity, and a zero speed bound clamps
  /// the cruise speed to zero - so a missing bound throws
  /// nav2_core::ControllerException naming it. `model_plugin` is the plugin name
  /// reported in that message.
  void readModelBounds(prox_mpc::Model & model, const std::string & model_plugin);

  /// Convert a requested speed limit to an absolute bound (a fraction of the
  /// model bound when `percentage`, the model bound itself on NO_SPEED_LIMIT),
  /// clamp it to the model bound, and apply it to the model's `u[0]` inequality.
  /// Called only from the control thread (configure() and the top of a control
  /// cycle), never concurrently with a running solve.
  void applySpeedLimit(double speed_limit, bool percentage);

  /// Fill the per-node (o_x, o_y, d_safe) obstacle matrix for one cycle. With
  /// predict_obstacles_ off, or no fresh tracked-obstacle message, this reproduces
  /// the costmap-only reduceCostmap() exactly. Otherwise it propagates each
  /// dynamic track over the horizon (interpolating the tracker's sampled curved
  /// prediction when present, constant-velocity straight ray otherwise), binds
  /// each to a fixed slot across nodes, and fills the remaining slots from the
  /// costmap (hybrid). `now` is the reference time predictions are aged to (the
  /// command stamp).
  void fillObstacles(const MatrixXd & reference, MatrixXd & obs, const rclcpp::Time & now);

  /// Reduce the local costmap to at most max_obstacles_ (o_x, o_y, d_safe) triples
  /// per predicted node, centered on the reference trajectory positions. This is
  /// the static (costmap-only) fill and the predict_obstacles_-off fallback.
  void reduceCostmap(const MatrixXd & reference, MatrixXd & obs);

  /// Windowed costmap scan that fills obstacle slots [slot_begin, max_obstacles_)
  /// per node from the nearest occupied cells, skipping cells inside any per-node
  /// exclusion disc (a dynamic track's footprint). Slots below slot_begin and the
  /// far-sentinel default are left untouched. exclusions[node] = list of
  /// (x, y, radius); an empty vector means no exclusions.
  void fillStaticObstacles(
    const MatrixXd & reference, MatrixXd & obs, std::size_t slot_begin,
    const std::vector<std::vector<std::array<double, 3>>> & exclusions);

  /// Cache the latest tracked-obstacle array (subscription callback; runs in the
  /// controller-server executor alongside computeVelocityCommands).
  void obstacleCallback(prox_mpc_msgs::msg::ObstacleArray::ConstSharedPtr msg);

  /// Publish the predicted dynamic-obstacle trajectories as RViz markers when the
  /// debug topic has a subscriber (no-op otherwise).
  void publishPredictedObstacleMarkers(const rclcpp::Time & now);

  /// Fill and publish one SolverDiagnostics for this control cycle, only when the
  /// publisher exists and the diagnostics topic has a subscriber (zero cost
  /// otherwise). `solve_ms` is the wall time measured around mpc_->solve();
  /// `converged` is the cycle's solve result; `num_active_obstacles` is the count
  /// of filled, non-sentinel obstacle slots at the current node.
  void publishDiagnostics(
    const rclcpp::Time & stamp, double solve_ms, bool converged,
    std::uint16_t num_active_obstacles);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("ProxMpcController")};
  rclcpp::Clock::SharedPtr clock_;

  /// Predicted NMPC trajectory, published each cycle for visualization.
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> traj_pub_;

  /// Optional RViz markers of the predicted dynamic-obstacle trajectories
  /// (created only when predict_obstacles_ is set).
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>>
  marker_pub_;

  /// Optional per-cycle solver telemetry (created only when publish_diagnostics_
  /// is set); publishes only-when-subscribed so the production default is free.
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<prox_mpc_msgs::msg::SolverDiagnostics>>
  diag_pub_;

  /// Tracked-obstacle input and its mutex-guarded latest message. The callback and
  /// computeVelocityCommands run in the same controller-server executor; the mutex
  /// guards the shared-pointer swap between them.
  rclcpp::Subscription<prox_mpc_msgs::msg::ObstacleArray>::SharedPtr obstacle_sub_;
  std::mutex obstacles_mutex_;
  prox_mpc_msgs::msg::ObstacleArray::ConstSharedPtr latest_obstacles_;

  /// One dynamic obstacle's predicted positions over the horizon (costmap global
  /// frame), retained from the fill so the marker publisher need not recompute.
  struct PredictedObstacle
  {
    std::uint32_t id{0};
    double radius{0.0};
    std::vector<std::array<double, 2>> positions;
  };
  std::vector<PredictedObstacle> predicted_obstacles_;

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

  /// Model wheelbase L [m] (forwarded to the model via model_params.L); used to
  /// pre-position the steering reference for models with a steering state.
  double wheelbase_{1.6};

  /// Cruise-speed reduction gain on path curvature; 0.0 disables the reduction.
  double curvature_gain_{0.0};

  /// Control-law parameters.
  int max_solver_failures_{3};
  int max_obstacles_{1};
  double safety_margin_{0.1};
  double robot_radius_{0.5};
  double cbf_gamma_{1.0};
  int costmap_cost_threshold_{200};
  double obstacle_cluster_radius_{0.3};
  /// Upper bound on the per-node costmap scan half-window [cells].
  int max_obstacle_scan_cells_{50};

  /// Predictive (dynamic) obstacle avoidance. predict_obstacles_ off reproduces
  /// the costmap-only behavior; the rest size the predictive + hybrid fill.
  bool predict_obstacles_{false};
  std::string obstacle_topic_{"tracked_obstacles"};
  double obstacle_timeout_{0.5};               // [s] staleness before costmap-only
  double dynamic_speed_threshold_{0.1};        // [m/s] propagate tracks above this
  double prediction_uncertainty_growth_{0.0};  // [m/s] extra clearance per second
  int max_dynamic_obstacles_{2};               // slot budget for dynamic tracks
  /// Reject tracks larger than this [m] from the predictive path (0 = no limit):
  /// a guard against extended structure (walls) reported as a moving obstacle,
  /// which would otherwise inflate d_safe and erase real costmap cells.
  double max_dynamic_obstacle_radius_{0.0};

  /// Opt-in per-cycle solver telemetry. Off by default so the production plugin
  /// carries no extra interface; when on, publishes only-when-subscribed.
  bool publish_diagnostics_{false};

  /// Speed bounds: v_max_ is the model's original upper bound on the speed
  /// channel; max_linear_vel_ is the currently applied limit.
  double v_max_{0.0};
  double max_linear_vel_{0.0};

  /// Deceleration limits read from the model's du bounds.
  double a_dec_lin_{0.5};
  double a_dec_ang_{0.5};

  /// Speed limit requested by the server. setSpeedLimit() runs on the node's
  /// executor thread while the solver reads the model's bounds on the action
  /// server's thread, so the request is cached here and applied on the control
  /// thread: in configure() when it arrived before the model was loaded, and
  /// otherwise at the top of the next control cycle. The release/acquire pair on
  /// speed_limit_pending_ publishes the value and the flag together.
  std::atomic<double> speed_limit_{0.0};
  std::atomic<bool> speed_limit_is_percentage_{false};
  std::atomic<bool> speed_limit_pending_{false};

  /// Runtime state, reset between tasks.
  int failure_count_{0};
  /// Consecutive footprint-veto count; escalates to NoValidControl past the
  /// max_solver_failures_ budget. Kept separate from failure_count_ so a single
  /// veto does not consume the solver-failure budget (and vice versa).
  int veto_count_{0};
  double steering_state_{0.0};
  double last_cmd_v_{0.0};
  double last_cmd_w_{0.0};
  bool cancelling_{false};
  std::size_t plan_index_{0};

  /// Inter-cycle wall clock for the control_period_ms diagnostics field; reset
  /// between tasks so the first cycle of a task reports NaN rather than a stale gap.
  std::chrono::steady_clock::time_point last_cycle_wall_;
  bool have_last_cycle_{false};
};

}  // namespace prox_mpc_controller

#endif  // PROX_MPC_CONTROLLER__PROX_MPC_CONTROLLER_HPP_
