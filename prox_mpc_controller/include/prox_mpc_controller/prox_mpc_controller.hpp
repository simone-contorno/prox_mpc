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
#include <rcpputils/thread_safety_annotations.hpp>
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

  /// Clear runtime state between tasks and remove the obstacle predictions the
  /// last cycle drew (keeps owned handles intact).
  void reset() override;

protected:
  /// Read the model's declared planar mapping into the cached indices and
  /// offsets, and reject a model this controller cannot drive before any state
  /// or control vector is indexed. EIGEN_NO_DEBUG leaves an out-of-range Eigen
  /// index unchecked, so the screening has to happen here rather than at the
  /// first access. Rejected: fewer than three states, an index outside the
  /// model's own dimensions, two planar quantities sharing one index, a model
  /// carrying a steering angle without declaring a usable wheelbase or with a
  /// lateral reference offset, a steering-rate control index outside the control
  /// vector or colliding with the speed channel. Each rejection throws
  /// nav2_core::ControllerException naming `model_plugin` and the reason.
  ///
  /// The declared position may sit at any pair of state indices, with or
  /// without the in-loop obstacle term: the solver reads the same declaration
  /// and indexes its keep-out rows through it.
  void readModelMapping(prox_mpc::Model & model, const std::string & model_plugin);

  /// Read the model's declared bounds into the cached control-law limits: both
  /// bounds of the speed control into v_max_, v_min_ and max_linear_vel_; the
  /// lower bound of the speed control's `du` entry into fallback_ramp_lin_, and
  /// that of `du[1]` into fallback_ramp_ang_ for a model that has a second
  /// control; every declared `du` bound into du_low_ and du_upp_; the bound of
  /// the declared steering-rate control, when the model has one, into
  /// steer_rate_low_ and steer_rate_upp_. It also sizes last_cmd_u_ to the
  /// model's control dimension. A model that declares none of a required bound
  /// cannot be driven safely - a zero deceleration
  /// limit leaves the brake ramp stuck at the current velocity, and a zero speed
  /// bound clamps the cruise speed to zero - so a missing bound throws
  /// nav2_core::ControllerException naming it. `model_plugin` is the plugin name
  /// reported in that message. Must run after readModelMapping(), whose cached
  /// speed-control index it reads.
  void readModelBounds(prox_mpc::Model & model, const std::string & model_plugin);

  /// Convert a requested speed limit to an absolute bound (a fraction of the
  /// model bound when `percentage`, the model bound itself on NO_SPEED_LIMIT),
  /// and intersect it with the model's own `u[0]` bounds rather than replacing
  /// them, so a model that declares an asymmetric speed range (a reverse limit
  /// tighter than the forward one, or no reverse at all) keeps it. Called only
  /// from the control thread (configure() and the top of a control cycle), never
  /// concurrently with a running solve.
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
  /// per predicted node, centered on the nominal predicted positions. This is
  /// the static (costmap-only) fill and the predict_obstacles_-off fallback.
  void reduceCostmap(MatrixXd & obs);

  /// Windowed costmap scan that fills obstacle slots [slot_begin, max_obstacles_)
  /// per node from the nearest occupied cells, skipping cells inside any per-node
  /// exclusion disc (a dynamic track's footprint). Slots below slot_begin and the
  /// far-sentinel default are left untouched. exclusions[node] = list of
  /// (x, y, radius); an empty vector means no exclusions.
  ///
  /// The scan is centred on the MPC's own nominal predicted trajectory, because
  /// that is what the QP linearizes each node's keep-out half-plane about; a plan
  /// reference centred elsewhere would size the window for a cross-track error
  /// the constraint does not carry. That is why no plan reference is passed here,
  /// while fillObstacles() does take one, for the dynamic-track priority.
  void fillStaticObstacles(
    MatrixXd & obs, std::size_t slot_begin,
    const std::vector<std::vector<std::array<double, 3>>> & exclusions);

  /// Per-node translation carrying an obstacle position from the frame the
  /// keep-out is meant to protect - base_link - into the frame the solver
  /// constrains, which is the model's own reference point. Both vectors are
  /// sized to np_ and left all-zero for a model referenced to base_link, where
  /// the two frames coincide and every obstacle is written unchanged.
  ///
  /// The solver constrains the model's reference point p_ref, so a keep-out
  /// about base_link, p_base = p_ref - R(yaw) * offset, is expressed by moving
  /// the obstacle to o + R(yaw) * offset: the residual is then p_base - o. The
  /// yaw comes from the same nominal predicted trajectory the obstacle scan is
  /// centred on, so it carries that trajectory's one-node staleness.
  void keepOutShift(std::vector<double> & shift_x, std::vector<double> & shift_y);

  /// Cache the latest tracked-obstacle array. This runs on the controller
  /// server's node executor, while computeVelocityCommands runs on the action
  /// server's own execution thread, so the two are concurrent and the exchange
  /// between them must be guarded.
  void obstacleCallback(prox_mpc_msgs::msg::ObstacleArray::ConstSharedPtr msg);

  /// Publish the predicted dynamic-obstacle trajectories as RViz markers when the
  /// debug topic has a subscriber (no-op otherwise).
  void publishPredictedObstacleMarkers(const rclcpp::Time & now);

  /// Close the previous control cycle's timing measurement and open this one's,
  /// returning the inter-cycle wall period [ms] (NaN when there is no previous
  /// cycle). Called once, at the top of computeVelocityCommands(), so every path
  /// out of the cycle - including the ones that brake or throw after the solve -
  /// reads the same measurement, and none of them leaves the reference point
  /// stale for the next cycle.
  double markCycleStart();

  /// Fill and publish one SolverDiagnostics for this control cycle, only when the
  /// publisher exists and the diagnostics topic has a subscriber (zero cost
  /// otherwise). `solve_ms` is the wall time measured around the solve;
  /// `period_ms` is this cycle's inter-cycle period as markCycleStart() measured
  /// it, so the telemetry and the deceleration ramp report the same number;
  /// `converged` reports whether the cycle's command was accepted and applied,
  /// which is what the message's own field documents - the QP's own outcome stays
  /// separately visible in `status`; `num_active_obstacles` is the count of
  /// filled, non-sentinel obstacle slots at the current node.
  void publishDiagnostics(
    const rclcpp::Time & stamp, double solve_ms, double period_ms, bool converged,
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

  /// Tracked-obstacle input and its mutex-guarded latest message. The callback
  /// runs on the node executor and computeVelocityCommands on the action server's
  /// own execution thread, so they are concurrent: the mutex is what makes the
  /// shared-pointer exchange between them safe, and removing it would be a race.
  rclcpp::Subscription<prox_mpc_msgs::msg::ObstacleArray>::SharedPtr obstacle_sub_;
  std::mutex obstacles_mutex_;
  prox_mpc_msgs::msg::ObstacleArray::ConstSharedPtr latest_obstacles_
  RCPPUTILS_TSA_GUARDED_BY(obstacles_mutex_);

  /// One dynamic obstacle's predicted positions over the horizon (costmap global
  /// frame), retained from the fill so the marker publisher need not recompute.
  struct PredictedObstacle
  {
    std::uint32_t id{0};
    double radius{0.0};
    std::vector<std::array<double, 2>> positions;
  };
  std::vector<PredictedObstacle> predicted_obstacles_
  RCPPUTILS_TSA_GUARDED_BY(obstacles_mutex_);

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

  /// Model wheelbase L [m], read back from the loaded model's declared planar
  /// mapping rather than from the model_params.L parameter that was forwarded to
  /// it, so the steering reference is built on the wheelbase the model actually
  /// uses. Meaningful only when has_steering_ is set.
  double wheelbase_{0.0};

  /// The loaded model's declared planar mapping, read once at configure() into
  /// plain members: the control path never queries the model for it.
  std::size_t idx_x_{0};
  std::size_t idx_y_{1};
  std::size_t idx_yaw_{2};
  std::size_t idx_v_{0};
  std::size_t idx_steer_{0};
  std::size_t idx_steer_rate_{0};
  bool has_steering_{false};
  bool has_steer_rate_{false};

  /// Position of the model's reference point in base_link [m]. The Nav2 pose is
  /// carried out to it before the solve and back before the footprint check,
  /// which is defined about base_link. Both are zero unless the model declares
  /// otherwise.
  double ref_offset_x_{0.0};
  double ref_offset_y_{0.0};

  /// Cruise-speed reduction gain on path curvature; 0.0 disables the reduction.
  double curvature_gain_{0.0};

  /// Whether the solver may plan reverse travel at all. Off by default, which
  /// narrows the model's linear control bound to [0, v_max]. Signing the
  /// reference into reverse is a separate opt-in, below.
  bool allow_reversing_{false};

  /// Whether the plan's pose orientations are trusted to encode travel
  /// direction. NavFn and the Smac 2D planners leave every pose at the identity
  /// quaternion, which reads as a reverse plan for any path running against it;
  /// a cusp-emitting planner (Smac Hybrid-A*, State Lattice) means it. Defaults
  /// false, so the reference stays forward-only unless a deployment says its
  /// planner sets orientations.
  bool reverse_from_plan_orientation_{false};

  /// Speed [m/s] at or below which the platform counts as stopped for the
  /// purpose of a travel-direction change. A drivetrain accepts a gear shift
  /// only at rest, and the same gate is what keeps the switch well posed here:
  /// the reference is held at the current arc length until the platform has
  /// actually stopped, so reversing costs a stop rather than a control cycle.
  double direction_switch_standstill_speed_mps_{0.05};

  /// Minimum dwell [s] between two accepted direction changes. Travel direction
  /// is a discrete mode, and a switched system whose mode is re-chosen every
  /// cycle with no dwell time chatters; this is that dwell.
  double direction_switch_dwell_s_{0.5};

  /// Band [m] beyond the goal-checker xy tolerance that the robot must re-cross
  /// before the terminal settle mode releases back to path tracking. Entry and
  /// exit on one threshold would flip mode to mode on tracking noise.
  double goal_settle_hysteresis_m_{0.10};

  /// Deceleration-ramp step [s] on a braking cycle; 0 measures the inter-cycle
  /// period instead, clamped into [dt_, kMaxBrakePeriodFactor * dt_].
  double brake_period_s_{0.0};

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
  /// Forward shadow [s] cast along a tracked obstacle's own heading: the keep-out
  /// centre is biased ahead by this many seconds of its travel and grown by the
  /// same distance, so the space it is about to occupy costs more than the space
  /// it is vacating. 0.0 disables it, reproducing a centred keep-out exactly.
  double prediction_forward_shadow_s_{0.0};
  /// Depth [m] of predicted keep-out breach over which the cruise speed is eased
  /// to zero. When the reference path, driven at this cycle's cruise, would cut
  /// into a tracked mover's predicted keep-out, the cruise is scaled by
  /// 1 + breach / band, so the robot waits for the mover to clear instead of
  /// racing it. 0.0 disables it, reproducing the obstacle-blind cruise exactly.
  double obstacle_yield_band_m_{0.0};

  /// Whether the obstacle-aware yield also caps the solver's forward speed bound,
  /// rather than only lowering the cruise the reference asks for. The cruise is a
  /// lightly weighted target the obstacle term can override - the solver then
  /// swerves at full speed instead of slowing - so a yield that must actually
  /// slow the robot has to bound the speed, not just request less of it. The cap
  /// falls no faster than the platform can brake and leaves the reverse bound
  /// alone, so the robot can still back away. Off by default.
  bool obstacle_yield_caps_speed_{false};
  int max_dynamic_obstacles_{2};               // slot budget for dynamic tracks
  /// Reject tracks larger than this [m] from the predictive path (0 = no limit):
  /// a guard against extended structure (walls) reported as a moving obstacle,
  /// which would otherwise inflate d_safe and erase real costmap cells.
  double max_dynamic_obstacle_radius_{0.0};

  /// Opt-in per-cycle solver telemetry. Off by default so the production plugin
  /// carries no extra interface; when on, publishes only-when-subscribed.
  bool publish_diagnostics_{false};

  /// Speed bounds: v_max_ and v_min_ are the model's original upper and lower
  /// bounds on the speed channel, cached so a runtime speed limit narrows them
  /// instead of overwriting them; max_linear_vel_ is the currently applied limit.
  double v_max_{0.0};
  double v_min_{0.0};
  double max_linear_vel_{0.0};

  /// Rates the last-resort deceleration ramp steps the two body-twist channels
  /// by, taken as the magnitudes of the model's declared lower `du` bounds for
  /// control channels 0 and 1.
  ///
  /// They are bounds on the model's own controls, not on a body twist: for a
  /// bicycle, channel 1 bounds a steering acceleration rather than a yaw one.
  /// The brake ramps the controls themselves under du_low_/du_upp_ and maps them
  /// through the model, so these two are read only when that mapping returns a
  /// non-finite twist and a body-twist rate is the only thing left to ramp. The
  /// names say where they are applied; the model's own rate bounds live in
  /// du_low_/du_upp_.
  double fallback_ramp_lin_{0.5};
  double fallback_ramp_ang_{0.5};

  /// The model's control-rate (du) bounds, indexed by control channel, with each
  /// side's sign preserved so an asymmetric model brakes at its own rate in each
  /// direction. A channel the model declares no bound for is left unbounded, so
  /// the brake takes it to zero in one step rather than freezing it.
  std::vector<double> du_low_;
  std::vector<double> du_upp_;

  /// Last commanded control vector; the ramp's starting point for every control
  /// channel that has no measurement. The speed channel starts from the measured
  /// speed instead, so the brake tracks the robot rather than a stale command.
  VectorXd last_cmd_u_;

  /// Rate bounds of the control the model declares as its steering-angle rate,
  /// used to decay the steering belief on a rejected cycle. Zero when the model
  /// declares no such control, or no bound on it, which keeps the belief frozen.
  double steer_rate_low_{0.0};
  double steer_rate_upp_{0.0};

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

  /// Latched travel direction: +1 forward, -1 reverse. Held across cycles so a
  /// direction change is a deliberate transition subject to the standstill and
  /// dwell gates above, rather than a fresh per-cycle read of the plan geometry.
  double dir_{1.0};

  /// Seconds since the last accepted direction change, advanced by the measured
  /// cycle period. Compared against direction_switch_dwell_s_.
  double dir_hold_s_{0.0};

  /// Whether the terminal settle mode is engaged. Latched: entered inside the
  /// goal-checker xy tolerance and released only past the hysteresis band, so
  /// the reference does not alternate between the two modes near the goal.
  bool settling_{false};

  /// Cruise scale the obstacle-aware yield applied last cycle, in [0, 1]. It
  /// drops at once when a breach deepens but recovers at a bounded rate, so a
  /// breach that clears for one cycle cannot snap the cruise back to full.
  double yield_factor_{1.0};

  /// Inter-cycle wall clock, read and advanced by markCycleStart() at the top of
  /// every control cycle; reset between tasks so the first cycle of a task
  /// reports NaN rather than a stale gap.
  std::chrono::steady_clock::time_point last_cycle_wall_;
  bool have_last_cycle_{false};
};

}  // namespace prox_mpc_controller

#endif  // PROX_MPC_CONTROLLER__PROX_MPC_CONTROLLER_HPP_
