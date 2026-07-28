// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


#include "prox_mpc_controller/prox_mpc_controller.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_core/controller_exceptions.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <nav2_util/node_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>

namespace
{

/// Map proxsuite's solver outcome onto the message's own STATUS_* contract.
///
/// Deliberately not a static_cast: proxsuite 0.6.5 inserted
/// PROXQP_SOLVED_CLOSEST_PRIMAL_FEASIBLE into the middle of QPSolverOutput, so a
/// cast silently reports a dual-infeasible solve as STATUS_NOT_RUN when built
/// against it. An enumerator added upstream after this mapping was written falls
/// through to STATUS_UNKNOWN rather than impersonating another state.
std::uint8_t solverStatusToMsg(proxsuite::proxqp::QPSolverOutput status)
{
  using QPOut = proxsuite::proxqp::QPSolverOutput;
  using Diag = prox_mpc_msgs::msg::SolverDiagnostics;
  switch (status) {
    case QPOut::PROXQP_SOLVED: return Diag::STATUS_SOLVED;
    case QPOut::PROXQP_MAX_ITER_REACHED: return Diag::STATUS_MAX_ITER_REACHED;
    case QPOut::PROXQP_PRIMAL_INFEASIBLE: return Diag::STATUS_PRIMAL_INFEASIBLE;
    case QPOut::PROXQP_SOLVED_CLOSEST_PRIMAL_FEASIBLE:
      return Diag::STATUS_SOLVED_CLOSEST_PRIMAL_FEASIBLE;
    case QPOut::PROXQP_DUAL_INFEASIBLE: return Diag::STATUS_DUAL_INFEASIBLE;
    case QPOut::PROXQP_NOT_RUN: return Diag::STATUS_NOT_RUN;
  }
  return Diag::STATUS_UNKNOWN;
}

/// Stop speed below which a cancel ramp is considered complete [m/s, rad/s].
constexpr double kCancelStopEpsilon = 0.01;
/// Upper bound on the costmap scan half-window [cells] to keep the per-cycle cost
/// bounded on constrained hardware.
constexpr int kMaxScanHalfWidth = 50;
/// Minimum strictly-positive cost weight, keeping the QP Hessian positive definite
/// (a negative weight would make the sub-problem non-convex).
constexpr double kMinCostWeight = 1e-9;
/// Discrete-time CBF rate floor; the rate must lie in (0, 1] (1.0 = pointwise term).
constexpr double kMinCbfGamma = 1e-3;
/// Largest costmap cost still treated as an obstacle threshold (255 = NO_INFORMATION,
/// handled separately; a higher threshold would silently disable avoidance).
constexpr int kMaxCostThreshold = 254;

/// Planar yaw from a quaternion.
double quat_yaw(const geometry_msgs::msg::Quaternion & q)
{
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny, cosy);
}

/// One deceleration step toward zero, respecting the sign of the previous value.
/// A non-finite previous value (NaN or +/-inf) yields zero: an infinite measured
/// velocity would otherwise survive the ramp and be published as the command.
double brake_toward(double prev, double decel, double dt)
{
  if (!std::isfinite(prev)) {return 0.0;}
  const double step = std::abs(decel) * dt;
  if (prev > 0.0) {return std::max(0.0, prev - step);}
  if (prev < 0.0) {return std::min(0.0, prev + step);}
  return 0.0;
}

/// Position at time t > 0 along the tracker's sampled prediction polyline
/// {(0, (x0, y0)), (k * sample_dt, samples[k-1])}: piecewise-linear
/// interpolation inside the sampled span, and extrapolation along the last
/// segment's direction beyond it (straight-line, the best remaining guess).
/// samples must be non-empty and sample_dt > 0.
std::array<double, 2> sample_polyline(
  double x0, double y0, const std::vector<std::array<double, 2>> & samples,
  double sample_dt, double t)
{
  const std::size_t n = samples.size();
  const double span = static_cast<double>(n) * sample_dt;
  if (t >= span) {
    const std::array<double, 2> & last = samples[n - 1];
    const double prev_x = (n >= 2) ? samples[n - 2][0] : x0;
    const double prev_y = (n >= 2) ? samples[n - 2][1] : y0;
    const double s = (t - span) / sample_dt;
    return {last[0] + (last[0] - prev_x) * s, last[1] + (last[1] - prev_y) * s};
  }
  // Segment index; the min guards the t ~ span floating-point edge.
  const std::size_t k = std::min(static_cast<std::size_t>(t / sample_dt), n - 1);
  const double alpha = (t - static_cast<double>(k) * sample_dt) / sample_dt;
  const double ax = (k == 0) ? x0 : samples[k - 1][0];
  const double ay = (k == 0) ? y0 : samples[k - 1][1];
  const std::array<double, 2> & b = samples[k];
  return {ax + (b[0] - ax) * alpha, ay + (b[1] - ay) * alpha};
}

/// One declared inequality bound of the model: entry layout is
/// (vector index, lower, upper), so `which` selects 1 for the lower bound and 2
/// for the upper one. A model that does not declare the bound cannot be driven
/// safely, so this throws instead of substituting a value.
double required_bound(
  prox_mpc::Model & model, const std::string & model_plugin, const std::string & var,
  std::size_t idx, std::size_t which, const std::string & channel,
  const std::string & consequence)
{
  for (const auto & entry : model.getIneq(var)) {
    if (static_cast<std::size_t>(entry.second[0]) == idx) {return entry.second[which];}
  }
  throw nav2_core::ControllerException(
          "ProxMpcController: model '" + model_plugin + "' declares no '" + var +
          "' bound for the " + channel + " control; " + consequence);
}

/// Apply the project log_level key to the plugin's own logger.
void apply_log_level(rclcpp::Logger & logger, const std::string & level)
{
  if (level == "debug") {
    logger.set_level(rclcpp::Logger::Level::Debug);
  } else if (level == "info") {
    logger.set_level(rclcpp::Logger::Level::Info);
  } else if (level == "warn") {logger.set_level(rclcpp::Logger::Level::Warn);} else if (
    level == "error")
  {
    logger.set_level(rclcpp::Logger::Level::Error);
  } else if (level == "fatal") {logger.set_level(rclcpp::Logger::Level::Fatal);} else {
    logger.set_level(rclcpp::Logger::Level::Info);
  }
}

}  // namespace

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
  clock_ = node->get_clock();

  /* Parameter-validation helpers: out-of-range tuning values are clamped and
   * warned (non-fatal, to keep the controller available), matching the predictive
   * clamps; structurally invalid horizon sizing is fatal and throws below. */
  auto clamp_low = [this](const char * name, double & v, double lo) {
      if (v < lo) {
        RCLCPP_WARN(logger_, "%s %.3f below %.3f; clamping.", name, v, lo);
        v = lo;
      }
    };
  auto clamp_range = [this](const char * name, double & v, double lo, double hi) {
      if (v < lo || v > hi) {
        const double c = std::clamp(v, lo, hi);
        RCLCPP_WARN(
          logger_, "%s %.3f outside [%.3f, %.3f]; clamping to %.3f.", name, v, lo, hi, c);
        v = c;
      }
    };

  /* Parameters live under the plugin-instance namespace (e.g. FollowPath.*) on
   * the controller server's node. The server destroys and recreates the plugin
   * across a cleanup()/configure() cycle while its node keeps the parameters
   * declared, so an unguarded declaration would throw
   * rclcpp::exceptions::ParameterAlreadyDeclaredException - not a
   * nav2_core::ControllerException - and fail the lifecycle transition. */
  const std::string p = plugin_name_ + ".";
  auto declare = [&node, &p](const std::string & key, auto & value, const auto & fallback) {
      nav2_util::declare_parameter_if_not_declared(
        node, p + key, rclcpp::ParameterValue(fallback));
      node->get_parameter(p + key, value);
    };

  std::string model_plugin;
  declare("model_plugin", model_plugin, std::string("prox_mpc_core/Bicycle"));
  double model_l = 0.0;
  declare("model_params.L", model_l, 1.6);
  /* Optional model input-velocity bound [m/s]; 0.0 (default) keeps the model's
   * built-in limit. Sourced from the robot config's max_linear_vel, so the speed
   * cap is a property of the model constraint rather than a controller output
   * clamp. A positive v_max caps the forward input; v_min defaults to -v_max. */
  double model_v_max = 0.0;
  declare("model_params.v_max", model_v_max, 0.0);
  double model_v_min = 0.0;
  declare("model_params.v_min", model_v_min, 0.0);

  /* Horizon sizing and step are structural: np/nc below 1 wrap to an astronomical
   * size_t allocation and dt <= 0 divides by zero, so fail configure outright. */
  int np_param = 0;
  declare("np", np_param, 20);
  int nc_param = 0;
  declare("nc", nc_param, 20);
  declare("dt", dt_, 0.1);
  if (np_param < 1 || nc_param < 1 || dt_ <= 0.0) {
    throw nav2_core::ControllerException(
            "ProxMpcController: np >= 1, nc >= 1, dt > 0 required (got np=" +
            std::to_string(np_param) + ", nc=" + std::to_string(nc_param) + ", dt=" +
            std::to_string(dt_) + ")");
  }
  np_ = static_cast<std::size_t>(np_param);
  nc_ = static_cast<std::size_t>(nc_param);

  declare("desired_linear_vel", desired_linear_vel_, 1.0);
  declare("curvature_gain", curvature_gain_, 0.0);

  /* Cost weights must be non-negative: a negative weight makes the QP Hessian
   * indefinite (non-convex sub-problem). Floor them at a small positive value. */
  double q_pos = 0.0;
  declare("q_pos", q_pos, 10.0);
  double q_theta = 0.0;
  declare("q_theta", q_theta, 1.0);
  double s_factor = 0.0;
  declare("s_factor", s_factor, 2.0);
  double r_weight = 0.0;
  declare("r_weight", r_weight, 0.1);
  double w_weight = 0.0;
  declare("w_weight", w_weight, 100.0);
  clamp_low("q_pos", q_pos, kMinCostWeight);
  clamp_low("q_theta", q_theta, kMinCostWeight);
  clamp_low("s_factor", s_factor, kMinCostWeight);
  clamp_low("r_weight", r_weight, kMinCostWeight);
  clamp_low("w_weight", w_weight, kMinCostWeight);

  /* Iteration caps are structural, like the horizon sizing: they reach the core
   * as size_t, so a negative value wraps to an astronomical bound (an effectively
   * unbounded SQP loop), and ProxQP rejects a zero cap outright. Both are fatal -
   * a one-iteration fallback would be a silent, never-converging bringup. */
  int max_int_iter_qp = 0;
  declare("max_int_iter_qp", max_int_iter_qp, 1500);
  int max_ext_iter_qp = 0;
  declare("max_ext_iter_qp", max_ext_iter_qp, 10000);
  int max_iter_sqp = 0;
  declare("max_iter_sqp", max_iter_sqp, 100);
  if (max_int_iter_qp < 1 || max_ext_iter_qp < 1 || max_iter_sqp < 1) {
    throw nav2_core::ControllerException(
            "ProxMpcController: max_int_iter_qp >= 1, max_ext_iter_qp >= 1, max_iter_sqp >= 1 "
            "required (got max_int_iter_qp=" + std::to_string(max_int_iter_qp) +
            ", max_ext_iter_qp=" + std::to_string(max_ext_iter_qp) + ", max_iter_sqp=" +
            std::to_string(max_iter_sqp) + ")");
  }
  /* Optional wall-clock budget [s] for the whole SQP loop (0 = disabled, iteration
   * caps only). On timeout the solve reports non-convergence and this cycle brakes. */
  double max_solve_time = 0.0;
  declare("max_solve_time", max_solve_time, 0.0);
  clamp_low("max_solve_time", max_solve_time, 0.0);
  bool qp_type = false;
  declare("qp_type", qp_type, false);
  bool guess = true;
  declare("guess", guess, true);
  declare("max_solver_failures", max_solver_failures_, 3);
  declare("max_obstacles", max_obstacles_, 1);
  if (max_obstacles_ < 0) {
    RCLCPP_WARN(logger_, "max_obstacles %d < 0; clamping to 0.", max_obstacles_);
    max_obstacles_ = 0;
  }
  declare("safety_margin", safety_margin_, 0.1);
  declare("robot_radius", robot_radius_, 0.5);
  clamp_low("safety_margin", safety_margin_, 0.0);
  clamp_low("robot_radius", robot_radius_, 0.0);
  declare("cbf_gamma", cbf_gamma_, 1.0);
  clamp_range("cbf_gamma", cbf_gamma_, kMinCbfGamma, 1.0);
  declare("costmap_cost_threshold", costmap_cost_threshold_, 200);
  if (costmap_cost_threshold_ < 0 || costmap_cost_threshold_ > kMaxCostThreshold) {
    const int c = std::clamp(costmap_cost_threshold_, 0, kMaxCostThreshold);
    RCLCPP_WARN(
      logger_, "costmap_cost_threshold %d outside [0, %d]; clamping to %d.",
      costmap_cost_threshold_, kMaxCostThreshold, c);
    costmap_cost_threshold_ = c;
  }
  declare("obstacle_cluster_radius", obstacle_cluster_radius_, 0.3);
  clamp_low("obstacle_cluster_radius", obstacle_cluster_radius_, 0.0);
  declare("max_obstacle_scan_cells", max_obstacle_scan_cells_, kMaxScanHalfWidth);
  if (max_obstacle_scan_cells_ < 1) {
    RCLCPP_WARN(
      logger_, "max_obstacle_scan_cells %d below 1; clamping to 1.",
      max_obstacle_scan_cells_);
    max_obstacle_scan_cells_ = 1;
  }

  /* Predictive (dynamic) obstacle avoidance. predict_obstacles off reproduces the
   * costmap-only behavior bit-for-bit; the rest size the predictive + hybrid fill. */
  declare("predict_obstacles", predict_obstacles_, false);
  declare("obstacle_topic", obstacle_topic_, std::string("tracked_obstacles"));
  declare("obstacle_timeout", obstacle_timeout_, 0.5);
  declare("dynamic_speed_threshold", dynamic_speed_threshold_, 0.1);
  declare("prediction_uncertainty_growth", prediction_uncertainty_growth_, 0.0);
  declare("max_dynamic_obstacles", max_dynamic_obstacles_, 2);
  declare("max_dynamic_obstacle_radius", max_dynamic_obstacle_radius_, 0.0);

  /* Opt-in solver telemetry (off by default); publishes only-when-subscribed. */
  declare("publish_diagnostics", publish_diagnostics_, false);

  /* Validate the predictive parameters; clamp out-of-range values (non-fatal, to
   * keep the controller available) and warn, matching the cruise-speed clamp. */
  clamp_low("obstacle_timeout", obstacle_timeout_, 0.0);
  clamp_low("dynamic_speed_threshold", dynamic_speed_threshold_, 0.0);
  clamp_low("prediction_uncertainty_growth", prediction_uncertainty_growth_, 0.0);
  clamp_low("max_dynamic_obstacle_radius", max_dynamic_obstacle_radius_, 0.0);
  if (max_dynamic_obstacles_ < 0) {
    RCLCPP_WARN(logger_, "max_dynamic_obstacles %d < 0; clamping to 0.", max_dynamic_obstacles_);
    max_dynamic_obstacles_ = 0;
  }

  std::string log_level;
  declare("log_level", log_level, std::string("info"));

  /* Keep the plugin's own ProxMpcController logger (do not adopt the server's),
   * and seed its level from the project log_level key. */
  apply_log_level(logger_, log_level);

  /* Load and configure the model plugin; the handle is retained for toTwist and
   * runtime speed-limit updates. */
  try {
    model_loader_ =
      std::make_shared<pluginlib::ClassLoader<prox_mpc::Model>>("prox_mpc_core", "prox_mpc::Model");
    model_ = model_loader_->createSharedInstance(model_plugin);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(logger_, "Failed to load model plugin '%s': %s", model_plugin.c_str(), ex.what());
    throw nav2_core::ControllerException(
            std::string("ProxMpcController: failed to load model plugin: ") + ex.what());
  }
  std::map<std::string, double> model_params{{"L", model_l}};
  if (model_v_max > 0.0) {
    model_params["v_max"] = model_v_max;
    model_params["v_min"] = (model_v_min < 0.0) ? model_v_min : -model_v_max;
  }
  model_->configure(model_params);
  wheelbase_ = model_l;
  n_ = model_->getN();
  m_ = model_->getM();

  readModelBounds(*model_, model_plugin);

  /* Cruise speed must sit within the model's speed bound. */
  if (desired_linear_vel_ > v_max_) {
    RCLCPP_WARN(
      logger_, "desired_linear_vel %.3f exceeds model v_max %.3f; clamping.",
      desired_linear_vel_, v_max_);
    desired_linear_vel_ = v_max_;
  }

  /* Cost weights sized to the model (matches the demo assembly). */
  VectorXd q_diag = VectorXd::Constant(n_, q_theta);
  q_diag(0) = q_pos;
  q_diag(1) = q_pos;
  const MatrixXd Q = q_diag.asDiagonal();
  const MatrixXd S = s_factor * Q;
  const MatrixXd R = r_weight * MatrixXd::Identity(m_, m_);
  const MatrixXd W = MatrixXd::Constant(1, 1, w_weight);

  /* Size and initialize the MPC once. init() invokes configProxQP() internally,
   * so the QP is sized here for the configured horizons, weights, and K. */
  const std::size_t k_obs =
    (model_->getObsFlag() && max_obstacles_ > 0) ? static_cast<std::size_t>(max_obstacles_) : 0;
  mpc_ = std::make_shared<prox_mpc::MPC>();
  mpc_->setNp(np_);
  mpc_->setNc(nc_);
  mpc_->setdt(dt_);
  mpc_->setQ(Q);
  mpc_->setS(S);
  mpc_->setR(R);
  mpc_->setW(W);
  mpc_->setMaxIntIterQP(static_cast<std::size_t>(max_int_iter_qp));
  mpc_->setMaxExtIterQP(static_cast<std::size_t>(max_ext_iter_qp));
  mpc_->setMaxIterSQP(static_cast<std::size_t>(max_iter_sqp));
  mpc_->setMaxSolveTime(max_solve_time);
  mpc_->setGuess(guess);
  mpc_->setQPtype(qp_type);
  mpc_->setCbfGamma(cbf_gamma_);
  mpc_->setMaxObs(k_obs);
  mpc_->init(model_);

  /* Predicted-trajectory publisher (visualization): the NMPC horizon as a Path in
   * the costmap global frame, distinct from the Nav2 global plan. */
  traj_pub_ = node->create_publisher<nav_msgs::msg::Path>("prox_mpc_local_plan", 1);

  /* Predictive obstacle avoidance: subscribe to the tracked-obstacle array
   * (reliable, depth 5) and create the optional RViz marker publisher. Created
   * only when enabled so the costmap-only default carries no extra interfaces. */
  if (predict_obstacles_) {
    obstacle_sub_ = node->create_subscription<prox_mpc_msgs::msg::ObstacleArray>(
      obstacle_topic_, rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
      std::bind(&ProxMpcController::obstacleCallback, this, std::placeholders::_1));
    marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "prox_mpc_predicted_obstacles", 1);
  }

  /* Opt-in per-cycle solver telemetry, namespaced under the plugin so it reads as
   * <plugin>/diagnostics (e.g. FollowPath/diagnostics). Reliable, depth 10. */
  if (publish_diagnostics_) {
    diag_pub_ = node->create_publisher<prox_mpc_msgs::msg::SolverDiagnostics>(
      plugin_name_ + "/diagnostics", rclcpp::QoS(10).reliable());
  }

  /* Apply a speed limit received before the model was available. configure() does
   * not run concurrently with the solver, so it is applied in place here rather
   * than deferred to the first control cycle. */
  if (speed_limit_pending_.exchange(false, std::memory_order_acquire)) {
    applySpeedLimit(
      speed_limit_.load(std::memory_order_relaxed),
      speed_limit_is_percentage_.load(std::memory_order_relaxed));
  }

  RCLCPP_INFO(
    logger_, "Configured ProxMpcController '%s' (model '%s', Np=%zu, Nc=%zu, dt=%.3f, K=%zu).",
    plugin_name_.c_str(), model_plugin.c_str(), np_, nc_, dt_, k_obs);
}

void ProxMpcController::readModelBounds(prox_mpc::Model & model, const std::string & model_plugin)
{
  v_max_ = required_bound(
    model, model_plugin, "u", 0, 2, "linear",
    "the speed cap would collapse to zero and the controller would never move.");
  max_linear_vel_ = v_max_;
  a_dec_lin_ = std::abs(
    required_bound(
      model, model_plugin, "du", 0, 1, "linear",
      "the solver-failure brake would never reach zero."));
  a_dec_ang_ = std::abs(
    required_bound(
      model, model_plugin, "du", 1, 1, "angular",
      "the solver-failure brake would never reach zero."));
}

void ProxMpcController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up ProxMpcController '%s'.", plugin_name_.c_str());
  mpc_.reset();
  model_.reset();
  model_loader_.reset();
  traj_pub_.reset();
  marker_pub_.reset();
  diag_pub_.reset();
  obstacle_sub_.reset();
  {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    latest_obstacles_.reset();
  }
  predicted_obstacles_.clear();
  costmap_ros_.reset();
  tf_.reset();
}

void ProxMpcController::activate()
{
  failure_count_ = 0;
  veto_count_ = 0;
  steering_state_ = 0.0;
  last_cmd_v_ = 0.0;
  last_cmd_w_ = 0.0;
  cancelling_ = false;
  have_last_cycle_ = false;
  if (traj_pub_) {traj_pub_->on_activate();}
  if (marker_pub_) {marker_pub_->on_activate();}
  if (diag_pub_) {diag_pub_->on_activate();}
  RCLCPP_INFO(logger_, "Activating ProxMpcController '%s'.", plugin_name_.c_str());
}

void ProxMpcController::deactivate()
{
  if (traj_pub_) {traj_pub_->on_deactivate();}
  if (marker_pub_) {marker_pub_->on_deactivate();}
  if (diag_pub_) {diag_pub_->on_deactivate();}
  /* Drop cached perception so a re-activated controller does not act on a tracked
   * obstacle observed before deactivation; it resumes costmap-only until a fresh
   * message arrives (the staleness timeout would also catch this). */
  {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    latest_obstacles_.reset();
  }
  predicted_obstacles_.clear();
  RCLCPP_INFO(logger_, "Deactivating ProxMpcController '%s'.", plugin_name_.c_str());
}

void ProxMpcController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  plan_index_ = 0;
}

geometry_msgs::msg::TwistStamped ProxMpcController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  /* Apply a speed limit requested since the last cycle. setSpeedLimit() runs on
   * the node's executor thread while this method runs on the action server's own
   * thread, so the model's inequality map is mutated here, where nothing else
   * reads it. The only behavioral difference is that a new limit takes effect on
   * the next control cycle rather than mid-cycle. */
  if (speed_limit_pending_.exchange(false, std::memory_order_acquire)) {
    applySpeedLimit(
      speed_limit_.load(std::memory_order_relaxed),
      speed_limit_is_percentage_.load(std::memory_order_relaxed));
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = costmap_ros_->getBaseFrameID();
  cmd.header.stamp = clock_->now();

  /* Deceleration ramp shared by the cancel, solver-failure, and veto paths. It
   * ramps down from the server-measured velocity (RPP/MPPI style) so the brake
   * tracks the robot's actual speed rather than the last command, which may be
   * stale (for example a cycle-1 failure while already moving). A non-finite
   * measured velocity yields a safe zero through brake_toward. */
  auto make_brake = [&]() -> geometry_msgs::msg::TwistStamped {
      last_cmd_v_ = brake_toward(velocity.linear.x, a_dec_lin_, dt_);
      last_cmd_w_ = brake_toward(velocity.angular.z, a_dec_ang_, dt_);
      cmd.twist.linear.x = last_cmd_v_;
      cmd.twist.angular.z = last_cmd_w_;
      return cmd;
    };

  /* Transient-failure path: decelerate, and escalate to a recovery once the
   * consecutive-failure budget is exhausted. */
  auto fail = [&](const std::string & why) -> geometry_msgs::msg::TwistStamped {
      failure_count_++;
      if (failure_count_ > max_solver_failures_) {
        throw nav2_core::NoValidControl("ProxMpcController: " + why);
      }
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000, "ProxMpcController: %s; decelerating (failure %d/%d).",
        why.c_str(), failure_count_, max_solver_failures_);
      return make_brake();
    };

  /* Current pose in the costmap global frame (the server supplies it there). */
  const double cx = pose.pose.position.x;
  const double cy = pose.pose.position.y;
  const double ctheta = quat_yaw(pose.pose.orientation);
  if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(ctheta)) {
    return fail("non-finite robot pose");
  }

  if (cancelling_) {return make_brake();}

  /* A missing or empty plan is a structural fault, not a transient one. */
  if (global_plan_.poses.empty()) {
    throw nav2_core::InvalidPath("ProxMpcController: received an empty global plan");
  }

  /* Transform the plan into the costmap global frame (single planar transform). */
  const std::string global_frame = costmap_ros_->getGlobalFrameID();
  const std::string plan_frame = global_plan_.header.frame_id;
  double tx = 0.0;
  double ty = 0.0;
  double tyaw = 0.0;
  if (!plan_frame.empty() && plan_frame != global_frame) {
    try {
      const geometry_msgs::msg::TransformStamped tfs =
        tf_->lookupTransform(global_frame, plan_frame, tf2::TimePointZero);
      tx = tfs.transform.translation.x;
      ty = tfs.transform.translation.y;
      tyaw = quat_yaw(tfs.transform.rotation);
    } catch (const tf2::TransformException & ex) {
      throw nav2_core::ControllerTFError(std::string("ProxMpcController: ") + ex.what());
    }
  }

  const double ct = std::cos(tyaw);
  const double st = std::sin(tyaw);
  const std::size_t plan_size = global_plan_.poses.size();
  std::vector<double> gx(plan_size);
  std::vector<double> gy(plan_size);
  std::vector<double> s(plan_size, 0.0);
  for (std::size_t i = 0; i < plan_size; ++i) {
    const auto & pp = global_plan_.poses[i].pose.position;
    gx[i] = tx + ct * pp.x - st * pp.y;
    gy[i] = ty + st * pp.x + ct * pp.y;
    if (i > 0) {
      s[i] = s[i - 1] + std::hypot(gx[i] - gx[i - 1], gy[i] - gy[i - 1]);
    }
  }

  /* Project the current pose onto the plan (forward-only), giving the arc-length
   * offset s0 the sampling starts from. */
  if (plan_index_ >= plan_size) {plan_index_ = 0;}
  std::size_t best = plan_index_;
  double best_d2 = std::numeric_limits<double>::max();
  for (std::size_t i = plan_index_; i < plan_size; ++i) {
    const double d2 = (gx[i] - cx) * (gx[i] - cx) + (gy[i] - cy) * (gy[i] - cy);
    if (d2 < best_d2) {
      best_d2 = d2;
      best = i;
    }
  }
  plan_index_ = best;
  const double s0 = s[best];

  /* Cruise speed, tapered so the horizon does not overshoot the plan end, and
   * clamped by any active speed limit (goal-hold near the end). */
  const double horizon_time = static_cast<double>(np_) * dt_;
  const double remaining = s.back() - s0;
  double v_ref = std::min({desired_linear_vel_, max_linear_vel_, remaining / horizon_time});
  if (v_ref < 0.0) {v_ref = 0.0;}

  /* Sample a plan pose at a given arc length (holds the final pose past the end). */
  auto sample = [&](double sk, double & x, double & y, double & th) {
      if (sk <= 0.0) {sk = 0.0;}
      if (sk >= s.back() || plan_size < 2) {
        x = gx.back();
        y = gy.back();
        /* Single-pose plan: the pose orientation is in the plan frame, so it
         * carries the same yaw offset the positions above were rotated by. */
        th = (plan_size < 2) ? quat_yaw(global_plan_.poses.back().pose.orientation) + tyaw :
          std::atan2(gy.back() - gy[plan_size - 2], gx.back() - gx[plan_size - 2]);
        return;
      }
      std::size_t i = 0;
      for (std::size_t j = 0; j + 1 < plan_size; ++j) {
        if (s[j] <= sk && sk <= s[j + 1]) {
          i = j;
          break;
        }
      }
      const double seg = s[i + 1] - s[i];
      const double t = seg > 1e-9 ? (sk - s[i]) / seg : 0.0;
      x = gx[i] + t * (gx[i + 1] - gx[i]);
      y = gy[i] + t * (gy[i + 1] - gy[i]);
      th = std::atan2(gy[i + 1] - gy[i], gx[i + 1] - gx[i]);
    };

  /* Ease the cruise speed inside the goal-checker xy tolerance so the robot
   * settles into the goal region. Unmeasured tolerance fields come back as
   * std::numeric_limits<double>::lowest() (negative), so accept only finite,
   * positive values. The pointer is read but never retained. */
  if (goal_checker != nullptr) {
    geometry_msgs::msg::Pose pose_tol;
    geometry_msgs::msg::Twist vel_tol;
    if (goal_checker->getTolerances(pose_tol, vel_tol)) {
      const double xy_tol = std::hypot(pose_tol.position.x, pose_tol.position.y);
      if (std::isfinite(xy_tol) && pose_tol.position.x > 0.0 && pose_tol.position.y > 0.0) {
        v_ref *= std::clamp(remaining / xy_tol, 0.0, 1.0);
      }
    }
  }

  /* Curvature-aware cruise reduction (inert when curvature_gain_ == 0): estimate
   * the peak path curvature over the horizon from heading samples at the current
   * cruise, then taper v_ref so high-curvature segments are sampled more slowly. */
  if (curvature_gain_ > 0.0 && v_ref > 0.0) {
    const double ds = v_ref * dt_;
    double prev_th = ctheta;
    double max_kappa = 0.0;
    for (std::size_t k = 0; k <= np_; ++k) {
      double sx = 0.0;
      double sy = 0.0;
      double sth = 0.0;
      sample(s0 + v_ref * static_cast<double>(k) * dt_, sx, sy, sth);
      const double dth = std::remainder(sth - prev_th, 2.0 * M_PI);
      if (ds > 1e-9) {max_kappa = std::max(max_kappa, std::abs(dth) / ds);}
      prev_th += dth;
    }
    v_ref /= 1.0 + curvature_gain_ * max_kappa;
  }

  /* Build the state and control references. Heading is kept continuous (unwrapped
   * relative to the robot heading, then node to node) so the QP tracking error
   * never wraps near +/-pi. */
  MatrixXd goal_x = MatrixXd::Zero(np_ + 1, n_);
  std::vector<double> th_cont(np_ + 1, 0.0);
  double prev_th = ctheta;
  for (std::size_t k = 0; k <= np_; ++k) {
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;
    sample(s0 + v_ref * static_cast<double>(k) * dt_, x, y, th);
    th = prev_th + std::remainder(th - prev_th, 2.0 * M_PI);
    prev_th = th;
    th_cont[k] = th;
    goal_x(k, 0) = x;
    goal_x(k, 1) = y;
    goal_x(k, 2) = th;
  }

  /* Bicycle steering reference: pre-position the wheel to the per-node path
   * curvature kappa = dtheta/ds, delta_ref = atan(L * kappa). Models without a
   * steering state (n_ == 3) keep the go-straight default (channel left at zero). */
  if (n_ > 3) {
    const double ds = v_ref * dt_;
    for (std::size_t k = 0; k <= np_; ++k) {
      double kappa = 0.0;
      if (ds > 1e-9) {
        kappa = (k < np_) ? (th_cont[k + 1] - th_cont[k]) / ds :
          (th_cont[k] - th_cont[k - 1]) / ds;
      }
      goal_x(k, 3) = std::atan(wheelbase_ * kappa);
    }
  }

  MatrixXd goal_u = MatrixXd::Zero(nc_, m_);
  for (std::size_t k = 0; k < nc_; ++k) {
    goal_u(k, 0) = v_ref;
  }

  /* Current full state, tracking the bicycle steering angle the Nav2 pose omits. */
  VectorXd state = VectorXd::Zero(n_);
  state(0) = cx;
  state(1) = cy;
  state(2) = ctheta;
  if (n_ > 3) {state(3) = steering_state_;}

  /* Fill the per-node obstacle triples for this cycle: predictive + hybrid when
   * enabled and fresh tracking data is available, else costmap-only. */
  const std::size_t k_obs = mpc_->getMaxObs();
  std::uint16_t num_active_obs = 0;
  if (k_obs > 0) {
    MatrixXd obs(static_cast<Eigen::Index>(np_ * k_obs), 3);
    fillObstacles(goal_x, obs, cmd.header.stamp);
    mpc_->setObs(obs);
    /* Count filled (non-sentinel) slots at the current node (rows 0..k_obs-1). */
    for (std::size_t s = 0; s < k_obs; ++s) {
      if (obs(static_cast<Eigen::Index>(s), 0) < 0.5 * prox_mpc::MPC::kObsFarSentinel) {
        ++num_active_obs;
      }
    }
  }

  /* Solve one SQP cycle, timing it with a steady clock for the real-time metric. */
  mpc_->setGoalX(goal_x);
  mpc_->setGoalU(goal_u);
  mpc_->setPose(state);
  const auto t_solve0 = std::chrono::steady_clock::now();
  auto [x_sol, u_sol] = mpc_->solve();
  const auto t_solve1 = std::chrono::steady_clock::now();
  const double solve_ms =
    std::chrono::duration<double, std::milli>(t_solve1 - t_solve0).count();

  const bool solved =
    (mpc_->qp_info.status == proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED);

  /* Publish per-cycle telemetry before the gate's early returns, so every solve
   * (including non-converged ones) is observable when diagnostics are enabled. */
  publishDiagnostics(cmd.header.stamp, solve_ms, solved, num_active_obs);

  if (!solved) {return fail("solver did not converge");}

  /* Defense-in-depth finiteness guard. In this architecture a PROXQP_SOLVED status
   * normally implies a finite iterate (x_sol/u_sol are accumulated QP increments,
   * not a divergent model rollout), but the check is kept so a non-finite state can
   * never reach the footprint veto, poison steering_state_, or be published as the
   * predicted trajectory. The command finiteness is still checked separately below
   * because the model's toTwist mapping can be non-finite even for finite u0. */
  const VectorXd u0 = u_sol.row(0);
  if (!u0.allFinite() || !x_sol.row(1).allFinite()) {
    return fail("non-finite solver output");
  }

  /* Exact polygon-footprint veto on the pose one step ahead. */
  {
    auto * costmap = costmap_ros_->getCostmap();
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
    const std::vector<geometry_msgs::msg::Point> footprint = costmap_ros_->getRobotFootprint();
    if (footprint.size() >= 3) {
      nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap);
      const double fcost = checker.footprintCostAtPose(
        x_sol(1, 0), x_sol(1, 1), x_sol(1, 2), footprint);
      if (fcost >= static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) {
        /* Escalate a persistent veto so a robot stuck behind a static obstacle one
         * step ahead triggers the behavior-tree recovery instead of braking
         * forever. The veto counter is kept separate from the solver-failure
         * budget, so a single veto still does not trip recovery. */
        veto_count_++;
        if (veto_count_ > max_solver_failures_) {
          throw nav2_core::NoValidControl(
                  "ProxMpcController: footprint veto persisted beyond the budget");
        }
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 2000,
          "ProxMpcController: footprint check vetoed the command; decelerating "
          "(veto %d/%d).", veto_count_, max_solver_failures_);
        return make_brake();
      }
    } else {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "ProxMpcController: footprint has %zu points (<3); polygon veto skipped this "
        "cycle (the in-loop QP half-planes still apply).", footprint.size());
    }
  }

  /* Map the first control to a body twist; the model reads the current state. */
  model_->setX(state);
  const geometry_msgs::msg::Twist twist = model_->toTwist(u0);
  if (!std::isfinite(twist.linear.x) || !std::isfinite(twist.angular.z)) {
    return fail("non-finite command");
  }

  failure_count_ = 0;
  veto_count_ = 0;
  if (n_ > 3) {steering_state_ = x_sol(1, 3);}
  last_cmd_v_ = twist.linear.x;
  last_cmd_w_ = twist.angular.z;

  /* Publish the predicted NMPC trajectory (costmap global frame) for
   * visualization, distinct from the Nav2 global plan. Skipped when unsubscribed. */
  if (traj_pub_ && traj_pub_->get_subscription_count() > 0) {
    nav_msgs::msg::Path traj;
    traj.header.frame_id = global_frame;
    traj.header.stamp = cmd.header.stamp;
    traj.poses.reserve(static_cast<std::size_t>(x_sol.rows()));
    for (Eigen::Index k = 0; k < x_sol.rows(); ++k) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = traj.header;
      ps.pose.position.x = x_sol(k, 0);
      ps.pose.position.y = x_sol(k, 1);
      const double th = x_sol(k, 2);
      ps.pose.orientation.z = std::sin(th / 2.0);
      ps.pose.orientation.w = std::cos(th / 2.0);
      traj.poses.push_back(ps);
    }
    traj_pub_->publish(traj);
  }

  /* Publish the predicted dynamic-obstacle trajectories for RViz (no-op unless
   * the predictive fill ran and the debug topic has a subscriber). */
  publishPredictedObstacleMarkers(cmd.header.stamp);

  cmd.twist = twist;
  return cmd;
}

void ProxMpcController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  /* Cache only. This runs on the node's executor thread; the request is applied
   * on the control thread (configure(), or the top of the next control cycle),
   * so the model's bounds are never mutated under a running solve. */
  speed_limit_.store(speed_limit, std::memory_order_relaxed);
  speed_limit_is_percentage_.store(percentage, std::memory_order_relaxed);
  speed_limit_pending_.store(true, std::memory_order_release);
}

void ProxMpcController::applySpeedLimit(double speed_limit, bool percentage)
{
  double v_lim;
  if (speed_limit <= 0.0) {
    v_lim = v_max_;  // NO_SPEED_LIMIT: restore the model's bound
  } else if (percentage) {
    v_lim = (speed_limit / 100.0) * v_max_;
  } else {
    v_lim = speed_limit;
  }
  v_lim = std::clamp(v_lim, 0.0, v_max_);
  model_->updateIneq("u", 0, -v_lim, v_lim);
  max_linear_vel_ = v_lim;
}

bool ProxMpcController::cancel()
{
  cancelling_ = true;
  if (std::abs(last_cmd_v_) < kCancelStopEpsilon && std::abs(last_cmd_w_) < kCancelStopEpsilon) {
    cancelling_ = false;
    return true;
  }
  return false;
}

void ProxMpcController::reset()
{
  failure_count_ = 0;
  veto_count_ = 0;
  steering_state_ = 0.0;
  last_cmd_v_ = 0.0;
  last_cmd_w_ = 0.0;
  cancelling_ = false;
  plan_index_ = 0;
  have_last_cycle_ = false;
}

void ProxMpcController::obstacleCallback(prox_mpc_msgs::msg::ObstacleArray::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  latest_obstacles_ = msg;
}

void ProxMpcController::fillObstacles(
  const MatrixXd & reference, MatrixXd & obs, const rclcpp::Time & now)
{
  predicted_obstacles_.clear();
  const std::size_t k_obs = static_cast<std::size_t>(max_obstacles_);

  /* Default every slot to the far sentinel so unfilled ones stay non-binding. */
  for (Eigen::Index r = 0; r < obs.rows(); ++r) {
    obs(r, 0) = prox_mpc::MPC::kObsFarSentinel;
    obs(r, 1) = prox_mpc::MPC::kObsFarSentinel;
    obs(r, 2) = 0.0;
  }

  /* Snapshot the latest tracked obstacles under the mutex. */
  prox_mpc_msgs::msg::ObstacleArray::ConstSharedPtr msg;
  if (predict_obstacles_) {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    msg = latest_obstacles_;
  }

  /* Staleness fallback: with predictions off, no message, or a stale one, fill
   * every slot from the costmap exactly as the costmap-only path does. */
  double age = 0.0;
  bool predictive = false;
  if (predict_obstacles_ && msg) {
    const rclcpp::Time stamp(msg->header.stamp, now.get_clock_type());
    age = (now - stamp).seconds();
    predictive = std::isfinite(age) && age >= 0.0 && age <= obstacle_timeout_;
  }
  if (!predictive) {
    fillStaticObstacles(reference, obs, 0, {});
    return;
  }

  /* Express obstacles in the costmap global frame the core solves in. A TF gap is
   * not a control fault: degrade to costmap-only for this cycle. */
  const std::string global_frame = costmap_ros_->getGlobalFrameID();
  const std::string obs_frame = msg->header.frame_id;
  double tx = 0.0;
  double ty = 0.0;
  double tyaw = 0.0;
  if (!obs_frame.empty() && obs_frame != global_frame) {
    try {
      const geometry_msgs::msg::TransformStamped tfs =
        tf_->lookupTransform(global_frame, obs_frame, tf2::TimePointZero);
      tx = tfs.transform.translation.x;
      ty = tfs.transform.translation.y;
      tyaw = quat_yaw(tfs.transform.rotation);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "ProxMpcController: tracked-obstacle TF %s <- %s unavailable (%s); "
        "costmap-only this cycle.", global_frame.c_str(), obs_frame.c_str(), ex.what());
      fillStaticObstacles(reference, obs, 0, {});
      return;
    }
  }
  const double ct = std::cos(tyaw);
  const double st = std::sin(tyaw);

  /* Dynamic tracks (speed above the threshold) in the global frame, with a
   * priority = closest approach to the reference trajectory over the horizon. */
  struct DynObs
  {
    double x;
    double y;
    double vx;
    double vy;
    double radius;
    std::uint32_t id;
    double priority;
    /* Transformed curved-prediction samples (empty -> straight-ray fill). */
    std::vector<std::array<double, 2>> samples;
    double sample_dt;
  };
  std::vector<DynObs> dyn;
  dyn.reserve(msg->obstacles.size());
  for (const auto & o : msg->obstacles) {
    // Fail closed: the tracked-obstacle topic is untrusted input, so drop any
    // ill-formed track (a non-finite field or a negative radius) before it is
    // written into the QP rows, rather than relying only on the downstream
    // solver guard.
    if (!std::isfinite(o.position.x) || !std::isfinite(o.position.y) ||
      !std::isfinite(o.velocity.x) || !std::isfinite(o.velocity.y) ||
      !std::isfinite(o.radius) || o.radius < 0.0)
    {
      continue;
    }
    // Reject extended structure (walls) reported as a moving track: a large
    // radius would inflate d_safe and erase real costmap cells. Leave it to the
    // costmap fill. (0 = no limit.)
    if (max_dynamic_obstacle_radius_ > 0.0 && o.radius > max_dynamic_obstacle_radius_) {continue;}
    const double ox = tx + ct * o.position.x - st * o.position.y;
    const double oy = ty + st * o.position.x + ct * o.position.y;
    const double ovx = ct * o.velocity.x - st * o.velocity.y;
    const double ovy = st * o.velocity.x + ct * o.velocity.y;
    if (std::hypot(ovx, ovy) < dynamic_speed_threshold_) {continue;}  // costmap covers static
    // Tracker-sampled curved prediction: validate fail-closed (prediction_dt
    // finite and > 0, every sample finite - else treat as empty, keeping the
    // straight-ray fill), then transform with the same rigid transform as
    // position/velocity above.
    std::vector<std::array<double, 2>> samples;
    double sample_dt = 0.0;
    if (std::isfinite(o.prediction_dt) && o.prediction_dt > 0.0 &&
      !o.predicted_positions.empty())
    {
      bool samples_finite = true;
      for (const auto & p : o.predicted_positions) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
          samples_finite = false;
          break;
        }
      }
      if (samples_finite) {
        samples.reserve(o.predicted_positions.size());
        for (const auto & p : o.predicted_positions) {
          samples.push_back({tx + ct * p.x - st * p.y, ty + st * p.x + ct * p.y});
        }
        sample_dt = o.prediction_dt;
      }
    }
    double best = std::numeric_limits<double>::max();
    for (std::size_t node = 0; node < np_; ++node) {
      const double dt_k = static_cast<double>(node + 1) * dt_ + age;
      double px;
      double py;
      if (samples.empty()) {
        px = ox + ovx * dt_k;
        py = oy + ovy * dt_k;
      } else {
        const std::array<double, 2> p = sample_polyline(ox, oy, samples, sample_dt, dt_k);
        px = p[0];
        py = p[1];
      }
      const double rx = reference(static_cast<Eigen::Index>(node + 1), 0);
      const double ry = reference(static_cast<Eigen::Index>(node + 1), 1);
      best = std::min(best, std::hypot(px - rx, py - ry));
    }
    dyn.push_back({ox, oy, ovx, ovy, o.radius, o.id, best, std::move(samples), sample_dt});
  }
  std::sort(
    dyn.begin(), dyn.end(), [](const DynObs & a, const DynObs & b) {
      return a.priority < b.priority;
      });

  std::size_t n_dyn = std::min(dyn.size(), k_obs);
  if (max_dynamic_obstacles_ >= 0) {
    n_dyn = std::min(n_dyn, static_cast<std::size_t>(max_dynamic_obstacles_));
  }

  /* Propagate each selected obstacle over the horizon, binding slot j to obstacle
   * j for every node so the half-planes track one object across the horizon
   * (tracker-sampled curved prediction when available, straight ray otherwise).
   * The clearance grows with prediction time as the prediction ages. */
  std::vector<std::vector<std::array<double, 3>>> exclusions(np_);
  predicted_obstacles_.resize(n_dyn);
  for (std::size_t j = 0; j < n_dyn; ++j) {
    predicted_obstacles_[j].id = dyn[j].id;
    predicted_obstacles_[j].radius = dyn[j].radius;
    predicted_obstacles_[j].positions.resize(np_);
  }
  for (std::size_t node = 0; node < np_; ++node) {
    const double dt_k = static_cast<double>(node + 1) * dt_ + age;
    for (std::size_t j = 0; j < n_dyn; ++j) {
      const DynObs & d = dyn[j];
      // Curved prediction when the tracker published samples; otherwise the
      // straight constant-velocity ray, unchanged.
      double px;
      double py;
      if (d.samples.empty()) {
        px = d.x + d.vx * dt_k;
        py = d.y + d.vy * dt_k;
      } else {
        const std::array<double, 2> p = sample_polyline(d.x, d.y, d.samples, d.sample_dt, dt_k);
        px = p[0];
        py = p[1];
      }
      const double d_safe = robot_radius_ + d.radius + safety_margin_ +
        prediction_uncertainty_growth_ * dt_k;
      const Eigen::Index row = static_cast<Eigen::Index>(node * k_obs + j);
      obs(row, 0) = px;
      obs(row, 1) = py;
      obs(row, 2) = d_safe;
      /* Exclude the obstacle's CURRENT footprint from the static scan, not its
       * predicted one: the local costmap is a now-snapshot, so the moving object's
       * occupied cells sit at its current position. Excluding the current footprint
       * at every node keeps the static fill from re-adding the same object the
       * predictive half-plane already covers (the predicted cells are not in the
       * snapshot, so excluding them would not de-duplicate anything). */
      exclusions[node].push_back({d.x, d.y, d.radius + obstacle_cluster_radius_});
      predicted_obstacles_[j].positions[node] = {px, py};
    }
  }

  /* Hybrid: fill the remaining slots from the costmap (static clutter), excluding
   * cells inside a dynamic footprint to avoid double-counting the moving object. */
  if (n_dyn < k_obs) {
    fillStaticObstacles(reference, obs, n_dyn, exclusions);
  }
}

void ProxMpcController::reduceCostmap(const MatrixXd & reference, MatrixXd & obs)
{
  /* Default every slot to the far sentinel so unfilled ones stay non-binding. */
  for (Eigen::Index r = 0; r < obs.rows(); ++r) {
    obs(r, 0) = prox_mpc::MPC::kObsFarSentinel;
    obs(r, 1) = prox_mpc::MPC::kObsFarSentinel;
    obs(r, 2) = 0.0;
  }
  fillStaticObstacles(reference, obs, 0, {});
}

void ProxMpcController::fillStaticObstacles(
  const MatrixXd & reference, MatrixXd & obs, std::size_t slot_begin,
  const std::vector<std::vector<std::array<double, 3>>> & exclusions)
{
  const std::size_t k_obs = static_cast<std::size_t>(max_obstacles_);
  if (slot_begin >= k_obs) {return;}
  const std::size_t budget = k_obs - slot_begin;
  const double d_safe = robot_radius_ + safety_margin_;

  auto * costmap = costmap_ros_->getCostmap();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
  const double res = costmap->getResolution();
  if (res <= 0.0) {return;}

  const double search_radius = d_safe + obstacle_cluster_radius_;
  const int win_full = static_cast<int>(std::ceil(search_radius / res));
  int win = win_full;
  if (win > max_obstacle_scan_cells_) {
    win = max_obstacle_scan_cells_;
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "ProxMpcController: obstacle scan window truncated to %d cells (search radius "
      "needs %d); obstacles farther than %.2f m within d_safe are not passed to the "
      "solver. Raise max_obstacle_scan_cells; the footprint veto remains the backstop.",
      max_obstacle_scan_cells_, win_full, static_cast<double>(max_obstacle_scan_cells_) * res);
  }
  const int size_x = static_cast<int>(costmap->getSizeInCellsX());
  const int size_y = static_cast<int>(costmap->getSizeInCellsY());
  const double sr2 = search_radius * search_radius;
  const double cr2 = obstacle_cluster_radius_ * obstacle_cluster_radius_;

  for (std::size_t node = 0; node < np_; ++node) {
    const double pcx = reference(static_cast<Eigen::Index>(node + 1), 0);
    const double pcy = reference(static_cast<Eigen::Index>(node + 1), 1);
    unsigned int mx0 = 0;
    unsigned int my0 = 0;
    if (!costmap->worldToMap(pcx, pcy, mx0, my0)) {continue;}

    /* Occupied cells within the search window, sorted by distance to the node. */
    std::vector<std::array<double, 3>> candidates;
    for (int dy = -win; dy <= win; ++dy) {
      const int my = static_cast<int>(my0) + dy;
      if (my < 0 || my >= size_y) {continue;}
      for (int dx = -win; dx <= win; ++dx) {
        const int mx = static_cast<int>(mx0) + dx;
        if (mx < 0 || mx >= size_x) {continue;}
        const unsigned char cost =
          costmap->getCost(static_cast<unsigned int>(mx), static_cast<unsigned int>(my));
        if (cost < costmap_cost_threshold_ || cost == nav2_costmap_2d::NO_INFORMATION) {continue;}
        double wx = 0.0;
        double wy = 0.0;
        costmap->mapToWorld(static_cast<unsigned int>(mx), static_cast<unsigned int>(my), wx, wy);
        const double dd = (wx - pcx) * (wx - pcx) + (wy - pcy) * (wy - pcy);
        if (dd > sr2) {continue;}
        /* Drop cells inside a dynamic footprint (already covered by its half-plane). */
        bool excluded = false;
        if (node < exclusions.size()) {
          for (const auto & e : exclusions[node]) {
            const double ex = wx - e[0];
            const double ey = wy - e[1];
            if (ex * ex + ey * ey < e[2] * e[2]) {
              excluded = true;
              break;
            }
          }
        }
        if (!excluded) {candidates.push_back({dd, wx, wy});}
      }
    }
    std::sort(
      candidates.begin(), candidates.end(),
      [](const std::array<double, 3> & a, const std::array<double, 3> & b) {return a[0] < b[0];});

    /* Cluster: keep the nearest representatives at least cluster-radius apart. */
    std::vector<std::array<double, 2>> picked;
    for (const auto & cd : candidates) {
      if (picked.size() >= budget) {break;}
      bool near = false;
      for (const auto & pk : picked) {
        const double pd = (cd[1] - pk[0]) * (cd[1] - pk[0]) + (cd[2] - pk[1]) * (cd[2] - pk[1]);
        if (pd < cr2) {
          near = true;
          break;
        }
      }
      if (!near) {picked.push_back({cd[1], cd[2]});}
    }
    for (std::size_t slot = 0; slot < picked.size(); ++slot) {
      const Eigen::Index row = static_cast<Eigen::Index>(node * k_obs + slot_begin + slot);
      obs(row, 0) = picked[slot][0];
      obs(row, 1) = picked[slot][1];
      obs(row, 2) = d_safe;
    }
  }
}

void ProxMpcController::publishPredictedObstacleMarkers(const rclcpp::Time & now)
{
  if (!marker_pub_ || marker_pub_->get_subscription_count() == 0) {return;}

  visualization_msgs::msg::MarkerArray arr;
  visualization_msgs::msg::Marker clear;
  clear.ns = "predicted_obstacles";
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(clear);

  const std::string global_frame = costmap_ros_->getGlobalFrameID();
  int id = 0;
  for (const auto & po : predicted_obstacles_) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = global_frame;
    m.header.stamp = now;
    m.ns = "predicted_obstacles";
    m.id = id++;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.05;
    m.color.r = 1.0f;
    m.color.g = 0.4f;
    m.color.a = 1.0f;
    m.pose.orientation.w = 1.0;
    for (const auto & p : po.positions) {
      geometry_msgs::msg::Point pt;
      pt.x = p[0];
      pt.y = p[1];
      m.points.push_back(pt);
    }
    arr.markers.push_back(m);
  }
  marker_pub_->publish(arr);
}

void ProxMpcController::publishDiagnostics(
  const rclcpp::Time & stamp, double solve_ms, bool converged,
  std::uint16_t num_active_obstacles)
{
  const auto t_now = std::chrono::steady_clock::now();
  double period_ms = std::numeric_limits<double>::quiet_NaN();
  if (have_last_cycle_) {
    period_ms = std::chrono::duration<double, std::milli>(t_now - last_cycle_wall_).count();
  }
  last_cycle_wall_ = t_now;
  have_last_cycle_ = true;

  /* Only-when-subscribed: zero cost in the production default. */
  if (!diag_pub_ || diag_pub_->get_subscription_count() == 0) {return;}

  prox_mpc_msgs::msg::SolverDiagnostics d;
  d.header.stamp = stamp;
  d.header.frame_id = costmap_ros_ ? costmap_ros_->getBaseFrameID() : std::string("base_link");
  d.solve_time_ms = solve_ms;
  d.qp_solve_time_ms = mpc_->qp_info.run_time / 1000.0;  // proxsuite reports microseconds
  d.status = solverStatusToMsg(mpc_->qp_info.status);
  d.converged = converged;
  d.sqp_iters = static_cast<std::uint32_t>(mpc_->sqp_iter);
  d.qp_iters_ext = static_cast<std::uint32_t>(mpc_->qp_iter_ext);
  d.primal_residual = mpc_->qp_info.pri_res;
  d.dual_residual = mpc_->qp_info.dua_res;
  d.objective = mpc_->qp_info.objValue;
  d.max_obstacle_slack = mpc_->getMaxObstacleSlack();
  d.control_period_ms = period_ms;
  // A missed deadline = the solve did not fit in the control budget (1000*dt ms).
  // The solve is the controller's compute cost and the budget is the period the
  // server schedules it at, so this is the well-defined real-time signal. The
  // measured control_period_ms is published as a separate field for analysis but
  // is deliberately not folded into this flag: the nominal period equals the
  // budget by construction, so any period threshold would need an arbitrary slack.
  d.deadline_missed = solve_ms > 1000.0 * dt_;
  d.num_active_obstacles = num_active_obstacles;
  diag_pub_->publish(d);
}

}  // namespace prox_mpc_controller

PLUGINLIB_EXPORT_CLASS(prox_mpc_controller::ProxMpcController, nav2_core::Controller)
