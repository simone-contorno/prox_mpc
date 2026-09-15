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
#include <numeric>
#include <string>
#include <tuple>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_core/controller_exceptions.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
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
/// Cap on the brake ramp's integration period, as a multiple of the configured
/// step: the measured inter-cycle period is used so a server running slower than
/// the configured step still brakes at the model's own rate, but a stale or
/// hiccuped measurement must not turn one ramp step into an abrupt stop.
constexpr double kMaxBrakePeriodFactor = 2.0;
/// Upper bound on the costmap scan half-window [cells] to keep the per-cycle cost
/// bounded on constrained hardware.
constexpr int kMaxScanHalfWidth = 50;
/// Reverse speed applied when reversing is enabled but no reverse bound is named
/// [m/s]. Whether the reverse direction is sensed at all is a property of the
/// platform, which this plugin cannot know: a forward-facing lidar leaves the
/// manoeuvre blind, while a 360-degree scanner covers it. The conservative
/// default therefore assumes the worse case and keeps an unverified manoeuvre
/// slow. Naming model_params.v_min overrides it, which is how a platform with
/// rear sensing states its real reverse envelope.
constexpr double kDefaultReverseSpeed = 0.15;
/// Track speed [m/s] below which no forward shadow is cast: the heading of a
/// near-stationary track is numerically meaningless, and biasing a keep-out along
/// it would move the disc in an arbitrary direction.
constexpr double kMinShadowSpeed = 1e-3;
/// Rate [1/s] at which the obstacle-aware yield may release the cruise back
/// toward full once a breach eases: a full release takes a second. The yield
/// itself tightens at once.
constexpr double kYieldRecoveryPerS = 1.0;
/// Distance [m] within which last cycle's planned trajectory must still meet the
/// robot for the yield to trust it: its next node has to lie this close to the
/// reference point's current position.
constexpr double kYieldTrajMatchRadius = 0.5;
/// Minimum strictly-positive cost weight, keeping the QP Hessian positive definite
/// (a negative weight would make the sub-problem non-convex).
constexpr double kMinCostWeight = 1e-9;
/// Discrete-time CBF rate floor; the rate must lie in (0, 1] (1.0 = pointwise term).
constexpr double kMinCbfGamma = 1e-3;
/// Largest costmap cost still treated as an obstacle threshold (255 = NO_INFORMATION,
/// handled separately; a higher threshold would silently disable avoidance).
constexpr int kMaxCostThreshold = 254;
/// Marks a scanned obstacle that won no slot in the per-cycle capacity.
constexpr std::size_t kNoObstacleSlot = std::numeric_limits<std::size_t>::max();
/// Largest |offset * curvature| the steering inverse is evaluated at. The
/// closed form divides by sqrt(1 - (offset * curvature)^2), which is the
/// geometry running out of steering angle; clamping keeps the reference finite.
constexpr double kMaxOffsetCurvature = 0.99;
/// Below this squared norm a quaternion carries no orientation (the goal
/// checker's unmeasured tolerance fields come back all-zero).
constexpr double kMinQuatNorm2 = 1e-12;

/// Planar yaw from a quaternion.
double quat_yaw(const geometry_msgs::msg::Quaternion & q)
{
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny, cosy);
}

/// One step toward zero under the value's own rate bounds, which are the rate of
/// change allowed downward (`rate_low`, negative) and upward (`rate_upp`), so an
/// asymmetric model decelerates at its own rate in each direction. A non-finite
/// previous value (NaN or +/-inf) yields zero: an infinite measured velocity
/// would otherwise survive the ramp and be published as the command.
double brake_toward(double prev, double rate_low, double rate_upp, double dt)
{
  if (!std::isfinite(prev)) {return 0.0;}
  if (prev > 0.0) {return std::max(0.0, prev - std::abs(rate_low) * dt);}
  if (prev < 0.0) {return std::min(0.0, prev + std::abs(rate_upp) * dt);}
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

/// Whether every component of a Twist is finite. All six are published, and a
/// model is free to fill any of them, so all six are checked: testing only the
/// two a planar base consumes would let a non-finite linear.y or angular.x reach
/// the wire unexamined.
bool twist_is_finite(const geometry_msgs::msg::Twist & twist)
{
  return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
         std::isfinite(twist.linear.z) && std::isfinite(twist.angular.x) &&
         std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
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
  auto clamp_low = [this](const char * param_name, double & v, double lo, double def) {
      if (!std::isfinite(v)) {
        RCLCPP_WARN(logger_, "%s is not finite; using default %.3f.", param_name, def);
        v = def;
        return;
      }
      if (v < lo) {
        RCLCPP_WARN(logger_, "%s %.3f below %.3f; clamping.", param_name, v, lo);
        v = lo;
      }
    };
  auto clamp_range = [this](const char * param_name, double & v, double lo, double hi, double def) {
      if (!std::isfinite(v)) {
        RCLCPP_WARN(logger_, "%s is not finite; using default %.3f.", param_name, def);
        v = def;
        return;
      }
      if (v < lo || v > hi) {
        const double c = std::clamp(v, lo, hi);
        RCLCPP_WARN(
          logger_, "%s %.3f outside [%.3f, %.3f]; clamping to %.3f.", param_name, v, lo, hi, c);
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
  declare("model_plugin", model_plugin, std::string("prox_mpc_core/Unicycle"));
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
  if (np_param < 1 || nc_param < 1 || nc_param > np_param || !std::isfinite(dt_) || dt_ <= 0.0) {
    throw nav2_core::ControllerException(
            "ProxMpcController: np >= 1, nc >= 1, nc <= np, dt > 0 required (got np=" +
            std::to_string(np_param) + ", nc=" + std::to_string(nc_param) + ", dt=" +
            std::to_string(dt_) + ")");
  }
  np_ = static_cast<std::size_t>(np_param);
  nc_ = static_cast<std::size_t>(nc_param);

  declare("desired_linear_vel", desired_linear_vel_, 1.0);
  declare("curvature_gain", curvature_gain_, 0.0);
  /* Reverse travel is opt-in: the reference speed becomes signed, which the
   * speed limit and the deceleration ramp both have to carry, so a stack that
   * does not plan reversing sections keeps the forward-only control bound. It is
   * also travel the plugin cannot confirm is sensed: both guards follow the
   * predicted trajectory, so they do cover a reversing one, but they see only
   * what the costmap holds, and whether the platform sweeps behind itself is a
   * property of its sensor rather than of this plugin. That is why an unguarded
   * reverse is speed-capped where the model states no reverse envelope. */
  declare("allow_reversing", allow_reversing_, false);
  /* Whether the plan's pose orientations may put the reference into reverse.
   * Only a planner that sets them means anything by them, and a plan cannot say
   * whether its planner did, so this is declared rather than inferred. */
  declare("reverse_from_plan_orientation", reverse_from_plan_orientation_, false);

  /* Travel direction is a discrete mode of a switched system. Re-deciding it
   * every control cycle from the plan geometry alone, with no cost and no dwell,
   * is what makes a direction change free enough to alternate; the two gates
   * below are the standard remedy, and between them they also reproduce what a
   * real drivetrain enforces, which is that a shift is accepted only at rest.
   * A change therefore reads: hold the arc length, stop, dwell, then reverse. */
  declare("direction_switch_standstill_speed_mps", direction_switch_standstill_speed_mps_, 0.05);
  declare("direction_switch_dwell_s", direction_switch_dwell_s_, 0.5);
  /* A standstill threshold at or below zero is never met, which would freeze the
   * reference at the first cusp; a negative dwell or band is meaningless. */
  direction_switch_standstill_speed_mps_ = std::max(direction_switch_standstill_speed_mps_, 1e-3);
  direction_switch_dwell_s_ = std::max(direction_switch_dwell_s_, 0.0);
  /* Seed the dwell spent. Nothing has been switched away from before the first
   * task, so its first direction is free; the platform is at rest there anyway,
   * which is the other half of what a change has to wait for. */
  dir_hold_s_ = direction_switch_dwell_s_;
  declare("goal_settle_hysteresis_m", goal_settle_hysteresis_m_, 0.10);
  goal_settle_hysteresis_m_ = std::max(goal_settle_hysteresis_m_, 0.0);

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
  declare("w_weight", w_weight, 1000.0);
  clamp_low("q_pos", q_pos, kMinCostWeight, 10.0);
  clamp_low("q_theta", q_theta, kMinCostWeight, 1.0);
  clamp_low("s_factor", s_factor, kMinCostWeight, 2.0);
  clamp_low("r_weight", r_weight, kMinCostWeight, 0.1);
  clamp_low("w_weight", w_weight, kMinCostWeight, 1000.0);

  /* Iteration caps are structural, like the horizon sizing: they reach the core
   * as size_t, so a negative value wraps to an astronomical bound (an effectively
   * unbounded SQP loop), and ProxQP rejects a zero cap outright. Both are fatal -
   * a one-iteration fallback would be a silent, never-converging bringup. */
  int max_int_iter_qp = 0;
  declare("max_int_iter_qp", max_int_iter_qp, 1500);
  int max_ext_iter_qp = 0;
  declare("max_ext_iter_qp", max_ext_iter_qp, 10000);
  int max_iter_sqp = 0;
  declare("max_iter_sqp", max_iter_sqp, 1);
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
  clamp_low("max_solve_time", max_solve_time, 0.0, 0.0);
  bool qp_type = false;
  declare("qp_type", qp_type, false);
  bool guess = true;
  declare("guess", guess, true);
  /* Cross-cycle QP warm start. The solver workspace is built once and updated in
   * place, so proxsuite keeps its factorization and previous primal/dual iterate
   * instead of being re-initialized every cycle. Rebuilding it per cycle was the
   * wrong use of the API: it discarded the iterate the 'guess' parameter implies
   * is being reused. */
  bool warm_start = true;
  declare("warm_start", warm_start, true);
  declare("max_solver_failures", max_solver_failures_, 3);
  if (max_solver_failures_ < 0) {
    RCLCPP_WARN(logger_, "max_solver_failures %d < 0; clamping to 0.", max_solver_failures_);
    max_solver_failures_ = 0;
  }
  /* Step [s] the deceleration ramp advances by on a braking cycle. 0 (default)
   * measures the inter-cycle period, so a server running slower than dt still
   * brakes at the model's declared rate, clamped below at dt and above at
   * kMaxBrakePeriodFactor * dt so a stale measurement cannot collapse the ramp
   * into a single step. A positive value overrides the measurement and is used
   * as-is, for a deployment that wants the ramp independent of scheduling
   * jitter. */
  declare("brake_period_s", brake_period_s_, 0.0);
  clamp_low("brake_period_s", brake_period_s_, 0.0, 0.0);
  /* Obstacle-slot capacity K per predicted node: the knob for how many obstacles
   * reach the solver, one per node at the default. Every extra slot is paid on
   * every cycle of every scenario, and the costmap scan cost is unchanged, so
   * the whole increase lands in the QP: at the shipped unicycle sizing (n=3,
   * m=2, Np=Nc=20), K=2 is 16% more decision variables and 33% more inequality
   * rows than K=1, and K=4 is 49% and 100% more. */
  declare("max_obstacles", max_obstacles_, 1);
  if (max_obstacles_ < 0) {
    RCLCPP_WARN(logger_, "max_obstacles %d < 0; clamping to 0.", max_obstacles_);
    max_obstacles_ = 0;
  }
  declare("safety_margin", safety_margin_, 0.1);
  declare("robot_radius", robot_radius_, 0.5);
  clamp_low("safety_margin", safety_margin_, 0.0, 0.1);
  clamp_low("robot_radius", robot_radius_, 0.0, 0.5);
  /* Cross-check the avoidance disc against the footprint the costmap actually
   * carries. d_safe = robot_radius + safety_margin sizes every keep-out
   * half-plane, so a robot_radius below the footprint's circumscribed radius
   * leaves part of the robot outside the keep-out with only the outline-only
   * endpoint veto behind it. The parameter is never overwritten - an operator
   * running a deliberately tighter disc keeps it - but the mismatch is named. */
  const double circumscribed_radius =
    costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
  if (std::isfinite(circumscribed_radius) && robot_radius_ < circumscribed_radius) {
    /* The advice is rounded up to the millimetre it is printed at. Reporting the
     * circumscribed radius itself at %.3f names a value that can be smaller than
     * the true one, and an operator who sets exactly what the message asks for
     * would then trip the same warning again. */
    const double advised_radius = std::ceil(circumscribed_radius * 1000.0) / 1000.0;
    RCLCPP_WARN(
      logger_,
      "robot_radius %.3f m is below the costmap footprint's circumscribed radius %.4f m, so "
      "the obstacle keep-out (d_safe = robot_radius + safety_margin = %.3f m) is undersized "
      "for this footprint and the endpoint footprint veto is the only remaining guard. Raise "
      "robot_radius to at least %.3f m, or shrink the footprint.",
      robot_radius_, circumscribed_radius, robot_radius_ + safety_margin_, advised_radius);
  }
  declare("cbf_gamma", cbf_gamma_, 1.0);
  clamp_range("cbf_gamma", cbf_gamma_, kMinCbfGamma, 1.0, 1.0);
  declare("costmap_cost_threshold", costmap_cost_threshold_, 200);
  if (costmap_cost_threshold_ < 0 || costmap_cost_threshold_ > kMaxCostThreshold) {
    const int c = std::clamp(costmap_cost_threshold_, 0, kMaxCostThreshold);
    RCLCPP_WARN(
      logger_, "costmap_cost_threshold %d outside [0, %d]; clamping to %d.",
      costmap_cost_threshold_, kMaxCostThreshold, c);
    costmap_cost_threshold_ = c;
  }
  declare("obstacle_cluster_radius", obstacle_cluster_radius_, 0.3);
  clamp_low("obstacle_cluster_radius", obstacle_cluster_radius_, 0.0, 0.3);
  declare("max_obstacle_scan_cells", max_obstacle_scan_cells_, kMaxScanHalfWidth);
  if (max_obstacle_scan_cells_ < 1) {
    RCLCPP_WARN(
      logger_, "max_obstacle_scan_cells %d below 1; clamping to 1.",
      max_obstacle_scan_cells_);
    max_obstacle_scan_cells_ = 1;
  }

  /* Predictive (dynamic) obstacle avoidance, on by default: the costmap-only
   * fill reacts to where an obstacle was seen, which is measurably the weaker
   * configuration once more than one obstacle moves. With no tracker publishing,
   * or a stale message, the fill degrades to the costmap-only path, so the
   * default costs a subscription rather than a behavior change. Setting
   * predict_obstacles off reproduces that behavior bit-for-bit. */
  declare("predict_obstacles", predict_obstacles_, true);
  declare("obstacle_topic", obstacle_topic_, std::string("tracked_obstacles"));
  declare("obstacle_timeout", obstacle_timeout_, 0.5);
  declare("dynamic_speed_threshold", dynamic_speed_threshold_, 0.1);
  declare("prediction_uncertainty_growth", prediction_uncertainty_growth_, 0.0);
  declare("prediction_forward_shadow_s", prediction_forward_shadow_s_, 0.0);
  declare("obstacle_yield_band_m", obstacle_yield_band_m_, 0.0);
  declare("obstacle_yield_caps_speed", obstacle_yield_caps_speed_, false);
  declare("max_dynamic_obstacles", max_dynamic_obstacles_, 2);
  declare("max_dynamic_obstacle_radius", max_dynamic_obstacle_radius_, 0.0);

  /* Opt-in solver telemetry (off by default); publishes only-when-subscribed. */
  declare("publish_diagnostics", publish_diagnostics_, false);

  /* Validate the predictive parameters; clamp out-of-range values (non-fatal, to
   * keep the controller available) and warn, matching the cruise-speed clamp. */
  clamp_low("obstacle_timeout", obstacle_timeout_, 0.0, 0.5);
  clamp_low("dynamic_speed_threshold", dynamic_speed_threshold_, 0.0, 0.1);
  clamp_low("prediction_uncertainty_growth", prediction_uncertainty_growth_, 0.0, 0.0);
  clamp_low("prediction_forward_shadow_s", prediction_forward_shadow_s_, 0.0, 0.0);
  clamp_low("obstacle_yield_band_m", obstacle_yield_band_m_, 0.0, 0.0);
  clamp_low("max_dynamic_obstacle_radius", max_dynamic_obstacle_radius_, 0.0, 0.0);
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
  } else if (model_v_min < 0.0) {
    /* A reverse bound is honoured on its own. Model::overrideBound keeps the side
     * whose key is absent, so capping reverse travel no longer requires naming a
     * forward cap the configuration does not mean to change. */
    model_params["v_min"] = model_v_min;
  }
  model_->configure(model_params);
  n_ = model_->getN();
  m_ = model_->getM();

  readModelMapping(*model_, model_plugin);
  readModelBounds(*model_, model_plugin);
  /* allow_reversing gates the control box, not only the reference. Left to the
   * reference alone, the solver still held a negative lower bound on the speed
   * control and could command reverse on a robot configured forward-only
   * whenever the cost made it attractive - most easily while manoeuvring around
   * an obstacle, which is where reverse is least wanted. The bound is narrowed
   * rather than the command clamped, so the plan the solver returns is one the
   * robot is actually allowed to execute. */
  if (!allow_reversing_ && v_min_ < 0.0) {
    RCLCPP_INFO(
      logger_,
      "allow_reversing is false; narrowing the model's linear control bound from [%.3f, %.3f] "
      "to [0.000, %.3f] so the solver cannot plan reverse travel.", v_min_, v_max_, v_max_);
    model_->updateIneq("u", idx_v_, 0.0, v_max_);
    v_min_ = 0.0;
  }
  /* Reversing was asked for without a reverse bound to go with it. The model's
   * own lower bound is a modelling limit, not a safety choice - for the bundled
   * models it is -v_max, so enabling reversing would otherwise silently
   * authorise reverse at full cruise speed on a platform whose rear coverage
   * this plugin has no way to check. */
  if (allow_reversing_ && model_v_min >= 0.0 && v_min_ < -kDefaultReverseSpeed) {
    RCLCPP_INFO(
      logger_,
      "allow_reversing is true but model_params.v_min was not set; capping reverse travel at "
      "%.3f m/s (was %.3f). Set model_params.v_min explicitly to the platform's real reverse "
      "envelope once its rear coverage is established.", -kDefaultReverseSpeed, v_min_);
    model_->updateIneq("u", idx_v_, -kDefaultReverseSpeed, v_max_);
    v_min_ = -kDefaultReverseSpeed;
  }
  if (allow_reversing_ && v_min_ >= 0.0) {
    RCLCPP_WARN(
      logger_,
      "allow_reversing is set but model '%s' declares no reverse travel (u[%zu] lower bound "
      "%.3f), so the reference stays forward-only.", model_plugin.c_str(), idx_v_, v_min_);
  }

  /* Cruise speed must be finite and sit within the model's speed bound. The
   * finiteness check comes first because the bound comparison below, like every
   * bare "<"/">", is false for NaN: a non-finite cruise speed would pass it,
   * poison the reference and turn every cycle into a non-finite solve, a brake
   * and eventually a recovery, with nothing said at configure(). */
  if (!std::isfinite(desired_linear_vel_)) {
    const double default_desired_linear_vel = 1.0;
    RCLCPP_WARN(
      logger_, "desired_linear_vel is not finite; using default %.3f.",
      default_desired_linear_vel);
    desired_linear_vel_ = default_desired_linear_vel;
  }
  if (desired_linear_vel_ > v_max_) {
    RCLCPP_WARN(
      logger_, "desired_linear_vel %.3f exceeds model v_max %.3f; clamping.",
      desired_linear_vel_, v_max_);
    desired_linear_vel_ = v_max_;
  }

  /* Cost weights sized to the model (matches the demo assembly), placed through
   * the model's own declared mapping rather than at state columns 0 and 1: a
   * model that carries its position elsewhere is legal whenever the in-loop
   * obstacle term is off, and indexing by position would then weight a heading
   * as a position and a position as a heading with no diagnostic. Every state
   * the mapping does not name - a steering angle, say - keeps q_theta. */
  VectorXd q_diag = VectorXd::Constant(n_, q_theta);
  q_diag(idx_x_) = q_pos;
  q_diag(idx_y_) = q_pos;
  q_diag(idx_yaw_) = q_theta;
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
  mpc_->setWarmStart(warm_start);
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

void ProxMpcController::readModelMapping(
  prox_mpc::Model & model, const std::string & model_plugin)
{
  const prox_mpc::PlanarMapping mapping = model.getPlanarMapping();
  auto reject = [&model_plugin](const std::string & why) {
      throw nav2_core::ControllerException(
              "ProxMpcController: model '" + model_plugin + "' " + why);
    };

  /* Three planar states and one speed control are the least this controller can
   * drive; below that every index below is out of range, and EIGEN_NO_DEBUG
   * turns that into a silent read rather than an abort. */
  if (n_ < 3) {
    reject("declares " + std::to_string(n_) + " states; at least 3 (x, y, yaw) are required");
  }
  if (m_ < 1) {
    reject("declares no control input");
  }
  if (mapping.idx_x >= n_ || mapping.idx_y >= n_ || mapping.idx_yaw >= n_) {
    reject("maps x, y or yaw to a state index outside its own state vector");
  }
  if (mapping.idx_x == mapping.idx_y || mapping.idx_x == mapping.idx_yaw ||
    mapping.idx_y == mapping.idx_yaw)
  {
    reject("maps two planar quantities to the same state index");
  }
  if (mapping.idx_speed >= m_) {
    reject("maps the longitudinal speed to a control index outside its own control vector");
  }
  if (!std::isfinite(mapping.ref_offset_x) || !std::isfinite(mapping.ref_offset_y)) {
    reject("declares a non-finite reference-point offset");
  }

  has_steering_ = mapping.idx_steering != prox_mpc::PlanarMapping::kNoIndex;
  has_steer_rate_ = has_steering_ &&
    mapping.idx_steer_rate != prox_mpc::PlanarMapping::kNoIndex;
  if (has_steer_rate_) {
    if (mapping.idx_steer_rate >= m_ || mapping.idx_steer_rate == mapping.idx_speed) {
      reject("maps its steering rate to an unusable control index");
    }
  }
  if (has_steering_) {
    if (mapping.idx_steering >= n_ || mapping.idx_steering == mapping.idx_x ||
      mapping.idx_steering == mapping.idx_y || mapping.idx_steering == mapping.idx_yaw)
    {
      reject("maps its steering angle to an unusable state index");
    }
    /* The steering reference is built from the wheelbase, so a model that
     * carries a steering angle has to say what its wheelbase is. The parameter
     * forwarded as model_params.L is not read back here: a model free to ignore
     * that key would otherwise be driven on a wheelbase it does not use. */
    if (!std::isfinite(mapping.wheelbase) || mapping.wheelbase <= 0.0) {
      reject(
        "carries a steering angle but declares no wheelbase; override "
        "getPlanarMapping() to declare one");
    }
    /* The steering inverse is derived for a reference point on the body x axis,
     * which is where both the front and the rear axle sit. */
    if (mapping.ref_offset_y != 0.0) {
      reject("carries a steering angle and a lateral reference-point offset, which is unsupported");
    }
  }

  idx_x_ = mapping.idx_x;
  idx_y_ = mapping.idx_y;
  idx_yaw_ = mapping.idx_yaw;
  idx_v_ = mapping.idx_speed;
  idx_steer_ = has_steering_ ? mapping.idx_steering : 0;
  idx_steer_rate_ = has_steer_rate_ ? mapping.idx_steer_rate : 0;
  ref_offset_x_ = mapping.ref_offset_x;
  ref_offset_y_ = mapping.ref_offset_y;
  wheelbase_ = has_steering_ ? mapping.wheelbase : 0.0;
}

void ProxMpcController::readModelBounds(prox_mpc::Model & model, const std::string & model_plugin)
{
  const std::size_t m = model.getM();
  v_max_ = required_bound(
    model, model_plugin, "u", idx_v_, 2, "linear",
    "the speed cap would collapse to zero and the controller would never move.");
  /* The lower bound is cached alongside it so a runtime speed limit narrows the
   * model's declared range instead of overwriting it; the same entry carries
   * both sides, so this lookup cannot fail once the one above succeeded. */
  v_min_ = required_bound(
    model, model_plugin, "u", idx_v_, 1, "linear",
    "the speed cap would collapse to zero and the controller would never move.");
  /* A declared bound is not the same as a usable one: required_bound only
   * throws when the entry is absent, so a model declaring an upper speed bound
   * of zero (or a non-finite one, which every comparison below passes) reaches
   * here, clamps the cruise speed to zero and never moves - the exact outcome
   * the missing-bound message says it prevents. */
  if (!(v_max_ > 0.0)) {
    throw nav2_core::ControllerException(
            "ProxMpcController: model '" + model_plugin + "' declares an upper 'u' bound of " +
            std::to_string(v_max_) + " for the linear control; the speed cap would collapse "
            "to zero and the controller would never move.");
  }
  max_linear_vel_ = v_max_;
  fallback_ramp_lin_ = std::abs(
    required_bound(
      model, model_plugin, "du", idx_v_, 1, "linear",
      "the solver-failure brake would never reach zero."));
  /* The second body-twist channel only exists to be ramped when the model has a
   * second control to source a rate from. A single-control model is legitimate -
   * the planar contract asks for one speed control and no more - so it keeps the
   * default here rather than being asked for a bound on a channel it never
   * declared, which would reject it while naming an "angular control" it does
   * not have. The gate is the control count and not the declared steering rate:
   * for a unicycle, channel 1 is a genuine body yaw rate and is exactly what
   * this ramp wants, while the model declares no steering at all. */
  if (m > 1) {
    fallback_ramp_ang_ = std::abs(
      required_bound(
        model, model_plugin, "du", 1, 1, "angular",
        "the solver-failure brake would never reach zero."));
  }

  /* Every declared control-rate bound, sign preserved, so the brake ramps each
   * control channel at the model's own rate in each direction. A channel with no
   * declared bound stays unbounded and is taken to zero in one step: there is no
   * rate to respect, and freezing it at its last commanded value would leave the
   * brake unable to stop that channel at all. */
  du_low_.assign(m, -std::numeric_limits<double>::infinity());
  du_upp_.assign(m, std::numeric_limits<double>::infinity());
  for (const auto & entry : model.getIneq("du")) {
    const auto idx = static_cast<std::size_t>(entry.second[0]);
    if (idx < m) {
      du_low_[idx] = entry.second[1];
      du_upp_[idx] = entry.second[2];
    }
  }
  last_cmd_u_ = VectorXd::Zero(m);

  /* Rate at which the steering belief may be decayed on a rejected cycle, for a
   * model that carries a steering state: the bound on the steering-rate control.
   * Left at zero when the model declares none, which freezes the belief rather
   * than moving it at a rate the model never stated. */
  steer_rate_low_ = 0.0;
  steer_rate_upp_ = 0.0;
  if (has_steer_rate_) {
    /* The model declares which control carries its steering-angle rate; the
     * bound on that channel is how fast the belief may be moved. */
    for (const auto & entry : model.getIneq("u")) {
      if (static_cast<std::size_t>(entry.second[0]) == idx_steer_rate_) {
        steer_rate_low_ = entry.second[1];
        steer_rate_upp_ = entry.second[2];
      }
    }
  }
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
    /* Both clears under the mutex, matching deactivate(): the subscription is
     * already gone by here, but splitting them left the two teardown paths
     * disagreeing about which member the mutex covers. */
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    latest_obstacles_.reset();
    predicted_obstacles_.clear();
  }
  costmap_ros_.reset();
  tf_.reset();
}

void ProxMpcController::activate()
{
  failure_count_ = 0;
  veto_count_ = 0;
  steering_state_ = 0.0;
  last_cmd_u_.setZero();
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
   * message arrives (the staleness timeout would also catch this). Both clears
   * are made under the mutex: the controller server deactivates its action
   * server, and so joins the thread that runs computeVelocityCommands, before
   * calling this, but that ordering is an upstream implementation detail this
   * teardown should not depend on. */
  {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    latest_obstacles_.reset();
    predicted_obstacles_.clear();
  }
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
  /* Close the previous cycle's timing measurement here, before any gate can
   * return or throw, so the deceleration ramp below and the telemetry at the end
   * read one and the same inter-cycle period and no path out of the cycle leaves
   * the reference point stale for the next one. */
  const double cycle_period_ms = markCycleStart();

  /* Advance the direction-change dwell on the measured period, falling back to
   * the nominal step on the first cycle of a task, where no period exists yet. */
  dir_hold_s_ += std::isfinite(cycle_period_ms) ? cycle_period_ms * 1e-3 : dt_;

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

  /* Current pose in the costmap global frame (the server supplies it there).
   * Read before the fail-safe ramp below, which needs it to set the model state
   * it maps the braked controls through; its finiteness is checked once the ramp
   * exists to handle the failure. */
  const double cx = pose.pose.position.x;
  const double cy = pose.pose.position.y;
  const double ctheta = quat_yaw(pose.pose.orientation);

  /* Deceleration ramp shared by the cancel, solver-failure, and veto paths. It
   * ramps the model's own controls toward zero under the model's own control-rate
   * bounds and maps the result through the model, the same seam the accepted
   * command path uses: a model whose controls are not a body twist (the bicycle's
   * second control is a steering rate, not a yaw rate) has no meaningful
   * twist-space ramp. The speed channel ramps down from the server-measured
   * velocity (RPP/MPPI style) so the brake tracks the robot's actual speed rather
   * than a stale command; the remaining channels have no measurement and ramp
   * from their last commanded value. A non-finite measured velocity yields a safe
   * zero through brake_toward. */
  auto make_brake = [&]() -> geometry_msgs::msg::TwistStamped {
      /* Step with brake_period_s_ when the operator set one, otherwise with the
       * inter-cycle period this cycle measured for its telemetry rather than the
       * configured step, so a server running slower than dt_ still brakes at the
       * model's declared rate; the measurement is clamped below at dt_ so the
       * ramp is never slower than the configured one, and above so a stale
       * reading cannot turn one step into an abrupt stop. The first cycle of a
       * task has no measurement (NaN) and falls back to dt_. The ramp saturates
       * at zero either way, so an over-large step shortens the stop rather than
       * reversing or overshooting it. */
      double period = brake_period_s_;
      if (period <= 0.0) {
        const double measured_period =
          std::isfinite(cycle_period_ms) ? 1e-3 * cycle_period_ms : dt_;
        period = std::clamp(measured_period, dt_, kMaxBrakePeriodFactor * dt_);
      }

      /* A rejected cycle used to freeze the steering belief, so the next solve
       * linearized about an angle the wheels no longer hold: every rejected cycle
       * brakes, and a downstream twist-to-steering converter straightens the
       * wheels as the commanded yaw rate falls. Decay it toward zero at the
       * model's own steering-rate bound instead, which is open-loop but correct
       * in direction and rate and cannot jump. A model that declares no steering
       * rate leaves the bound at zero, which keeps the belief frozen rather than
       * guessing a rate for it. */
      if (has_steering_) {
        steering_state_ =
          brake_toward(steering_state_, steer_rate_low_, steer_rate_upp_, period);
      }

      /* Seed the ramp from the measured twist through the model's own inverse of
       * its twist mapping, channel by channel. The model is the only thing that
       * knows what its controls mean: for a front-axle-referenced model the
       * speed channel is a front-wheel speed, and writing the base_link speed
       * into it projects the measurement by cos(delta) a second time on the way
       * out, which turns a ramp step into a step change at large steering
       * angles.
       *
       * fromTwist() states per channel what a body twist determines. A finite
       * entry is a measurement of that control and is taken; a non-finite entry
       * says the twist does not observe it - a steering rate, for instance - and
       * that channel keeps its last commanded value, the only estimate there is
       * for it. A measurement that is itself non-finite is held the same way,
       * so a broken velocity estimate decelerates the last command rather than
       * entering the ramp. No channel is named here: the controller applies what
       * the model declared rather than a rule of its own. */
      VectorXd u_brake = last_cmd_u_;
      const VectorXd u_measured = model_->fromTwist(velocity);
      const Eigen::Index determined = std::min(u_brake.size(), u_measured.size());
      for (Eigen::Index j = 0; j < determined; ++j) {
        if (std::isfinite(u_measured(j))) {u_brake(j) = u_measured(j);}
      }
      for (Eigen::Index j = 0; j < u_brake.size(); ++j) {
        u_brake(j) = brake_toward(
          u_brake(j), du_low_[static_cast<std::size_t>(j)],
          du_upp_[static_cast<std::size_t>(j)], period);
      }
      last_cmd_u_ = u_brake;

      /* The model reads its own state through toTwist, and the accepted path's
       * setX runs past the veto, so no braking cycle reaches it: set the state
       * here from the current pose, with a non-finite pose leaving it at zero. */
      VectorXd brake_state = VectorXd::Zero(n_);
      if (std::isfinite(cx) && std::isfinite(cy) && std::isfinite(ctheta)) {
        brake_state(idx_x_) = cx + ref_offset_x_ * std::cos(ctheta) -
          ref_offset_y_ * std::sin(ctheta);
        brake_state(idx_y_) = cy + ref_offset_x_ * std::sin(ctheta) +
          ref_offset_y_ * std::cos(ctheta);
        brake_state(idx_yaw_) = ctheta;
      }
      if (has_steering_) {brake_state(idx_steer_) = steering_state_;}
      model_->setX(brake_state);

      geometry_msgs::msg::Twist twist = model_->toTwist(u_brake);
      if (!twist_is_finite(twist)) {
        /* Degraded mode of last resort: a model that maps braked controls to a
         * non-finite twist would otherwise publish it on the safety path, so fall
         * back to ramping the measured twist within the channel-0 and channel-1
         * deceleration limits, which is what this path did before it went through
         * the model. */
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 2000,
          "ProxMpcController: the model mapped the braked controls to a non-finite twist; "
          "ramping the measured twist directly this cycle.");
        twist = geometry_msgs::msg::Twist();
        twist.linear.x = brake_toward(
          velocity.linear.x, fallback_ramp_lin_, fallback_ramp_lin_, period);
        twist.angular.z = brake_toward(
          velocity.angular.z, fallback_ramp_ang_, fallback_ramp_ang_, period);
      }
      last_cmd_v_ = twist.linear.x;
      last_cmd_w_ = twist.angular.z;
      /* The ramped controls are what the robot is actually given, so they become
       * the anchor of the next cycle's control-rate constraint. Leaving the last
       * accepted solve's first control there would let the next cycle plan a step
       * away from a command the robot never received. */
      mpc_->setU0(u_brake);
      cmd.twist = twist;
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
    if (!std::isfinite(gx[i]) || !std::isfinite(gy[i])) {
      return fail("non-finite plan pose");
    }
    if (i > 0) {
      s[i] = s[i - 1] + std::hypot(gx[i] - gx[i - 1], gy[i] - gy[i - 1]);
    }
  }

  /* Project the current pose onto the plan (forward-only), giving the arc-length
   * offset s0 the sampling starts from. The projection is onto the plan segments
   * and not onto its vertices: taking the nearest vertex's arc length quantizes
   * s0 to the vertex spacing, about 5 cm for the NavFn and Smac plans emitted at
   * the default 0.05 m costmap resolution and unbounded for a coarse or sparse
   * third-party plan. */
  if (plan_index_ >= plan_size) {plan_index_ = 0;}
  std::size_t best = plan_index_;
  double best_t = 0.0;
  double best_d2 = std::numeric_limits<double>::max();
  for (std::size_t i = plan_index_; i + 1 < plan_size; ++i) {
    const double ex = gx[i + 1] - gx[i];
    const double ey = gy[i + 1] - gy[i];
    const double len2 = ex * ex + ey * ey;
    /* A duplicate consecutive plan position carries no direction; project onto
     * its start vertex instead of dividing by zero. */
    const double t = (len2 > 1e-18) ?
      std::clamp(((cx - gx[i]) * ex + (cy - gy[i]) * ey) / len2, 0.0, 1.0) : 0.0;
    const double px = gx[i] + t * ex;
    const double py = gy[i] + t * ey;
    const double d2 = (px - cx) * (px - cx) + (py - cy) * (py - cy);
    if (d2 < best_d2) {
      best_d2 = d2;
      best = i;
      best_t = t;
    }
  }
  plan_index_ = best;
  /* s is the cumulative segment length, so the arc length at the projected point
   * is exact rather than interpolated. A single-pose plan, and a plan already
   * tracked to its last vertex, leave the loop unentered at best_t = 0. */
  const double s0 =
    (best + 1 < plan_size) ? s[best] + best_t * (s[best + 1] - s[best]) : s[best];

  /* Travel direction, and the arc length the reference may not pass.
   *
   * A plan's pose orientations say which way the robot faces along it, so a
   * segment whose direction opposes its own start pose's heading is driven in
   * reverse. With allow_reversing off the reference stays forward-only, which is
   * what every previous release produced. With it on, the direction under the
   * robot sets the sign of the reference speed and the reference is truncated at
   * the first direction change, so one horizon never spans a cusp - the same
   * bound upstream's regulated pure pursuit places on the same problem.
   *
   * That reading holds only for a planner that sets those orientations, and a
   * plan cannot say whether it does: NavFn and the Smac 2D planners leave every
   * pose at the identity quaternion, which is indistinguishable from a genuine
   * straight reverse plan whose poses all face the same way. With one constant
   * pose yaw the expression below collapses to the segment's component along that
   * fixed heading - the segment's x component in the plan frame - so every path
   * running the other way reads as a reverse plan however the robot faces, and
   * the robot drives the whole path backwards instead of turning around; a path
   * whose x component changes sign flips the reference cycle to cycle. Which it
   * is, is a property of the planner, so it is declared rather than guessed:
   * reverse_from_plan_orientation defaults false and the reference stays
   * forward-only, and a deployment running a cusp-emitting planner (Smac
   * Hybrid-A*, State Lattice) sets it true to get the cusp handling above. The
   * control box keeps whatever reverse travel allow_reversing granted either way,
   * so the solver may still back out of a keep-out; it is just not asked to
   * reverse along the path. */
  auto plan_direction = [&](std::size_t i) {
      const double ex = gx[i + 1] - gx[i];
      const double ey = gy[i + 1] - gy[i];
      const double yaw = quat_yaw(global_plan_.poses[i].pose.orientation) + tyaw;
      return std::cos(yaw) * ex + std::sin(yaw) * ey;
    };
  double s_end = s.back();
  if (allow_reversing_ && reverse_from_plan_orientation_ && v_min_ < 0.0 &&
    best + 1 < plan_size)
  {
    /* The direction the plan asks for under the robot, and the gates that decide
     * whether this cycle may act on it. Travel direction is held in dir_ rather
     * than re-read here, because a mode re-chosen every cycle at no cost and with
     * no dwell is free to alternate. A change is accepted only from rest and only
     * once the dwell has run; until both hold, the reference is pinned to the
     * current arc length, so v_ref falls to zero and the deceleration ramp brings
     * the platform to the standstill the change is waiting on. A cusp is then
     * driven the way a vehicle drives one: arrive, stop, shift, pull away. */
    const double d0 = plan_direction(best);
    const double dir_cmd = (d0 < 0.0) ? -1.0 : 1.0;
    if (dir_cmd != dir_) {
      if (std::abs(last_cmd_v_) <= direction_switch_standstill_speed_mps_ &&
        dir_hold_s_ >= direction_switch_dwell_s_)
      {
        dir_ = dir_cmd;
        dir_hold_s_ = 0.0;
      } else {
        s_end = s0;
      }
    }
    /* Truncate at the next cusp so one horizon never spans a direction change.
     * Skipped while a change is pending, where the reference is already held. */
    if (s_end > s0) {
      for (std::size_t i = best + 1; i + 1 < plan_size; ++i) {
        const double d = plan_direction(i);
        if (d != 0.0 && ((d < 0.0) != (dir_ < 0.0))) {
          s_end = s[i];
          break;
        }
      }
    }
  } else {
    /* Forward-only: leave no latched reverse state for the next task to inherit. */
    dir_ = 1.0;
  }
  const double dir = dir_;

  /* Cruise speed, tapered so the horizon does not overshoot the plan end (or the
   * cusp it stops at), and clamped by any active speed limit (goal-hold near the
   * end). It is a magnitude here; `dir` applies the sign once it is final. */
  const double horizon_time = static_cast<double>(np_) * dt_;
  const double remaining = s_end - s0;
  double v_ref = std::min({desired_linear_vel_, max_linear_vel_, remaining / horizon_time});
  if (v_ref < 0.0) {v_ref = 0.0;}

  /* Goal-checker tolerances, read once. The xy tolerance eases the cruise speed
   * into the goal region below; the yaw tolerance is how the checker says it
   * enforces a terminal heading at all, which is what turns the terminal-yaw
   * reference on. The pointer is read but never retained.
   *
   * Each position field IS the radial bound, not a per-axis half-extent: upstream
   * SimpleGoalChecker tests dx*dx + dy*dy <= T*T and writes the same scalar T
   * into position.x and position.y, so hypot() would report sqrt(2)*T and start
   * the taper 41% too far out. std::min is deliberately NOT RPP's
   * position.x-only read (regulated_pure_pursuit_controller.cpp:180): the two
   * are identical for every goal checker Nav2 ships, and the minimum is the
   * conservative reading for a custom anisotropic checker whose y tolerance is
   * tighter than its x. Do not "correct" this back to the x field alone.
   *
   * Unmeasured tolerance fields come back as std::numeric_limits<double>::lowest()
   * (negative) and an all-zero quaternion, so both are screened. */
  double xy_tol = 0.0;
  double yaw_tol = 0.0;
  bool have_xy_tol = false;
  bool have_yaw_tol = false;
  if (goal_checker != nullptr) {
    geometry_msgs::msg::Pose pose_tol;
    geometry_msgs::msg::Twist vel_tol;
    if (goal_checker->getTolerances(pose_tol, vel_tol)) {
      xy_tol = std::min(pose_tol.position.x, pose_tol.position.y);
      have_xy_tol = std::isfinite(xy_tol) && pose_tol.position.x > 0.0 &&
        pose_tol.position.y > 0.0;
      const auto & qt = pose_tol.orientation;
      if (qt.x * qt.x + qt.y * qt.y + qt.z * qt.z + qt.w * qt.w > kMinQuatNorm2) {
        yaw_tol = std::abs(quat_yaw(qt));
        have_yaw_tol = std::isfinite(yaw_tol) && yaw_tol > 0.0 && yaw_tol < M_PI;
      }
    }
  }

  /* Terminal heading. Past the plan end the reference pose becomes the goal
   * pose, so the horizon settles onto the orientation the goal carries instead
   * of holding whatever the last two plan vertices point at. It is engaged only
   * when the goal checker publishes a yaw tolerance it enforces, and never on a
   * reference truncated at a cusp, whose end is not the goal. */
  bool have_terminal_yaw = false;
  double terminal_yaw = 0.0;
  if (have_yaw_tol && s_end >= s.back() && plan_size >= 2) {
    const auto & qg = global_plan_.poses.back().pose.orientation;
    if (qg.x * qg.x + qg.y * qg.y + qg.z * qg.z + qg.w * qg.w > kMinQuatNorm2) {
      terminal_yaw = quat_yaw(qg) + tyaw;
      have_terminal_yaw = std::isfinite(terminal_yaw);
    }
  }

  /* Sample a plan pose at a given arc length. Past the end (or the cusp the
   * reference stops at) it holds the final pose. */
  auto sample = [&](double sk, double & x, double & y, double & th) {
      if (sk <= 0.0) {sk = 0.0;}
      if (sk > s_end) {sk = s_end;}
      if (sk >= s.back() || plan_size < 2) {
        x = gx.back();
        y = gy.back();
        /* Single-pose plan: the pose orientation is in the plan frame, so it
         * carries the same yaw offset the positions above were rotated by. */
        th = (plan_size < 2) ? quat_yaw(global_plan_.poses.back().pose.orientation) + tyaw :
          (have_terminal_yaw ? terminal_yaw :
          std::atan2(gy.back() - gy[plan_size - 2], gx.back() - gx[plan_size - 2]));
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
      /* A duplicate consecutive plan position collapses this segment to zero
       * length, which carries no heading; walk forward to the next segment with
       * positive length instead of feeding atan2(0, 0) a degenerate delta. */
      std::size_t hi = i;
      while (s[hi + 1] - s[hi] <= 1e-9 && hi + 2 < plan_size) {++hi;}
      th = std::atan2(gy[hi + 1] - gy[hi], gx[hi + 1] - gx[hi]);
    };

  /* Terminal settle.
   *
   * The cruise speed is eased to zero across the goal-checker xy tolerance, and
   * the reference is sampled at v_ref * k * dt ahead of the robot's own
   * projection onto the plan. Composing the two makes the horizon's arc reach
   * remaining^2 / xy_tol, which is shorter than `remaining` for every point
   * inside the tolerance: once the robot is in the goal region the reference
   * collapses to a stub a few millimetres ahead of the projection, and it is
   * carried along by the projection as the robot moves. That reference has no
   * fixed point. Any motion re-projects the robot and re-centres the stub, so a
   * small tracking error can be traded down as cheaply one way as the other and
   * the solver alternates between them; and because the horizon never reaches
   * the plan end, the goal's own orientation never enters the reference at all,
   * which is exactly when it is the only error left to correct.
   *
   * Inside the tolerance the reference therefore stops tracking the projection
   * and is pinned to the goal pose, which turns the last stretch from tracking a
   * receding stub into regulation about a fixed setpoint: one minimiser, and a
   * standing yaw error the solver can act on. The speed reference keeps its
   * taper, so the approach profile that reaches this point is unchanged and mode
   * entry introduces no step in the commanded speed.
   *
   * The mode is latched and released on a wider band than it is entered on, so
   * tracking noise about the tolerance cannot flip the reference mode to mode;
   * a goal that moves further away than the band still releases it. It is never
   * entered on a reference truncated at a cusp, whose end is not the goal. */
  if (have_xy_tol && s_end >= s.back()) {
    if (settling_) {
      settling_ = remaining <= xy_tol + goal_settle_hysteresis_m_;
    } else {
      settling_ = remaining <= xy_tol;
    }
  } else {
    settling_ = false;
  }

  /* Pure rotation to the goal heading.
   *
   * Once the checker's own xy condition is met, no translation is left to do and
   * only the heading is outstanding, which is the case the reference above still
   * handles badly: the goal sits a fraction of the tolerance away, and as the
   * body sweeps round, the body-frame projection of that offset changes sign, so
   * the solver reverses and re-advances to hold a position it is already close
   * enough to. Holding the reference at the robot's own reference point instead
   * leaves the heading as the only standing error, and the platform turns on the
   * spot. The distance test is the checker's, so this engages only where the
   * translation really is finished.
   *
   * Restricted to a platform with no steering channel. A car-like model cannot
   * turn on the spot at all, and pinning the position would leave it with no
   * admissible way to correct its heading; it manoeuvres out of a terminal
   * heading error instead, which is what allow_reversing is for. */
  bool rotate_in_place = false;
  if (settling_ && !has_steering_ && have_terminal_yaw && have_yaw_tol) {
    const double dgx = gx.back() - cx;
    const double dgy = gy.back() - cy;
    rotate_in_place = (dgx * dgx + dgy * dgy) <= xy_tol * xy_tol &&
      std::abs(std::remainder(terminal_yaw - ctheta, 2.0 * M_PI)) > yaw_tol;
  }

  if (rotate_in_place) {
    v_ref = 0.0;
  } else if (have_xy_tol) {
    v_ref *= std::clamp(remaining / xy_tol, 0.0, 1.0);
  }

  /* Curvature-aware cruise reduction (inert when curvature_gain_ == 0): estimate
   * the peak path curvature over the horizon from heading samples at the current
   * cruise, then taper v_ref so high-curvature segments are sampled more slowly. */
  if (curvature_gain_ > 0.0 && v_ref > 0.0 && !settling_) {
    const double ds = v_ref * dt_;
    /* Seed from the first sampled path tangent, not the robot's own yaw: seeding
     * from ctheta would make the first heading delta the robot-to-path tracking
     * error rather than path curvature. */
    double sx0 = 0.0;
    double sy0 = 0.0;
    double prev_th = 0.0;
    sample(s0, sx0, sy0, prev_th);
    double max_kappa = 0.0;
    for (std::size_t k = 1; k <= np_; ++k) {
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

  /* Obstacle-aware cruise.
   *
   * The cruise above is obstacle-blind: the reference advances along the plan at
   * full cruise whatever is crossing it, so the only way the solver can yield to
   * a mover is to lag a reference that keeps moving, which the tracking cost
   * charges as error; faced with a mover the cheaper option is nearly always to
   * race it across. Here the reference itself yields: where the robot is heading
   * would cut into a tracked mover's predicted keep-out, the cruise is eased in
   * proportion to the deepest breach, reaching zero at obstacle_yield_band_m.
   *
   * "Where the robot is heading" is checked two ways, and the deeper breach wins.
   * The plan, driven at this cycle's intended cruise, is one: it does not depend
   * on any speed an earlier yield left behind, so it cannot restore full cruise by
   * clearing the very breach that caused a reduction. The trajectory the solver
   * actually planned last cycle is the other, because the solver does not follow
   * the plan exactly - with light stage weights it strays from it by the better
   * part of a metre - and a check on the plan alone then watches a path the robot
   * is not driving. That trajectory was driven at last cycle's speed, though, so
   * it can shorten when the robot slows and seem to clear; the plan check covers
   * that, and the release below is rate-limited so a breach that seems to clear
   * for one cycle cannot snap the cruise back to full.
   *
   * The trajectory is trusted only while it still meets this robot: before the
   * first solve it is all zeros, and after a reset or a plan jump it belongs to a
   * robot somewhere else. Both checks compare the model's reference point with
   * the base_link keep-out, which is exact for a model referenced to base_link
   * and approximate for one referenced elsewhere. The predictions are the ones
   * the previous cycle's fill retained, on the same time base as that trajectory;
   * with no fresh tracker message they are empty and the cruise is untouched.
   * Skipped while settling, where the reference is already pinned to the goal. */
  if (obstacle_yield_band_m_ > 0.0 && v_ref > 0.0 && !settling_) {
    const MatrixXd traj = mpc_->getX();
    const double px_now = cx + ref_offset_x_ * std::cos(ctheta) - ref_offset_y_ * std::sin(ctheta);
    const double py_now = cy + ref_offset_x_ * std::sin(ctheta) + ref_offset_y_ * std::cos(ctheta);
    const bool traj_ok = traj.rows() >= 2 &&
      std::hypot(
      traj(1, static_cast<Eigen::Index>(idx_x_)) - px_now,
      traj(1, static_cast<Eigen::Index>(idx_y_)) - py_now) <= kYieldTrajMatchRadius;
    double deepest = 0.0;
    {
      std::lock_guard<std::mutex> lock(obstacles_mutex_);
      for (const auto & p : predicted_obstacles_) {
        const std::size_t nodes = std::min(p.positions.size(), np_);
        for (std::size_t k = 0; k < nodes; ++k) {
          const double tk = static_cast<double>(k + 1) * dt_;
          const double d_safe = robot_radius_ + p.radius + safety_margin_ +
            prediction_uncertainty_growth_ * tk;
          const double ox = p.positions[k][0];
          const double oy = p.positions[k][1];
          double rx = 0.0;
          double ry = 0.0;
          double rth = 0.0;
          sample(s0 + v_ref * tk, rx, ry, rth);
          deepest = std::max(deepest, d_safe - std::hypot(rx - ox, ry - oy));
          const Eigen::Index row = static_cast<Eigen::Index>(k + 1);
          if (traj_ok && row < traj.rows()) {
            deepest = std::max(
              deepest, d_safe - std::hypot(
                traj(row, static_cast<Eigen::Index>(idx_x_)) - ox,
                traj(row, static_cast<Eigen::Index>(idx_y_)) - oy));
          }
        }
      }
    }
    const double target = std::clamp(1.0 - deepest / obstacle_yield_band_m_, 0.0, 1.0);
    const double period_s = std::isfinite(cycle_period_ms) ? cycle_period_ms * 1e-3 : dt_;
    yield_factor_ = (target < yield_factor_) ? target :
      std::min(target, yield_factor_ + kYieldRecoveryPerS * period_s);
    v_ref *= yield_factor_;
  } else {
    yield_factor_ = 1.0;
  }

  /* Hard speed cap. The eased cruise above is only a target, and a lightly
   * weighted one: the control-tracking cost that carries it sits far below the
   * path and obstacle terms, so when a mover is closing the solver overrides it
   * and swerves at full speed rather than slowing - measured, the cruise was cut
   * to a tenth while the robot held 0.5 m/s. Capping the forward bound by the
   * same factor makes the yield a speed limit rather than a request, the same
   * way a Nav2 speed limit is applied to this box.
   *
   * The cap never falls below the speed the platform can still reach this step:
   * the solver bounds the first control to within its deceleration times dt of
   * the command last applied, so a bound under that would leave the QP no
   * feasible first control and trip the solver-failure path instead of braking.
   * The reverse bound is left as the speed limit set it, so a robot capped to a
   * crawl can still back away from a mover it cannot out-wait. It runs every
   * cycle while enabled, including when the yield is idle, so the bound returns
   * to the speed limit as soon as the cap is released. */
  if (obstacle_yield_caps_speed_ && obstacle_yield_band_m_ > 0.0) {
    const double v_lim = std::min(v_max_, max_linear_vel_);
    const double reachable =
      last_cmd_u_(static_cast<Eigen::Index>(idx_v_)) - fallback_ramp_lin_ * dt_;
    const double v_low = std::max(v_min_, -max_linear_vel_);
    /* A model may declare a positive minimum speed; the cap cannot go under it,
     * and updateIneq rejects a lower bound above the upper one. */
    const double v_cap =
      std::max(v_low, std::min(v_lim, std::max(v_lim * yield_factor_, reachable)));
    model_->updateIneq("u", idx_v_, v_low, v_cap);
  }

  /* Sample the reference path the model's own reference point is to follow. The
   * tangent psi is kept continuous (unwrapped relative to the direction the robot
   * is expected to travel in, then node to node) so the QP tracking error never
   * wraps near +/-pi. */
  MatrixXd goal_x = MatrixXd::Zero(np_ + 1, n_);
  std::vector<double> psi_cont(np_ + 1, 0.0);
  double prev_psi = ctheta + (dir < 0.0 ? M_PI : 0.0);
  for (std::size_t k = 0; k <= np_; ++k) {
    double x = 0.0;
    double y = 0.0;
    double psi = 0.0;
    /* Pinned to the plan end while settling, which sample() reports as the goal
     * position and, where the checker enforces one, the goal orientation; pinned
     * to the reference point's own position while turning on the spot, so the
     * heading is the only error the solver is left to close. */
    if (rotate_in_place) {
      x = cx + ref_offset_x_ * std::cos(ctheta) - ref_offset_y_ * std::sin(ctheta);
      y = cy + ref_offset_x_ * std::sin(ctheta) + ref_offset_y_ * std::cos(ctheta);
      psi = terminal_yaw;
    } else {
      sample(settling_ ? s_end : s0 + v_ref * static_cast<double>(k) * dt_, x, y, psi);
    }
    psi = prev_psi + std::remainder(psi - prev_psi, 2.0 * M_PI);
    prev_psi = psi;
    psi_cont[k] = psi;
    goal_x(k, idx_x_) = x;
    goal_x(k, idx_y_) = y;
  }

  /* Build the state reference.
   *
   * The reference path is the path of the point the model's state refers to,
   * which the model declares as an offset `a` along the body x axis from
   * base_link. For a model referenced to base_link itself (a = 0) the body
   * heading is the path tangent and the steering inverse is the familiar
   * delta = atan(L * kappa). For a model referenced ahead of base_link the two
   * differ, and both follow from one relation: with z = tan(delta) / L, the
   * reference point's path curvature is kappa = dir * z / sqrt(1 + (a z)^2), and
   * the body heading trails the path tangent by atan(a z). Inverting gives
   * z = kappa * dir / sqrt(1 - (a kappa)^2), which reduces to today's expression
   * at a = 0 and to asin(L * kappa) / L at a = L, the front axle. Travelling in
   * reverse flips the sign of the curvature the same steering angle produces, and
   * turns the body around, which is what `dir` carries. */
  const double a_off = ref_offset_x_;
  const double ds_ref = v_ref * dt_;
  const double yaw_flip = (dir < 0.0) ? M_PI : 0.0;
  for (std::size_t k = 0; k <= np_; ++k) {
    double kappa = 0.0;
    if (ds_ref > 1e-9) {
      kappa = (k < np_) ? (psi_cont[k + 1] - psi_cont[k]) / ds_ref :
        (psi_cont[k] - psi_cont[k - 1]) / ds_ref;
    }
    double beta = 0.0;
    if (has_steering_) {
      const double k_eff = kappa * dir;
      const double ak = std::clamp(a_off * k_eff, -kMaxOffsetCurvature, kMaxOffsetCurvature);
      const double z = k_eff / std::sqrt(1.0 - ak * ak);
      beta = std::atan(a_off * z);
      goal_x(k, idx_steer_) = std::atan(wheelbase_ * z);
    }
    goal_x(k, idx_yaw_) = psi_cont[k] - beta - yaw_flip;
  }

  MatrixXd goal_u = MatrixXd::Zero(nc_, m_);
  for (std::size_t k = 0; k < nc_; ++k) {
    goal_u(k, idx_v_) = dir * v_ref;
  }

  /* Current full state. The Nav2 pose is base_link; the model's state refers to
   * its own declared reference point, so the pose is carried out to it. The
   * steering angle the pose omits is the controller's own retained belief. */
  VectorXd state = VectorXd::Zero(n_);
  const double cos_ct = std::cos(ctheta);
  const double sin_ct = std::sin(ctheta);
  state(idx_x_) = cx + ref_offset_x_ * cos_ct - ref_offset_y_ * sin_ct;
  state(idx_y_) = cy + ref_offset_x_ * sin_ct + ref_offset_y_ * cos_ct;
  state(idx_yaw_) = ctheta;
  if (has_steering_) {state(idx_steer_) = steering_state_;}

  /* Fill the per-node obstacle triples for this cycle: predictive + hybrid when
   * enabled and fresh tracking data is available, else costmap-only. */
  const std::size_t k_obs = mpc_->getMaxObs();
  std::uint16_t num_active_obs = 0;
  if (k_obs > 0) {
    MatrixXd obs(static_cast<Eigen::Index>(np_ * k_obs), 3);
    fillObstacles(goal_x, obs, cmd.header.stamp);
    mpc_->setObs(obs);
    /* Count filled (non-sentinel) slots at the current node (rows 0..k_obs-1). */
    for (std::size_t slot = 0; slot < k_obs; ++slot) {
      if (obs(static_cast<Eigen::Index>(slot), 0) < 0.5 * prox_mpc::MPC::kObsFarSentinel) {
        ++num_active_obs;
      }
    }
  }

  /* Solve one SQP cycle, timing it with a steady clock for the real-time metric.
   * The candidate is not retained: the core keeps the last accepted x, u, w and
   * previous control until every gate below has passed and commitCandidate() runs,
   * so a rejected cycle neither warm-starts the next one from a command that was
   * never sent nor anchors its control-rate constraint on it. */
  mpc_->setGoalX(goal_x);
  mpc_->setGoalU(goal_u);
  mpc_->setPose(state);
  const auto t_solve0 = std::chrono::steady_clock::now();
  auto [x_sol, u_sol] = mpc_->solveCandidate();
  const auto t_solve1 = std::chrono::steady_clock::now();
  const double solve_ms =
    std::chrono::duration<double, std::milli>(t_solve1 - t_solve0).count();

  const bool solved =
    (mpc_->qp_info.status == proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED);

  /* The message defines `converged` as a converged solve whose applied iterate is
   * finite, so it is set only past every acceptance gate; the QP's own outcome
   * stays separately readable in the `status` field. Publication moves with it, so
   * a vetoed or braked cycle is recorded as what it was. Every path out of the
   * cycle publishes before returning or throwing, including the escalation. */
  bool converged = false;
  auto publish_cycle = [&]() {
      publishDiagnostics(
        cmd.header.stamp, solve_ms, cycle_period_ms, converged, num_active_obs);
    };

  if (!solved) {
    publish_cycle();
    return fail("solver did not converge");
  }

  /* Defense-in-depth finiteness guard. In this architecture a PROXQP_SOLVED status
   * normally implies a finite iterate (the candidate is accumulated QP increments,
   * not a divergent model rollout), but the check is kept so a non-finite state can
   * never reach the footprint veto, poison steering_state_, or be published as the
   * predicted trajectory. The core tests the whole horizon rather than the first
   * node alone, so a non-finite tail cannot be committed and warm-start the next
   * cycle. The command finiteness is still checked separately below because the
   * model's toTwist mapping can be non-finite even for a finite control. */
  const VectorXd u0 = u_sol.row(0);
  if (!mpc_->getCandidateFinite()) {
    publish_cycle();
    return fail("non-finite solver output");
  }

  /* Polygon-footprint veto over the predicted stopping distance.
   *
   * The footprint is copied before the grid lock is taken. getRobotFootprint()
   * returns Costmap2DROS's padded_footprint_ by value, and the grid mutex guards
   * the cell data rather than that member, so reading it inside the lock implied
   * a protection that does not exist. The copy is a best-effort snapshot of a
   * member written unsynchronised by the costmap's own thread - the same
   * per-cycle read RPP (collision_checker.cpp:144) and MPPI (cost_critic.hpp:68)
   * both perform, and what keeps a runtime footprint update taking effect. */
  {
    const std::vector<geometry_msgs::msg::Point> footprint = costmap_ros_->getRobotFootprint();
    auto * costmap = costmap_ros_->getCostmap();
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
    if (footprint.size() >= 3) {
      nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap);
      /* How far the veto looks: the distance the robot needs to stop, not one
       * step. Braking from the commanded speed at the model's declared rate
       * covers v^2 / (2a) - 0.25 m at the 0.5 m/s operating point, against the
       * 0.05 m a single step spans - so a veto that only ever saw node 1 found
       * an obstacle five times later than it could still stop short of. The
       * walk stops at the first node past that distance, which keeps the
       * lookahead proportional to speed and costs one footprint test per step
       * of travel rather than one per horizon node. */
      const double v_cmd = std::abs(u0(static_cast<Eigen::Index>(idx_v_)));
      const double stop_distance =
        (std::isfinite(fallback_ramp_lin_) && fallback_ramp_lin_ > 0.0) ?
        (v_cmd * v_cmd) / (2.0 * fallback_ramp_lin_) : 0.0;

      bool vetoed = false;
      double travelled = 0.0;
      for (Eigen::Index node = 1; node < x_sol.rows(); ++node) {
        /* The padded footprint is defined about base_link, while the predicted
         * pose refers to the model's own reference point, so it is carried back
         * before the check. The two coincide for a model referenced to
         * base_link. */
        const double pth = x_sol(node, idx_yaw_);
        const double pbx =
          x_sol(node, idx_x_) - (ref_offset_x_ * std::cos(pth) - ref_offset_y_ * std::sin(pth));
        const double pby =
          x_sol(node, idx_y_) - (ref_offset_x_ * std::sin(pth) + ref_offset_y_ * std::cos(pth));
        const double fcost = checker.footprintCostAtPose(pbx, pby, pth, footprint);
        /* Upstream Nav2's own collision policy, in upstream's order: unknown
         * space is not a collision when the costmap tracks it, and everything
         * else is judged at LETHAL_OBSTACLE rather than
         * INSCRIBED_INFLATED_OBSTACLE, because a real polygon check has already
         * been performed and an inflated cell is not by itself a collision (RPP
         * collision_checker.cpp:143-154, MPPI cost_critic.hpp:71-78). Nav2's own
         * doc comment describes footprintCostAtPose as returning the maximum
         * cost under the footprint, which would mask a lethal cell behind an
         * adjoining unknown one; measured against the installed nav2_costmap_2d
         * (1.3.12+), it does not - a footprint spanning both reports the lethal
         * cost, so the veto still fires (see
         * FootprintVetoStillFiresWhenLethalAdjoinsUnknown). */
        const bool unknown_is_clear =
          fcost == static_cast<double>(nav2_costmap_2d::NO_INFORMATION) &&
          costmap_ros_->getLayeredCostmap()->isTrackingUnknown();
        if (!unknown_is_clear &&
          fcost >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE))
        {
          vetoed = true;
          break;
        }
        travelled += std::hypot(
          x_sol(node, idx_x_) - x_sol(node - 1, idx_x_),
          x_sol(node, idx_y_) - x_sol(node - 1, idx_y_));
        if (travelled >= stop_distance) {break;}
      }

      if (vetoed) {
        /* Escalate a persistent veto so a robot stuck behind a static obstacle
         * inside its stopping distance triggers the behavior-tree recovery
         * instead of braking forever. The veto counter is kept separate from
         * the solver-failure budget, so a single veto still does not trip
         * recovery. */
        veto_count_++;
        /* Published before the escalation test, not after it: the cycle that
         * ends the task is still a control cycle, and the record of why it
         * ended is the last message on the topic. The solver-failure
         * escalation publishes on the same terms. */
        publish_cycle();
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
  if (!twist_is_finite(twist)) {
    publish_cycle();
    return fail("non-finite command");
  }

  /* Last gate passed: this candidate becomes the retained solver state, and its
   * first control the anchor of the next cycle's control-rate constraint. */
  converged = true;
  mpc_->commitCandidate();

  failure_count_ = 0;
  veto_count_ = 0;
  if (has_steering_) {steering_state_ = x_sol(1, idx_steer_);}
  last_cmd_u_ = u0;
  last_cmd_v_ = twist.linear.x;
  last_cmd_w_ = twist.angular.z;
  publish_cycle();

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
      ps.pose.position.x = x_sol(k, idx_x_);
      ps.pose.position.y = x_sol(k, idx_y_);
      const double th = x_sol(k, idx_yaw_);
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
  /* Intersect with the model's declared range rather than replacing it. A model
   * whose reverse limit is tighter than its forward one - or which declares no
   * reverse travel at all - keeps that asymmetry through every speed-limit
   * request, including the NO_SPEED_LIMIT restore, which would otherwise widen
   * the lower bound to -v_max_. Both bundled models declare symmetric bounds, so
   * this leaves their commanded sequence unchanged. */
  model_->updateIneq("u", idx_v_, std::max(v_min_, -v_lim), std::min(v_max_, v_lim));
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
  last_cmd_u_.setZero();
  last_cmd_v_ = 0.0;
  last_cmd_w_ = 0.0;
  cancelling_ = false;
  plan_index_ = 0;
  have_last_cycle_ = false;
  /* A new task inherits no travel direction and no settle latch. The dwell is
   * seeded spent rather than zero: nothing has been switched away from yet, so
   * the task's first direction is free rather than held for the dwell. */
  dir_ = 1.0;
  dir_hold_s_ = direction_switch_dwell_s_;
  settling_ = false;
  yield_factor_ = 1.0;
  /* The controller server stops cycling when a task ends, so the predictions the
   * last cycle drew are cleared here; RViz would otherwise keep drawing them while
   * the obstacles move on. The server also resets on deactivate, when the
   * publisher is already inactive and a publish would only log a warning. */
  {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    predicted_obstacles_.clear();
  }
  if (marker_pub_ && marker_pub_->is_activated()) {
    publishPredictedObstacleMarkers(clock_->now());
  }
}

void ProxMpcController::obstacleCallback(prox_mpc_msgs::msg::ObstacleArray::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  latest_obstacles_ = msg;
}

void ProxMpcController::keepOutShift(
  std::vector<double> & shift_x, std::vector<double> & shift_y)
{
  shift_x.assign(np_, 0.0);
  shift_y.assign(np_, 0.0);
  if (ref_offset_x_ == 0.0 && ref_offset_y_ == 0.0) {return;}

  /* The nominal trajectory is the last committed one, un-shifted, so node j's
   * constraint is linearized about the row read here as j + 2, clamped at the
   * last node - the same one-node lag the scan centres carry, and for the same
   * reason. Before the first solve it is zero, which is what that first QP
   * linearizes about, so the two stay consistent. */
  const MatrixXd nominal = mpc_->getX();
  for (std::size_t node = 0; node < np_; ++node) {
    const Eigen::Index row = std::min(
      static_cast<Eigen::Index>(node + 2), static_cast<Eigen::Index>(np_));
    const double yaw = nominal(row, static_cast<Eigen::Index>(idx_yaw_));
    const double c = std::cos(yaw);
    const double sn = std::sin(yaw);
    shift_x[node] = ref_offset_x_ * c - ref_offset_y_ * sn;
    shift_y[node] = ref_offset_x_ * sn + ref_offset_y_ * c;
  }
}

void ProxMpcController::fillObstacles(
  const MatrixXd & reference, MatrixXd & obs, const rclcpp::Time & now)
{
  /* deactivate()/cleanup() clear this under the same mutex from the executor
   * thread, so the write here (the action server's own thread) takes it too. */
  {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    predicted_obstacles_.clear();
  }
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
    fillStaticObstacles(obs, 0, {});
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
      fillStaticObstacles(obs, 0, {});
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
  std::vector<double> shift_x;
  std::vector<double> shift_y;
  keepOutShift(shift_x, shift_y);
  /* Held for the whole fill: deactivate()/cleanup() can clear predicted_obstacles_
   * from the executor thread while this (the action server's thread) is mid-fill.
   * The loop is bounded (Np * K, both small and configure-time fixed) and does no
   * I/O or blocking call, so the hold is on the order of the loop's own cost, not
   * a stall - negligible against the control cycle it runs once per. */
  {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
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
        /* Written in the solver's frame: the keep-out is meant to protect
         * base_link, and the solver constrains the model's reference point. */
        double ox = px + shift_x[node];
        double oy = py + shift_y[node];
        double d_safe_node = d_safe;
        /* Forward shadow. The keep-out is a disc about the predicted position and
         * the constraint normal points from the obstacle to the robot, so nothing
         * in the formulation distinguishes the space a mover is about to occupy
         * from the space it is vacating; passing in front and passing behind cost
         * the same, and the reference - which carries no obstacle term - keeps
         * advancing, so the solver takes the front, which is the side that does
         * not require lagging it.
         *
         * Biasing the disc along the track's own heading prices that difference in
         * without leaving the one-constraint-per-slot form the RTI core solves.
         * The centre moves forward by s and the radius grows by the same s, which
         * keeps the obstacle's own position covered - the disc still reaches
         * d_safe behind it - while extending the covered band to d_safe + 2s
         * ahead. A bare shift would open a hole over the obstacle itself as soon
         * as s passed d_safe. The disc also grows sideways by s, which is the
         * price of keeping a disc rather than a capsule; s is a fraction of a
         * second of travel, so that stays small.
         *
         * Scaled by the track's speed, so a fast mover casts a longer shadow and a
         * near-stationary one casts none. For a curving track this follows the
         * instantaneous heading, the same first-order reading the prediction
         * itself uses between samples. */
        if (prediction_forward_shadow_s_ > 0.0) {
          const double speed = std::hypot(d.vx, d.vy);
          if (speed > kMinShadowSpeed) {
            const double shadow = prediction_forward_shadow_s_ * speed;
            ox += shadow * d.vx / speed;
            oy += shadow * d.vy / speed;
            d_safe_node += shadow;
          }
        }
        obs(row, 0) = ox;
        obs(row, 1) = oy;
        obs(row, 2) = d_safe_node;
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
  }

  /* Hybrid: fill the remaining slots from the costmap (static clutter), excluding
   * cells inside a dynamic footprint to avoid double-counting the moving object. */
  if (n_dyn < k_obs) {
    fillStaticObstacles(obs, n_dyn, exclusions);
  }
}

void ProxMpcController::reduceCostmap(MatrixXd & obs)
{
  /* Default every slot to the far sentinel so unfilled ones stay non-binding. */
  for (Eigen::Index r = 0; r < obs.rows(); ++r) {
    obs(r, 0) = prox_mpc::MPC::kObsFarSentinel;
    obs(r, 1) = prox_mpc::MPC::kObsFarSentinel;
    obs(r, 2) = 0.0;
  }
  fillStaticObstacles(obs, 0, {});
}

void ProxMpcController::fillStaticObstacles(
  MatrixXd & obs, std::size_t slot_begin,
  const std::vector<std::vector<std::array<double, 3>>> & exclusions)
{
  const std::size_t k_obs = static_cast<std::size_t>(max_obstacles_);
  if (slot_begin >= k_obs) {return;}
  const std::size_t budget = k_obs - slot_begin;
  /* The keep-out disc is centred on base_link, which is the point the robot disc
   * and the costmap footprint are both defined about, whatever point the model's
   * state refers to. The scan therefore looks around base_link and each hit is
   * written in the solver's frame, so the QP half-plane and the footprint veto
   * protect the same physical point. */
  const double d_safe = robot_radius_ + safety_margin_;

  /* The grid lock is taken only around the cell reads, once per node, and is
   * released for the sort, the clustering, the slot binding and the matrix
   * writes. Those are the expensive part of the fill, and holding the lock
   * through them blocks the costmap's own update thread for no gain. The grid
   * geometry is re-read inside each locked region, so a resize between nodes
   * cannot send the scan out of bounds. */
  auto * costmap = costmap_ros_->getCostmap();
  double res = 0.0;
  {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
    res = costmap->getResolution();
  }
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
  const double sr2 = search_radius * search_radius;
  const double cr2 = obstacle_cluster_radius_ * obstacle_cluster_radius_;

  /* Scan centres come from the nominal predicted trajectory, not from the plan
   * reference: the QP linearizes node k's keep-out half-plane about its own
   * predicted position, so centring the scan there removes the cross-track error
   * the window would otherwise have to cover. The fill runs before solve(), and
   * solve() shifts the trajectory by one node before assembling, so getX() here
   * is the previous cycle's trajectory, un-shifted and one node stale: node k's
   * constraint will be linearized about the row this reads as k + 2, clamped at
   * the last node. Before the first solve the trajectory is zero, which is also
   * what that first QP linearizes about, so the two stay consistent. */
  const MatrixXd nominal = mpc_->getX();

  /* Carries a scanned cell from base_link's frame into the solver's; all zero
   * for a model referenced to base_link. */
  std::vector<double> shift_x;
  std::vector<double> shift_y;
  keepOutShift(shift_x, shift_y);

  /* One distinct physical object, tracked across nodes: (x, y) is where it was
   * seen most recently, best_d2 its closest approach to any node's scan centre,
   * first_node the earliest node it was seen at and last_node the latest. Nodes
   * are scanned in increasing order, so first_node is fixed at creation. */
  struct StaticObject
  {
    double x;
    double y;
    double best_d2;
    std::size_t first_node;
    std::size_t last_node;
  };
  /* One (node, object) sighting, carrying the position seen at that node. */
  struct StaticHit
  {
    std::size_t node;
    std::size_t object;
    double x;
    double y;
  };
  std::vector<StaticObject> objects;
  std::vector<StaticHit> hits;
  objects.reserve(np_ * budget);
  hits.reserve(np_ * budget);

  /* Reused across nodes so the per-cycle scan grows its buffers at most once. */
  std::vector<std::array<double, 3>> candidates;
  std::vector<std::array<double, 2>> picked;
  picked.reserve(budget);

  for (std::size_t node = 0; node < np_; ++node) {
    const Eigen::Index centre_row = std::min(
      static_cast<Eigen::Index>(node + 2), static_cast<Eigen::Index>(np_));
    const double pcx =
      nominal(centre_row, static_cast<Eigen::Index>(idx_x_)) - shift_x[node];
    const double pcy =
      nominal(centre_row, static_cast<Eigen::Index>(idx_y_)) - shift_y[node];

    /* Occupied cells within the search window, sorted by distance to the node. */
    candidates.clear();
    {
      std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
      const int size_x = static_cast<int>(costmap->getSizeInCellsX());
      const int size_y = static_cast<int>(costmap->getSizeInCellsY());
      unsigned int mx0 = 0;
      unsigned int my0 = 0;
      if (!costmap->worldToMap(pcx, pcy, mx0, my0)) {continue;}
      for (int dy = -win; dy <= win; ++dy) {
        const int my = static_cast<int>(my0) + dy;
        if (my < 0 || my >= size_y) {continue;}
        for (int dx = -win; dx <= win; ++dx) {
          const int mx = static_cast<int>(mx0) + dx;
          if (mx < 0 || mx >= size_x) {continue;}
          const unsigned char cost =
            costmap->getCost(static_cast<unsigned int>(mx), static_cast<unsigned int>(my));
          if (cost < costmap_cost_threshold_ || cost == nav2_costmap_2d::NO_INFORMATION) {
            continue;
          }
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
    }
    std::sort(
      candidates.begin(), candidates.end(),
      [](const std::array<double, 3> & a, const std::array<double, 3> & b) {return a[0] < b[0];});

    /* Cluster: keep the nearest representatives at least cluster-radius apart. */
    picked.clear();
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

    /* Bind each representative to the object it continues, so a slot carries one
     * physical object across the horizon instead of whatever happened to be
     * nearest at each node independently. The coupled CBF compares slot s at node
     * k against slot s at node k-1, so a slot that changes object between them
     * compares two unrelated distances. Matching is against the object's most
     * recently seen position, which follows an extended obstacle whose nearest
     * representative slides along it as the scan centre advances; an object
     * already claimed at this node cannot be claimed twice. */
    for (const auto & pk : picked) {
      const double pd2 = (pk[0] - pcx) * (pk[0] - pcx) + (pk[1] - pcy) * (pk[1] - pcy);
      std::size_t match = objects.size();
      double match_d2 = cr2;
      for (std::size_t oi = 0; oi < objects.size(); ++oi) {
        if (objects[oi].last_node == node) {continue;}
        const double ox = pk[0] - objects[oi].x;
        const double oy = pk[1] - objects[oi].y;
        const double od2 = ox * ox + oy * oy;
        if (od2 < match_d2) {
          match_d2 = od2;
          match = oi;
        }
      }
      if (match == objects.size()) {
        objects.push_back({pk[0], pk[1], pd2, node, node});
      } else {
        objects[match].x = pk[0];
        objects[match].y = pk[1];
        objects[match].best_d2 = std::min(objects[match].best_d2, pd2);
        objects[match].last_node = node;
      }
      hits.push_back({node, match, pk[0], pk[1]});
    }
  }

  /* Slots go to the objects encountered earliest along the predicted trajectory,
   * ties broken by closest approach. Ranking on closest approach alone spends the
   * capacity on whichever object comes nearest anywhere on the horizon, which
   * degrades as np grows: at a long horizon the winner can be an encounter many
   * seconds out that is re-planned long before it happens, while a near obstacle
   * gets no in-loop constraint. Time-to-encounter is invariant to horizon length,
   * because extra nodes are appended past the ones that decide the order, and it
   * is the rule Autoware's cruise planner uses (nearest along the trajectory). An
   * object that wins a slot keeps it at every node it was seen at; the nodes it
   * was not seen at keep the far sentinel, which the core reads as an unfilled
   * slot and excludes from the CBF coupling. */
  std::vector<std::size_t> order(objects.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(
    order.begin(), order.end(),
    [&objects](std::size_t a, std::size_t b) {
      if (objects[a].first_node != objects[b].first_node) {
        return objects[a].first_node < objects[b].first_node;
      }
      return objects[a].best_d2 < objects[b].best_d2;
    });

  std::vector<std::size_t> slot_of(objects.size(), kNoObstacleSlot);
  for (std::size_t rank = 0; rank < order.size() && rank < budget; ++rank) {
    slot_of[order[rank]] = slot_begin + rank;
  }
  for (const auto & h : hits) {
    const std::size_t slot = slot_of[h.object];
    if (slot == kNoObstacleSlot) {continue;}
    const Eigen::Index row = static_cast<Eigen::Index>(h.node * k_obs + slot);
    obs(row, 0) = h.x + shift_x[h.node];
    obs(row, 1) = h.y + shift_y[h.node];
    obs(row, 2) = d_safe;
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
  /* predicted_obstacles_ is filled by this thread but cleared by deactivate()
   * and cleanup() on the lifecycle thread, so the read is guarded like the fill.
   * The array is published after the lock is dropped: the tracker callback takes
   * the same mutex, and this path only runs while something is subscribed. */
  {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
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
  }
  marker_pub_->publish(arr);
}

double ProxMpcController::markCycleStart()
{
  const auto t_now = std::chrono::steady_clock::now();
  double period_ms = std::numeric_limits<double>::quiet_NaN();
  if (have_last_cycle_) {
    period_ms = std::chrono::duration<double, std::milli>(t_now - last_cycle_wall_).count();
  }
  last_cycle_wall_ = t_now;
  have_last_cycle_ = true;
  return period_ms;
}

void ProxMpcController::publishDiagnostics(
  const rclcpp::Time & stamp, double solve_ms, double period_ms, bool converged,
  std::uint16_t num_active_obstacles)
{
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
