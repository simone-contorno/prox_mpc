// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Self-contained closed-loop NMPC simulation node.
//
// It drives one of the bundled kinematic models (bicycle / unicycle) toward a goal (or a
// waypoint set) with no external simulator: each cycle it solves the MPC,
// publishes the first control on /robot/cmd_vel and the predicted trajectory on
// /prox_mpc/path, then advances the simulated pose to the model's own predicted
// next state. It also measures the solve time (min / avg / max in ms) and logs it
// periodically, and (when publish_diagnostics is true) publishes one
// prox_mpc_msgs/SolverDiagnostics per cycle on /prox_mpc/diagnostics so the
// real-time and feasibility metric classes are observable.
//
// Obstacles may be a single fixed obstacle (obs_x / obs_y / d_safe) or a
// time-varying list (obs_motion = static | circle | line). For a dynamic
// obstacle the per-node predicted positions are filled.
//
// step() and broadcastPose() are protected so a unit test can drive one control
// cycle deterministically without spinning the wall timer.

#ifndef PROX_MPC_DEMO__SIMULATION_NODE_HPP_
#define PROX_MPC_DEMO__SIMULATION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <prox_mpc_msgs/msg/solver_diagnostics.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <prox_mpc/mpc.hpp>
#include <prox_mpc/models/bicycle.hpp>
#include <prox_mpc/models/unicycle.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

class SimulationNode : public rclcpp::Node
{
public:
  explicit SimulationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("prox_mpc_simulation", options)
  {
    /* Parameters (defaults let the node run without a YAML). */
    model_name_ = declare_parameter<std::string>("model", "bicycle");
    // Validate sizing before the size_t cast: a negative np/nc would wrap to an
    // astronomical allocation, and dt <= 0 divides by zero in the solver.
    const int np_param = declare_parameter<int>("np", 20);
    const int nc_param = declare_parameter<int>("nc", 20);
    dt_ = declare_parameter<double>("dt", 0.1);
    if (np_param < 1 || nc_param < 1 || dt_ <= 0.0) {
      throw std::invalid_argument("prox_mpc_simulation: np >= 1, nc >= 1, dt > 0 required");
    }
    np_ = static_cast<size_t>(np_param);
    nc_ = static_cast<size_t>(nc_param);
    const double q_pos = declare_parameter<double>("q_pos", 10.0);
    const double q_theta = declare_parameter<double>("q_theta", 1.0);
    const double s_factor = declare_parameter<double>("s_factor", 2.0);
    const double r_weight = declare_parameter<double>("r_weight", 0.1);
    const double w_weight = declare_parameter<double>("w_weight", 100.0);
    v_ref_ = declare_parameter<double>("v_ref", 1.0);
    goal_x_ = declare_parameter<double>("goal_x", 5.0);
    goal_y_ = declare_parameter<double>("goal_y", 0.0);
    goal_theta_ = declare_parameter<double>("goal_theta", 0.0);

    /* Waypoint set (parallel arrays). Empty -> the single goal above. The active
     * goal advances to the next waypoint once the pose is within goal_tol. */
    goals_x_ = declare_parameter<std::vector<double>>("goals_x", std::vector<double>{});
    goals_y_ = declare_parameter<std::vector<double>>("goals_y", std::vector<double>{});
    goals_theta_ = declare_parameter<std::vector<double>>("goals_theta", std::vector<double>{});
    goal_tol_ = declare_parameter<double>("goal_tol", 0.25);
    if (goals_x_.empty()) {
      goals_x_ = {goal_x_};
      goals_y_ = {goal_y_};
      goals_theta_ = {goal_theta_};
    }
    if (goals_y_.size() != goals_x_.size() || goals_theta_.size() != goals_x_.size()) {
      throw std::invalid_argument(
              "prox_mpc_simulation: goals_x / goals_y / goals_theta must have equal length");
    }

    obstacle_enable_ = declare_parameter<bool>("obstacle_enable", false);
    const int max_obstacles_param = declare_parameter<int>("max_obstacles", 1);
    d_safe_ = declare_parameter<double>("d_safe", 1.0);
    if (max_obstacles_param < 0 || d_safe_ < 0.0) {
      throw std::invalid_argument(
              "prox_mpc_simulation: max_obstacles >= 0 and d_safe >= 0 required");
    }
    max_obstacles_ = static_cast<size_t>(max_obstacles_param);
    obs_x_ = declare_parameter<double>("obs_x", 2.5);
    obs_y_ = declare_parameter<double>("obs_y", 0.6);
    report_period_ = static_cast<size_t>(declare_parameter<int>("report_period", 50));
    publish_diagnostics_ = declare_parameter<bool>("publish_diagnostics", true);

    /* Time-varying obstacle list (parallel arrays). When non-empty it supersedes
     * the single fixed obstacle and enables avoidance regardless of
     * obstacle_enable. Each obstacle is static | circle | line (see positionAt). */
    parseObstacleList();
    const bool avoidance_on = obstacle_enable_ || !obstacles_.empty();
    const size_t obs_count =
      !obstacles_.empty() ? obstacles_.size() : (obstacle_enable_ ? 1u : 0u);
    k_obs_ = avoidance_on ? std::max(max_obstacles_, obs_count) : 0;

    /* Model. The handle is retained so the node can map controls to a twist. */
    if (model_name_ == "unicycle") {model_ = std::make_shared<prox_mpc::Unicycle>();} else {
      model_ = std::make_shared<prox_mpc::Bicycle>();
    }
    n_ = model_->getN();
    m_ = model_->getM();

    /* Deceleration limits for the solver-failure ramp, taken from the model's own
     * control-rate ("du") bounds - the same source the Nav2 plugin uses. Index 1 is
     * the model's second control rate (the unicycle's angular acceleration, the
     * bicycle's steering acceleration), which is what the plugin ramps too. */
    a_dec_lin_ = decelBound(*model_, 0);
    a_dec_ang_ = decelBound(*model_, 1);

    /* Weights sized to the chosen model. */
    VectorXd q_diag = VectorXd::Constant(n_, q_theta);
    q_diag(0) = q_pos;
    q_diag(1) = q_pos;
    MatrixXd Q = q_diag.asDiagonal();
    MatrixXd S = s_factor * Q;
    MatrixXd R = r_weight * MatrixXd::Identity(m_, m_);
    MatrixXd W = MatrixXd::Constant(1, 1, w_weight);

    /* MPC. */
    mpc_ = std::make_shared<prox_mpc::MPC>();
    mpc_->setNp(np_);
    mpc_->setNc(nc_);
    mpc_->setdt(dt_);
    mpc_->setQ(Q);
    mpc_->setS(S);
    mpc_->setR(R);
    mpc_->setW(W);
    mpc_->setMaxObs(k_obs_);   // capacity K per node; sizes the QP once (0 = avoidance off)
    mpc_->init(model_);

    /* References. The goal heading is re-unwrapped relative to the current pose
     * each step (see step()) so the QP tracking error never wraps near +/-pi. */
    goal_states_ = MatrixXd::Zero(np_ + 1, n_);
    setGoalRows(goals_x_.front(), goals_y_.front(), goals_theta_.front());
    MatrixXd goal_u = MatrixXd::Zero(nc_, m_);
    for (size_t k = 0; k < nc_; k++) {goal_u(k, 0) = v_ref_;}
    mpc_->setGoalX(goal_states_);
    mpc_->setGoalU(goal_u);

    pose_ = VectorXd::Zero(n_);
    /* Optional start pose so a scenario can place the robot away from the origin. */
    pose_(0) = declare_parameter<double>("start_x", 0.0);
    pose_(1) = declare_parameter<double>("start_y", 0.0);
    if (n_ >= 3) {pose_(2) = declare_parameter<double>("start_theta", 0.0);}

    /* Interfaces. Reliable, depth 1: a command/visualization stream where only
     * the freshest sample matters (explicit profile rather than the depth-only
     * default). */
    pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>(
      "/robot/cmd_vel", rclcpp::QoS(1).reliable());
    pub_path_ = create_publisher<nav_msgs::msg::Path>(
      "/prox_mpc/path", rclcpp::QoS(1).reliable());
    if (publish_diagnostics_) {
      pub_diag_ = create_publisher<prox_mpc_msgs::msg::SolverDiagnostics>(
        "/prox_mpc/diagnostics", rclcpp::QoS(10).reliable());
    }
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_), std::bind(&SimulationNode::step, this));

    RCLCPP_INFO(get_logger(),
                "prox_mpc simulation: model=%s, Np=%zu, Nc=%zu, dt=%.3f s, "
                "waypoints=%zu, obstacles=%zu, K=%zu",
                model_name_.c_str(), np_, nc_, dt_, goals_x_.size(), obs_count, k_obs_);
  }

protected:
  /* One closed-loop control cycle: pick the active waypoint, refresh the
   * (time-varying) obstacle field, solve, gate, command, advance the pose, and
   * publish diagnostics. */
  void step()
  {
    const std::chrono::steady_clock::time_point t_entry = std::chrono::steady_clock::now();
    double period_ms = std::numeric_limits<double>::quiet_NaN();
    if (have_last_step_) {
      period_ms = std::chrono::duration<double, std::milli>(t_entry - last_step_wall_).count();
    }
    last_step_wall_ = t_entry;
    have_last_step_ = true;

    /* Waypoint advance: switch to the next waypoint once the current one is
     * reached (within goal_tol), holding the final waypoint as the terminal goal. */
    advanceWaypoint();
    const double gx = goals_x_[current_wp_];
    const double gy = goals_y_[current_wp_];
    const double gtheta = goals_theta_[current_wp_];

    /* Latch arrival at the final waypoint: a waypoint follower stops at its goal
     * rather than driving through it (the MPC's v_ref reference would otherwise
     * overshoot). Once arrived the robot holds; the solver still runs so the
     * real-time/feasibility telemetry keeps flowing. */
    if (current_wp_ + 1 == goals_x_.size() &&
      std::hypot(pose_(0) - gx, pose_(1) - gy) <= goal_tol_)
    {
      arrived_ = true;
    }

    /* Unwrap the goal heading onto the current pose's branch so the QP heading
     * error is the shortest rotation, not the long way around the +/-pi wrap. */
    const double goal_theta_cont = pose_(2) + std::remainder(gtheta - pose_(2), 2.0 * M_PI);
    setGoalRows(gx, gy, goal_theta_cont);
    mpc_->setGoalX(goal_states_);

    /* Refresh the obstacle field for this cycle, filling each predicted node with
     * the obstacle positions at that node's time (predictive for dynamic ones). */
    refreshObstacles();

    mpc_->setPose(pose_);

    const auto t0 = std::chrono::high_resolution_clock::now();
    auto [x, u] = mpc_->solve();
    const auto t1 = std::chrono::high_resolution_clock::now();
    const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    /* Solve-time statistics. */
    min_ms_ = std::min(min_ms_, ms);
    max_ms_ = std::max(max_ms_, ms);
    sum_ms_ += ms;
    count_++;
    if (report_period_ > 0 && count_ % report_period_ == 0) {
      RCLCPP_INFO(get_logger(),
                  "solve over %zu steps [ms]  min=%.3f  avg=%.3f  max=%.3f  "
                  "(sqp_iter=%zu, qp_iter=%zu)  ~%.1f Hz budget",
                  report_period_, min_ms_, sum_ms_ / static_cast<double>(count_),
                  max_ms_, mpc_->sqp_iter, mpc_->qp_iter_ext, 1.0 / dt_);
    }

    /* Gate on convergence and finiteness before applying the solve. MPC::solve
     * takes no safety action, so on a non-converged or non-finite iterate ramp the
     * command down toward zero and hold the pose instead of folding a bad iterate
     * into pose_. */
    const VectorXd u0 = u.row(0);
    const bool solved =
      mpc_->qp_info.status == proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED &&
      u0.allFinite() && x.row(1).allFinite();

    publishDiagnostics(ms, period_ms, solved);

    /* Hold at the goal: command zero and keep the pose parked, so the final goal
     * error reflects the stopping point rather than post-goal drift. Clearing the
     * retained command keeps a later ramp from braking off a stale value. */
    if (arrived_) {
      last_cmd_v_ = 0.0;
      last_cmd_w_ = 0.0;
      pub_cmd_->publish(geometry_msgs::msg::Twist());
      broadcastPose();
      sim_time_ += dt_;
      return;
    }

    if (!solved) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "solve did not converge or returned a non-finite iterate; ramping down.");
      /* Decelerate toward zero within the model's acceleration limits instead of
       * stopping dead. This node is its own plant, so there is no separately
       * measured velocity: the ramp starts from the last command it published,
       * which is exactly what the simulated robot is executing. */
      geometry_msgs::msg::Twist brake;
      last_cmd_v_ = brakeToward(last_cmd_v_, a_dec_lin_, dt_);
      last_cmd_w_ = brakeToward(last_cmd_w_, a_dec_ang_, dt_);
      brake.linear.x = last_cmd_v_;
      brake.angular.z = last_cmd_w_;
      pub_cmd_->publish(brake);
      broadcastPose();
      sim_time_ += dt_;
      return;
    }

    /* Publish the first control as a body twist. The mapping is model specific,
     * so the model derives it (and reads the current state where needed). */
    model_->setX(pose_);
    const geometry_msgs::msg::Twist cmd = model_->toTwist(u0);
    last_cmd_v_ = cmd.linear.x;
    last_cmd_w_ = cmd.angular.z;
    pub_cmd_->publish(cmd);

    /* Publish the predicted trajectory for visualization. */
    pub_path_->publish(prox_mpc::optimPath(x, now()));

    /* Closed-loop: advance to the model's predicted next state. */
    pose_ = x.row(1);
    prox_mpc::normalizeAngle(pose_(2));

    /* Broadcast map -> base_link so RViz tracks the simulated pose. */
    broadcastPose();
    sim_time_ += dt_;
  }

  /* Broadcast the planar simulated pose as the map -> base_link transform. */
  void broadcastPose()
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = "map";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = pose_(0);
    tf.transform.translation.y = pose_(1);
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.z = std::sin(pose_(2) / 2.0);
    tf.transform.rotation.w = std::cos(pose_(2) / 2.0);
    tf_broadcaster_->sendTransform(tf);
  }

  /* A single time-varying obstacle. motion selects how positionAt() integrates
   * the centre over time; static obstacles ignore the motion fields. */
  struct ObstacleSpec
  {
    std::string motion = "static";  // static | circle | line
    double cx = 0.0, cy = 0.0;      // static pos / circle centre / line "from"
    double ex = 0.0, ey = 0.0;      // line "to"
    double radius = 1.0;            // circle orbit radius [m]
    double speed = 0.0;             // signed speed [m/s] (direction by sign)
    double clearance = 1.0;         // keep-out radius d_safe [m]
  };

  /* Obstacle centre at a given absolute sim time. */
  static std::pair<double, double> positionAt(const ObstacleSpec & o, double time)
  {
    if (o.motion == "circle") {
      const double r = std::max(o.radius, 1e-9);
      const double ang = (o.speed / r) * time;  // sign of speed = ccw / cw
      return {o.cx + r * std::cos(ang), o.cy + r * std::sin(ang)};
    }
    if (o.motion == "line") {
      const double dx = o.ex - o.cx;
      const double dy = o.ey - o.cy;
      const double len = std::hypot(dx, dy);
      if (len < 1e-9) {return {o.cx, o.cy};}
      // Patrol from start -> end -> start; speed sign selects the first leg.
      const double sx = o.speed >= 0.0 ? o.cx : o.ex;
      const double sy = o.speed >= 0.0 ? o.cy : o.ey;
      const double tx = o.speed >= 0.0 ? o.ex : o.cx;
      const double ty = o.speed >= 0.0 ? o.ey : o.cy;
      const double s = std::fmod(std::abs(o.speed) * time, 2.0 * len);
      const double frac = s <= len ? s / len : (2.0 * len - s) / len;
      return {sx + frac * (tx - sx), sy + frac * (ty - sy)};
    }
    return {o.cx, o.cy};  // static
  }

  /* Deceleration limit used when the model declares no (or a zero) lower bound on
   * a control rate. A zero limit would make the failure ramp never reach zero.
   * The demo drives no hardware, so it falls back rather than refusing to start
   * (the Nav2 plugin, which does drive hardware, throws instead). */
  static constexpr double kFallbackDecel = 0.5;

  /* Magnitude of the model's lower "du" bound for control component idx, i.e. its
   * deceleration limit, or kFallbackDecel when that bound is absent or zero. */
  static double decelBound(prox_mpc::Model & model, size_t idx)
  {
    for (const auto & entry : model.getIneq("du")) {
      if (static_cast<size_t>(entry.second[0]) == idx) {
        const double decel = std::abs(entry.second[1]);
        if (decel > 0.0) {return decel;}
      }
    }
    return kFallbackDecel;
  }

  /* One deceleration step from prev toward zero, preserving sign and clamped at
   * zero. A non-finite previous command yields zero rather than propagating it. */
  static double brakeToward(double prev, double decel, double dt)
  {
    if (!std::isfinite(prev)) {return 0.0;}
    const double step = std::abs(decel) * dt;
    if (prev > 0.0) {return std::max(0.0, prev - step);}
    if (prev < 0.0) {return std::min(0.0, prev + step);}
    return 0.0;
  }

  std::string model_name_;
  size_t np_, nc_, n_, m_;
  double dt_, v_ref_, goal_x_, goal_y_, goal_theta_, d_safe_, obs_x_, obs_y_;
  bool obstacle_enable_;
  size_t max_obstacles_;
  size_t k_obs_ = 0;  // effective obstacle-slot capacity per node passed to the MPC
  size_t report_period_;
  bool publish_diagnostics_ = true;

  std::vector<double> goals_x_, goals_y_, goals_theta_;
  double goal_tol_ = 0.25;
  size_t current_wp_ = 0;
  bool arrived_ = false;

  std::vector<ObstacleSpec> obstacles_;
  double sim_time_ = 0.0;  // deterministic sim clock (step_index * dt)

  /* Last twist published, and the per-axis deceleration limits the failure ramp
   * walks it down with. */
  double last_cmd_v_ = 0.0;
  double last_cmd_w_ = 0.0;
  double a_dec_lin_ = kFallbackDecel;
  double a_dec_ang_ = kFallbackDecel;

  std::shared_ptr<prox_mpc::Model> model_;
  std::shared_ptr<prox_mpc::MPC> mpc_;
  VectorXd pose_;
  MatrixXd goal_states_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<prox_mpc_msgs::msg::SolverDiagnostics>::SharedPtr pub_diag_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  double min_ms_ = std::numeric_limits<double>::infinity();
  double max_ms_ = 0.0;
  double sum_ms_ = 0.0;
  size_t count_ = 0;
  std::chrono::steady_clock::time_point last_step_wall_;
  bool have_last_step_ = false;

private:
  /* Fill every horizon row of goal_states_ with one (x, y, theta) goal. */
  void setGoalRows(double gx, double gy, double gtheta)
  {
    for (size_t k = 0; k <= np_; k++) {
      goal_states_(k, 0) = gx;
      goal_states_(k, 1) = gy;
      if (n_ >= 3) {goal_states_(k, 2) = gtheta;}
    }
  }

  /* Advance current_wp_ once the active waypoint is reached, holding the last. */
  void advanceWaypoint()
  {
    while (current_wp_ + 1 < goals_x_.size()) {
      const double dx = pose_(0) - goals_x_[current_wp_];
      const double dy = pose_(1) - goals_y_[current_wp_];
      if (std::hypot(dx, dy) <= goal_tol_) {current_wp_++;} else {break;}
    }
  }

  /* Build the (Np*K) x 3 obstacle matrix for this cycle. Each predicted node k
   * gets the obstacle centres at time (sim_time_ + k*dt) so a dynamic obstacle is
   * tracked predictively; unused slots stay at the far sentinel (non-binding). */
  void refreshObstacles()
  {
    if (k_obs_ == 0) {return;}
    MatrixXd obs = MatrixXd::Zero(static_cast<Eigen::Index>(np_ * k_obs_), 3);
    for (Eigen::Index r = 0; r < obs.rows(); r++) {
      obs(r, 0) = prox_mpc::MPC::kObsFarSentinel;
      obs(r, 1) = prox_mpc::MPC::kObsFarSentinel;
      obs(r, 2) = 0.0;
    }
    if (!obstacles_.empty()) {
      for (size_t node = 0; node < np_; node++) {
        const double t = sim_time_ + static_cast<double>(node) * dt_;
        for (size_t i = 0; i < obstacles_.size() && i < k_obs_; i++) {
          const auto [ox, oy] = positionAt(obstacles_[i], t);
          const Eigen::Index row = static_cast<Eigen::Index>(node * k_obs_ + i);
          obs(row, 0) = ox;
          obs(row, 1) = oy;
          obs(row, 2) = obstacles_[i].clearance;
        }
      }
    } else {
      // Single-obstacle form: written into slot 0 of every node.
      for (size_t node = 0; node < np_; node++) {
        const Eigen::Index row = static_cast<Eigen::Index>(node * k_obs_);
        obs(row, 0) = obs_x_;
        obs(row, 1) = obs_y_;
        obs(row, 2) = d_safe_;
      }
    }
    mpc_->setObs(obs);
  }

  /* Read the parallel obstacle-list parameters into obstacles_. Lengths must
   * match obs_motion; a shorter array is padded with its default. */
  void parseObstacleList()
  {
    const auto motion = declare_parameter<std::vector<std::string>>(
      "obs_motion", std::vector<std::string>{});
    if (motion.empty()) {return;}
    const auto cx = declare_parameter<std::vector<double>>("obs_cx", std::vector<double>{});
    const auto cy = declare_parameter<std::vector<double>>("obs_cy", std::vector<double>{});
    const auto ex = declare_parameter<std::vector<double>>("obs_ex", std::vector<double>{});
    const auto ey = declare_parameter<std::vector<double>>("obs_ey", std::vector<double>{});
    const auto radius = declare_parameter<std::vector<double>>("obs_radius", std::vector<double>{});
    const auto speed = declare_parameter<std::vector<double>>("obs_speed", std::vector<double>{});
    const auto clearance =
      declare_parameter<std::vector<double>>("obs_clearance", std::vector<double>{});
    auto at = [](const std::vector<double> & v, size_t i, double dflt) {
        return i < v.size() ? v[i] : dflt;
      };
    obstacles_.reserve(motion.size());
    for (size_t i = 0; i < motion.size(); i++) {
      ObstacleSpec o;
      o.motion = motion[i];
      o.cx = at(cx, i, 0.0);
      o.cy = at(cy, i, 0.0);
      o.ex = at(ex, i, 0.0);
      o.ey = at(ey, i, 0.0);
      o.radius = at(radius, i, 1.0);
      o.speed = at(speed, i, 0.0);
      o.clearance = at(clearance, i, d_safe_);
      obstacles_.push_back(o);
    }
  }

  /* Map proxsuite's solver outcome onto the message's own STATUS_* contract.
   * Deliberately not a static_cast: proxsuite 0.6.5 inserted
   * PROXQP_SOLVED_CLOSEST_PRIMAL_FEASIBLE into the middle of QPSolverOutput, so a
   * cast reports a dual-infeasible solve as STATUS_NOT_RUN. An enumerator added
   * upstream after this mapping was written falls through to STATUS_UNKNOWN
   * rather than impersonating another state. */
  static uint8_t solverStatusToMsg(proxsuite::proxqp::QPSolverOutput status)
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

  /* Fill and publish one SolverDiagnostics for this cycle (both gate branches). */
  void publishDiagnostics(double solve_ms, double period_ms, bool solved)
  {
    if (!pub_diag_) {return;}
    prox_mpc_msgs::msg::SolverDiagnostics d;
    d.header.stamp = now();
    d.header.frame_id = "base_link";
    d.solve_time_ms = solve_ms;
    d.qp_solve_time_ms = mpc_->qp_info.run_time / 1000.0;  // proxsuite reports microseconds
    d.status = solverStatusToMsg(mpc_->qp_info.status);
    d.converged = solved;
    d.sqp_iters = static_cast<uint32_t>(mpc_->sqp_iter);
    d.qp_iters_ext = static_cast<uint32_t>(mpc_->qp_iter_ext);
    d.primal_residual = mpc_->qp_info.pri_res;
    d.dual_residual = mpc_->qp_info.dua_res;
    d.objective = mpc_->qp_info.objValue;
    d.max_obstacle_slack = mpc_->getMaxObstacleSlack();
    d.control_period_ms = period_ms;
    // A missed deadline = the solve did not fit in the control budget (1000*dt ms).
    // The solve is the controller's compute cost and the budget is the wall-timer
    // period it is scheduled at, so this is the well-defined real-time signal. The
    // measured control_period_ms is published as a separate field for analysis but
    // is deliberately not folded into this flag: the nominal period equals the
    // budget by construction, so any period threshold would need an arbitrary slack.
    d.deadline_missed = solve_ms > 1000.0 * dt_;
    d.num_active_obstacles = static_cast<uint16_t>(
      !obstacles_.empty() ? std::min(obstacles_.size(), k_obs_) :
      (k_obs_ > 0 && obstacle_enable_ ? 1u : 0u));
    pub_diag_->publish(d);
  }
};

#endif  // PROX_MPC_DEMO__SIMULATION_NODE_HPP_
