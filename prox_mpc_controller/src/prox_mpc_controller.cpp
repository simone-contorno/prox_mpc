// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


#include "prox_mpc_controller/prox_mpc_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <tuple>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_core/controller_exceptions.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>

namespace
{

/// Stop speed below which a cancel ramp is considered complete [m/s, rad/s].
constexpr double kCancelStopEpsilon = 0.01;
/// Upper bound on the costmap scan half-window [cells] to keep the per-cycle cost
/// bounded on constrained hardware.
constexpr int kMaxScanHalfWidth = 50;

/// Planar yaw from a quaternion.
double quat_yaw(const geometry_msgs::msg::Quaternion & q)
{
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny, cosy);
}

/// One deceleration step toward zero, respecting the sign of the previous value.
double brake_toward(double prev, double decel, double dt)
{
  const double step = std::abs(decel) * dt;
  if (prev > 0.0) {return std::max(0.0, prev - step);}
  if (prev < 0.0) {return std::min(0.0, prev + step);}
  return 0.0;
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

  /* Parameters (declared under the plugin instance namespace, e.g. FollowPath.*). */
  const std::string p = plugin_name_ + ".";
  const std::string model_plugin =
    node->declare_parameter<std::string>(p + "model_plugin", "prox_mpc_core/Bicycle");
  const double model_l = node->declare_parameter<double>(p + "model_params.L", 1.6);
  np_ = static_cast<std::size_t>(node->declare_parameter<int>(p + "np", 20));
  nc_ = static_cast<std::size_t>(node->declare_parameter<int>(p + "nc", 20));
  dt_ = node->declare_parameter<double>(p + "dt", 0.1);
  desired_linear_vel_ = node->declare_parameter<double>(p + "desired_linear_vel", 1.0);
  const double q_pos = node->declare_parameter<double>(p + "q_pos", 10.0);
  const double q_theta = node->declare_parameter<double>(p + "q_theta", 1.0);
  const double s_factor = node->declare_parameter<double>(p + "s_factor", 2.0);
  const double r_weight = node->declare_parameter<double>(p + "r_weight", 0.1);
  const double w_weight = node->declare_parameter<double>(p + "w_weight", 100.0);
  const int max_int_iter_qp = node->declare_parameter<int>(p + "max_int_iter_qp", 1500);
  const int max_ext_iter_qp = node->declare_parameter<int>(p + "max_ext_iter_qp", 10000);
  const int max_iter_sqp = node->declare_parameter<int>(p + "max_iter_sqp", 100);
  const bool qp_type = node->declare_parameter<bool>(p + "qp_type", false);
  const bool guess = node->declare_parameter<bool>(p + "guess", true);
  max_solver_failures_ = node->declare_parameter<int>(p + "max_solver_failures", 3);
  max_obstacles_ = node->declare_parameter<int>(p + "max_obstacles", 1);
  safety_margin_ = node->declare_parameter<double>(p + "safety_margin", 0.1);
  robot_radius_ = node->declare_parameter<double>(p + "robot_radius", 0.5);
  cbf_gamma_ = node->declare_parameter<double>(p + "cbf_gamma", 1.0);
  costmap_cost_threshold_ = node->declare_parameter<int>(p + "costmap_cost_threshold", 200);
  obstacle_cluster_radius_ = node->declare_parameter<double>(p + "obstacle_cluster_radius", 0.3);
  const std::string log_level = node->declare_parameter<std::string>(p + "log_level", "info");

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
  const std::map<std::string, double> model_params{{"L", model_l}};
  model_->configure(model_params);
  n_ = model_->getN();
  m_ = model_->getM();

  /* Read the model's speed bound and deceleration limits from its constraints. */
  auto bound = [](prox_mpc::Model & model, const std::string & var, std::size_t idx,
    std::size_t which) -> double {
      const auto & ineq = model.getIneq(var);
      for (const auto & entry : ineq) {
        if (static_cast<std::size_t>(entry.second[0]) == idx) {return entry.second[which];}
      }
      return 0.0;
    };
  v_max_ = bound(*model_, "u", 0, 2);
  max_linear_vel_ = v_max_;
  a_dec_lin_ = std::abs(bound(*model_, "du", 0, 1));
  a_dec_ang_ = std::abs(bound(*model_, "du", 1, 1));

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
  mpc_->setGuess(guess);
  mpc_->setQPtype(qp_type);
  mpc_->setMaxObs(k_obs);
  mpc_->init(model_);

  /* Re-apply a speed limit received before the model was available. */
  if (speed_limit_ != 0.0) {setSpeedLimit(speed_limit_, speed_limit_is_percentage_);}

  RCLCPP_INFO(
    logger_, "Configured ProxMpcController '%s' (model '%s', Np=%zu, Nc=%zu, dt=%.3f, K=%zu).",
    plugin_name_.c_str(), model_plugin.c_str(), np_, nc_, dt_, k_obs);
}

void ProxMpcController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up ProxMpcController '%s'.", plugin_name_.c_str());
  mpc_.reset();
  model_.reset();
  model_loader_.reset();
  costmap_ros_.reset();
  tf_.reset();
}

void ProxMpcController::activate()
{
  failure_count_ = 0;
  steering_state_ = 0.0;
  last_cmd_v_ = 0.0;
  last_cmd_w_ = 0.0;
  cancelling_ = false;
  RCLCPP_INFO(logger_, "Activating ProxMpcController '%s'.", plugin_name_.c_str());
}

void ProxMpcController::deactivate()
{
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
  (void)velocity;
  (void)goal_checker;

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = costmap_ros_->getBaseFrameID();
  cmd.header.stamp = clock_->now();

  /* Deceleration ramp shared by the cancel, solver-failure, and veto paths. */
  auto make_brake = [&]() -> geometry_msgs::msg::TwistStamped {
      last_cmd_v_ = brake_toward(last_cmd_v_, a_dec_lin_, dt_);
      last_cmd_w_ = brake_toward(last_cmd_w_, a_dec_ang_, dt_);
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
        th = (plan_size < 2) ? quat_yaw(global_plan_.poses.back().pose.orientation) :
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

  /* Build the state and control references. The steering channel of goal_x is
   * left at zero (go-straight preference), matching the core reference setup. */
  MatrixXd goal_x = MatrixXd::Zero(np_ + 1, n_);
  for (std::size_t k = 0; k <= np_; ++k) {
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;
    sample(s0 + v_ref * static_cast<double>(k) * dt_, x, y, th);
    goal_x(k, 0) = x;
    goal_x(k, 1) = y;
    goal_x(k, 2) = th;
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

  /* Reduce the local costmap to obstacle triples for this cycle. */
  const std::size_t k_obs = mpc_->getMaxObs();
  if (k_obs > 0) {
    MatrixXd obs(static_cast<Eigen::Index>(np_ * k_obs), 3);
    reduceCostmap(goal_x, obs);
    mpc_->setObs(obs);
  }

  /* Solve one SQP cycle. */
  mpc_->setGoalX(goal_x);
  mpc_->setGoalU(goal_u);
  mpc_->setPose(state);
  auto [x_sol, u_sol] = mpc_->solve();

  const bool solved =
    (mpc_->qp_info.status == proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED);
  if (!solved) {return fail("solver did not converge");}

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
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 2000,
          "ProxMpcController: footprint check vetoed the command; decelerating.");
        return make_brake();
      }
    }
  }

  /* Map the first control to a body twist; the model reads the current state. */
  model_->setX(state);
  const geometry_msgs::msg::Twist twist = model_->toTwist(u0);
  if (!std::isfinite(twist.linear.x) || !std::isfinite(twist.angular.z)) {
    return fail("non-finite command");
  }

  failure_count_ = 0;
  if (n_ > 3) {steering_state_ = x_sol(1, 3);}
  last_cmd_v_ = twist.linear.x;
  last_cmd_w_ = twist.angular.z;
  cmd.twist = twist;
  return cmd;
}

void ProxMpcController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
  if (!model_) {return;}  // applied in configure() once the model is loaded

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
  steering_state_ = 0.0;
  last_cmd_v_ = 0.0;
  last_cmd_w_ = 0.0;
  cancelling_ = false;
  plan_index_ = 0;
}

void ProxMpcController::reduceCostmap(const MatrixXd & reference, MatrixXd & obs)
{
  const std::size_t k_obs = static_cast<std::size_t>(max_obstacles_);
  const double d_safe = robot_radius_ + safety_margin_;

  /* Default every slot to the far sentinel so unfilled ones stay non-binding. */
  for (Eigen::Index r = 0; r < obs.rows(); ++r) {
    obs(r, 0) = prox_mpc::MPC::kObsFarSentinel;
    obs(r, 1) = prox_mpc::MPC::kObsFarSentinel;
    obs(r, 2) = 0.0;
  }

  auto * costmap = costmap_ros_->getCostmap();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
  const double res = costmap->getResolution();
  if (res <= 0.0) {return;}

  const double search_radius = d_safe + obstacle_cluster_radius_;
  int win = static_cast<int>(std::ceil(search_radius / res));
  if (win > kMaxScanHalfWidth) {win = kMaxScanHalfWidth;}
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
        if (dd <= sr2) {candidates.push_back({dd, wx, wy});}
      }
    }
    std::sort(
      candidates.begin(), candidates.end(),
      [](const std::array<double, 3> & a, const std::array<double, 3> & b) {return a[0] < b[0];});

    /* Cluster: keep the nearest representatives at least cluster-radius apart. */
    std::vector<std::array<double, 2>> picked;
    for (const auto & cd : candidates) {
      if (picked.size() >= k_obs) {break;}
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
      const Eigen::Index row = static_cast<Eigen::Index>(node * k_obs + slot);
      obs(row, 0) = picked[slot][0];
      obs(row, 1) = picked[slot][1];
      obs(row, 2) = d_safe;
    }
  }
}

}  // namespace prox_mpc_controller

PLUGINLIB_EXPORT_CLASS(prox_mpc_controller::ProxMpcController, nav2_core::Controller)
