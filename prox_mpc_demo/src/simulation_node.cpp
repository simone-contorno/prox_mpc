// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Self-contained closed-loop NMPC simulation node.
//
// It drives one of the bundled kinematic models (bike / r2d2) toward a goal with
// no external simulator: each cycle it solves the MPC, publishes the first
// control on /robot/cmd_vel and the predicted trajectory on /prox_mpc/path, then
// advances the simulated pose to the model's own predicted next state. It also
// measures the solve time (min / avg / max in ms) and logs it periodically, so
// the controller's compute cost and rate are observable. The effective control
// rate can also be checked with `ros2 topic hz /robot/cmd_vel`.

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <prox_mpc/mpc.hpp>
#include <prox_mpc/models/bike.hpp>
#include <prox_mpc/models/r2d2.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

using namespace prox_mpc;

class SimulationNode : public rclcpp::Node
{
public:
  SimulationNode()
  : Node("prox_mpc_simulation")
  {
    /* Parameters (defaults let the node run without a YAML). */
    model_name_ = declare_parameter<std::string>("model", "bike");
    np_ = static_cast<size_t>(declare_parameter<int>("np", 20));
    nc_ = static_cast<size_t>(declare_parameter<int>("nc", 20));
    dt_ = declare_parameter<double>("dt", 0.1);
    const double q_pos = declare_parameter<double>("q_pos", 10.0);
    const double q_theta = declare_parameter<double>("q_theta", 1.0);
    const double s_factor = declare_parameter<double>("s_factor", 2.0);
    const double r_weight = declare_parameter<double>("r_weight", 0.1);
    const double w_weight = declare_parameter<double>("w_weight", 100.0);
    v_ref_ = declare_parameter<double>("v_ref", 1.0);
    goal_x_ = declare_parameter<double>("goal_x", 5.0);
    goal_y_ = declare_parameter<double>("goal_y", 0.0);
    goal_theta_ = declare_parameter<double>("goal_theta", 0.0);
    obstacle_enable_ = declare_parameter<bool>("obstacle_enable", false);
    obs_x_ = declare_parameter<double>("obs_x", 2.5);
    obs_y_ = declare_parameter<double>("obs_y", 0.6);
    report_period_ = static_cast<size_t>(declare_parameter<int>("report_period", 50));

    /* Model. */
    std::shared_ptr<Model> model;
    if (model_name_ == "r2d2") {model = std::make_shared<Unicycle>();} else {
      model = std::make_shared<Bicycle>();
    }
    model->setObsAvoid(obstacle_enable_, model->getObsDist());
    n_ = model->getN();
    m_ = model->getM();

    /* Weights sized to the chosen model. */
    VectorXd q_diag = VectorXd::Constant(n_, q_theta);
    q_diag(0) = q_pos;
    q_diag(1) = q_pos;
    MatrixXd Q = q_diag.asDiagonal();
    MatrixXd S = s_factor * Q;
    MatrixXd R = r_weight * MatrixXd::Identity(m_, m_);
    MatrixXd W = MatrixXd::Constant(1, 1, w_weight);

    /* MPC. */
    mpc_ = std::make_shared<MPC>();
    mpc_->setNp(np_);
    mpc_->setNc(nc_);
    mpc_->setdt(dt_);
    mpc_->setQ(Q);
    mpc_->setS(S);
    mpc_->setR(R);
    mpc_->setW(W);
    mpc_->init(model);

    /* References. */
    MatrixXd goal_x = MatrixXd::Zero(np_ + 1, n_);
    for (size_t k = 0; k <= np_; k++) {
      goal_x(k, 0) = goal_x_;
      goal_x(k, 1) = goal_y_;
      goal_x(k, 2) = goal_theta_;
    }
    MatrixXd goal_u = MatrixXd::Zero(nc_, m_);
    for (size_t k = 0; k < nc_; k++) {goal_u(k, 0) = v_ref_;}
    mpc_->setGoalX(goal_x);
    mpc_->setGoalU(goal_u);

    MatrixXd obs = MatrixXd::Zero(np_ + 1, 2);
    for (size_t k = 0; k <= np_; k++) {obs(k, 0) = obs_x_; obs(k, 1) = obs_y_;}
    mpc_->setObs(obs);
    mpc_->setObsDim(1.0, 1.0, 0.5, 0.5);

    pose_ = VectorXd::Zero(n_);

    /* Interfaces. */
    pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 1);
    pub_path_ = create_publisher<nav_msgs::msg::Path>("/prox_mpc/path", 1);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_), std::bind(&SimulationNode::step, this));

    RCLCPP_INFO(get_logger(),
                "prox_mpc simulation: model=%s, Np=%zu, Nc=%zu, dt=%.3f s, obstacle=%s",
                model_name_.c_str(), np_, nc_, dt_, obstacle_enable_ ? "on" : "off");
  }

private:
  void step()
  {
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
                  "(sqp_iter=%zu, qp_iter=%u)  ~%.1f Hz budget",
                  report_period_, min_ms_, sum_ms_ / static_cast<double>(count_),
                  max_ms_, mpc_->sqp_iter, mpc_->qp_iter_ext, 1.0 / dt_);
    }

    /* Publish first control as a twist (linear.x = v, angular.z = 2nd input). */
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = u(0, 0);
    cmd.angular.z = u(0, 1);
    pub_cmd_->publish(cmd);

    /* Publish the predicted trajectory for visualization. */
    pub_path_->publish(optimPath(x, now()));

    /* Closed-loop: advance to the model's predicted next state. */
    pose_ = x.row(1);
    normalizeAngle(pose_(2));
  }

  std::string model_name_;
  size_t np_, nc_, n_, m_;
  double dt_, v_ref_, goal_x_, goal_y_, goal_theta_, obs_x_, obs_y_;
  bool obstacle_enable_;
  size_t report_period_;

  std::shared_ptr<MPC> mpc_;
  VectorXd pose_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::TimerBase::SharedPtr timer_;

  double min_ms_ = std::numeric_limits<double>::infinity();
  double max_ms_ = 0.0;
  double sum_ms_ = 0.0;
  size_t count_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulationNode>());
  rclcpp::shutdown();
  return 0;
}
