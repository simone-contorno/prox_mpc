// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Controller-agnostic live benchmark metrics node.
//
// It measures the accuracy class (cross-track error vs a reference polyline and
// goal error) from generic signals — the robot pose (TF map -> base_link, or a
// bridged ground-truth pose in Gazebo) plus the scenario reference — so every
// controller under test is measured identically (SIM_SPEC D4). It also taps the
// SolverDiagnostics stream for the real-time / feasibility class. Live cross-track
// and goal-distance are published as std_msgs/Float64 for inspection/recording,
// and a per-run summary JSON is written at shutdown (summary_json param) so the
// orchestrator can aggregate repeats without re-parsing a bag.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <prox_mpc_msgs/msg/solver_diagnostics.hpp>

#include <prox_mpc_benchmark/metrics_math.hpp>

using prox_mpc_benchmark::crossTrack;
using prox_mpc_benchmark::percentile;

class MetricsNode : public rclcpp::Node
{
public:
  MetricsNode()
  : Node("prox_mpc_metrics")
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    ref_x_ = declare_parameter<std::vector<double>>("ref_x", std::vector<double>{});
    ref_y_ = declare_parameter<std::vector<double>>("ref_y", std::vector<double>{});
    goal_x_ = declare_parameter<double>("goal_x", 0.0);
    goal_y_ = declare_parameter<double>("goal_y", 0.0);
    goal_tol_ = declare_parameter<double>("goal_tol", 0.25);
    const std::string diag_topic =
      declare_parameter<std::string>("diagnostics_topic", "/prox_mpc/diagnostics");
    const double rate = declare_parameter<double>("sample_rate_hz", 50.0);
    summary_json_ = declare_parameter<std::string>("summary_json", "");
    auto_exit_ = declare_parameter<bool>("auto_exit", true);
    settle_s_ = declare_parameter<double>("settle_s", 1.0);

    // Labels echoed into the summary so the aggregator can index the matrix cell.
    label_scenario_ = declare_parameter<std::string>("scenario", "");
    label_model_ = declare_parameter<std::string>("model", "");
    label_mode_ = declare_parameter<std::string>("mode", "");
    label_controller_ = declare_parameter<std::string>("controller", "proxmpc");
    label_repeat_ = declare_parameter<int>("repeat", 0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_ct_ = create_publisher<std_msgs::msg::Float64>("~/cross_track_error", 10);
    pub_gd_ = create_publisher<std_msgs::msg::Float64>("~/goal_distance", 10);

    diag_sub_ = create_subscription<prox_mpc_msgs::msg::SolverDiagnostics>(
      diag_topic, rclcpp::QoS(50),
      std::bind(&MetricsNode::onDiag, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(rate > 0.0 ? 1.0 / rate : 0.02);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MetricsNode::sample, this));

    RCLCPP_INFO(
      get_logger(),
      "metrics: scenario=%s model=%s mode=%s controller=%s repeat=%d; ref pts=%zu; goal=(%.2f,%.2f) tol=%.2f",
      label_scenario_.c_str(), label_model_.c_str(), label_mode_.c_str(),
      label_controller_.c_str(), label_repeat_, ref_x_.size(), goal_x_, goal_y_, goal_tol_);
  }

  // Compute the per-run summary and write it as JSON (called on shutdown).
  void writeSummary()
  {
    if (summary_json_.empty()) {return;}
    const double ct_rms = ct_count_ >
      0 ? std::sqrt(ct_sumsq_ / static_cast<double>(ct_count_)) : 0.0;
    const double mean_sqp =
      diag_count_ > 0 ? sqp_sum_ / static_cast<double>(diag_count_) : 0.0;
    const double mean_qp =
      diag_count_ > 0 ? qp_ext_sum_ / static_cast<double>(diag_count_) : 0.0;
    const double miss_rate =
      diag_count_ >
      0 ? static_cast<double>(deadline_miss_) / static_cast<double>(diag_count_) : 0.0;
    const double infeas_rate =
      diag_count_ > 0 ? static_cast<double>(infeasible_) / static_cast<double>(diag_count_) : 0.0;
    const double solve_mean =
      solve_ms_.empty() ? 0.0 :
      std::accumulate(solve_ms_.begin(), solve_ms_.end(), 0.0) /
      static_cast<double>(solve_ms_.size());

    std::ofstream os(summary_json_);
    if (!os) {
      RCLCPP_ERROR(get_logger(), "cannot open summary_json '%s'", summary_json_.c_str());
      return;
    }
    auto num = [](double v) -> std::string {
        if (!std::isfinite(v)) {return "null";}
        std::ostringstream ss;
        ss.precision(6);
        ss << std::fixed << v;
        return ss.str();
      };
    os << "{\n";
    os << "  \"scenario\": \"" << label_scenario_ << "\",\n";
    os << "  \"model\": \"" << label_model_ << "\",\n";
    os << "  \"mode\": \"" << label_mode_ << "\",\n";
    os << "  \"controller\": \"" << label_controller_ << "\",\n";
    os << "  \"repeat\": " << label_repeat_ << ",\n";
    os << "  \"success\": " << (reached_ ? "true" : "false") << ",\n";
    os << "  \"time_to_goal_s\": " << (reached_ ? num(time_to_goal_) : std::string("null")) <<
      ",\n";
    os << "  \"path_length_m\": " << num(path_length_) << ",\n";
    os << "  \"goal_error_m\": " << num(last_goal_dist_) << ",\n";
    os << "  \"min_goal_distance_m\": " << num(min_goal_dist_) << ",\n";
    os << "  \"cross_track_rms_m\": " << num(ct_rms) << ",\n";
    os << "  \"cross_track_max_m\": " << num(ct_max_) << ",\n";
    os << "  \"solve_ms_p50\": " << num(percentile(solve_ms_, 0.50)) << ",\n";
    os << "  \"solve_ms_p95\": " << num(percentile(solve_ms_, 0.95)) << ",\n";
    os << "  \"solve_ms_max\": " << num(solve_ms_.empty() ? std::nan("") :
    *std::max_element(solve_ms_.begin(), solve_ms_.end())) << ",\n";
    os << "  \"solve_ms_mean\": " << num(solve_mean) << ",\n";
    os << "  \"deadline_miss_rate\": " << num(miss_rate) << ",\n";
    os << "  \"mean_sqp_iters\": " << num(mean_sqp) << ",\n";
    os << "  \"mean_qp_iters_ext\": " << num(mean_qp) << ",\n";
    os << "  \"infeasible_rate\": " << num(infeas_rate) << ",\n";
    os << "  \"max_obstacle_slack_m\": " << num(slack_max_) << ",\n";
    os << "  \"recoveries\": " << recoveries_ << ",\n";
    os << "  \"num_pose_samples\": " << pose_samples_ << ",\n";
    os << "  \"num_diag_samples\": " << diag_count_ << "\n";
    os << "}\n";
    RCLCPP_INFO(
      get_logger(),
      "summary -> %s | success=%s ttg=%.2fs path=%.2fm goal_err=%.3fm ct_rms=%.3fm "
      "solve_p95=%.3fms miss=%.1f%% infeas=%.1f%%",
      summary_json_.c_str(), reached_ ? "true" : "false",
      reached_ ? time_to_goal_ : 0.0, path_length_, last_goal_dist_, ct_rms,
      percentile(solve_ms_, 0.95), 100.0 * miss_rate, 100.0 * infeas_rate);
  }

private:
  void onDiag(prox_mpc_msgs::msg::SolverDiagnostics::ConstSharedPtr msg)
  {
    diag_count_++;
    solve_ms_.push_back(msg->solve_time_ms);
    sqp_sum_ += static_cast<double>(msg->sqp_iters);
    qp_ext_sum_ += static_cast<double>(msg->qp_iters_ext);
    if (msg->deadline_missed) {deadline_miss_++;}
    if (msg->status != prox_mpc_msgs::msg::SolverDiagnostics::STATUS_SOLVED) {infeasible_++;}
    slack_max_ = std::max(slack_max_, msg->max_obstacle_slack);
  }

  void sample()
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return;  // pose not yet available
    }
    const double px = tf.transform.translation.x;
    const double py = tf.transform.translation.y;

    const rclcpp::Time stamp = now();
    if (!have_first_) {
      have_first_ = true;
      start_stamp_ = stamp;
      prev_x_ = px;
      prev_y_ = py;
    } else {
      path_length_ += std::hypot(px - prev_x_, py - prev_y_);
      prev_x_ = px;
      prev_y_ = py;
    }
    pose_samples_++;

    const double ct = crossTrack(ref_x_, ref_y_, px, py);
    ct_sumsq_ += ct * ct;
    ct_count_++;
    ct_max_ = std::max(ct_max_, ct);

    const double gd = std::hypot(px - goal_x_, py - goal_y_);
    last_goal_dist_ = gd;
    min_goal_dist_ = std::min(min_goal_dist_, gd);
    if (!reached_ && gd <= goal_tol_) {
      reached_ = true;
      reach_stamp_ = stamp;
      time_to_goal_ = (stamp - start_stamp_).seconds();
    }

    // Self-terminate a short settle after the goal so the orchestrator detects the
    // run finished (the summary is written by main once spin returns).
    if (reached_ && auto_exit_ && (stamp - reach_stamp_).seconds() >= settle_s_) {
      RCLCPP_INFO(get_logger(), "goal reached; shutting down to write summary.");
      rclcpp::shutdown();
      return;
    }

    std_msgs::msg::Float64 m;
    m.data = ct;
    pub_ct_->publish(m);
    m.data = gd;
    pub_gd_->publish(m);
  }

  std::string map_frame_, base_frame_, summary_json_;
  std::string label_scenario_, label_model_, label_mode_, label_controller_;
  int label_repeat_{0};
  std::vector<double> ref_x_, ref_y_;
  double goal_x_{0.0}, goal_y_{0.0}, goal_tol_{0.25};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_ct_, pub_gd_;
  rclcpp::Subscription<prox_mpc_msgs::msg::SolverDiagnostics>::SharedPtr diag_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Accuracy accumulators.
  bool have_first_{false};
  rclcpp::Time start_stamp_;
  double prev_x_{0.0}, prev_y_{0.0}, path_length_{0.0};
  double ct_sumsq_{0.0}, ct_max_{0.0};
  std::size_t ct_count_{0}, pose_samples_{0};
  bool reached_{false};
  bool auto_exit_{true};
  double settle_s_{1.0};
  rclcpp::Time reach_stamp_;
  double time_to_goal_{0.0};
  double last_goal_dist_{std::numeric_limits<double>::quiet_NaN()};
  double min_goal_dist_{std::numeric_limits<double>::infinity()};

  // Diagnostics accumulators.
  std::size_t diag_count_{0}, deadline_miss_{0}, infeasible_{0};
  double sqp_sum_{0.0}, qp_ext_sum_{0.0}, slack_max_{0.0};
  std::vector<double> solve_ms_;
  int recoveries_{0};  // not observable in standalone; bag-based path fills it for mode (a)
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MetricsNode>();
  rclcpp::spin(node);
  node->writeSummary();
  rclcpp::shutdown();
  return 0;
}
