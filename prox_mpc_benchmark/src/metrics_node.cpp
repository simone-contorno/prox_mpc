// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Controller-agnostic live benchmark metrics node.
//
// It measures the accuracy class (cross-track error vs a reference polyline and
// goal error) from generic signals - the robot pose (TF map -> base_link, or a
// bridged ground-truth pose in Gazebo) plus the scenario reference - so every
// controller under test is measured identically. It also taps the
// SolverDiagnostics stream for the real-time / feasibility class. Live cross-track
// and goal-distance are published as std_msgs/Float64 for inspection/recording,
// and a per-run summary JSON is written at shutdown (summary_json param) so the
// orchestrator can aggregate repeats without re-parsing a bag.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <future>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <prox_mpc_msgs/msg/solver_diagnostics.hpp>

#include <prox_mpc_benchmark/metrics_math.hpp>
#include <prox_mpc_benchmark/obstacle_field.hpp>

using prox_mpc_benchmark::crossTrack;
using prox_mpc_benchmark::ObstacleSpec;
using prox_mpc_benchmark::obstacleCenterAt;
using prox_mpc_benchmark::parseObstacleSpecs;
using prox_mpc_benchmark::percentile;
using prox_mpc_benchmark::robotObstacleGap;

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

    // Obstacle field (parallel arrays, same source as the scan simulator) for the
    // controller-agnostic clearance metric. Empty on the obstacle-free cell.
    robot_radius_ = declare_parameter<double>("robot_radius", 0.22);
    motion_eps_ = declare_parameter<double>("motion_eps", 1.0e-3);
    obstacles_ = parseObstacleSpecs(
      declare_parameter<std::vector<std::string>>("obs_motion", std::vector<std::string>{}),
      declare_parameter<std::vector<double>>("obs_cx", std::vector<double>{}),
      declare_parameter<std::vector<double>>("obs_cy", std::vector<double>{}),
      declare_parameter<std::vector<double>>("obs_ex", std::vector<double>{}),
      declare_parameter<std::vector<double>>("obs_ey", std::vector<double>{}),
      declare_parameter<std::vector<double>>("obs_radius", std::vector<double>{}),
      declare_parameter<std::vector<double>>("obs_speed", std::vector<double>{}),
      declare_parameter<std::vector<double>>("obs_body", std::vector<double>{}));

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

    // Obstacle clock anchor: latch on the same /odom stream and stamps the scan
    // simulator latches on, so the scored obstacle trajectory is phase-identical
    // to the one ray-cast into /scan. The TF-displacement latch in sample() is
    // kept only as a fallback for setups without this odom topic; the two clocks
    // can drift by seconds in both directions under load, which mis-scores
    // collisions.
    const std::string odom_topic = declare_parameter<std::string>("odom_topic", "odom");
    if (!obstacles_.empty()) {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::QoS(50),
        std::bind(&MetricsNode::onOdom, this, std::placeholders::_1));
    }

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
  /* Signalled once the run has finished, so main can stop the executor instead
   * of tearing the context down from inside a callback. */
  std::shared_future<void> finished() const {return finished_future_;}

  void writeSummary()
  {
    if (nonfinite_solve_ > 0) {
      RCLCPP_WARN(
        get_logger(),
        "%zu of %zu solve_time_ms samples were non-finite and are excluded from "
        "solve_ms_p50/p95/max/mean",
        nonfinite_solve_, diag_count_);
    }
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
    // Controller-agnostic avoidance metrics (null on the obstacle-free cell). The
    // gap is robot-disc to obstacle-disc; < 0 means the discs overlapped, which on
    // the collision-free kinematic plant is a would-be collision.
    const bool has_obs = !obstacles_.empty() && std::isfinite(min_obs_gap_);
    os << "  \"min_obstacle_gap_m\": " <<
      (has_obs ? num(min_obs_gap_) : std::string("null")) << ",\n";
    os << "  \"min_obstacle_dist_m\": " <<
      (has_obs ? num(min_obs_dist_) : std::string("null")) << ",\n";
    os << "  \"collision\": " <<
      (has_obs ? (min_obs_gap_ < 0.0 ? "true" : "false") : std::string("null")) << ",\n";
    os << "  \"solve_ms_p50\": " << num(percentile(solve_ms_, 0.50)) << ",\n";
    os << "  \"solve_ms_p95\": " << num(percentile(solve_ms_, 0.95)) << ",\n";
    os << "  \"solve_ms_max\": " << num(solve_ms_.empty() ? std::nan("") :
    *std::max_element(solve_ms_.begin(), solve_ms_.end())) << ",\n";
    os << "  \"solve_ms_mean\": " << num(solve_mean) << ",\n";
    os << "  \"deadline_miss_rate\": " << num(miss_rate) << ",\n";
    os << "  \"mean_sqp_iters\": " << num(mean_sqp) << ",\n";
    os << "  \"mean_qp_iters_ext\": " << num(mean_qp) << ",\n";
    os << "  \"infeasible_rate\": " << num(infeas_rate) << ",\n";
    os << "  \"max_sqp_iters\": " << sqp_max_ << ",\n";
    os << "  \"max_qp_iters_ext\": " << qp_ext_max_ << ",\n";
    os << "  \"status_counts\": {";
    {
      bool first = true;
      for (const auto & [code, count] : status_counts_) {
        if (!first) {os << ", ";}
        os << "\"" << static_cast<int>(code) << "\": " << count;
        first = false;
      }
    }
    os << "},\n";
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
  void onOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    const double px = msg->pose.pose.position.x;
    const double py = msg->pose.pose.position.y;
    odom_stamp_ = msg->header.stamp;
    if (!have_odom_) {
      have_odom_ = true;
      odom_first_x_ = px;
      odom_first_y_ = py;
    } else if (!odom_moved_ && std::hypot(px - odom_first_x_, py - odom_first_y_) > motion_eps_) {
      // Mirrors scan_simulator::onOdom: same stream, same eps, same stamp source,
      // so both latches fire on the same message.
      odom_moved_ = true;
      odom_t0_ = odom_stamp_;
    }
  }

  void onDiag(prox_mpc_msgs::msg::SolverDiagnostics::ConstSharedPtr msg)
  {
    diag_count_++;
    // A non-finite solve time breaks the strict weak ordering the percentile's
    // sort requires, so it is rejected here rather than inside percentile(),
    // leaving the quantile definition untouched. Rejects are counted and
    // reported so a truncated timing distribution is visible, not silent.
    if (std::isfinite(msg->solve_time_ms)) {
      solve_ms_.push_back(msg->solve_time_ms);
    } else {
      nonfinite_solve_++;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), kNonFiniteWarnPeriodMs,
        "dropped %zu non-finite solve_time_ms sample(s) from the timing distribution",
        nonfinite_solve_);
    }
    sqp_sum_ += static_cast<double>(msg->sqp_iters);
    qp_ext_sum_ += static_cast<double>(msg->qp_iters_ext);
    /* Retain the per-cycle maxima and the status histogram, not only the means:
     * a single cycle that exhausts the QP iteration budget and then succeeds on
     * an SQP retry is averaged away by the means and reported as converged, so
     * the worst latency event in a run leaves no trace in infeasible_rate. */
    sqp_max_ = std::max(sqp_max_, static_cast<std::size_t>(msg->sqp_iters));
    qp_ext_max_ = std::max(qp_ext_max_, static_cast<std::size_t>(msg->qp_iters_ext));
    status_counts_[msg->status]++;
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
      first_x_ = px;
      first_y_ = py;
    } else {
      path_length_ += std::hypot(px - prev_x_, py - prev_y_);
      prev_x_ = px;
      prev_y_ = py;
    }
    pose_samples_++;

    // Clearance to the (time-varying) obstacle field, evaluated on the scan
    // simulator's own clock (odom-anchored latch above) so the scored obstacle is
    // phase-identical to the one the controller actually faced. The TF-based
    // latch below only serves setups where no odom message ever arrives.
    if (!obstacles_.empty()) {
      if (!obs_started_ && std::hypot(px - first_x_, py - first_y_) > motion_eps_) {
        obs_started_ = true;
        obs_t0_ = stamp;
      }
      double t = 0.0;
      if (have_odom_) {
        t = odom_moved_ ? (odom_stamp_ - odom_t0_).seconds() : 0.0;
      } else if (obs_started_) {
        t = (stamp - obs_t0_).seconds();
      }
      for (const auto & o : obstacles_) {
        const double gap = robotObstacleGap(o, t, px, py, robot_radius_);
        min_obs_gap_ = std::min(min_obs_gap_, gap);
        const auto [ox, oy] = obstacleCenterAt(o, t);
        min_obs_dist_ = std::min(min_obs_dist_, std::hypot(px - ox, py - oy));
      }
    }

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
      RCLCPP_INFO(get_logger(), "goal reached; stopping to write summary.");
      // The promise is one-shot; later callbacks may still arrive before the
      // executor unwinds.
      if (!finished_signalled_) {
        finished_signalled_ = true;
        finished_.set_value();
      }
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
  std::vector<ObstacleSpec> obstacles_;
  double robot_radius_{0.22}, motion_eps_{1.0e-3};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_ct_, pub_gd_;
  rclcpp::Subscription<prox_mpc_msgs::msg::SolverDiagnostics>::SharedPtr diag_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Accuracy accumulators.
  bool have_first_{false};
  rclcpp::Time start_stamp_;
  double prev_x_{0.0}, prev_y_{0.0}, path_length_{0.0};
  double first_x_{0.0}, first_y_{0.0};
  bool obs_started_{false};
  rclcpp::Time obs_t0_;
  // Odom-anchored obstacle clock (authoritative when the odom topic exists).
  bool have_odom_{false};
  bool odom_moved_{false};
  double odom_first_x_{0.0}, odom_first_y_{0.0};
  rclcpp::Time odom_t0_;
  rclcpp::Time odom_stamp_;
  double min_obs_gap_{std::numeric_limits<double>::infinity()};
  double min_obs_dist_{std::numeric_limits<double>::infinity()};
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
  std::size_t sqp_max_{0}, qp_ext_max_{0};
  std::map<std::uint8_t, std::size_t> status_counts_;
  double sqp_sum_{0.0}, qp_ext_sum_{0.0}, slack_max_{0.0};
  std::vector<double> solve_ms_;
  std::size_t nonfinite_solve_{0};
  static constexpr int kNonFiniteWarnPeriodMs = 5000;
  int recoveries_{0};  // not observable in standalone; bag-based path fills it for mode (a)

  std::promise<void> finished_;
  std::shared_future<void> finished_future_{finished_.get_future().share()};
  bool finished_signalled_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MetricsNode>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  // Returns when the node signals completion, or when an external SIGINT
  // invalidates the context; the summary is written either way.
  exec.spin_until_future_complete(node->finished());
  node->writeSummary();
  rclcpp::shutdown();
  return 0;
}
