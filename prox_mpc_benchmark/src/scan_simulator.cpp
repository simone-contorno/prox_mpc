// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Obstacle scan simulator for mode (b2): Nav2 controller comparison without Gazebo.
//
// The kinematic plant integrates the command but has no sensor, so in a laser-free
// b2 world the costmap only carries the static map. This node closes that gap: it
// ray-casts the scenario obstacles (static box / moving circle / patrolling line,
// with the identical motion law as mode b1) from the robot pose and publishes a
// 2D LaserScan on /scan, which a costmap obstacle_layer marks and clears. Every
// controller under test (DWB / MPPI / RPP / Graceful / Vector Pursuit, and
// ProxMPC's costmap fill) therefore perceives the same obstacle through the
// same costmap and inflation, so the obstacle-avoidance comparison is fair by
// construction - no controller gets privileged ground-truth obstacle knowledge.
//
// The virtual lidar is mounted at the body origin (scan frame defaults to
// base_link), so ranges are measured from the robot centre. Obstacles are sensed
// as discs of their physical radius (obs_body), which the runner derives so the
// shared robot radius plus costmap inflation reproduces the scenario keep-out.
//
// The obstacle clock starts when the robot first moves (pose displacement past
// motion_eps), the same event the metrics node uses to time the clearance metric,
// so a dynamic obstacle's phase aligns with the trajectory the controller drove.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <prox_mpc_benchmark/obstacle_field.hpp>

namespace prox_mpc_benchmark
{

class ScanSimulator : public rclcpp::Node
{
public:
  ScanSimulator()
  : Node("scan_simulator")
  {
    scan_frame_ = declare_parameter<std::string>("scan_frame", "base_link");
    const std::string odom_topic = declare_parameter<std::string>("odom_topic", "odom");
    const std::string scan_topic = declare_parameter<std::string>("scan_topic", "scan");
    rate_hz_ = declare_parameter<double>("rate_hz", 10.0);
    num_beams_ = declare_parameter<int>("num_beams", 360);
    range_min_ = declare_parameter<double>("range_min", 0.05);
    range_max_ = declare_parameter<double>("range_max", 4.0);
    motion_eps_ = declare_parameter<double>("motion_eps", 1.0e-3);
    // Optional Gaussian range noise [m std]; 0.0 = ideal sensor (unchanged
    // behaviour). Only returns that strike an obstacle are perturbed, so
    // clearing beams never fabricate a phantom near-return. Seeded so a run is
    // reproducible; the runner varies the seed per repeat to sample noise.
    range_noise_std_ = declare_parameter<double>("range_noise_std", 0.0);
    const int noise_seed = declare_parameter<int>("range_noise_seed", 0);
    if (range_noise_std_ < 0.0) {range_noise_std_ = 0.0;}
    rng_.seed(static_cast<std::uint_fast32_t>(noise_seed));
    if (rate_hz_ <= 0.0) {rate_hz_ = 10.0;}
    if (num_beams_ < 8) {num_beams_ = 8;}

    obstacles_ = parseObstacleSpecs(
      declare_parameter<std::vector<std::string>>("obs_motion", std::vector<std::string>{}),
      declare_parameter<std::vector<double>>("obs_cx", std::vector<double>{}),
      declare_parameter<std::vector<double>>("obs_cy", std::vector<double>{}),
      declare_parameter<std::vector<double>>("obs_ex", std::vector<double>{}),
      declare_parameter<std::vector<double>>("obs_ey", std::vector<double>{}),
      declare_parameter<std::vector<double>>("obs_radius", std::vector<double>{}),
      declare_parameter<std::vector<double>>("obs_speed", std::vector<double>{}),
      declare_parameter<std::vector<double>>("obs_body", std::vector<double>{}));

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS());
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(20),
      std::bind(&ScanSimulator::onOdom, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ScanSimulator::publishScan, this));

    RCLCPP_INFO(
      get_logger(),
      "scan_simulator: %zu obstacle(s), %d beams, range [%.2f, %.2f] m @ %.1f Hz, "
      "noise_std=%.3f m, frame=%s",
      obstacles_.size(), num_beams_, range_min_, range_max_, rate_hz_,
      range_noise_std_, scan_frame_.c_str());
  }

private:
  void onOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    px_ = msg->pose.pose.position.x;
    py_ = msg->pose.pose.position.y;
    const double qz = msg->pose.pose.orientation.z;
    const double qw = msg->pose.pose.orientation.w;
    theta_ = std::atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz);
    pose_stamp_ = msg->header.stamp;
    if (!have_pose_) {
      have_pose_ = true;
      first_x_ = px_;
      first_y_ = py_;
    } else if (!moved_ && std::hypot(px_ - first_x_, py_ - first_y_) > motion_eps_) {
      // Latch the obstacle clock to first motion so a dynamic obstacle's phase
      // matches the trajectory this controller actually drove.
      moved_ = true;
      t0_ = pose_stamp_;
    }
  }

  void publishScan()
  {
    if (!have_pose_) {return;}  // wait for the plant's first odom

    // Stamp the scan with the pose's odom time so the costmap's TF lookup matches
    // the pose the ranges were cast from.
    const rclcpp::Time stamp = pose_stamp_;
    const double t = moved_ ? (stamp - t0_).seconds() : 0.0;

    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = stamp;
    scan.header.frame_id = scan_frame_;
    scan.angle_min = static_cast<float>(-M_PI);
    scan.angle_max = static_cast<float>(M_PI);
    scan.angle_increment = static_cast<float>(2.0 * M_PI / static_cast<double>(num_beams_));
    scan.range_min = static_cast<float>(range_min_);
    scan.range_max = static_cast<float>(range_max_);
    // Clear beams are published as +inf, not range_max: the costmap's laser
    // projection drops readings at range_max, so a finite range_max beam yields no
    // clearing ray and a moving obstacle's vacated cells never clear (they pile up
    // into a trail). +inf, with the obstacle_layer's inf_is_valid, is treated as a
    // max-range clearing ray, so empty directions are swept free every scan.
    const float clear = std::numeric_limits<float>::infinity();
    scan.ranges.assign(static_cast<std::size_t>(num_beams_), clear);

    if (obstacles_.empty()) {
      scan_pub_->publish(scan);  // empty world: all-clear (inf) sweeping beams
      return;
    }

    std::vector<std::pair<double, double>> centres;
    centres.reserve(obstacles_.size());
    for (const auto & o : obstacles_) {centres.push_back(obstacleCenterAt(o, t));}

    for (int i = 0; i < num_beams_; ++i) {
      const double ang = theta_ + (-M_PI + static_cast<double>(i) * (2.0 * M_PI / num_beams_));
      double best = range_max_;
      for (std::size_t k = 0; k < obstacles_.size(); ++k) {
        const double r = rayDiscRange(
          px_, py_, ang, centres[k].first, centres[k].second, obstacles_[k].body,
          range_min_, range_max_);
        if (std::isfinite(r) && r < best) {best = r;}
      }
      // Perturb only actual obstacle returns; clamp to the valid range window.
      if (range_noise_std_ > 0.0 && best < range_max_) {
        std::normal_distribution<double> noise(0.0, range_noise_std_);
        best = std::clamp(best + noise(rng_), range_min_, range_max_);
      }
      // A real hit is finite and marks; otherwise leave the beam at +inf so it
      // clears the ray instead of marking a phantom obstacle at range_max.
      if (best < range_max_) {
        scan.ranges[static_cast<std::size_t>(i)] = static_cast<float>(best);
      }
    }
    scan_pub_->publish(scan);
  }

  std::string scan_frame_;
  double rate_hz_{10.0};
  int num_beams_{360};
  double range_min_{0.05}, range_max_{4.0}, motion_eps_{1.0e-3};
  double range_noise_std_{0.0};
  std::mt19937 rng_;
  std::vector<ObstacleSpec> obstacles_;

  bool have_pose_{false}, moved_{false};
  double px_{0.0}, py_{0.0}, theta_{0.0}, first_x_{0.0}, first_y_{0.0};
  rclcpp::Time t0_, pose_stamp_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace prox_mpc_benchmark

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<prox_mpc_benchmark::ScanSimulator>());
  rclcpp::shutdown();
  return 0;
}
