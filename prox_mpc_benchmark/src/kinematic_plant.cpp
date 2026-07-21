// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Kinematic plant for mode (b2): Nav2 controller comparison without Gazebo.
//
// It integrates the body twist a Nav2 controller publishes on /cmd_vel as a
// unicycle (x, y, theta) and publishes the resulting pose as /odom plus the
// odom -> base_link transform. A separate static map -> odom identity makes the
// integrated pose the ground truth in the map frame, so the metrics node reads an
// exact pose with no localization noise - every controller under test drives the
// same deterministic plant (mode b2). Because the command is a body twist
// (v forward, omega yaw), a unicycle integrator is the controller-agnostic plant:
// diff-drive (DWB / MPPI / RPP / ProxMPC-Unicycle) all map 1:1 onto it.
//
// On command staleness (no /cmd_vel within cmd_timeout) the plant holds zero, so
// it parks at the goal once the controller stops and never coasts on a lost link.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

class KinematicPlant : public rclcpp::Node
{
public:
  KinematicPlant()
  : Node("kinematic_plant")
  {
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    const std::string cmd_topic = declare_parameter<std::string>("cmd_topic", "cmd_vel");
    const std::string odom_topic = declare_parameter<std::string>("odom_topic", "odom");
    rate_hz_ = declare_parameter<double>("rate_hz", 50.0);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.5);

    // Initial pose in the odom frame. With a static map -> odom identity this is
    // also the start pose in the map frame, matching the scenario start.
    x_ = declare_parameter<double>("start_x", 0.0);
    y_ = declare_parameter<double>("start_y", 0.0);
    theta_ = declare_parameter<double>("start_theta", 0.0);

    // Defensive bound on an out-of-range command (a misconfigured controller must
    // not be able to fling the plant); generous so it never clips a sane command.
    v_max_ = declare_parameter<double>("v_max", 2.0);
    w_max_ = declare_parameter<double>("w_max", 4.0);
    if (rate_hz_ <= 0.0) {rate_hz_ = 50.0;}

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, rclcpp::QoS(20));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic, rclcpp::QoS(10),
      std::bind(&KinematicPlant::onCmd, this, std::placeholders::_1));

    last_cmd_time_ = now();
    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&KinematicPlant::step, this));

    RCLCPP_INFO(
      get_logger(),
      "kinematic_plant: start=(%.2f, %.2f, %.2f) rate=%.1f Hz cmd='%s' odom='%s' frames=%s->%s",
      x_, y_, theta_, rate_hz_, cmd_topic.c_str(), odom_topic.c_str(),
      odom_frame_.c_str(), base_frame_.c_str());
  }

private:
  void onCmd(geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    cmd_v_ = std::isfinite(msg->linear.x) ? std::clamp(msg->linear.x, -v_max_, v_max_) : 0.0;
    cmd_w_ = std::isfinite(msg->angular.z) ? std::clamp(msg->angular.z, -w_max_, w_max_) : 0.0;
    last_cmd_time_ = now();
  }

  void step()
  {
    const rclcpp::Time stamp = now();
    const double dt = 1.0 / rate_hz_;

    // Hold zero on a stale command so the plant parks instead of coasting.
    double v = cmd_v_;
    double w = cmd_w_;
    if ((stamp - last_cmd_time_).seconds() > cmd_timeout_) {
      v = 0.0;
      w = 0.0;
    }

    // Exact unicycle integration over the step for a constant (v, w).
    constexpr double kStraightOmega = 1e-6;
    if (std::abs(w) < kStraightOmega) {
      x_ += v * std::cos(theta_) * dt;
      y_ += v * std::sin(theta_) * dt;
    } else {
      const double th1 = theta_ + w * dt;
      x_ += (v / w) * (std::sin(th1) - std::sin(theta_));
      y_ -= (v / w) * (std::cos(th1) - std::cos(theta_));
      theta_ = th1;
    }
    theta_ = std::remainder(theta_, 2.0 * M_PI);

    const double qz = std::sin(theta_ / 2.0);
    const double qw = std::cos(theta_ / 2.0);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.rotation.z = qz;
    tf.transform.rotation.w = qw;
    tf_broadcaster_->sendTransform(tf);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.z = qz;
    odom.pose.pose.orientation.w = qw;
    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = w;
    odom_pub_->publish(odom);
  }

  std::string odom_frame_, base_frame_;
  double rate_hz_{50.0}, cmd_timeout_{0.5};
  double x_{0.0}, y_{0.0}, theta_{0.0};
  double v_max_{2.0}, w_max_{4.0};
  double cmd_v_{0.0}, cmd_w_{0.0};
  rclcpp::Time last_cmd_time_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicPlant>());
  rclcpp::shutdown();
  return 0;
}
