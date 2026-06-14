// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0

#ifndef PROX_MPC__UTILS_HPP_
#define PROX_MPC__UTILS_HPP_

// ROS 2 C++
#include <rclcpp/rclcpp.hpp>

// Messages
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>

// Other libraries
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <tuple>
#include <Eigen/Dense>
#include <Eigen/Sparse>

// Eigen and ROS aliases are kept at global scope so the prox_mpc code below
// resolves them unqualified, and so do downstream consumers (controller, demo).

// Eigen
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Types
using nav_msgs::msg::Path;
using SetParametersResult = rcl_interfaces::msg::SetParametersResult_<std::allocator<void>>;
using PubFloat64 = std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64, std::allocator<void>>>;
using PubTwist = std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist,
    std::allocator<void>>>;
using PubFloat32MultiArray =
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32MultiArray, std::allocator<void>>>;

namespace prox_mpc
{

void normalizeAngle(double & angle);
Path optimPath(const MatrixXd & x, const rclcpp::Time & now);

}  // namespace prox_mpc

#endif  // PROX_MPC__UTILS_HPP_
