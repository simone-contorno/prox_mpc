// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0

#ifndef PROX_MPC__UTILS_HPP_
#define PROX_MPC__UTILS_HPP_

// ROS 2 C++
#include <rclcpp/time.hpp>

// Messages
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

// Other libraries
#include <cmath>
#include <cstdlib>
#include <memory>
#include <tuple>
#include <Eigen/Dense>
#include <Eigen/Sparse>

// The Eigen and Path aliases are kept at global scope so the prox_mpc code below
// resolves them unqualified, and so do downstream consumers (controller, demo).

// Eigen
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Types
using nav_msgs::msg::Path;

namespace prox_mpc
{

void normalizeAngle(double & angle);
Path optimPath(const MatrixXd & x, const rclcpp::Time & now);

}  // namespace prox_mpc

#endif  // PROX_MPC__UTILS_HPP_
