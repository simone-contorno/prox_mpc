// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Unit tests for the small ROS-facing helpers in utils.cpp (optimPath and
// normalizeAngle). optimPath is otherwise only driven by the demo node, so it is
// exercised here directly.

#include <cmath>

#include <gtest/gtest.h>

#include <rclcpp/time.hpp>

#include <prox_mpc/utils.hpp>

using prox_mpc::normalizeAngle;
using prox_mpc::optimPath;

// optimPath maps each predicted state row to a PoseStamped (x, y in the map
// frame), stamps every pose with `now`, and leaves a unit (identity) orientation.
TEST(Utils, OptimPathMapsStatesToPath)
{
  MatrixXd x(3, 4);
  x << 0.0, 0.0, 0.0, 0.0,
    1.0, 2.0, 0.0, 0.0,
    3.0, 4.0, 0.0, 0.0;
  const rclcpp::Time now(123, 456, RCL_ROS_TIME);

  const Path path = optimPath(x, now);

  ASSERT_EQ(path.poses.size(), 3u);
  EXPECT_EQ(path.header.frame_id, "map");
  EXPECT_EQ(path.poses[2].header.frame_id, "map");
  EXPECT_DOUBLE_EQ(path.poses[1].pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(path.poses[1].pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(path.poses[2].pose.position.x, 3.0);
  EXPECT_DOUBLE_EQ(path.poses[0].pose.orientation.w, 1.0);
  EXPECT_EQ(path.poses[0].header.stamp.sec, 123);
  EXPECT_EQ(path.poses[0].header.stamp.nanosec, 456u);
}

// normalizeAngle wraps any angle into [-pi, pi].
TEST(Utils, NormalizeAngleWrapsIntoRange)
{
  double a = 3.0 * M_PI;            // odd multiple of pi -> +/-pi
  normalizeAngle(a);
  EXPECT_NEAR(std::abs(a), M_PI, 1e-9);

  double b = -3.0 * M_PI / 2.0;     // -> +pi/2
  normalizeAngle(b);
  EXPECT_NEAR(b, M_PI / 2.0, 1e-9);

  double c = 0.5;                   // already in range -> unchanged
  normalizeAngle(c);
  EXPECT_NEAR(c, 0.5, 1e-9);
}
