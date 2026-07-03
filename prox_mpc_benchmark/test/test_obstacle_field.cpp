// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Unit tests for the shared obstacle-field helpers: the time-varying obstacle
// motion law, the ray/disc range used by the scan simulator, the robot/obstacle
// clearance used by the metrics node, and the parameter-array parser.

#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <prox_mpc_benchmark/obstacle_field.hpp>

using prox_mpc_benchmark::ObstacleSpec;
using prox_mpc_benchmark::obstacleCenterAt;
using prox_mpc_benchmark::parseObstacleSpecs;
using prox_mpc_benchmark::rayDiscRange;
using prox_mpc_benchmark::robotObstacleGap;

TEST(ObstacleCenter, StaticIsTimeInvariant)
{
  ObstacleSpec o;
  o.motion = "static";
  o.cx = 1.5;
  o.cy = -0.5;
  auto [x0, y0] = obstacleCenterAt(o, 0.0);
  auto [x1, y1] = obstacleCenterAt(o, 12.3);
  EXPECT_NEAR(x0, 1.5, 1e-12);
  EXPECT_NEAR(y0, -0.5, 1e-12);
  EXPECT_NEAR(x1, 1.5, 1e-12);
  EXPECT_NEAR(y1, -0.5, 1e-12);
}

TEST(ObstacleCenter, CircleOrbitsCentreAtOrbitRadius)
{
  ObstacleSpec o;
  o.motion = "circle";
  o.cx = 1.0;
  o.cy = 0.0;
  o.radius = 0.8;
  o.speed = 0.5;
  // At t=0 the phase is 0, so the obstacle sits at centre + (radius, 0).
  auto [x0, y0] = obstacleCenterAt(o, 0.0);
  EXPECT_NEAR(x0, 1.8, 1e-9);
  EXPECT_NEAR(y0, 0.0, 1e-9);
  // A quarter period (angle = pi/2) puts it at centre + (0, radius).
  const double t_quarter = (M_PI / 2.0) * o.radius / o.speed;
  auto [xq, yq] = obstacleCenterAt(o, t_quarter);
  EXPECT_NEAR(xq, 1.0, 1e-6);
  EXPECT_NEAR(yq, 0.8, 1e-6);
}

TEST(ObstacleCenter, LinePatrolReachesFarEndAtHalfPeriod)
{
  ObstacleSpec o;
  o.motion = "line";
  o.cx = 0.0;
  o.cy = -2.0;
  o.ex = 0.0;
  o.ey = 2.0;
  o.speed = 0.6;  // +speed starts at the "from" end
  auto [x0, y0] = obstacleCenterAt(o, 0.0);
  EXPECT_NEAR(x0, 0.0, 1e-9);
  EXPECT_NEAR(y0, -2.0, 1e-9);
  // Length 4 m at 0.6 m/s => reaches the far end after 4/0.6 s.
  auto [xh, yh] = obstacleCenterAt(o, 4.0 / 0.6);
  EXPECT_NEAR(xh, 0.0, 1e-6);
  EXPECT_NEAR(yh, 2.0, 1e-6);
}

TEST(ObstacleCenter, LineNegativeSpeedStartsAtFarEnd)
{
  ObstacleSpec o;
  o.motion = "line";
  o.cx = 0.0;
  o.cy = -2.0;
  o.ex = 0.0;
  o.ey = 2.0;
  o.speed = -0.6;  // -speed starts at the "to" end
  auto [x0, y0] = obstacleCenterAt(o, 0.0);
  EXPECT_NEAR(x0, 0.0, 1e-9);
  EXPECT_NEAR(y0, 2.0, 1e-9);
}

TEST(RayDisc, HitsDiscAheadAtNearSurface)
{
  // Disc centred 2 m ahead (+x), radius 0.3: the near surface is at 1.7 m.
  const double r = rayDiscRange(0.0, 0.0, 0.0, 2.0, 0.0, 0.3, 0.05, 4.0);
  ASSERT_TRUE(std::isfinite(r));
  EXPECT_NEAR(r, 1.7, 1e-9);
}

TEST(RayDisc, MissesWhenPointedAway)
{
  // Same disc ahead but the ray points backward (-x): no hit.
  const double r = rayDiscRange(0.0, 0.0, M_PI, 2.0, 0.0, 0.3, 0.05, 4.0);
  EXPECT_TRUE(std::isnan(r));
}

TEST(RayDisc, MissesBeyondMaxRange)
{
  // Disc near surface at 4.7 m, beyond the 4 m max range: no valid return.
  const double r = rayDiscRange(0.0, 0.0, 0.0, 5.0, 0.0, 0.3, 0.05, 4.0);
  EXPECT_TRUE(std::isnan(r));
}

TEST(RayObstacleGap, PositiveWhenClearNegativeWhenOverlapping)
{
  ObstacleSpec o;
  o.motion = "static";
  o.cx = 1.0;
  o.cy = 0.0;
  o.body = 0.28;
  // Robot centre 1 m away: gap = 1.0 - 0.28(body) - 0.22(robot) = 0.50.
  EXPECT_NEAR(robotObstacleGap(o, 0.0, 0.0, 0.0, 0.22), 0.50, 1e-9);
  // Robot centre 0.3 m away: discs overlap, gap is negative (a collision).
  EXPECT_LT(robotObstacleGap(o, 0.0, 0.7, 0.0, 0.22), 0.0);
}

TEST(ParseSpecs, PadsShortArraysAndKeepsMotion)
{
  const std::vector<std::string> motion{"circle", "static"};
  const std::vector<double> cx{1.0, 2.0};
  const std::vector<double> cy{0.1, 0.2};
  const std::vector<double> empty{};
  const std::vector<double> body{0.28};  // shorter than motion -> padded
  const auto specs = parseObstacleSpecs(
    motion, cx, cy, empty, empty, empty, empty, body);
  ASSERT_EQ(specs.size(), 2u);
  EXPECT_EQ(specs[0].motion, "circle");
  EXPECT_NEAR(specs[0].cx, 1.0, 1e-12);
  EXPECT_NEAR(specs[0].body, 0.28, 1e-12);
  EXPECT_EQ(specs[1].motion, "static");
  EXPECT_NEAR(specs[1].body, 0.25, 1e-12);  // default pad
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
