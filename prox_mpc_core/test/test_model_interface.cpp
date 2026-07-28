// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Tests for the Model interface using the bundled models: name, dimensions,
// declared bounds, the analytic forward-Euler residual and Jacobians, and the
// configure() and toTwist() hooks.

#include <cmath>
#include <map>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include <prox_mpc/model.hpp>
#include <prox_mpc/models/bicycle.hpp>
#include <prox_mpc/models/unicycle.hpp>

using prox_mpc::Bicycle;
using prox_mpc::Unicycle;
using prox_mpc::Model;

namespace
{
constexpr double kTol = 1e-12;

// Read a declared [low, upp] bound for a vector index from a model's ineq map.
void getBound(Model & model, const std::string & var, size_t idx, double & low, double & upp)
{
  bool found = false;
  for (const auto & entry : model.getIneq(var)) {
    if (static_cast<size_t>(entry.second[0]) == idx) {
      low = entry.second[1];
      upp = entry.second[2];
      found = true;
    }
  }
  ASSERT_TRUE(found) << "bound not declared: " << var << "[" << idx << "]";
}
}  // namespace

TEST(ModelInterface, BicycleIdentityAndBounds)
{
  Bicycle model;
  EXPECT_EQ(model.getName(), "bicycle");
  EXPECT_EQ(model.getN(), 4u);
  EXPECT_EQ(model.getM(), 2u);
  EXPECT_NEAR(model.getParams()(0), 1.6, kTol);  // wheelbase L

  double low = 0.0;
  double upp = 0.0;
  getBound(model, "x", 3, low, upp);
  EXPECT_NEAR(low, -M_PI / 2, kTol);
  EXPECT_NEAR(upp, M_PI / 2, kTol);
  getBound(model, "u", 0, low, upp);
  EXPECT_NEAR(low, -3.0, kTol);
  EXPECT_NEAR(upp, 3.0, kTol);
  getBound(model, "u", 1, low, upp);
  EXPECT_NEAR(low, -1.0, kTol);
  EXPECT_NEAR(upp, 1.0, kTol);
  getBound(model, "du", 0, low, upp);
  EXPECT_NEAR(low, -0.5, kTol);
  EXPECT_NEAR(upp, 0.5, kTol);
  getBound(model, "du", 1, low, upp);
  EXPECT_NEAR(low, -0.5, kTol);
  EXPECT_NEAR(upp, 0.5, kTol);
}

TEST(ModelInterface, UnicycleIdentityAndBounds)
{
  Unicycle model;
  EXPECT_EQ(model.getName(), "unicycle");
  EXPECT_EQ(model.getN(), 3u);
  EXPECT_EQ(model.getM(), 2u);

  double low = 0.0;
  double upp = 0.0;
  getBound(model, "u", 0, low, upp);
  EXPECT_NEAR(low, -3.0, kTol);
  EXPECT_NEAR(upp, 3.0, kTol);
  getBound(model, "u", 1, low, upp);
  EXPECT_NEAR(low, -1.0, kTol);
  EXPECT_NEAR(upp, 1.0, kTol);
  getBound(model, "du", 0, low, upp);
  EXPECT_NEAR(low, -0.5, kTol);
  EXPECT_NEAR(upp, 0.5, kTol);
  getBound(model, "du", 1, low, upp);
  EXPECT_NEAR(low, -0.5, kTol);
  EXPECT_NEAR(upp, 0.5, kTol);
}

TEST(ModelInterface, BicycleEulerResidualAndJacobians)
{
  Bicycle model;
  const double dt = 0.1;
  const double L = 1.6;

  VectorXd x(4);
  x << 1.0, 2.0, 0.3, 0.1;          // [x, y, theta, delta]
  VectorXd u(2);
  u << 0.7, 0.2;                    // [v, delta_dot]
  VectorXd x_next(4);
  x_next << 1.05, 2.07, 0.34, 0.12;

  model.setX(x);
  model.setU(u);
  model.updatec(dt, x_next);
  model.updateA(dt);
  model.updateB();

  const double v = u(0);
  const double theta = x(2);
  const double delta = x(3);

  // Residual c = x_k - x_{k+1} + dt * f(x_k, u_k).
  VectorXd c_exp(4);
  c_exp << x(0) - x_next(0) + dt * v * cos(theta + delta),
    x(1) - x_next(1) + dt * v * sin(theta + delta),
    x(2) - x_next(2) + dt * v * sin(delta) / L,
    x(3) - x_next(3) + dt * u(1);
  for (int i = 0; i < 4; i++) {
    EXPECT_NEAR(model.getc()(i), c_exp(i), kTol);
  }

  // A = d(residual)/d x_k.
  MatrixXd a_exp(4, 4);
  a_exp << 1.0, 0.0, -dt * v * sin(theta + delta), -dt * v * sin(theta + delta),
    0.0, 1.0, dt * v * cos(theta + delta), dt * v * cos(theta + delta),
    0.0, 0.0, 1.0, dt * v * cos(delta) / L,
    0.0, 0.0, 0.0, 1.0;
  for (int r = 0; r < 4; r++) {
    for (int col = 0; col < 4; col++) {
      EXPECT_NEAR(model.getA()(r, col), a_exp(r, col), kTol);
    }
  }

  // B = d f / d u_k.
  MatrixXd b_exp(4, 2);
  b_exp << cos(theta + delta), 0.0,
    sin(theta + delta), 0.0,
    sin(delta) / L, 0.0,
    0.0, 1.0;
  for (int r = 0; r < 4; r++) {
    for (int col = 0; col < 2; col++) {
      EXPECT_NEAR(model.getB()(r, col), b_exp(r, col), kTol);
    }
  }
}

TEST(ModelInterface, UnicycleEulerResidualAndJacobians)
{
  Unicycle model;
  const double dt = 0.1;

  VectorXd x(3);
  x << -0.5, 1.0, 0.7;             // [x, y, theta]
  VectorXd u(2);
  u << 1.2, -0.4;                  // [v, omega]
  VectorXd x_next(3);
  x_next << -0.4, 1.08, 0.66;

  model.setX(x);
  model.setU(u);
  model.updatec(dt, x_next);
  model.updateA(dt);
  model.updateB();

  const double v = u(0);
  const double theta = x(2);

  VectorXd c_exp(3);
  c_exp << x(0) - x_next(0) + dt * v * cos(theta),
    x(1) - x_next(1) + dt * v * sin(theta),
    x(2) - x_next(2) + dt * u(1);
  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(model.getc()(i), c_exp(i), kTol);
  }

  MatrixXd a_exp(3, 3);
  a_exp << 1.0, 0.0, -dt * v * sin(theta),
    0.0, 1.0, dt * v * cos(theta),
    0.0, 0.0, 1.0;
  for (int r = 0; r < 3; r++) {
    for (int col = 0; col < 3; col++) {
      EXPECT_NEAR(model.getA()(r, col), a_exp(r, col), kTol);
    }
  }

  MatrixXd b_exp(3, 2);
  b_exp << cos(theta), 0.0,
    sin(theta), 0.0,
    0.0, 1.0;
  for (int r = 0; r < 3; r++) {
    for (int col = 0; col < 2; col++) {
      EXPECT_NEAR(model.getB()(r, col), b_exp(r, col), kTol);
    }
  }
}

// An empty configure() keeps the constructor literals; a present key overrides
// only the named bound side, leaving the absent side at its literal.
TEST(ModelInterface, ConfigureKeepsLiteralsAndOverrides)
{
  Bicycle model;

  model.configure({});  // no overrides
  EXPECT_NEAR(model.getParams()(0), 1.6, kTol);
  double low = 0.0;
  double upp = 0.0;
  getBound(model, "u", 0, low, upp);
  EXPECT_NEAR(low, -3.0, kTol);
  EXPECT_NEAR(upp, 3.0, kTol);

  model.configure({{"L", 2.5}, {"v_max", 4.0}});
  EXPECT_NEAR(model.getParams()(0), 2.5, kTol);
  getBound(model, "u", 0, low, upp);
  EXPECT_NEAR(low, -3.0, kTol);   // lower kept (key absent)
  EXPECT_NEAR(upp, 4.0, kTol);    // upper overridden

  // Symmetric cap as the benchmark launch emits it from robot max_linear_vel.
  model.configure({{"v_min", -0.5}, {"v_max", 0.5}});
  getBound(model, "u", 0, low, upp);
  EXPECT_NEAR(low, -0.5, kTol);
  EXPECT_NEAR(upp, 0.5, kTol);
}

// The wheelbase divides the bicycle's yaw and steering Jacobians, so a
// non-positive override is rejected rather than producing a non-finite QP.
TEST(ModelInterface, BicycleRejectsNonPositiveWheelbase)
{
  Bicycle model;

  EXPECT_THROW(model.configure({{"L", 0.0}}), std::invalid_argument);
  EXPECT_THROW(model.configure({{"L", -1.5}}), std::invalid_argument);
  EXPECT_NO_THROW(model.configure({{"L", 1.6}}));
  EXPECT_NEAR(model.getParams()(0), 1.6, kTol);
}

// Each model maps its control vector to the expected body Twist.
TEST(ModelInterface, ToTwistSemantics)
{
  Unicycle uni;
  VectorXd uu(2);
  uu << 0.7, 0.3;                 // [v, omega] maps directly
  auto tw_u = uni.toTwist(uu);
  EXPECT_NEAR(tw_u.linear.x, 0.7, kTol);
  EXPECT_NEAR(tw_u.angular.z, 0.3, kTol);

  Bicycle bicycle;
  const double L = 1.6;
  const double delta = 0.2;
  VectorXd xb(4);
  xb << 0.0, 0.0, 0.0, delta;
  bicycle.setX(xb);
  VectorXd ub(2);
  ub << 0.7, 0.1;                 // [v, delta_dot]; omega = v sin(delta)/L
  auto tw_b = bicycle.toTwist(ub);
  EXPECT_NEAR(tw_b.linear.x, 0.7, kTol);
  EXPECT_NEAR(tw_b.angular.z, 0.7 * std::sin(delta) / L, kTol);
}
