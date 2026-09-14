// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Tests for the Model interface using the bundled models: name, dimensions,
// declared bounds, the analytic forward-Euler residual and Jacobians, and the
// configure(), toTwist() and fromTwist() hooks.

#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include <geometry_msgs/msg/twist.hpp>

#include <prox_mpc/model.hpp>
#include <prox_mpc/models/bicycle.hpp>
#include <prox_mpc/models/bicycle_front_axle.hpp>
#include <prox_mpc/models/bicycle_rear_axle.hpp>
#include <prox_mpc/models/unicycle.hpp>

using prox_mpc::Bicycle;
using prox_mpc::BicycleFrontAxle;
using prox_mpc::BicycleRearAxle;
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

// A four-state, two-control model that overrides nothing: no updatec/updateA/
// updateB call matters here (getPlanarMapping() never solves), so they are
// stubs. Used to pin Model::getPlanarMapping()'s default inference for a model
// that declares no mapping of its own: state count > 3 infers a steering angle
// at index 3, and control count > 1 infers its rate at control index 1.
class BareFourStateModel : public Model
{
public:
  BareFourStateModel()
  {
    setName("bare_four_state");
    setN(4);
    setM(2);
  }
  void updatec(double, VectorXd) override {}
  void updateA(double) override {}
  void updateB() override {}
};
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

  const double L = 1.6;
  const double delta = 0.2;
  VectorXd xb(4);
  xb << 0.0, 0.0, 0.0, delta;
  VectorXd ub(2);
  ub << 0.7, 0.1;                 // [v, delta_dot]

  // Both bicycles report the body twist of base_link, so the front-axle model's
  // control - the front-wheel speed - is projected onto the body x axis, while
  // the rear-axle model's control is already that speed. The yaw rate follows
  // each model's own steering law.
  BicycleFrontAxle front;
  front.setX(xb);
  auto tw_f = front.toTwist(ub);
  EXPECT_NEAR(tw_f.linear.x, 0.7 * std::cos(delta), kTol);
  EXPECT_NEAR(tw_f.angular.z, 0.7 * std::sin(delta) / L, kTol);

  BicycleRearAxle rear;
  rear.setX(xb);
  auto tw_r = rear.toTwist(ub);
  EXPECT_NEAR(tw_r.linear.x, 0.7, kTol);
  EXPECT_NEAR(tw_r.angular.z, 0.7 * std::tan(delta) / L, kTol);

  // The deprecated alias is the front-axle model.
  Bicycle bicycle;
  bicycle.setX(xb);
  auto tw_b = bicycle.toTwist(ub);
  EXPECT_NEAR(tw_b.linear.x, tw_f.linear.x, kTol);
  EXPECT_NEAR(tw_b.angular.z, tw_f.angular.z, kTol);
}

// fromTwist recovers the control a twist was produced from, for the channels a
// twist determines. The unicycle and the rear-axle model both carry the
// base_link speed in control 0, so the base class default reads it straight
// out; the front-axle model carries a front-wheel speed there, which only the
// (linear.x, angular.z) pair together determines, and its override says so. No
// twist observes a steering rate, so the front-axle override leaves control 1
// undetermined for the caller to fill from somewhere else.
TEST(ModelInterface, FromTwistInvertsToTwist)
{
  Unicycle uni;
  geometry_msgs::msg::Twist tw;
  tw.linear.x = 0.7;
  tw.angular.z = 0.3;
  const VectorXd u_uni = uni.fromTwist(tw);
  ASSERT_EQ(u_uni.size(), 2);
  EXPECT_NEAR(u_uni(0), 0.7, kTol);
  EXPECT_NEAR(u_uni(1), 0.3, kTol);

  const double delta = 0.2;
  VectorXd xb(4);
  xb << 0.0, 0.0, 0.0, delta;
  VectorXd ub(2);
  ub << 0.7, 0.1;                 // [v, delta_dot]

  BicycleFrontAxle front;
  front.setX(xb);
  const VectorXd u_front = front.fromTwist(front.toTwist(ub));
  ASSERT_EQ(u_front.size(), 2);
  EXPECT_NEAR(u_front(0), 0.7, kTol);        // the front-wheel speed, not its projection
  EXPECT_FALSE(std::isfinite(u_front(1)));   // delta_dot is not observable in a twist

  BicycleRearAxle rear;
  rear.setX(xb);
  const VectorXd u_rear = rear.fromTwist(rear.toTwist(ub));
  ASSERT_EQ(u_rear.size(), 2);
  EXPECT_NEAR(u_rear(0), 0.7, kTol);         // control 0 is already the base_link speed
  EXPECT_FALSE(std::isfinite(u_rear(1)));    // delta_dot is not observable here either
}

// Both bicycles override the inverse because both override toTwist, and the
// base class default is the inverse of the mapping neither of them uses: it
// returns angular.z in control 1, a body yaw rate, while control 1 on either
// bicycle is a steering rate. The rear axle is the case where that is easy to
// miss, since its control 0 does match the default. Driven head-on, where the
// emitted yaw rate is zero, so an inherited default would return a plausible
// finite 0.0 in the steering-rate slot rather than an obviously wrong number.
TEST(ModelInterface, RearAxleFromTwistLeavesTheSteeringRateUndetermined)
{
  BicycleRearAxle rear;
  VectorXd xb(4);
  xb << 0.0, 0.0, 0.0, 0.0;      // straight ahead: toTwist emits angular.z == 0
  rear.setX(xb);
  VectorXd ub(2);
  ub << 1.3, 0.4;                // [v, delta_dot], with a non-zero steering rate

  const geometry_msgs::msg::Twist tw = rear.toTwist(ub);
  ASSERT_NEAR(tw.angular.z, 0.0, kTol);

  const VectorXd u = rear.fromTwist(tw);
  ASSERT_EQ(u.size(), 2);
  EXPECT_NEAR(u(0), 1.3, kTol);        // round-trips the base_link speed
  EXPECT_FALSE(std::isfinite(u(1)));   // not 0.0: the twist says nothing about delta_dot

  // Reverse travel keeps its sign; there is no magnitude to recover here.
  ub(0) = -0.8;
  rear.setX(xb);
  EXPECT_NEAR(rear.fromTwist(rear.toTwist(ub))(0), -0.8, kTol);
}

// The front-axle inverse is singularity-free at the model's own +/- pi/2
// steering bound, where cos(delta) is zero and the projection carries no
// direction, and it recovers the sign of a reversing speed.
TEST(ModelInterface, FrontAxleFromTwistHandlesFullLockAndReverse)
{
  BicycleFrontAxle front;
  VectorXd xb(4);
  xb << 0.0, 0.0, 0.0, M_PI / 2;
  front.setX(xb);
  VectorXd ub(2);
  ub << 0.7, 0.0;
  EXPECT_NEAR(front.fromTwist(front.toTwist(ub))(0), 0.7, 1e-9);

  xb(3) = -0.4;
  front.setX(xb);
  ub(0) = -0.9;
  EXPECT_NEAR(front.fromTwist(front.toTwist(ub))(0), -0.9, kTol);
}

// Every bundled model's two twist mappings agree: for each control fromTwist()
// reports as determined, it returns the control toTwist() was given. This is
// the guard on the hazard the pair creates - a model that overrides toTwist()
// and inherits the base fromTwist() gets an inverse of a mapping it does not
// use, and the consumer that seeds a control vector from a measured twist then
// writes a plausible-looking wrong number into that channel. It is the
// steering-rate channel of either bicycle that this catches: the base default
// returns angular.z there, a body yaw rate, which is neither of their control
// 1. A channel reported non-finite carries no claim and is skipped.
TEST(ModelInterface, FromTwistAgreesWithToTwistOnEveryDeterminedChannel)
{
  VectorXd x3 = VectorXd::Zero(3);
  VectorXd x4(4);
  x4 << 0.0, 0.0, 0.0, 0.35;      // a steering angle the yaw laws actually use
  VectorXd u(2);
  // Control 1 is deliberately far from the yaw rate this state and speed imply
  // (about 0.25 rad/s for either bicycle), so a channel filled from angular.z
  // instead of from the control is unmistakable rather than merely off.
  u << 1.1, 0.9;                  // [v, omega] or [v, delta_dot], per model

  Unicycle uni;
  uni.setX(x3);
  BicycleFrontAxle front;
  front.setX(x4);
  BicycleRearAxle rear;
  rear.setX(x4);
  Bicycle alias;                  // deprecated, and still held to the same contract
  alias.setX(x4);

  struct Case
  {
    const char * name;
    Model * model;
  };
  const Case cases[] = {
    {"Unicycle", &uni},
    {"BicycleFrontAxle", &front},
    {"BicycleRearAxle", &rear},
    {"Bicycle", &alias},
  };

  for (const auto & item : cases) {
    SCOPED_TRACE(item.name);
    const VectorXd back = item.model->fromTwist(item.model->toTwist(u));
    ASSERT_EQ(back.size(), static_cast<Eigen::Index>(item.model->getM()));
    bool any_determined = false;
    for (Eigen::Index j = 0; j < back.size(); j++) {
      if (!std::isfinite(back(j))) {continue;}
      any_determined = true;
      EXPECT_NEAR(back(j), u(j), 1e-9) << "control " << j;
    }
    // A model determining nothing would pass the loop vacuously.
    EXPECT_TRUE(any_determined);
  }
}

// --- BicycleRearAxle: identity, bounds, residual and Jacobians --------------
//
// BicycleFrontAxle (via the Bicycle alias) is covered above; BicycleRearAxle's
// own name, bounds and kinematics (v tan(delta)/L rather than v sin(delta)/L,
// and the steering bound capped at 1.0 rad rather than pi/2) are untested
// anywhere before this.

TEST(ModelInterface, RearAxleIdentityAndBounds)
{
  BicycleRearAxle model;
  EXPECT_EQ(model.getName(), "bicycle_rear_axle");
  EXPECT_EQ(model.getN(), 4u);
  EXPECT_EQ(model.getM(), 2u);
  EXPECT_NEAR(model.getParams()(0), 1.6, kTol);  // wheelbase L

  double low = 0.0;
  double upp = 0.0;
  // Capped at 1.0 rad (not the front axle's pi/2): the rear-axle yaw law
  // v tan(delta)/L diverges as |delta| approaches pi/2.
  getBound(model, "x", 3, low, upp);
  EXPECT_NEAR(low, -1.0, kTol);
  EXPECT_NEAR(upp, 1.0, kTol);
  getBound(model, "u", 0, low, upp);
  EXPECT_NEAR(low, -3.0, kTol);
  EXPECT_NEAR(upp, 3.0, kTol);
  getBound(model, "du", 0, low, upp);
  EXPECT_NEAR(low, -0.5, kTol);
  EXPECT_NEAR(upp, 0.5, kTol);
}

TEST(ModelInterface, RearAxleEulerResidualAndJacobians)
{
  BicycleRearAxle model;
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
  const double sec2 = 1.0 / (std::cos(delta) * std::cos(delta));

  VectorXd c_exp(4);
  c_exp << x(0) - x_next(0) + dt * v * std::cos(theta),
    x(1) - x_next(1) + dt * v * std::sin(theta),
    x(2) - x_next(2) + dt * v * std::tan(delta) / L,
    x(3) - x_next(3) + dt * u(1);
  for (int i = 0; i < 4; i++) {
    EXPECT_NEAR(model.getc()(i), c_exp(i), kTol);
  }

  MatrixXd a_exp(4, 4);
  a_exp << 1.0, 0.0, -dt * v * std::sin(theta), 0.0,
    0.0, 1.0, dt * v * std::cos(theta), 0.0,
    0.0, 0.0, 1.0, dt * v * sec2 / L,
    0.0, 0.0, 0.0, 1.0;
  for (int r = 0; r < 4; r++) {
    for (int col = 0; col < 4; col++) {
      EXPECT_NEAR(model.getA()(r, col), a_exp(r, col), kTol);
    }
  }

  MatrixXd b_exp(4, 2);
  b_exp << std::cos(theta), 0.0,
    std::sin(theta), 0.0,
    std::tan(delta) / L, 0.0,
    0.0, 1.0;
  for (int r = 0; r < 4; r++) {
    for (int col = 0; col < 2; col++) {
      EXPECT_NEAR(model.getB()(r, col), b_exp(r, col), kTol);
    }
  }
}

// The rear-axle yaw law diverges as |delta| -> pi/2, so a steering bound past
// the 1.0 rad cap is rejected at configure() rather than silently poisoning A,
// B and c with a near-infinite Jacobian entry.
TEST(ModelInterface, RearAxleRejectsSteerBoundBeyondCap)
{
  BicycleRearAxle model;
  EXPECT_THROW(model.configure({{"delta_max", 1.2}}), std::invalid_argument);
  EXPECT_THROW(model.configure({{"delta_min", -1.4}}), std::invalid_argument);
  EXPECT_NO_THROW(model.configure({{"delta_min", -1.0}, {"delta_max", 1.0}}));
  double low = 0.0;
  double upp = 0.0;
  getBound(model, "x", 3, low, upp);
  EXPECT_NEAR(low, -1.0, kTol);
  EXPECT_NEAR(upp, 1.0, kTol);
}

// A non-finite steering bound is rejected too. The cap test is a bare ">",
// which is false for NaN, so nothing but an explicit finiteness check keeps a
// NaN out of the state-bound rows the QP is assembled from.
TEST(ModelInterface, RearAxleRejectsNonFiniteSteerBound)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  BicycleRearAxle model;
  EXPECT_THROW(model.configure({{"delta_max", nan}}), std::invalid_argument);
  EXPECT_THROW(model.configure({{"delta_min", -inf}}), std::invalid_argument);

  // The declared bounds are untouched by a rejected configure().
  double low = 0.0;
  double upp = 0.0;
  getBound(model, "x", 3, low, upp);
  EXPECT_NEAR(low, -BicycleRearAxle::kMaxSteerAngle, kTol);
  EXPECT_NEAR(upp, BicycleRearAxle::kMaxSteerAngle, kTol);
}

// --- Model::getPlanarMapping() -----------------------------------------------
//
// A non-pure virtual added after 1.0.0. Pin the bundled models' declared
// mappings and the base class default a model that overrides nothing falls
// back to.

// Unicycle carries no steering state (n == 3), so the default mapping declares
// none: the base class only infers a steering angle for a model with more than
// three states.
TEST(ModelInterface, UnicyclePlanarMappingDeclaresNoSteering)
{
  Unicycle model;
  const auto mapping = model.getPlanarMapping();
  EXPECT_EQ(mapping.idx_x, 0u);
  EXPECT_EQ(mapping.idx_y, 1u);
  EXPECT_EQ(mapping.idx_yaw, 2u);
  EXPECT_EQ(mapping.idx_speed, 0u);
  EXPECT_EQ(mapping.idx_steering, prox_mpc::PlanarMapping::kNoIndex);
  EXPECT_EQ(mapping.idx_steer_rate, prox_mpc::PlanarMapping::kNoIndex);
  EXPECT_NEAR(mapping.ref_offset_x, 0.0, kTol);
  EXPECT_NEAR(mapping.ref_offset_y, 0.0, kTol);
  EXPECT_NEAR(mapping.wheelbase, 0.0, kTol);
}

// A model with more than three states and more than one control that declares
// no override still drives: the base class default infers a steering angle at
// state index 3 and its rate at control index 1, with no reference offset and
// no wheelbase declared.
TEST(ModelInterface, DefaultPlanarMappingInfersSteeringForUndeclaredFourStateModel)
{
  BareFourStateModel model;
  const auto mapping = model.getPlanarMapping();
  EXPECT_EQ(mapping.idx_steering, 3u);
  EXPECT_EQ(mapping.idx_steer_rate, 1u);
  EXPECT_NEAR(mapping.ref_offset_x, 0.0, kTol);
  EXPECT_NEAR(mapping.ref_offset_y, 0.0, kTol);
  EXPECT_NEAR(mapping.wheelbase, 0.0, kTol);  // undeclared
}

// BicycleFrontAxle declares its reference point one wheelbase ahead of
// base_link on the body x axis, which is also the wheelbase the steering
// geometry uses.
TEST(ModelInterface, FrontAxlePlanarMappingDeclaresForwardOffset)
{
  BicycleFrontAxle model;
  const auto mapping = model.getPlanarMapping();
  EXPECT_EQ(mapping.idx_steering, 3u);
  EXPECT_EQ(mapping.idx_steer_rate, 1u);
  EXPECT_NEAR(mapping.ref_offset_x, 1.6, kTol);
  EXPECT_NEAR(mapping.ref_offset_y, 0.0, kTol);
  EXPECT_NEAR(mapping.wheelbase, 1.6, kTol);
}

// BicycleRearAxle's state already refers to base_link, so its reference offset
// is zero even though it declares a steering angle.
TEST(ModelInterface, RearAxlePlanarMappingDeclaresNoOffset)
{
  BicycleRearAxle model;
  const auto mapping = model.getPlanarMapping();
  EXPECT_EQ(mapping.idx_steering, 3u);
  EXPECT_EQ(mapping.idx_steer_rate, 1u);
  EXPECT_NEAR(mapping.ref_offset_x, 0.0, kTol);
  EXPECT_NEAR(mapping.ref_offset_y, 0.0, kTol);
  EXPECT_NEAR(mapping.wheelbase, 1.6, kTol);
}

// The deprecated Bicycle alias does not override getPlanarMapping(), so it
// inherits BicycleFrontAxle's, resolving the alias physically as well as by
// dynamics.
TEST(ModelInterface, BicycleAliasInheritsFrontAxlePlanarMapping)
{
  Bicycle alias;
  BicycleFrontAxle front;
  const auto ma = alias.getPlanarMapping();
  const auto mf = front.getPlanarMapping();
  EXPECT_EQ(ma.idx_steering, mf.idx_steering);
  EXPECT_EQ(ma.idx_steer_rate, mf.idx_steer_rate);
  EXPECT_NEAR(ma.ref_offset_x, mf.ref_offset_x, kTol);
  EXPECT_NEAR(ma.wheelbase, mf.wheelbase, kTol);
}

// --- Finite-difference Jacobian checks ---------------------------------------
//
// The existing residual/Jacobian tests above compare updateA()/updateB()
// against an independently hand-derived closed form. This instead compares
// them against a numerical (central finite-difference) derivative of
// updatec()'s own residual, which catches a mismatch between the two
// hand-written analytic expressions (residual and Jacobian) even if both were
// transcribed from the same wrong formula.

// d(residual)/d(x_k) and d(residual)/d(u_k) via central differences, compared
// against updateA()/updateB() at the same operating point.
void checkFiniteDifferenceJacobian(Model & model, const VectorXd & x, const VectorXd & u)
{
  const double dt = 0.1;
  const double eps = 1e-6;
  const VectorXd x_next = VectorXd::Zero(x.size());  // held fixed; only c's x_k-dependence is probed

  auto residual = [&](const VectorXd & xx, const VectorXd & uu) {
      model.setX(xx);
      model.setU(uu);
      model.updatec(dt, x_next);
      return model.getc();
    };

  model.setX(x);
  model.setU(u);
  model.updateA(dt);
  model.updateB();
  const MatrixXd A = model.getA();
  const MatrixXd B = model.getB();

  for (Eigen::Index j = 0; j < x.size(); ++j) {
    VectorXd xp = x;
    VectorXd xm = x;
    xp(j) += eps;
    xm(j) -= eps;
    const VectorXd fd = (residual(xp, u) - residual(xm, u)) / (2.0 * eps);
    for (Eigen::Index i = 0; i < x.size(); ++i) {
      EXPECT_NEAR(A(i, j), fd(i), 1e-4) << "dA/dx mismatch at (" << i << "," << j << ")";
    }
  }
  // updateB() returns the raw kinematics Jacobian df/du (proxqp.cpp scales it
  // by dt externally when assembling the equality matrix, model->getB()*dt),
  // while the residual c = x_k - x_next + dt*f(x_k, u_k) is what the finite
  // difference below actually differentiates, so its du derivative is dt*B,
  // not B itself.
  for (Eigen::Index j = 0; j < u.size(); ++j) {
    VectorXd up = u;
    VectorXd um = u;
    up(j) += eps;
    um(j) -= eps;
    const VectorXd fd = (residual(x, up) - residual(x, um)) / (2.0 * eps);
    for (Eigen::Index i = 0; i < x.size(); ++i) {
      EXPECT_NEAR(dt * B(i, j), fd(i), 1e-4) << "dB/du mismatch at (" << i << "," << j << ")";
    }
  }
}

TEST(ModelInterface, FrontAxleFiniteDifferenceJacobianMatchesAnalytic)
{
  BicycleFrontAxle model;
  VectorXd x(4);
  x << 1.0, 2.0, 0.3, 0.1;
  VectorXd u(2);
  u << 0.7, 0.2;
  checkFiniteDifferenceJacobian(model, x, u);
}

TEST(ModelInterface, RearAxleFiniteDifferenceJacobianMatchesAnalytic)
{
  BicycleRearAxle model;
  VectorXd x(4);
  x << 1.0, 2.0, 0.3, 0.1;
  VectorXd u(2);
  u << 0.7, 0.2;
  checkFiniteDifferenceJacobian(model, x, u);
}

// --- Negative steering --------------------------------------------------------
//
// ToTwistSemantics above exercises only delta = +0.2; the sign of the yaw rate
// (and, for the front axle, of the projected linear speed) must flip with it.

TEST(ModelInterface, ToTwistWithNegativeSteering)
{
  const double L = 1.6;
  const double delta = -0.25;
  VectorXd xb(4);
  xb << 0.0, 0.0, 0.0, delta;
  VectorXd ub(2);
  ub << 0.7, -0.1;

  BicycleFrontAxle front;
  front.setX(xb);
  const auto tw_f = front.toTwist(ub);
  EXPECT_NEAR(tw_f.linear.x, 0.7 * std::cos(delta), kTol);
  EXPECT_NEAR(tw_f.angular.z, 0.7 * std::sin(delta) / L, kTol);
  EXPECT_LT(tw_f.angular.z, 0.0);  // negative steering yaws the opposite way

  BicycleRearAxle rear;
  rear.setX(xb);
  const auto tw_r = rear.toTwist(ub);
  EXPECT_NEAR(tw_r.linear.x, 0.7, kTol);
  EXPECT_NEAR(tw_r.angular.z, 0.7 * std::tan(delta) / L, kTol);
  EXPECT_LT(tw_r.angular.z, 0.0);
}

// --- Twist integration vs. state propagation at the reference point ---------
//
// For a rigid body, the reference point's world-frame velocity is the body
// twist's linear velocity (rotated into world) plus the rotational term of the
// offset: d/dt p_ref = R(theta) * v_body + omega * (offset rotated 90 deg).
// For the front axle this is an exact algebraic identity given v_body =
// v cos(delta), omega = v sin(delta)/L and offset = (L, 0):
//   xdot = v cos(delta) cos(theta) - omega L sin(theta) = v cos(theta + delta)
//   ydot = v cos(delta) sin(theta) + omega L cos(theta) = v sin(theta + delta)
// which is exactly the model's own propagation (BicycleEulerResidualAndJacobians
// above). This cross-checks toTwist() against updateA()/updateB()/updatec()
// independently of both: a bug that corrupted one but not the other would break
// this identity even if each half's own tests still passed.
TEST(ModelInterface, FrontAxleTwistIntegrationMatchesStatePropagation)
{
  const double L = 1.6;
  const double theta = 0.5;
  const double delta = 0.3;
  const double v = 0.9;

  VectorXd xb(4);
  xb << 0.0, 0.0, theta, delta;
  BicycleFrontAxle model;
  model.setX(xb);
  VectorXd u(2);
  u << v, 0.0;
  const auto twist = model.toTwist(u);

  const double xdot_from_twist =
    twist.linear.x * std::cos(theta) - twist.angular.z * L * std::sin(theta);
  const double ydot_from_twist =
    twist.linear.x * std::sin(theta) + twist.angular.z * L * std::cos(theta);

  EXPECT_NEAR(xdot_from_twist, v * std::cos(theta + delta), 1e-9);
  EXPECT_NEAR(ydot_from_twist, v * std::sin(theta + delta), 1e-9);
}

// The rear axle's reference point is base_link itself (zero offset), so its
// twist trivially equals its own state velocity with no rotational offset term.
TEST(ModelInterface, RearAxleTwistIntegrationMatchesStatePropagation)
{
  const double L = 1.6;
  const double theta = 0.5;
  const double delta = 0.3;
  const double v = 0.9;

  VectorXd xb(4);
  xb << 0.0, 0.0, theta, delta;
  BicycleRearAxle model;
  model.setX(xb);
  VectorXd u(2);
  u << v, 0.0;
  const auto twist = model.toTwist(u);

  const double xdot_from_twist = twist.linear.x * std::cos(theta);
  const double ydot_from_twist = twist.linear.x * std::sin(theta);

  EXPECT_NEAR(xdot_from_twist, v * std::cos(theta), 1e-9);
  EXPECT_NEAR(ydot_from_twist, v * std::sin(theta), 1e-9);
  EXPECT_NEAR(twist.angular.z, v * std::tan(delta) / L, 1e-9);
}
