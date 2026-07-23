// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0

#ifndef PROX_MPC_TEST_MODELS__NON_FINITE_TWIST_MODEL_HPP_
#define PROX_MPC_TEST_MODELS__NON_FINITE_TWIST_MODEL_HPP_

#include <limits>
#include <string>

#include <geometry_msgs/msg/twist.hpp>

#include <prox_mpc/model.hpp>
#include <prox_mpc/utils.hpp>

namespace prox_mpc_test_models
{

/// Fault-injection model for testing the controller's non-finite-command
/// fail-safe through the pluginlib path. Its dynamics are a finite linear
/// integrator (state [x, y, theta], control [v, omega]) so the QP solves and
/// reports PROXQP_SOLVED with a finite first control, but toTwist() deliberately
/// returns a non-finite command. This is the only seam that exercises the
/// non-finite-command branch, which a finite-mapping model cannot reach.
class NonFiniteTwistModel : public prox_mpc::Model
{
public:
  NonFiniteTwistModel()
  {
    setName("non_finite_twist");
    setN(3);  // state: [x, y, theta]
    setM(2);  // control: [v, omega]

    setA(MatrixXd::Zero(getN(), getN()));
    setB(MatrixXd::Zero(getN(), getM()));
    setc(VectorXd::Zero(getN()));

    setIneq("u", 0, -3.0, 3.0);  // linear velocity [m/s]
    setIneq("u", 1, -1.0, 1.0);  // angular velocity [rad/s]
    setIneq("du", 0, -0.5, 0.5);  // linear acceleration [m/s^2]
    setIneq("du", 1, -0.5, 0.5);  // angular acceleration [rad/s^2]

    setObsAvoid(false);
  }

  /// Deliberately non-finite command from a finite control, to drive the
  /// controller's non-finite-command fail-safe.
  geometry_msgs::msg::Twist toTwist(const VectorXd &) const override
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = std::numeric_limits<double>::quiet_NaN();
    twist.angular.z = 0.0;
    return twist;
  }

  void updatec(double dt, VectorXd x_next) override
  {
    c << getX()(0) - x_next(0) + dt * getU()(0),
      getX()(1) - x_next(1),
      getX()(2) - x_next(2) + dt * getU()(1);
  }

  void updateA(double /*dt*/) override
  {
    A = MatrixXd::Identity(getN(), getN());
  }

  void updateB() override
  {
    B << 1.0, 0.0,
      0.0, 0.0,
      0.0, 1.0;
  }
};

}  // namespace prox_mpc_test_models

#endif  // PROX_MPC_TEST_MODELS__NON_FINITE_TWIST_MODEL_HPP_
