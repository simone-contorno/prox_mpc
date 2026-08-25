// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0

#ifndef PROX_MPC_TEST_MODELS__NON_FINITE_TWIST_OTHER_AXES_MODEL_HPP_
#define PROX_MPC_TEST_MODELS__NON_FINITE_TWIST_OTHER_AXES_MODEL_HPP_

#include <limits>
#include <string>

#include <geometry_msgs/msg/twist.hpp>

#include <prox_mpc/model.hpp>
#include <prox_mpc/utils.hpp>

namespace prox_mpc_test_models
{

/// Fault-injection model for testing the controller's command-validation path
/// on a Twist component OTHER than linear.x, which NonFiniteTwistModel already
/// covers. A planar Nav2 consumer reads only linear.x and angular.z, so
/// twist_is_finite() checking all six components (not only the two a planar
/// base consumes) is untested unless something actually produces a non-finite
/// value in one of the other four. Its dynamics are the same finite linear
/// integrator as NonFiniteTwistModel (state [x, y, theta], control [v, omega]),
/// so the QP solves and reports PROXQP_SOLVED with a finite first control, but
/// toTwist() deliberately fills linear.y and angular.x with non-finite values.
class NonFiniteTwistOtherAxesModel : public prox_mpc::Model
{
public:
  NonFiniteTwistOtherAxesModel()
  {
    setName("non_finite_twist_other_axes");
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

  /// Finite command mapped normally onto linear.x and angular.z, but with
  /// linear.y and angular.x deliberately set non-finite -- the two components
  /// a planar Nav2 consumer never reads, so nothing downstream of the
  /// controller would ever notice unless the controller itself validates them.
  geometry_msgs::msg::Twist toTwist(const VectorXd & u_in) const override
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = u_in.size() > 0 ? u_in(0) : 0.0;
    twist.linear.y = std::numeric_limits<double>::quiet_NaN();
    twist.angular.x = std::numeric_limits<double>::infinity();
    twist.angular.z = u_in.size() > 1 ? u_in(1) : 0.0;
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

#endif  // PROX_MPC_TEST_MODELS__NON_FINITE_TWIST_OTHER_AXES_MODEL_HPP_
