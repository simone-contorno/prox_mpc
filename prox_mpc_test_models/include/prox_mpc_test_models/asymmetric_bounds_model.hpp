// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0

#ifndef PROX_MPC_TEST_MODELS__ASYMMETRIC_BOUNDS_MODEL_HPP_
#define PROX_MPC_TEST_MODELS__ASYMMETRIC_BOUNDS_MODEL_HPP_

#include <string>

#include <prox_mpc/model.hpp>
#include <prox_mpc/utils.hpp>

namespace prox_mpc_test_models
{

/// Fixture model whose declared bounds are asymmetric on every side the
/// controller reads, used to test that the controller preserves an asymmetric
/// range instead of assuming symmetry: both bundled production models (Bicycle,
/// Unicycle) declare symmetric bounds, so no controller behavior on asymmetry
/// can be exercised through them. Its dynamics are the same finite linear
/// integrator as NonFiniteTwistModel (state [x, y, theta], control [v, omega]),
/// and toTwist() is the Model base class's identity mapping, so the control
/// vector IS the body twist (no steering channel).
class AsymmetricBoundsModel : public prox_mpc::Model
{
public:
  AsymmetricBoundsModel()
  {
    setName("asymmetric_bounds");
    setN(3);  // state: [x, y, theta]
    setM(2);  // control: [v, omega]

    setA(MatrixXd::Zero(getN(), getN()));
    setB(MatrixXd::Zero(getN(), getM()));
    setc(VectorXd::Zero(getN()));

    // Reverse travel is capped tighter than forward travel.
    setIneq("u", 0, -0.3, 2.0);   // linear velocity [m/s]
    setIneq("u", 1, -1.0, 1.0);   // angular velocity [rad/s]
    // Braking (toward zero) is bounded harder than accelerating (away from
    // zero) on the linear channel, so the ramp step differs by direction.
    setIneq("du", 0, -2.0, 0.5);  // linear acceleration [m/s^2]
    setIneq("du", 1, -1.0, 1.0);  // angular acceleration [rad/s^2]

    setObsAvoid(false);
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

#endif  // PROX_MPC_TEST_MODELS__ASYMMETRIC_BOUNDS_MODEL_HPP_
