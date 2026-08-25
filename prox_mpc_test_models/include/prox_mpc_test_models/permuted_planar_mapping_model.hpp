// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0

#ifndef PROX_MPC_TEST_MODELS__PERMUTED_PLANAR_MAPPING_MODEL_HPP_
#define PROX_MPC_TEST_MODELS__PERMUTED_PLANAR_MAPPING_MODEL_HPP_

#include <cmath>
#include <string>

#include <prox_mpc/model.hpp>
#include <prox_mpc/utils.hpp>

namespace prox_mpc_test_models
{

/// Fixture model whose state vector is ordered [theta, x, y] instead of the
/// [x, y, theta] every bundled model uses, so its declared planar mapping is
/// idx_yaw = 0, idx_x = 1, idx_y = 2. It is the only seam that can show the
/// controller honouring the declared mapping rather than the state ordering it
/// assumed before the mapping hook existed: every bundled model maps its
/// position to state columns 0 and 1, so no permutation defect is observable
/// through them.
///
/// Obstacle avoidance is on, so the fixture also covers the in-loop keep-out
/// term against a permuted state: the solver reads the position's indices from
/// the same declared mapping rather than assuming state columns 0 and 1.
///
/// The dynamics are the unicycle's, written against the permuted ordering, so
/// the model both configures and solves.
class PermutedPlanarMappingModel : public prox_mpc::Model
{
public:
  PermutedPlanarMappingModel()
  {
    setName("permuted_planar_mapping");
    setN(3);  // state: [theta, x, y]
    setM(2);  // control: [v, omega]

    setA(MatrixXd::Zero(getN(), getN()));
    setB(MatrixXd::Zero(getN(), getM()));
    setc(VectorXd::Zero(getN()));

    setIneq("u", 0, -3.0, 3.0);     // linear velocity [m/s]
    setIneq("u", 1, -1.0, 1.0);     // angular velocity [rad/s]
    setIneq("du", 0, -0.5, 0.5);    // linear acceleration [m/s^2]
    setIneq("du", 1, -0.5, 0.5);    // angular acceleration [rad/s^2]

    setObsAvoid(true);
  }

  prox_mpc::PlanarMapping getPlanarMapping() const override
  {
    prox_mpc::PlanarMapping mapping;
    mapping.idx_yaw = 0;
    mapping.idx_x = 1;
    mapping.idx_y = 2;
    return mapping;
  }

  void updatec(double dt, VectorXd x_next) override
  {
    const double th = getX()(0);
    c << getX()(0) - x_next(0) + dt * getU()(1),
      getX()(1) - x_next(1) + dt * getU()(0) * std::cos(th),
      getX()(2) - x_next(2) + dt * getU()(0) * std::sin(th);
  }

  void updateA(double dt) override
  {
    const double th = getX()(0);
    const double v = getU()(0);
    A << 1.0, 0.0, 0.0,
      -dt * v * std::sin(th), 1.0, 0.0,
      dt * v * std::cos(th), 0.0, 1.0;
  }

  void updateB() override
  {
    const double th = getX()(0);
    B << 0.0, 1.0,
      std::cos(th), 0.0,
      std::sin(th), 0.0;
  }
};

}  // namespace prox_mpc_test_models

#endif  // PROX_MPC_TEST_MODELS__PERMUTED_PLANAR_MAPPING_MODEL_HPP_
