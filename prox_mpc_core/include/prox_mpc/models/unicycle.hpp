// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0

#ifndef PROX_MPC__MODELS__UNICYCLE_HPP_
#define PROX_MPC__MODELS__UNICYCLE_HPP_

#include <map>
#include <string>

#include <prox_mpc/model.hpp>
#include <prox_mpc/utils.hpp>

namespace prox_mpc
{

/* Unicycle (differential-drive) kinematic model.
 * Control is [v, omega], a body twist, so toTwist() uses the base identity
 * mapping (linear.x = v, angular.z = omega). */
class Unicycle : public Model
{
public:
  Unicycle()
  {
    setName("unicycle");

    setN(3);  // state: [x, y, theta]
    setM(2);  // control: [v, omega]

    double L = 0.0;
    VectorXd init_params(1);
    init_params << L;
    setParams(init_params);

    setA(MatrixXd::Zero(getN(), getN()));
    setB(MatrixXd::Zero(getN(), getM()));
    setc(VectorXd::Zero(getN()));

    setIneq("u", 0, -3.0, 3.0);  // linear velocity [m/s]
    setIneq("u", 1, -1., 1.);    // angular velocity [rad/s]
    setIneq("du", 0, -.5, .5);   // linear acceleration [m/s^2]
    setIneq("du", 1, -.5, .5);   // angular acceleration [rad/s^2]

    setObsAvoid(true);  // this model supports obstacle avoidance
  }

  /*!
   * Configure the model constants from a params map. Absent keys keep the
   * constructor literals, so an empty map reproduces the hardcoded values.
   * Keys: bound limits "v_min"/"v_max" (u[0]), "w_min"/"w_max" (u[1]),
   * "a_min"/"a_max" (du[0]), "alpha_min"/"alpha_max" (du[1]).
   */
  void configure(const std::map<std::string, double> & config_params) override
  {
    overrideBound(config_params, "u", 0, "v_min", "v_max");
    overrideBound(config_params, "u", 1, "w_min", "w_max");
    overrideBound(config_params, "du", 0, "a_min", "a_max");
    overrideBound(config_params, "du", 1, "alpha_min", "alpha_max");
  }

  void updatec(double dt, VectorXd x_next) override
  {
    c << getX()(0) - x_next(0) + dt * getU()(0) * cos(getX()(2)),
      getX()(1) - x_next(1) + dt * getU()(0) * sin(getX()(2)),
      getX()(2) - x_next(2) + dt * getU()(1);
  }

  void updateA(double dt) override
  {
    A << 1.0, 0.0, -dt * getU()(0) * sin(getX()(2)),
      0.0, 1.0, +dt * getU()(0) * cos(getX()(2)),
      0.0, 0.0, 1.0;
  }

  void updateB() override
  {
    B << cos(getX()(2)), 0.0,
      sin(getX()(2)), 0.0,
      0.0, 1.0;
  }
};

}  // namespace prox_mpc

#endif  // PROX_MPC__MODELS__UNICYCLE_HPP_
