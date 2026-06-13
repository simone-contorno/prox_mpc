// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0

#ifndef PROX_MPC__MODELS__R2D2_HPP_
#define PROX_MPC__MODELS__R2D2_HPP_

#include <prox_mpc/model.hpp>
#include <prox_mpc/utils.hpp>

namespace prox_mpc
{

/* Unicycle (differential-drive) kinematic model. */
class Unicycle : public Model
{
public:
  Unicycle()
  {
    setName("r2d2");

    setN(3);  // state: [x, y, theta]
    setM(2);  // control: [v, omega]

    double L = 0.0;
    VectorXd params(1);
    params << L;
    setParams(params);

    setA(MatrixXd::Zero(getN(), getN()));
    setB(MatrixXd::Zero(getN(), getM()));
    setc(VectorXd::Zero(getN()));

    setIneq("u", 0, -3.0, 3.0);  // linear velocity [m/s]
    setIneq("u", 1, -1., 1.);    // angular velocity [rad/s]
    setIneq("du", 0, -.5, .5);   // linear acceleration [m/s^2]
    setIneq("du", 1, -.5, .5);   // angular acceleration [rad/s^2]

    setObsAvoid(true, 0.2);  // Euclidean clearance [m]

    setBoxWidth(.5);
    setBoxLength(.4);
    setPoseWidth(getBoxWidth() / 2);
    setPoseLength(getBoxLength() / 2);
    setBoxPoints(20);
  }

  void updatec(double dt, VectorXd x_next)
  {
    c << getX()(0) - x_next(0) + dt * getU()(0) * cos(getX()(2)),
      getX()(1) - x_next(1) + dt * getU()(0) * sin(getX()(2)),
      getX()(2) - x_next(2) + dt * getU()(1);
  }

  void updateA(double dt)
  {
    A << 1.0, 0.0, -dt * getU()(0) * sin(getX()(2)),
      0.0, 1.0, +dt * getU()(0) * cos(getX()(2)),
      0.0, 0.0, 1.0;
  }

  void updateB()
  {
    B << cos(getX()(2)), 0.0,
      sin(getX()(2)), 0.0,
      0.0, 1.0;
  }
};

}  // namespace prox_mpc

#endif  // PROX_MPC__MODELS__R2D2_HPP_
