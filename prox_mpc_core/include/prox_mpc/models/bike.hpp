// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0

#ifndef PROX_MPC__MODELS__BIKE_HPP_
#define PROX_MPC__MODELS__BIKE_HPP_

#include <prox_mpc/model.hpp>
#include <prox_mpc/utils.hpp>

namespace prox_mpc
{

/* Kinematic bicycle model. */
class Bicycle : public Model
{
public:
  Bicycle()
  {
    setName("bike");

    setN(4);  // state: [x, y, theta, delta]
    setM(2);  // control: [v, delta_dot]

    double L = 1.6;  // wheelbase [m]
    VectorXd params(1);
    params << L;
    setParams(params);

    setA(MatrixXd::Zero(getN(), getN()));
    setB(MatrixXd::Zero(getN(), getM()));
    setc(VectorXd::Zero(getN()));

    setIneq("x", 3, -M_PI / 2, M_PI / 2);  // steering angle [rad]
    setIneq("u", 0, -3.0, 3.0);            // linear velocity [m/s]
    setIneq("u", 1, -1., 1.);              // steering rate [rad/s]
    setIneq("du", 0, -.5, .5);             // linear acceleration [m/s^2]
    setIneq("du", 1, -.5, .5);             // steering acceleration [rad/s^2]

    setObsAvoid(true, 0.2);  // Euclidean clearance [m]

    setBoxWidth(1.);
    setBoxLength(2.7);
    setPoseWidth(getBoxWidth() / 2);
    setPoseLength(L);
    setBoxPoints(20);
  }

  void updatec(double dt, VectorXd x_next)
  {
    c << getX()(0) - x_next(0) + dt * getU()(0) * cos(getX()(2) + getX()(3)),
      getX()(1) - x_next(1) + dt * getU()(0) * sin(getX()(2) + getX()(3)),
      getX()(2) - x_next(2) + dt * getU()(0) * sin(getX()(3)) / getParams()[0],
      getX()(3) - x_next(3) + dt * getU()(1);
  }

  void updateA(double dt)
  {
    A << 1.0, 0.0, -dt * getU()(0) * sin(getX()(2) + getX()(3)),
      -dt * getU()(0) * sin(getX()(2) + getX()(3)),
      0.0, 1.0, +dt * getU()(0) * cos(getX()(2) + getX()(3)),
      +dt * getU()(0) * cos(getX()(2) + getX()(3)),
      0.0, 0.0, 1.0, +dt * getU()(0) * cos(getX()(3)) / getParams()[0],
      0.0, 0.0, 0.0, 1.0;
  }

  void updateB()
  {
    B << cos(getX()(2) + getX()(3)), 0.0,
      sin(getX()(2) + getX()(3)), 0.0,
      sin(getX()(3)) / getParams()[0], 0.0,
      0.0, 1.0;
  }
};

}  // namespace prox_mpc

#endif  // PROX_MPC__MODELS__BIKE_HPP_
