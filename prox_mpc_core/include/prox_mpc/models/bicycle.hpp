// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0

#ifndef PROX_MPC__MODELS__BICYCLE_HPP_
#define PROX_MPC__MODELS__BICYCLE_HPP_

#include <map>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist.hpp>

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
    setName("bicycle");

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

    setObsAvoid(true);  // this model supports obstacle avoidance
  }

  /*!
   * Configure the model constants from a params map. Absent keys keep the
   * constructor literals, so an empty map reproduces the hardcoded values.
   * Keys: "L" (wheelbase); bound limits "delta_min"/"delta_max" (x[3]),
   * "v_min"/"v_max" (u[0]), "delta_rate_min"/"delta_rate_max" (u[1]),
   * "a_min"/"a_max" (du[0]), "delta_acc_min"/"delta_acc_max" (du[1]).
   */
  void configure(const std::map<std::string, double> & params) override
  {
    if (params.count("L") > 0) {
      // The wheelbase divides the yaw and steering Jacobians; a non-positive
      // value makes A, B and c non-finite and poisons the whole QP.
      if (!(params.at("L") > 0.0)) {
        throw std::invalid_argument("Bicycle::configure: L must be > 0");
      }
      VectorXd p(1);
      p << params.at("L");
      setParams(p);
    }
    overrideBound(params, "x", 3, "delta_min", "delta_max");
    overrideBound(params, "u", 0, "v_min", "v_max");
    overrideBound(params, "u", 1, "delta_rate_min", "delta_rate_max");
    overrideBound(params, "du", 0, "a_min", "a_max");
    overrideBound(params, "du", 1, "delta_acc_min", "delta_acc_max");
  }

  /*!
   * Map control [v, delta_dot] to a body twist. The bicycle's second control is
   * a steering rate, not a yaw rate, so the yaw rate is derived from the current
   * steering state delta: omega = v * sin(delta) / L. The caller must have set
   * the model state (delta at index 3) to the current state before calling.
   */
  geometry_msgs::msg::Twist toTwist(const VectorXd & u) const override
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = u(0);                                  // forward speed v
    twist.angular.z = u(0) * sin(this->x(3)) / this->params(0);  // omega = v sin(delta)/L
    return twist;
  }

  void updatec(double dt, VectorXd x_next) override
  {
    c << getX()(0) - x_next(0) + dt * getU()(0) * cos(getX()(2) + getX()(3)),
      getX()(1) - x_next(1) + dt * getU()(0) * sin(getX()(2) + getX()(3)),
      getX()(2) - x_next(2) + dt * getU()(0) * sin(getX()(3)) / getParams()[0],
      getX()(3) - x_next(3) + dt * getU()(1);
  }

  void updateA(double dt) override
  {
    A << 1.0, 0.0, -dt * getU()(0) * sin(getX()(2) + getX()(3)),
      -dt * getU()(0) * sin(getX()(2) + getX()(3)),
      0.0, 1.0, +dt * getU()(0) * cos(getX()(2) + getX()(3)),
      +dt * getU()(0) * cos(getX()(2) + getX()(3)),
      0.0, 0.0, 1.0, +dt * getU()(0) * cos(getX()(3)) / getParams()[0],
      0.0, 0.0, 0.0, 1.0;
  }

  void updateB() override
  {
    B << cos(getX()(2) + getX()(3)), 0.0,
      sin(getX()(2) + getX()(3)), 0.0,
      sin(getX()(3)) / getParams()[0], 0.0,
      0.0, 1.0;
  }
};

}  // namespace prox_mpc

#endif  // PROX_MPC__MODELS__BICYCLE_HPP_
