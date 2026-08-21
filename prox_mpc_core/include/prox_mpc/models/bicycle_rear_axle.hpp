// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0

#ifndef PROX_MPC__MODELS__BICYCLE_REAR_AXLE_HPP_
#define PROX_MPC__MODELS__BICYCLE_REAR_AXLE_HPP_

#include <cmath>
#include <map>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist.hpp>

#include <prox_mpc/model.hpp>
#include <prox_mpc/structs.hpp>
#include <prox_mpc/utils.hpp>

namespace prox_mpc
{

/// Kinematic bicycle referenced to the REAR axle.
///
/// State [x, y, theta, delta] (n=4) and control [v, delta_dot] (m=2), where
/// (x, y) is the rear-axle centre, theta the body heading, delta the steering
/// angle and v the rear-axle (body) speed. The rear axle travels along theta, so
/// the propagation is
///
///   x_dot = v cos(theta), y_dot = v sin(theta),
///   theta_dot = v tan(delta) / L, delta_dot = u[1].
///
/// Every consumer-facing quantity refers to that same rear-axle point:
/// getPlanarMapping() declares the reference point at the base_link origin, so
/// the pose needs no transform before a footprint check, and toTwist() reports
/// the body twist of that point directly.
///
/// base_link is assumed to sit on the rear axle, which is the ROS convention for
/// a car-like base.
class BicycleRearAxle : public Model
{
public:
  /* The rear-axle yaw law v tan(delta) / L diverges as |delta| approaches
   * pi/2, so the steering-angle bound is capped strictly inside it. */
  static constexpr double kMaxSteerAngle = 1.0;  // [rad]

  BicycleRearAxle()
  {
    setName("bicycle_rear_axle");

    setN(4);  // state: [x, y, theta, delta]
    setM(2);  // control: [v, delta_dot]

    double L = 1.6;  // wheelbase [m]
    VectorXd params(1);
    params << L;
    setParams(params);

    setA(MatrixXd::Zero(getN(), getN()));
    setB(MatrixXd::Zero(getN(), getM()));
    setc(VectorXd::Zero(getN()));

    setIneq("x", 3, -kMaxSteerAngle, kMaxSteerAngle);  // steering angle [rad]
    setIneq("u", 0, -3.0, 3.0);                        // linear velocity [m/s]
    setIneq("u", 1, -1., 1.);                          // steering rate [rad/s]
    setIneq("du", 0, -.5, .5);                         // linear acceleration [m/s^2]
    setIneq("du", 1, -.5, .5);                         // steering acceleration [rad/s^2]

    setObsAvoid(true);  // this model supports obstacle avoidance
  }

  /*!
   * Configure the model constants from a params map. Absent keys keep the
   * constructor literals, so an empty map reproduces the hardcoded values.
   * Keys: "L" (wheelbase); bound limits "delta_min"/"delta_max" (x[3]),
   * "v_min"/"v_max" (u[0]), "delta_rate_min"/"delta_rate_max" (u[1]),
   * "a_min"/"a_max" (du[0]), "delta_acc_min"/"delta_acc_max" (du[1]).
   * A steering bound beyond kMaxSteerAngle is rejected rather than clamped: the
   * yaw law is unbounded there, so accepting it would poison A, B and c.
   */
  void configure(const std::map<std::string, double> & params) override
  {
    if (params.count("L") > 0) {
      // The wheelbase divides the yaw and steering Jacobians; a non-positive
      // value makes A, B and c non-finite and poisons the whole QP.
      if (!(params.at("L") > 0.0)) {
        throw std::invalid_argument("BicycleRearAxle::configure: L must be > 0");
      }
      VectorXd p(1);
      p << params.at("L");
      setParams(p);
    }
    /* Checked before the magnitude test below, which is a bare ">" and so is
     * false for NaN: without this a non-finite bound would pass straight into
     * overrideBound and from there into every state-bound row of the QP. */
    for (const char * key : {"delta_min", "delta_max"}) {
      const auto it = params.find(key);
      if (it != params.end() && !std::isfinite(it->second)) {
        throw std::invalid_argument(
                "BicycleRearAxle::configure: the steering-angle bound must be finite");
      }
    }
    if (steerBoundExceeded(params, "delta_min") || steerBoundExceeded(params, "delta_max")) {
      throw std::invalid_argument(
              "BicycleRearAxle::configure: the steering-angle bound must not exceed "
              "1.0 rad in magnitude");
    }
    overrideBound(params, "x", 3, "delta_min", "delta_max");
    overrideBound(params, "u", 0, "v_min", "v_max");
    overrideBound(params, "u", 1, "delta_rate_min", "delta_rate_max");
    overrideBound(params, "du", 0, "a_min", "a_max");
    overrideBound(params, "du", 1, "delta_acc_min", "delta_acc_max");
  }

  /*!
   * The state's (x, y) is the rear axle, which is the base_link origin, and the
   * steering angle is state index 3, with its rate at control index 1.
   */
  PlanarMapping getPlanarMapping() const override
  {
    PlanarMapping mapping;
    mapping.idx_steering = 3;
    mapping.idx_steer_rate = 1;
    mapping.wheelbase = this->params(0);
    return mapping;
  }

  /*!
   * Map control [v, delta_dot] to the body twist of base_link. v is already the
   * rear-axle speed, and the yaw rate is v tan(delta) / L. The caller must have
   * set the model state (delta at index 3) to the current state before calling.
   */
  geometry_msgs::msg::Twist toTwist(const VectorXd & u) const override
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = u(0);                                       // base_link speed
    twist.angular.z = u(0) * tan(this->x(3)) / this->params(0);  // omega = v tan(delta)/L
    return twist;
  }

  void updatec(double dt, VectorXd x_next) override
  {
    c << getX()(0) - x_next(0) + dt * getU()(0) * cos(getX()(2)),
      getX()(1) - x_next(1) + dt * getU()(0) * sin(getX()(2)),
      getX()(2) - x_next(2) + dt * getU()(0) * tan(getX()(3)) / getParams()[0],
      getX()(3) - x_next(3) + dt * getU()(1);
  }

  void updateA(double dt) override
  {
    const double sec2 = 1.0 / (cos(getX()(3)) * cos(getX()(3)));
    A << 1.0, 0.0, -dt * getU()(0) * sin(getX()(2)), 0.0,
      0.0, 1.0, +dt * getU()(0) * cos(getX()(2)), 0.0,
      0.0, 0.0, 1.0, +dt * getU()(0) * sec2 / getParams()[0],
      0.0, 0.0, 0.0, 1.0;
  }

  void updateB() override
  {
    B << cos(getX()(2)), 0.0,
      sin(getX()(2)), 0.0,
      tan(getX()(3)) / getParams()[0], 0.0,
      0.0, 1.0;
  }

private:
  /* True when the params map carries `key` with a magnitude past the bound the
   * yaw law stays finite on. */
  static bool steerBoundExceeded(const std::map<std::string, double> & params, std::string key)
  {
    const auto it = params.find(key);
    return it != params.end() && std::abs(it->second) > kMaxSteerAngle;
  }
};

}  // namespace prox_mpc

#endif  // PROX_MPC__MODELS__BICYCLE_REAR_AXLE_HPP_
