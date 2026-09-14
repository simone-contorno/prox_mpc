// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0

#ifndef PROX_MPC__MODELS__BICYCLE_FRONT_AXLE_HPP_
#define PROX_MPC__MODELS__BICYCLE_FRONT_AXLE_HPP_

#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist.hpp>

#include <prox_mpc/model.hpp>
#include <prox_mpc/structs.hpp>
#include <prox_mpc/utils.hpp>

namespace prox_mpc
{

/// Kinematic bicycle referenced to the FRONT axle.
///
/// State [x, y, theta, delta] (n=4) and control [v, delta_dot] (m=2), where
/// (x, y) is the front-axle centre, theta the body heading, delta the steering
/// angle and v the front-wheel speed. The front wheel travels along
/// theta + delta, so the propagation is
///
///   x_dot = v cos(theta + delta), y_dot = v sin(theta + delta),
///   theta_dot = v sin(delta) / L, delta_dot = u[1].
///
/// Every consumer-facing quantity refers to that same front-axle point:
/// getPlanarMapping() declares the reference point at (L, 0) in base_link, so a
/// consumer transforms the pose back to base_link before a footprint check, and
/// toTwist() reports the body twist of base_link, whose longitudinal speed is
/// v cos(delta) rather than the front-wheel speed v.
///
/// base_link is assumed to sit on the rear axle, which is the ROS convention for
/// a car-like base and what the declared reference offset encodes.
class BicycleFrontAxle : public Model
{
public:
  BicycleFrontAxle()
  {
    setName("bicycle_front_axle");

    setN(4);  // state: [x, y, theta, delta]
    setM(2);  // control: [v, delta_dot]

    double L = 1.6;  // wheelbase [m]
    VectorXd init_params(1);
    init_params << L;
    setParams(init_params);

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
  void configure(const std::map<std::string, double> & config_params) override
  {
    if (config_params.count("L") > 0) {
      // The wheelbase divides the yaw and steering Jacobians; a non-positive
      // value makes A, B and c non-finite and poisons the whole QP.
      if (!(config_params.at("L") > 0.0)) {
        throw std::invalid_argument("BicycleFrontAxle::configure: L must be > 0");
      }
      VectorXd p(1);
      p << config_params.at("L");
      setParams(p);
    }
    overrideBound(config_params, "x", 3, "delta_min", "delta_max");
    overrideBound(config_params, "u", 0, "v_min", "v_max");
    overrideBound(config_params, "u", 1, "delta_rate_min", "delta_rate_max");
    overrideBound(config_params, "du", 0, "a_min", "a_max");
    overrideBound(config_params, "du", 1, "delta_acc_min", "delta_acc_max");
  }

  /*!
   * The state's (x, y) is the front axle, one wheelbase ahead of base_link on
   * the body x axis, the steering angle is state index 3 and its rate is
   * control index 1.
   */
  PlanarMapping getPlanarMapping() const override
  {
    PlanarMapping mapping;
    mapping.idx_steering = 3;
    mapping.idx_steer_rate = 1;
    mapping.ref_offset_x = this->params(0);
    mapping.wheelbase = this->params(0);
    return mapping;
  }

  /*!
   * Map control [v, delta_dot] to the body twist of base_link. v is the
   * front-wheel speed, whose projection on the body x axis is v cos(delta); the
   * yaw rate is v sin(delta) / L. The caller must have set the model state
   * (delta at index 3) to the current state before calling.
   */
  geometry_msgs::msg::Twist toTwist(const VectorXd & u_in) const override
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = u_in(0) * cos(this->x(3));                     // base_link speed
    twist.angular.z = u_in(0) * sin(this->x(3)) / this->params(0);  // omega = v sin(delta)/L
    return twist;
  }

  /*!
   * Recover the front-wheel speed from a base_link twist. toTwist() emits
   * linear.x = v cos(delta) and angular.z = v sin(delta) / L, so
   * v = sign(linear.x) sqrt(linear.x^2 + (angular.z L)^2) inverts the pair
   * without dividing by cos(delta), which vanishes at this model's own
   * +/- pi/2 steering bound. The projection carries no direction at
   * linear.x == 0, where the sign is taken as forward.
   *
   * The steering rate is left undetermined: a twist shows the steering angle's
   * effect, not the rate the angle is changing at.
   */
  VectorXd fromTwist(const geometry_msgs::msg::Twist & twist) const override
  {
    VectorXd u_out = VectorXd::Constant(2, std::numeric_limits<double>::quiet_NaN());
    const double wheel_arc = twist.angular.z * this->params(0);
    const double speed = std::sqrt(twist.linear.x * twist.linear.x + wheel_arc * wheel_arc);
    u_out(0) = (twist.linear.x < 0.0) ? -speed : speed;
    return u_out;
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

#endif  // PROX_MPC__MODELS__BICYCLE_FRONT_AXLE_HPP_
