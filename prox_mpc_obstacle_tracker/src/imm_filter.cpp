// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


#include "prox_mpc_obstacle_tracker/imm_filter.hpp"

#include <cmath>
#include <cstddef>
#include <vector>

namespace
{

using Vector5d = prox_mpc_obstacle_tracker::ImmFilter::Vector5d;
using Matrix5d = prox_mpc_obstacle_tracker::ImmFilter::Matrix5d;

/// CTRV straight-line-limit threshold [rad/s]: a numerical guard on the
/// v/omega closed form, not a tuning knob (both branches are continuous here).
constexpr double kOmegaEps = 1e-4;
/// Speed [m/s] below which the CV -> CTRV heading conversion is
/// ill-conditioned (atan2 of a near-zero velocity): the converter then keeps
/// the CTRV prior heading with an uninformative pi^2 variance.
constexpr double kMinHeadingSpeed = 1e-3;
/// Underflow floor on sum_l cbar_l * L_l: below it both models are equally
/// implausible (no information), so the prior model probabilities are kept.
constexpr double kLikelihoodSumFloor = 1e-30;

/// Normalize an angle to (-pi, pi].
double wrap_angle(double a)
{
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a <= 0.0) {a += 2.0 * M_PI;}
  return a - M_PI;
}

/// State converter T_{cv<-ctrv}: [x, y, v, theta, omega] -> [x, y, v cos, v sin].
Eigen::Vector4d ctrv_to_cv(const Vector5d & x)
{
  return {x(0), x(1), x(2) * std::cos(x(3)), x(2) * std::sin(x(3))};
}

/// Jacobian J_{cv<-ctrv} of the converter above.
Eigen::Matrix<double, 4, 5> ctrv_to_cv_jacobian(const Vector5d & x)
{
  const double v = x(2);
  const double sin_t = std::sin(x(3));
  const double cos_t = std::cos(x(3));
  Eigen::Matrix<double, 4, 5> j = Eigen::Matrix<double, 4, 5>::Zero();
  j(0, 0) = 1.0;
  j(1, 1) = 1.0;
  j(2, 2) = cos_t;
  j(2, 3) = -v * sin_t;
  j(3, 2) = sin_t;
  j(3, 3) = v * cos_t;
  return j;
}

/// State/covariance converter T_{ctrv<-cv}:
/// [x, y, vx, vy] -> [x, y, hypot(vx,vy), atan2(vy,vx), 0].
/// CV is by definition the omega = 0 hypothesis, so omega is an explicit zero
/// with ctrv_init_omega_variance injected on its diagonal (its Jacobian row is
/// zero). At v ~ 0 the atan2 heading is ill-conditioned: the converter keeps
/// the CTRV prior heading with a pi^2 variance, and the speed gradient follows
/// that heading (the analytic limit of grad hypot along it).
void cv_to_ctrv(
  const Eigen::Vector4d & x_cv, const Eigen::Matrix4d & p_cv, double prior_theta,
  double init_omega_variance, Vector5d & x_out, Matrix5d & p_out)
{
  const double v = std::hypot(x_cv(2), x_cv(3));
  Eigen::Matrix<double, 5, 4> j = Eigen::Matrix<double, 5, 4>::Zero();
  j(0, 0) = 1.0;
  j(1, 1) = 1.0;
  x_out(0) = x_cv(0);
  x_out(1) = x_cv(1);
  x_out(2) = v;
  x_out(4) = 0.0;
  const bool heading_ok = v >= kMinHeadingSpeed;
  if (heading_ok) {
    x_out(3) = std::atan2(x_cv(3), x_cv(2));
    j(2, 2) = x_cv(2) / v;
    j(2, 3) = x_cv(3) / v;
    j(3, 2) = -x_cv(3) / (v * v);
    j(3, 3) = x_cv(2) / (v * v);
  } else {
    x_out(3) = prior_theta;
    j(2, 2) = std::cos(prior_theta);
    j(2, 3) = std::sin(prior_theta);
  }
  p_out = j * p_cv * j.transpose();
  if (!heading_ok) {p_out(3, 3) = M_PI * M_PI;}
  p_out(4, 4) = init_omega_variance;
}

/// Gaussian measurement likelihood
/// L = exp(-0.5 y^T S^-1 y) / (2 pi sqrt(det S)) for a 2-D innovation.
double gaussian_likelihood(const Eigen::Vector2d & y, const Eigen::Matrix2d & s)
{
  const double det = s.determinant();
  if (!(det > 0.0) || !std::isfinite(det)) {return 0.0;}
  const double md2 = y.dot(s.inverse() * y);
  const double l = std::exp(-0.5 * md2) / (2.0 * M_PI * std::sqrt(det));
  return std::isfinite(l) ? l : 0.0;
}

}  // namespace

namespace prox_mpc_obstacle_tracker
{

ImmFilter::Vector5d ImmFilter::ctrv_transition(const Vector5d & x, double dt)
{
  const double v = x(2);
  const double theta = x(3);
  const double omega = x(4);
  const double theta_plus = theta + omega * dt;
  Vector5d out = x;
  if (std::abs(omega) >= kOmegaEps) {
    out(0) = x(0) + (v / omega) * (std::sin(theta_plus) - std::sin(theta));
    out(1) = x(1) + (v / omega) * (-std::cos(theta_plus) + std::cos(theta));
  } else {
    out(0) = x(0) + v * std::cos(theta) * dt;
    out(1) = x(1) + v * std::sin(theta) * dt;
  }
  out(3) = wrap_angle(theta_plus);
  return out;
}

ImmFilter::Matrix5d ImmFilter::ctrv_transition_jacobian(const Vector5d & x, double dt)
{
  const double v = x(2);
  const double theta = x(3);
  const double omega = x(4);
  const double sin_t = std::sin(theta);
  const double cos_t = std::cos(theta);
  Matrix5d f = Matrix5d::Identity();
  f(3, 4) = dt;
  if (std::abs(omega) >= kOmegaEps) {
    const double theta_plus = theta + omega * dt;
    const double sin_p = std::sin(theta_plus);
    const double cos_p = std::cos(theta_plus);
    f(0, 2) = (sin_p - sin_t) / omega;
    f(0, 3) = (v / omega) * (cos_p - cos_t);
    f(0, 4) = (v * dt / omega) * cos_p - (v / (omega * omega)) * (sin_p - sin_t);
    f(1, 2) = (cos_t - cos_p) / omega;
    f(1, 3) = (v / omega) * (sin_p - sin_t);
    f(1, 4) = (v * dt / omega) * sin_p - (v / (omega * omega)) * (cos_t - cos_p);
  } else {
    f(0, 2) = dt * cos_t;
    f(0, 3) = -v * dt * sin_t;
    f(0, 4) = -0.5 * v * dt * dt * sin_t;
    f(1, 2) = dt * sin_t;
    f(1, 3) = v * dt * cos_t;
    f(1, 4) = 0.5 * v * dt * dt * cos_t;
  }
  return f;
}

void ImmFilter::reset(double x, double y)
{
  x_cv_ << x, y, 0.0, 0.0;
  p_cv_ = Eigen::Matrix4d::Zero();
  p_cv_(0, 0) = params_.measurement_noise;
  p_cv_(1, 1) = params_.measurement_noise;
  p_cv_(2, 2) = params_.initial_velocity_variance;
  p_cv_(3, 3) = params_.initial_velocity_variance;

  x_ctrv_ << x, y, 0.0, 0.0, 0.0;
  p_ctrv_ = Matrix5d::Zero();
  p_ctrv_(0, 0) = params_.measurement_noise;
  p_ctrv_(1, 1) = params_.measurement_noise;
  p_ctrv_(2, 2) = params_.initial_velocity_variance;
  p_ctrv_(3, 3) = M_PI * M_PI;  // heading unobservable at v = 0: uninformative prior
  p_ctrv_(4, 4) = params_.ctrv_init_omega_variance;

  mu_ << 0.5, 0.5;
  cbar_ = mu_;
}

void ImmFilter::predict(double dt)
{
  if (dt <= 0.0) {
    // No time elapsed: the Markov transition degenerates to the identity, so
    // mixing is the identity and neither model propagates (this also guards
    // against a non-monotonic stamp). cbar = mu keeps the update consistent.
    cbar_ = mu_;
    return;
  }

  // Markov transition matrix, rows CV/CTRV; rows sum to 1 by construction.
  Eigen::Matrix2d pi;
  pi << params_.p_cv_stay, 1.0 - params_.p_cv_stay,
    1.0 - params_.p_ctrv_stay, params_.p_ctrv_stay;

  // Mixing: cbar_j = sum_i PI(i,j) mu_i; mu_{i|j} = PI(i,j) mu_i / cbar_j.
  cbar_ = pi.transpose() * mu_;
  Eigen::Matrix2d mix;  // mix(i, j) = mu_{i|j}
  for (int j = 0; j < 2; ++j) {
    for (int i = 0; i < 2; ++i) {
      mix(i, j) = pi(i, j) * mu_(i) / cbar_(j);
    }
  }

  // Mixed prior for the CV model (CTRV contribution via T_{cv<-ctrv} / J).
  const Eigen::Vector4d x_ctrv_in_cv = ctrv_to_cv(x_ctrv_);
  const Eigen::Matrix<double, 4, 5> j_cv = ctrv_to_cv_jacobian(x_ctrv_);
  const Eigen::Vector4d x0_cv = mix(0, 0) * x_cv_ + mix(1, 0) * x_ctrv_in_cv;
  const Eigen::Vector4d d_cv_cv = x_cv_ - x0_cv;
  const Eigen::Vector4d d_ctrv_cv = x_ctrv_in_cv - x0_cv;
  const Eigen::Matrix4d p0_cv =
    mix(0, 0) * (p_cv_ + d_cv_cv * d_cv_cv.transpose()) +
    mix(1, 0) * (j_cv * p_ctrv_ * j_cv.transpose() + d_ctrv_cv * d_ctrv_cv.transpose());

  // Mixed prior for the CTRV model (CV contribution via T_{ctrv<-cv} / J).
  Vector5d x_cv_in_ctrv;
  Matrix5d p_cv_in_ctrv;
  cv_to_ctrv(
    x_cv_, p_cv_, x_ctrv_(3), params_.ctrv_init_omega_variance, x_cv_in_ctrv, p_cv_in_ctrv);
  Vector5d x0_ctrv = mix(0, 1) * x_cv_in_ctrv + mix(1, 1) * x_ctrv_;
  // The heading blend is evaluated in a chart centered on the CTRV prior
  // heading (wrapped residuals), so straddling the +-pi seam cannot average
  // two near-pi headings to ~0; the result is re-normalized to (-pi, pi].
  const double theta_ref = x_ctrv_(3);
  x0_ctrv(3) = wrap_angle(
    theta_ref + mix(0, 1) * wrap_angle(x_cv_in_ctrv(3) - theta_ref) +
    mix(1, 1) * wrap_angle(x_ctrv_(3) - theta_ref));
  Vector5d d_cv_ctrv = x_cv_in_ctrv - x0_ctrv;
  d_cv_ctrv(3) = wrap_angle(x_cv_in_ctrv(3) - x0_ctrv(3));
  Vector5d d_ctrv_ctrv = x_ctrv_ - x0_ctrv;
  d_ctrv_ctrv(3) = wrap_angle(x_ctrv_(3) - x0_ctrv(3));
  const Matrix5d p0_ctrv =
    mix(0, 1) * (p_cv_in_ctrv + d_cv_ctrv * d_cv_ctrv.transpose()) +
    mix(1, 1) * (p_ctrv_ + d_ctrv_ctrv * d_ctrv_ctrv.transpose());

  // CV predict: discrete white-noise acceleration process model, spectral
  // density q = process_noise.
  Eigen::Matrix4d f = Eigen::Matrix4d::Identity();
  f(0, 2) = dt;
  f(1, 3) = dt;
  const double q = params_.process_noise;
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt2 * dt2;
  Eigen::Matrix4d qn = Eigen::Matrix4d::Zero();
  qn(0, 0) = dt4 / 4.0 * q;
  qn(0, 2) = dt3 / 2.0 * q;
  qn(2, 0) = dt3 / 2.0 * q;
  qn(2, 2) = dt2 * q;
  qn(1, 1) = dt4 / 4.0 * q;
  qn(1, 3) = dt3 / 2.0 * q;
  qn(3, 1) = dt3 / 2.0 * q;
  qn(3, 3) = dt2 * q;
  x_cv_ = f * x0_cv;
  p_cv_ = f * p0_cv * f.transpose() + qn;

  // CTRV predict (EKF): closed-form transition, Jacobian, and the discrete
  // white-noise acceleration Q via the G matrix, evaluated at the pre-predict
  // heading.
  const double theta_pre = x0_ctrv(3);
  const Matrix5d f_ctrv = ctrv_transition_jacobian(x0_ctrv, dt);
  x_ctrv_ = ctrv_transition(x0_ctrv, dt);
  Eigen::Matrix<double, 5, 2> g = Eigen::Matrix<double, 5, 2>::Zero();
  g(0, 0) = 0.5 * dt2 * std::cos(theta_pre);
  g(1, 0) = 0.5 * dt2 * std::sin(theta_pre);
  g(2, 0) = dt;
  g(3, 1) = 0.5 * dt2;
  g(4, 1) = dt;
  Eigen::Matrix2d noise = Eigen::Matrix2d::Zero();
  noise(0, 0) = params_.ctrv_process_noise_accel;
  noise(1, 1) = params_.ctrv_process_noise_yaw_accel;
  p_ctrv_ = f_ctrv * p0_ctrv * f_ctrv.transpose() + g * noise * g.transpose();
}

void ImmFilter::update(const Eigen::Vector2d & z)
{
  const Eigen::Matrix2d r = params_.measurement_noise * Eigen::Matrix2d::Identity();

  // CV update: linear Kalman update with a position-only measurement model.
  Eigen::Matrix<double, 2, 4> h_cv = Eigen::Matrix<double, 2, 4>::Zero();
  h_cv(0, 0) = 1.0;
  h_cv(1, 1) = 1.0;
  const Eigen::Vector2d y_cv = z - h_cv * x_cv_;
  const Eigen::Matrix2d s_cv = h_cv * p_cv_ * h_cv.transpose() + r;
  const Eigen::Matrix<double, 4, 2> k_cv = p_cv_ * h_cv.transpose() * s_cv.inverse();
  x_cv_ += k_cv * y_cv;
  p_cv_ = (Eigen::Matrix4d::Identity() - k_cv * h_cv) * p_cv_;
  const double l_cv = gaussian_likelihood(y_cv, s_cv);

  // CTRV update: linear measurement model, so the EKF update is exact.
  Eigen::Matrix<double, 2, 5> h_ctrv = Eigen::Matrix<double, 2, 5>::Zero();
  h_ctrv(0, 0) = 1.0;
  h_ctrv(1, 1) = 1.0;
  const Eigen::Vector2d y_ctrv = z - h_ctrv * x_ctrv_;
  const Eigen::Matrix2d s_ctrv = h_ctrv * p_ctrv_ * h_ctrv.transpose() + r;
  const Eigen::Matrix<double, 5, 2> k_ctrv = p_ctrv_ * h_ctrv.transpose() * s_ctrv.inverse();
  x_ctrv_ += k_ctrv * y_ctrv;
  x_ctrv_(3) = wrap_angle(x_ctrv_(3));
  p_ctrv_ = (Matrix5d::Identity() - k_ctrv * h_ctrv) * p_ctrv_;
  const double l_ctrv = gaussian_likelihood(y_ctrv, s_ctrv);

  // Model-probability update mu_j = cbar_j L_j / sum_l cbar_l L_l, keeping the
  // prior mu on underflow (both models equally implausible: no information).
  const double num_cv = cbar_(0) * l_cv;
  const double num_ctrv = cbar_(1) * l_ctrv;
  const double sum = num_cv + num_ctrv;
  if (sum >= kLikelihoodSumFloor) {
    mu_(0) = num_cv / sum;
    mu_(1) = num_ctrv / sum;
  }
}

Eigen::Vector4d ImmFilter::combined_state() const
{
  return mu_(0) * x_cv_ + mu_(1) * ctrv_to_cv(x_ctrv_);
}

Eigen::Matrix4d ImmFilter::combined_covariance() const
{
  const Eigen::Vector4d x_ctrv_cv = ctrv_to_cv(x_ctrv_);
  const Eigen::Vector4d x_out = mu_(0) * x_cv_ + mu_(1) * x_ctrv_cv;
  const Eigen::Matrix<double, 4, 5> j = ctrv_to_cv_jacobian(x_ctrv_);
  const Eigen::Vector4d d_cv = x_cv_ - x_out;
  const Eigen::Vector4d d_ctrv = x_ctrv_cv - x_out;
  return mu_(0) * (p_cv_ + d_cv * d_cv.transpose()) +
         mu_(1) * (j * p_ctrv_ * j.transpose() + d_ctrv * d_ctrv.transpose());
}

std::vector<Eigen::Vector2d> ImmFilter::predicted_samples(std::size_t steps, double dt) const
{
  std::vector<Eigen::Vector2d> out;
  if (steps == 0 || !(dt > 0.0)) {return out;}
  out.reserve(steps);
  for (std::size_t k = 1; k <= steps; ++k) {
    const double t = static_cast<double>(k) * dt;
    // Noise-free propagation of each model's own post-update state; the CTRV
    // closed form is exact for any horizon, so one step by t equals k of dt.
    const double cv_x = x_cv_(0) + x_cv_(2) * t;
    const double cv_y = x_cv_(1) + x_cv_(3) * t;
    const Vector5d ctrv = ctrv_transition(x_ctrv_, t);
    out.emplace_back(mu_(0) * cv_x + mu_(1) * ctrv(0), mu_(0) * cv_y + mu_(1) * ctrv(1));
  }
  return out;
}

}  // namespace prox_mpc_obstacle_tracker
