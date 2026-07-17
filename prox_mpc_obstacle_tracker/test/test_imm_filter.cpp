// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Unit tests for the pure-Eigen ImmFilter (IMM CV+CTRV per-track estimator),
// driven only through its public API. The straight-line case checks CV
// equivalence against a reference plain CV Kalman filter (the legacy tracker
// math, re-implemented here); the orbit cases mirror the benchmark's
// dynamic_circle geometry (center (1, 0), r = 0.8 m, v = 0.5 m/s, so
// omega = 0.625 rad/s); the degenerate-branch case exercises the public
// static ctrv_transition / ctrv_transition_jacobian across |omega| = eps.
// Measurements are noiseless, so every case is deterministic.

#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "prox_mpc_obstacle_tracker/imm_filter.hpp"

namespace
{
using prox_mpc_obstacle_tracker::ImmFilter;

constexpr double kDt = 0.1;     // 10 Hz scan cadence [s]

// dynamic_circle geometry (prox_mpc_benchmark obstacle_field.hpp).
constexpr double kOrbitCx = 1.0;
constexpr double kOrbitCy = 0.0;
constexpr double kOrbitR = 0.8;
constexpr double kOrbitV = 0.5;
constexpr double kOrbitOmega = kOrbitV / kOrbitR;   // 0.625 rad/s

Eigen::Vector2d orbit_pos(double t)
{
  return {
    kOrbitCx + kOrbitR * std::cos(kOrbitOmega * t),
    kOrbitCy + kOrbitR * std::sin(kOrbitOmega * t)};
}

// Filter tuning that tracks the CTRV orbit (the config defaults are stickier on
// CV and only reach mu_ctrv ~ 0.5 on this orbit).
ImmFilter::Params orbitParams()
{
  ImmFilter::Params p;
  p.measurement_noise = 4e-4;
  p.process_noise = 0.1;
  p.p_ctrv_stay = 0.99;
  return p;
}

// Reference plain CV Kalman filter: the exact legacy tracker math (discrete
// white-noise acceleration, position-only measurement).
struct ReferenceCvKf
{
  Eigen::Vector4d x{Eigen::Vector4d::Zero()};
  Eigen::Matrix4d p{Eigen::Matrix4d::Identity()};
  double q{1.0};
  double r{0.01};

  void reset(double mx, double my, double initial_velocity_variance)
  {
    x << mx, my, 0.0, 0.0;
    p = Eigen::Matrix4d::Zero();
    p(0, 0) = r;
    p(1, 1) = r;
    p(2, 2) = initial_velocity_variance;
    p(3, 3) = initial_velocity_variance;
  }

  void predict(double dt)
  {
    if (dt <= 0.0) {return;}
    Eigen::Matrix4d f = Eigen::Matrix4d::Identity();
    f(0, 2) = dt;
    f(1, 3) = dt;
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
    x = f * x;
    p = f * p * f.transpose() + qn;
  }

  void update(const Eigen::Vector2d & z)
  {
    Eigen::Matrix<double, 2, 4> h = Eigen::Matrix<double, 2, 4>::Zero();
    h(0, 0) = 1.0;
    h(1, 1) = 1.0;
    const Eigen::Matrix2d rm = r * Eigen::Matrix2d::Identity();
    const Eigen::Vector2d y = z - h * x;
    const Eigen::Matrix2d s = h * p * h.transpose() + rm;
    const Eigen::Matrix<double, 4, 2> k = p * h.transpose() * s.inverse();
    x += k * y;
    p = (Eigen::Matrix4d::Identity() - k * h) * p;
  }
};
}  // namespace

// CV-equivalence: on straight-line motion the IMM combined state converges to
// the reference CV KF fed the same measurements, and the CV model dominates
// (mu_cv >= 0.9) after ~20 updates.
TEST(ImmFilter, MatchesCvKalmanOnStraightMotion)
{
  ImmFilter::Params ip;
  ip.process_noise = 0.01;      // sticky-CV tuning: mu_cv >= 0.9 needs a calm CV model
  ip.measurement_noise = 4e-4;
  ImmFilter imm(ip);

  ReferenceCvKf kf;
  kf.q = ip.process_noise;
  kf.r = ip.measurement_noise;

  const double vx = 1.0;
  const double vy = 0.5;
  imm.reset(0.0, 0.0);
  kf.reset(0.0, 0.0, ip.initial_velocity_variance);

  for (int k = 1; k <= 60; ++k) {
    const double t = static_cast<double>(k) * kDt;
    const Eigen::Vector2d z(vx * t, vy * t);
    imm.predict(kDt);
    imm.update(z);
    kf.predict(kDt);
    kf.update(z);
    if (k >= 20) {
      const Eigen::Vector4d d = imm.combined_state() - kf.x;
      EXPECT_LT(std::hypot(d(0), d(1)), 1e-3) << "position diverged at scan " << k;
      EXPECT_LT(std::hypot(d(2), d(3)), 1e-3) << "velocity diverged at scan " << k;
      EXPECT_GE(imm.model_probabilities()(0), 0.9) << "mu_cv dropped at scan " << k;
    }
  }
}

// CTRV turn-rate convergence on the dynamic_circle orbit: |omega_hat - 0.625|
// <= 0.1 within 30 scans at 10 Hz, and mu_ctrv >= 0.7 once established (it
// must not fall back below afterwards).
TEST(ImmFilter, ConvergesTurnRateOnOrbit)
{
  ImmFilter imm(orbitParams());
  imm.reset(orbit_pos(0.0)(0), orbit_pos(0.0)(1));

  int first_converged = -1;
  bool ctrv_established = false;
  for (int k = 1; k <= 60; ++k) {
    imm.predict(kDt);
    imm.update(orbit_pos(static_cast<double>(k) * kDt));
    const double omega_hat = imm.ctrv_state()(4);
    const double mu_ctrv = imm.model_probabilities()(1);
    if (first_converged < 0 && std::abs(omega_hat - kOrbitOmega) <= 0.1 && mu_ctrv >= 0.7) {
      first_converged = k;
    }
    if (ctrv_established) {
      EXPECT_GE(mu_ctrv, 0.7) << "mu_ctrv fell back below 0.7 at scan " << k;
    }
    ctrv_established = ctrv_established || mu_ctrv >= 0.7;
    if (k == 30) {
      EXPECT_LE(std::abs(omega_hat - kOrbitOmega), 0.1);
      EXPECT_GE(mu_ctrv, 0.7);
    }
  }
  ASSERT_GT(first_converged, 0);
  EXPECT_LE(first_converged, 30);
}

// On the same orbit, the 2.0 s-ahead blended sample lands within 0.2 m of the
// true arc, while the straight ray from the combined CV-space state (what the
// legacy fill would extrapolate) is far off (~0.6 m for this geometry).
TEST(ImmFilter, PredictionBeatsStraightRayOnOrbit)
{
  ImmFilter imm(orbitParams());
  imm.reset(orbit_pos(0.0)(0), orbit_pos(0.0)(1));
  const int scans = 30;
  for (int k = 1; k <= scans; ++k) {
    imm.predict(kDt);
    imm.update(orbit_pos(static_cast<double>(k) * kDt));
  }

  const double t_now = static_cast<double>(scans) * kDt;
  const double horizon = 2.0;
  const std::vector<Eigen::Vector2d> samples = imm.predicted_samples(20, kDt);
  ASSERT_EQ(samples.size(), 20u);
  const Eigen::Vector2d truth = orbit_pos(t_now + horizon);
  const Eigen::Vector2d blended = samples[19];   // sample k = 20 -> t = 2.0 s

  const Eigen::Vector4d st = imm.combined_state();
  const Eigen::Vector2d ray(st(0) + st(2) * horizon, st(1) + st(3) * horizon);

  EXPECT_LE((blended - truth).norm(), 0.2);
  EXPECT_GT((ray - truth).norm(), 0.2);          // the straight ray misses the arc
  EXPECT_LT((blended - truth).norm(), (ray - truth).norm());
}

// The CTRV transition and its Jacobian are continuous across the
// |omega| = eps branch boundary (eps = 1e-4 rad/s): evaluating just above
// (closed form) and just below (straight-line limit) agrees within the
// inherent O(v * eps * dt^2) branch gap.
TEST(ImmFilter, DegenerateOmegaContinuity)
{
  constexpr double kEps = 1e-4;   // must match the implementation's guard
  const double v = 0.5;
  for (const double dt : {0.05, 0.1, 0.2, 0.5}) {
    // Leading branch-gap term is 0.5 * v * eps * dt^2 (transition) and
    // v * eps * dt^2 (Jacobian); 2x headroom on each.
    const double f_tol = v * kEps * dt * dt;
    const double j_tol = 2.0 * v * kEps * dt * dt;
    for (const double theta : {0.0, 0.7, -2.5, 3.0}) {
      for (const double sign : {1.0, -1.0}) {
        ImmFilter::Vector5d above;   // |omega| = eps: closed-form branch
        above << 0.3, -0.2, v, theta, sign * kEps;
        ImmFilter::Vector5d below = above;   // just inside the degenerate branch
        below(4) = sign * std::nextafter(kEps, 0.0);

        const ImmFilter::Vector5d f_gap =
          ImmFilter::ctrv_transition(above, dt) - ImmFilter::ctrv_transition(below, dt);
        const ImmFilter::Matrix5d j_gap =
          ImmFilter::ctrv_transition_jacobian(above, dt) -
          ImmFilter::ctrv_transition_jacobian(below, dt);

        EXPECT_LE(f_gap.cwiseAbs().maxCoeff(), f_tol)
          << "transition gap at dt=" << dt << " theta=" << theta << " sign=" << sign;
        EXPECT_LE(j_gap.cwiseAbs().maxCoeff(), j_tol)
          << "Jacobian gap at dt=" << dt << " theta=" << theta << " sign=" << sign;
      }
    }
  }
}

// mu stays a probability simplex through mixed hit/miss sequences; a missed
// scan (predict-only) leaves mu untouched; the likelihood underflow guard
// keeps the prior mu both when the measurement is absurdly far (likelihoods
// underflow to 0) and when a degenerate all-zero-noise covariance makes the
// innovation covariance singular (det <= 0 -> zero likelihood).
TEST(ImmFilter, ModelProbabilitiesStayNormalized)
{
  ImmFilter imm{ImmFilter::Params{}};
  imm.reset(0.0, 0.0);

  for (int k = 1; k <= 20; ++k) {
    const Eigen::Vector2d mu_prior = imm.model_probabilities();
    imm.predict(kDt);
    if (k % 3 == 0) {
      // Missed scan: predict-only cycle, mu unchanged by contract.
      EXPECT_EQ(imm.model_probabilities()(0), mu_prior(0));
      EXPECT_EQ(imm.model_probabilities()(1), mu_prior(1));
    } else {
      imm.update(Eigen::Vector2d(0.1 * k, 0.05 * k));
    }
    const Eigen::Vector2d mu = imm.model_probabilities();
    EXPECT_GE(mu(0), 0.0);
    EXPECT_LE(mu(0), 1.0);
    EXPECT_GE(mu(1), 0.0);
    EXPECT_LE(mu(1), 1.0);
    EXPECT_NEAR(mu(0) + mu(1), 1.0, 1e-12) << "mu left the simplex at scan " << k;
  }

  // Underflow guard: an absurd measurement drives both Gaussian likelihoods to
  // zero, so the prior mu must be kept exactly.
  const Eigen::Vector2d mu_before = imm.model_probabilities();
  imm.predict(kDt);
  imm.update(Eigen::Vector2d(1e9, 1e9));
  EXPECT_EQ(imm.model_probabilities()(0), mu_before(0));
  EXPECT_EQ(imm.model_probabilities()(1), mu_before(1));

  // Degenerate covariance: all-zero noise makes S singular (det = 0), the
  // likelihood is defined as 0, and the same underflow floor keeps the birth
  // prior [0.5, 0.5].
  ImmFilter::Params zero_noise;
  zero_noise.process_noise = 0.0;
  zero_noise.measurement_noise = 0.0;
  zero_noise.initial_velocity_variance = 0.0;
  zero_noise.ctrv_process_noise_accel = 0.0;
  zero_noise.ctrv_process_noise_yaw_accel = 0.0;
  ImmFilter degenerate(zero_noise);
  degenerate.reset(0.0, 0.0);
  degenerate.predict(kDt);
  degenerate.update(Eigen::Vector2d(0.0, 0.0));
  EXPECT_EQ(degenerate.model_probabilities()(0), 0.5);
  EXPECT_EQ(degenerate.model_probabilities()(1), 0.5);
}

// dt <= 0 predict cycles are no-ops (the legacy non-monotonic-stamp guard) and
// degenerate sampling arguments yield no samples.
TEST(ImmFilter, DegenerateArgumentsAreNoOps)
{
  ImmFilter imm{ImmFilter::Params{}};
  imm.reset(1.0, -2.0);
  for (int k = 1; k <= 5; ++k) {
    imm.predict(kDt);
    imm.update(Eigen::Vector2d(1.0 + 0.05 * k, -2.0));
  }

  const Eigen::Vector4d state_before = imm.combined_state();
  const Eigen::Matrix4d cov_before = imm.combined_covariance();
  const Eigen::Vector2d mu_before = imm.model_probabilities();

  imm.predict(0.0);
  imm.predict(-0.1);

  // Exact: nothing may propagate on a dt <= 0 cycle.
  EXPECT_EQ((imm.combined_state() - state_before).cwiseAbs().maxCoeff(), 0.0);
  EXPECT_EQ((imm.combined_covariance() - cov_before).cwiseAbs().maxCoeff(), 0.0);
  EXPECT_EQ(imm.model_probabilities()(0), mu_before(0));
  EXPECT_EQ(imm.model_probabilities()(1), mu_before(1));

  EXPECT_TRUE(imm.predicted_samples(0, kDt).empty());    // steps == 0: none
  EXPECT_TRUE(imm.predicted_samples(5, 0.0).empty());    // dt <= 0: none
  EXPECT_TRUE(imm.predicted_samples(5, -1.0).empty());
}
