// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


#ifndef PROX_MPC_OBSTACLE_TRACKER__IMM_FILTER_HPP_
#define PROX_MPC_OBSTACLE_TRACKER__IMM_FILTER_HPP_

#include <cstddef>
#include <vector>

#include <Eigen/Dense>

namespace prox_mpc_obstacle_tracker
{

/// Per-track Interacting Multiple Model (IMM) estimator running a
/// constant-velocity (CV) linear Kalman filter and a constant-turn-rate-and-
/// velocity (CTRV) EKF in parallel. Pure (Eigen only, no ROS) so it is
/// unit-testable without ROS. All filter state is double.
///
/// Model states: CV [x, y, vx, vy] (m, m, m/s, m/s); CTRV
/// [x, y, v, theta, omega] (m, m, m/s, rad, rad/s), theta normalized to
/// (-pi, pi]. The measurement for both models is the planar centroid
/// z = [x, y] in the tracking frame with R = measurement_noise * I2.
///
/// One IMM cycle per tracker update: predict(dt) mixes the model states
/// (Markov transition + moment-matched mixed priors) and runs both model
/// predicts; update(z) runs both model updates, computes the Gaussian
/// likelihoods, and updates the model probabilities. On a missed scan the
/// cycle is predict-only and the model probabilities are left unchanged.
/// The combined output is moment-matched in CV space, so the published
/// position/velocity/covariance are always [x, y, vx, vy] and a 4x4
/// covariance, regardless of which model currently dominates.
class ImmFilter
{
public:
  using Vector5d = Eigen::Matrix<double, 5, 1>;
  using Matrix5d = Eigen::Matrix<double, 5, 5>;

  struct Params
  {
    double process_noise{1.0};                // CV accel spectral density q [m^2/s^4]
    double measurement_noise{0.01};           // position measurement variance [m^2]
    double initial_velocity_variance{1.0};    // birth vx/vy (CV) and v (CTRV) variance [m^2/s^2]
    double p_cv_stay{0.95};                   // Markov P(CV -> CV), in (0, 1) exclusive
    double p_ctrv_stay{0.95};                 // Markov P(CTRV -> CTRV), in (0, 1) exclusive
    double ctrv_process_noise_accel{1.0};     // sigma_a^2 [m^2/s^4] (discrete white-noise form)
    double ctrv_process_noise_yaw_accel{1.0}; // sigma_wd^2 [rad^2/s^4]
    double ctrv_init_omega_variance{1.0};     // omega variance at birth / CV->CTRV mix [rad^2/s^2]
  };

  ImmFilter() = default;

  explicit ImmFilter(const Params & params)
  : params_(params) {}

  /// Track birth at measurement (x, y) [m]: CV part with zero velocity and
  /// measurement/initial-velocity variances; CTRV part with zero speed and
  /// turn rate, heading variance pi^2 (heading is unobservable at v = 0), and
  /// omega variance ctrv_init_omega_variance. Model probabilities [0.5, 0.5].
  void reset(double x, double y);

  /// First half of one IMM cycle: mix the model states (Markov transition,
  /// mixing probabilities, moment-matched mixed priors via the cross-model
  /// state converters), then run both model predicts by dt [s]. dt <= 0 is
  /// the no-time-elapsed limit (identity transition), so it is a no-op -
  /// this also guards against a non-monotonic stamp.
  void predict(double dt);

  /// Second half of one IMM cycle: both model updates with the measurement
  /// z = [x, y] [m], Gaussian likelihoods, and the model-probability update.
  /// Skip this call on a missed scan (predict-only cycle, mu unchanged).
  void update(const Eigen::Vector2d & z);

  /// Moment-matched combined state in CV space [x, y, vx, vy]; its position
  /// drives association gating.
  Eigen::Vector4d combined_state() const;

  /// Moment-matched combined 4x4 covariance in CV space.
  Eigen::Matrix4d combined_covariance() const;

  /// Model probabilities [mu_cv, mu_ctrv].
  const Eigen::Vector2d & model_probabilities() const {return mu_;}

  /// CV model state [x, y, vx, vy] (read-only test/diagnostic visibility).
  const Eigen::Vector4d & cv_state() const {return x_cv_;}

  /// CTRV model state [x, y, v, theta, omega] (read-only test/diagnostic
  /// visibility; omega is the estimated turn rate [rad/s]).
  const Vector5d & ctrv_state() const {return x_ctrv_;}

  /// Predicted centroid positions: sample k (1-based, k = 1..steps) is the
  /// mu-weighted blend of each model's noise-free propagation of its own
  /// post-update state by k * dt [s] (CTRV uses the closed-form transition,
  /// including the degenerate low-omega branch). Empty when steps == 0 or
  /// dt <= 0.
  std::vector<Eigen::Vector2d> predicted_samples(std::size_t steps, double dt) const;

  /// Noise-free CTRV closed-form transition f(x, dt), exact for any dt, with
  /// the |omega| < eps straight-line degenerate branch (eps = 1e-4 rad/s, a
  /// numerical guard, not a parameter); theta is re-normalized to (-pi, pi].
  /// Pure and static so the branch continuity is directly testable.
  static Vector5d ctrv_transition(const Vector5d & x, double dt);

  /// CTRV transition Jacobian df/dx; the degenerate branch uses the analytic
  /// omega -> 0 limits (second-order domega terms kept so omega stays
  /// observable near zero). Continuous with the main branch at |omega| = eps.
  static Matrix5d ctrv_transition_jacobian(const Vector5d & x, double dt);

private:
  Params params_;

  Eigen::Vector4d x_cv_{Eigen::Vector4d::Zero()};
  Eigen::Matrix4d p_cv_{Eigen::Matrix4d::Identity()};
  Vector5d x_ctrv_{Vector5d::Zero()};
  Matrix5d p_ctrv_{Matrix5d::Identity()};

  Eigen::Vector2d mu_{Eigen::Vector2d::Constant(0.5)};    // [mu_cv, mu_ctrv]
  Eigen::Vector2d cbar_{Eigen::Vector2d::Constant(0.5)};  // predicted model probabilities
};

}  // namespace prox_mpc_obstacle_tracker

#endif  // PROX_MPC_OBSTACLE_TRACKER__IMM_FILTER_HPP_
