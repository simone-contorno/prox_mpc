// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


#ifndef PROX_MPC_OBSTACLE_TRACKER__TRACKER_HPP_
#define PROX_MPC_OBSTACLE_TRACKER__TRACKER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Dense>

#include "prox_mpc_obstacle_tracker/clustering.hpp"
#include "prox_mpc_obstacle_tracker/imm_filter.hpp"

namespace prox_mpc_obstacle_tracker
{

/// One tracked obstacle. state/cov are always the CV-space [x, y, vx, vy]
/// estimate and its 4x4 covariance in the tracking frame: the IMM's
/// moment-matched combined output when IMM is enabled, or the bare CV Kalman
/// state on the legacy path - so association gating and publishing read the
/// same fields either way.
struct Track
{
  std::uint32_t id{0};
  Eigen::Vector4d state{Eigen::Vector4d::Zero()};
  Eigen::Matrix4d cov{Eigen::Matrix4d::Identity()};
  ImmFilter filter;         // per-track IMM(CV+CTRV) estimator (unused on the legacy path)
  double radius{0.0};       // smoothed enclosing radius [m]
  int hits{0};              // consecutive associated scans
  int misses{0};            // consecutive missed scans
  bool confirmed{false};    // promoted after confirm_count consecutive hits
};

/// In-house multi-object tracker: gated greedy nearest-neighbour association,
/// one filter per track (IMM CV+CTRV by default, single-CV Kalman when
/// imm_enabled is false), and a birth/death lifecycle. Pure (Eigen only, no
/// ROS) so the pipeline is unit-testable. Measurements are cluster centroids
/// already transformed into a fixed, non-rotating tracking frame; stamps are
/// absolute seconds.
class Tracker
{
public:
  struct Params
  {
    double process_noise{1.0};               // CV accel spectral density q [m^2/s^4]
    double measurement_noise{0.01};          // position measurement variance [m^2]
    double association_gate{0.5};            // max association distance [m]
    double initial_velocity_variance{1.0};   // initial vx/vy variance [m^2/s^2]
    int confirm_count{3};                    // consecutive hits to confirm
    int drop_count{3};                       // consecutive misses before drop
    std::size_t max_tracks{10};              // cap on simultaneously held tracks

    bool imm_enabled{true};                  // false = legacy single-CV KF path
    double imm_p_cv_stay{0.95};              // Markov P(CV -> CV), (0, 1) exclusive
    double imm_p_ctrv_stay{0.95};            // Markov P(CTRV -> CTRV), (0, 1) exclusive
    double ctrv_process_noise_accel{1.0};    // sigma_a^2 [m^2/s^4]
    double ctrv_process_noise_yaw_accel{1.0};  // sigma_wd^2 [rad^2/s^4]
    double ctrv_init_omega_variance{1.0};    // omega variance at birth/mixing [rad^2/s^2]
    int prediction_steps{25};                // predicted samples per track (0 = none)
    double prediction_dt{0.1};               // spacing between predicted samples [s]
  };

  explicit Tracker(const Params & params);

  /// Advance every track to `stamp` (dt from the previous call) and fuse the
  /// measurements: matched tracks take a filter update, unmatched measurements
  /// spawn tentative tracks, and unmatched tracks age toward death.
  void update(const std::vector<Cluster> & measurements, double stamp);

  /// All held tracks (tentative and confirmed).
  const std::vector<Track> & tracks() const {return tracks_;}

  /// Predicted centroid positions for one track: prediction_steps samples at
  /// prediction_dt spacing (empty when prediction_steps is 0). IMM path: the
  /// mu-weighted blend of both models' noise-free propagation; legacy path:
  /// the straight constant-velocity ray sampled at the same times.
  std::vector<Eigen::Vector2d> predicted_samples(const Track & track) const;

  /// Drop all tracks and the time origin.
  void reset();

private:
  void predict(double dt);   // legacy single-CV predict over all tracks

  Params params_;
  ImmFilter::Params imm_params_;
  std::vector<Track> tracks_;
  std::uint32_t next_id_{0};
  double last_stamp_{0.0};
  bool has_last_stamp_{false};
};

}  // namespace prox_mpc_obstacle_tracker

#endif  // PROX_MPC_OBSTACLE_TRACKER__TRACKER_HPP_
