// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


#ifndef PROX_MPC_OBSTACLE_TRACKER__TRACKER_HPP_
#define PROX_MPC_OBSTACLE_TRACKER__TRACKER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Dense>

#include "prox_mpc_obstacle_tracker/clustering.hpp"

namespace prox_mpc_obstacle_tracker
{

/// One tracked obstacle with a constant-velocity Kalman state [x, y, vx, vy] in
/// the tracking frame and its 4x4 covariance.
struct Track
{
  std::uint32_t id{0};
  Eigen::Vector4d state{Eigen::Vector4d::Zero()};
  Eigen::Matrix4d cov{Eigen::Matrix4d::Identity()};
  double radius{0.0};       // smoothed enclosing radius [m]
  int hits{0};              // consecutive associated scans
  int misses{0};            // consecutive missed scans
  bool confirmed{false};    // promoted after confirm_count consecutive hits
};

/// In-house constant-velocity multi-object tracker: gated greedy
/// nearest-neighbour association, one Kalman filter per track, and a
/// birth/death lifecycle. Pure (Eigen only, no ROS) so the pipeline is
/// unit-testable. Measurements are cluster centroids already transformed into a
/// fixed, non-rotating tracking frame; stamps are absolute seconds.
class Tracker
{
public:
  struct Params
  {
    double process_noise{1.0};               // accel spectral density q [m^2/s^4]
    double measurement_noise{0.01};          // position measurement variance [m^2]
    double association_gate{0.5};            // max association distance [m]
    double initial_velocity_variance{1.0};   // initial vx/vy variance [m^2/s^2]
    int confirm_count{3};                    // consecutive hits to confirm
    int drop_count{3};                       // consecutive misses before drop
    std::size_t max_tracks{10};              // cap on simultaneously held tracks
  };

  explicit Tracker(const Params & params)
  : params_(params) {}

  /// Advance every track to `stamp` (dt from the previous call) and fuse the
  /// measurements: matched tracks take a Kalman update, unmatched measurements
  /// spawn tentative tracks, and unmatched tracks age toward death.
  void update(const std::vector<Cluster> & measurements, double stamp);

  /// All held tracks (tentative and confirmed).
  const std::vector<Track> & tracks() const {return tracks_;}

  /// Drop all tracks and the time origin.
  void reset();

private:
  void predict(double dt);

  Params params_;
  std::vector<Track> tracks_;
  std::uint32_t next_id_{0};
  double last_stamp_{0.0};
  bool has_last_stamp_{false};
};

}  // namespace prox_mpc_obstacle_tracker

#endif  // PROX_MPC_OBSTACLE_TRACKER__TRACKER_HPP_
