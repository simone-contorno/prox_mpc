// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


#include "prox_mpc_obstacle_tracker/tracker.hpp"

#include <algorithm>
#include <cstddef>
#include <vector>

namespace prox_mpc_obstacle_tracker
{

Tracker::Tracker(const Params & params)
: params_(params)
{
  imm_params_.process_noise = params_.process_noise;
  imm_params_.measurement_noise = params_.measurement_noise;
  imm_params_.initial_velocity_variance = params_.initial_velocity_variance;
  imm_params_.p_cv_stay = params_.imm_p_cv_stay;
  imm_params_.p_ctrv_stay = params_.imm_p_ctrv_stay;
  imm_params_.ctrv_process_noise_accel = params_.ctrv_process_noise_accel;
  imm_params_.ctrv_process_noise_yaw_accel = params_.ctrv_process_noise_yaw_accel;
  imm_params_.ctrv_init_omega_variance = params_.ctrv_init_omega_variance;
}

void Tracker::reset()
{
  tracks_.clear();
  next_id_ = 0;
  last_stamp_ = 0.0;
  has_last_stamp_ = false;
}

void Tracker::predict(double dt)
{
  if (dt <= 0.0) {return;}

  Eigen::Matrix4d f = Eigen::Matrix4d::Identity();
  f(0, 2) = dt;
  f(1, 3) = dt;

  // Discrete white-noise acceleration process noise (per axis), spectral
  // density q = process_noise.
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

  for (auto & t : tracks_) {
    t.state = f * t.state;
    t.cov = f * t.cov * f.transpose() + qn;
  }
}

void Tracker::update(const std::vector<Cluster> & measurements, double stamp)
{
  double dt = 0.0;
  if (has_last_stamp_) {dt = stamp - last_stamp_;}
  if (dt < 0.0) {dt = 0.0;}  // non-monotonic stamps: hold position, no prediction
  last_stamp_ = stamp;
  has_last_stamp_ = true;

  // Predict: one IMM mixing+predict cycle per track, refreshing the combined
  // CV-space state association gates on; or the legacy single-CV predict.
  if (params_.imm_enabled) {
    for (auto & t : tracks_) {
      t.filter.predict(dt);
      t.state = t.filter.combined_state();
      t.cov = t.filter.combined_covariance();
    }
  } else {
    predict(dt);
  }

  const std::size_t nt = tracks_.size();
  const std::size_t nm = measurements.size();
  std::vector<int> track_meas(nt, -1);
  std::vector<bool> meas_used(nm, false);

  // Gated greedy nearest-neighbour association: build every track-measurement
  // pair inside the distance gate, then assign closest-first.
  struct Pair
  {
    double d2;
    std::size_t ti;
    std::size_t mi;
  };
  std::vector<Pair> pairs;
  const double gate2 = params_.association_gate * params_.association_gate;
  for (std::size_t ti = 0; ti < nt; ++ti) {
    for (std::size_t mi = 0; mi < nm; ++mi) {
      const double dx = tracks_[ti].state(0) - measurements[mi].x;
      const double dy = tracks_[ti].state(1) - measurements[mi].y;
      const double d2 = dx * dx + dy * dy;
      if (d2 <= gate2) {pairs.push_back({d2, ti, mi});}
    }
  }
  std::sort(
    pairs.begin(), pairs.end(), [](const Pair & a, const Pair & b) {return a.d2 < b.d2;});
  for (const auto & p : pairs) {
    if (track_meas[p.ti] != -1 || meas_used[p.mi]) {continue;}
    track_meas[p.ti] = static_cast<int>(p.mi);
    meas_used[p.mi] = true;
  }

  // Filter update for matched tracks; lifecycle bookkeeping for all. On the
  // legacy path this is an inline linear Kalman update with a position-only
  // measurement model.
  Eigen::Matrix<double, 2, 4> h = Eigen::Matrix<double, 2, 4>::Zero();
  h(0, 0) = 1.0;
  h(1, 1) = 1.0;
  const Eigen::Matrix2d r = params_.measurement_noise * Eigen::Matrix2d::Identity();
  const Eigen::Matrix4d eye = Eigen::Matrix4d::Identity();
  for (std::size_t ti = 0; ti < nt; ++ti) {
    if (track_meas[ti] >= 0) {
      const Cluster & m = measurements[static_cast<std::size_t>(track_meas[ti])];
      if (params_.imm_enabled) {
        tracks_[ti].filter.update(Eigen::Vector2d(m.x, m.y));
        tracks_[ti].state = tracks_[ti].filter.combined_state();
        tracks_[ti].cov = tracks_[ti].filter.combined_covariance();
      } else {
        const Eigen::Vector2d z(m.x, m.y);
        const Eigen::Vector2d y = z - h * tracks_[ti].state;
        const Eigen::Matrix2d s = h * tracks_[ti].cov * h.transpose() + r;
        const Eigen::Matrix<double, 4, 2> k = tracks_[ti].cov * h.transpose() * s.inverse();
        tracks_[ti].state += k * y;
        tracks_[ti].cov = (eye - k * h) * tracks_[ti].cov;
      }
      tracks_[ti].radius = 0.5 * tracks_[ti].radius + 0.5 * m.radius;
      tracks_[ti].hits += 1;
      tracks_[ti].misses = 0;
      if (!tracks_[ti].confirmed && tracks_[ti].hits >= params_.confirm_count) {
        tracks_[ti].confirmed = true;
      }
    } else {
      tracks_[ti].misses += 1;
      tracks_[ti].hits = 0;  // confirmation needs consecutive hits
    }
  }

  // Death: drop tracks that have missed too many consecutive scans.
  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(),
      [&](const Track & t) {return t.misses > params_.drop_count;}),
    tracks_.end());

  // Birth: spawn a tentative track per unmatched measurement, up to the cap.
  for (std::size_t mi = 0; mi < nm; ++mi) {
    if (meas_used[mi]) {continue;}
    if (tracks_.size() >= params_.max_tracks) {break;}
    Track t;
    t.id = next_id_++;
    if (params_.imm_enabled) {
      t.filter = ImmFilter(imm_params_);
      t.filter.reset(measurements[mi].x, measurements[mi].y);
      t.state = t.filter.combined_state();
      t.cov = t.filter.combined_covariance();
    } else {
      t.state << measurements[mi].x, measurements[mi].y, 0.0, 0.0;
      t.cov = Eigen::Matrix4d::Identity();
      t.cov(0, 0) = params_.measurement_noise;
      t.cov(1, 1) = params_.measurement_noise;
      t.cov(2, 2) = params_.initial_velocity_variance;
      t.cov(3, 3) = params_.initial_velocity_variance;
    }
    t.radius = measurements[mi].radius;
    t.hits = 1;
    t.misses = 0;
    // Same confirmation predicate as the update path, applied to the first hit.
    t.confirmed = (t.hits >= params_.confirm_count);
    tracks_.push_back(t);
  }
}

std::vector<Eigen::Vector2d> Tracker::predicted_samples(const Track & track) const
{
  if (params_.prediction_steps <= 0 || !(params_.prediction_dt > 0.0)) {return {};}
  const std::size_t steps = static_cast<std::size_t>(params_.prediction_steps);
  if (params_.imm_enabled) {
    return track.filter.predicted_samples(steps, params_.prediction_dt);
  }
  // Legacy path: the constant-velocity straight ray sampled at the same times.
  std::vector<Eigen::Vector2d> out;
  out.reserve(steps);
  for (std::size_t k = 1; k <= steps; ++k) {
    const double t = static_cast<double>(k) * params_.prediction_dt;
    out.emplace_back(
      track.state(0) + track.state(2) * t, track.state(1) + track.state(3) * t);
  }
  return out;
}

}  // namespace prox_mpc_obstacle_tracker
