// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Unit tests for the in-house constant-velocity multi-object Tracker: birth and
// death lifecycle, Kalman velocity convergence on a constant-velocity target,
// and identity maintenance / gating across frames. The Tracker is pure (Eigen
// only), so measurements are fed directly as tracking-frame cluster centroids.
// The IMM-backed cases add curved predicted-sample emission on an orbit, the
// prediction_steps / prediction_dt sampling contract, and the legacy
// (imm_enabled: false) single-CV path.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "prox_mpc_obstacle_tracker/tracker.hpp"

namespace
{
using prox_mpc_obstacle_tracker::Cluster;
using prox_mpc_obstacle_tracker::Tracker;

Cluster meas(double x, double y, double radius = 0.2)
{
  Cluster c;
  c.x = x;
  c.y = y;
  c.radius = radius;
  c.count = 5;
  return c;
}
}  // namespace

// A track is published (confirmed) only after confirm_count consecutive hits,
// and dropped after more than drop_count consecutive misses.
TEST(Tracker, ConfirmsAndDropsAcrossLifecycle)
{
  Tracker::Params p;  // confirm_count = 3, drop_count = 3 (defaults)
  Tracker t(p);

  t.update({meas(0.0, 0.0)}, 0.0);
  ASSERT_EQ(t.tracks().size(), 1u);
  EXPECT_FALSE(t.tracks()[0].confirmed);   // 1 hit
  t.update({meas(0.0, 0.0)}, 0.1);
  EXPECT_FALSE(t.tracks()[0].confirmed);   // 2 hits
  t.update({meas(0.0, 0.0)}, 0.2);
  ASSERT_EQ(t.tracks().size(), 1u);
  EXPECT_TRUE(t.tracks()[0].confirmed);    // 3 hits -> confirmed

  t.update({}, 0.3);   // miss 1
  t.update({}, 0.4);   // miss 2
  t.update({}, 0.5);   // miss 3 (still held)
  EXPECT_EQ(t.tracks().size(), 1u);
  t.update({}, 0.6);   // miss 4 (> drop_count) -> dropped
  EXPECT_TRUE(t.tracks().empty());
}

// The Kalman filter recovers the velocity of a constant-velocity target.
TEST(Tracker, EstimatesConstantVelocity)
{
  Tracker::Params p;
  p.confirm_count = 3;
  p.measurement_noise = 4e-4;   // ~2 cm position std
  p.process_noise = 0.1;
  Tracker t(p);

  const double vx = 1.0;
  const double vy = 0.5;
  const double dt = 0.1;
  for (int k = 0; k < 100; ++k) {
    const double tk = static_cast<double>(k) * dt;
    t.update({meas(vx * tk, vy * tk)}, tk);
  }

  ASSERT_EQ(t.tracks().size(), 1u);
  EXPECT_TRUE(t.tracks()[0].confirmed);
  EXPECT_NEAR(t.tracks()[0].state(2), vx, 0.05);
  EXPECT_NEAR(t.tracks()[0].state(3), vy, 0.05);
}

// A target that moves within the gate keeps its identity; one that jumps outside
// the gate spawns a second track rather than hijacking the first.
TEST(Tracker, MaintainsIdentityAndGates)
{
  Tracker::Params p;
  p.association_gate = 0.5;
  Tracker t(p);

  t.update({meas(0.0, 0.0)}, 0.0);
  ASSERT_EQ(t.tracks().size(), 1u);
  const std::uint32_t id0 = t.tracks()[0].id;

  t.update({meas(0.1, 0.0)}, 0.1);   // within gate -> same track
  ASSERT_EQ(t.tracks().size(), 1u);
  EXPECT_EQ(t.tracks()[0].id, id0);
  t.update({meas(0.2, 0.0)}, 0.2);
  ASSERT_EQ(t.tracks().size(), 1u);
  EXPECT_EQ(t.tracks()[0].id, id0);

  // A measurement far from any track's prediction spawns a new track.
  t.update({meas(5.0, 5.0)}, 0.3);
  EXPECT_EQ(t.tracks().size(), 2u);
}

// reset() drops all tracks and the time origin.
TEST(Tracker, ResetClearsState)
{
  Tracker::Params p;
  Tracker t(p);
  t.update({meas(0.0, 0.0)}, 0.0);
  ASSERT_EQ(t.tracks().size(), 1u);
  t.reset();
  EXPECT_TRUE(t.tracks().empty());
}

// Measurements on the benchmark dynamic_circle orbit (center (1, 0),
// r = 0.8 m, v = 0.5 m/s, omega = 0.625 rad/s) yield curved predicted samples:
// the 2.0 s-ahead sample lands within 0.2 m of the true arc and clearly off
// the straight constant-velocity ray from the published state.
TEST(Tracker, OrbitMeasurementsYieldCurvedPredictedSamples)
{
  Tracker::Params p;
  p.measurement_noise = 4e-4;
  p.process_noise = 0.1;
  p.imm_p_ctrv_stay = 0.99;
  Tracker t(p);

  const double cx = 1.0;
  const double r = 0.8;
  const double v = 0.5;
  const double omega = v / r;    // 0.625 rad/s
  const double dt = 0.1;
  const int scans = 30;
  for (int k = 0; k <= scans; ++k) {
    const double tk = static_cast<double>(k) * dt;
    t.update({meas(cx + r * std::cos(omega * tk), r * std::sin(omega * tk))}, tk);
  }

  ASSERT_EQ(t.tracks().size(), 1u);
  const auto & track = t.tracks()[0];
  const std::vector<Eigen::Vector2d> samples = t.predicted_samples(track);
  ASSERT_EQ(samples.size(), 25u);   // default prediction_steps

  const double t_now = static_cast<double>(scans) * dt;
  const double horizon = 2.0;       // sample k = 20 at prediction_dt = 0.1
  const Eigen::Vector2d truth(
    cx + r * std::cos(omega * (t_now + horizon)), r * std::sin(omega * (t_now + horizon)));
  const Eigen::Vector2d blended = samples[19];
  const Eigen::Vector2d ray(
    track.state(0) + track.state(2) * horizon, track.state(1) + track.state(3) * horizon);

  EXPECT_LE((blended - truth).norm(), 0.2);   // curved: follows the arc
  EXPECT_GT((blended - ray).norm(), 0.3);     // and is not the straight ray
}

// Sample count and spacing follow prediction_steps / prediction_dt; zero steps
// or a non-positive spacing yield no samples.
TEST(Tracker, SampleCountAndSpacingFollowParams)
{
  Tracker::Params p;
  p.measurement_noise = 4e-4;
  p.process_noise = 0.01;
  p.prediction_steps = 7;
  p.prediction_dt = 0.2;
  Tracker t(p);

  const double vx = 1.0;
  const double vy = 0.5;
  const double dt = 0.1;
  for (int k = 0; k <= 60; ++k) {
    const double tk = static_cast<double>(k) * dt;
    t.update({meas(vx * tk, vy * tk)}, tk);
  }

  ASSERT_EQ(t.tracks().size(), 1u);
  const auto & track = t.tracks()[0];
  const std::vector<Eigen::Vector2d> samples = t.predicted_samples(track);
  ASSERT_EQ(samples.size(), 7u);              // count follows prediction_steps

  // On straight motion the blend is (numerically) the state's own ray, so
  // sample k sits at state + velocity * (k * prediction_dt) and consecutive
  // samples are speed * prediction_dt apart.
  for (std::size_t k = 1; k <= samples.size(); ++k) {
    const double tk = static_cast<double>(k) * p.prediction_dt;
    EXPECT_NEAR(samples[k - 1](0), track.state(0) + track.state(2) * tk, 1e-6);
    EXPECT_NEAR(samples[k - 1](1), track.state(1) + track.state(3) * tk, 1e-6);
  }
  const double speed = std::hypot(track.state(2), track.state(3));
  for (std::size_t k = 1; k < samples.size(); ++k) {
    EXPECT_NEAR((samples[k] - samples[k - 1]).norm(), speed * p.prediction_dt, 1e-6);
  }

  Tracker::Params no_steps = p;
  no_steps.prediction_steps = 0;
  Tracker t_no_steps(no_steps);
  t_no_steps.update({meas(0.0, 0.0)}, 0.0);
  EXPECT_TRUE(t_no_steps.predicted_samples(t_no_steps.tracks()[0]).empty());

  Tracker::Params no_dt = p;
  no_dt.prediction_dt = 0.0;
  Tracker t_no_dt(no_dt);
  t_no_dt.update({meas(0.0, 0.0)}, 0.0);
  EXPECT_TRUE(t_no_dt.predicted_samples(t_no_dt.tracks()[0]).empty());
}

// imm_enabled: false runs the legacy single-CV Kalman path: velocity still
// converges and the predicted samples are exactly the straight
// constant-velocity ray from the published state (bit-for-bit).
TEST(Tracker, LegacyCvPathEmitsStraightRaySamples)
{
  Tracker::Params p;
  p.imm_enabled = false;
  p.measurement_noise = 4e-4;
  p.process_noise = 0.1;
  Tracker t(p);

  const double vx = 1.0;
  const double vy = 0.5;
  const double dt = 0.1;
  for (int k = 0; k < 100; ++k) {
    const double tk = static_cast<double>(k) * dt;
    t.update({meas(vx * tk, vy * tk)}, tk);
  }

  ASSERT_EQ(t.tracks().size(), 1u);
  const auto & track = t.tracks()[0];
  EXPECT_TRUE(track.confirmed);
  EXPECT_NEAR(track.state(2), vx, 0.05);
  EXPECT_NEAR(track.state(3), vy, 0.05);

  const std::vector<Eigen::Vector2d> samples = t.predicted_samples(track);
  ASSERT_EQ(samples.size(), 25u);   // default prediction_steps
  for (std::size_t k = 1; k <= samples.size(); ++k) {
    const double tk = static_cast<double>(k) * p.prediction_dt;
    EXPECT_EQ(samples[k - 1](0), track.state(0) + track.state(2) * tk);
    EXPECT_EQ(samples[k - 1](1), track.state(1) + track.state(3) * tk);
  }
}

// A non-monotonic stamp clamps dt to zero: no prediction happens, the update
// still fuses, and the track neither jumps nor is lost (both filter paths
// share the clamp; the legacy variant covers its own predict guard).
TEST(Tracker, NonMonotonicStampHoldsPrediction)
{
  for (const bool imm_enabled : {true, false}) {
    Tracker::Params p;
    p.imm_enabled = imm_enabled;
    Tracker t(p);
    t.update({meas(0.00, 0.0)}, 1.0);
    t.update({meas(0.05, 0.0)}, 1.1);
    ASSERT_EQ(t.tracks().size(), 1u);
    const std::uint32_t id0 = t.tracks()[0].id;

    t.update({meas(0.05, 0.0)}, 0.9);   // stamp regressed: hold position
    ASSERT_EQ(t.tracks().size(), 1u);
    EXPECT_EQ(t.tracks()[0].id, id0);   // same track, no re-birth
    EXPECT_NEAR(t.tracks()[0].state(0), 0.05, 0.05);
    EXPECT_TRUE(t.tracks()[0].state.allFinite());
  }
}

// With two tracks and two measurements all inside each other's gates, the
// greedy association assigns closest-first, so each track keeps its own
// measurement instead of hijacking the neighbour's.
TEST(Tracker, GreedyAssociationResolvesClosestFirst)
{
  Tracker::Params p;
  p.association_gate = 0.5;
  Tracker t(p);

  t.update({meas(0.0, 0.0), meas(0.4, 0.0)}, 0.0);
  ASSERT_EQ(t.tracks().size(), 2u);
  const std::uint32_t id_left = t.tracks()[0].id;
  const std::uint32_t id_right = t.tracks()[1].id;

  // Both measurements gate to both tracks (all four pair distances < 0.5 m);
  // closest-first must give the left track the left measurement.
  t.update({meas(0.05, 0.0), meas(0.35, 0.0)}, 0.1);
  ASSERT_EQ(t.tracks().size(), 2u);
  for (const auto & track : t.tracks()) {
    if (track.id == id_left) {
      EXPECT_LT(track.state(0), 0.2);
    } else {
      EXPECT_EQ(track.id, id_right);
      EXPECT_GT(track.state(0), 0.2);
    }
  }
}

// Birth respects the max_tracks cap: surplus unmatched measurements do not
// spawn tracks.
TEST(Tracker, BirthRespectsMaxTracksCap)
{
  Tracker::Params p;
  p.max_tracks = 1;
  Tracker t(p);
  t.update({meas(0.0, 0.0), meas(5.0, 5.0)}, 0.0);
  EXPECT_EQ(t.tracks().size(), 1u);
  t.update({meas(0.0, 0.0), meas(5.0, 5.0)}, 0.1);
  EXPECT_EQ(t.tracks().size(), 1u);
}
