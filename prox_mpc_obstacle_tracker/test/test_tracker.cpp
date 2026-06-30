// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Unit tests for the in-house constant-velocity multi-object Tracker: birth and
// death lifecycle, Kalman velocity convergence on a constant-velocity target,
// and identity maintenance / gating across frames. The Tracker is pure (Eigen
// only), so measurements are fed directly as tracking-frame cluster centroids.

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
