// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Unit tests for the scan-to-points conversion and Euclidean-adjacency
// clustering used by the obstacle tracker. Both are pure functions (no ROS), so
// they are driven directly with synthetic scans and point sets.

#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "prox_mpc_obstacle_tracker/clustering.hpp"

namespace
{
using prox_mpc_obstacle_tracker::Cluster;
using prox_mpc_obstacle_tracker::Point2;
using prox_mpc_obstacle_tracker::cluster_points;
using prox_mpc_obstacle_tracker::scan_to_points;
constexpr double kTol = 1e-9;

Point2 pt(double x, double y)
{
  Point2 p;
  p.x = x;
  p.y = y;
  return p;
}

// Append `n` points spaced 0.05 m along +x starting at (x0, y0) (one cluster).
void append_run(std::vector<Point2> & pts, double x0, double y0, int n)
{
  for (int i = 0; i < n; ++i) {
    pts.push_back(pt(x0 + 0.05 * i, y0));
  }
}
}  // namespace

// scan_to_points drops NaN, infinite, below-min, and at/above-max returns, and
// places the survivors at the correct planar bearings.
TEST(Clustering, ScanToPointsRejectsInvalidAndMaxRange)
{
  const std::vector<float> ranges = {
    1.0f,
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::infinity(),
    0.05f,   // below range_min
    5.0f,    // at range_max (rejected: half-open interval)
    2.0f};
  const double inc = M_PI / 2.0;
  const auto pts = scan_to_points(ranges, 0.0, inc, 0.1, 5.0);

  ASSERT_EQ(pts.size(), 2u);            // only the 1.0 and 2.0 returns survive
  EXPECT_NEAR(pts[0].x, 1.0, kTol);     // bearing 0
  EXPECT_NEAR(pts[0].y, 0.0, kTol);
  EXPECT_NEAR(pts[1].x, 0.0, 1e-9);     // bearing 5*pi/2 == pi/2
  EXPECT_NEAR(pts[1].y, 2.0, 1e-9);
}

// Two point groups separated by more than the gap form two clusters, each with
// the mean centroid and the enclosing radius.
TEST(Clustering, SegmentsTwoClustersWithCentroidAndRadius)
{
  std::vector<Point2> pts;
  append_run(pts, 1.0, 0.0, 4);   // group A near y = 0
  append_run(pts, 1.0, 2.0, 4);   // group B near y = 2 (a large jump between them)

  const auto cl = cluster_points(pts, 0.3, 3, 20);
  ASSERT_EQ(cl.size(), 2u);

  EXPECT_NEAR(cl[0].x, 1.075, kTol);       // mean of 1.00, 1.05, 1.10, 1.15
  EXPECT_NEAR(cl[0].y, 0.0, kTol);
  EXPECT_NEAR(cl[0].radius, 0.075, kTol);  // max |x - centroid|
  EXPECT_EQ(cl[0].count, 4u);
  EXPECT_NEAR(cl[1].y, 2.0, kTol);
}

// A segment with fewer than min_points members is dropped; the dense group near
// 3.0 survives.
TEST(Clustering, DropsClustersBelowMinPoints)
{
  std::vector<Point2> pts;
  pts.push_back(pt(0.0, 0.0));    // lone 1-point segment (far from the rest)
  append_run(pts, 3.0, 0.0, 5);   // 5-point group

  const auto cl = cluster_points(pts, 0.3, 3, 20);
  ASSERT_EQ(cl.size(), 1u);       // the lone point is dropped
  EXPECT_EQ(cl[0].count, 5u);
}

// max_clusters keeps the largest clusters by member count.
TEST(Clustering, CapsToLargestClusters)
{
  std::vector<Point2> pts;
  append_run(pts, 0.0, 0.0, 3);    // three well-separated groups of sizes 3, 5, 4
  append_run(pts, 5.0, 0.0, 5);
  append_run(pts, 10.0, 0.0, 4);

  const auto cl = cluster_points(pts, 0.3, 3, 2);
  ASSERT_EQ(cl.size(), 2u);             // capped to 2
  EXPECT_GE(cl[0].count, cl[1].count);  // largest first
  EXPECT_EQ(cl[0].count, 5u);
  EXPECT_EQ(cl[1].count, 4u);
}

// An object straddling the +-pi bearing seam arrives as one segment at each end
// of the sweep; the seam splice must yield a single cluster whose centroid and
// count span both halves, not two half-arc duplicates.
TEST(Clustering, MergesClusterAcrossScanSeam)
{
  // Disc of ~0.2 m extent centred behind the sensor at (-2, 0): returns appear
  // at the start (bearing ~ -pi side) and end (~ +pi side) of the sweep.
  std::vector<Point2> pts;
  pts.push_back(pt(-2.0, -0.10));   // leading half (start of sweep)
  pts.push_back(pt(-2.0, -0.05));
  pts.push_back(pt(-2.0, 0.00));
  append_run(pts, 1.0, 0.0, 4);     // unrelated mid-sweep cluster
  pts.push_back(pt(-2.0, 0.10));    // trailing half (end of sweep)
  pts.push_back(pt(-2.0, 0.05));    // last return is 0.05 m from the first: seam-adjacent

  const auto cl = cluster_points(pts, 0.3, 3, 20);
  ASSERT_EQ(cl.size(), 2u);
  const Cluster & seam = cl[0].count == 5u ? cl[0] : cl[1];
  const Cluster & mid = cl[0].count == 5u ? cl[1] : cl[0];
  EXPECT_EQ(seam.count, 5u);                // both halves merged
  EXPECT_NEAR(seam.x, -2.0, kTol);
  EXPECT_NEAR(seam.y, 0.0, kTol);           // centroid spans the seam
  EXPECT_NEAR(seam.radius, 0.10, kTol);
  EXPECT_EQ(mid.count, 4u);
}

// Distant first/last returns must not trigger the seam splice.
TEST(Clustering, NoSeamMergeWhenEndsApart)
{
  std::vector<Point2> pts;
  append_run(pts, 0.0, 0.0, 4);
  append_run(pts, 5.0, 0.0, 4);
  const auto cl = cluster_points(pts, 0.3, 3, 20);
  ASSERT_EQ(cl.size(), 2u);
  EXPECT_EQ(cl[0].count, 4u);
  EXPECT_EQ(cl[1].count, 4u);
}

// max_radius rejects extended clusters (walls) whose enclosing radius is large,
// while keeping compact obstacles.
TEST(Clustering, RejectsOversizedClusters)
{
  std::vector<Point2> pts;
  for (int i = 0; i < 11; ++i) {            // wide span ~1 m (enclosing radius 0.5)
    pts.push_back(pt(0.1 * i, 0.0));
  }
  append_run(pts, 5.0, 0.0, 4);            // compact group (radius 0.075)

  EXPECT_EQ(cluster_points(pts, 0.3, 3, 20, 0.0).size(), 2u);   // no cap: both survive
  const auto cl = cluster_points(pts, 0.3, 3, 20, 0.2);         // 0.2 m cap drops the wide one
  ASSERT_EQ(cl.size(), 1u);
  EXPECT_NEAR(cl[0].x, 5.075, 1e-9);
}
