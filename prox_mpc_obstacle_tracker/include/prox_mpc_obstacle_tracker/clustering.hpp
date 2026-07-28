// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


#ifndef PROX_MPC_OBSTACLE_TRACKER__CLUSTERING_HPP_
#define PROX_MPC_OBSTACLE_TRACKER__CLUSTERING_HPP_

#include <cstddef>
#include <vector>

namespace prox_mpc_obstacle_tracker
{

/// A planar scan return in the sensor frame [m].
struct Point2
{
  double x;
  double y;
};

/// A cluster of adjacent scan returns: planar centroid and enclosing radius.
struct Cluster
{
  double x{0.0};            // centroid x [m]
  double y{0.0};            // centroid y [m]
  double radius{0.0};       // max member distance from the centroid [m]
  std::size_t count{0};     // number of member returns
};

/// Convert one LaserScan ring to planar points, rejecting non-finite returns and
/// those outside [range_min, range_max) (so maximum-range and invalid beams are
/// dropped). Points are returned in scan (bearing) order, which the segmentation
/// below relies on. range_max must be strictly greater than range_min.
std::vector<Point2> scan_to_points(
  const std::vector<float> & ranges, double angle_min, double angle_increment,
  double range_min, double range_max);

/// Segment scan-ordered points into clusters by Euclidean adjacency: a gap larger
/// than cluster_gap [m] between consecutive points closes the current cluster.
/// Clusters with fewer than min_points members are dropped, and clusters whose
/// enclosing radius exceeds max_radius [m] are dropped too (max_radius <= 0
/// disables this) - this rejects extended structure (walls), whose centroid is
/// not a stable physical point and drifts as the robot moves, which would
/// otherwise be tracked as a phantom moving obstacle. The result is capped to the
/// max_clusters largest clusters (by member count) to bound downstream cost. The
/// scan seam (last-to-first wrap) is merged when the sweep's end points are
/// gap-adjacent: an object straddling +-pi bearing must yield one cluster, not
/// two half-arc duplicates that spawn a second track with a corrupted centroid.
std::vector<Cluster> cluster_points(
  const std::vector<Point2> & points, double cluster_gap, std::size_t min_points,
  std::size_t max_clusters, double max_radius = 0.0);

}  // namespace prox_mpc_obstacle_tracker

#endif  // PROX_MPC_OBSTACLE_TRACKER__CLUSTERING_HPP_
