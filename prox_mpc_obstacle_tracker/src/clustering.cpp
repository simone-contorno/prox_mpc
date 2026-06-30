// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


#include "prox_mpc_obstacle_tracker/clustering.hpp"

#include <algorithm>
#include <cmath>

namespace prox_mpc_obstacle_tracker
{

std::vector<Point2> scan_to_points(
  const std::vector<float> & ranges, double angle_min, double angle_increment,
  double range_min, double range_max)
{
  std::vector<Point2> points;
  points.reserve(ranges.size());
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double r = static_cast<double>(ranges[i]);
    if (!std::isfinite(r) || r < range_min || r >= range_max) {continue;}
    const double a = angle_min + static_cast<double>(i) * angle_increment;
    points.push_back({r * std::cos(a), r * std::sin(a)});
  }
  return points;
}

std::vector<Cluster> cluster_points(
  const std::vector<Point2> & points, double cluster_gap, std::size_t min_points,
  std::size_t max_clusters, double max_radius)
{
  std::vector<Cluster> clusters;
  if (points.empty()) {return clusters;}

  const double gap2 = cluster_gap * cluster_gap;

  // Sequential segmentation over bearing-ordered points; a member-index range
  // [begin, end) per segment lets us recompute centroid and radius after closing.
  auto close_segment = [&](std::size_t begin, std::size_t end) {
      const std::size_t count = end - begin;
      if (count < min_points) {return;}
      double sx = 0.0;
      double sy = 0.0;
      for (std::size_t i = begin; i < end; ++i) {
        sx += points[i].x;
        sy += points[i].y;
      }
      Cluster c;
      c.count = count;
      c.x = sx / static_cast<double>(count);
      c.y = sy / static_cast<double>(count);
      double r2 = 0.0;
      for (std::size_t i = begin; i < end; ++i) {
        const double dx = points[i].x - c.x;
        const double dy = points[i].y - c.y;
        r2 = std::max(r2, dx * dx + dy * dy);
      }
      c.radius = std::sqrt(r2);
      if (max_radius > 0.0 && c.radius > max_radius) {return;}  // reject walls / extended structure
      clusters.push_back(c);
    };

  std::size_t seg_begin = 0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    const double dx = points[i].x - points[i - 1].x;
    const double dy = points[i].y - points[i - 1].y;
    if (dx * dx + dy * dy > gap2) {
      close_segment(seg_begin, i);
      seg_begin = i;
    }
  }
  close_segment(seg_begin, points.size());

  // Cap to the largest clusters so the per-scan cost stays bounded.
  if (clusters.size() > max_clusters) {
    std::partial_sort(
      clusters.begin(), clusters.begin() + static_cast<std::ptrdiff_t>(max_clusters),
      clusters.end(),
      [](const Cluster & a, const Cluster & b) {return a.count > b.count;});
    clusters.resize(max_clusters);
  }
  return clusters;
}

}  // namespace prox_mpc_obstacle_tracker
