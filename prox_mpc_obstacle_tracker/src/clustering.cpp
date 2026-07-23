// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


#include "prox_mpc_obstacle_tracker/clustering.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

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

  // Sequential segmentation over bearing-ordered points: index ranges [begin, end)
  // are collected first so the scan-seam pair can be spliced before closing.
  std::vector<std::pair<std::size_t, std::size_t>> segments;
  std::size_t seg_begin = 0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    const double dx = points[i].x - points[i - 1].x;
    const double dy = points[i].y - points[i - 1].y;
    if (dx * dx + dy * dy > gap2) {
      segments.push_back({seg_begin, i});
      seg_begin = i;
    }
  }
  segments.push_back({seg_begin, points.size()});

  // The scan is angularly cyclic: an object straddling the +-pi bearing seam
  // arrives as one segment at each end of the sweep, and left unmerged it becomes
  // two half-arc clusters, i.e. two duplicate tracks with corrupted centroids.
  // If the sweep's last and first returns are gap-adjacent, splice the trailing
  // segment onto the leading one.
  bool wrap = false;
  if (segments.size() >= 2) {
    const double dx = points.front().x - points.back().x;
    const double dy = points.front().y - points.back().y;
    wrap = dx * dx + dy * dy <= gap2;
  }

  // Close a segment made of up to two index ranges (the second is the leading
  // range of a seam-spliced pair; empty otherwise).
  auto close_segment = [&](std::size_t b1, std::size_t e1, std::size_t b2, std::size_t e2) {
      const std::size_t count = (e1 - b1) + (e2 - b2);
      if (count < min_points) {return;}
      double sx = 0.0;
      double sy = 0.0;
      for (std::size_t i = b1; i < e1; ++i) {
        sx += points[i].x;
        sy += points[i].y;
      }
      for (std::size_t i = b2; i < e2; ++i) {
        sx += points[i].x;
        sy += points[i].y;
      }
      Cluster c;
      c.count = count;
      c.x = sx / static_cast<double>(count);
      c.y = sy / static_cast<double>(count);
      double r2 = 0.0;
      for (std::size_t i = b1; i < e1; ++i) {
        const double dx = points[i].x - c.x;
        const double dy = points[i].y - c.y;
        r2 = std::max(r2, dx * dx + dy * dy);
      }
      for (std::size_t i = b2; i < e2; ++i) {
        const double dx = points[i].x - c.x;
        const double dy = points[i].y - c.y;
        r2 = std::max(r2, dx * dx + dy * dy);
      }
      c.radius = std::sqrt(r2);
      if (max_radius > 0.0 && c.radius > max_radius) {return;}  // reject walls / extended structure
      clusters.push_back(c);
    };

  for (std::size_t s = 0; s < segments.size(); ++s) {
    if (wrap && s == 0) {
      // Seam splice: trailing segment first (bearing order across the wrap).
      close_segment(
        segments.back().first, segments.back().second,
        segments.front().first, segments.front().second);
      continue;
    }
    if (wrap && s + 1 == segments.size()) {continue;}  // consumed by the splice
    close_segment(segments[s].first, segments[s].second, 0, 0);
  }

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
