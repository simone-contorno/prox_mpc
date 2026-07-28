// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Pure geometric / statistical helpers for the benchmark metrics node, factored
// out so they can be unit-tested without spinning a node.

#ifndef PROX_MPC_BENCHMARK__METRICS_MATH_HPP_
#define PROX_MPC_BENCHMARK__METRICS_MATH_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace prox_mpc_benchmark
{

/// Shortest distance from point (px, py) to the polyline through (xs, ys).
inline double crossTrack(
  const std::vector<double> & xs, const std::vector<double> & ys, double px, double py)
{
  // xs and ys come from independent ref_x / ref_y parameter arrays, so a
  // malformed pair of unequal length is truncated to the common prefix instead
  // of indexing the shorter array out of bounds.
  const std::size_t n = std::min(xs.size(), ys.size());
  if (n == 0) {return 0.0;}
  if (n == 1) {return std::hypot(px - xs[0], py - ys[0]);}
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < n; ++i) {
    const double ax = xs[i], ay = ys[i], bx = xs[i + 1], by = ys[i + 1];
    const double dx = bx - ax, dy = by - ay;
    const double len2 = dx * dx + dy * dy;
    double t = len2 > 0.0 ? ((px - ax) * dx + (py - ay) * dy) / len2 : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    const double cx = ax + t * dx, cy = ay + t * dy;
    best = std::min(best, std::hypot(px - cx, py - cy));
  }
  return best;
}

/// Linear-interpolated quantile q in [0, 1] of v (v is copied and sorted).
inline double percentile(std::vector<double> v, double q)
{
  if (v.empty()) {return std::nan("");}
  std::sort(v.begin(), v.end());
  const double idx = q * (static_cast<double>(v.size()) - 1.0);
  const std::size_t lo = static_cast<std::size_t>(std::floor(idx));
  const std::size_t hi = static_cast<std::size_t>(std::ceil(idx));
  if (lo == hi) {return v[lo];}
  const double frac = idx - static_cast<double>(lo);
  return v[lo] * (1.0 - frac) + v[hi] * frac;
}

}  // namespace prox_mpc_benchmark

#endif  // PROX_MPC_BENCHMARK__METRICS_MATH_HPP_
