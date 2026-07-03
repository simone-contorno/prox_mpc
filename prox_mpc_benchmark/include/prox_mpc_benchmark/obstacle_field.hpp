// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Pure obstacle-field helpers shared by the mode (b2) scan simulator (which turns
// the scenario obstacles into a LaserScan so every controller perceives them
// through the same Nav2 costmap) and the metrics node (which measures the
// controller-agnostic robot/obstacle clearance). Factored out so the geometry can
// be unit-tested without spinning a node.
//
// obstacleCenterAt mirrors prox_mpc_demo SimulationNode::positionAt so the mode
// (b1) engine plant and the mode (b2) Nav2 stack drive the identical obstacle
// geometry; keep the two in sync.

#ifndef PROX_MPC_BENCHMARK__OBSTACLE_FIELD_HPP_
#define PROX_MPC_BENCHMARK__OBSTACLE_FIELD_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace prox_mpc_benchmark
{

/// One time-varying obstacle. motion selects how obstacleCenterAt integrates the
/// centre over time; static obstacles ignore the motion fields.
struct ObstacleSpec
{
  std::string motion = "static";  // static | circle | line
  double cx = 0.0, cy = 0.0;      // static pos / circle centre / line "from"
  double ex = 0.0, ey = 0.0;      // line "to"
  double radius = 1.0;            // circle orbit radius [m]
  double speed = 0.0;             // signed speed [m/s] (direction by sign)
  double body = 0.25;             // physical obstacle radius [m] (sensing + clearance)
};

/// Obstacle centre at absolute time `t` [s]. Mirrors the mode (b1) motion law.
inline std::pair<double, double> obstacleCenterAt(const ObstacleSpec & o, double t)
{
  if (o.motion == "circle") {
    const double r = std::max(o.radius, 1e-9);
    const double ang = (o.speed / r) * t;  // sign of speed = ccw / cw
    return {o.cx + r * std::cos(ang), o.cy + r * std::sin(ang)};
  }
  if (o.motion == "line") {
    const double dx = o.ex - o.cx;
    const double dy = o.ey - o.cy;
    const double len = std::hypot(dx, dy);
    if (len < 1e-9) {return {o.cx, o.cy};}
    // Patrol from start -> end -> start; speed sign selects the first leg.
    const double sx = o.speed >= 0.0 ? o.cx : o.ex;
    const double sy = o.speed >= 0.0 ? o.cy : o.ey;
    const double tx = o.speed >= 0.0 ? o.ex : o.cx;
    const double ty = o.speed >= 0.0 ? o.ey : o.cy;
    const double s = std::fmod(std::abs(o.speed) * t, 2.0 * len);
    const double frac = s <= len ? s / len : (2.0 * len - s) / len;
    return {sx + frac * (tx - sx), sy + frac * (ty - sy)};
  }
  return {o.cx, o.cy};  // static
}

/// Range from ray origin (ox, oy) along world heading `ang` to the near surface of
/// a disc centred (cx, cy) with radius r. Returns the smallest hit distance within
/// [range_min, range_max], or NaN when the ray misses (a max-range clearing beam).
inline double rayDiscRange(
  double ox, double oy, double ang, double cx, double cy, double r,
  double range_min, double range_max)
{
  const double dx = std::cos(ang);
  const double dy = std::sin(ang);
  const double fx = ox - cx;
  const double fy = oy - cy;
  const double h = fx * dx + fy * dy;          // projection of centre-offset on ray
  const double c = fx * fx + fy * fy - r * r;  // <0 => origin inside the disc
  const double disc = h * h - c;
  if (disc < 0.0) {return std::nan("");}
  const double sq = std::sqrt(disc);
  const double t_near = -h - sq;
  const double t_far = -h + sq;
  const double t = t_near >= range_min ? t_near : t_far;
  if (t >= range_min && t <= range_max) {return t;}
  return std::nan("");
}

/// Signed gap [m] between a robot disc (centre (px, py), radius robot_radius) and
/// the obstacle body at time `t`: negative means the two discs overlap (a
/// collision on the collision-free kinematic plant).
inline double robotObstacleGap(
  const ObstacleSpec & o, double t, double px, double py, double robot_radius)
{
  const auto [ox, oy] = obstacleCenterAt(o, t);
  return std::hypot(px - ox, py - oy) - o.body - robot_radius;
}

/// Build obstacle specs from the parallel parameter arrays the runner writes.
/// Shorter arrays are padded with the per-field default; an empty motion list
/// yields no obstacles.
inline std::vector<ObstacleSpec> parseObstacleSpecs(
  const std::vector<std::string> & motion,
  const std::vector<double> & cx, const std::vector<double> & cy,
  const std::vector<double> & ex, const std::vector<double> & ey,
  const std::vector<double> & radius, const std::vector<double> & speed,
  const std::vector<double> & body)
{
  auto at = [](const std::vector<double> & v, std::size_t i, double dflt) {
      return i < v.size() ? v[i] : dflt;
    };
  std::vector<ObstacleSpec> out;
  out.reserve(motion.size());
  for (std::size_t i = 0; i < motion.size(); ++i) {
    ObstacleSpec o;
    o.motion = motion[i];
    o.cx = at(cx, i, 0.0);
    o.cy = at(cy, i, 0.0);
    o.ex = at(ex, i, 0.0);
    o.ey = at(ey, i, 0.0);
    o.radius = at(radius, i, 1.0);
    o.speed = at(speed, i, 0.0);
    o.body = at(body, i, 0.25);
    out.push_back(o);
  }
  return out;
}

}  // namespace prox_mpc_benchmark

#endif  // PROX_MPC_BENCHMARK__OBSTACLE_FIELD_HPP_
