#!/usr/bin/env python3
# Copyright 2026 Simone Contorno
# SPDX-License-Identifier: Apache-2.0

"""
Controller-independent space-time feasibility check for a b2 scenario.

Answers "does any collision-free, speed-capped trajectory exist from start to
goal against the moving obstacles?" with a holonomic, speed-capped disc robot -
independent of any controller. Reachability is over-approximated (a generous
robot), so an infeasible verdict is trustworthy: if even this robot cannot reach
the goal, no real (non-holonomic, cap-0.5) robot can. It is used to gate the
blind held-out cells: a cell the checker calls infeasible is discarded (it is
impossible for anyone, not a controller failure), so the surviving cells give a
clean read on the controllers.

Usage: space_time_feasible.py <scenario.yaml> [<scenario.yaml> ...]
"""

import math
import sys

import numpy as np
import yaml

ROBOT_R = 0.22
SAFETY = 0.10        # matches run_nav2 body = clearance - (ROBOT_R + SAFETY)
VMAX = 0.5
GOAL_TOL = 0.24
BOUND = 2.95         # navigable interior half-width [m]
RES = 0.08           # grid resolution [m]
DT = 0.5             # time step [s]
TMAX = 60.0


def parse_obstacles(scn):
    obs = []
    for o in scn.get('obstacles', []) or []:
        otype = o.get('type', 'static')
        motion = 'static' if otype == 'static' else o.get('motion', 'static')
        clearance = float(o.get('clearance', 0.7))
        body = max(0.05, clearance - (ROBOT_R + SAFETY))
        if motion == 'circle':
            c = o.get('center', {})
            obs.append({'motion': 'circle', 'cx': float(c.get('x', 0.0)),
                        'cy': float(c.get('y', 0.0)), 'radius': float(o.get('radius', 1.0)),
                        'speed': float(o.get('speed', 0.0)), 'body': body})
        elif motion == 'line':
            a, b = o.get('from', {}), o.get('to', {})
            obs.append({'motion': 'line', 'cx': float(a.get('x', 0.0)),
                        'cy': float(a.get('y', 0.0)), 'ex': float(b.get('x', 0.0)),
                        'ey': float(b.get('y', 0.0)), 'speed': float(o.get('speed', 0.0)),
                        'body': body})
        else:
            p = o.get('pose', {})
            obs.append({'motion': 'static', 'cx': float(p.get('x', 0.0)),
                        'cy': float(p.get('y', 0.0)), 'body': body})
    return obs


def center_at(o, t):
    if o['motion'] == 'circle':
        r = max(o['radius'], 1e-9)
        ang = (o['speed'] / r) * t
        return o['cx'] + r * math.cos(ang), o['cy'] + r * math.sin(ang)
    if o['motion'] == 'line':
        dx, dy = o['ex'] - o['cx'], o['ey'] - o['cy']
        length = math.hypot(dx, dy)
        if length < 1e-9:
            return o['cx'], o['cy']
        sx, sy = (o['cx'], o['cy']) if o['speed'] >= 0.0 else (o['ex'], o['ey'])
        tx, ty = (o['ex'], o['ey']) if o['speed'] >= 0.0 else (o['cx'], o['cy'])
        s = math.fmod(abs(o['speed']) * t, 2.0 * length)
        frac = s / length if s <= length else (2.0 * length - s) / length
        return sx + frac * (tx - sx), sy + frac * (ty - sy)
    return o['cx'], o['cy']


def feasible(scn):
    obs = parse_obstacles(scn)
    axis = np.arange(-BOUND, BOUND + RES, RES)
    gx, gy = np.meshgrid(axis, axis, indexing='xy')

    def free_at(t):
        occ = np.zeros_like(gx, dtype=bool)
        for o in obs:
            ox, oy = center_at(o, t)
            occ |= (gx - ox) ** 2 + (gy - oy) ** 2 < (ROBOT_R + o['body']) ** 2
        return ~occ

    # One-step reachability disc at the 0.5 m/s cap, slightly generous (+0.5 cell)
    # so the model is a mild over-approximation and an infeasible verdict is safe.
    step = VMAX * DT / RES + 0.5
    offs = [(dx, dy) for dx in range(-int(step) - 1, int(step) + 2)
            for dy in range(-int(step) - 1, int(step) + 2)
            if dx * dx + dy * dy <= step * step]

    h, w = gx.shape

    def dilate(src):
        out = np.zeros_like(src)
        for dx, dy in offs:
            # Non-wrapping shift: out[y+dy, x+dx] |= src[y, x], clipped at borders.
            out[max(0, dy):h + min(0, dy), max(0, dx):w + min(0, dx)] |= \
                src[max(0, -dy):h + min(0, -dy), max(0, -dx):w + min(0, -dx)]
        return out

    start = scn['start']
    goal = scn['goals'][-1]

    def cell(x, y):
        return (int(round((y + BOUND) / RES)), int(round((x + BOUND) / RES)))

    reach = free_at(0.0).copy()
    seed = np.zeros_like(reach)
    r0, c0 = cell(float(start['x']), float(start['y']))
    seed[r0, c0] = True
    reach &= seed
    if not reach.any():
        return False, 0.0

    goal_mask = (gx - float(goal['x'])) ** 2 + (gy - float(goal['y'])) ** 2 <= GOAL_TOL ** 2
    t = 0.0
    while t < TMAX:
        t += DT
        reach = dilate(reach) & free_at(t)
        if (reach & goal_mask).any():
            return True, t
    return False, TMAX


def main():
    for path in sys.argv[1:]:
        scn = yaml.safe_load(open(path))['scenario']
        ok, t = feasible(scn)
        verdict = 'FEASIBLE' if ok else 'INFEASIBLE'
        print(f"{scn['name']:22} {verdict:11} (goal-reach lower bound {t:.1f}s)")


if __name__ == '__main__':
    main()
