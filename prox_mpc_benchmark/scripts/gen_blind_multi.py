#!/usr/bin/env python3
# Copyright 2026 Simone Contorno
# SPDX-License-Identifier: Apache-2.0

"""
Generate N *blind* two-mover held-out scenarios for the b2 cross-controller run.

Blind = the obstacle geometry is drawn from a seeded RNG within fixed feasible
bounds and written out without ever running a controller to screen it. This
removes the author-selection bias that a hand-built held-out cell carries (the
`dynamic_multi` cell was picked after rejecting geometries where pred failed).

The only geometric constraint is a solvability precondition applied blind to all
controllers alike: the orbiter's left edge must clear the line patrol by a
margin, so a feasible lane always exists and the cell is not an unavoidable pinch
that fails every controller (uninformative). It does not look at performance.

Every generated scenario must be run and reported; dropping the ones where pred
does badly would re-introduce the exact bias this tool removes.

Usage:
    python3 gen_blind_multi.py --n 4 --seed 20260707
    # then run each printed scenario name through run_nav2.py (all seven controllers)
"""

# The scenario YAML this script emits has structurally long lines (obstacle mappings
# and the description); those source lines exceed the 99-char limit by construction.
# flake8: noqa: E501

import argparse
from pathlib import Path
import random

CLEARANCE = 0.45  # same physical obstacle size as every other b2 cell (fair)


def sample_scenario(rng, name):
    """Draw one feasible two-mover geometry; resample until the lane precondition holds."""
    for _ in range(1000):
        line_x = rng.uniform(-1.3, -0.3)
        line_speed = rng.choice([-1.0, 1.0]) * rng.uniform(0.4, 0.8)
        cx = rng.uniform(0.6, 1.6)
        cy = rng.uniform(-0.3, 0.3)
        radius = rng.uniform(0.6, 0.9)
        cspeed = rng.choice([-1.0, 1.0]) * rng.uniform(0.4, 0.6)
        # Solvability precondition (blind to controller performance): the orbiter's
        # leftmost reach clears the line patrol so a y~0 lane is not pinched shut.
        if (cx - radius) - line_x >= 0.7:
            break
    else:
        raise RuntimeError('could not sample a feasible geometry')
    return {
        'name': name,
        'line_x': round(line_x, 3),
        'line_speed': round(line_speed, 3),
        'cx': round(cx, 3),
        'cy': round(cy, 3),
        'radius': round(radius, 3),
        'cspeed': round(cspeed, 3),
    }


def render(s):
    return f"""scenario:
  name: {s['name']}
  description: "BLIND held-out two-mover cell (seeded generator, not author-screened). Line patrol at x={s['line_x']} and an orbiter at ({s['cx']}, {s['cy']})."
  map:
    base: prox_mpc_open
    yaml: prox_mpc_open.yaml
    scale: 1.0
  start: {{x: -2.5, y: 0.0, yaw: 0.0}}
  goals:
    - {{x: 2.5, y: 0.0, yaw: 0.0}}
  obstacles:
    - {{type: dynamic, motion: line, from: {{x: {s['line_x']}, y: -1.8}}, to: {{x: {s['line_x']}, y: 1.8}}, speed: {s['line_speed']}, clearance: {CLEARANCE}}}
    - {{type: dynamic, motion: circle, center: {{x: {s['cx']}, y: {s['cy']}}}, radius: {s['radius']}, speed: {s['cspeed']}, clearance: {CLEARANCE}}}
  models: [unicycle]
  controllers: [proxmpc_pred, proxmpc, dwb, mppi, regulated_pure_pursuit, graceful, vector_pursuit]
  run: {{repeats: 5, timeout_s: 60, seed: 0}}
"""


def main():
    ap = argparse.ArgumentParser(description='Generate blind two-mover held-out scenarios')
    ap.add_argument('--n', type=int, default=4)
    ap.add_argument('--seed', type=int, default=20260707)
    ap.add_argument('--out', default='',
                    help='scenarios dir (default: this package\'s config/scenarios)')
    args = ap.parse_args()

    out = Path(args.out) if args.out else \
        Path(__file__).resolve().parents[1] / 'config' / 'scenarios'
    out.mkdir(parents=True, exist_ok=True)
    rng = random.Random(args.seed)

    names = []
    for i in range(args.n):
        name = f'blind_multi_{i}'
        s = sample_scenario(rng, name)
        (out / f'{name}.yaml').write_text(render(s))
        names.append(name)
        print(f'[gen] {name}: line_x={s["line_x"]} line_v={s["line_speed"]} '
              f'circle=({s["cx"]},{s["cy"]}) r={s["radius"]} v={s["cspeed"]}')
    print('\nRun ALL of these (do not drop any):')
    print('  ' + ','.join(names))


if __name__ == '__main__':
    raise SystemExit(main())
