#!/usr/bin/env python3

# Copyright 2026 Simone Contorno
# SPDX-License-Identifier: Apache-2.0

"""
Render a trajectory plot from a ProxMPC simulation CSV.

Usage:
    plot_results.py <csv_path> <png_path>

The CSV is written by the simulation node on a clean shutdown. Its first line is a
``#`` metadata comment (``key=value`` pairs: model, goal_x, goal_y, obstacle_enable,
obs_x, obs_y), followed by a header row and one row per control step with columns:
``step,t,x,y,theta,delta,v,u2,solve_ms``.

If the CSV is missing or has no data rows (for example after a SIGKILL), this
script logs a warning and exits 0 so it never fails a launch teardown.
"""

import csv
import os
import sys

import matplotlib

matplotlib.use('Agg')  # headless: render to file, never open a window.
import matplotlib.pyplot as plt  # noqa: E402


def _parse_metadata(line):
    """Parse a leading ``# key=value key=value`` comment into a dict."""
    meta = {}
    for token in line.lstrip('#').split():
        if '=' in token:
            key, value = token.split('=', 1)
            meta[key] = value
    return meta


def _read_csv(csv_path):
    """Return (metadata dict, list of row dicts). Rows may be empty."""
    meta = {}
    rows = []
    with open(csv_path, 'r', newline='') as handle:
        first = handle.readline()
        if first.startswith('#'):
            meta = _parse_metadata(first)
        else:
            handle.seek(0)
        reader = csv.DictReader(handle)
        for row in reader:
            rows.append(row)
    return meta, rows


def _to_float(value):
    """Convert a CSV field to float, or None when it is missing/empty."""
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def main(argv):
    """Render the trajectory/speed/solve-time plot; return a process exit code."""
    if len(argv) != 3:
        print('usage: plot_results.py <csv_path> <png_path>', file=sys.stderr)
        return 2

    csv_path, png_path = argv[1], argv[2]

    if not os.path.isfile(csv_path):
        print(f'[plot_results] CSV not found, nothing to plot: {csv_path}',
              file=sys.stderr)
        return 0

    meta, rows = _read_csv(csv_path)
    if not rows:
        print(f'[plot_results] CSV has no data rows, nothing to plot: {csv_path}',
              file=sys.stderr)
        return 0

    t = [_to_float(r.get('t')) for r in rows]
    x = [_to_float(r.get('x')) for r in rows]
    y = [_to_float(r.get('y')) for r in rows]
    v = [_to_float(r.get('v')) for r in rows]
    solve_ms = [_to_float(r.get('solve_ms')) for r in rows]

    goal_x = _to_float(meta.get('goal_x'))
    goal_y = _to_float(meta.get('goal_y'))
    obs_on = meta.get('obstacle_enable') in ('1', 'true', 'True')
    obs_x = _to_float(meta.get('obs_x'))
    obs_y = _to_float(meta.get('obs_y'))
    model = meta.get('model', 'unknown')

    fig, (ax_xy, ax_v, ax_solve) = plt.subplots(3, 1, figsize=(8, 11))
    fig.suptitle(f'prox_mpc simulation ({model}) - {len(rows)} steps')

    # Panel 1: closed-loop X-Y trajectory.
    ax_xy.plot(x, y, '-', color='#1f77b4', label='trajectory')
    ax_xy.plot(x[0], y[0], 'o', color='green', label='start')
    if goal_x is not None and goal_y is not None:
        ax_xy.plot(goal_x, goal_y, '*', color='red', markersize=14, label='goal')
    if obs_on and obs_x is not None and obs_y is not None:
        ax_xy.add_patch(plt.Circle((obs_x, obs_y), 0.5, color='orange',
                                   alpha=0.4, label='obstacle'))
    ax_xy.set_xlabel('x [m]')
    ax_xy.set_ylabel('y [m]')
    ax_xy.set_title('Closed-loop trajectory')
    ax_xy.set_aspect('equal', adjustable='datalim')
    ax_xy.grid(True)
    ax_xy.legend(loc='best')

    # Panel 2: commanded forward speed.
    ax_v.plot(t, v, '-', color='#2ca02c')
    ax_v.set_xlabel('t [s]')
    ax_v.set_ylabel('v [m/s]')
    ax_v.set_title('Commanded forward speed')
    ax_v.grid(True)

    # Panel 3: solver compute time.
    ax_solve.plot(t, solve_ms, '-', color='#d62728')
    ax_solve.set_xlabel('t [s]')
    ax_solve.set_ylabel('solve [ms]')
    ax_solve.set_title('MPC solve time')
    ax_solve.grid(True)

    fig.tight_layout(rect=(0, 0, 1, 0.98))

    out_dir = os.path.dirname(png_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    fig.savefig(png_path, dpi=120)
    plt.close(fig)
    print(f'[plot_results] wrote {png_path}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
