#!/usr/bin/env python3
# Copyright 2026 Simone Contorno
# SPDX-License-Identifier: Apache-2.0

"""
Reduce a recorded rosbag2 to one per-run metrics JSON.

The bag-based path for modes (a) Gazebo+Nav2 and (b2) Nav2-without-Gazebo, where a
run is recorded rather than measured live. It reads the SolverDiagnostics stream
(real-time / feasibility) and the metrics node's live cross_track_error /
goal_distance topics (accuracy), and emits the same JSON schema run_matrix.py
collects from the live metrics node, so aggregate.py treats both paths uniformly.
"""

import argparse
import json
import math
from pathlib import Path
import statistics as st

from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=''),
        rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic in types:
            msg = deserialize_message(data, get_message(types[topic]))
            yield topic, msg, stamp


def percentile(vals, q):
    if not vals:
        return None
    s = sorted(vals)
    idx = q * (len(s) - 1)
    lo, hi = int(idx), min(int(idx) + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (idx - lo)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--scenario', default='')
    ap.add_argument('--model', default='')
    ap.add_argument('--mode', default='a')
    ap.add_argument('--controller', default='proxmpc')
    ap.add_argument('--repeat', type=int, default=0)
    ap.add_argument('--goal-tol', type=float, default=0.25)
    ap.add_argument('--diag-topic', default='')
    # Required, unlike --diag-topic: both carry Float64, so an unset topic cannot
    # be matched by message type the way SolverDiagnostics can. Defaulting to ''
    # matched nothing and reported success: False for a run that had reached the
    # goal, which is worse than refusing to run.
    ap.add_argument('--ct-topic', required=True,
                    help='cross-track error topic (std_msgs/Float64)')
    ap.add_argument('--gd-topic', required=True,
                    help='goal distance topic (std_msgs/Float64); drives success')
    args = ap.parse_args()

    solve, sqp, qp = [], [], []
    miss = infeas = diag_n = solve_dropped = 0
    slack_max = 0.0
    ct, gd = [], []
    SOLVED = 0
    for topic, msg, _stamp in read_bag(Path(args.bag)):
        tn = type(msg).__name__
        if tn == 'SolverDiagnostics' and (not args.diag_topic or topic == args.diag_topic):
            diag_n += 1
            # A non-finite solve time sorts into an arbitrary position and would
            # corrupt the percentiles, so it is rejected at ingestion (matching
            # metrics_node) rather than inside percentile().
            if math.isfinite(msg.solve_time_ms):
                solve.append(msg.solve_time_ms)
            else:
                solve_dropped += 1
            sqp.append(msg.sqp_iters)
            qp.append(msg.qp_iters_ext)
            miss += int(msg.deadline_missed)
            infeas += int(msg.status != SOLVED)
            slack_max = max(slack_max, msg.max_obstacle_slack)
        elif tn == 'Float64' and topic == args.ct_topic:
            ct.append(msg.data)
        elif tn == 'Float64' and topic == args.gd_topic:
            gd.append(msg.data)

    reached = any(v <= args.goal_tol for v in gd) if gd else False
    ct_rms = (sum(v * v for v in ct) / len(ct)) ** 0.5 if ct else 0.0
    rec = {
        'scenario': args.scenario, 'model': args.model, 'mode': args.mode,
        'controller': args.controller, 'repeat': args.repeat,
        'success': bool(reached),
        'goal_error_m': gd[-1] if gd else None,
        'min_goal_distance_m': min(gd) if gd else None,
        'cross_track_rms_m': ct_rms,
        'cross_track_max_m': max(ct) if ct else 0.0,
        'solve_ms_p50': percentile(solve, 0.5),
        'solve_ms_p95': percentile(solve, 0.95),
        'solve_ms_max': max(solve) if solve else None,
        'solve_ms_mean': (sum(solve) / len(solve)) if solve else None,
        'deadline_miss_rate': (miss / diag_n) if diag_n else 0.0,
        'mean_sqp_iters': st.mean(sqp) if sqp else 0.0,
        'mean_qp_iters_ext': st.mean(qp) if qp else 0.0,
        'infeasible_rate': (infeas / diag_n) if diag_n else 0.0,
        'max_obstacle_slack_m': slack_max,
        'num_diag_samples': diag_n,
        'status': 'ok',
    }
    Path(args.out).write_text(json.dumps(rec, indent=2))
    dropped = f', non-finite solve_time_ms dropped={solve_dropped}' if solve_dropped else ''
    print(f'[compute_metrics] {args.bag} -> {args.out} '
          f'(diag={diag_n}, success={reached}{dropped})')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
