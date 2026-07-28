#!/usr/bin/env python3
# Copyright 2026 Simone Contorno
# SPDX-License-Identifier: Apache-2.0

"""
Orchestrate the benchmark matrix (scenario x model x mode x repeats).

For mode (b1) - the deterministic standalone core sim - this launches the
prox_mpc_demo simulation node and the prox_mpc_benchmark metrics node against the
same scenario geometry, waits for the goal (the metrics node self-terminates a
short settle after reaching it) or the scenario timeout, and collects the per-run
metrics JSON the metrics node writes. Per-run JSONs land in <results>/runs/ and a
structured index is written to <results>/scenarios.json.

Modes (a) Gazebo+Nav2 and (b2) Nav2-without-Gazebo are delegated to
benchmark.launch.py (heavier; see the package README for status).

The environment must already be sourced (see init.sh). No git operations.
"""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from prox_mpc_benchmark.scenario_obstacles import obstacle_arrays
import yaml

PKG = 'prox_mpc_benchmark'
SCENARIO_NAMES = [
    'static_box',
    'dynamic_circle',
    'dynamic_line_forward',
    'dynamic_line_backward',
]
MODELS = ['unicycle', 'bicycle']


def share_dir() -> Path:
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(PKG))


def load_yaml(path: Path) -> dict:
    with open(path) as fh:
        return yaml.safe_load(fh)


def scenario_reference(scn: dict):
    """Build the reference polyline: start followed by every goal (cross-track)."""
    s = scn['start']
    xs = [float(s['x'])]
    ys = [float(s['y'])]
    for g in scn['goals']:
        xs.append(float(g['x']))
        ys.append(float(g['y']))
    return xs, ys


def write_params(path: Path, scn: dict, model: str, control: dict,
                 summary_json: Path, scenario_name: str, controller: str,
                 repeat: int):
    s = scn['start']
    goal = scn['goals'][-1]
    goals_x = [float(g['x']) for g in scn['goals']]
    goals_y = [float(g['y']) for g in scn['goals']]
    goals_t = [float(g.get('yaw', 0.0)) for g in scn['goals']]
    ref_x, ref_y = scenario_reference(scn)
    motion, cx, cy, ex, ey, radius, speed, clearance = obstacle_arrays(scn)
    n_obs = max(1, len(motion))

    sim = {
        'model': model,
        'np': int(control['np']),
        'nc': int(control['nc']),
        'dt': float(control['dt']),
        'v_ref': float(control['v_ref']),
        'goal_tol': float(control['goal_tol']),
        'report_period': 0,
        'publish_diagnostics': True,
        'start_x': float(s['x']),
        'start_y': float(s['y']),
        'start_theta': float(s.get('yaw', 0.0)),
        'goals_x': goals_x,
        'goals_y': goals_y,
        'goals_theta': goals_t,
        'max_obstacles': n_obs,
    }
    if motion:
        sim.update({
            'obs_motion': motion,
            'obs_cx': cx, 'obs_cy': cy, 'obs_ex': ex, 'obs_ey': ey,
            'obs_radius': radius, 'obs_speed': speed, 'obs_clearance': clearance,
        })

    metrics = {
        'map_frame': 'map',
        'base_frame': 'base_link',
        'ref_x': ref_x,
        'ref_y': ref_y,
        'goal_x': float(goal['x']),
        'goal_y': float(goal['y']),
        'goal_theta': float(goal.get('yaw', 0.0)),
        'goal_tol': float(control['goal_tol']),
        'diagnostics_topic': '/prox_mpc/diagnostics',
        'summary_json': str(summary_json),
        'scenario': scenario_name,
        'model': model,
        'mode': 'b1',
        'controller': controller,
        'repeat': int(repeat),
        'auto_exit': True,
        'settle_s': 1.0,
    }
    doc = {
        'prox_mpc_simulation': {'ros__parameters': sim},
        'prox_mpc_metrics': {'ros__parameters': metrics},
    }
    with open(path, 'w') as fh:
        yaml.safe_dump(doc, fh, default_flow_style=None, sort_keys=False)


def run_b1(scn: dict, scenario_name: str, model: str, repeat: int, control: dict,
           timeout_s: float, results_dir: Path, controller: str) -> dict:
    runs_dir = results_dir / 'runs'
    runs_dir.mkdir(parents=True, exist_ok=True)
    tag = f'{scenario_name}__{model}__b1__{controller}__r{repeat}'
    summary_json = runs_dir / f'{tag}.json'
    params_file = runs_dir / f'{tag}.params.yaml'
    if summary_json.exists():
        summary_json.unlink()
    write_params(params_file, scn, model, control, summary_json,
                 scenario_name, controller, repeat)

    env = dict(os.environ)
    sim_log = open(runs_dir / f'{tag}.sim.log', 'w')
    met_log = open(runs_dir / f'{tag}.metrics.log', 'w')
    sim = subprocess.Popen(
        ['ros2', 'run', 'prox_mpc_demo', 'prox_mpc_simulation',
         '--ros-args', '--params-file', str(params_file)],
        stdout=sim_log, stderr=subprocess.STDOUT, env=env, preexec_fn=os.setsid)
    metrics = subprocess.Popen(
        ['ros2', 'run', PKG, 'metrics_node',
         '--ros-args', '--params-file', str(params_file)],
        stdout=met_log, stderr=subprocess.STDOUT, env=env, preexec_fn=os.setsid)

    deadline = time.time() + timeout_s + 8.0
    try:
        while time.time() < deadline:
            if metrics.poll() is not None:
                break  # metrics node self-exited (goal reached) or died
            time.sleep(0.3)
        # Timed out without the goal: ask the metrics node to write success=false.
        if metrics.poll() is None:
            os.killpg(os.getpgid(metrics.pid), signal.SIGINT)
            try:
                metrics.wait(timeout=6.0)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(metrics.pid), signal.SIGKILL)
    finally:
        for p in (sim, metrics):
            if p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGINT)
                except ProcessLookupError:
                    pass
        for p in (sim, metrics):
            try:
                p.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
        sim_log.close()
        met_log.close()

    if summary_json.exists():
        with open(summary_json) as fh:
            rec = json.load(fh)
        rec['status'] = 'ok'
        return rec
    return {
        'scenario': scenario_name, 'model': model, 'mode': 'b1',
        'controller': controller, 'repeat': repeat,
        'status': 'no_summary', 'success': False,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description='ProxMPC benchmark matrix runner')
    ap.add_argument('--scenarios', default='all',
                    help="comma-separated scenario names or 'all'")
    ap.add_argument('--models', default=','.join(MODELS),
                    help='comma-separated models (unicycle,bicycle)')
    ap.add_argument('--modes', default='b1',
                    help='comma-separated run modes (b1 supported here; a,b2 via launch)')
    ap.add_argument('--controller', default='proxmpc')
    ap.add_argument('--repeats', type=int, default=0, help='override metrics.yaml repeats')
    ap.add_argument('--results-dir', default='')
    args = ap.parse_args()

    share = share_dir()
    control = load_yaml(share / 'config' / 'metrics.yaml')['control']
    repeats = args.repeats or load_yaml(share / 'config' / 'metrics.yaml')['repeats']

    results_dir = Path(args.results_dir) if args.results_dir else \
        Path(__file__).resolve().parents[1] / 'results'
    # When installed, __file__ is under install/; fall back to a stable source path.
    if not results_dir.parent.exists():
        results_dir = Path.home() / 'ros2_ws' / 'src' / 'prox_mpc' / PKG / 'results'
    results_dir.mkdir(parents=True, exist_ok=True)

    scen_names = SCENARIO_NAMES if args.scenarios == 'all' else args.scenarios.split(',')
    models = args.models.split(',')
    modes = args.modes.split(',')

    index_path = results_dir / 'scenarios.json'
    index = []
    if index_path.exists():
        try:
            index = json.load(open(index_path))
        except json.JSONDecodeError:
            index = []

    for mode in modes:
        if mode != 'b1':
            print(f"[run_matrix] mode '{mode}' is delegated to benchmark.launch.py; "
                  f'skipping in this runner.', flush=True)
            continue
        for name in scen_names:
            scn_file = share / 'config' / 'scenarios' / f'{name}.yaml'
            scn = load_yaml(scn_file)['scenario']
            timeout_s = float(scn.get('run', {}).get('timeout_s', 60))
            for model in models:
                for r in range(repeats):
                    print(f'[run_matrix] {name} | {model} | b1 | repeat {r+1}/{repeats}',
                          flush=True)
                    rec = run_b1(scn, name, model, r, control, timeout_s,
                                 results_dir, args.controller)
                    ok = rec.get('success')
                    print(f'           -> success={ok} '
                          f"ttg={rec.get('time_to_goal_s')} "
                          f"goal_err={rec.get('goal_error_m')} "
                          f"ct_rms={rec.get('cross_track_rms_m')} "
                          f"solve_p95={rec.get('solve_ms_p95')}", flush=True)
                    index = [x for x in index if not (
                        x.get('scenario') == name and x.get('model') == model and
                        x.get('mode') == mode and x.get('repeat') == r and
                        x.get('controller') == args.controller)]
                    index.append(rec)
                    json.dump(index, open(index_path, 'w'), indent=2)

    print(f'[run_matrix] wrote {index_path} ({len(index)} run records)', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
