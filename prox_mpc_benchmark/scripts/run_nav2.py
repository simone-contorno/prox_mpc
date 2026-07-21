#!/usr/bin/env python3
# Copyright 2026 Simone Contorno
# SPDX-License-Identifier: Apache-2.0

"""
Orchestrate the mode (b2) cross-controller comparison, no Gazebo.

For each controller it brings up benchmark_nav2.launch.py (Nav2 stack + kinematic
plant), starts the live metrics node, sends the scenario goal with goal_sender,
waits for the goal (the metrics node self-terminates a short settle after reaching
it) or the timeout, and records the per-run metrics JSON. Every controller drives
the identical deterministic plant, so the accuracy / timing metrics are directly
comparable. Diagnostics (solver) metrics are only populated for ProxMPC, which is
the only controller that publishes SolverDiagnostics.

Per-run JSONs land in <results>/runs/ and are merged into <results>/scenarios.json
under mode 'b2'. The environment must already be sourced (see init.sh). No git
operations.
"""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from prox_mpc_benchmark.scenario_obstacles import body_radii, obstacle_arrays
import yaml

PKG = 'prox_mpc_benchmark'
DEFAULT_CONTROLLERS = [
    'proxmpc', 'dwb', 'mppi', 'regulated_pure_pursuit', 'graceful', 'vector_pursuit',
]

# Shared robot radius (nav2_b2_base costmap) and ProxMPC safety margin. The physical
# obstacle radius is back-computed so that this robot inflation plus the marked
# obstacle disc reproduces the scenario keep-out clearance - so every controller
# faces the identical physical obstacle and ProxMPC's effective keep-out matches b1.
ROBOT_RADIUS = 0.22
SAFETY_MARGIN = 0.10


def share_dir() -> Path:
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(PKG))


def load_yaml(path: Path) -> dict:
    with open(path) as fh:
        return yaml.safe_load(fh)


def write_scan_params(path: Path, scn: dict, repeat: int = 0):
    """
    Write the scan-simulator params (obstacle field + scan geometry).

    An optional scenario `sensor.range_noise_std` [m] enables a realistic
    Gaussian range-noise model on obstacle returns; the seed is varied per
    repeat so repeats sample independent (but reproducible) noise.
    """
    motion, cx, cy, ex, ey, radius, speed, clearance = obstacle_arrays(scn)
    body = body_radii(clearance, ROBOT_RADIUS, SAFETY_MARGIN)
    noise_std = float(scn.get('sensor', {}).get('range_noise_std', 0.0))
    params = {
        'scan_frame': 'base_link',
        'odom_topic': 'odom',
        'scan_topic': 'scan',
        'rate_hz': 10.0,
        'num_beams': 360,
        'range_min': 0.05,
        'range_max': 4.0,
        'range_noise_std': noise_std,
        'range_noise_seed': int(repeat),
    }
    if motion:
        params.update({
            'obs_motion': motion,
            'obs_cx': cx, 'obs_cy': cy, 'obs_ex': ex, 'obs_ey': ey,
            'obs_radius': radius, 'obs_speed': speed, 'obs_body': body,
        })
    doc = {'scan_simulator': {'ros__parameters': params}}
    with open(path, 'w') as fh:
        yaml.safe_dump(doc, fh, default_flow_style=None, sort_keys=False)


def write_gt_params(path: Path, scn: dict):
    """
    Write the ground-truth obstacle-publisher (oracle) params.

    Uses the same obstacle field as the scan simulator, republished as perfect
    /tracked_obstacles.
    """
    motion, cx, cy, ex, ey, radius, speed, clearance = obstacle_arrays(scn)
    body = body_radii(clearance, ROBOT_RADIUS, SAFETY_MARGIN)
    params = {'odom_topic': 'odom', 'tracking_frame': 'odom', 'rate_hz': 20.0}
    if motion:
        params.update({
            'obs_motion': motion,
            'obs_cx': cx, 'obs_cy': cy, 'obs_ex': ex, 'obs_ey': ey,
            'obs_radius': radius, 'obs_speed': speed, 'obs_body': body,
        })
    doc = {'gt_obstacle_publisher': {'ros__parameters': params}}
    with open(path, 'w') as fh:
        yaml.safe_dump(doc, fh, default_flow_style=None, sort_keys=False)


def reference(scn: dict):
    s = scn['start']
    xs = [float(s['x'])]
    ys = [float(s['y'])]
    for g in scn['goals']:
        xs.append(float(g['x']))
        ys.append(float(g['y']))
    return xs, ys


def write_metrics_params(path: Path, scn: dict, control: dict, controller: str,
                         repeat: int, summary_json: Path):
    goal = scn['goals'][-1]
    ref_x, ref_y = reference(scn)
    diag_topic = '/FollowPath/diagnostics' \
        if controller in ('proxmpc', 'proxmpc_pred') else '/__none__'
    metrics = {
        'map_frame': 'map',
        'base_frame': 'base_link',
        'ref_x': ref_x,
        'ref_y': ref_y,
        'goal_x': float(goal['x']),
        'goal_y': float(goal['y']),
        'goal_theta': float(goal.get('yaw', 0.0)),
        'goal_tol': float(control['goal_tol']),
        'diagnostics_topic': diag_topic,
        'summary_json': str(summary_json),
        'scenario': scn['name'],
        'model': scn.get('models', ['unicycle'])[0],
        'mode': 'b2',
        'controller': controller,
        'repeat': int(repeat),
        'auto_exit': True,
        'settle_s': 1.5,
        'robot_radius': ROBOT_RADIUS,
    }
    # Same obstacle field the scan simulator uses, so the clearance metric measures
    # the obstacles the controller actually faced (controller-agnostic).
    motion, cx, cy, ex, ey, radius, speed, clearance = obstacle_arrays(scn)
    body = body_radii(clearance, ROBOT_RADIUS, SAFETY_MARGIN)
    if motion:
        metrics.update({
            'obs_motion': motion,
            'obs_cx': cx, 'obs_cy': cy, 'obs_ex': ex, 'obs_ey': ey,
            'obs_radius': radius, 'obs_speed': speed, 'obs_body': body,
        })
    doc = {'prox_mpc_metrics': {'ros__parameters': metrics}}
    with open(path, 'w') as fh:
        yaml.safe_dump(doc, fh, default_flow_style=None, sort_keys=False)


def find_pid_by_comm(prefix):
    """Return the first PID whose /proc comm starts with prefix, or None."""
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            with open(f'/proc/{entry}/comm') as fh:
                comm = fh.read().strip()
        except OSError:
            continue
        if comm.startswith(prefix):
            return int(entry)
    return None


def find_controller_pid():
    """Return the controller_server PID (its /proc comm), or None if not up yet."""
    return find_pid_by_comm('controller_serv')  # comm is truncated to 15 chars


def popen_group(cmd, log_path):
    log = open(log_path, 'w')
    proc = subprocess.Popen(
        cmd, stdout=log, stderr=subprocess.STDOUT,
        env=dict(os.environ), preexec_fn=os.setsid)
    return proc, log


def terminate(proc):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=8.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


def run_cell(scn, controller, repeat, control, results_dir, robot, map_yaml,
             warmup_s, timeout_s, oracle=False):
    runs_dir = results_dir / 'runs'
    runs_dir.mkdir(parents=True, exist_ok=True)
    tag = f"{scn['name']}__{scn.get('models', ['unicycle'])[0]}__b2__{controller}__r{repeat}"
    predictive = controller == 'proxmpc_pred'
    # Oracle mode: feed proxmpc_pred perfect obstacle knowledge (ground-truth
    # publisher) instead of the IMM tracker, for the feasibility gate.
    use_tracker = predictive and not oracle
    use_oracle = predictive and oracle
    summary_json = runs_dir / f'{tag}.json'
    resource_json = runs_dir / f'{tag}.resource.json'
    tracker_resource_json = runs_dir / f'{tag}.tracker_resource.json'
    metrics_params = runs_dir / f'{tag}.metrics.yaml'
    scan_params = runs_dir / f'{tag}.scan.yaml'
    if summary_json.exists():
        summary_json.unlink()
    if resource_json.exists():
        resource_json.unlink()
    if tracker_resource_json.exists():
        tracker_resource_json.unlink()
    write_metrics_params(metrics_params, scn, control, controller, repeat, summary_json)
    write_scan_params(scan_params, scn, repeat)
    gt_params = runs_dir / f'{tag}.gt.yaml'
    if use_oracle:
        write_gt_params(gt_params, scn)

    s = scn['start']
    goal = scn['goals'][-1]
    points = f"{goal['x']},{goal['y']},{goal.get('yaw', 0.0)}"

    stack, stack_log = popen_group(
        ['ros2', 'launch', PKG, 'benchmark_nav2.launch.py',
         f'controller:={controller}', f'robot:={robot}', f'map_yaml:={map_yaml}',
         f"start_x:={s['x']}", f"start_y:={s['y']}", f"start_theta:={s.get('yaw', 0.0)}",
         f'scan_params_file:={scan_params}',
         f"obstacle_tracker:={'true' if use_tracker else 'false'}"],
        runs_dir / f'{tag}.stack.log')

    gt_proc = None
    if use_oracle:
        gt_proc, _ = popen_group(
            ['ros2', 'run', PKG, 'gt_obstacle_publisher.py',
             '--ros-args', '--params-file', str(gt_params)],
            runs_dir / f'{tag}.gt.log')

    metrics = goal_proc = sampler = tracker_sampler = None
    try:
        time.sleep(warmup_s)  # let the lifecycle manager activate the servers
        # Instrument the controller_server process for the embedded-resource metrics.
        pid = None
        for _ in range(10):
            pid = find_controller_pid()
            if pid is not None:
                break
            time.sleep(0.5)
        if pid is not None:
            sampler, _ = popen_group(
                ['ros2', 'run', PKG, 'resource_sampler.py', '--pid', str(pid),
                 '--out', str(resource_json), '--cmd-topic', '/cmd_vel',
                 '--compute-topic', '/FollowPath/compute_time_ms'],
                runs_dir / f'{tag}.resource.log')
        # Predictive mode adds a companion tracker process; sample it separately so
        # the true system cost of prediction (controller + tracker) is reported, not
        # just the controller_server the decorator times.
        if use_tracker:
            tpid = find_pid_by_comm('obstacle_tracke')  # comm truncated to 15 chars
            if tpid is not None:
                tracker_sampler, _ = popen_group(
                    ['ros2', 'run', PKG, 'resource_sampler.py', '--pid', str(tpid),
                     '--out', str(tracker_resource_json), '--cmd-topic', '/cmd_vel',
                     '--node-name', 'prox_mpc_tracker_resource_sampler'],
                    runs_dir / f'{tag}.tracker_resource.log')
        metrics, _ = popen_group(
            ['ros2', 'run', PKG, 'metrics_node',
             '--ros-args', '--params-file', str(metrics_params)],
            runs_dir / f'{tag}.metrics.log')
        # The metrics node anchors its obstacle clock to the first robot motion
        # IT observes. If the goal races ahead of its subscriptions (slow
        # discovery under load), the clock starts late and the whole obstacle
        # field is evaluated out of phase - false collision/clearance numbers.
        # Gate the goal on the node being discoverable.
        for _ in range(20):
            probe = subprocess.run(
                ['ros2', 'node', 'list'], capture_output=True, text=True, timeout=10.0)
            if '/prox_mpc_metrics' in probe.stdout:
                break
            time.sleep(0.5)
        goal_proc, _ = popen_group(
            ['ros2', 'run', PKG, 'goal_sender.py', '--points', points,
             '--timeout', str(timeout_s)],
            runs_dir / f'{tag}.goal.log')

        deadline = time.time() + timeout_s + warmup_s + 10.0
        while time.time() < deadline:
            if metrics.poll() is not None:
                break  # metrics node self-exited on goal+settle
            time.sleep(0.3)
        if metrics.poll() is None:
            # Timed out without the goal: SIGINT so the node writes success=false.
            terminate(metrics)
    finally:
        terminate(goal_proc)
        terminate(metrics)
        terminate(sampler)  # SIGINT before the stack so it samples a live PID
        terminate(tracker_sampler)
        terminate(gt_proc)
        terminate(stack)
        stack_log.close()
        time.sleep(3.0)  # let DDS discovery settle before the next cell

    if summary_json.exists():
        rec = json.load(open(summary_json))
        rec['status'] = 'ok'
        if resource_json.exists():
            try:
                rec.update(json.load(open(resource_json)))
            except (json.JSONDecodeError, OSError):
                pass
        if predictive and tracker_resource_json.exists():
            try:
                tr = json.load(open(tracker_resource_json))
                rec['tracker_cpu_mean_pct'] = tr.get('cpu_mean_pct')
                rec['tracker_cpu_peak_pct'] = tr.get('cpu_peak_pct')
                rec['tracker_rss_peak_mb'] = tr.get('rss_peak_mb')
            except (json.JSONDecodeError, OSError):
                pass
        return rec
    return {
        'scenario': scn['name'], 'model': scn.get('models', ['unicycle'])[0],
        'mode': 'b2', 'controller': controller, 'repeat': repeat,
        'status': 'no_summary', 'success': False,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description='ProxMPC mode-b2 cross-controller runner')
    ap.add_argument('--scenario', default='nav2_open')
    ap.add_argument('--controllers', default=','.join(DEFAULT_CONTROLLERS))
    ap.add_argument('--robot', default='waffle')
    ap.add_argument('--repeats', type=int, default=0, help='override scenario repeats')
    ap.add_argument('--repeat-indices', default='',
                    help='comma-separated repeat indices to run (overrides --repeats); '
                         'used to re-run only specific runs, e.g. a transient bringup failure')
    ap.add_argument('--warmup', type=float, default=10.0, help='stack activation wait [s]')
    ap.add_argument('--results-dir', default='')
    ap.add_argument('--oracle', action='store_true',
                    help='feed proxmpc_pred ground-truth obstacles (feasibility-gate oracle) '
                         'instead of the IMM tracker')
    args = ap.parse_args()

    share = share_dir()
    control = load_yaml(share / 'config' / 'metrics.yaml')['control']
    scn = load_yaml(share / 'config' / 'scenarios' / f'{args.scenario}.yaml')['scenario']
    map_yaml = scn.get('map', {}).get('yaml', 'prox_mpc_open.yaml')
    repeats = args.repeats or int(scn.get('run', {}).get('repeats', 3))
    timeout_s = float(scn.get('run', {}).get('timeout_s', 60))
    controllers = args.controllers.split(',')

    results_dir = Path(args.results_dir) if args.results_dir else \
        Path(__file__).resolve().parents[1] / 'results'
    if not results_dir.parent.exists():
        results_dir = Path.home() / 'ros2_ws' / 'src' / 'prox_mpc' / PKG / 'results'
    results_dir.mkdir(parents=True, exist_ok=True)

    index_path = results_dir / 'scenarios.json'
    index = []
    if index_path.exists():
        try:
            index = json.load(open(index_path))
        except json.JSONDecodeError:
            index = []

    rep_indices = ([int(x) for x in args.repeat_indices.split(',') if x != '']
                   if args.repeat_indices else list(range(repeats)))

    for controller in controllers:
        for r in rep_indices:
            print(f'[run_nav2] {scn["name"]} | {controller} | b2 | repeat {r}',
                  flush=True)
            rec = run_cell(scn, controller, r, control, results_dir, args.robot,
                           map_yaml, args.warmup, timeout_s, oracle=args.oracle)
            print(f"           -> success={rec.get('success')} "
                  f"ttg={rec.get('time_to_goal_s')} "
                  f"goal_err={rec.get('goal_error_m')} "
                  f"ct_rms={rec.get('cross_track_rms_m')} "
                  f"path={rec.get('path_length_m')}", flush=True)
            index = [x for x in index if not (
                x.get('scenario') == scn['name'] and x.get('mode') == 'b2' and
                x.get('controller') == controller and x.get('repeat') == r)]
            index.append(rec)
            json.dump(index, open(index_path, 'w'), indent=2)

    print(f'[run_nav2] wrote {index_path} ({len(index)} run records)', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
