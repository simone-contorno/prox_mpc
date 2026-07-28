#!/usr/bin/env python3
# Copyright 2026 Simone Contorno
# SPDX-License-Identifier: Apache-2.0

"""
Record the mode (b2) demo scenarios to per-scenario screen-capture clips.

For each scenario this stands up the mode-b2 Nav2 stack (benchmark_nav2.launch.py:
kinematic plant, scan simulator, the selected controller, and - with
obstacle_tracker:=true - the IMM tracker), plus a path-corrected waffle
robot_state_publisher and RViz, waits for lifecycle activation, screen-records a
fixed-length clip with ffmpeg x11grab, sends the scenario goal, and tears the whole
tree down with the run_nav2 process-group SIGINT -> grace -> SIGKILL discipline.

The b2 stack runs on wall/system time (no /clock publisher), so RViz and the
robot_state_publisher use use_sim_time:=false - the opposite of the Gazebo demo.

ffmpeg is the recorder. Clips are fixed-duration, so the four are length-synced and
combine cleanly into a 2x2 grid (see combine_grid.sh). Nothing is committed:
results/ is gitignored. The environment must already be sourced (see init.sh).
"""

import argparse
import importlib.util
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
import yaml

PKG = 'prox_mpc_benchmark'
TB3_SIM_PKG = 'nav2_minimal_tb3_sim'

# The stock waffle URDF points its RViz meshes at models/<name>.dae, but they
# install under models/turtlebot3_model/meshes/<name>.dae; correcting the subpath is
# what lets the RViz RobotModel load instead of raising "Error loading geometries"
# (mirrors prox_mpc_demo/launch/nav2_simulation.launch.py).
WAFFLE_MESHES = ('waffle_base', 'tire', 'lds', 'r200')

DEFAULT_SCENARIOS = 'nav2_open,static_box,dynamic_line_forward,dynamic_circle'

# Grace after DDS teardown before the next scenario, so a lingering participant does
# not shadow the fresh stack's discovery (mirrors run_nav2's inter-cell settle).
INTER_SCENARIO_SETTLE_S = 3.0

# The bt_navigator aborts a goal if the planner does not acknowledge within its
# ~20 ms action server_timeout; a transient load/discovery hiccup trips it. Send the
# goal up to GOAL_ATTEMPTS times, treating a non-zero exit within GOAL_ABORT_PROBE_S
# as a spurious abort (a real navigation stays busy well past the probe window).
GOAL_ATTEMPTS = 4
GOAL_ABORT_PROBE_S = 4.0


def load_run_nav2():
    """
    Import run_nav2.py from this script's own directory (symlink-install safe).

    Under colcon --symlink-install the installed script is a symlink back to the
    source tree, so run_nav2.py resolves next to this file either way.
    """
    here = Path(__file__).resolve().parent
    spec = importlib.util.spec_from_file_location('run_nav2', here / 'run_nav2.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def waffle_description() -> str:
    """
    Return the path-corrected waffle URDF, re-rooted at base_link.

    The mode-b2 kinematic_plant broadcasts odom -> base_link, but the stock waffle
    URDF roots at base_footprint (a fixed base_footprint -> base_link joint).
    Publishing it unaltered gives base_link two TF parents (odom from the plant and
    base_footprint from this RSP); the resulting conflict corrupts the costmap's
    scan transform (phantom, uncleared obstacle marks that box the robot in) and
    reddens the RViz RobotModel. Dropping base_footprint and its joint makes
    base_link the URDF root, so odom -> base_link is its only parent and the
    costmap sees the same clean TF the headless benchmark does.
    """
    urdf = Path(get_package_share_directory(TB3_SIM_PKG)) / 'urdf' / \
        'turtlebot3_waffle.urdf'
    text = urdf.read_text()
    for mesh in WAFFLE_MESHES:
        text = text.replace(
            f'models/{mesh}.dae', f'models/turtlebot3_model/meshes/{mesh}.dae')
    text = re.sub(r'\s*<link name="base_footprint"\s*/>', '', text)
    text = re.sub(r'\s*<joint name="base_joint".*?</joint>', '', text, flags=re.DOTALL)
    return text


def write_rsp_params(path: Path, description: str):
    """Write robot_state_publisher params: the corrected URDF on wall time."""
    doc = {'robot_state_publisher': {'ros__parameters': {
        'use_sim_time': False, 'robot_description': description}}}
    with open(path, 'w') as fh:
        yaml.safe_dump(doc, fh, default_flow_style=False, sort_keys=False)


def write_marker_params(path: Path, rn, scn):
    """
    Write the obstacle_markers params.

    Uses the same obstacle field the scan uses, so the drawn cylinder bodies line
    up with the laser returns and the costmap.
    """
    motion, cx, cy, ex, ey, radius, speed, clearance = rn.obstacle_arrays(scn)
    body = rn.body_radii(clearance, rn.ROBOT_RADIUS, rn.SAFETY_MARGIN)
    params = {'tracking_frame': 'odom', 'odom_topic': 'odom', 'rate_hz': 20.0}
    if motion:
        params.update({
            'obs_motion': motion, 'obs_cx': cx, 'obs_cy': cy,
            'obs_ex': ex, 'obs_ey': ey, 'obs_radius': radius,
            'obs_speed': speed, 'obs_body': body,
        })
    doc = {'obstacle_markers': {'ros__parameters': params}}
    with open(path, 'w') as fh:
        yaml.safe_dump(doc, fh, default_flow_style=None, sort_keys=False)


def build_ffmpeg_cmd(display, offset, resolution, framerate, duration, out_path):
    """Build the x11grab record command; -t makes the clip self-terminating."""
    ox, oy = (v.strip() for v in offset.split(','))
    grab = f'{display}.0+{ox},{oy}'
    return [
        'ffmpeg', '-y', '-nostdin',
        '-f', 'x11grab',
        '-framerate', str(framerate),
        '-video_size', resolution,
        '-i', grab,
        '-t', str(duration),
        '-c:v', 'libx264',
        '-preset', 'veryfast',
        '-pix_fmt', 'yuv420p',
        '-movflags', '+faststart',
        str(out_path),
    ]


def place_rviz_window(offset, resolution, log):
    """
    Best-effort move/resize of the RViz window to fill the capture region.

    Deliberately omits windowactivate: it needs EWMH (_NET_ACTIVE_WINDOW), which a
    bare X server without a window manager (the headless Xvfb case) does not
    provide, and a failed activate aborts the chained windowmove/windowsize. Plain
    windowmove/windowsize are honoured directly by the X server with no WM to
    arbitrate. xdotool is optional; any failure is logged and ignored so a
    fixed-region grab still succeeds without it.
    """
    if shutil.which('xdotool') is None:
        return
    ox, oy = (v.strip() for v in offset.split(','))
    w, h = resolution.split('x')
    try:
        subprocess.run(
            ['xdotool', 'search', '--sync', '--name', 'RViz',
             'windowmove', ox, oy, 'windowsize', w, h],
            stdout=log, stderr=subprocess.STDOUT, timeout=10.0, check=False)
    except (subprocess.SubprocessError, OSError):
        pass


def record_one(scenario, args, rn, out_dir, logs_dir, description):
    """Bring up one scenario's stack, record it, send its goal, then tear down."""
    share = rn.share_dir()
    scn = rn.load_yaml(
        share / 'config' / 'scenarios' / f'{scenario}.yaml')['scenario']
    map_yaml = scn.get('map', {}).get('yaml', 'prox_mpc_open.yaml')
    start = scn['start']
    goal = scn['goals'][-1]
    points = f"{goal['x']},{goal['y']},{goal.get('yaw', 0.0)}"

    scan_params = logs_dir / f'{scenario}.scan.yaml'
    rsp_params = logs_dir / f'{scenario}.rsp.yaml'
    marker_params = logs_dir / f'{scenario}.markers.yaml'
    rn.write_scan_params(scan_params, scn, 0)
    write_rsp_params(rsp_params, description)
    write_marker_params(marker_params, rn, scn)

    out_path = out_dir / f'{scenario}.mp4'
    # A recording-only RViz profile: filled to the display, docks and the Nav2
    # panel hidden, camera framing the corridor. Resolve the installed copy first,
    # falling back to the source tree so it works before a rebuild (the script is
    # symlink-installed, so __file__ resolves into the source package).
    rviz_cfg = share / 'rviz' / 'recording.rviz'
    if not rviz_cfg.is_file():
        rviz_cfg = Path(__file__).resolve().parents[1] / 'rviz' / 'recording.rviz'

    procs = []  # (Popen, log_file) pairs, torn down in the finally block

    def launch(role, cmd):
        proc, log = rn.popen_group(cmd, logs_dir / f'{scenario}.{role}.log')
        procs.append((proc, log))
        return proc

    launch('stack', [
        'ros2', 'launch', PKG, 'benchmark_nav2.launch.py',
        f'controller:={args.controller}', f'robot:={args.robot}',
        f'map_yaml:={map_yaml}',
        f"start_x:={start['x']}", f"start_y:={start['y']}",
        f"start_theta:={start.get('yaw', 0.0)}",
        f'scan_params_file:={scan_params}',
        f"obstacle_tracker:={'true' if args.controller == 'proxmpc_pred' else 'false'}"])
    launch('robot_state_publisher', [
        'ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
        '--ros-args', '--params-file', str(rsp_params)])
    # The waffle's wheels are continuous joints; without joint states the RSP
    # cannot transform the wheel links and RViz reddens the RobotModel. This
    # publishes zeroed joint states (from /robot_description) so every link renders.
    launch('joint_state_publisher', [
        'ros2', 'run', 'joint_state_publisher', 'joint_state_publisher',
        '--ros-args', '-p', 'use_sim_time:=false'])
    # Ground-truth obstacle bodies (viz only, on /obstacle_bodies) so the video
    # shows the actual obstacle, exactly tracking, alongside its costmap footprint.
    # Run by source-sibling path so it works without a rebuild (symlink-install
    # safe), the same way run_nav2 is imported.
    markers_py = Path(__file__).resolve().parent / 'obstacle_markers.py'
    launch('obstacle_markers', [
        'python3', str(markers_py),
        '--ros-args', '--params-file', str(marker_params)])
    # RViz renders on software GL under a virtual display (llvmpipe), which is
    # CPU-heavy. Run it at the lowest scheduling priority so the time-sensitive Nav2
    # servers (planner acknowledge, controller solve, costmap clearing) always win
    # the CPU: at normal priority the contention starves the planner (goals abort on
    # the bt_navigator action-acknowledge timeout) and the local costmap (obstacle
    # clearing lags, leaving a moving-obstacle inflation trail).
    launch('rviz', [
        'nice', '-n', '19', 'rviz2', '-d', str(rviz_cfg),
        '--ros-args', '-p', 'use_sim_time:=false'])

    try:
        time.sleep(args.warmup)  # let the lifecycle manager activate the servers
        with open(logs_dir / f'{scenario}.xdotool.log', 'w') as xlog:
            place_rviz_window(args.offset, args.resolution, xlog)
        cmd = build_ffmpeg_cmd(args.display, args.offset, args.resolution,
                               args.framerate, args.duration, out_path)
        print(f'[record] {scenario} ffmpeg: {shlex.join(cmd)}', flush=True)
        ffmpeg = launch('ffmpeg', cmd)
        time.sleep(args.goal_delay)
        goal_cmd = ['ros2', 'run', PKG, 'goal_sender.py', '--points', points,
                    '--timeout', str(args.timeout)]
        for attempt in range(GOAL_ATTEMPTS):
            goal = launch('goal', goal_cmd)
            try:
                # Still running after the probe window -> the goal was accepted and
                # navigation is under way; stop retrying.
                if goal.wait(timeout=GOAL_ABORT_PROBE_S) == 0:
                    break  # reached the goal already (fast cell)
            except subprocess.TimeoutExpired:
                break
            print(f'[record] {scenario} goal aborted fast (attempt '
                  f'{attempt + 1}/{GOAL_ATTEMPTS}); retrying', flush=True)
            time.sleep(1.0)
        try:
            ffmpeg.wait(timeout=args.duration + 30.0)
        except subprocess.TimeoutExpired:
            pass
    finally:
        # Reverse order tears down the goal sender and ffmpeg (which finalizes the
        # mp4 on SIGINT) before the stack; process-group SIGINT -> grace -> SIGKILL.
        for proc, _ in reversed(procs):
            rn.terminate(proc)
        for _, log in procs:
            log.close()
        # Belt-and-suspenders: a Nav2 node occasionally outlives the launch's SIGINT
        # (slow lifecycle shutdown) and poisons the next scenario's graph -- stale
        # odom freezes the robot at the previous goal, colliding map_servers blank the
        # map. Force-kill any b2 stack straggler by executable before the settle.
        subprocess.run(
            ['pkill', '-9', '-f',
             'lib/(nav2_|prox_mpc_benchmark|prox_mpc_obstacle_tracker)/[a-z_]+ --ros-args'],
            check=False)
        time.sleep(INTER_SCENARIO_SETTLE_S)
    return out_path


def main() -> int:
    ap = argparse.ArgumentParser(
        description='Record the mode-b2 demo scenarios to screen-capture clips.')
    ap.add_argument('--scenarios', default=DEFAULT_SCENARIOS)
    ap.add_argument('--controller', default='proxmpc_pred')
    ap.add_argument('--robot', default='waffle')
    ap.add_argument('--duration', type=float, default=20.0, help='clip length [s]')
    ap.add_argument('--resolution', default='1920x1080', help='grab size WxH')
    ap.add_argument('--offset', default='0,0',
                    help="grab top-left origin 'x,y' -> x11grab input :0.0+x,y")
    ap.add_argument('--display', default=os.environ.get('DISPLAY', ':0'))
    ap.add_argument('--framerate', type=int, default=30)
    ap.add_argument('--warmup', type=float, default=14.0,
                    help='stack activation wait before recording [s]')
    ap.add_argument('--goal-delay', type=float, default=3.0,
                    help='wait after recording starts before sending the goal [s]')
    ap.add_argument('--timeout', type=float, default=55.0, help='goal timeout [s]')
    ap.add_argument('--out-dir', default='', help='clip output dir')
    args = ap.parse_args()

    rn = load_run_nav2()
    os.environ['DISPLAY'] = args.display  # RViz renders on the capture display

    if args.out_dir:
        out_dir = Path(args.out_dir)
    else:
        results = Path(__file__).resolve().parents[1] / 'results'
        if not results.parent.exists():
            results = Path.home() / 'ros2_ws' / 'src' / 'prox_mpc' / PKG / 'results'
        out_dir = results / 'videos'
    logs_dir = out_dir / 'logs'
    out_dir.mkdir(parents=True, exist_ok=True)
    logs_dir.mkdir(parents=True, exist_ok=True)

    description = waffle_description()
    scenarios = [s.strip() for s in args.scenarios.split(',') if s.strip()]
    failed = []
    for scenario in scenarios:
        print(f'[record] === {scenario} ===', flush=True)
        try:
            out_path = record_one(
                scenario, args, rn, out_dir, logs_dir, description)
            print(f'[record] wrote {out_path}', flush=True)
        except Exception as exc:  # one bad scenario must not sink the rest
            failed.append(scenario)
            print(f'[record] {scenario} FAILED: {exc}', flush=True)

    print(f'[record] done: {len(scenarios) - len(failed)}/{len(scenarios)} clips '
          f'in {out_dir}', flush=True)
    if failed:
        print(f'[record] failed scenarios: {",".join(failed)}', flush=True)
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
