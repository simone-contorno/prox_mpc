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

ffmpeg is the recorder. It captures a raw window long enough to cover the stack's
start-up latency and a goal retry, and each clip is then cut to start at the moment
the robot first moves - the same event that releases the b2 obstacles, read from
odom - so a clip opens on the scene coming alive rather than on seconds of a
frozen robot. Clips are fixed-duration per scenario (a scenario may lengthen its
own with video.duration_s), so the four controllers of one scenario are
length-synced and combine cleanly into a 2x2 grid (see combine_grid.sh). Nothing
is committed: results/ is gitignored. The environment must already be sourced
(see init.sh).
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
import threading
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

# A clip opens this long before the robot's first motion, so the robot is seen at
# rest for an instant rather than already moving on the first frame.
PREROLL_S = 0.3

# Raw capture beyond the goal delay and the clip itself: the stack takes a few
# seconds from goal to first motion, and a spuriously aborted goal is retried after
# GOAL_ABORT_PROBE_S, so this covers the latency plus one retry.
CAPTURE_MARGIN_S = 12.0

# The robot counts as moving once it leaves its first pose by this much - the same
# motion_eps the scan simulator and the obstacle publishers gate their clocks on.
MOTION_EPS_M = 1.0e-3


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


def clip_duration(scn, default):
    """Clip length [s] for a scenario: its own video.duration_s, else the default."""
    video = scn.get('video') or {}
    return float(video.get('duration_s', default))


def trim_offset(capture_start, motion_start, preroll):
    """
    Seconds into the raw capture at which the clip should begin.

    Both times are on one monotonic clock. None when the robot never moved, which
    leaves the caller to decide what to keep; otherwise the moment of first motion
    less the preroll, clamped to the start of the capture.
    """
    if motion_start is None:
        return None
    return max(0.0, motion_start - capture_start - preroll)


def build_trim_cmd(raw_path, offset, duration, out_path):
    """Cut `duration` seconds from `offset` into the raw capture, re-encoded to be exact."""
    return [
        'ffmpeg', '-y', '-nostdin', '-v', 'error',
        '-i', str(raw_path),
        '-ss', f'{offset:.3f}',
        '-t', f'{duration:.3f}',
        '-c:v', 'libx264',
        '-preset', 'veryfast',
        '-pix_fmt', 'yuv420p',
        '-movflags', '+faststart',
        str(out_path),
    ]


# NVIDIA PRIME render offload for a single process on a hybrid-graphics host, whose X
# server otherwise renders every GL client on the integrated GPU.
GPU_OFFLOAD_ENV = ('__NV_PRIME_RENDER_OFFLOAD=1', '__GLX_VENDOR_LIBRARY_NAME=nvidia')


def rviz_command(rviz_cfg, gpu_offload, fullscreen):
    """
    Return the RViz launch argv, at the lowest scheduling priority.

    With gpu_offload the PRIME offload variables are set for RViz alone through
    env(1), so only the renderer moves to the discrete GPU and the Nav2 stack keeps
    its environment. Offload needs a real X server running the NVIDIA driver; a
    virtual display such as Xvfb has no hardware GL for it to reach.

    With fullscreen RViz covers the whole screen, which on a desktop session is the
    only way to keep the window manager's panels and the window's own title bar out
    of the capture: a managed window cannot be moved over them.
    """
    cmd = ['nice', '-n', '19', 'rviz2', '-d', str(rviz_cfg)]
    if fullscreen:
        cmd.append('--fullscreen')
    cmd += ['--ros-args', '-p', 'use_sim_time:=false']
    return ['env', *GPU_OFFLOAD_ENV, *cmd] if gpu_offload else cmd


class MotionWatch:
    """
    Record the monotonic time of the robot's first motion, from odom.

    The b2 obstacles start on this same event, so it is the instant the scene comes
    alive. A background executor spins the subscription while the scenario runs.
    rclpy is imported here rather than at module level so the pure helpers above
    stay importable without a ROS context.
    """

    def __init__(self, topic='odom', eps=MOTION_EPS_M):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from nav_msgs.msg import Odometry

        if not rclpy.ok():
            rclpy.init()
        self.motion_start = None
        self._eps = eps
        self._first = None
        self._node = rclpy.create_node('record_scenarios_motion_watch')
        self._node.create_subscription(Odometry, topic, self._on_odom, 20)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        if self._first is None:
            self._first = (p.x, p.y)
        elif self.motion_start is None and \
                ((p.x - self._first[0]) ** 2 + (p.y - self._first[1]) ** 2) ** 0.5 > self._eps:
            self.motion_start = time.monotonic()

    def close(self):
        """Stop spinning and release the node."""
        self._executor.shutdown()
        self._thread.join(timeout=5.0)
        self._node.destroy_node()


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
    raw_path = logs_dir / f'{scenario}.raw.mp4'
    duration = clip_duration(scn, args.duration)
    raw_len = args.goal_delay + CAPTURE_MARGIN_S + duration
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
    # RViz on a virtual display renders in software (llvmpipe), which is CPU-heavy, so
    # it runs at the lowest scheduling priority and the time-sensitive Nav2 servers
    # (planner acknowledge, controller solve, costmap clearing) always win the CPU: at
    # normal priority the contention starves the planner (goals abort on the
    # bt_navigator action-acknowledge timeout) and the local costmap (obstacle clearing
    # lags, leaving a moving-obstacle inflation trail). --gpu-offload takes the
    # rendering off the CPU entirely, on a real X server with the NVIDIA driver.
    launch('rviz', rviz_command(rviz_cfg, args.gpu_offload, args.fullscreen))

    watch = MotionWatch()
    capture_start = None
    try:
        time.sleep(args.warmup)  # let the lifecycle manager activate the servers
        # A fullscreen window already covers the screen, so it is not moved or resized.
        if not args.fullscreen:
            with open(logs_dir / f'{scenario}.xdotool.log', 'w') as xlog:
                place_rviz_window(args.offset, args.resolution, xlog)
        cmd = build_ffmpeg_cmd(args.display, args.offset, args.resolution,
                               args.framerate, raw_len, raw_path)
        print(f'[record] {scenario} ffmpeg: {shlex.join(cmd)}', flush=True)
        capture_start = time.monotonic()
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
            ffmpeg.wait(timeout=raw_len + 30.0)
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
        watch.close()
        time.sleep(INTER_SCENARIO_SETTLE_S)

    # Cut the clip to open as the robot first moves. A robot that never moved has
    # nothing worth trimming to; keep the head of the capture and say so, so a
    # failed navigation is not mistaken for a good clip.
    offset = None if capture_start is None else \
        trim_offset(capture_start, watch.motion_start, PREROLL_S)
    if offset is None:
        print(f'[record] {scenario} WARNING: the robot never moved; keeping the first '
              f'{duration:.0f} s of the capture untrimmed', flush=True)
        offset = 0.0
    elif offset + duration > raw_len:
        print(f'[record] {scenario} WARNING: motion began {offset:.1f} s in, so the clip '
              f'runs past the {raw_len:.0f} s capture and will be short', flush=True)
    with open(logs_dir / f'{scenario}.trim.log', 'w') as tlog:
        subprocess.run(build_trim_cmd(raw_path, offset, duration, out_path),
                       stdout=tlog, stderr=subprocess.STDOUT, check=True)
    raw_path.unlink(missing_ok=True)
    print(f'[record] {scenario} clip: {duration:.0f} s from {offset:.1f} s into the capture',
          flush=True)
    return out_path


def main() -> int:
    ap = argparse.ArgumentParser(
        description='Record the mode-b2 demo scenarios to screen-capture clips.')
    ap.add_argument('--scenarios', default=DEFAULT_SCENARIOS)
    ap.add_argument('--controller', default='proxmpc_pred')
    ap.add_argument('--robot', default='waffle')
    ap.add_argument('--duration', type=float, default=20.0,
                    help="clip length [s]; a scenario's video.duration_s overrides it")
    ap.add_argument('--resolution', default='1920x1080', help='grab size WxH')
    ap.add_argument('--offset', default='0,0',
                    help="grab top-left origin 'x,y' -> x11grab input :0.0+x,y")
    ap.add_argument('--display', default=os.environ.get('DISPLAY', ':0'))
    ap.add_argument('--framerate', type=int, default=30)
    ap.add_argument('--warmup', type=float, default=14.0,
                    help='stack activation wait before recording [s]')
    ap.add_argument('--goal-delay', type=float, default=3.0,
                    help='wait after recording starts before sending the goal [s]; '
                         'the clip is cut at first motion, so this is not dead time')
    ap.add_argument('--timeout', type=float, default=55.0, help='goal timeout [s]')
    ap.add_argument('--out-dir', default='', help='clip output dir')
    ap.add_argument('--gpu-offload', action='store_true',
                    help='render RViz on the NVIDIA GPU through PRIME render offload; '
                         'needs a real X server with the NVIDIA driver, not Xvfb')
    ap.add_argument('--fullscreen', action='store_true',
                    help='start RViz fullscreen and skip the window placement, so the panels '
                         'of a desktop session stay out of the capture; set --resolution to '
                         'the full screen size')
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

    try:
        import rclpy
        if rclpy.ok():
            rclpy.shutdown()
    except ImportError:
        pass
    print(f'[record] done: {len(scenarios) - len(failed)}/{len(scenarios)} clips '
          f'in {out_dir}', flush=True)
    if failed:
        print(f'[record] failed scenarios: {",".join(failed)}', flush=True)
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
