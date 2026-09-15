# Copyright 2026 Simone Contorno
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for record_scenarios.py's clip timing (no ROS graph, no display)."""

import importlib.util
from pathlib import Path

import yaml

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
SCENARIOS = Path(__file__).resolve().parents[1] / 'config' / 'scenarios'


def _load():
    spec = importlib.util.spec_from_file_location(
        'record_scenarios', SCRIPTS / 'record_scenarios.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


rs = _load()


def test_clip_opens_a_preroll_before_first_motion():
    # Capture started at t = 100 s and the robot moved at 106.5 s: the clip opens
    # 6.2 s in, so the robot is seen at rest for the 0.3 s preroll and no longer.
    assert abs(rs.trim_offset(100.0, 106.5, 0.3) - 6.2) < 1e-9


def test_clip_never_opens_before_the_capture():
    # Motion before the capture began (or within the preroll of it) clamps to zero.
    assert rs.trim_offset(100.0, 100.1, 0.3) == 0.0
    assert rs.trim_offset(100.0, 99.0, 0.3) == 0.0


def test_a_robot_that_never_moved_gives_no_offset():
    # None is the caller's cue that the navigation failed, not a zero offset.
    assert rs.trim_offset(100.0, None, 0.3) is None


def test_trim_command_cuts_the_requested_window():
    cmd = rs.build_trim_cmd(Path('raw.mp4'), 6.2, 25.0, Path('out.mp4'))
    assert cmd[cmd.index('-ss') + 1] == '6.200'
    assert cmd[cmd.index('-t') + 1] == '25.000'
    assert cmd[cmd.index('-i') + 1] == 'raw.mp4'
    assert cmd[-1] == 'out.mp4'


def test_scenario_duration_overrides_the_default():
    assert rs.clip_duration({'video': {'duration_s': 25.0}}, 20.0) == 25.0
    assert rs.clip_duration({}, 20.0) == 20.0
    assert rs.clip_duration({'video': None}, 20.0) == 20.0


def test_dynamic_circle_records_long_enough_to_finish():
    # The orbit cell is the one that needs the longer clip; the others keep the
    # default so each scenario's four controllers stay length-synced.
    scn = yaml.safe_load((SCENARIOS / 'dynamic_circle.yaml').read_text())['scenario']
    assert rs.clip_duration(scn, 20.0) == 25.0
    scn = yaml.safe_load((SCENARIOS / 'static_box.yaml').read_text())['scenario']
    assert rs.clip_duration(scn, 20.0) == 20.0


def test_rviz_runs_at_the_lowest_priority_without_offload():
    cmd = rs.rviz_command(Path('recording.rviz'), False, False)
    assert cmd[:4] == ['nice', '-n', '19', 'rviz2']
    assert not any(arg.startswith(('__NV_', '__GLX_')) for arg in cmd)
    assert '--fullscreen' not in cmd


def test_gpu_offload_moves_only_rviz_to_the_nvidia_gpu():
    cmd = rs.rviz_command(Path('recording.rviz'), True, False)
    assert cmd[0] == 'env'
    assert '__NV_PRIME_RENDER_OFFLOAD=1' in cmd
    assert '__GLX_VENDOR_LIBRARY_NAME=nvidia' in cmd
    # Apart from the offload variables it is the same niced RViz launch.
    assert cmd[cmd.index('nice'):] == rs.rviz_command(Path('recording.rviz'), False, False)


def test_fullscreen_is_an_rviz_option_not_a_ros_argument():
    cmd = rs.rviz_command(Path('recording.rviz'), True, True)
    # RViz options have to precede --ros-args, which hands the rest to ROS.
    assert cmd.index('--fullscreen') < cmd.index('--ros-args')
