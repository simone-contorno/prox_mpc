# Demonstration videos

Reproducible screen-capture automation for the ProxMPC demo.
It records the four mode (b2) benchmark scenarios - driven by the kinematic plant,
no Gazebo - as fixed-length clips and stitches them into one 2x2 grid video.
The four scenarios are the no-obstacle, static, dynamic-line, and dynamic-circle
cells, all run with the predictive controller (`proxmpc_pred`) and the IMM obstacle
tracker enabled so the predicted-obstacle overlay is exercised.

Everything lands under `results/videos/`, which is gitignored; the videos are a
locally regenerated artifact, not committed to the repository.

## Table of Contents

- [What is recorded](#what-is-recorded)
- [Prerequisites](#prerequisites)
- [Record the per-scenario clips](#record-the-per-scenario-clips)
- [Combine into a 2x2 grid](#combine-into-a-2x2-grid)
- [Multi-controller grids](#multi-controller-grids)
- [Outputs](#outputs)
- [Parameters](#parameters)
- [Notes and gotchas](#notes-and-gotchas)
- [License](#license)

## What is recorded

Each clip brings up the mode (b2) Nav2 stack through
[`launch/benchmark_nav2.launch.py`](../launch/benchmark_nav2.launch.py): the
kinematic plant, the scan simulator, the selected controller, and - because the
recorder passes `obstacle_tracker:=true` - the IMM (CV + CTRV) tracker.
Alongside the stack the recorder starts a `robot_state_publisher` and a
`joint_state_publisher` on the waffle URDF, and RViz on the recording profile
[`rviz/recording.rviz`](../rviz/recording.rviz) (a full-frame, dock-free view
framing the corridor).
Two URDF details matter for a clean render: the description is **re-rooted at
`base_link`** (the stock waffle roots at `base_footprint`, whose fixed
`base_footprint -> base_link` joint would fight the plant's `odom -> base_link`
and give `base_link` two TF parents - corrupting the costmap's scan transform into
phantom, uncleared keep-outs that trap the robot), and the `joint_state_publisher`
supplies the continuous wheel joints (without it RViz reddens the RobotModel).
The mesh paths are also corrected so the model loads without "Error loading
geometries".

An `obstacle_markers` node draws each scenario obstacle as a ground-truth cylinder
on `/obstacle_bodies` (a `MarkerArray`, visualization only - never
`/tracked_obstacles`, so it does not feed the controller), using the same analytic
motion law and first-motion clock as the scan simulator. The body therefore tracks
the obstacle exactly, so the video shows where the obstacle really is alongside its
(lagging) costmap footprint.

The b2 stack runs on wall/system time - there is no `/clock` publisher - so RViz,
the `robot_state_publisher`, and the `joint_state_publisher` are started with
`use_sim_time:=false`. This is the opposite of the Gazebo demo launch, and using
sim time here would leave RViz waiting on a clock that never ticks.

The four default scenarios and their grid labels:

| Basename | Cell | Grid label |
| --- | --- | --- |
| `nav2_open` | no obstacle, straight traverse | No obstacle |
| `static_box` | one static box off-centre | Static box |
| `dynamic_line_forward` | obstacle patrolling a line across the path | Dynamic line |
| `dynamic_circle` | obstacle orbiting near mid-path | Dynamic circle |

## Prerequisites

`ffmpeg` is the recorder (x11grab) and the grid combiner (xstack); `xdotool` is
optional (a best-effort window nudge on bare X servers); `xvfb` provides a virtual
X display for the headless / Wayland path below:

```bash
sudo apt-get install -y ffmpeg xdotool xvfb
```

The workspace must already be built and sourced (see `init.sh`).

### Display: X11 vs Wayland

`x11grab` reads the X11 **root window**. Under a Wayland session (the GNOME
default on Ubuntu 24.04) the X server is rootless `Xwayland`: the compositor draws
each window through Wayland, so the grabbable X root stays **black** and every clip
records black regardless of what is on screen. Pick one of:

- **Xorg session (simplest, GPU-rendered):** log in via the GDM gear menu as
  "Ubuntu on Xorg", then run the recorder unchanged (it captures `$DISPLAY`, `:0`).
- **Virtual Xvfb display (headless, scriptable - how the bundled clips were made):**
  start a virtual X server sized to the capture, then point the recorder at it with
  `--display`. RViz falls back to software GL (llvmpipe), which renders correctly:

  ```bash
  Xvfb :99 -screen 0 1920x1080x24 +extension GLX +render -nolisten tcp &
  ros2 run prox_mpc_benchmark record_scenarios.py --display :99 --resolution 1920x1080
  ```

Confirm a display is capturable before a long run - a single frame should not be
near-black (`YAVG` well above ~16):

```bash
ffmpeg -f x11grab -video_size 640x480 -i :99.0+0,0 -frames:v 1 /tmp/probe.png
ffmpeg -i /tmp/probe.png -vf signalstats,metadata=print:key=lavfi.signalstats.YAVG -f null -
```

## Record the per-scenario clips

Record all four scenarios with the defaults (predictive controller, waffle,
20 s clips, 1920x1080 grab of `$DISPLAY`). On Wayland, start Xvfb first and pass
`--display` (see [Display: X11 vs Wayland](#display-x11-vs-wayland)):

```bash
Xvfb :99 -screen 0 1920x1080x24 +extension GLX +render -nolisten tcp &
ros2 run prox_mpc_benchmark record_scenarios.py --display :99
```

On a native Xorg session the no-argument form captures `:0` directly:

```bash
ros2 run prox_mpc_benchmark record_scenarios.py
```

The equivalent fully-explicit invocation:

```bash
ros2 run prox_mpc_benchmark record_scenarios.py \
  --scenarios nav2_open,static_box,dynamic_line_forward,dynamic_circle \
  --controller proxmpc_pred \
  --robot waffle \
  --display :99 \
  --duration 20 \
  --resolution 1920x1080 \
  --offset 0,0 \
  --framerate 30 \
  --warmup 14 \
  --goal-delay 3 \
  --timeout 55
```

For each scenario the recorder waits `--warmup` seconds for lifecycle activation,
starts a fixed-length ffmpeg x11grab recording, sends the scenario goal after
`--goal-delay` seconds, waits for ffmpeg to reach its `--duration`, then tears the
whole process tree down with a process-group `SIGINT` (graceful) escalating to
`SIGKILL`.
The exact ffmpeg command is printed for every clip.
One scenario failing does not abort the rest; the exit code is non-zero if any
scenario failed.

Every child's stdout and stderr is logged under `results/videos/logs/` as
`<scenario>.<role>.log` (roles: `stack`, `robot_state_publisher`, `rviz`, `ffmpeg`,
`goal`, `xdotool`), so RViz load errors are auditable after the fact.

## Combine into a 2x2 grid

Stitch the four clips into one labelled grid video plus an inline GIF:

```bash
ros2 run prox_mpc_benchmark combine_grid.sh
```

The combiner scales each clip to the cell size, draws the human-readable label in
the top-left of each cell, arranges them
`top-left | top-right / bottom-left | bottom-right`, and writes
`prox_mpc_demo_grid.mp4` next to the clips.
It also renders an inline **`.gif`** (default 960 px wide, 10 fps, palette two-pass)
next to the output, so the grid embeds and loops directly in the README instead of
appearing as a click-to-download attachment (`--gif-width` / `--gif-fps` tune it).
Override the defaults with flags:

```bash
ros2 run prox_mpc_benchmark combine_grid.sh \
  --input-dir "$HOME/ros2_ws/src/prox_mpc/prox_mpc_benchmark/results/videos" \
  --cell 960x540 \
  --framerate 30 \
  --output "$HOME/ros2_ws/src/prox_mpc/prox_mpc_benchmark/results/videos/prox_mpc_demo_grid.mp4"
```

The full ffmpeg command is echoed before it runs.

## Multi-controller grids

To compare controllers on the *same* scenario (ProxMPC vs the stock Nav2 peers),
record each controller into its own folder with `--controller` / `--out-dir`, then
combine one scenario across four folders with the explicit `--inputs` / `--labels`
mode (the four paths and four cell labels, top-left -> bottom-right):

```bash
# one folder per controller (the tracker starts only for proxmpc_pred)
for c in proxmpc_pred dwb mppi regulated_pure_pursuit; do
  ros2 run prox_mpc_benchmark record_scenarios.py --controller "$c" \
    --out-dir results/videos/"$c"
done

# a 4-controller grid for one scenario (static_box shown)
ros2 run prox_mpc_benchmark combine_grid.sh \
  --inputs results/videos/proxmpc_pred/static_box.mp4,results/videos/dwb/static_box.mp4,results/videos/mppi/static_box.mp4,results/videos/regulated_pure_pursuit/static_box.mp4 \
  --labels ProxMPC,DWB,MPPI,RPP \
  --output doc/media/static_box_controllers.mp4
```

The stock peers publish their own trajectory on `/local_plan` (shown yellow in the
recording profile) rather than ProxMPC's `/prox_mpc_local_plan` (green).

## Outputs

All artifacts land under `results/videos/` (gitignored):

- `nav2_open.mp4`, `static_box.mp4`, `dynamic_line_forward.mp4`,
  `dynamic_circle.mp4` - the per-scenario clips.
- `prox_mpc_demo_grid.mp4` - the combined 2x2 grid.
- `prox_mpc_demo_grid.gif` - the inline GIF embedded (and looping) in the README.
- `logs/` - per-child capture logs for auditing bringup and RViz.

## Parameters

`record_scenarios.py`:

| Flag | Default | Meaning |
| --- | --- | --- |
| `--scenarios` | `nav2_open,static_box,dynamic_line_forward,dynamic_circle` | scenario basenames to record |
| `--controller` | `proxmpc_pred` | controller preset injected into `FollowPath` |
| `--robot` | `waffle` | robot shown (URDF + model pairing) |
| `--duration` | `20` | clip length in seconds (self-terminating via `-t`) |
| `--resolution` | `1920x1080` | grab size `WxH` |
| `--offset` | `0,0` | grab top-left origin `x,y` -> x11grab input `:0.0+x,y` |
| `--display` | `$DISPLAY` or `:0` | X display to capture and render on |
| `--framerate` | `30` | capture frame rate |
| `--warmup` | `14` | activation wait before recording starts [s] |
| `--goal-delay` | `3` | wait after recording starts before the goal is sent [s] |
| `--timeout` | `55` | goal timeout passed to `goal_sender.py` [s] |
| `--out-dir` | `results/videos` | clip output directory |

`combine_grid.sh`:

| Flag | Default | Meaning |
| --- | --- | --- |
| `-i`, `--input-dir` | `<pkg>/results/videos` | directory holding the clips |
| `-o`, `--output` | `<input-dir>/prox_mpc_demo_grid.mp4` | grid mp4 path |
| `-c`, `--cell` | `960x540` | per-cell size (grid is 2x cell) |
| `-r`, `--framerate` | `30` | output frame rate |
| `-b`, `--basenames` | the four defaults | clip basenames in grid order |

## Notes and gotchas

- Wall time, not sim time.
  The b2 stack has no `/clock`; RViz and the `robot_state_publisher` must use
  `use_sim_time:=false`, which the recorder sets.
- x11grab needs a real display.
  The `--offset` and `--resolution` must fit inside the actual screen geometry;
  the defaults grab a 1920x1080 region anchored at the top-left.
- `xdotool` is optional.
  When installed it best-effort moves and resizes the RViz window into the capture
  region; when absent the recorder simply grabs the full region.
- ffmpeg `drawtext` requires an ffmpeg built with libfreetype (the stock Ubuntu
  `ffmpeg` package qualifies); the label filter fails otherwise.
- Clip length is fixed by `--duration`, so the four clips stay length-synced and
  xstack combines them cleanly.

## License

[Apache-2.0](../../LICENSE) - the full text is in [LICENSE](../../LICENSE) and
attribution in [NOTICE](../../NOTICE).
