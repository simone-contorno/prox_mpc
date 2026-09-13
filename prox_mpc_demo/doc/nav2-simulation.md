# ProxMPC Nav2 + Gazebo Harmonic simulation

The real-behaviour gate for the `prox_mpc_controller` Nav2 plugin: it runs the
plugin inside a live `controller_server`, driving a simulated diff-drive robot in
Gazebo Harmonic under a full Nav2 stack. All scenarios below were run headless and
verified (see "Verified results").

## Table of Contents

- [What is wired](#what-is-wired)
- [Launch arguments](#launch-arguments)
- [RViz robot model and displays](#rviz-robot-model-and-displays)
- [Build](#build)
- [Scenario 0 - plugin loads in controller_server (no Gazebo)](#scenario-0---plugin-loads-in-controller_server-no-gazebo)
- [Scenario 1 - clean run, NavigateToPose SUCCEEDED (open room)](#scenario-1---clean-run-navigatetopose-succeeded-open-room)
- [Scenario 2 - obstacle avoidance (unmapped static + dynamic)](#scenario-2---obstacle-avoidance-unmapped-static--dynamic)
- [Verified results](#verified-results)
- [Controller config notes (what these defaults encode)](#controller-config-notes-what-these-defaults-encode)
- [tb3 pillar-maze world (prox_mpc_world.sdf.xacro)](#tb3-pillar-maze-world-prox_mpc_worldsdfxacro)

## What is wired

A thin wrapper ([launch/nav2_simulation.launch.py](../launch/nav2_simulation.launch.py))
over the canonical Nav2 Jazzy scenario (`nav2_bringup/tb3_simulation_launch.py` +
`nav2_minimal_tb3_sim`), changing only:

- **Controller**: `controller_server -> FollowPath` is repointed from MPPI to
  `prox_mpc_controller::ProxMpcController`
  ([config/nav2_prox_mpc.yaml](../config/nav2_prox_mpc.yaml)), using the **Unicycle**
  model whose body twist `[v, omega]` maps 1:1 onto the diff-drive TurtleBot3
  waffle the scenario spawns. The rest of the stack (planner, costmaps, smoother,
  collision monitor, BT) is the stock, plugin-agnostic reference, so the cmd_vel
  chain `controller_server (cmd_vel_nav) -> velocity_smoother (cmd_vel_smoothed)
  -> collision_monitor (cmd_vel) -> ros_gz bridge -> Gazebo` is preserved.
- **World + map** (defaults): an open 6x6 m room
  ([worlds/prox_mpc_open.sdf.xacro](../worlds/prox_mpc_open.sdf.xacro) +
  [maps/prox_mpc_open.yaml](../maps/prox_mpc_open.yaml)). AMCL is seeded with the
  spawn pose (`set_initial_pose`), so headless runs localize without a manual
  "2D Pose Estimate".
- **Obstacle tracker (opt-in)**: with `predictive:=True` the launch starts
  `prox_mpc_obstacle_tracker` on `/scan`, feeding `/tracked_obstacles` to the
  controller for predictive (dynamic) obstacle avoidance, and switches the params
  to `config/nav2_prox_mpc_predictive.yaml`. The default (`predictive:=False`)
  starts no tracker and uses the verified baseline params - normal navigation
  behaves exactly as the stock plugin-agnostic stack.
- **RViz (opt-in)**: with `use_rviz:=True` the launch opens
  [rviz/nav2_simulation.rviz](../rviz/nav2_simulation.rviz) - the stock Nav2 view
  plus the ProxMPC local-plan / predicted-obstacle displays - and drives the waffle
  RobotModel from a path-corrected copy of the waffle URDF. See
  [RViz robot model and displays](#rviz-robot-model-and-displays).

AMCL self-seeds at the spawn pose `(-2.0, -0.5)`.

## Launch arguments

`nav2_simulation.launch.py` declares the arguments below. It forwards `world`,
`map`, `params_file`, and `headless` to `nav2_bringup/tb3_simulation_launch.py`
and owns the robot-description / RViz wiring itself (see
[RViz robot model and displays](#rviz-robot-model-and-displays)), forcing
`use_robot_state_pub:=False` and `use_rviz:=False` on the included launch so
exactly one publisher owns `/robot_description` and the RViz session.

| Argument | Default | Meaning |
| --- | --- | --- |
| `world` | `worlds/prox_mpc_open.sdf.xacro` | Full path to the Gazebo world (xacro). |
| `map` | `maps/prox_mpc_open.yaml` | Full path to the occupancy map yaml (must match the world). |
| `params_file` | `''` | Full path to the Nav2 params; empty selects the file from `predictive`. Point it at another `FollowPath` plugin to run a different controller. |
| `headless` | `True` | Run Gazebo headless (no GUI / SceneBroadcaster). |
| `use_rviz` | `False` | Start RViz on `rviz/nav2_simulation.rviz` (requires a display). |
| `predictive` | `False` | Enable predictive obstacle avoidance and start the tracker on `/scan`. |

An explicit `params_file:=<path>` overrides the file chosen by `predictive`.

> Rendering note: the Gazebo `Sensors` system (gpu_lidar) needs an OGRE2 render
> context. These scenarios were verified on a host with a GPU + display; on a
> headless host without a GPU/EGL the lidar may fail to start. Scenario 0 (plugin
> load) needs no Gazebo and runs anywhere.

## RViz robot model and displays

Under `use_rviz:=True` this wrapper starts RViz on
[rviz/nav2_simulation.rviz](../rviz/nav2_simulation.rviz) - the stock
`nav2_default_view.rviz` (map, laser scan, global/local costmaps, AMCL particle
cloud, plans, TF, and the Nav2 toolbar) extended with two ProxMPC displays. The
standalone `simulation.launch.py` keeps its own lighter
[rviz/simulation.rviz](../rviz/simulation.rviz).

Robot model. The stock `turtlebot3_waffle.urdf` points its four RViz meshes at
`package://nav2_minimal_tb3_sim/models/*.dae`, but those install one level deeper
under `models/turtlebot3_model/meshes/*.dae`, so an RViz RobotModel pointed at the
waffle `/robot_description` raised "Error loading geometries". This wrapper runs its
own `robot_state_publisher` on a path-corrected copy of the waffle URDF (the four
mesh subpaths fixed at launch time) so the RobotModel loads. That RSP is also
load-bearing for navigation - it is the only source of the
`base_footprint -> base_link -> base_scan` and wheel TF that AMCL and the costmaps
consume (Gazebo's DiffDrive plugin publishes only `odom -> base_footprint`) - so
`tb3_simulation_launch.py`'s own RSP and `nav2_default_view.rviz` are disabled
(`use_robot_state_pub:=False`, `use_rviz:=False`) and this wrapper owns both
`/robot_description` (latched: `KEEP_LAST` depth 1, `reliable`, `transient_local`)
and the RViz session.

Default displays. `nav2_simulation.rviz` inherits the stock `nav2_default_view.rviz`
display set - RobotModel (`/robot_description`), Map (`/map`), LaserScan (`/scan`),
the global and local costmaps, the AMCL particle cloud, the global/local plans, and
TF - and adds two ProxMPC controller-plugin displays:

| Display | Topic | Type | Notes |
| --- | --- | --- | --- |
| ProxMPC Local Plan | `/prox_mpc_local_plan` | `nav_msgs/Path` | controller NMPC horizon; published lazily when a subscriber exists |
| ProxMPC Predicted Obstacles | `/prox_mpc_predicted_obstacles` | `visualization_msgs/MarkerArray` | only under `predictive:=True` with a live subscriber |

`/prox_mpc_local_plan` and `/prox_mpc_predicted_obstacles` are the
`controller_server` plugin's relative publishers; the demo applies no namespace,
so they resolve at the root. The predicted-obstacle markers appear only in
predictive mode (`predictive:=True`) once RViz is subscribed. The Nav2 demo and the
standalone demo use separate profiles (`nav2_simulation.rviz` and `simulation.rviz`)
because only the Nav2 stack publishes the map, costmaps, and scan the full view
shows.

## Build

```bash
colcon build --symlink-install \
  --packages-select prox_mpc_msgs prox_mpc_core prox_mpc_controller prox_mpc_demo
source install/setup.bash
```

The predictive mode (`predictive:=True`) additionally needs
`prox_mpc_obstacle_tracker` built, since the launch then starts the tracker on
`/scan`.

Reproducible commands (see the scenarios below for the full walkthrough):

```bash
# baseline, headless (no Gazebo GUI, no RViz)
ros2 launch prox_mpc_demo nav2_simulation.launch.py

# Gazebo GUI + RViz + predictive path
ros2 launch prox_mpc_demo nav2_simulation.launch.py predictive:=True headless:=False use_rviz:=True

# send a goal into the running demo
ros2 run prox_mpc_benchmark goal_sender.py --points 2.0,-0.5,0.0 --timeout 120
```

## Scenario 0 - plugin loads in controller_server (no Gazebo)

```bash
ros2 plugin list --package prox_mpc_controller   # lists prox_mpc_controller::ProxMpcController
```

The load line appears at controller_server configure/activate (Scenarios 1-2):

```text
[controller_server]: Created controller : FollowPath of type prox_mpc_controller::ProxMpcController
[ProxMpcController]: Configured ProxMpcController 'FollowPath' (model 'prox_mpc_core/Unicycle', Np=20, Nc=20, dt=0.100, K=0).
[ProxMpcController]: Activating ProxMpcController 'FollowPath'.
```

## Scenario 1 - clean run, NavigateToPose SUCCEEDED (open room)

Terminal A - bring up Gazebo Harmonic + Nav2 + ProxMPC, headless (open world + map
are the defaults):

```bash
ros2 launch prox_mpc_demo nav2_simulation.launch.py
```

Terminal B - (optional) the estimate-position mechanism. AMCL already self-seeds;
this is the manual equivalent of RViz "2D Pose Estimate":

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map},
    pose: {pose: {position: {x: -2.0, y: -0.5, z: 0.0}, orientation: {w: 1.0}}}}"
```

Terminal B - watch the controller's commands during navigation:

```bash
ros2 topic echo /cmd_vel_nav        # raw ProxMpcController output (Twist)
```

Terminal C - send the scenario goal (~4 m straight drive) and read the result:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map},
           pose: {position: {x: 2.0, y: -0.5, z: 0.0},
                  orientation: {w: 1.0}}}}" --feedback
```

### Pass criteria / expected output

- Controller load line (Terminal A) as in Scenario 0.
- Non-zero commands while navigating (Terminal B): `linear.x` around 0.2-0.3 m/s.
- Goal result (Terminal C): `Goal finished with status: SUCCEEDED`.
- Stop: the final command is `linear.x: 0.0, angular.z: 0.0` - the controller
  commands a stop at the goal.

## Scenario 2 - obstacle avoidance (unmapped static + dynamic)

Same open world; spawn obstacles the static map does not contain into the running
world, then navigate. Nav2's global costmap (scan obstacle layer) + replanning
route around them; the controller tracks the rerouted collision-free path.

Terminal A - bring up (as Scenario 1):

```bash
ros2 launch prox_mpc_demo nav2_simulation.launch.py
```

Terminal B - spawn an unmapped static box on the path, plus a moving box that
repeatedly crosses it. The moving box is not an SDF `<actor>`: it is a rigid box
driven by the `gz-sim` `VelocityControl` system at a constant body-frame forward
speed and yaw rate, which traces a bounded 0.6 m-radius circle in place (0.3 m/s
over 0.5 rad/s) rather than driving off the map.

```bash
DEMO=$(ros2 pkg prefix prox_mpc_demo)/share/prox_mpc_demo
ros2 run ros_gz_sim create -name path_box \
  -file $DEMO/models/prox_mpc_static_box/model.sdf   -x 0.0 -y -0.5 -z 0.25
ros2 run ros_gz_sim create -name walker \
  -file $DEMO/models/prox_mpc_dynamic_actor/model.sdf -x 0.8 -y -1.6 -z 0.5
```

> **The two obstacles together do not exercise predictive avoidance.** The box
> sits on the path at `(0.0, -0.5)` and the walker circles beyond it, so for
> about 71% of the approach the box lies between the robot and the walker and
> hides it from the lidar. Replaying the tracker's own clustering over recorded
> scans, the walker is detected in 19% of scans with the box present against 55%
> without it; at that hit rate a track rarely survives `confirm_count: 3` to be
> published at all, and across ten runs of this pair no moving track was ever
> published. What the robot avoids here is the box, from the costmap.
>
> To watch the predictive path actually run, spawn the walker on its own -
> omit the `path_box` line above. The walker is then tracked within a few
> seconds and the controller plans against its predicted positions.
>
> Moving the walker is not a fix on its own: shifting it further along the path
> (`-x 1.8`) raises occlusion to 78%, and every spawn that lowers it either
> brings the walker within the tracker's `cluster_gap` of the box or puts its
> orbit on top of the robot's start pose. Separating the two properly is a
> scenario redesign - a smaller orbit, or more room between start, box and
> crossing - not a coordinate tweak.

Terminal C - send the same goal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map},
           pose: {position: {x: 2.0, y: -0.5, z: 0.0},
                  orientation: {w: 1.0}}}}" --feedback
```

**Expected behaviour**: the lidar marks both boxes into the costmaps, the global
planner reroutes around them, the controller steers around (non-trivial `angular.z`)
and reports `SUCCEEDED`.

## Verified results

Run headless on a GPU host (Gazebo Sim 8.11.0 / Harmonic, ROS 2 Jazzy):

| Scenario | Result | Evidence |
| --- | --- | --- |
| 0 - plugin load | PASS | `Created controller : FollowPath of type prox_mpc_controller::ProxMpcController`; clean configure/activate |
| 1 - clean SUCCEEDED | PASS | `Goal finished with status: SUCCEEDED`, 0 recoveries; `cmd_vel.linear.x` 0.22-0.27 during nav; final cmd `0.0/0.0` (stop) |
| 2 - obstacle avoidance | PASS | both obstacles spawned; `SUCCEEDED`, 1 recovery; 23 global replans; 264 cmd cycles with `abs(angular.z) > 0.2` steering around the obstacles |

Verified headless with predictive obstacle avoidance (Gazebo Sim 8.11.0 /
Jazzy), goal `(2.0, -0.5)`:

| Scenario | Result | Evidence |
| --- | --- | --- |
| Baseline normal nav (`predictive:=False`, default) | PASS | controller `K=0`; `Reached the goal!` / `Goal succeeded` (~19 s drive); 0 footprint vetoes, 0 solver failures |
| Predictive (`predictive:=True`, moving box circling the mid-path) - *historical, pre-retune* | PASS | controller `K=2`, tracker active; `Reached the goal!` / `Goal succeeded` (~22 s); 0 footprint vetoes, 0 solver failures, 0 `NoValidControl`, 0 TF errors, no recoveries |

The predictive row records a run made with `max_obstacles: 2`, while
[config/nav2_prox_mpc_predictive.yaml](../config/nav2_prox_mpc_predictive.yaml)
ships `max_obstacles: 4`, `cbf_gamma: 1.0`, and `max_dynamic_obstacles: 2`. The
row therefore does not describe the shipped configuration, and the values in it
are the ones observed at the time rather than a re-measurement.

The baseline run confirms predictive avoidance is a clean enable/disable feature:
with `predictive:=False` normal path tracking + Nav2 replanning behave exactly as
the stock stack. The predictive run uses the wall-rejection guards (`max_cluster_radius` in
the tracker, `max_dynamic_obstacle_radius` in the controller); without them an
extended wall is tracked as a phantom fast-moving obstacle (its visible-segment
centroid drifts at ~robot speed) and the robot drives erratically.

## Controller config notes (what these defaults encode)

The defaults in [config/nav2_prox_mpc.yaml](../config/nav2_prox_mpc.yaml) were tuned
against the live loop:

- `desired_linear_vel: 0.5` - at the velocity-smoother cap. A slower value samples
  the reference too close to the robot (~0.5 m over the horizon), so heading
  tracking dominates and the unicycle rotates-in-place to align instead of
  translating; 0.5 places the reference ~1 m ahead so forward motion is optimal.
- `q_theta: 1.0` - heading tracking keeps the robot tight on the collision-free
  global path (with `q_theta: 0` it over-swings and drifts into obstacles).
- `max_obstacles: 0` (baseline) - the in-loop NMPC obstacle term is OFF, so
  avoidance is delegated entirely to Nav2's planner + costmaps (global replanning
  around marked obstacles), the standard Nav2 architecture. Keep it for
  path-tracking runs that should behave as plain Nav2 navigation.
- `allow_reversing: true` - the solver may plan reverse travel, so the linear
  control bound keeps its negative half and the robot backs up to adjust before
  pursuing the path forward, the way a vehicle manoeuvres. Reverse is capped at
  0.15 m/s by the controller's own guard. Both guards follow the predicted
  trajectory, so they do cover a reversing one, but they see only what the
  costmap holds, and whether a platform sweeps behind itself is a property of its
  sensor rather than of the controller. The waffle carries a 360-degree lidar, so
  the rear is covered here; naming `model_params.v_min` is how a platform states
  the reverse envelope its own sensing supports.

  An earlier release shipped this `false`. Reverse was not the problem: inside
  the goal-checker xy tolerance the reference collapsed to a stub a few
  millimetres ahead of the robot's own projection onto the plan, which the
  projection then carried along, so a small tracking error could be traded down
  as cheaply one way as the other and an open reverse bound gave the solver a
  second way to do it. Pinning the reference to the goal pose in that region, and
  turning on the spot for the last of the heading, removes the degeneracy at
  source; with that in place reverse measures better than forward-only on every
  scenario. Same goals, same configuration otherwise, measured over the commanded
  `cmd_vel_nav`, headless:

  | Scenario | Before the fix (reverse on) | Forward-only | Now (reverse on) |
  | --- | --- | --- | --- |
  | East 4 m then west 4.5 m | 1 change, 41.1 s, 73% reversing | 0 changes, 38.6 s | 2 changes, 27.8 s |
  | Static box + circling walker | 28 changes, 59.8 s | 0 changes, 19.8 s | 0 changes, 12.2 s |
  | North, 90 degree turn | 35 changes, 68.8 s | 0 changes, 21.7 s | 1 change, 9.5 s |

  Nav2 logged 9 `Failed to make progress` events across the three "before"
  runs and 2 across the forward-only runs, where the controller stalled on a
  terminal heading error and a recovery behaviour took over; it logs none now.
  The direction changes that remain are single deliberate manoeuvres - back up,
  then drive forward - rather than an alternation.

  `reverse_from_plan_orientation` stays at its `false` default: this demo plans
  with NavFn, which leaves every plan pose at the identity quaternion, so those
  orientations carry no travel direction to read (see
  [prox_mpc_controller/doc/architecture.md](../../prox_mpc_controller/doc/architecture.md)).
  The standstill and dwell gates on a direction change are inert while it is off.
- `obstacle_yield_band_m: 0.5` (predictive file only) - when where the robot is
  heading would cut into a tracked mover's predicted keep-out, the cruise eases
  so the robot waits for the mover rather than racing it. Without it the robot
  tends to pass in front of a mover heading for its path, because the global
  planner picks the side from where the obstacle is now and knows nothing of
  where it is going. Measured with this demo's reversing on, over ten
  `dynamic_circle` runs: passes behind 6/10 against 1/10, with the best closest
  approach and no collisions; with two movers it is no worse than off (3/30
  collisions either way). It is absent from the non-predictive file, which runs
  no tracker and so has no predictions to act on.
- `docking_server` block is kept from the stock params because the navigation
  lifecycle manager brings it up and aborts the whole bringup if its `dock_plugins`
  is unset.

### Predictive (dynamic) obstacle avoidance - `predictive:=True`

The feature is opt-in and lives in
[config/nav2_prox_mpc_predictive.yaml](../config/nav2_prox_mpc_predictive.yaml),
selected by the launch switch which also starts the tracker:

```bash
ros2 launch prox_mpc_demo nav2_simulation.launch.py predictive:=True
```

What it changes from the baseline (the rest of the stack is identical):

- `max_obstacles: 4`, `cbf_gamma: 1.0` - the in-loop NMPC obstacle term runs
  alongside Nav2, with four slots so the box and the adjacent wall cells are all
  captured. `cbf_gamma: 1.0` is the pointwise keep-out, which is what this config
  targets: the open-world single-obstacle and dynamic cells. A dense obstacle
  field is a case the discrete-time CBF coupling (`cbf_gamma < 1`) is meant for
  - decaying the safety margin gradually rather than binding at every node -
  but its effect at the shipped slack weight has not yet been benchmarked (see
  [control-law.md](../../prox_mpc_controller/doc/control-law.md#discrete-time-control-barrier-coupling)),
  so treat a lower gamma as a tunable option to measure, not a settled
  recommendation for this scenario.
- `predict_obstacles: true`, `max_dynamic_obstacles: 2` - a confirmed *moving*
  track is propagated over the horizon along its tracker-sampled predicted
  trajectory (a constant-velocity ray when no samples are provided) and bound to a
  dynamic slot; the static box and walls keep coming from the costmap (hybrid). With no
  moving obstacle the predictive fill degrades to the costmap-only result. The
  second dynamic slot exists because a single slot is not enough after an
  association break (an obstacle reversing direction): the old track coasts as a
  phantom while the real obstacle is re-acquired as a new track, and with one slot
  the phantom can outrank the real track and the solver goes blind to it. The
  predicted trajectories publish on `prox_mpc_predicted_obstacles`
  (`visualization_msgs/MarkerArray`) for RViz.
- `max_dynamic_obstacle_radius: 0.5` (controller) and `max_cluster_radius: 0.6`
  (tracker) - the wall-rejection guards. An extended wall's cluster centroid drifts
  at ~robot speed as the robot moves, so without these it is tracked as a phantom
  fast-moving obstacle that inflates the keep-out and erases real costmap cells,
  and the robot drives erratically. The guards keep walls out of the predictive
  path (they remain the costmap's job).

The tracker is a self-activating lifecycle node started by the launch only when
`predictive:=True`, with `use_sim_time: true`; its parameters live in
[prox_mpc_obstacle_tracker/config/obstacle_tracker.yaml](../../prox_mpc_obstacle_tracker/config/obstacle_tracker.yaml).
Watch its output with `ros2 topic echo /tracked_obstacles`.

## tb3 pillar-maze world (prox_mpc_world.sdf.xacro)

[worlds/prox_mpc_world.sdf.xacro](../worlds/prox_mpc_world.sdf.xacro) (the tb3 sandbox
plus baked static boxes and a dynamic actor) is provided for completeness:

```bash
ros2 launch prox_mpc_demo nav2_simulation.launch.py \
  world:=$(ros2 pkg prefix prox_mpc_demo)/share/prox_mpc_demo/worlds/prox_mpc_world.sdf.xacro \
  map:=$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/maps/tb3_sandbox.yaml
```

Note: threading the dense `turtlebot3_world` pillar cluster (~0.5 m gaps) reliably
needs the in-the-loop NMPC obstacle term tuned (or a controller tuned specifically
for tight maze following); the open-room scenarios are the verified gate.
