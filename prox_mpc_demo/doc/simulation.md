# Standalone simulation and benchmark

The `prox_mpc_simulation` node is a self-contained, closed-loop driver for the
[prox_mpc_core](../../prox_mpc_core) engine - no external simulator.
Each control step it solves the MPC, publishes the first control and the predicted
trajectory, advances the simulated pose to the model's own predicted next state,
and broadcasts `map -> base_link` so RViz tracks the robot.
It also measures and logs the solve time, and (when `publish_diagnostics` is true)
publishes one `SolverDiagnostics` per cycle, so the engine's compute cost, control
rate, and feasibility are observable without Nav2.

## Table of Contents

- [Run](#run)
- [Parameters](#parameters)
- [Interfaces](#interfaces)
- [Measuring controller performance](#measuring-controller-performance)

## Run

```bash
colcon build --symlink-install --packages-select prox_mpc_msgs prox_mpc_core prox_mpc_demo
source install/setup.bash
ros2 launch prox_mpc_demo simulation.launch.py
```

[../config/simulation.yaml](../config/simulation.yaml) is the single source of
truth for the parameters.
Switch model or tune the run by editing it, or override a parameter on the command
line:

```bash
ros2 run prox_mpc_demo prox_mpc_simulation --ros-args -p model:=unicycle
```

Add `rviz:=true` to the launch to also start RViz, `robot_state_publisher`, and
`joint_state_publisher` and visualize the chosen robot.

## Parameters

Every parameter below is declared in the node with the default shown, so the node
runs without a YAML.
The one exception is `log_level`, which the node never declares: the launch file
reads it out of the YAML and turns it into a `--ros-args --log-level` argument, so
it has no effect when the executable is run directly without the launch file.
The bundled [../config/simulation.yaml](../config/simulation.yaml) sets the
tracking-demo defaults.

| Parameter | Type | Default | Unit | Meaning |
| --- | --- | --- | --- | --- |
| `model` | string | `bicycle` | - | `bicycle` (4-state) or `unicycle` (3-state). |
| `np` | int | 20 | nodes | Prediction horizon (must be >= 1). |
| `nc` | int | 20 | nodes | Control horizon (must be >= 1). |
| `dt` | double | 0.1 | s | Step size, also the control period (must be > 0). |
| `q_pos` | double | 10.0 | - | Position tracking weight. |
| `q_theta` | double | 1.0 | - | Heading (and remaining state) tracking weight. |
| `s_factor` | double | 2.0 | - | Terminal-weight factor `S = s_factor * Q`. |
| `r_weight` | double | 0.1 | - | Control-effort weight. |
| `w_weight` | double | 100.0 | - | Obstacle-slack penalty. |
| `v_ref` | double | 1.0 | m/s | Reference forward speed. |
| `goal_x`, `goal_y`, `goal_theta` | double | 5.0, 0.0, 0.0 | m, m, rad | Single goal pose (used when no waypoint set is given). |
| `goal_tol` | double | 0.25 | m | Waypoint-arrival radius; also latches the final-goal stop. |
| `obstacle_enable` | bool | false | - | Enable the single legacy fixed obstacle. |
| `max_obstacles` | int | 1 | slots | Obstacle-slot capacity `K` per node when avoidance is on (must be >= 0). |
| `d_safe` | double | 1.0 | m | Required clearance for the legacy obstacle (must be >= 0). |
| `obs_x`, `obs_y` | double | 2.5, 0.6 | m | Legacy obstacle position. |
| `report_period` | int | 50 | steps | Log solve-time stats every N steps (0 disables). |
| `publish_diagnostics` | bool | true | - | Publish one `SolverDiagnostics` per cycle. |
| `log_level` | string | `info` | - | Node logger verbosity (`debug`...`fatal`). Not a declared node parameter: the launch file reads it from the YAML and passes it as `--ros-args --log-level`. |

<details>
<summary>Waypoint set and time-varying obstacles (parallel-array parameters)</summary>

For scenario-style runs the node also accepts an ordered waypoint set and a
time-varying obstacle list, each as parallel arrays.
These default to empty, in which case the node uses the single `goal_*` and the
legacy `obs_*` obstacle above.

| Parameter | Type | Default | Unit | Meaning |
| --- | --- | --- | --- | --- |
| `goals_x`, `goals_y`, `goals_theta` | double[] | `[]` | m, m, rad | Ordered waypoints; the active goal advances once the pose is within `goal_tol`. The three arrays must have equal length. |
| `start_x`, `start_y`, `start_theta` | double | 0.0 | m, m, rad | Initial simulated pose. `start_theta` is declared for any model carrying at least three states, which covers both the 3-state unicycle and the 4-state bicycle. |
| `obs_motion` | string[] | `[]` | - | Per-obstacle motion: `static`, `circle`, or `line`. A non-empty list supersedes the legacy obstacle and enables avoidance. |
| `obs_cx`, `obs_cy` | double[] | `[]` | m | Static position, circle centre, or line "from" point. |
| `obs_ex`, `obs_ey` | double[] | `[]` | m | Line "to" point. |
| `obs_radius` | double[] | `[]` (1.0) | m | Circle orbit radius. |
| `obs_speed` | double[] | `[]` (0.0) | m/s | Signed speed; the sign selects direction (circle) or the first leg (line). |
| `obs_clearance` | double[] | `[]` (`d_safe`) | m | Per-obstacle keep-out radius. |

Shorter obstacle arrays are padded per element with the defaults shown in
parentheses.

</details>

## Interfaces

| Topic | Type | QoS | Direction | Description |
| --- | --- | --- | --- | --- |
| `/robot/cmd_vel` | `geometry_msgs/msg/Twist` | Reliable, depth 1 | Published | First control mapped to a body twist each cycle. |
| `/prox_mpc/path` | `nav_msgs/msg/Path` | Reliable, depth 1 | Published | Predicted optimal trajectory. |
| `/prox_mpc/diagnostics` | `prox_mpc_msgs/msg/SolverDiagnostics` | Reliable, depth 10 | Published | Per-cycle solver telemetry, only when `publish_diagnostics` is true. |
| `map -> base_link` | TF | - | Broadcast | Simulated planar pose, for RViz. |

On a non-converged or non-finite solve the node holds the pose rather than folding
a bad iterate into the state, and ramps the command down instead of stopping dead:
each failed cycle it publishes one deceleration step from the command it last
published toward zero, keeping the sign and clamping at zero.
This is the failure policy [prox_mpc_core](../../prox_mpc_core/doc/nmpc.md)
recommends and the one the Nav2 plugin applies.
The per-axis deceleration limits come from the model's own control-rate (`du`)
bounds, so the ramp respects the same acceleration limits the MPC does; a model
declaring no such bound falls back to 0.5 (m/s^2, rad/s^2).
Because this node is its own plant there is no separately measured velocity, so
the ramp starts from the last published command - which is exactly what the
simulated robot is executing.
The retained command is cleared once the final goal is latched, so a later ramp
never brakes off a stale value.
The node still publishes diagnostics for a failed cycle so the feasibility signal
keeps flowing.
Unlike the Nav2 plugin the demo does not count failures or escalate: it has no
recovery behaviour to escalate to.
Once the final waypoint is reached the node latches a stop (zero command, parked
pose) so the reported goal error is the stopping accuracy.

## Measuring controller performance

The node logs the **min / average / max** solve time in milliseconds with the SQP
and QP iteration counts every `report_period` steps:

```text
solve over 50 steps [ms]  min=0.375  avg=0.542  max=1.677  (sqp_iter=1, qp_iter=9)  ~10.0 Hz budget
```

The `~Hz budget` is `1/dt`.
Confirm the actual published rate with:

```bash
ros2 topic hz /robot/cmd_vel
```

The per-cycle `SolverDiagnostics` stream is what the
[benchmark harness](../../prox_mpc_benchmark/README.md) taps for the real-time and
feasibility metrics; inspect it directly with:

```bash
ros2 topic echo /prox_mpc/diagnostics
```
