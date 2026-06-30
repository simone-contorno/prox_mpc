# Standalone simulation and benchmark

The `prox_mpc_simulation` node is a self-contained, closed-loop driver for the
[prox_mpc_core](../../prox_mpc_core) engine — no external simulator.
Each control step it solves the MPC, publishes the first control and the predicted
trajectory, advances the simulated pose to the model's own predicted next state,
and broadcasts `map → base_link` so RViz tracks the robot.
It also measures and logs the solve time, so the engine's compute cost and control
rate are observable without Nav2.

## Run

```bash
colcon build --symlink-install --packages-select prox_mpc_core prox_mpc_demo
source install/setup.bash
ros2 launch prox_mpc_demo simulation.launch.py
```

[../config/simulation.yaml](../config/simulation.yaml) is the single source of
truth for the parameters.
Switch model or tune the run by editing it, or override a parameter on the command
line:

```bash
ros2 run prox_mpc_demo prox_mpc_simulation --ros-args -p model:=r2d2
```

Add `rviz:=true` to the launch to also start RViz, `robot_state_publisher`, and
`joint_state_publisher` and visualize the chosen robot.

## Parameters

| Parameter | Type | Default | Unit | Meaning |
| --- | --- | --- | --- | --- |
| `model` | string | `bike` | — | `bike` (bicycle, 4 states) or `r2d2` (unicycle, 3 states). |
| `np`, `nc` | int | 20 | nodes | Prediction / control horizon. |
| `dt` | double | 0.1 | s | Step size (also the control period). |
| `q_pos` | double | 10.0 | — | Position tracking weight. |
| `q_theta` | double | 1.0 | — | Heading (and remaining state) tracking weight. |
| `s_factor` | double | 2.0 | — | Terminal-weight factor `S = s_factor * Q`. |
| `r_weight` | double | 0.1 | — | Control-effort weight. |
| `w_weight` | double | 100.0 | — | Obstacle-slack penalty. |
| `v_ref` | double | 1.0 | m/s | Reference speed. |
| `goal_x`, `goal_y`, `goal_theta` | double | 5.0, 0.0, 0.0 | m, m, rad | Goal pose. |
| `obstacle_enable` | bool | false | — | Enable the single fixed obstacle. |
| `max_obstacles` | int | 1 | slots | Obstacle-slot capacity `K` per node when avoidance is on. |
| `d_safe` | double | 1.0 | m | Required clearance for the obstacle. |
| `obs_x`, `obs_y` | double | 2.5, 0.6 | m | Obstacle position. |
| `report_period` | int | 50 | steps | Log solve-time stats every N steps (0 disables). |

## Interfaces

| Topic | Type | Direction | Description |
| --- | --- | --- | --- |
| `/robot/cmd_vel` | `geometry_msgs/msg/Twist` | Published | First control mapped to a body twist each cycle. |
| `/prox_mpc/path` | `nav_msgs/msg/Path` | Published | Predicted optimal trajectory. |
| `map → base_link` | TF | Broadcast | Simulated planar pose, for RViz. |

The node holds the pose (and publishes a zero command) on a non-converged or
non-finite solve rather than folding a bad iterate into the state.

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
