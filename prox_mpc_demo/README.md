# prox_mpc_demo

A self-contained, closed-loop simulation and benchmark for
[prox_mpc_core](../prox_mpc_core) — no external simulator required.

It drives one of the bundled kinematic models toward a goal, publishes the first
control and the predicted path, and advances the simulated state to the model's
own prediction each cycle.
Use it to run, visualize, and benchmark the controller without Nav2.

## Build and run

```bash
colcon build --symlink-install --packages-select prox_mpc_core prox_mpc_demo
source install/setup.bash
ros2 launch prox_mpc_demo simulation.launch.py
```

Switch model and tune the run by editing
[config/simulation.yaml](config/simulation.yaml) (the single source of truth for
parameters), or override a parameter on the command line:

```bash
ros2 run prox_mpc_demo prox_mpc_simulation --ros-args -p model:=r2d2
```

| Parameter | Default | Meaning |
| --- | --- | --- |
| `model` | `bike` | `bike` (4 states) or `r2d2` (unicycle, 3 states) |
| `np`, `nc` | `20` | prediction / control horizon nodes |
| `dt` | `0.1` | step size [s] (also the control period) |
| `q_pos`, `q_theta`, `s_factor`, `r_weight`, `w_weight` | — | cost weights |
| `v_ref`, `goal_x`, `goal_y`, `goal_theta` | — | reference speed and goal pose |
| `obstacle_enable`, `obs_x`, `obs_y` | `false` | optional obstacle |
| `report_period` | `50` | log solve-time stats every N control steps |
| `log_level` | `info` | this node's logger level only |

Published topics:

- `/robot/cmd_vel` (`geometry_msgs/Twist`) — first control input each cycle.
- `/prox_mpc/path` (`nav_msgs/Path`) — the predicted optimal trajectory.

Add `rviz:=true` to also launch RViz, `robot_state_publisher`, and
`joint_state_publisher` and visualize the chosen robot.

## Measure controller performance

The node logs the **min / average / max** solve time in milliseconds with the
SQP and QP iteration counts every `report_period` steps:

```text
solve over 50 steps [ms]  min=0.375  avg=0.542  max=1.677  (sqp_iter=1, qp_iter=9)  ~10.0 Hz budget
```

The `~Hz budget` is `1/dt`. Confirm the actual published rate with:

```bash
ros2 topic hz /robot/cmd_vel
```

## Status

This package carries the simulation as it stood when the project was split out of
the legacy repository. The launch file references a trajectory-CSV / PNG plotting
hook (`scripts/plot_results.py`) whose node-side CSV writing and RViz marker
publishing are not yet wired in the node; the plot step no-ops gracefully when no
CSV is produced. Completing that tooling is tracked as follow-up work.
