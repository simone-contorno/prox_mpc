# prox_mpc_demo

Runnable demonstrations of the ProxMPC stack, from a no-simulator benchmark of the
[prox_mpc_core](../prox_mpc_core) engine to a full Nav2 + Gazebo bring-up of the
[prox_mpc_controller](../prox_mpc_controller) plugin.

## Two ways to run

- **Standalone simulation and benchmark** — a self-contained, closed-loop driver
  for the engine with no external simulator: it drives a bundled kinematic model
  toward a goal, publishes the command and predicted path, and logs the solve
  time. Use it to run, visualize, and benchmark the controller without Nav2.
  See [docs/simulation.md](docs/simulation.md).

  ```bash
  colcon build --symlink-install --packages-select prox_mpc_core prox_mpc_demo
  source install/setup.bash
  ros2 launch prox_mpc_demo simulation.launch.py
  ```

- **Nav2 + Gazebo Harmonic simulation** — the real-behaviour gate for the
  controller plugin: it runs the plugin inside a live `controller_server` driving
  a TurtleBot3 waffle under a full Nav2 stack, with an opt-in predictive
  obstacle-avoidance mode. See [docs/nav2-simulation.md](docs/nav2-simulation.md).

  ```bash
  colcon build --symlink-install \
    --packages-select prox_mpc_core prox_mpc_controller prox_mpc_demo
  source install/setup.bash
  ros2 launch prox_mpc_demo nav2_simulation.launch.py            # baseline
  ros2 launch prox_mpc_demo nav2_simulation.launch.py predictive:=True
  ```

## What is in the package

| Path | Contents |
| --- | --- |
| `src/simulation_node.cpp` | The standalone closed-loop simulation node (`prox_mpc_simulation`). |
| `launch/simulation.launch.py` | Standalone simulation (optional RViz). |
| `launch/nav2_simulation.launch.py` | Nav2 + Gazebo Harmonic bring-up (baseline / predictive). |
| `config/` | `simulation.yaml`, and the Nav2 params (`nav2_prox_mpc.yaml`, `nav2_prox_mpc_predictive.yaml`). |
| `worlds/`, `maps/`, `models/` | Gazebo worlds, occupancy maps, and the static-box / dynamic-actor models. |
| `urdf/`, `rviz/` | Robot description and RViz configuration. |

The Nav2 bring-up depends on Nav2, `ros_gz`, and the canonical
`nav2_minimal_tb3_sim` scenario; the standalone simulation needs only the core.
Third-party assets are attributed in
[THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md).

## License

[Apache-2.0](../LICENSE) for the package code; see
[THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md) for bundled third-party assets.
