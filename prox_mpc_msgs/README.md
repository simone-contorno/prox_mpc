# prox_mpc_msgs

The message contract between the ProxMPC obstacle tracker and the Nav2 controller
plugin.

This is an interface-only package (`rosidl` messages, no node, no library code).
It defines the two messages that carry tracked dynamic obstacles from
[prox_mpc_obstacle_tracker](../prox_mpc_obstacle_tracker) to
[prox_mpc_controller](../prox_mpc_controller) for predictive obstacle avoidance.

## Messages

| Message | Purpose |
| --- | --- |
| `prox_mpc_msgs/msg/Obstacle` | One tracked dynamic obstacle: id, planar position, velocity, enclosing radius, and covariances. |
| `prox_mpc_msgs/msg/ObstacleArray` | A set of tracked obstacles published once per processed scan, with a header carrying the scan stamp and tracking frame. |

### `Obstacle`

| Field | Type | Unit | Meaning |
| --- | --- | --- | --- |
| `id` | `uint32` | — | Stable track identifier, reused across scans for the same physical object. |
| `position` | `geometry_msgs/Point` | m | Planar centroid; `z` unused (0) by the 2D tracker. |
| `velocity` | `geometry_msgs/Vector3` | m/s | Estimated velocity; `z` unused (0). |
| `radius` | `float64` | m | Enclosing radius of the detected cluster. |
| `position_covariance` | `float64[4]` | m² | 2×2 position covariance, row-major `[xx, xy, yx, yy]`. |
| `velocity_covariance` | `float64[4]` | m²/s² | 2×2 velocity covariance, row-major `[xx, xy, yx, yy]`. |

### `ObstacleArray`

| Field | Type | Meaning |
| --- | --- | --- |
| `header` | `std_msgs/Header` | `stamp` is the source scan time; `frame_id` is the tracking frame (a non-rotating frame, for example `odom`). |
| `obstacles` | `Obstacle[]` | The confirmed tracks for this scan. |

## Interface contract

The producer ([prox_mpc_obstacle_tracker](../prox_mpc_obstacle_tracker)) publishes
an `ObstacleArray` per processed scan on `tracked_obstacles`
(`rclcpp::QoS(KeepLast(5))`, reliable), with positions and velocities expressed in
a fixed, non-rotating tracking frame and the header stamp set to the scan time.

The consumer ([prox_mpc_controller](../prox_mpc_controller)) subscribes when
`predict_obstacles` is enabled (reliable, depth 5).
It uses `header.stamp` to age the constant-velocity prediction, `header.frame_id`
to transform the obstacles into the costmap global frame, the `radius` to size the
keep-out clearance, and the velocity-covariance trace to optionally grow the
prediction-uncertainty clearance.
See the controller's [control-law.md](../prox_mpc_controller/docs/control-law.md)
for how the fields drive predictive avoidance.

## Build

```bash
colcon build --symlink-install --packages-select prox_mpc_msgs
source install/setup.bash
ros2 interface show prox_mpc_msgs/msg/ObstacleArray
```

## License

[Apache-2.0](../LICENSE).
