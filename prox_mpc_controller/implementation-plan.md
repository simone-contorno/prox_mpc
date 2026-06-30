# Implementation plan: complete the ProxMpcController Nav2 plugin

## Context

The `prox_mpc_controller` package is already scaffolded as a Nav2 controller plugin, not a standalone node that still needs converting.
The class `prox_mpc_controller::ProxMpcController` already derives from `nav2_core::Controller`, overrides all seven pure-virtual methods, exports itself through `PLUGINLIB_EXPORT_CLASS`, ships a valid plugin-description XML, and constructs a `prox_mpc::MPC` in `configure()`.
What is missing is the control law: `computeVelocityCommands()` is a stub that logs a throttled warning and returns a zero `TwistStamped`, and `configure()` does not yet read parameters, load a model, or size the solver.

The intended design is already written down in [docs/architecture.md](docs/architecture.md) and [docs/control-law.md](docs/control-law.md).
This plan turns that design into a concrete, verifiable implementation task list, grounded in the actual code on disk and the actual Jazzy `nav2_core` headers, and flags every detail that must be confirmed during implementation rather than assumed now.

This plan was produced by reading files only.
All interface signatures below were read from the installed Jazzy headers, not from memory.

## Verified current state

The scaffolding that already exists and is correct, so it is reused rather than recreated:

- The class derives from `nav2_core::Controller` and overrides `configure`, `cleanup`, `activate`, `deactivate`, `setPlan`, `computeVelocityCommands`, and `setSpeedLimit` ([include/prox_mpc_controller/prox_mpc_controller.hpp:33-83](include/prox_mpc_controller/prox_mpc_controller.hpp#L33-L83)).
- The method signatures exactly match the Jazzy base class in `/opt/ros/jazzy/include/nav2_core/controller.hpp`, including `configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &, std::string, std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>)` and `computeVelocityCommands(...) -> geometry_msgs::msg::TwistStamped`.
- `cancel()` and `reset()` are not overridden; the base class provides usable defaults (`cancel()` returns `true`, `reset()` is empty), so overriding them is optional.
- The plugin export macro is present at [src/prox_mpc_controller.cpp:93](src/prox_mpc_controller.cpp#L93).
- The plugin-description XML is present and correct ([prox_mpc_controller_plugin.xml](prox_mpc_controller_plugin.xml)).
- The CMake registration `pluginlib_export_plugin_description_file(nav2_core prox_mpc_controller_plugin.xml)` is present ([CMakeLists.txt:62](CMakeLists.txt#L62)), and `package.xml` already declares every needed dependency.
- The configuration file exists and is the single source of truth ([config/prox_mpc_controller.yaml](config/prox_mpc_controller.yaml)).
- `prox_mpc_core` already exports `prox_mpc::Bicycle` and `prox_mpc::Unicycle` as `prox_mpc::Model` pluginlib plugins under the names `prox_mpc_core/Bicycle` and `prox_mpc_core/Unicycle` ([../prox_mpc_core/prox_mpc_core_plugins.xml](../prox_mpc_core/prox_mpc_core_plugins.xml)), so model selection by name is already possible.

The only stubs are the control law in `computeVelocityCommands()` and the unfinished `configure()` / `setSpeedLimit()` / `setPlan()` bodies.
The work is therefore "complete the skeleton," and most of the build-system and pluginlib items below are already done and only need to be kept consistent.

## 1. Wrapping the NMPC core behind `nav2_core::Controller`

The split follows the responsibility split in [docs/architecture.md](docs/architecture.md): the engine stays frozen, the plugin owns the ROS integration.

Reused as-is, no edits to `prox_mpc_core`:

- `prox_mpc::MPC` — the SQP driver and ProxQP wiring. The plugin only calls its public entry points, confirmed in [../prox_mpc_core/include/prox_mpc/mpc.hpp](../prox_mpc_core/include/prox_mpc/mpc.hpp): `init(model)`, `configProxQP()`, `setNp/setNc/setdt/setT`, `setQ/setR/setS/setW`, `setMaxIntIterQP/setMaxExtIterQP/setMaxIterSQP`, `setGuess/setQPtype`, `setMaxObs`, `setPose`, `setGoalX/setGoalU`, `setObs`, `solve() -> std::tuple<MatrixXd, MatrixXd>`, the `qp_info` member, and the `kObsFarSentinel` constant.
- `prox_mpc::Model` and the bundled `Bicycle`/`Unicycle` plugins — used through `configure(params)`, `toTwist(u)`, `updateIneq(var, idx, low, upp)`, `getIneq(var)`, `getN()`, `getM()` ([../prox_mpc_core/include/prox_mpc/model.hpp](../prox_mpc_core/include/prox_mpc/model.hpp)).
- ProxQP and the obstacle half-plane math — untouched; the solver logic is unchanged.

Adapted or newly added, controller-side only:

- Parameter declaration and reading in `configure()`.
- Model loading via `pluginlib::ClassLoader<prox_mpc::Model>`.
- MPC sizing and cost-matrix assembly from scalar weights.
- Reference construction (`goal_x`, `goal_u`) from the global plan.
- Local-costmap reduction to obstacle triples.
- Model-state tracking (the bicycle steering angle the Nav2 pose does not carry).
- Control-to-`Twist` mapping (delegated to `model->toTwist`).
- Solver-failure deceleration fallback and escalation.
- Exact footprint safety check.

No file in `prox_mpc_core` is modified by this conversion.

## 2. Interface methods

| Method | Status now | Work to do |
| --- | --- | --- |
| `configure(parent, name, tf, costmap_ros)` | caches handles, builds empty `MPC` | declare and read all parameters under the plugin namespace; load the model plugin and `model->configure(model_params)`; `mpc_->init(model)`; set `Np/Nc/dt` (and `T`); assemble and set `Q/R/S/W` from the scalar weights using `model->getN()/getM()`; set solver limits, `setGuess`, `setQPtype`; `setMaxObs(K)`; `configProxQP()`; create optional debug publishers |
| `activate()` | logs only | activate debug publishers; reset the failure counter and warm-start / state-tracking caches |
| `deactivate()` | logs only | deactivate debug publishers; stop processing |
| `cleanup()` | resets `mpc_`, `costmap_ros_`, `tf_` | also reset the model handle, the class loader, and publishers |
| `setPlan(path)` | stores `global_plan_` | keep storing; reset the plan-projection index / pruning state for the new plan |
| `computeVelocityCommands(pose, velocity, goal_checker)` | returns zero `TwistStamped` | full control law (Section 6 and the cycle in [docs/architecture.md](docs/architecture.md): build reference, reduce costmap, set pose/goals/obstacles, `solve()`, check `qp_info.status`, map first control or brake, footprint-check) |
| `setSpeedLimit(limit, percentage)` | caches the value | convert percentage to absolute (fraction of the model maximum) and apply with `model->updateIneq("u", 0, -v_lim, v_lim)` so the next solve respects it without rebuilding the problem |

Optional methods — recommended to override both for production-grade behaviour:

- `cancel()` — override for a graceful stop. The base default returns `true`, which lets the server stop control immediately; for a bicycle / car-like platform an instantaneous stop is dynamically infeasible. The Jazzy contract is explicit in the header: "If false is returned, `computeVelocityCommands` will be called until cancel returns true." So set a `cancelling_` flag on the first `cancel()` call and return `false` while the robot is still moving; meanwhile `computeVelocityCommands` detects `cancelling_` and emits the same deceleration ramp used for the solver-failure fallback (Section 7); once the commanded speed is within an epsilon of zero, `cancel()` returns `true`. This decelerates within the robot's limits instead of jumping to zero and reuses the existing brake routine.
- `reset()` — override to clear runtime state between tasks (not owned resources, which belong to `cleanup()`): the warm-start, the consecutive-failure counter, the tracked bicycle steering integrator, the last-command cache used by the brake ramp, the `cancelling_` flag, and the plan-projection index. This stops stale state from one navigation task leaking into the next; the MPC, model, costmap, and TF handles stay intact.

## 3. pluginlib export

Everything in this section already exists and is correct; the task is to keep it consistent, not to author it.

- Plugin-description XML — present at [prox_mpc_controller_plugin.xml](prox_mpc_controller_plugin.xml): `<library path="prox_mpc_controller">` with `<class type="prox_mpc_controller::ProxMpcController" base_class_type="nav2_core::Controller">`. This matches the verified Jazzy format. No change needed.
- Export macro — present: `PLUGINLIB_EXPORT_CLASS(prox_mpc_controller::ProxMpcController, nav2_core::Controller)` at the end of [src/prox_mpc_controller.cpp:93](src/prox_mpc_controller.cpp#L93). No change needed.
- CMake — present: the `SHARED` library target, `ament_target_dependencies(...)`, and `pluginlib_export_plugin_description_file(nav2_core prox_mpc_controller_plugin.xml)` at [CMakeLists.txt:62](CMakeLists.txt#L62), plus install and export rules. The registration uses the modern CMake macro, so a `<nav2_core plugin="..."/>` tag in `package.xml` `<export>` is intentionally not required and must not be added (it would double-register).
- `package.xml` — already lists every dependency the implementation needs (`nav2_core`, `nav2_costmap_2d`, `pluginlib`, `prox_mpc_core`, `geometry_msgs`, `nav_msgs`, `rclcpp`, `rclcpp_lifecycle`, `tf2`, `tf2_ros`, `Eigen3`, `proxsuite`). No new runtime dependency is anticipated; if the footprint check uses a `nav2_costmap_2d` collision checker it is already covered by the existing `nav2_costmap_2d` dependency.

The only CMake change that may be needed: if helper logic (reference builder, costmap reduction, fallback) is split into additional `.cpp` translation units, add those sources to the existing `add_library(${PROJECT_NAME} SHARED ...)` list.
Keeping it all in `src/prox_mpc_controller.cpp` needs no CMake change at all.

## 4. controller_server integration

Parameter namespace.
Nav2 namespaces a controller plugin's parameters under its instance name.
With `controller_plugins: ["FollowPath"]` the plugin reads each key as `FollowPath.<key>`, which in code is `node->declare_parameter(plugin_name_ + "." + "<key>", default)` then `node->get_parameter(...)`.
The shipped config already nests the keys under `controller_server -> ros__parameters -> FollowPath` ([config/prox_mpc_controller.yaml](config/prox_mpc_controller.yaml)), matching this convention.

Exposed parameters (from the config; declare with these defaults).
Per the workspace typing rules, numeric parameters are declared as `double`, counts as `int`, and flags as `bool`; the YAML float literals (for example `10.0`) map to `double`, not `float`.

| Name | Type | Default | Meaning |
| --- | --- | --- | --- |
| `model_plugin` | string | `prox_mpc_core/Bicycle` | model loaded by name via pluginlib |
| `model_params.L` | double | 1.6 | wheelbase [m], forwarded to `Model::configure()` |
| `np` | int | 20 | prediction horizon nodes |
| `nc` | int | 20 | control horizon nodes |
| `dt` | double | 0.1 | step [s] |
| `desired_linear_vel` | double | 1.0 | cruise speed [m/s] the reference is sampled at; must not exceed the model `v_max` (bundled models default to 3.0) |
| `q_pos` | double | 10.0 | position weight in `Q` |
| `q_theta` | double | 1.0 | heading weight in `Q` |
| `s_factor` | double | 2.0 | terminal weight, `S = s_factor * Q` |
| `r_weight` | double | 0.1 | control weight `R` |
| `w_weight` | double | 100.0 | slack weight `W` |
| `max_int_iter_qp` | int | 1500 | ProxQP internal iterations |
| `max_ext_iter_qp` | int | 10000 | ProxQP external iterations |
| `max_iter_sqp` | int | 100 | SQP iterations per solve |
| `qp_type` | bool | false | sparse (false) / dense (true) |
| `guess` | bool | true | warm start |
| `max_solver_failures` | int | 3 | consecutive failures before recovery |
| `max_obstacles` | int | 1 | per-node obstacle slot capacity K (0 disables) |
| `safety_margin` | double | 0.1 | extra clearance [m] folded into `d_safe` |
| `robot_radius` | double | 0.5 | disc radius [m] folded into `d_safe` |
| `cbf_gamma` | double | 1.0 | reserved; 1.0 keeps the pointwise constraint |
| `costmap_cost_threshold` | int | 200 | occupied-cell threshold [0-255] |
| `obstacle_cluster_radius` | double | 0.3 | cell-grouping radius [m] |
| `log_level` | string | info | plugin logger severity |

Plugin registration in a controller_server config.
The shipped YAML deliberately documents only the `FollowPath` block and omits the standard server wiring.
A runnable config additionally needs the server frequency and the progress/goal-checker plugins, for example:

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
    FollowPath:
      plugin: "prox_mpc_controller::ProxMpcController"
      # ... the keys from the table above ...
```

Cruise-speed parameter (resolved).
The "configured cruise speed" that [docs/control-law.md](docs/control-law.md) needs for the reference is now declared as `desired_linear_vel` (double, m/s) in [config/prox_mpc_controller.yaml](config/prox_mpc_controller.yaml), defaulting to 1.0 m/s.
It drives the arc-length sampling `s_k = s_0 + desired_linear_vel * k * dt` and the speed channel of `goal_u`, is reduced near high-curvature segments, and is clamped by any active speed limit.
The default sits well below the bundled models' speed bound (`v_max = 3.0 m/s`, [../prox_mpc_core/include/prox_mpc/models/bike.hpp:39](../prox_mpc_core/include/prox_mpc/models/bike.hpp#L39)); `configure()` should validate `0 < desired_linear_vel <= v_max` and clamp with a warning otherwise.

`log_level` handling (verified against the in-tree Nav2 controllers).
The in-tree Jazzy controllers were checked directly — Regulated Pure Pursuit, MPPI, Graceful, DWB, and `nav2_controller` — and none of them exposes a `log_level` parameter or calls a logging-level setter, so a per-plugin `log_level` is not a Nav2 convention.
What every reference controller does instead is declare a fixed, top-level named logger after its class, for example `rclcpp::Logger logger_ {rclcpp::get_logger("RegulatedPurePursuitController")}` (`regulated_pure_pursuit_controller.hpp:215`) and `rclcpp::get_logger("GracefulController")` (`graceful_controller.hpp:165`).
Verbosity is then controlled by the standard ROS 2 mechanism — `--ros-args --log-level ProxMpcController:=<level>` on the host process, or the host's `<controller_server>/set_logger_levels` service at runtime — never by a parameter.

There are two consequences for this plugin.

- Keep the scoped logger and drop the override. The header already declares `logger_{rclcpp::get_logger("ProxMpcController")}` ([include/prox_mpc_controller/prox_mpc_controller.hpp:72](include/prox_mpc_controller/prox_mpc_controller.hpp#L72)), which matches the Nav2 standard, but `configure()` currently overwrites it with `logger_ = node->get_logger();` ([src/prox_mpc_controller.cpp:27](src/prox_mpc_controller.cpp#L27)). That reassignment makes the plugin log under the `controller_server` logger name, so a `--log-level controller_server:=debug` would also raise the verbosity of the server and every sibling plugin. Remove the reassignment so the plugin keeps its own `ProxMpcController` logger.
- Honour the project `log_level` key as the source of truth. This project's `CLAUDE.md` mandates a `log_level` YAML key, whose launch-file mechanism (`--log-level <node>:=<level>`) does not apply to a plugin that has no node and no launch file. The faithful adaptation is to read `log_level` in `configure()` and apply it to the plugin's own logger programmatically with the idiomatic rclcpp call `logger_.set_level(rclcpp::Logger::Level::<...>)` (`rclcpp/logger.hpp:169`, `enum class Level` at `:96`; the C-level `rcutils_logging_set_logger_level(name, level)` at `rcutils/logging.h:392` is the same mechanism one layer down).

Net: this is a superset of the Nav2 norm, not a replacement for it.
The standard `--log-level` / `set_logger_levels` path keeps working on the `ProxMpcController` logger, and the project's YAML key additionally seeds that logger's level at configure time, scoped to this plugin only, with no global or RMW change.
Flag: the per-plugin `log_level` parameter is this project's convention, not a Nav2 one; if matching upstream Nav2 exactly is preferred, drop the parameter and rely solely on the standard ROS log-level mechanism.

## 5. Lifecycle wiring

`controller_server` is a lifecycle node and drives the plugin from its own transitions; the plugin itself is not a node.
The mapping, all already wired and only expanded in behavior:

- Server `on_configure` -> plugin `configure()`: one-time setup. Read parameters, load and configure the model, size and configure the MPC, create (unactivated) debug publishers.
- Server `on_activate` -> plugin `activate()`: activate debug publishers; reset failure counter, warm-start, and tracked steering.
- Server `on_deactivate` -> plugin `deactivate()`: deactivate publishers; stop processing.
- Server `on_cleanup` -> plugin `cleanup()`: release the MPC, model, class loader, costmap and TF handles, and publishers, returning to a configurable state.

Deterministic teardown: every handle acquired in `configure()`/`activate()` is released in `cleanup()`/`deactivate()`, preserving the RAII and cleanup expectations.

## 6. Costmap, global plan, and goal-checker handling

Costmap.
`costmap_ros_` is already cached.
The accessors below were read from the installed headers `/opt/ros/jazzy/include/nav2_costmap_2d/nav2_costmap_2d/costmap_2d_ros.hpp` and `costmap_2d.hpp`.

- Obtain the master grid with `nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();` (`costmap_2d_ros.hpp:234`).
- Lock it for the read with `std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));`; `mutex_t` is a `std::recursive_mutex` (`costmap_2d.hpp:371-372`).
- Read a cell cost with `costmap->getCost(mx, my)` returning `unsigned char` (`costmap_2d.hpp:149`); convert between frames with `costmap->worldToMap(wx, wy, mx, my)` (bounds-checked, returns `bool`, `costmap_2d.hpp:183`) and `costmap->mapToWorld(mx, my, wx, wy)` (`:173`); the grid extent and scale come from `getSizeInCellsX()/getSizeInCellsY()`, `getResolution()`, and `getOriginX()/getOriginY()`.
- An occupied cell is `getCost(mx, my) >= costmap_cost_threshold` (200). The reference cost values are in `cost_values.hpp`: `LETHAL_OBSTACLE = 254`, `INSCRIBED_INFLATED_OBSTACLE = 253`, `MAX_NON_OBSTACLE = 252`, `NO_INFORMATION = 255`, `FREE_SPACE = 0`. The 200 threshold captures lethal, inscribed, and strongly inflated cells; `NO_INFORMATION` (255) also clears the threshold, so the reduction should exclude unknown cells unless unknown space is intended to block (see Section 11).

For each predicted node, select the nearest occupied cells or clusters (merged within `obstacle_cluster_radius`) and emit at most `K` `(o_x, o_y, d_safe)` triples with `d_safe = robot_radius + inflation + safety_margin`.
The local costmap ships no distance-transform / ESDF layer, so the implementation uses a bounded windowed scan around each predicted position (window sized from `getResolution()` and the obstacle search radius); this keeps the per-cycle cost at roughly `O(window_cells * Np)`, which must stay bounded for the Jetson budget.
Empty slots are filled with `MPC::kObsFarSentinel` so a fixed `K` degrades cleanly.
Pass the `(Np*K) x 3` matrix with `mpc_->setObs(...)` after `mpc_->setMaxObs(K)` was set in `configure()`.

Global plan.
`setPlan()` already stores `global_plan_`.
Per cycle, transform the relevant plan poses into the costmap global frame (`costmap_ros_->getGlobalFrameID()`, `costmap_2d_ros.hpp:243`) with `tf_`, prune already-passed poses, project the current pose onto the plan to get `s_0`, then sample at `s_k = s_0 + desired_linear_vel * k * dt` to build `goal_x` (`(Np+1) x n`) and `goal_u` (`Nc x m`), with heading from the path tangent and, for the bicycle, reference steering `delta_k = atan(L * kappa_k)`, as specified in [docs/control-law.md](docs/control-law.md).
The current pose can come straight from the `pose` argument of `computeVelocityCommands`, or from `costmap_ros_->getRobotPose(global_pose)` (`costmap_2d_ros.hpp:205`); single-pose frame conversions can reuse `costmap_ros_->transformPoseToGlobalFrame(in, out)` (`:213`).
The bicycle steering angle `delta` is not in the Nav2 pose, so the controller tracks it (integrating `delta += dt * delta_dot` from the previous command, or from a steering sensor) and feeds the full state to `setPose`.

Goal checker.
The `nav2_core::GoalChecker *` is passed per cycle and must not be stored (it is an unowned raw pointer).
Use `goal_checker->getTolerances(...)` to taper `v_ref` and shape the terminal reference as the goal approaches; `controller_server` independently uses the same checker to decide task completion.

## 7. Edge cases and fail-safe behavior

The fail-safe types below were read from `/opt/ros/jazzy/include/nav2_core/controller_exceptions.hpp`; throwing one of these from `computeVelocityCommands()` is how a controller signals the behavior tree to recover.

Convergence check (the success qualifier).
After `solve()` the caller reads `mpc_->qp_info.status`.
`qp_info` is a `proxsuite::proxqp::Info<double>` ([../prox_mpc_core/include/prox_mpc/mpc.hpp:90](../prox_mpc_core/include/prox_mpc/mpc.hpp#L90)) and its `status` field is of type `proxsuite::proxqp::QPSolverOutput`, declared in `/usr/local/include/proxsuite/proxqp/status.hpp` as `enum struct QPSolverOutput { PROXQP_SOLVED, PROXQP_MAX_ITER_REACHED, PROXQP_PRIMAL_INFEASIBLE, PROXQP_DUAL_INFEASIBLE, PROXQP_NOT_RUN }`.
Because it is a scoped enum (`enum struct`), the enumerator cannot be written bare; it must be fully qualified as `proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED`.
This is not an assumption: the core already compares against exactly this qualifier ([../prox_mpc_core/src/mpc.cpp:126](../prox_mpc_core/src/mpc.cpp#L126) and [:133](../prox_mpc_core/src/mpc.cpp#L133)), so the controller must mirror it.
Success is `status == proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED`; any other value (max-iter reached, primal/dual infeasible, or not-run) is a failure and triggers the deceleration fallback below.

- Empty or unset plan (`global_plan_.poses.empty()`, or zero usable poses after transform and prune): treat it as a structural fault and throw `nav2_core::InvalidPath` immediately. This is the production-grade behaviour and matches the in-tree Nav2 controllers (Regulated Pure Pursuit, for example, throws `InvalidPath("Received plan with zero length")` from its plan transform): `controller_server` catches the `ControllerException`, the `FollowPath` BT action returns failure, and the recovery / contingency subtree replans or aborts cleanly. Commanding zero velocity indefinitely is rejected because it masks the fault — the robot sits still while the behaviour tree believes control is progressing and no recovery ever fires. The deceleration ramp is reserved for transient faults (a single non-converged solve); a missing or empty plan is not transient, so it escalates on the first cycle rather than crawling.
- Short plan (fewer poses than the horizon): clamp sampling and hold the final pose as the reference for the remaining nodes (goal-hold), tapering `v_ref` to zero near the end.
- Solver failure (`qp_info.status != proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED`): do not command a hard zero. Ramp the last command toward zero at the robot deceleration limit `a_dec = |lower du bound on speed|`, read from the model with `model->getIneq("du", 0)` (the bundled models default this bound to 0.5 m/s^2, [../prox_mpc_core/include/prox_mpc/models/bike.hpp:41](../prox_mpc_core/include/prox_mpc/models/bike.hpp#L41)); `v_cmd = max(0, v_prev - a_dec * dt)`, with the yaw/steering channel ramped likewise. Increment a consecutive-failure counter; once it exceeds `max_solver_failures`, throw `nav2_core::NoValidControl` so the behaviour tree replans. Reset the counter on any converged solve.
- Infeasible or invalid state (non-finite pose, NaN in the solved control, current cell in collision): guard before mapping `toTwist`; on a non-finite solution treat the cycle as a solver failure (decelerate). For TF failures while transforming the plan or pose, throw `nav2_core::ControllerTFError`.
- Footprint veto: after a converged solve, run an exact polygon-footprint collision check with `nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>` (template class in `/opt/ros/jazzy/include/nav2_costmap_2d/nav2_costmap_2d/footprint_collision_checker.hpp`). Construct or `setCostmap(...)` it over `costmap_ros_->getCostmap()`, take the footprint from `costmap_ros_->getRobotFootprint()` (the padded footprint, a `std::vector<geometry_msgs::msg::Point>` aliased as `nav2_costmap_2d::Footprint`, `costmap_2d_ros.hpp:279`), and call `footprintCostAtPose(x, y, theta, footprint)` at the pose the first optimal control would reach one step ahead. Treat a returned cost of `LETHAL_OBSTACLE` (254) or `NO_INFORMATION` (255) — or `>= INSCRIBED_INFLATED_OBSTACLE` (253) for a conservative check — as a collision and veto the command, decelerating instead. The cost constants are in `cost_values.hpp`.
- Speed limit edge values: `setSpeedLimit` with `percentage = true` converts to a fraction of the model maximum; a limit of zero clamps the applied bound to zero.

## 8. Out of scope

- Any change to `prox_mpc_core` (the SQP loop, the QP, the obstacle math, or the `Model` interface). The engine stays frozen.
- New vehicle models. Model selection is a configuration change via `model_plugin`.
- The control-barrier-function coupling across horizon nodes; `cbf_gamma` stays at 1.0 (pointwise) until cross-node obstacle association exists.
- A bespoke velocity smoother; the deceleration fallback is self-contained and does not assume one downstream.
- Multi-robot, 3D/SE(3), or non-planar costmaps.
- A dedicated launch file or bringup for a full Nav2 stack beyond the controller_server snippet needed for verification.
- Performance tuning and Jetson profiling beyond keeping the costmap scan bounded; a dedicated optimization pass is separate follow-up.
- Unit/integration test authoring is acknowledged as required follow-up but is not specified in detail by this conversion plan.

## 9. Verification

End-to-end checks, run from a clean overlay workspace (for example `/tmp/ros_ws`) and only after the maintainer confirms the build command and package set:

1. Clean build. `colcon build --symlink-install --packages-select prox_mpc_core prox_mpc_controller` completes with no errors and no new warnings, with full unfiltered output.
2. Plugin discovery. After sourcing the overlay, confirm the controller plugin is registered, for example `ros2 plugin list` / inspecting the installed `nav2_core` plugin XML index shows `prox_mpc_controller::ProxMpcController`.
3. Plugin loads in `controller_server`. Launch `controller_server` (isolated `ROS_DOMAIN_ID=99`, under `timeout`) with a complete config selecting `FollowPath -> prox_mpc_controller::ProxMpcController`; confirm the log line `Configured ProxMpcController 'FollowPath'` and a clean `configure`/`activate` lifecycle transition with parameters loaded and the model plugin loaded, and no errors.
4. Emits velocity commands. With the server active, publish (or drive from a minimal Nav2 stack) a costmap, a TF for the robot pose, and a non-trivial global plan to `FollowPath`, then confirm a non-zero `geometry_msgs/msg/TwistStamped` stream on the controller command topic (`ros2 topic echo /cmd_vel`, or the controller_server `computeVelocityCommands` output), with the throttled "not implemented" warning gone.
5. Fail-safe spot check. Feed an empty/short plan and an infeasible obstacle field and confirm the documented braking ramp and the recovery escalation after `max_solver_failures`.

Each verification step is gated on explicit confirmation of the build/run commands per the workspace operating contract; this stage does not run any build or test.

## 10. Resolved decisions

The questions raised by the first draft are now resolved against the code and the installed headers:

- Cruise speed: added as `desired_linear_vel` (double, default 1.0 m/s) in [config/prox_mpc_controller.yaml](config/prox_mpc_controller.yaml); validated against the model `v_max` (Section 4).
- `log_level`: verified that no in-tree Nav2 controller uses a `log_level` param; keep the fixed `ProxMpcController` named logger (drop the `node->get_logger()` override) and seed its level from the param via `logger_.set_level(...)`, while the standard `--log-level` / `set_logger_levels` path stays the primary mechanism (Section 4).
- `nav2_costmap_2d` access: the `Costmap2DROS` / `Costmap2D` accessors, the `std::recursive_mutex` lock, and the `FootprintCollisionChecker` API are read from the installed headers and pinned in Sections 6 and 7.
- Empty-plan reaction: throw `nav2_core::InvalidPath` immediately, matching the production Nav2 controllers; the deceleration ramp is reserved for transient solver failures (Section 7).
- Success qualifier: `proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED`, a scoped enum that must be fully qualified, confirmed against the core's own usage (Section 7).
- `cancel()` / `reset()`: both overridden — `cancel()` performs a graceful deceleration ramp and returns `true` only once stopped; `reset()` clears runtime state between tasks (Section 2).

## 11. Residual items for robot-specific tuning

These are deployment choices, not interface unknowns, to settle with the target platform:

- The numeric `desired_linear_vel` (and any per-robot `model_params` overrides of `v_max` / acceleration bounds) for the real vehicle.
- Whether `NO_INFORMATION` (255) cells count as obstacles in the costmap reduction, or are skipped so unknown space does not hard-block the optimizer.
- Whether the footprint veto uses `getRobotFootprint()` with `footprintCostAtPose(...)` or the pre-oriented `costmap_ros_->getOrientedFootprint(...)` at the predicted pose.
- The `cancel()` stop epsilon and whether a dedicated `deceleration_limit` parameter should override the model-derived `a_dec`.
