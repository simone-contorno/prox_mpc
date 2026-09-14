# Migrating from prox_mpc_controller 1.0.0

What changed on the released surface of `prox_mpc_controller`, and what a 1.0.0
deployment has to do about it.
The version number this lands under is fixed by the maintainer against the final
diff and is not stated here.

## Summary

| Change | Source compatible | ABI compatible | Wire compatible |
| --- | --- | --- | --- |
| `model_plugin` defaults to `prox_mpc_core/Unicycle` | yes | yes | yes |
| New `allow_reversing` parameter, default `false`, closes the reverse travel 1.0.0 always allowed | yes | yes | yes |
| New `reverse_from_plan_orientation` parameter, default `false` | yes | yes | yes |
| New `direction_switch_standstill_speed_mps` / `direction_switch_dwell_s` parameters | yes | yes | yes |
| New `goal_settle_hysteresis_m` parameter, default `0.10` | yes | yes | yes |
| New `prediction_forward_shadow_s` parameter, default `0.0` | yes | yes | yes |
| New `obstacle_yield_band_m` / `obstacle_yield_caps_speed` parameters | yes | yes | yes |
| The reference is pinned to the goal pose inside the goal-checker xy tolerance | yes | yes | yes |
| New `brake_period_s` parameter, default `0.0` | yes | yes | yes |
| `ProxMpcController` gains `readModelMapping()` and cached mapping members | yes | **no** | yes |
| `fillStaticObstacles`, `reduceCostmap` and `publishDiagnostics` change signature; `a_dec_lin_`/`a_dec_ang_` are renamed | **no, for a class deriving from `ProxMpcController`** | **no** | yes |
| The model contract is validated at `configure()` and rejects what cannot be driven | yes | yes | yes |
| The in-loop keep-out is centred on `base_link` | yes | yes | yes |
| The deceleration ramp runs in control space, through the model's `toTwist()` | yes | yes | yes |
| `SolverDiagnostics.converged` reports the applied command, not the QP status | yes | yes | **yes, layout and hash unchanged** |
| A rejected cycle no longer advances the solver's retained state | yes | yes | yes |
| Terminal-heading reference past the plan end | yes | yes | yes |

No `.msg` field is added, removed, renamed, retyped or reordered, so nothing here
changes a type hash or a CDR layout.

## The default model changed

`model_plugin` now defaults to `prox_mpc_core/Unicycle`. It previously defaulted
to `prox_mpc_core/Bicycle`, which no longer describes one model.

A `params.yaml` that sets `model_plugin` explicitly is unaffected, including one
that sets `prox_mpc_core/Bicycle`: that name still resolves, to the front-axle
bicycle, and logs a deprecation warning naming its replacement.

A deployment that relied on the default to get a four-state steered model must
now set it:

```yaml
FollowPath:
  model_plugin: "prox_mpc_core/BicycleFrontAxle"   # or BicycleRearAxle
```

Choose by where the model's state refers: `BicycleFrontAxle` puts `(x, y)` at the
front axle, `BicycleRearAxle` at the rear axle, which is the `base_link` origin
for a car-like base. `prox_mpc_core/Bicycle` maps to the front-axle model.

## Reverse travel is now opt-in

1.0.0 had no reverse switch. Its reference only asked for forward travel, but the
solver's linear control bound was symmetric, `[-v_max, v_max]`, so the solver
could plan reverse on its own - backing out of a closing gap, for instance.

The new `allow_reversing` parameter defaults to `false`, which narrows that bound
to `[0, v_max]`. A deployment that upgrades without setting it no longer reverses
at all.

To keep reverse travel, set `allow_reversing: true`. Without an explicit negative
`model_params.v_min`, reverse is then capped at 0.15 m/s, because the footprint
veto and the keep-out see only what the costmap holds, and whether the platform
senses behind itself is a property of its sensor rather than of this plugin. Set
`model_params.v_min` to allow a faster reverse once the platform's rear coverage
is known.

## Closed-loop behaviour of a bicycle model changed

The previous bicycle mixed three conventions. Each new plugin is consistent, so a
deployment using one will see different motion:

- The commanded twist is the body twist of `base_link`. For the front-axle model
  that is `linear.x = v cos(delta)`, not the front-wheel speed.
- The pose checked against the costmap footprint is the `base_link` pose, carried
  back from the model's reference point. The footprint check was previously
  performed at the front axle for a front-axle model.
- The steering reference is derived for the model's own reference point:
  `asin(L * kappa)` for the front axle, `atan(L * kappa)` for the rear.
- The wheelbase is read from the loaded model, not from `model_params.L`. The two
  agree for the bundled models, which apply that key.

The in-loop keep-out disc is centred on `base_link`, which is the point the robot
disc and the costmap footprint are both defined about, whatever point the model's
state refers to. The solver constrains the model's reference point, so each
obstacle is written into the solver's frame by the rotated reference-point offset
and the costmap scan is centred on `base_link`. The QP half-plane and the
footprint veto therefore protect the same physical point, which they did not for
a front-axle model before.

`d_safe` is unchanged at `robot_radius + safety_margin`, and so is the scan
window. For a model referenced to `base_link` - every model that declares no
offset - the shift is exactly zero and the obstacle fill is bit-identical.

## A model this controller cannot drive is now rejected

`configure()` reads the model's declared planar mapping and throws
`nav2_core::ControllerException` rather than indexing out of range, which
`EIGEN_NO_DEBUG` would leave unchecked. It rejects a model with fewer than three
states or no control, an index outside the model's own dimensions, two planar
quantities sharing one index, and a steering angle without a usable declared
wheelbase or with a lateral reference offset.

The declared position may sit at any pair of state indices, with the in-loop
obstacle term active or not. The solver reads the same declaration and indexes
its keep-out rows through it, so a model that 1.0.0 could only drive with
`max_obstacles` set to 0 now drives with avoidance on.

It also rejects a model whose declared upper speed bound is zero or non-finite.
1.0.0 required the bound to be present but not to be usable, so a model
declaring `setIneq("u", 0, 0.0, 0.0)` configured cleanly, had its cruise speed
clamped to zero and never moved - the outcome the missing-bound message already
said it prevented.

A custom `prox_mpc::Model` that follows the previous implicit convention passes
unchanged, except that one carrying a steering angle must now declare its
wheelbase. See `prox_mpc_core/doc/migration.md`.

## Command validation covers every twist component

The command gate tested `linear.x` and `angular.z` while all six components of
the published twist come from the model. All six are now checked for finiteness,
on the accepted path and on the deceleration ramp. A model that fills only the
two planar components is unaffected.

## The deceleration ramp works in control space

1.0.0's brake wrote `linear.x` and `angular.z` directly, stepping both down from
the velocity the `controller_server` measured for that cycle. The ramp now runs
on the model's own controls, under the model's own `du` bounds, and maps the
result through `Model::toTwist()`. That is the only form that is correct for a
model whose second control is not a body yaw rate: the bicycle's is a steering
rate, so a twist-space ramp had no meaning for it.

Every control channel still starts from the measured velocity, carried into the
model's own control units through `Model::fromTwist()` rather than by writing
`linear.x` and `angular.z` into control slots directly: for
`prox_mpc_core/BicycleFrontAxle` the speed control is a front-wheel speed, and a
`base_link` speed placed there is projected by `cos(delta)` a second time on the
way out. The unicycle and `prox_mpc_core/BicycleRearAxle` both carry the
`base_link` speed in that channel already, so the number they start from is
unchanged.

`Model::fromTwist()` declares per channel what a body twist determines, and the
controller seeds exactly the channels it reports as determined. Both bicycles
report their steering rate undetermined, because a twist shows the steering
angle's effect and not the rate it is changing at, so that control starts from
the last commanded value. The unicycle's second control *is* a body yaw rate, so
it is determined and starts from the measured `angular.z`, as it did in 1.0.0.

A model that carries a steering angle keeps a separate belief about it, and
`activate()` and `reset()` zero that belief along with the last-commanded
control vector. If the first cycle to brake after either - from a cancel, a
solver failure, or a footprint veto - finds such a robot already turning, the
twist it emits is derived from a zero steering angle and so carries no yaw rate,
rather than ramping down from the yaw the robot actually has. Later cycles ramp
normally, because an accepted cycle records both the command it applied and the
steering angle it solved for. The speed channel is unaffected throughout,
because it starts from the measurement.

## `SolverDiagnostics.converged` means something different

The message defines `converged` as a converged solve whose applied iterate is
finite. The controller previously set it from the QP status alone and published
before its own gates ran, so a cycle that was vetoed by the footprint check and
braked was recorded as converged.

It is now set only past the finiteness, footprint-veto and command-conversion
gates, and publication moves with it.

**The message is unchanged**: field names, types, order and defaults are
identical, the generated IDL is identical, and the RIHS01 type hash is
unchanged. A subscriber needs no rebuild. What changed is the runtime meaning of
one field, so a bag recorded before this and a bag recorded after are not
directly comparable on `converged` counts. The QP's own outcome is still
available, unchanged, in the `status` field; compare on that instead when a
cross-version comparison is needed.

## A rejected cycle no longer advances the solver

The controller now solves into a candidate and commits it only once its gates
accept. A cycle that fails to converge, produces a non-finite iterate, is vetoed
by the footprint check, or maps to a non-finite command leaves the solver's
retained trajectory, slack and previous control exactly as the last accepted
cycle left them, and the deceleration ramp anchors the next cycle's control-rate
constraint on the command it actually sent.

This changes motion after a rejected cycle: the next solve no longer plans a step
away from a command the robot never received. No configuration changes.

## Terminal behaviour in the goal region

This one is not behind a parameter, so it applies on upgrade.

Inside the goal-checker xy tolerance the reference is now pinned to the goal
pose instead of tracking the robot's own projection onto the plan, and once the
checker's xy condition is met and only the heading is outstanding, a platform
with no steering channel holds station and turns on the spot. Previously the
cruise taper and the sampling step composed to give the reference horizon an arc
reach of `remaining^2 / xy_tol` - shorter than `remaining` everywhere inside the
tolerance - so the goal region was tracked against a stub a few millimetres ahead
of the projection that moved with the robot, and the goal's own orientation never
entered the reference at all. A deployment that relied on the controller stalling
at a terminal heading error and a recovery behaviour taking over will now see the
controller close that error itself.

The change is source, ABI and wire compatible, and the two gates that bound
direction switching are inert unless `reverse_from_plan_orientation` is set.

## New behaviour behind parameters

| Parameter | Type | Default | Units | Meaning |
| --- | --- | --- | --- | --- |
| `allow_reversing` | bool | `false` | - | Whether the solver may plan reverse travel; off narrows the linear control bound to `[0, v_max]`. |
| `reverse_from_plan_orientation` | bool | `false` | - | Whether the plan's pose orientations may sign the reference into reverse and truncate it at the first direction change. Requires `allow_reversing`. |
| `direction_switch_standstill_speed_mps` | double | `0.05` | m/s | Speed at or below which the platform counts as stopped for a travel-direction change. Inert unless `reverse_from_plan_orientation`. |
| `direction_switch_dwell_s` | double | `0.5` | s | Minimum time between two accepted direction changes. Inert unless `reverse_from_plan_orientation`. |
| `goal_settle_hysteresis_m` | double | `0.10` | m | Band beyond the goal-checker xy tolerance that the robot must re-cross before the terminal settle releases back to path tracking. |
| `prediction_forward_shadow_s` | double | `0.0` | s | Biases a tracked mover's predicted keep-out forward along its own heading by this many seconds of its travel, growing the radius by the same distance so its current position stays covered. `0.0` reproduces a centred keep-out exactly. |
| `obstacle_yield_band_m` | double | `0.0` | m | Depth of predicted keep-out breach over which the cruise is eased to zero, so the robot waits for a crossing mover instead of racing it. `0.0` reproduces the obstacle-blind cruise exactly. |
| `obstacle_yield_caps_speed` | bool | `false` | - | Makes that yield cap the solver's forward speed bound rather than only lowering a cruise target the obstacle term can override. Requires `obstacle_yield_band_m > 0`. |
| `brake_period_s` | double | `0.0` | s | Step the deceleration ramp advances by on a braking cycle. `0.0` measures the inter-cycle period instead and clamps it into `[dt, 2 * dt]`; a positive value overrides the measurement and is used as-is. |

`allow_reversing`'s default keeps 1.0.0's forward-only reference and, unlike
1.0.0, also closes the solver's reverse bound: see
[Reverse travel is now opt-in](#reverse-travel-is-now-opt-in). With it on, a
model that declares no reverse travel keeps the forward-only reference and logs
a warning at `configure()`.

`reverse_from_plan_orientation` is the switch that reads travel direction out of
the plan. It defaults `false` because only a planner that sets pose orientations
means anything by them, and a plan carries nothing that says whether its planner
did: NavFn and Smac 2D emit the identity quaternion on every pose, which is
byte-identical to a genuine straight reverse plan. Trusting them reads any path
running against that one fixed heading as a reverse traverse, so the robot drives
the whole path backwards instead of turning around, and a path whose heading
component changes sign flips the reference from cycle to cycle. Set it true with
a cusp-emitting planner (Smac Hybrid-A*, State Lattice); leave it false with
NavFn or Smac 2D.

The three prediction parameters are all inert at their defaults, so a 1.0.0
deployment upgrading without touching them keeps the previous obstacle
behaviour exactly. They are the release's avoidance work and are worth reading
before enabling: `obstacle_yield_band_m` makes the *reference* yield to a
crossing mover, and `obstacle_yield_caps_speed` is what turns that yield from a
request into a speed limit - on its own the eased cruise is the lightest term in
the cost and the solver overrides it, swerving at full speed rather than slowing.
Enable the pair together with `allow_reversing`: waiting is safe when the robot
can back off, and with reversing off a waiting robot is boxed in by a second
mover. `prediction_forward_shadow_s` is unit-tested but left off, its A/B having
been directionally favourable and statistically inconclusive. The measured
figures for all three are in [architecture.md](architecture.md).

`brake_period_s`'s default of `0.0` reproduces 1.0.0's ramp on a
`controller_server` running at `dt`, which stepped by the configured `dt`
unconditionally. On a server running slower than `dt` the measured period is
used instead, so the ramp still decelerates at the rate the model declares
rather than at a fraction of it. Set a positive value to make the ramp
independent of scheduling jitter.

The terminal-heading reference is not behind a parameter. Past the plan end the
reference pose is now the goal pose rather than the final segment's tangent,
engaged only when the goal checker publishes a yaw tolerance it enforces. Goal
approach on a plan whose final orientation differs from its final segment tangent
will differ from 1.0.0.

## Object layout

`ProxMpcController` gains three protected methods and twenty-nine protected
members, which changes `sizeof(ProxMpcController)`. The pluginlib load path is
unaffected, because the class is allocated and freed inside the same library. A
downstream package that links the exported target and derives from or holds
`ProxMpcController` directly must be rebuilt.

Its protected surface also changes in ways a derived class sees at compile time,
so the release is not source compatible for such a class. `fillStaticObstacles`
and `reduceCostmap` lose their leading plan-reference parameter, which the scan
stopped reading when it moved onto the solver's own nominal trajectory;
`publishDiagnostics` gains a `period_ms` parameter after `solve_ms`, so the
telemetry and the deceleration ramp report the same measured period; and
`a_dec_lin_`/`a_dec_ang_` become `fallback_ramp_lin_`/`fallback_ramp_ang_`,
which is what they hold: the rates the last-resort twist ramp steps by, not a
linear/angular acceleration pair. A derived controller that overrode or called
any of them must be updated. The three added protected methods are
`readModelMapping`, `keepOutShift` and `markCycleStart`.
