^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package prox_mpc_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2026-09-13)
------------------
* **Breaking:** ``predict_obstacles`` defaults to ``true``. With no tracker
  publishing, or a stale message, the fill degrades to the costmap-only path, so
  the default costs a subscription rather than a behavior change. Set it
  ``false`` to reproduce the previous behavior bit-for-bit.
* **Breaking:** ``model_plugin`` defaults to ``prox_mpc_core/Unicycle`` rather
  than ``prox_mpc_core/Bicycle``.
* **Breaking:** new ``allow_reversing`` parameter, default ``false``, bounds the
  solver's linear control to ``[0, v_max]``, so it cannot plan reverse travel at
  all. 1.0.0 had no such switch; its reference was forward-only, but its solver
  always admitted reverse down to ``-v_max``, so a deployment that upgrades
  without setting it loses reverse. With it ``true`` and no explicit
  ``model_params.v_min``, reverse is capped at 0.15 m/s: both guards follow the
  predicted trajectory and so do cover a reversing one, but they see only what
  the costmap holds, and whether the platform sweeps behind itself is a property
  of its sensor rather than of this plugin.
  ``model_params.v_min`` is also forwarded on its own when negative.
* New ``reverse_from_plan_orientation`` parameter, default ``false``: with it
  and ``allow_reversing`` set, the plan's pose orientations sign the reference
  into reverse and truncate it at the first direction change. It defaults off
  because only a planner that sets pose orientations means anything by them, and
  a plan carries nothing that reports which planner produced it: NavFn and Smac
  2D emit the identity quaternion on every pose, which is indistinguishable from
  a straight reverse plan. Trusting them would read any path running against
  that one fixed heading as a reverse traverse, so the robot would drive the
  whole path backwards instead of turning around, and a path whose heading
  component changes sign would flip the reference from cycle to cycle. Set it
  ``true`` with a cusp-emitting planner (Smac Hybrid-A*, State Lattice).
* The reference is pinned to the goal pose inside the goal-checker xy tolerance
  instead of tracking the robot's own projection onto the plan. The cruise taper
  and the sampling step composed to give the reference horizon an arc reach of
  ``remaining^2 / xy_tol``, shorter than ``remaining`` at every point inside the
  tolerance, so the goal region was tracked against a stub a few millimetres
  ahead of the projection that the projection then carried along: a reference
  with no fixed point, which let a small tracking error be traded down as cheaply
  one way as the other, and whose horizon never reached the plan end, so the
  goal's own orientation never entered it. Pinning gives the last stretch a fixed
  setpoint and a standing heading error. The mode is latched and released only
  past the new ``goal_settle_hysteresis_m``, default 0.10 m.
* Once the goal checker's xy condition is met and only the heading is left, a
  platform with no steering channel holds station and turns on the spot rather
  than reversing and re-advancing as the body sweeps round. A steering model is
  excluded, having no way to turn on the spot.
* Travel direction is a latched mode rather than a fresh read of the plan
  geometry each cycle. A change is accepted only once the platform is at or below
  ``direction_switch_standstill_speed_mps`` (default 0.05 m/s) and
  ``direction_switch_dwell_s`` (default 0.5 s) has elapsed since the last change;
  until then the reference is held at the arc length under the robot, so the
  commanded speed falls to zero and the platform stops first. A cusp is driven
  the way a vehicle drives one: arrive, stop, shift, pull away. Both gates are
  inert unless ``reverse_from_plan_orientation`` is set.
* New ``prediction_forward_shadow_s`` parameter, default ``0.0`` (inert). The
  predictive keep-out is a disc about the predicted position and the constraint
  normal runs from the obstacle to the robot, so the formulation does not
  distinguish the space a mover is about to occupy from the space it is vacating;
  passing in front and passing behind cost the same, and because the reference
  carries no obstacle term, the front is the side that does not require lagging
  it. Set positive, the disc is biased forward by that many seconds of the
  track's own travel, with the radius grown by the same distance so the
  obstacle's own position stays covered. Left off by default: the mechanism is
  unit-tested, but an A/B on the demo walker was directionally favourable and
  statistically inconclusive at four runs per arm.
* New ``obstacle_yield_band_m`` parameter, default ``0.0`` (inert): eases the
  cruise when where the robot is heading - the plan at the intended cruise, or
  the trajectory the solver planned last cycle, whichever breaches deeper -
  would cut into a tracked mover's predicted keep-out, so the robot waits for
  the mover instead of racing it. The release is rate-limited so it cannot snap
  back to full cruise. Enable it with ``allow_reversing``: with reversing off
  a waiting robot is boxed in by a second mover, which is why it stays off by
  default. It is not to be combined with light stage weights.
* New ``obstacle_yield_caps_speed`` parameter, default ``false``: the
  obstacle-aware yield also caps the solver's forward speed bound, so it slows
  the robot rather than only lowering a cruise target the obstacle term can
  override. The cap falls no faster than the platform can brake, so the solver
  always has a feasible first control, and leaves the reverse bound alone.
* The footprint veto walks the predicted trajectory as far as the robot's
  stopping distance rather than checking a single pose one step ahead.
* Obstacle slots are bound to one object per cycle and ranked by time to
  encounter; the scan is centred on the predicted state and the keep-out on
  ``base_link``.
* Solver acceptance is transactional: a candidate is committed only once the
  finiteness, veto and command gates accept, and convergence is reported past
  those gates.
* The solver-failure brake ramps in control space through the model, seeded from
  the model's declared inverse in its own units, with a configurable step.
* Perception state is mutex-guarded on every path, the costmap lock is narrowed
  to the cell reads, and the footprint read is hoisted out of the grid lock.
* ``reset()`` clears the predicted-obstacle markers. They were cleared only by
  the next control cycle, and the controller server stops cycling when a task
  ends, so RViz kept drawing the last prediction after the goal while the
  obstacle moved on.
* ``configure()`` validates its parameters, warns when ``robot_radius``
  undersizes the costmap footprint, rejects a zero model speed bound and a
  non-finite ``desired_linear_vel``, and clamps a Nav2 speed limit into the
  model's own bounds rather than replacing them.
* Contributors: Simone Contorno

1.0.0 (2026-07-28)
------------------
* Initial release: ``nav2_core::Controller`` plugin wrapping the ProxMPC core,
  with global-plan reference building, costmap and predictive obstacle fills, a
  measured-velocity deceleration ramp, and an exact footprint veto.
* Fail-safe escalation to ``NoValidControl`` on a persistent solver failure or a
  persistent footprint veto; structural parameter validation with tuning clamps.
* Added ``max_solve_time`` and ``publish_diagnostics`` defaults to the shipped
  ``prox_mpc_controller.yaml`` config.
* Contributors: Simone Contorno
