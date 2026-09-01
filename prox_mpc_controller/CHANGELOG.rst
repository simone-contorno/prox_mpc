^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package prox_mpc_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2026-09-01)
------------------
* **Breaking:** ``predict_obstacles`` defaults to ``true``. With no tracker
  publishing, or a stale message, the fill degrades to the costmap-only path, so
  the default costs a subscription rather than a behavior change. Set it
  ``false`` to reproduce the previous behavior bit-for-bit.
* **Breaking:** ``model_plugin`` defaults to ``prox_mpc_core/Unicycle`` rather
  than ``prox_mpc_core/Bicycle``.
* ``allow_reversing`` now gates the control box, not only the reference: with it
  ``false`` the linear bound is narrowed to ``[0, v_max]`` so the solver cannot
  plan reverse travel at all. With it ``true`` and no explicit
  ``model_params.v_min``, reverse is capped at 0.15 m/s, because neither the
  keep-out fill nor the footprint veto observes the area behind the robot.
  ``model_params.v_min`` is also forwarded on its own when negative.
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
