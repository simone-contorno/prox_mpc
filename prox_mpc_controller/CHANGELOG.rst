^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package prox_mpc_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
