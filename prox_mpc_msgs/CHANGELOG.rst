^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package prox_mpc_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-07-01)
------------------
* Initial release: ``Obstacle`` and ``ObstacleArray`` messages for tracked
  dynamic obstacles (position, velocity, radius, and covariances).
* ``SolverDiagnostics`` message: per-control-cycle NMPC/QP solver telemetry
  (solve times, QP status and convergence, SQP/QP iteration counts, residuals,
  objective, max obstacle slack, control period, and deadline/active-obstacle
  signals) for benchmarking; publishers are opt-in.
* Contributors: Simone Contorno
