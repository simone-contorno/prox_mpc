^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package prox_mpc_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2026-09-13)
------------------
* No interface change: the message definitions and their wire format are
  unchanged from 1.0.0. Version raised to keep the repository's packages
  aligned.
* Added a wire-compatibility test for ``SolverDiagnostics``.
* Contributors: Simone Contorno

1.0.0 (2026-07-28)
------------------
* Initial release: ``Obstacle`` and ``ObstacleArray`` messages for tracked
  dynamic obstacles (id, position, velocity, radius, covariances, and the
  sampled predicted positions with their time step).
* ``SolverDiagnostics`` message: per-control-cycle NMPC/QP solver telemetry
  (solve times, QP status and convergence, SQP/QP iteration counts, residuals,
  objective, max obstacle slack, control period, and deadline/active-obstacle
  signals) for benchmarking; publishers are opt-in.
* Contributors: Simone Contorno
