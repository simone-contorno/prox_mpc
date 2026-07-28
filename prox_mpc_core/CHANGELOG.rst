^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package prox_mpc_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-07-28)
------------------
* Initial release: NMPC core solved by a Sequential Quadratic Programming scheme
  over the ProxQP solver, with a pluginlib ``prox_mpc::Model`` base and bundled
  Bicycle / Unicycle models.
* Linearized signed-distance obstacle half-planes with a discrete-time CBF
  coupling and bounded per-node slot capacity.
* Optional wall-clock budget for the SQP loop (fail-safe on overrun).
* Contributors: Simone Contorno
