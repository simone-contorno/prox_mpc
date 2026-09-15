^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package prox_mpc_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2026-09-13)
------------------
* **Breaking:** ``warm_start`` defaults to ``true``. The ProxQP workspace is
  built once and updated in place, so the factorization and the previous
  primal/dual iterate carry over between cycles instead of being rebuilt every
  solve. Set it ``false`` to restore the previous behavior.
* **Breaking:** ``max_iter_sqp`` defaults to 1, making the real-time iteration
  scheme unconditional and per-cycle latency bounded by a single QP. The
  previous default of 100 could spend 99 further linearizations around a
  corrupted iterate after a failed solve.
* The horizon shift no longer overwrites the warm start's first control with the
  control already executed, which had left it violating its own control-rate
  chain on every cycle.
* Models declare their planar mapping and, where they have one, their
  steering-rate control; the keep-out rows are indexed by that mapping rather
  than by assumed state columns. The bicycle is split by reference point.
* Transactional solve entry point: a candidate is retained only once the
  caller's acceptance gates pass.
* Coupled obstacle linearisation completed and the CBF constraint assembly
  hardened; the inert first-node obstacle gradient is no longer built.
* Configuration-time validation across ``MPC`` and ``Model``: horizon setters,
  ``dt``/``T``, ``setMaxObs``, ``setCbfGamma``, inequality bound indices, and the
  rear-axle steering bound are all checked, and structural setters are rejected
  after ``init()``.
* Contributors: Simone Contorno

1.0.0 (2026-07-28)
------------------
* Initial release: NMPC core solved by a Sequential Quadratic Programming scheme
  over the ProxQP solver, with a pluginlib ``prox_mpc::Model`` base and bundled
  Bicycle / Unicycle models.
* Linearized signed-distance obstacle half-planes with a discrete-time CBF
  coupling and bounded per-node slot capacity.
* Optional wall-clock budget for the SQP loop (fail-safe on overrun).
* Contributors: Simone Contorno
