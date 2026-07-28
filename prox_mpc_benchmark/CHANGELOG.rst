^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package prox_mpc_benchmark
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-07-28)
------------------
* Initial release: scenario-driven benchmarking harness with the standalone
  matrix (mode a/b1, four scenarios x bicycle/unicycle models) and the Nav2
  cross-controller comparison (mode b2) against DWB, MPPI, Graceful, Regulated
  Pure Pursuit, and Vector Pursuit, reusing the ``prox_mpc_demo`` simulation node and
  ``prox_mpc_open`` map and the shared bicycle/unicycle/waffle robots, and
  shipping its own scalable world and Ackermann robot model.
* Live C++ metrics node tapping cross-track/goal error and
  ``SolverDiagnostics``, plus installed Python tooling for orchestration, map
  generation, goal sending, bag reduction, and aggregation.
* Added a ``nav2_core::Controller`` timing decorator (``timing_controller_wrapper``)
  so every controller's per-cycle ``computeVelocityCommands`` compute is
  measured identically, plus fair-tuned per-cycle compute/resource metrics
  (``compute_ms_p50``/``p95``/``max``) alongside the existing CPU/RSS/control-rate
  sampling for the cross-controller comparison.
* ``vector_pursuit_controller`` (apt ``ros-jazzy-vector-pursuit-controller``
  v2.0.0, Apache-2.0) is the comparison's single external fair peer, with a
  ``config/controllers/vector_pursuit.yaml`` preset, the ``exec_depend`` in
  ``package.xml``, and the controller in ``DEFAULT_CONTROLLERS`` in
  ``run_nav2.py`` and in every scenario's ``controllers:`` list.
* Contributors: Simone Contorno
