^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package prox_mpc_benchmark
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2026-09-13)
------------------
* ``record_scenarios.py`` cuts each clip to open as the robot first moves,
  read from ``/odom`` at the same 1 mm threshold that releases the scenario's
  obstacles, instead of starting a fixed clip before the goal is sent. The
  goal delay and the stack's start-up latency are no longer dead time in the
  clip, and a retried goal no longer shortens it. A scenario may set its own
  clip length with ``video.duration_s``; ``dynamic_circle`` sets 25 s so the
  orbit is seen through to the goal.
* ``record_scenarios.py --gpu-offload`` renders RViz on the NVIDIA GPU through
  PRIME render offload, set for the RViz process alone; it needs a real X server
  running the NVIDIA driver. ``--fullscreen`` starts RViz fullscreen and skips
  the window placement, which keeps a desktop session's panels and the window's
  title bar out of the capture. Both are off by default.
* The mode-(b2) run index is rebuilt from the records on disk, so a re-run of a
  subset no longer leaves stale rows from a previous campaign in the index.
* Per-run metrics record the QP status histogram and the SQP / QP external
  iteration maxima.
* Scenario definitions and the ProxMPC presets follow the controller's changed
  defaults.
* Contributors: Simone Contorno

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
