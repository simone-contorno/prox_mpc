^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package prox_mpc_demo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2026-09-13)
------------------
* The local costmap tracks unknown space, so cells the sensor has not swept are
  marked ``NO_INFORMATION`` instead of being published as free.
* Nav2 parameter files follow the controller's changed defaults.
* The Nav2 parameter files set ``allow_reversing: true``, so the demo shows the
  robot backing up to adjust and then pursuing the path forward. Reverse used to
  read as a shuffle near the goal; that was the reference collapsing to a stub
  ahead of the robot's own projection inside the goal tolerance, not reverse
  itself, and the controller now pins the reference to the goal pose there and
  turns on the spot for the last of the heading. Measured over ``cmd_vel_nav``,
  reverse-enabled now completes every demo scenario faster than forward-only
  (27.8 s vs 38.6 s, 12.2 s vs 19.8 s, 9.5 s vs 21.7 s) with no
  ``Failed to make progress`` events against two for forward-only.
* The predictive Nav2 parameter file sets ``obstacle_yield_band_m: 0.5``, so the
  robot waits for a crossing mover rather than racing it: it passed behind 6/10
  against 1/10 on ``dynamic_circle`` with no collisions, and was no worse with
  two movers. Not set in the non-predictive file, which runs no tracker.
* Contributors: Simone Contorno

1.0.0 (2026-07-28)
------------------
* Initial release: self-contained closed-loop NMPC simulation node (bicycle / unicycle)
  with solve-time benchmarking, plus the Nav2 + Gazebo Harmonic bringup that
  exercises the controller plugin and the obstacle tracker.
* ``config/nav2_prox_mpc_predictive.yaml`` ships the benchmark's validated
  single-/sparse-obstacle preset (``w_weight`` 1000, ``costmap_cost_threshold``
  253, ``cbf_gamma`` 1.0, ``max_obstacles`` 4,
  ``prediction_uncertainty_growth`` 0.05, ``max_dynamic_obstacles`` 2), so the
  demo's predictive path matches the benchmark's measured behavior.
* Contributors: Simone Contorno
