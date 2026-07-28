^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package prox_mpc_demo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
