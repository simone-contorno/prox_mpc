^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package prox_mpc_demo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Retuned ``config/nav2_prox_mpc_predictive.yaml`` to the benchmark's validated
  single-/sparse-obstacle preset (``w_weight`` 100 -> 1000,
  ``costmap_cost_threshold`` 200 -> 253, ``cbf_gamma`` 0.3 -> 1.0,
  ``max_obstacles`` 2 -> 4, ``prediction_uncertainty_growth`` 0.0 -> 0.05,
  ``max_dynamic_obstacles`` 1 -> 2), so the demo's predictive path matches the
  benchmark's measured behavior instead of stalling short of a single obstacle.
* Contributors: Simone Contorno

1.0.0 (2026-07-01)
------------------
* Initial release: self-contained closed-loop NMPC simulation node (bicycle / unicycle)
  with solve-time benchmarking, plus the Nav2 + Gazebo Harmonic bringup that
  exercises the controller plugin and the obstacle tracker.
* Contributors: Simone Contorno
