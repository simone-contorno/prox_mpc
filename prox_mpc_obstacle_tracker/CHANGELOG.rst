^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package prox_mpc_obstacle_tracker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2026-06-30)
------------------
* Initial release: lifecycle node that clusters a 2D ``LaserScan``, associates
  clusters to constant-velocity Kalman tracks in a fixed frame, and publishes a
  ``prox_mpc_msgs/ObstacleArray``.
* Composable component registration plus a mutually-exclusive callback group and
  teardown guard, so it is safe under a MultiThreadedExecutor; node-only
  ``log_level`` parameter and a standalone launch file.
* Contributors: Simone Contorno
