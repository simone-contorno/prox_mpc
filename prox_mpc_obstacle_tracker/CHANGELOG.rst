^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package prox_mpc_obstacle_tracker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2026-09-13)
------------------
* Repeat shutdown signals are idempotent. One shutdown reaches the process
  twice whenever a supervisor signals the process group and the launch parent
  also forwards to each child, and treating the second as a force-quit skipped
  the finalize ladder on an ordinary shutdown.
* Mutex-guarded perception state is annotated.
* Contributors: Simone Contorno

1.0.0 (2026-07-28)
------------------
* Initial release: lifecycle node that clusters a 2D ``LaserScan``, associates
  clusters to an IMM (constant-velocity + constant-turn-rate) filter in a fixed
  frame, and publishes a ``prox_mpc_msgs/ObstacleArray``.
* Composable component registration plus a mutually-exclusive callback group and
  teardown guard, so it is safe under a MultiThreadedExecutor; node-only
  ``log_level`` parameter and a standalone launch file.
* Contributors: Simone Contorno
