# A single tracked dynamic obstacle, expressed in the frame of the enclosing
# ObstacleArray header. Position and velocity come from an in-house 2D-lidar
# detector and constant-velocity Kalman tracker (prox_mpc_obstacle_tracker).

# Stable track identifier, reused across scans for the same physical object.
uint32 id

# Planar centroid [m]; z is unused (kept 0) by the 2D tracker.
geometry_msgs/Point position

# Estimated velocity [m/s]; z is unused (kept 0) by the 2D tracker.
geometry_msgs/Vector3 velocity

# Enclosing radius of the detected cluster [m].
float64 radius

# 2x2 position covariance [m^2], row-major [xx, xy, yx, yy]. Informational: the
# tracker fills it, but the bundled controller consumes only velocity_covariance.
float64[4] position_covariance

# 2x2 velocity covariance [m^2/s^2], row-major [xx, xy, yx, yy]; the controller
# sizes the prediction-uncertainty clearance growth from its trace.
float64[4] velocity_covariance
