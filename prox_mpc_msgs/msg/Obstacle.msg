# A single tracked dynamic obstacle, expressed in the frame of the enclosing
# ObstacleArray header. Position and velocity come from an in-house 2D-lidar
# detector and IMM (CV+CTRV) tracker (prox_mpc_obstacle_tracker); the legacy
# single-CV Kalman path is selected by imm_enabled: false.

# Stable track identifier, reused across scans for the same physical object.
uint32 id

# Planar centroid [m]; z is unused (kept 0) by the 2D tracker.
geometry_msgs/Point position

# Estimated velocity [m/s]; z is unused (kept 0) by the 2D tracker.
geometry_msgs/Vector3 velocity

# Enclosing radius of the detected cluster [m].
float64 radius

# 2x2 position covariance [m^2], row-major [xx, xy, yx, yy]. Informational: the
# tracker fills it, but the bundled controller reads neither covariance field.
float64[4] position_covariance

# 2x2 velocity covariance [m^2/s^2], row-major [xx, xy, yx, yy]. Informational:
# the bundled controller sizes its prediction-uncertainty clearance growth from
# its own prediction_uncertainty_growth parameter, not from this field.
float64[4] velocity_covariance

# Sampled predicted centroid positions [m], in the ObstacleArray header frame;
# sample k (0-based) is the prediction at header.stamp + (k+1) * prediction_dt.
# z is unused (kept 0). Empty when prediction sampling is disabled.
geometry_msgs/Point[] predicted_positions

# Spacing between predicted samples [s]; 0.0 when predicted_positions is empty.
float64 prediction_dt
