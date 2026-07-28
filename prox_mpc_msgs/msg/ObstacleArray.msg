# A set of tracked dynamic obstacles published once per processed scan. The
# header stamp is the source scan time (used by the controller to age the
# constant-velocity prediction) and the header frame_id is the tracking frame the
# positions and velocities are expressed in (a non-rotating frame, e.g. odom).

std_msgs/Header header
Obstacle[] obstacles
