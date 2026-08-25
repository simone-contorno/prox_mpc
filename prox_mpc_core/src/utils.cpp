// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


#include <prox_mpc/utils.hpp>

namespace prox_mpc
{

/*!
 * Normalize the angle in the range [-pi, pi].
 * @param angle angle to normalize.
 */
void normalizeAngle(double & angle)
{
  angle = atan2(sin(angle), cos(angle));
}

/*!
 * Get the optimal path computed by the MPC.
 * @param x optimal states.
 * @param now current clock time.
 * @note Poses are stamped in the "map" frame.
 */
Path optimPath(const MatrixXd & x, const rclcpp::Time & now)
{
  Path path;

  path.header.frame_id = "map";

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.orientation.w = 1.;

  for (Eigen::Index i = 0; i < x.rows(); i++) {
    pose.pose.position.x = x.row(i)[0];
    pose.pose.position.y = x.row(i)[1];
    path.poses.push_back(pose);
  }

  for (auto & stamped_pose: path.poses) {
    stamped_pose.header.stamp = now;
  }

  return path;
}

}  // namespace prox_mpc
