// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


#ifndef PROX_MPC_OBSTACLE_TRACKER__OBSTACLE_TRACKER_NODE_HPP_
#define PROX_MPC_OBSTACLE_TRACKER__OBSTACLE_TRACKER_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rcpputils/thread_safety_annotations.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <prox_mpc_msgs/msg/obstacle_array.hpp>

#include "prox_mpc_obstacle_tracker/tracker.hpp"

namespace prox_mpc_obstacle_tracker
{

/// Managed lifecycle node that detects and tracks dynamic obstacles from a 2D
/// lidar and publishes them as a prox_mpc_msgs/ObstacleArray. The per-scan
/// pipeline (LaserScan -> planar points -> clusters -> tracking-frame centroids
/// -> IMM CV+CTRV tracks with sampled predicted positions) feeds the in-house
/// Tracker; only the algorithm-free ROS integration lives here.
class ObstacleTrackerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit ObstacleTrackerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// Declare and validate parameters; create the publisher and the tracker. No
  /// subscription or TF listener yet (both created on activate).
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /// Create the TF listener, activate the publisher, and subscribe to the scan.
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /// Stop processing: drop the subscription and TF listener, deactivate the publisher.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /// Release the publisher, TF, and tracker, returning to unconfigured.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /// Final teardown (same release path as cleanup), idempotent.
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  /// Process one scan: cluster, transform centroids to the tracking frame, update
  /// the tracker, and publish the confirmed tracks. Skips the scan on a TF gap.
  void scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);

  /// Release the publisher, TF buffer/listener, and tracker (cleanup/shutdown).
  void teardown();

  // Parameters (declared and validated in on_configure).
  std::string log_level_;   // node-only logger verbosity (debug/info/warn/error/fatal)
  std::string scan_topic_;
  std::string output_topic_;
  std::string tracking_frame_;
  double cluster_gap_{0.3};
  int min_cluster_points_{3};
  int max_clusters_{20};
  double max_cluster_radius_{0.0};   // 0 = no limit; >0 rejects extended structure (walls)
  double min_detection_range_{0.0};
  double max_detection_range_{0.0};   // 0 -> use the scan's own range_max
  // Arc-centroid -> disc-centre correction: fraction of the enclosing cluster
  // radius the centroid is pushed away from the sensor along its ray (a lidar
  // sees only the near arc, so the raw centroid is biased toward the sensor and
  // slides around the disc as the viewpoint changes). 0 disables (raw centroid).
  double cluster_center_offset_gain_{0.0};
  double transform_timeout_{0.1};
  // Predicted-sample spacing [s], mirrored from the tracker params for the
  // publish loop (Obstacle.prediction_dt; 0.0 is written when a track has no
  // samples). The other IMM/CTRV parameters live only in Tracker::Params.
  double prediction_dt_{0.1};

  // ROS interfaces. scan_sub_ is written only from the executor thread that runs
  // the lifecycle transitions (on_activate/on_deactivate/teardown), which never
  // overlaps a scan callback because the subscription itself does not exist yet
  // or has already been reset by the time this member changes, so it is not
  // part of the state_mutex_-guarded set below.
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<prox_mpc_msgs::msg::ObstacleArray>>
  obstacle_pub_ RCPPUTILS_TSA_GUARDED_BY(state_mutex_);
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_ RCPPUTILS_TSA_GUARDED_BY(state_mutex_);
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_ RCPPUTILS_TSA_GUARDED_BY(state_mutex_);

  std::unique_ptr<Tracker> tracker_ RCPPUTILS_TSA_GUARDED_BY(state_mutex_);
  std::atomic<bool> active_{false};

  /// Mutually-exclusive group for the scan callback so it never overlaps itself,
  /// and a mutex serializing the scan callback against lifecycle teardown so the
  /// shared perception state (tracker_, obstacle_pub_, tf_buffer_, tf_listener_)
  /// is never used after reset. This makes the node safe to compose into a
  /// MultiThreadedExecutor as well as to run standalone (SingleThreadedExecutor).
  rclcpp::CallbackGroup::SharedPtr scan_callback_group_;
  std::mutex state_mutex_;
};

}  // namespace prox_mpc_obstacle_tracker

#endif  // PROX_MPC_OBSTACLE_TRACKER__OBSTACLE_TRACKER_NODE_HPP_
