// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


#include "prox_mpc_obstacle_tracker/obstacle_tracker_node.hpp"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <tf2/exceptions.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "prox_mpc_obstacle_tracker/clustering.hpp"

namespace
{

/// Planar yaw from a quaternion.
double quat_yaw(const geometry_msgs::msg::Quaternion & q)
{
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny, cosy);
}

/// Apply the project log_level key to this node's own logger only. Taken by value
/// because Logger is a lightweight name-keyed handle and set_level is non-const.
void apply_log_level(rclcpp::Logger logger, const std::string & level)
{
  if (level == "debug") {
    logger.set_level(rclcpp::Logger::Level::Debug);
  } else if (level == "warn") {
    logger.set_level(rclcpp::Logger::Level::Warn);
  } else if (level == "error") {
    logger.set_level(rclcpp::Logger::Level::Error);
  } else if (level == "fatal") {
    logger.set_level(rclcpp::Logger::Level::Fatal);
  } else {
    logger.set_level(rclcpp::Logger::Level::Info);   // "info" and any unknown value
  }
}

}  // namespace

namespace prox_mpc_obstacle_tracker
{

ObstacleTrackerNode::ObstacleTrackerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("prox_mpc_obstacle_tracker", options)
{
  // Mutually-exclusive group so the scan callback is serialized under a
  // MultiThreadedExecutor (composition); the state mutex below guards it against
  // lifecycle teardown.
  scan_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

ObstacleTrackerNode::CallbackReturn ObstacleTrackerNode::on_configure(
  const rclcpp_lifecycle::State & /*state*/)
{
  try {
    log_level_ = declare_parameter<std::string>("log_level", "info");
    apply_log_level(get_logger(), log_level_);   // node-only verbosity

    scan_topic_ = declare_parameter<std::string>("scan_topic", "scan");
    output_topic_ = declare_parameter<std::string>("output_topic", "tracked_obstacles");
    tracking_frame_ = declare_parameter<std::string>("tracking_frame", "odom");
    cluster_gap_ = declare_parameter<double>("cluster_gap", 0.3);
    min_cluster_points_ = declare_parameter<int>("min_cluster_points", 3);
    max_clusters_ = declare_parameter<int>("max_clusters", 20);
    max_cluster_radius_ = declare_parameter<double>("max_cluster_radius", 0.0);
    min_detection_range_ = declare_parameter<double>("min_detection_range", 0.0);
    max_detection_range_ = declare_parameter<double>("max_detection_range", 0.0);
    transform_timeout_ = declare_parameter<double>("transform_timeout", 0.1);

    Tracker::Params tp;
    tp.process_noise = declare_parameter<double>("process_noise", 1.0);
    tp.measurement_noise = declare_parameter<double>("measurement_noise", 0.01);
    tp.association_gate = declare_parameter<double>("association_gate", 0.5);
    tp.initial_velocity_variance =
      declare_parameter<double>("initial_velocity_variance", 1.0);
    tp.confirm_count = declare_parameter<int>("confirm_count", 3);
    tp.drop_count = declare_parameter<int>("drop_count", 3);
    tp.max_tracks = static_cast<std::size_t>(declare_parameter<int>("max_tracks", 10));

    // Validate early; a bad value fails configure rather than corrupting the loop.
    const bool ok =
      tracking_frame_.size() > 0 &&
      cluster_gap_ > 0.0 &&
      min_cluster_points_ >= 1 &&
      max_clusters_ >= 1 &&
      max_cluster_radius_ >= 0.0 &&
      min_detection_range_ >= 0.0 &&
      max_detection_range_ >= 0.0 &&
      transform_timeout_ >= 0.0 &&
      tp.process_noise >= 0.0 &&
      tp.measurement_noise > 0.0 &&
      tp.association_gate > 0.0 &&
      tp.initial_velocity_variance >= 0.0 &&
      tp.confirm_count >= 1 &&
      tp.drop_count >= 0 &&
      tp.max_tracks >= 1;
    if (!ok) {
      RCLCPP_ERROR(get_logger(), "Invalid parameter(s); refusing to configure.");
      return CallbackReturn::FAILURE;
    }

    tracker_ = std::make_unique<Tracker>(tp);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    // spin_thread = true: a dedicated thread services /tf and /tf_static so the
    // scan callback can lookupTransform at the scan stamp with a timeout (without
    // it tf2 logs a per-scan error and the timed lookup always fails).
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

    obstacle_pub_ = create_publisher<prox_mpc_msgs::msg::ObstacleArray>(
      output_topic_, rclcpp::QoS(rclcpp::KeepLast(5)));

    RCLCPP_INFO(
      get_logger(),
      "Configured prox_mpc_obstacle_tracker (scan '%s' -> '%s', tracking frame '%s', "
      "cluster_gap=%.2f, gate=%.2f, confirm=%d, drop=%d).",
      scan_topic_.c_str(), output_topic_.c_str(), tracking_frame_.c_str(), cluster_gap_,
      tp.association_gate, tp.confirm_count, tp.drop_count);
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "on_configure failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }
}

ObstacleTrackerNode::CallbackReturn ObstacleTrackerNode::on_activate(
  const rclcpp_lifecycle::State & state)
{
  try {
    LifecycleNode::on_activate(state);  // activates managed entities (the publisher)
    if (tracker_) {tracker_->reset();}  // start the velocity estimate fresh

    // Depth 1 (large-sensor profile): keep only the freshest scan and minimize
    // memory, rather than the SensorDataQoS default of KeepLast(5).
    rclcpp::SubscriptionOptions scan_options;
    scan_options.callback_group = scan_callback_group_;
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&ObstacleTrackerNode::scanCallback, this, std::placeholders::_1),
      scan_options);

    active_.store(true);
    RCLCPP_INFO(get_logger(), "Activated prox_mpc_obstacle_tracker.");
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "on_activate failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }
}

ObstacleTrackerNode::CallbackReturn ObstacleTrackerNode::on_deactivate(
  const rclcpp_lifecycle::State & state)
{
  try {
    active_.store(false);
    scan_sub_.reset();
    LifecycleNode::on_deactivate(state);  // deactivates the publisher (fail-safe: stop output)
    RCLCPP_INFO(get_logger(), "Deactivated prox_mpc_obstacle_tracker.");
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "on_deactivate failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
}

ObstacleTrackerNode::CallbackReturn ObstacleTrackerNode::on_cleanup(
  const rclcpp_lifecycle::State & /*state*/)
{
  teardown();
  RCLCPP_INFO(get_logger(), "Cleaned up prox_mpc_obstacle_tracker.");
  return CallbackReturn::SUCCESS;
}

ObstacleTrackerNode::CallbackReturn ObstacleTrackerNode::on_shutdown(
  const rclcpp_lifecycle::State & /*state*/)
{
  teardown();
  RCLCPP_INFO(get_logger(), "Shut down prox_mpc_obstacle_tracker.");
  return CallbackReturn::SUCCESS;
}

void ObstacleTrackerNode::teardown()
{
  active_.store(false);
  scan_sub_.reset();
  // Serialize against any in-flight scan callback (MultiThreadedExecutor) before
  // releasing the perception state it reads.
  std::lock_guard<std::mutex> lock(state_mutex_);
  obstacle_pub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  tracker_.reset();
}

void ObstacleTrackerNode::scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  // Hold the state mutex for the whole callback so teardown cannot release
  // tracker_/obstacle_pub_/tf_buffer_ from another thread mid-use.
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!active_.load() || !tracker_ || !tf_buffer_) {return;}

  const double range_min = std::max(static_cast<double>(msg->range_min), min_detection_range_);
  double range_max = static_cast<double>(msg->range_max);
  if (max_detection_range_ > 0.0) {range_max = std::min(range_max, max_detection_range_);}
  if (!(range_max > range_min)) {return;}

  const std::vector<Point2> points = scan_to_points(
    msg->ranges, msg->angle_min, msg->angle_increment, range_min, range_max);
  const std::vector<Cluster> local = cluster_points(
    points, cluster_gap_, static_cast<std::size_t>(min_cluster_points_),
    static_cast<std::size_t>(max_clusters_), max_cluster_radius_);

  // One rigid transform for the whole scan (all returns share frame and stamp).
  // A TF gap means we cannot place the obstacles in the tracking frame, so skip
  // this scan; the controller's staleness fallback covers the gap.
  double tx = 0.0;
  double ty = 0.0;
  double tyaw = 0.0;
  if (msg->header.frame_id != tracking_frame_) {
    try {
      const geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
        tracking_frame_, msg->header.frame_id, rclcpp::Time(msg->header.stamp),
        rclcpp::Duration::from_seconds(transform_timeout_));
      tx = tf.transform.translation.x;
      ty = tf.transform.translation.y;
      tyaw = quat_yaw(tf.transform.rotation);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TF %s <- %s unavailable (%s); skipping scan.",
        tracking_frame_.c_str(), msg->header.frame_id.c_str(), ex.what());
      return;
    }
  }
  const double ct = std::cos(tyaw);
  const double st = std::sin(tyaw);

  std::vector<Cluster> measurements;
  measurements.reserve(local.size());
  for (const Cluster & c : local) {
    Cluster m = c;
    m.x = tx + ct * c.x - st * c.y;
    m.y = ty + st * c.x + ct * c.y;
    measurements.push_back(m);
  }

  tracker_->update(measurements, rclcpp::Time(msg->header.stamp).seconds());

  prox_mpc_msgs::msg::ObstacleArray out;
  out.header.stamp = msg->header.stamp;
  out.header.frame_id = tracking_frame_;
  for (const Track & t : tracker_->tracks()) {
    if (!t.confirmed) {continue;}
    prox_mpc_msgs::msg::Obstacle o;
    o.id = t.id;
    o.position.x = t.state(0);
    o.position.y = t.state(1);
    o.position.z = 0.0;
    o.velocity.x = t.state(2);
    o.velocity.y = t.state(3);
    o.velocity.z = 0.0;
    o.radius = t.radius;
    o.position_covariance = {t.cov(0, 0), t.cov(0, 1), t.cov(1, 0), t.cov(1, 1)};
    o.velocity_covariance = {t.cov(2, 2), t.cov(2, 3), t.cov(3, 2), t.cov(3, 3)};
    out.obstacles.push_back(o);
  }
  obstacle_pub_->publish(out);
}

}  // namespace prox_mpc_obstacle_tracker

// Register as a composable component so the tracker can be loaded into a shared
// process (MultiThreadedExecutor) in addition to the standalone executable.
RCLCPP_COMPONENTS_REGISTER_NODE(prox_mpc_obstacle_tracker::ObstacleTrackerNode)
