// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Integration tests for the ObstacleTrackerNode ROS lifecycle wrapper (the
// algorithm-free integration around the ROS-free clustering/tracker core). They
// cover the lifecycle ladder and teardown, the on_configure validation-failure
// branch, and the end-to-end scan path (publish a synthetic scan in the tracking
// frame, so no TF is needed, and observe the published ObstacleArray). The
// clustering and tracking filters themselves - IMM is the default tracking path,
// not plain Kalman - are covered by test_clustering / test_imm_filter /
// test_tracker.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <prox_mpc_msgs/msg/obstacle_array.hpp>

#include "prox_mpc_obstacle_tracker/obstacle_tracker_node.hpp"

using lifecycle_msgs::msg::State;
using prox_mpc_obstacle_tracker::ObstacleTrackerNode;

namespace
{

// A tracker node with sensible test defaults plus the given overrides.
std::shared_ptr<ObstacleTrackerNode> makeNode(
  const std::vector<rclcpp::Parameter> & overrides = {})
{
  std::vector<rclcpp::Parameter> params{
    rclcpp::Parameter("tracking_frame", std::string("odom")),
    rclcpp::Parameter("scan_topic", std::string("scan")),
    rclcpp::Parameter("output_topic", std::string("tracked_obstacles")),
    rclcpp::Parameter("cluster_gap", 0.3),
    rclcpp::Parameter("min_cluster_points", 3),
    rclcpp::Parameter("max_cluster_radius", 0.0),
    rclcpp::Parameter("association_gate", 0.5),
    rclcpp::Parameter("confirm_count", 3),
  };
  params.insert(params.end(), overrides.begin(), overrides.end());
  rclcpp::NodeOptions opts;
  opts.parameter_overrides(params);
  return std::make_shared<ObstacleTrackerNode>(opts);
}

// A scan in the "odom" frame with one ~2 m cluster of 6 contiguous returns in
// front of the sensor; every other beam is a non-return (infinite range).
sensor_msgs::msg::LaserScan makeScan(const rclcpp::Time & stamp)
{
  sensor_msgs::msg::LaserScan s;
  s.header.frame_id = "odom";
  s.header.stamp = stamp;
  s.angle_min = -static_cast<float>(M_PI);
  s.angle_max = static_cast<float>(M_PI);
  s.angle_increment = static_cast<float>(2.0 * M_PI / 360.0);
  s.range_min = 0.1f;
  s.range_max = 10.0f;
  s.ranges.assign(360, std::numeric_limits<float>::infinity());
  for (int i = 178; i <= 183; ++i) {
    s.ranges[static_cast<std::size_t>(i)] = 2.0f;
  }
  return s;
}

}  // namespace

// The full lifecycle ladder transitions cleanly and teardown reaches Finalized.
TEST(ObstacleTrackerNode, LifecycleLadder)
{
  auto node = makeNode();
  EXPECT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(node->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(node->shutdown().id(), State::PRIMARY_STATE_FINALIZED);
}

// An invalid parameter fails on_configure, leaving the node Unconfigured.
TEST(ObstacleTrackerNode, ConfigureFailsOnInvalidParameter)
{
  auto node = makeNode({rclcpp::Parameter("cluster_gap", -1.0)});   // must be > 0
  EXPECT_EQ(node->configure().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

// max_tracks is validated as a signed int: a non-positive value fails configure
// instead of wrapping to a huge std::size_t and removing the track cap.
TEST(ObstacleTrackerNode, ConfigureFailsOnNonPositiveMaxTracks)
{
  for (const int max_tracks : {0, -1, -10}) {
    auto node = makeNode({rclcpp::Parameter("max_tracks", max_tracks)});
    EXPECT_EQ(node->configure().id(), State::PRIMARY_STATE_UNCONFIGURED)
      << "max_tracks = " << max_tracks;
  }
}

// The detection-range pair is validated as a pair: a reversed (or degenerate)
// pair fails configure instead of activating a tracker that skips every scan.
TEST(ObstacleTrackerNode, ConfigureFailsOnMisorderedDetectionRange)
{
  for (const double min_range : {4.0, 3.0}) {   // reversed, then equal
    auto node = makeNode(
    {
      rclcpp::Parameter("min_detection_range", min_range),
      rclcpp::Parameter("max_detection_range", 3.0),
    });
    EXPECT_EQ(node->configure().id(), State::PRIMARY_STATE_UNCONFIGURED)
      << "min_detection_range = " << min_range;
  }
}

// max_detection_range 0.0 means "no cap" (the scan's own range_max applies), so
// it stays valid whatever the lower cutoff is.
TEST(ObstacleTrackerNode, ConfiguresWithUncappedDetectionRange)
{
  auto node = makeNode(
  {
    rclcpp::Parameter("min_detection_range", 4.0),
    rclcpp::Parameter("max_detection_range", 0.0),
  });
  EXPECT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  node->shutdown();   // exercise shutdown straight from the inactive state
}

// Every log_level keyword (and an unrecognized value) is accepted at configure.
TEST(ObstacleTrackerNode, AcceptsAllLogLevels)
{
  for (const std::string level : {"debug", "info", "warn", "error", "fatal", "bogus"}) {
    auto node = makeNode({rclcpp::Parameter("log_level", level)});
    EXPECT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
    node->shutdown();   // exercise shutdown straight from the inactive state
  }
}

// shutdown() straight from Active runs the teardown ladder and finalizes.
TEST(ObstacleTrackerNode, ShutdownFromActiveFinalizes)
{
  auto node = makeNode();
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(node->shutdown().id(), State::PRIMARY_STATE_FINALIZED);
}

// End-to-end: a synthetic scan in the tracking frame drives the scan callback,
// and a confirmed track is published on the output topic.
TEST(ObstacleTrackerNode, ProcessesScanAndPublishesConfirmedTrack)
{
  auto node = makeNode();
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);

  auto helper = std::make_shared<rclcpp::Node>("tracker_test_helper");
  auto scan_pub = helper->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS());

  int arrays_received = 0;
  std::size_t max_obstacles_seen = 0;
  auto sub = helper->create_subscription<prox_mpc_msgs::msg::ObstacleArray>(
    "tracked_obstacles", rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
    [&](prox_mpc_msgs::msg::ObstacleArray::SharedPtr msg) {
      ++arrays_received;
      max_obstacles_seen = std::max(max_obstacles_seen, msg->obstacles.size());
    });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(helper);

  const rclcpp::Time base(1000, 0, RCL_ROS_TIME);
  for (int i = 0; i < 8 && max_obstacles_seen == 0; ++i) {
    scan_pub->publish(makeScan(base + rclcpp::Duration::from_seconds(0.1 * i)));
    for (int s = 0; s < 5; ++s) {
      exec.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  EXPECT_GT(arrays_received, 0);          // the scan callback ran and published
  EXPECT_GE(max_obstacles_seen, 1u);      // the cluster confirmed into a track

  exec.remove_node(helper);
  exec.remove_node(node->get_node_base_interface());
  node->shutdown();
}

// With the centroid->centre correction enabled, the published position is the
// cluster centroid pushed away from the sensor along its ray by
// gain * enclosing radius (computed here from the same synthetic returns).
TEST(ObstacleTrackerNode, CentroidOffsetPushesPositionAwayFromSensor)
{
  auto node = makeNode({rclcpp::Parameter("cluster_center_offset_gain", 0.5)});
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);

  auto helper = std::make_shared<rclcpp::Node>("tracker_offset_test_helper");
  auto scan_pub = helper->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS());

  double seen_x = 0.0;
  double seen_y = 0.0;
  bool seen = false;
  auto sub = helper->create_subscription<prox_mpc_msgs::msg::ObstacleArray>(
    "tracked_obstacles", rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
    [&](prox_mpc_msgs::msg::ObstacleArray::SharedPtr msg) {
      if (!msg->obstacles.empty()) {
        seen = true;
        seen_x = msg->obstacles[0].position.x;
        seen_y = msg->obstacles[0].position.y;
      }
    });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(helper);

  const rclcpp::Time base(1000, 0, RCL_ROS_TIME);
  for (int i = 0; i < 10 && !seen; ++i) {
    scan_pub->publish(makeScan(base + rclcpp::Duration::from_seconds(0.1 * i)));
    for (int s = 0; s < 5; ++s) {
      exec.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  ASSERT_TRUE(seen);

  // Expected: centroid and enclosing radius of the makeScan returns (beams
  // 178..183 at 2.0 m), then the 0.5 * radius push along the sensor ray.
  const sensor_msgs::msg::LaserScan s = makeScan(base);
  double sx = 0.0;
  double sy = 0.0;
  std::vector<std::array<double, 2>> pts;
  for (int i = 178; i <= 183; ++i) {
    const double a = s.angle_min + i * s.angle_increment;
    pts.push_back({2.0 * std::cos(a), 2.0 * std::sin(a)});
    sx += pts.back()[0];
    sy += pts.back()[1];
  }
  const double cx = sx / 6.0;
  const double cy = sy / 6.0;
  double r2 = 0.0;
  for (const auto & p : pts) {
    r2 = std::max(r2, (p[0] - cx) * (p[0] - cx) + (p[1] - cy) * (p[1] - cy));
  }
  const double scale = 1.0 + 0.5 * std::sqrt(r2) / std::hypot(cx, cy);
  EXPECT_NEAR(seen_x, cx * scale, 0.02);   // KF settles onto the constant measurement
  EXPECT_NEAR(seen_y, cy * scale, 0.02);
  EXPECT_GT(std::hypot(seen_x, seen_y), std::hypot(cx, cy));   // pushed away, not toward

  exec.remove_node(helper);
  exec.remove_node(node->get_node_base_interface());
  node->shutdown();
}

// Every out-of-range IMM/prediction parameter fails on_configure (fail-closed):
// stay probabilities strictly inside (0, 1), non-negative CTRV noise, positive
// omega variance, prediction_steps in [0, 100], positive prediction_dt.
TEST(ObstacleTrackerNode, ConfigureFailsOnOutOfRangeImmParameters)
{
  const std::vector<rclcpp::Parameter> bad{
    rclcpp::Parameter("cluster_center_offset_gain", -0.1),
    rclcpp::Parameter("cluster_center_offset_gain", 1.5),
    rclcpp::Parameter("imm_p_cv_stay", 1.5),
    rclcpp::Parameter("imm_p_cv_stay", 0.0),
    rclcpp::Parameter("imm_p_ctrv_stay", 1.0),
    rclcpp::Parameter("ctrv_process_noise_accel", -0.1),
    rclcpp::Parameter("ctrv_process_noise_yaw_accel", -1.0),
    rclcpp::Parameter("ctrv_init_omega_variance", 0.0),
    rclcpp::Parameter("prediction_steps", 101),
    rclcpp::Parameter("prediction_steps", -1),
    rclcpp::Parameter("prediction_dt", 0.0),
  };
  for (const auto & param : bad) {
    auto node = makeNode({param});
    EXPECT_EQ(node->configure().id(), State::PRIMARY_STATE_UNCONFIGURED)
      << param.get_name() << " = " << param.value_to_string();
  }
}

// A published Obstacle carries the sampled prediction: prediction_steps
// samples spaced by prediction_dt, planar (z = 0).
TEST(ObstacleTrackerNode, PublishedObstacleCarriesPredictionSamples)
{
  const int prediction_steps = 5;
  const double prediction_dt = 0.2;
  auto node = makeNode(
  {
    rclcpp::Parameter("prediction_steps", prediction_steps),
    rclcpp::Parameter("prediction_dt", prediction_dt),
  });
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);

  auto helper = std::make_shared<rclcpp::Node>("tracker_prediction_test_helper");
  auto scan_pub = helper->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS());

  prox_mpc_msgs::msg::Obstacle tracked;
  bool got_obstacle = false;
  auto sub = helper->create_subscription<prox_mpc_msgs::msg::ObstacleArray>(
    "tracked_obstacles", rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
    [&](prox_mpc_msgs::msg::ObstacleArray::SharedPtr msg) {
      if (!msg->obstacles.empty()) {
        tracked = msg->obstacles.front();
        got_obstacle = true;
      }
    });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(helper);

  const rclcpp::Time base(1000, 0, RCL_ROS_TIME);
  for (int i = 0; i < 8 && !got_obstacle; ++i) {
    scan_pub->publish(makeScan(base + rclcpp::Duration::from_seconds(0.1 * i)));
    for (int s = 0; s < 5; ++s) {
      exec.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  ASSERT_TRUE(got_obstacle);
  ASSERT_EQ(tracked.predicted_positions.size(), static_cast<std::size_t>(prediction_steps));
  EXPECT_EQ(tracked.prediction_dt, prediction_dt);   // exact: same double end to end
  for (const auto & sample : tracked.predicted_positions) {
    EXPECT_TRUE(std::isfinite(sample.x));
    EXPECT_TRUE(std::isfinite(sample.y));
    EXPECT_EQ(sample.z, 0.0);                        // planar tracker: z unused
  }

  exec.remove_node(helper);
  exec.remove_node(node->get_node_base_interface());
  node->shutdown();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
