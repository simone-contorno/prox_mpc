// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Unit tests for the ObstacleTrackerNode ROS lifecycle wrapper (the algorithm-free
// integration around the ROS-tested clustering/tracker core). They cover the
// lifecycle ladder and teardown, the on_configure validation-failure branch, and
// the end-to-end scan path (publish a synthetic scan in the tracking frame, so no
// TF is needed, and observe the published ObstacleArray). The clustering and
// Kalman tracking themselves are covered by test_clustering / test_tracker.

#include <chrono>
#include <cmath>
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

// Every log_level keyword (and an unrecognized value) is accepted at configure.
TEST(ObstacleTrackerNode, AcceptsAllLogLevels)
{
  for (const std::string level : {"debug", "info", "warn", "error", "fatal", "bogus"}) {
    auto node = makeNode({rclcpp::Parameter("log_level", level)});
    EXPECT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
