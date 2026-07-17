// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Unit tests for the demo SimulationNode. The node logic is in the header, so a
// test subclass drives step() directly (no wall-timer spin) and reads the
// simulated pose: this exercises the convergence/finiteness gate, the pose
// advance, and the goal-heading unwrap, plus the parameter-sizing validation.

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "prox_mpc_demo/simulation_node.hpp"

namespace
{

// Exposes step() and the simulated pose (white-box; the production node keeps
// them protected so only the wall timer drives the loop).
class TestableSimulationNode : public SimulationNode
{
public:
  explicit TestableSimulationNode(const rclcpp::NodeOptions & options)
  : SimulationNode(options) {}
  using SimulationNode::step;
  const VectorXd & pose() const {return pose_;}
};

// A short-horizon node driving toward a goal 2 m ahead, with periodic logging off.
std::shared_ptr<TestableSimulationNode> makeNode(
  const std::vector<rclcpp::Parameter> & overrides = {})
{
  std::vector<rclcpp::Parameter> params{
    rclcpp::Parameter("model", std::string("bicycle")),
    rclcpp::Parameter("np", 10),
    rclcpp::Parameter("nc", 10),
    rclcpp::Parameter("dt", 0.1),
    rclcpp::Parameter("goal_x", 2.0),
    rclcpp::Parameter("goal_y", 0.0),
    rclcpp::Parameter("v_ref", 1.0),
    rclcpp::Parameter("report_period", 0),
  };
  params.insert(params.end(), overrides.begin(), overrides.end());
  rclcpp::NodeOptions opts;
  opts.parameter_overrides(params);
  return std::make_shared<TestableSimulationNode>(opts);
}

}  // namespace

// Closed-loop stepping advances the simulated pose toward the goal and keeps it
// finite (the convergence/finiteness gate folds only good iterates into pose_).
TEST(SimulationNode, StepAdvancesPoseTowardGoal)
{
  auto node = makeNode();
  ASSERT_NEAR(node->pose()(0), 0.0, 1e-12);

  for (int i = 0; i < 30; ++i) {
    node->step();
  }

  EXPECT_TRUE(node->pose().allFinite());
  EXPECT_GT(node->pose()(0), 0.5);     // moved forward toward goal_x = 2
  EXPECT_LT(node->pose()(0), 2.5);     // did not run away past the goal
  EXPECT_NEAR(node->pose()(1), 0.0, 0.5);
}

// The unicycle (unicycle, 3-state) model also steps forward without going non-finite.
TEST(SimulationNode, UnicycleModelSteps)
{
  auto node = makeNode({rclcpp::Parameter("model", std::string("unicycle"))});
  for (int i = 0; i < 30; ++i) {
    node->step();
  }
  EXPECT_TRUE(node->pose().allFinite());
  EXPECT_GT(node->pose()(0), 0.5);
}

// With obstacle avoidance enabled the node sizes the obstacle slots and still
// drives forward past the fixed world obstacle toward the goal.
TEST(SimulationNode, ObstacleEnabledSteps)
{
  auto node = makeNode(
  {
    rclcpp::Parameter("obstacle_enable", true),
    rclcpp::Parameter("max_obstacles", 1),
    rclcpp::Parameter("d_safe", 1.0),
    rclcpp::Parameter("obs_x", 2.5),
    rclcpp::Parameter("obs_y", 0.6),
    rclcpp::Parameter("goal_x", 5.0),
  });
  for (int i = 0; i < 30; ++i) {
    node->step();
  }
  EXPECT_TRUE(node->pose().allFinite());
  EXPECT_GT(node->pose()(0), 0.5);     // advances toward the goal past the obstacle
}

// Structurally invalid sizing throws from the constructor rather than wrapping
// into an astronomical allocation.
TEST(SimulationNode, InvalidSizingThrows)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({rclcpp::Parameter("np", 0)});
  EXPECT_THROW(std::make_shared<TestableSimulationNode>(opts), std::invalid_argument);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
