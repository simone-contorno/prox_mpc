// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Unit tests for the demo SimulationNode. The node logic is in the header, so a
// test subclass drives step() directly (no wall-timer spin) and reads the
// simulated pose: this exercises the convergence/finiteness gate, the pose
// advance, and the goal-heading unwrap, plus the parameter-sizing validation.
// The solver-failure deceleration ramp is covered through its helper, which the
// subclass re-exposes; the gate itself cannot be forced from parameters.

#include <algorithm>
#include <cmath>
#include <limits>
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
  using SimulationNode::brakeToward;
  const VectorXd & pose() const {return pose_;}
  double last_cmd_v() const {return last_cmd_v_;}
  double last_cmd_w() const {return last_cmd_w_;}
  double a_dec_lin() const {return a_dec_lin_;}
  double a_dec_ang() const {return a_dec_ang_;}
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

// The unicycle (3-state) model also steps forward without going non-finite.
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

// The solver-failure ramp walks a command toward zero by one deceleration step
// per cycle, keeps its sign, and clamps at zero rather than overshooting into
// reverse (a command smaller than one step lands exactly on zero).
TEST(SimulationNode, BrakeTowardStepsTowardZeroAndClamps)
{
  using Node = TestableSimulationNode;
  EXPECT_DOUBLE_EQ(Node::brakeToward(1.0, 0.5, 0.1), 0.95);
  EXPECT_DOUBLE_EQ(Node::brakeToward(-1.0, 0.5, 0.1), -0.95);
  EXPECT_DOUBLE_EQ(Node::brakeToward(0.0, 0.5, 0.1), 0.0);
  EXPECT_DOUBLE_EQ(Node::brakeToward(0.01, 0.5, 0.1), 0.0);
  EXPECT_DOUBLE_EQ(Node::brakeToward(-0.01, 0.5, 0.1), 0.0);
}

// A non-finite retained command must not propagate into the published twist.
TEST(SimulationNode, BrakeTowardZerosNonFiniteInput)
{
  using Node = TestableSimulationNode;
  constexpr double nan_v = std::numeric_limits<double>::quiet_NaN();
  constexpr double inf_v = std::numeric_limits<double>::infinity();
  EXPECT_DOUBLE_EQ(Node::brakeToward(nan_v, 0.5, 0.1), 0.0);
  EXPECT_DOUBLE_EQ(Node::brakeToward(inf_v, 0.5, 0.1), 0.0);
  EXPECT_DOUBLE_EQ(Node::brakeToward(-inf_v, 0.5, 0.1), 0.0);
}

// The node retains what it commanded (that is the ramp's starting point, since
// this node is its own plant) and clears it once the final goal is latched, so a
// later ramp cannot brake off a stale command.
TEST(SimulationNode, RetainsCommandAndClearsItOnArrival)
{
  auto node = makeNode();
  EXPECT_GT(node->a_dec_lin(), 0.0);
  EXPECT_GT(node->a_dec_ang(), 0.0);

  double max_cmd_v = 0.0;
  for (int i = 0; i < 60; ++i) {
    node->step();
    max_cmd_v = std::max(max_cmd_v, std::abs(node->last_cmd_v()));
  }

  EXPECT_GT(max_cmd_v, 0.1);                                  // it did command motion
  EXPECT_LE(std::hypot(node->pose()(0) - 2.0, node->pose()(1)), 0.25);
  EXPECT_DOUBLE_EQ(node->last_cmd_v(), 0.0);                  // cleared at the goal
  EXPECT_DOUBLE_EQ(node->last_cmd_w(), 0.0);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
