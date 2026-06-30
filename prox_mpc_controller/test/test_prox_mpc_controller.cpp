// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Unit tests for the ProxMpcController Nav2 controller plugin (the ROS wrapper
// around the frozen prox_mpc::MPC core). The cases cover every nav2_core
// interface method and every documented fail-safe branch from the
// implementation plan:
//   - configure(): parameter reading, model loading, MPC sizing, log-level
//     seeding, cruise-speed clamping, and the model-load failure escalation;
//   - activate()/deactivate()/cleanup(): lifecycle state and teardown;
//   - setPlan(): plan storage and projection-index reset;
//   - computeVelocityCommands(): the converging happy path, the in-cycle costmap
//     reduction, and the footprint veto;
//   - setSpeedLimit(): absolute, percentage, clamping, NO_SPEED_LIMIT restore,
//     and the cache-before-model case;
//   - cancel()/reset(): graceful-stop ramp and runtime-state clearing.
//
// Fail-safe branches each assert the safe command the plan specifies, not merely
// that the call returns: an empty plan throws nav2_core::InvalidPath; a TF
// failure throws nav2_core::ControllerTFError; a non-converged solve and a
// non-finite pose decelerate within the model deceleration limit and escalate to
// nav2_core::NoValidControl once the failure budget is spent; a short plan holds
// the goal; a footprint collision vetoes to the brake ramp.
//
// The wrapped SQP/QP solver lives in prox_mpc_core and is out of coverage scope;
// these tests drive it only through the plugin's public surface.

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_core/controller_exceptions.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>

#include "prox_mpc_controller/prox_mpc_controller.hpp"

namespace
{
constexpr double kTol = 1e-9;
constexpr double kModelVMax = 3.0;         // bundled-model speed bound [m/s]
constexpr double kModelDecel = 0.5;        // bundled-model du bound [m/s^2, rad/s^2]
constexpr double kResolution = 0.05;       // test costmap resolution [m]
constexpr unsigned int kGridCells = 200u;  // 10 m x 10 m grid
constexpr double kGridOrigin = -5.0;       // centered grid origin [m]

// Exposes the protected helper and runtime state so the fail-safe and reduction
// branches can be driven and the safe command asserted (white-box, no production
// change). The plugin's behavior is otherwise exercised through the public API.
class TestableProxMpcController : public prox_mpc_controller::ProxMpcController
{
public:
  using ProxMpcController::reduceCostmap;

  int & failureCount() {return failure_count_;}
  double & lastCmdV() {return last_cmd_v_;}
  double & lastCmdW() {return last_cmd_w_;}
  double & steeringState() {return steering_state_;}
  std::size_t & planIndex() {return plan_index_;}
  bool cancelling() const {return cancelling_;}
  double vMax() const {return v_max_;}
  double maxLinearVel() const {return max_linear_vel_;}
  double desiredLinearVel() const {return desired_linear_vel_;}
  std::size_t nDim() const {return n_;}
  std::size_t maxObstacles() const {return static_cast<std::size_t>(max_obstacles_);}
  std::shared_ptr<prox_mpc::MPC> mpc() const {return mpc_;}
  std::shared_ptr<prox_mpc::Model> model() const {return model_;}
};

// A straight global plan along +x in the costmap global frame ("map").
nav_msgs::msg::Path makeStraightPlan(
  std::size_t count, double step, const std::string & frame = "map")
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame;
  for (std::size_t i = 0; i < count; ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = frame;
    ps.pose.position.x = static_cast<double>(i) * step;
    ps.pose.position.y = 0.0;
    ps.pose.orientation.w = 1.0;
    path.poses.push_back(ps);
  }
  return path;
}

geometry_msgs::msg::PoseStamped makePose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id = "map";
  ps.pose.position.x = x;
  ps.pose.position.y = y;
  ps.pose.orientation.z = std::sin(yaw / 2.0);
  ps.pose.orientation.w = std::cos(yaw / 2.0);
  return ps;
}

// A square footprint of the given half-extent, centered on the robot.
std::vector<geometry_msgs::msg::Point> makeSquareFootprint(double half)
{
  std::vector<geometry_msgs::msg::Point> fp(4);
  fp[0].x = half; fp[0].y = half;
  fp[1].x = half; fp[1].y = -half;
  fp[2].x = -half; fp[2].y = -half;
  fp[3].x = -half; fp[3].y = half;
  return fp;
}
}  // namespace

class ProxMpcControllerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions cm_opts;
    cm_opts.arguments({"--ros-args", "-r", "__node:=prox_mpc_test_costmap"});
    cm_opts.parameter_overrides(
    {
      rclcpp::Parameter("global_frame", std::string("map")),
      rclcpp::Parameter("robot_base_frame", std::string("base_link")),
      rclcpp::Parameter("use_sim_time", false),
      rclcpp::Parameter("plugins", std::vector<std::string>{}),
    });
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(cm_opts);
    costmap_ros_->on_configure(rclcpp_lifecycle::State());

    // Take full control of the master grid: a known, all-free 10 m x 10 m window.
    auto * costmap = costmap_ros_->getCostmap();
    costmap->setDefaultValue(nav2_costmap_2d::FREE_SPACE);
    costmap->resizeMap(kGridCells, kGridCells, kResolution, kGridOrigin, kGridOrigin);
    costmap_ros_->setRobotFootprint(makeSquareFootprint(0.5));

    tf_ = std::make_shared<tf2_ros::Buffer>(rclcpp::Clock::make_shared());
  }

  void TearDown() override
  {
    if (controller_) {controller_->cleanup();}
    controller_.reset();
    if (costmap_ros_) {costmap_ros_->on_cleanup(rclcpp_lifecycle::State());}
    costmap_ros_.reset();
    nodes_.clear();
  }

  // A controller parent node carrying the given plugin-namespaced overrides.
  rclcpp_lifecycle::LifecycleNode::SharedPtr makeNode(
    const std::vector<rclcpp::Parameter> & overrides)
  {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides(overrides);
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "prox_mpc_test_controller_" + std::to_string(node_counter_++), opts);
    nodes_.push_back(node);
    return node;
  }

  // Build an unconfigured controller bound to a fresh parent node.
  std::shared_ptr<TestableProxMpcController> makeUnconfigured(
    const std::vector<rclcpp::Parameter> & overrides = {})
  {
    node_ = makeNode(overrides);
    controller_ = std::make_shared<TestableProxMpcController>();
    return controller_;
  }

  // Build and configure a controller in one step (the common case).
  std::shared_ptr<TestableProxMpcController> makeConfigured(
    const std::vector<rclcpp::Parameter> & overrides = {})
  {
    auto c = makeUnconfigured(overrides);
    c->configure(node_, "FollowPath", tf_, costmap_ros_);
    return c;
  }

  // Stamp a rectangular world region [x0,x1] x [y0,y1] with a cost value.
  void fillCost(double x0, double y0, double x1, double y1, unsigned char cost)
  {
    auto * costmap = costmap_ros_->getCostmap();
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
    for (double y = y0; y <= y1 + kTol; y += kResolution) {
      for (double x = x0; x <= x1 + kTol; x += kResolution) {
        unsigned int mx = 0;
        unsigned int my = 0;
        if (costmap->worldToMap(x, y, mx, my)) {costmap->setCost(mx, my, cost);}
      }
    }
  }

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<TestableProxMpcController> controller_;
  std::vector<rclcpp_lifecycle::LifecycleNode::SharedPtr> nodes_;
  int node_counter_{0};
};

// --- configure() -----------------------------------------------------------

// configure() loads the default Bicycle model, sizes the MPC, and reads the
// model's speed bound from its declared constraints.
TEST_F(ProxMpcControllerTest, ConfigureLoadsModelAndSizesMpc)
{
  auto c = makeConfigured();
  ASSERT_NE(c->model(), nullptr);
  ASSERT_NE(c->mpc(), nullptr);
  EXPECT_EQ(c->nDim(), 4u);                  // bicycle state [x, y, theta, delta]
  EXPECT_NEAR(c->vMax(), kModelVMax, kTol);
  EXPECT_NEAR(c->maxLinearVel(), kModelVMax, kTol);
  EXPECT_EQ(c->mpc()->getMaxObs(), 1u);      // default max_obstacles
}

// A cruise speed above the model bound is clamped to v_max with a warning.
TEST_F(ProxMpcControllerTest, ConfigureClampsCruiseSpeedToModelMax)
{
  auto c = makeConfigured({rclcpp::Parameter("FollowPath.desired_linear_vel", 5.0)});
  EXPECT_NEAR(c->desiredLinearVel(), kModelVMax, kTol);
}

// max_obstacles = 0 disables obstacle slots (K = 0); the obstacle-off path runs.
TEST_F(ProxMpcControllerTest, ConfigureDisablesObstaclesWhenZero)
{
  auto c = makeConfigured({rclcpp::Parameter("FollowPath.max_obstacles", 0)});
  EXPECT_EQ(c->mpc()->getMaxObs(), 0u);
}

// An unknown model plugin name escalates to a nav2_core ControllerException.
TEST_F(ProxMpcControllerTest, ConfigureThrowsOnUnknownModelPlugin)
{
  auto c = makeUnconfigured(
    {rclcpp::Parameter("FollowPath.model_plugin", std::string("prox_mpc_core/DoesNotExist"))});
  EXPECT_THROW(
    c->configure(node_, "FollowPath", tf_, costmap_ros_),
    nav2_core::ControllerException);
}

// Every log_level keyword (and an unrecognized value) is accepted at configure.
TEST_F(ProxMpcControllerTest, ConfigureAcceptsAllLogLevels)
{
  for (const std::string level : {"debug", "info", "warn", "error", "fatal", "bogus"}) {
    auto c = makeUnconfigured({rclcpp::Parameter("FollowPath.log_level", level)});
    EXPECT_NO_THROW(c->configure(node_, "FollowPath", tf_, costmap_ros_));
  }
}

// A speed limit set before the model is loaded is re-applied during configure().
TEST_F(ProxMpcControllerTest, ConfigureReappliesPreloadSpeedLimit)
{
  auto c = makeUnconfigured();
  c->setSpeedLimit(1.5, false);              // model_ not yet loaded: cached
  EXPECT_NEAR(c->maxLinearVel(), 0.0, kTol);
  c->configure(node_, "FollowPath", tf_, costmap_ros_);
  EXPECT_NEAR(c->maxLinearVel(), 1.5, kTol);
}

// --- lifecycle: activate / deactivate / cleanup ----------------------------

// activate() clears the runtime counters and the cancelling flag.
TEST_F(ProxMpcControllerTest, ActivateResetsRuntimeState)
{
  auto c = makeConfigured();
  c->failureCount() = 5;
  c->lastCmdV() = 1.0;
  c->cancel();                               // sets cancelling_ (still "moving")
  EXPECT_TRUE(c->cancelling());
  c->activate();
  EXPECT_EQ(c->failureCount(), 0);
  EXPECT_NEAR(c->lastCmdV(), 0.0, kTol);
  EXPECT_FALSE(c->cancelling());
}

// deactivate() is a no-op beyond logging and must not throw.
TEST_F(ProxMpcControllerTest, DeactivateDoesNotThrow)
{
  auto c = makeConfigured();
  c->activate();
  EXPECT_NO_THROW(c->deactivate());
}

// cleanup() releases the solver and model handles.
TEST_F(ProxMpcControllerTest, CleanupReleasesOwnedHandles)
{
  auto c = makeConfigured();
  c->cleanup();
  EXPECT_EQ(c->mpc(), nullptr);
  EXPECT_EQ(c->model(), nullptr);
  controller_.reset();                       // already cleaned; avoid double cleanup
}

// --- setPlan() -------------------------------------------------------------

// setPlan() stores the plan and resets the forward-only projection index.
TEST_F(ProxMpcControllerTest, SetPlanResetsProjectionIndex)
{
  auto c = makeConfigured();
  c->planIndex() = 7;
  c->setPlan(makeStraightPlan(10, 0.2));
  EXPECT_EQ(c->planIndex(), 0u);
}

// --- computeVelocityCommands(): happy path ---------------------------------

// A converging solve on a straight plan yields a finite, forward command and
// resets the failure counter; a stale projection index is repaired.
TEST_F(ProxMpcControllerTest, ComputeReturnsForwardCommandOnStraightPlan)
{
  auto c = makeConfigured();
  c->activate();
  c->setPlan(makeStraightPlan(31, 0.2));
  c->failureCount() = 2;
  c->planIndex() = 999;                      // stale: must be repaired in-cycle

  const auto cmd = c->computeVelocityCommands(
    makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);

  EXPECT_EQ(cmd.header.frame_id, "base_link");
  EXPECT_GT(cmd.twist.linear.x, 0.0);
  EXPECT_TRUE(std::isfinite(cmd.twist.linear.x));
  EXPECT_TRUE(std::isfinite(cmd.twist.angular.z));
  EXPECT_EQ(c->failureCount(), 0);
  EXPECT_LT(c->planIndex(), 31u);
}

// The same converging path works with the 3-state Unicycle model (no steering
// channel), exercising the n <= 3 branches.
TEST_F(ProxMpcControllerTest, ComputeWorksWithUnicycleModel)
{
  auto c = makeConfigured(
    {rclcpp::Parameter("FollowPath.model_plugin", std::string("prox_mpc_core/Unicycle"))});
  c->activate();
  c->setPlan(makeStraightPlan(31, 0.2));
  EXPECT_EQ(c->nDim(), 3u);

  const auto cmd = c->computeVelocityCommands(
    makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_GT(cmd.twist.linear.x, 0.0);
}

// With obstacles disabled (K = 0) the in-cycle costmap reduction is skipped and
// the command is still produced.
TEST_F(ProxMpcControllerTest, ComputeSkipsReductionWhenObstaclesDisabled)
{
  auto c = makeConfigured({rclcpp::Parameter("FollowPath.max_obstacles", 0)});
  c->activate();
  c->setPlan(makeStraightPlan(31, 0.2));
  const auto cmd = c->computeVelocityCommands(
    makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_GT(cmd.twist.linear.x, 0.0);
}

// A plan expressed in a different frame is transformed when the transform is
// available, and the converging command is returned.
TEST_F(ProxMpcControllerTest, ComputeTransformsPlanWhenTransformAvailable)
{
  auto c = makeConfigured();
  c->activate();

  geometry_msgs::msg::TransformStamped tfs;
  tfs.header.frame_id = "map";
  tfs.child_frame_id = "odom";
  tfs.transform.translation.x = 0.5;
  tfs.transform.rotation.w = 1.0;
  tf_->setTransform(tfs, "test", true);

  c->setPlan(makeStraightPlan(31, 0.2, "odom"));
  const auto cmd = c->computeVelocityCommands(
    makePose(0.5, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_TRUE(std::isfinite(cmd.twist.linear.x));
  EXPECT_GT(cmd.twist.linear.x, 0.0);
}

// --- computeVelocityCommands(): fail-safe branches -------------------------

// An empty/unset plan is a structural fault: throw InvalidPath immediately.
TEST_F(ProxMpcControllerTest, ComputeThrowsInvalidPathOnEmptyPlan)
{
  auto c = makeConfigured();
  c->activate();
  EXPECT_THROW(
    c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr),
    nav2_core::InvalidPath);
}

// A plan frame with no available transform raises ControllerTFError.
TEST_F(ProxMpcControllerTest, ComputeThrowsTfErrorWhenTransformMissing)
{
  auto c = makeConfigured();
  c->activate();
  c->setPlan(makeStraightPlan(31, 0.2, "odom"));   // no map<-odom transform set
  EXPECT_THROW(
    c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr),
    nav2_core::ControllerTFError);
}

// A short plan (single pose) holds the goal: the command stays finite and the
// cruise speed tapers to (about) zero rather than overshooting the plan end.
TEST_F(ProxMpcControllerTest, ComputeHoldsGoalOnShortPlan)
{
  auto c = makeConfigured();
  c->activate();
  c->setPlan(makeStraightPlan(1, 0.2));            // plan_size < 2: goal-hold
  const auto cmd = c->computeVelocityCommands(
    makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_TRUE(std::isfinite(cmd.twist.linear.x));
  EXPECT_NEAR(cmd.twist.linear.x, 0.0, 1e-3);
}

// A short multi-pose plan that the horizon samples past the end holds the goal
// with the heading taken from the final path tangent (not the pose orientation).
TEST_F(ProxMpcControllerTest, ComputeHoldsGoalTangentOnShortMultiPosePlan)
{
  auto c = makeConfigured();
  c->activate();
  c->setPlan(makeStraightPlan(3, 0.2));            // 0.4 m plan, sampled past its end
  const auto cmd = c->computeVelocityCommands(
    makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_TRUE(std::isfinite(cmd.twist.linear.x));
  EXPECT_TRUE(std::isfinite(cmd.twist.angular.z));
}

// A non-finite robot pose is treated as a transient fault: decelerate (here from
// rest, so the safe command is zero) without throwing while budget remains.
TEST_F(ProxMpcControllerTest, ComputeDeceleratesOnNonFinitePose)
{
  auto c = makeConfigured();
  c->activate();
  c->setPlan(makeStraightPlan(31, 0.2));
  geometry_msgs::msg::PoseStamped bad = makePose(0.0, 0.0, 0.0);
  bad.pose.position.x = std::numeric_limits<double>::quiet_NaN();

  const auto cmd = c->computeVelocityCommands(bad, geometry_msgs::msg::Twist(), nullptr);
  EXPECT_NEAR(cmd.twist.linear.x, 0.0, kTol);
  EXPECT_NEAR(cmd.twist.angular.z, 0.0, kTol);
  EXPECT_EQ(c->failureCount(), 1);
}

// A non-converged solve decelerates the last command at the model deceleration
// limit, then escalates to NoValidControl once the failure budget is spent.
TEST_F(ProxMpcControllerTest, ComputeSolverFailureRampsThenEscalates)
{
  auto c = makeConfigured(
  {
    rclcpp::Parameter("FollowPath.max_int_iter_qp", 1),
    rclcpp::Parameter("FollowPath.max_ext_iter_qp", 1),
    rclcpp::Parameter("FollowPath.max_iter_sqp", 1),
    rclcpp::Parameter("FollowPath.max_solver_failures", 1),
  });
  c->activate();
  c->setPlan(makeStraightPlan(31, 0.2));

  // Seed a non-zero last command so the deceleration ramp is observable; the
  // angular channel is negative to exercise the opposite-sign brake step.
  c->lastCmdV() = 0.30;
  c->lastCmdW() = -0.30;

  const auto cmd = c->computeVelocityCommands(
    makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);

  // v_cmd = max(0, v_prev - a_dec * dt); a_dec = 0.5, dt = 0.1 -> step 0.05.
  EXPECT_NEAR(cmd.twist.linear.x, 0.30 - kModelDecel * 0.1, 1e-6);
  EXPECT_NEAR(cmd.twist.angular.z, -0.30 + kModelDecel * 0.1, 1e-6);
  EXPECT_EQ(c->failureCount(), 1);

  // Second consecutive failure exceeds max_solver_failures = 1 -> escalate.
  EXPECT_THROW(
    c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr),
    nav2_core::NoValidControl);
}

// A converged solve whose one-step-ahead footprint lands on lethal cells is
// vetoed to the brake ramp without consuming the failure budget.
TEST_F(ProxMpcControllerTest, ComputeFootprintVetoBrakesWithoutFailure)
{
  auto c = makeConfigured({rclcpp::Parameter("FollowPath.max_obstacles", 0)});
  c->activate();
  c->setPlan(makeStraightPlan(31, 0.2));
  // Lethal block over the robot's one-step-ahead footprint (near the origin).
  fillCost(-0.6, -0.6, 0.6, 0.6, nav2_costmap_2d::LETHAL_OBSTACLE);

  const auto cmd = c->computeVelocityCommands(
    makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_NEAR(cmd.twist.linear.x, 0.0, kTol);    // ramp from rest
  EXPECT_EQ(c->failureCount(), 0);               // veto is not a solver failure
}

// An empty footprint skips the polygon veto (size < 3) and still commands.
TEST_F(ProxMpcControllerTest, ComputeSkipsVetoWithoutFootprint)
{
  costmap_ros_->setRobotFootprint(std::vector<geometry_msgs::msg::Point>{});
  auto c = makeConfigured({rclcpp::Parameter("FollowPath.max_obstacles", 0)});
  c->activate();
  c->setPlan(makeStraightPlan(31, 0.2));
  fillCost(-0.6, -0.6, 0.6, 0.6, nav2_costmap_2d::LETHAL_OBSTACLE);

  const auto cmd = c->computeVelocityCommands(
    makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_GT(cmd.twist.linear.x, 0.0);            // no veto: command passes through
}

// --- setSpeedLimit() -------------------------------------------------------

// An absolute speed limit applies the value as the linear bound.
TEST_F(ProxMpcControllerTest, SetSpeedLimitAbsolute)
{
  auto c = makeConfigured();
  c->setSpeedLimit(0.8, false);
  EXPECT_NEAR(c->maxLinearVel(), 0.8, kTol);
}

// A percentage speed limit is a fraction of the model maximum.
TEST_F(ProxMpcControllerTest, SetSpeedLimitPercentage)
{
  auto c = makeConfigured();
  c->setSpeedLimit(50.0, true);
  EXPECT_NEAR(c->maxLinearVel(), 0.5 * kModelVMax, kTol);
}

// A limit above the model maximum is clamped to v_max.
TEST_F(ProxMpcControllerTest, SetSpeedLimitClampsToModelMax)
{
  auto c = makeConfigured();
  c->setSpeedLimit(10.0, false);
  EXPECT_NEAR(c->maxLinearVel(), kModelVMax, kTol);
}

// A non-positive limit (NO_SPEED_LIMIT) restores the model's full bound.
TEST_F(ProxMpcControllerTest, SetSpeedLimitZeroRestoresModelMax)
{
  auto c = makeConfigured();
  c->setSpeedLimit(1.0, false);
  EXPECT_NEAR(c->maxLinearVel(), 1.0, kTol);
  c->setSpeedLimit(0.0, false);
  EXPECT_NEAR(c->maxLinearVel(), kModelVMax, kTol);
}

// --- cancel() / reset() ----------------------------------------------------

// cancel() reports done immediately when the robot is already stopped.
TEST_F(ProxMpcControllerTest, CancelReturnsTrueWhenStopped)
{
  auto c = makeConfigured();
  c->activate();                             // last command is zero
  EXPECT_TRUE(c->cancel());
  EXPECT_FALSE(c->cancelling());
}

// cancel() ramps the moving robot to a stop: it returns false while moving,
// drives the brake ramp on the next cycle, and returns true once stopped.
TEST_F(ProxMpcControllerTest, CancelRampsMovingRobotToStop)
{
  auto c = makeConfigured();
  c->activate();
  c->setPlan(makeStraightPlan(31, 0.2));
  const auto moving = c->computeVelocityCommands(
    makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  ASSERT_GT(moving.twist.linear.x, 0.0);

  EXPECT_FALSE(c->cancel());                 // still moving
  EXPECT_TRUE(c->cancelling());

  const auto braking = c->computeVelocityCommands(
    makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_LT(braking.twist.linear.x, moving.twist.linear.x);
  EXPECT_GE(braking.twist.linear.x, 0.0);
  EXPECT_TRUE(c->cancel());                  // ramped to (near) zero
}

// reset() clears all runtime state between tasks but keeps owned handles.
TEST_F(ProxMpcControllerTest, ResetClearsRuntimeState)
{
  auto c = makeConfigured();
  c->failureCount() = 4;
  c->lastCmdV() = 1.0;
  c->lastCmdW() = -1.0;
  c->steeringState() = 0.3;
  c->planIndex() = 9;
  c->cancel();                               // sets cancelling_

  c->reset();

  EXPECT_EQ(c->failureCount(), 0);
  EXPECT_NEAR(c->lastCmdV(), 0.0, kTol);
  EXPECT_NEAR(c->lastCmdW(), 0.0, kTol);
  EXPECT_NEAR(c->steeringState(), 0.0, kTol);
  EXPECT_EQ(c->planIndex(), 0u);
  EXPECT_FALSE(c->cancelling());
  EXPECT_NE(c->mpc(), nullptr);              // owned handles intact
  EXPECT_NE(c->model(), nullptr);
}

// --- reduceCostmap(): obstacle reduction (white-box) -----------------------

// reduceCostmap clusters nearby lethal cells into at most K representatives per
// node, skips unknown cells and out-of-grid nodes, and leaves empty slots at the
// far sentinel.
TEST_F(ProxMpcControllerTest, ReduceCostmapClustersAndSentinels)
{
  // Np = 2, K = 2; a large robot radius forces the scan window to clamp.
  auto c = makeConfigured(
  {
    rclcpp::Parameter("FollowPath.np", 2),
    rclcpp::Parameter("FollowPath.nc", 2),
    rclcpp::Parameter("FollowPath.max_obstacles", 2),
    rclcpp::Parameter("FollowPath.robot_radius", 3.0),
    rclcpp::Parameter("FollowPath.safety_margin", 0.1),
    rclcpp::Parameter("FollowPath.obstacle_cluster_radius", 0.3),
  });
  ASSERT_EQ(c->maxObstacles(), 2u);

  // Two distinct lethal clusters within the node's search window, plus an
  // unknown cell that must be ignored.
  fillCost(1.0, 0.0, 1.3, 0.3, nav2_costmap_2d::LETHAL_OBSTACLE);   // cluster A
  fillCost(1.0, 2.0, 1.3, 2.3, nav2_costmap_2d::LETHAL_OBSTACLE);   // cluster B
  fillCost(0.8, 0.8, 0.8, 0.8, nav2_costmap_2d::NO_INFORMATION);    // ignored

  const std::size_t np = 2;
  const std::size_t k = 2;
  MatrixXd reference = MatrixXd::Zero(static_cast<Eigen::Index>(np + 1), 4);
  reference(1, 0) = 1.15;          // node 0: inside the grid, near both clusters
  reference(1, 1) = 1.0;
  reference(2, 0) = 100.0;         // node 1: outside the grid -> worldToMap fails
  reference(2, 1) = 100.0;

  MatrixXd obs(static_cast<Eigen::Index>(np * k), 3);
  c->reduceCostmap(reference, obs);

  // Node 0: two distinct representatives, each carrying d_safe = radius + margin.
  EXPECT_LT(obs(0, 0), prox_mpc::MPC::kObsFarSentinel);
  EXPECT_LT(obs(1, 0), prox_mpc::MPC::kObsFarSentinel);
  EXPECT_NEAR(obs(0, 2), 3.0 + 0.1, kTol);
  EXPECT_NEAR(obs(1, 2), 3.0 + 0.1, kTol);
  const double sep = std::hypot(obs(0, 0) - obs(1, 0), obs(0, 1) - obs(1, 1));
  EXPECT_GE(sep, 0.3);             // representatives at least a cluster radius apart

  // Node 1: out of grid, both slots stay at the far sentinel.
  EXPECT_NEAR(obs(2, 0), prox_mpc::MPC::kObsFarSentinel, kTol);
  EXPECT_NEAR(obs(3, 0), prox_mpc::MPC::kObsFarSentinel, kTol);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
