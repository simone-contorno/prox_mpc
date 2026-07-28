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
//     the cache-before-model case, and the deferred apply-on-next-cycle contract;
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

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_core/controller_exceptions.hpp>
#include <nav2_core/goal_checker.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>

#include <prox_mpc_msgs/msg/obstacle_array.hpp>

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
  using ProxMpcController::fillObstacles;
  using ProxMpcController::readModelBounds;

  // Inject the latest tracked-obstacle message directly (white-box), bypassing
  // the subscription so the predictive fill can be driven deterministically.
  void injectObstacles(prox_mpc_msgs::msg::ObstacleArray::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    latest_obstacles_ = msg;
  }

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

// A model that declares its control bounds but no control-rate (du) bounds, so
// the deceleration limits the brake ramp needs do not exist. Defined here rather
// than in prox_mpc_test_models because it only has to reach readModelBounds(),
// not the pluginlib load path. The dynamics are never solved.
class NoDuBoundModel : public prox_mpc::Model
{
public:
  NoDuBoundModel()
  {
    setName("no_du_bound");
    setN(3);
    setM(2);
    setIneq("u", 0, -3.0, 3.0);
    setIneq("u", 1, -1.0, 1.0);
  }
  void updatec(double, VectorXd) override {}
  void updateA(double) override {}
  void updateB() override {}
};

// The mirror case: control-rate bounds but no bound on the speed channel, which
// would leave v_max (and hence the cruise-speed clamp) at zero.
class NoUBoundModel : public prox_mpc::Model
{
public:
  NoUBoundModel()
  {
    setName("no_u_bound");
    setN(3);
    setM(2);
    setIneq("du", 0, -0.5, 0.5);
    setIneq("du", 1, -0.5, 0.5);
  }
  void updatec(double, VectorXd) override {}
  void updateA(double) override {}
  void updateB() override {}
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

// A constant-curvature arc plan: a circle of radius R starting at the origin
// tangent to +x and curving CCW, sampled every ds meters of arc length.
nav_msgs::msg::Path makeArcPlan(std::size_t count, double ds, double radius)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  const double dphi = ds / radius;
  for (std::size_t i = 0; i < count; ++i) {
    const double phi = static_cast<double>(i) * dphi;
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map";
    p.pose.position.x = radius * std::sin(phi);
    p.pose.position.y = radius * (1.0 - std::cos(phi));
    p.pose.orientation.w = 1.0;
    path.poses.push_back(p);
  }
  return path;
}

// An ObstacleArray with a single tracked obstacle expressed in the costmap
// global frame ("map"), so the predictive fill needs no TF to place it.
prox_mpc_msgs::msg::ObstacleArray::SharedPtr makeObstacleMsg(
  const rclcpp::Time & stamp, double x, double y, double vx, double vy, double radius)
{
  auto msg = std::make_shared<prox_mpc_msgs::msg::ObstacleArray>();
  msg->header.frame_id = "map";
  msg->header.stamp = stamp;
  prox_mpc_msgs::msg::Obstacle o;
  o.id = 7;
  o.position.x = x;
  o.position.y = y;
  o.velocity.x = vx;
  o.velocity.y = vy;
  o.radius = radius;
  msg->obstacles.push_back(o);
  return msg;
}

// Attach tracker-sampled predicted positions (sample k, 1-based, is the
// position at stamp + k * prediction_dt) to the last obstacle of the message.
void addPredictedSamples(
  prox_mpc_msgs::msg::ObstacleArray & msg,
  const std::vector<std::array<double, 2>> & samples, double prediction_dt)
{
  auto & o = msg.obstacles.back();
  o.prediction_dt = prediction_dt;
  o.predicted_positions.clear();
  for (const auto & s : samples) {
    geometry_msgs::msg::Point p;
    p.x = s[0];
    p.y = s[1];
    p.z = 0.0;
    o.predicted_positions.push_back(p);
  }
}

// Minimal GoalChecker stub returning a fixed xy tolerance. goal_checker is a raw
// argument to computeVelocityCommands (not a pluginlib plugin), so the approach
// easing can be driven by injecting this directly, no registration needed.
class StubGoalChecker : public nav2_core::GoalChecker
{
public:
  StubGoalChecker(double xy_tol, bool valid)
  : xy_tol_(xy_tol), valid_(valid) {}
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &, const std::string &,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void reset() override {}
  bool isGoalReached(
    const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Twist &) override {return false;}
  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance, geometry_msgs::msg::Twist &) override
  {
    pose_tolerance.position.x = xy_tol_;
    pose_tolerance.position.y = xy_tol_;
    return valid_;
  }

private:
  double xy_tol_;
  bool valid_;
};
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

  // A configured, activated controller with a straight plan, ready to run cycles.
  std::shared_ptr<TestableProxMpcController> makeRunning(
    const std::vector<rclcpp::Parameter> & overrides = {})
  {
    auto c = makeConfigured(overrides);
    c->activate();
    c->setPlan(makeStraightPlan(31, 0.2));
    return c;
  }

  // Run one control cycle from the plan origin (the seam where a speed limit
  // requested since the previous cycle is applied).
  void runCycle(const std::shared_ptr<TestableProxMpcController> & c)
  {
    c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
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

// Nav2's controller_server destroys the plugin instance on cleanup() but its
// node keeps every plugin parameter declared, so a cleanup() -> configure()
// cycle must not redeclare them: an unguarded declaration throws
// rclcpp::exceptions::ParameterAlreadyDeclaredException, which is not a
// nav2_core::ControllerException, and the lifecycle transition fails. The same
// holds for a fresh plugin instance configured against the same node.
TEST_F(ProxMpcControllerTest, ReconfigureOnSameNodeDoesNotRedeclareParameters)
{
  auto c = makeUnconfigured({rclcpp::Parameter("FollowPath.desired_linear_vel", 0.7)});
  ASSERT_NO_THROW(c->configure(node_, "FollowPath", tf_, costmap_ros_));
  c->activate();
  c->deactivate();
  c->cleanup();
  ASSERT_TRUE(node_->has_parameter("FollowPath.desired_linear_vel"));

  EXPECT_NO_THROW(c->configure(node_, "FollowPath", tf_, costmap_ros_));
  EXPECT_NEAR(c->desiredLinearVel(), 0.7, kTol);   // the declared value is re-read

  auto fresh = std::make_shared<TestableProxMpcController>();
  EXPECT_NO_THROW(fresh->configure(node_, "FollowPath", tf_, costmap_ros_));
  EXPECT_NEAR(fresh->desiredLinearVel(), 0.7, kTol);
  fresh->cleanup();
}

// A model that declares no control-rate (du) bound cannot brake - the ramp step
// would be zero, so the solver-failure path would command the current velocity
// forever and cancel() would never complete - and one with no bound on the speed
// channel would clamp the cruise speed to zero. Both fail configure with a
// ControllerException naming the missing bound.
TEST_F(ProxMpcControllerTest, MissingModelBoundIsFatal)
{
  auto c = makeUnconfigured();

  NoDuBoundModel no_du;
  try {
    c->readModelBounds(no_du, "test/NoDuBound");
    FAIL() << "a model without du bounds must not configure";
  } catch (const nav2_core::ControllerException & ex) {
    const std::string what(ex.what());
    EXPECT_NE(what.find("'du'"), std::string::npos) << what;
    EXPECT_NE(what.find("test/NoDuBound"), std::string::npos) << what;
  }

  NoUBoundModel no_u;
  try {
    c->readModelBounds(no_u, "test/NoUBound");
    FAIL() << "a model without a u bound must not configure";
  } catch (const nav2_core::ControllerException & ex) {
    const std::string what(ex.what());
    EXPECT_NE(what.find("'u'"), std::string::npos) << what;
  }

  // The bundled model declares both, so the same read succeeds and yields the
  // model's own bounds.
  auto configured = makeConfigured();
  EXPECT_NEAR(configured->vMax(), kModelVMax, kTol);
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

// A plan whose heading sits across the +/-pi wrap from the robot's heading must
// not drive the QP heading error the long way around: the controller keeps the
// goal_x heading continuous, so the command tracks forward rather than spinning
// in place. Without the fix the raw error is ~6 rad and the turn saturates.
TEST_F(ProxMpcControllerTest, ComputeTracksAcrossHeadingWrap)
{
  // Heading-dominant weights so the wrap error drives the angular command.
  auto c = makeConfigured(
  {
    rclcpp::Parameter("FollowPath.model_plugin", std::string("prox_mpc_core/Unicycle")),
    rclcpp::Parameter("FollowPath.q_pos", 1.0),
    rclcpp::Parameter("FollowPath.q_theta", 50.0),
  });
  c->activate();

  // Straight plan heading -3.0 rad; robot heading +3.0 rad. The true heading
  // error is ~0.28 rad, but the unwrapped difference is ~6.0 rad.
  const double dir = -3.0;
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (std::size_t i = 0; i < 31; ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose.position.x = static_cast<double>(i) * 0.2 * std::cos(dir);
    ps.pose.position.y = static_cast<double>(i) * 0.2 * std::sin(dir);
    ps.pose.orientation.w = 1.0;
    path.poses.push_back(ps);
  }
  c->setPlan(path);

  const auto cmd = c->computeVelocityCommands(
    makePose(0.0, 0.0, 3.0), geometry_msgs::msg::Twist(), nullptr);

  // The continuous goal heading is ~+3.28 rad, so the shortest correction from
  // +3.0 rad is a small CCW (positive) turn. Without the fix the error is ~-6 rad
  // and the QP turns the long way (strongly negative omega).
  EXPECT_TRUE(std::isfinite(cmd.twist.linear.x));
  EXPECT_TRUE(std::isfinite(cmd.twist.angular.z));
  EXPECT_GT(cmd.twist.angular.z, 0.0);             // turns the short (CCW) way, not the long way
}

// On a curved plan the bicycle steering reference is pre-positioned to the path
// curvature (delta_ref = atan(L*kappa) != 0); on a straight plan it stays zero.
// Read back from the state reference the controller hands the MPC.
TEST_F(ProxMpcControllerTest, CurvatureSetsBicycleSteeringReference)
{
  auto c = makeConfigured();   // default Bicycle (n = 4, L = 1.6)
  c->activate();
  ASSERT_EQ(c->nDim(), 4u);

  c->setPlan(makeStraightPlan(31, 0.1));
  c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  const MatrixXd gx_straight = c->mpc()->getGoalX();
  double max_delta_straight = 0.0;
  for (Eigen::Index k = 0; k < gx_straight.rows(); ++k) {
    max_delta_straight = std::max(max_delta_straight, std::abs(gx_straight(k, 3)));
  }
  EXPECT_LT(max_delta_straight, 1e-6);             // straight: wheel reference stays centered

  c->setPlan(makeArcPlan(60, 0.1, 2.0));           // R = 2 m -> kappa = 0.5
  c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  const MatrixXd gx_arc = c->mpc()->getGoalX();
  double max_delta_arc = 0.0;
  for (Eigen::Index k = 0; k < gx_arc.rows(); ++k) {
    max_delta_arc = std::max(max_delta_arc, std::abs(gx_arc(k, 3)));
  }
  EXPECT_GT(max_delta_arc, 0.2);                   // atan(L*kappa) ~ atan(0.8) = 0.675 rad
}

// The unicycle (no steering state, n = 3) keeps the go-straight default even on
// a curved plan: the steering channel does not exist, so none is written.
TEST_F(ProxMpcControllerTest, CurvatureLeavesUnicycleReferenceAtThreeState)
{
  auto c = makeConfigured(
    {rclcpp::Parameter("FollowPath.model_plugin", std::string("prox_mpc_core/Unicycle"))});
  c->activate();
  ASSERT_EQ(c->nDim(), 3u);
  c->setPlan(makeArcPlan(60, 0.1, 2.0));
  c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_EQ(c->mpc()->getGoalX().cols(), static_cast<Eigen::Index>(3));   // no steering channel
}

// curvature_gain > 0 tapers the cruise reference on a curved plan; the default
// (0.0) leaves it at the arc-length cruise.
TEST_F(ProxMpcControllerTest, CurvatureGainReducesCruiseOnCurvedPlan)
{
  auto c0 = makeConfigured();   // curvature_gain default 0.0
  c0->activate();
  c0->setPlan(makeArcPlan(60, 0.1, 2.0));
  c0->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  const double vref_gain0 = c0->mpc()->getGoalU()(0, 0);

  auto cg = makeConfigured({rclcpp::Parameter("FollowPath.curvature_gain", 2.0)});
  cg->activate();
  cg->setPlan(makeArcPlan(60, 0.1, 2.0));
  cg->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  const double vref_gain = cg->mpc()->getGoalU()(0, 0);

  EXPECT_GT(vref_gain0, 0.0);
  EXPECT_LT(vref_gain, vref_gain0);                // curvature taper slows the cruise
}

// goal_checker xy tolerance eases the cruise reference once the robot is inside
// the tolerance band; the lowest() sentinel and a false return leave it intact.
TEST_F(ProxMpcControllerTest, GoalCheckerToleranceEasesApproach)
{
  auto c = makeConfigured();
  c->activate();
  c->setPlan(makeStraightPlan(6, 0.2));            // 1 m plan
  const auto pose = makePose(0.8, 0.0, 0.0);       // 0.2 m remaining to the end

  c->computeVelocityCommands(pose, geometry_msgs::msg::Twist(), nullptr);
  const double vref_base = c->mpc()->getGoalU()(0, 0);
  ASSERT_GT(vref_base, 0.0);

  StubGoalChecker inside(0.25, true);              // remaining 0.2 m < xy_tol 0.25 m
  c->computeVelocityCommands(pose, geometry_msgs::msg::Twist(), &inside);
  EXPECT_LT(c->mpc()->getGoalU()(0, 0), vref_base);

  StubGoalChecker sentinel(std::numeric_limits<double>::lowest(), true);
  c->computeVelocityCommands(pose, geometry_msgs::msg::Twist(), &sentinel);
  EXPECT_NEAR(c->mpc()->getGoalU()(0, 0), vref_base, kTol);   // sentinel ignored

  StubGoalChecker invalid(0.25, false);            // getTolerances() returns false
  c->computeVelocityCommands(pose, geometry_msgs::msg::Twist(), &invalid);
  EXPECT_NEAR(c->mpc()->getGoalU()(0, 0), vref_base, kTol);   // no tolerances -> no easing
}

// Outside the tolerance band (remaining >> xy_tol) the approach easing clamps to
// 1.0 and the cruise reference is unchanged.
TEST_F(ProxMpcControllerTest, GoalCheckerToleranceInertOutsideBand)
{
  auto c = makeConfigured();
  c->activate();
  c->setPlan(makeStraightPlan(31, 0.2));           // 6 m plan, robot at the start
  const auto pose = makePose(0.0, 0.0, 0.0);

  c->computeVelocityCommands(pose, geometry_msgs::msg::Twist(), nullptr);
  const double vref_base = c->mpc()->getGoalU()(0, 0);
  StubGoalChecker checker(0.25, true);
  c->computeVelocityCommands(pose, geometry_msgs::msg::Twist(), &checker);
  EXPECT_NEAR(c->mpc()->getGoalU()(0, 0), vref_base, kTol);
}

// The predicted NMPC trajectory is published as a Path (Np+1 poses, costmap
// global frame) for visualization, distinct from the Nav2 global plan.
TEST_F(ProxMpcControllerTest, PublishesPredictedTrajectory)
{
  auto c = makeConfigured();
  c->activate();
  c->setPlan(makeStraightPlan(31, 0.2));

  nav_msgs::msg::Path received;
  bool got = false;
  auto sub = node_->create_subscription<nav_msgs::msg::Path>(
    "prox_mpc_local_plan", 1,
    [&](nav_msgs::msg::Path::SharedPtr msg) {received = *msg; got = true;});

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_->get_node_base_interface());

  // The publish is skipped until a subscriber connects, so re-drive the controller
  // while spinning until the message is delivered.
  for (int i = 0; i < 100 && !got; ++i) {
    c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  ASSERT_TRUE(got);
  EXPECT_EQ(received.header.frame_id, "map");
  EXPECT_EQ(received.poses.size(), 21u);          // Np + 1
  EXPECT_TRUE(std::isfinite(received.poses.front().pose.position.x));
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

// The single-pose goal-hold heading is taken from the plan pose orientation,
// which lives in the plan frame: it must be rotated into the costmap global
// frame like the plan positions are, or the held heading is off by the plan
// transform's yaw.
TEST_F(ProxMpcControllerTest, ShortPlanGoalHoldHeadingIsTransformed)
{
  auto c = makeConfigured();
  c->activate();

  // map <- odom rotated by +pi/2 (no translation).
  geometry_msgs::msg::TransformStamped tfs;
  tfs.header.frame_id = "map";
  tfs.child_frame_id = "odom";
  tfs.transform.rotation.z = std::sin(M_PI / 4.0);
  tfs.transform.rotation.w = std::cos(M_PI / 4.0);
  tf_->setTransform(tfs, "test", true);

  c->setPlan(makeStraightPlan(1, 0.2, "odom"));    // single pose, yaw 0 in odom
  c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);

  const MatrixXd gx = c->mpc()->getGoalX();
  for (Eigen::Index k = 0; k < gx.rows(); ++k) {
    EXPECT_NEAR(gx(k, 2), M_PI / 2.0, 1e-9);       // plan yaw 0 rotated into map
  }
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

  // The brake ramps from the server-measured velocity, so supply a non-zero
  // measured twist (angular negative to exercise the opposite-sign brake step).
  geometry_msgs::msg::Twist measured;
  measured.linear.x = 0.30;
  measured.angular.z = -0.30;

  const auto cmd = c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), measured, nullptr);

  // Arrange-phase guard: confirm the one-iteration caps actually prevented
  // convergence, so this exercises the failure path instead of silently inverting
  // if the QP ever converged in a single iteration.
  ASSERT_NE(c->mpc()->qp_info.status, proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED);

  // v_cmd = max(0, v_meas - a_dec * dt); a_dec = 0.5, dt = 0.1 -> step 0.05.
  EXPECT_NEAR(cmd.twist.linear.x, 0.30 - kModelDecel * 0.1, 1e-6);
  EXPECT_NEAR(cmd.twist.angular.z, -0.30 + kModelDecel * 0.1, 1e-6);
  EXPECT_EQ(c->failureCount(), 1);

  // Second consecutive failure exceeds max_solver_failures = 1 -> escalate.
  EXPECT_THROW(
    c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), measured, nullptr),
    nav2_core::NoValidControl);
}

// A model with finite dynamics (the QP converges) but a non-finite toTwist drives
// the non-finite-command fail-safe: brake within the budget, then escalate. The
// fault-injection model is loaded through the same pluginlib path as production
// models (prox_mpc_test_models fixture).
TEST_F(ProxMpcControllerTest, NonFiniteCommandRampsThenEscalates)
{
  auto c = makeConfigured(
  {
    rclcpp::Parameter(
      "FollowPath.model_plugin", std::string("prox_mpc_test_models/NonFiniteTwist")),
    rclcpp::Parameter("FollowPath.max_solver_failures", 1),
  });
  c->activate();
  c->setPlan(makeStraightPlan(31, 0.2));

  // The brake ramps from the server-measured velocity, so supply a non-zero one.
  geometry_msgs::msg::Twist measured;
  measured.linear.x = 0.30;
  measured.angular.z = -0.30;

  // First cycle: the QP converges but the command maps to a non-finite twist, so
  // the controller brakes at the model deceleration limit without escalating yet.
  const auto cmd = c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), measured, nullptr);
  EXPECT_NEAR(cmd.twist.linear.x, 0.30 - kModelDecel * 0.1, 1e-6);
  EXPECT_NEAR(cmd.twist.angular.z, -0.30 + kModelDecel * 0.1, 1e-6);
  EXPECT_EQ(c->failureCount(), 1);

  // Second consecutive failure exceeds max_solver_failures = 1 -> escalate.
  EXPECT_THROW(
    c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), measured, nullptr),
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

// A footprint veto that persists past the budget escalates to NoValidControl so
// the behavior-tree recovery is triggered (a stuck robot does not brake forever),
// while the first vetoes brake without consuming the solver-failure budget.
TEST_F(ProxMpcControllerTest, PersistentFootprintVetoEscalates)
{
  auto c = makeConfigured(
  {
    rclcpp::Parameter("FollowPath.max_obstacles", 0),
    rclcpp::Parameter("FollowPath.max_solver_failures", 3),
  });
  c->activate();
  c->setPlan(makeStraightPlan(31, 0.2));
  fillCost(-0.6, -0.6, 0.6, 0.6, nav2_costmap_2d::LETHAL_OBSTACLE);

  // Budget 3: the first three vetoes brake (no throw); the fourth escalates.
  for (int i = 0; i < 3; ++i) {
    const auto cmd = c->computeVelocityCommands(
      makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
    EXPECT_NEAR(cmd.twist.linear.x, 0.0, kTol);
    EXPECT_EQ(c->failureCount(), 0);             // veto does not consume the solver budget
  }
  EXPECT_THROW(
    c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr),
    nav2_core::NoValidControl);
}

// --- setSpeedLimit() -------------------------------------------------------

// An absolute speed limit applies the value as the linear bound.
TEST_F(ProxMpcControllerTest, SetSpeedLimitAbsolute)
{
  auto c = makeRunning();
  c->setSpeedLimit(0.8, false);
  runCycle(c);
  EXPECT_NEAR(c->maxLinearVel(), 0.8, kTol);
}

// A percentage speed limit is a fraction of the model maximum.
TEST_F(ProxMpcControllerTest, SetSpeedLimitPercentage)
{
  auto c = makeRunning();
  c->setSpeedLimit(50.0, true);
  runCycle(c);
  EXPECT_NEAR(c->maxLinearVel(), 0.5 * kModelVMax, kTol);
}

// A limit above the model maximum is clamped to v_max.
TEST_F(ProxMpcControllerTest, SetSpeedLimitClampsToModelMax)
{
  auto c = makeRunning();
  c->setSpeedLimit(10.0, false);
  runCycle(c);
  EXPECT_NEAR(c->maxLinearVel(), kModelVMax, kTol);
}

// A non-positive limit (NO_SPEED_LIMIT) restores the model's full bound.
TEST_F(ProxMpcControllerTest, SetSpeedLimitZeroRestoresModelMax)
{
  auto c = makeRunning();
  c->setSpeedLimit(1.0, false);
  runCycle(c);
  EXPECT_NEAR(c->maxLinearVel(), 1.0, kTol);
  c->setSpeedLimit(0.0, false);
  runCycle(c);
  EXPECT_NEAR(c->maxLinearVel(), kModelVMax, kTol);
}

// setSpeedLimit() runs on the node executor thread while computeVelocityCommands
// runs on the action server's own thread, so the request is only cached there
// and applied at the top of the next control cycle: the model's inequality map
// is never mutated under a running solve. Both the absolute and the percentage
// forms take effect exactly one cycle later, and the model bound follows.
TEST_F(ProxMpcControllerTest, SpeedLimitAppliesOnNextControlCycle)
{
  auto c = makeRunning();
  ASSERT_NEAR(c->maxLinearVel(), kModelVMax, kTol);

  c->setSpeedLimit(0.8, false);
  EXPECT_NEAR(c->maxLinearVel(), kModelVMax, kTol);        // not applied in place
  runCycle(c);
  EXPECT_NEAR(c->maxLinearVel(), 0.8, kTol);
  EXPECT_NEAR(c->model()->getIneq("u").at(0)[2], 0.8, kTol);   // model bound updated

  c->setSpeedLimit(50.0, true);
  EXPECT_NEAR(c->maxLinearVel(), 0.8, kTol);               // still the previous limit
  runCycle(c);
  EXPECT_NEAR(c->maxLinearVel(), 0.5 * kModelVMax, kTol);
  EXPECT_NEAR(c->model()->getIneq("u").at(0)[2], 0.5 * kModelVMax, kTol);
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

// --- fillObstacles(): predictive + hybrid fill (white-box) -----------------

// The predictive fill propagates a tracked obstacle by position + velocity*dt_k
// per node and binds it to one slot across the whole horizon (identity).
TEST_F(ProxMpcControllerTest, PredictiveFillPropagatesObstacleAndKeepsIdentity)
{
  auto c = makeConfigured(
  {
    rclcpp::Parameter("FollowPath.np", 5),
    rclcpp::Parameter("FollowPath.nc", 5),
    rclcpp::Parameter("FollowPath.max_obstacles", 1),
    rclcpp::Parameter("FollowPath.predict_obstacles", true),
    rclcpp::Parameter("FollowPath.dynamic_speed_threshold", 0.05),
    rclcpp::Parameter("FollowPath.robot_radius", 0.5),
    rclcpp::Parameter("FollowPath.safety_margin", 0.1),
  });
  const std::size_t np = 5;
  const std::size_t k = 1;
  const double dt = 0.1;
  const double vx = 1.0;
  const rclcpp::Time now(1000, 0, RCL_ROS_TIME);

  // Obstacle at (2,0) moving +x at 1 m/s, radius 0.2; stamp == now so age = 0.
  c->injectObstacles(makeObstacleMsg(now, 2.0, 0.0, vx, 0.0, 0.2));

  MatrixXd reference = MatrixXd::Zero(static_cast<Eigen::Index>(np + 1), c->nDim());
  for (std::size_t i = 0; i <= np; ++i) {
    reference(static_cast<Eigen::Index>(i), 0) = static_cast<double>(i) * 0.2;  // along +x
  }
  MatrixXd obs(static_cast<Eigen::Index>(np * k), 3);
  c->fillObstacles(reference, obs, now);

  double prev_x = -1.0;
  for (std::size_t node = 0; node < np; ++node) {
    const double dt_k = static_cast<double>(node + 1) * dt;
    const Eigen::Index row = static_cast<Eigen::Index>(node * k);
    EXPECT_NEAR(obs(row, 0), 2.0 + vx * dt_k, 1e-9);    // position + velocity*dt_k
    EXPECT_NEAR(obs(row, 1), 0.0, 1e-9);
    EXPECT_NEAR(obs(row, 2), 0.5 + 0.2 + 0.1, 1e-9);    // robot + obstacle radius + margin
    EXPECT_GT(obs(row, 0), prev_x);                     // identity: one object advancing +x
    prev_x = obs(row, 0);
  }
}

// The hybrid fill keeps a static costmap obstacle in a remaining slot while a
// dynamic track occupies the reserved slot.
TEST_F(ProxMpcControllerTest, HybridFillKeepsStaticObstacle)
{
  auto c = makeConfigured(
  {
    rclcpp::Parameter("FollowPath.np", 2),
    rclcpp::Parameter("FollowPath.nc", 2),
    rclcpp::Parameter("FollowPath.max_obstacles", 2),
    rclcpp::Parameter("FollowPath.max_dynamic_obstacles", 1),
    rclcpp::Parameter("FollowPath.predict_obstacles", true),
    rclcpp::Parameter("FollowPath.dynamic_speed_threshold", 0.05),
    rclcpp::Parameter("FollowPath.robot_radius", 0.5),
    rclcpp::Parameter("FollowPath.safety_margin", 0.1),
    rclcpp::Parameter("FollowPath.obstacle_cluster_radius", 0.3),
  });
  const std::size_t k = 2;
  const rclcpp::Time now(1000, 0, RCL_ROS_TIME);

  // A static lethal block near node 0's reference position.
  fillCost(1.0, 0.8, 1.3, 1.2, nav2_costmap_2d::LETHAL_OBSTACLE);
  // A dynamic obstacle far away, so its exclusion disc does not cover the block.
  c->injectObstacles(makeObstacleMsg(now, 5.0, 5.0, 1.0, 0.0, 0.2));

  MatrixXd reference = MatrixXd::Zero(3, c->nDim());
  reference(1, 0) = 1.15; reference(1, 1) = 1.0;     // node 0 near the static block
  reference(2, 0) = 1.15; reference(2, 1) = 1.0;

  MatrixXd obs(static_cast<Eigen::Index>(2 * k), 3);
  c->fillObstacles(reference, obs, now);

  // Slot 0 (node 0) is the propagated dynamic obstacle (far, x > 5).
  EXPECT_GT(obs(0, 0), 5.0);
  // Slot 1 (node 0) is the static block from the costmap.
  EXPECT_LT(obs(1, 0), prox_mpc::MPC::kObsFarSentinel);
  EXPECT_NEAR(obs(1, 0), 1.15, 0.25);
  EXPECT_NEAR(obs(1, 1), 1.0, 0.25);
  EXPECT_NEAR(obs(1, 2), 0.5 + 0.1, kTol);           // static d_safe = robot_radius + margin
}

// A tracked-obstacle message older than obstacle_timeout falls back to the
// costmap-only fill, byte-for-byte equal to reduceCostmap().
TEST_F(ProxMpcControllerTest, StalenessFallbackRestoresCostmapOnly)
{
  auto c = makeConfigured(
  {
    rclcpp::Parameter("FollowPath.np", 2),
    rclcpp::Parameter("FollowPath.nc", 2),
    rclcpp::Parameter("FollowPath.max_obstacles", 2),
    rclcpp::Parameter("FollowPath.predict_obstacles", true),
    rclcpp::Parameter("FollowPath.obstacle_timeout", 0.5),
    rclcpp::Parameter("FollowPath.dynamic_speed_threshold", 0.05),
    rclcpp::Parameter("FollowPath.robot_radius", 3.0),
    rclcpp::Parameter("FollowPath.safety_margin", 0.1),
    rclcpp::Parameter("FollowPath.obstacle_cluster_radius", 0.3),
  });
  const std::size_t k = 2;
  const rclcpp::Time now(1000, 0, RCL_ROS_TIME);
  const rclcpp::Time stale = now - rclcpp::Duration::from_seconds(1.0);  // > timeout

  fillCost(1.0, 0.0, 1.3, 0.3, nav2_costmap_2d::LETHAL_OBSTACLE);
  // Fresh-looking content but a stale stamp; if used it would dominate slot 0.
  c->injectObstacles(makeObstacleMsg(stale, 1.15, 0.15, 1.0, 0.0, 0.2));

  MatrixXd reference = MatrixXd::Zero(3, c->nDim());
  reference(1, 0) = 1.15; reference(1, 1) = 0.15;
  reference(2, 0) = 100.0; reference(2, 1) = 100.0;    // out of grid

  MatrixXd obs_fill(static_cast<Eigen::Index>(2 * k), 3);
  c->fillObstacles(reference, obs_fill, now);
  MatrixXd obs_reduce(static_cast<Eigen::Index>(2 * k), 3);
  c->reduceCostmap(reference, obs_reduce);

  for (Eigen::Index r = 0; r < obs_fill.rows(); ++r) {
    EXPECT_NEAR(obs_fill(r, 0), obs_reduce(r, 0), kTol);
    EXPECT_NEAR(obs_fill(r, 1), obs_reduce(r, 1), kTol);
    EXPECT_NEAR(obs_fill(r, 2), obs_reduce(r, 2), kTol);
  }
}

// With predict_obstacles off, the fill ignores even a fresh tracked-obstacle
// message and reproduces the costmap-only result.
TEST_F(ProxMpcControllerTest, PredictDisabledIgnoresTrackedObstacles)
{
  auto c = makeConfigured(
  {
    rclcpp::Parameter("FollowPath.np", 2),
    rclcpp::Parameter("FollowPath.nc", 2),
    rclcpp::Parameter("FollowPath.max_obstacles", 2),
    rclcpp::Parameter("FollowPath.predict_obstacles", false),
    rclcpp::Parameter("FollowPath.robot_radius", 3.0),
  });
  const std::size_t k = 2;
  const rclcpp::Time now(1000, 0, RCL_ROS_TIME);

  fillCost(1.0, 0.0, 1.3, 0.3, nav2_costmap_2d::LETHAL_OBSTACLE);
  c->injectObstacles(makeObstacleMsg(now, 1.15, 0.15, 1.0, 0.0, 0.2));   // fresh, would dominate

  MatrixXd reference = MatrixXd::Zero(3, c->nDim());
  reference(1, 0) = 1.15; reference(1, 1) = 0.15;
  reference(2, 0) = 100.0; reference(2, 1) = 100.0;

  MatrixXd obs_fill(static_cast<Eigen::Index>(2 * k), 3);
  c->fillObstacles(reference, obs_fill, now);
  MatrixXd obs_reduce(static_cast<Eigen::Index>(2 * k), 3);
  c->reduceCostmap(reference, obs_reduce);

  for (Eigen::Index r = 0; r < obs_fill.rows(); ++r) {
    EXPECT_NEAR(obs_fill(r, 0), obs_reduce(r, 0), kTol);
    EXPECT_NEAR(obs_fill(r, 1), obs_reduce(r, 1), kTol);
    EXPECT_NEAR(obs_fill(r, 2), obs_reduce(r, 2), kTol);
  }
}

// The brake ramp seeds from the server-measured velocity, not the last command:
// a cycle-1 solver failure while the robot is moving (last command still zero)
// still ramps down from the measured speed instead of commanding an abrupt zero.
TEST_F(ProxMpcControllerTest, SolverFailureBrakesFromMeasuredVelocity)
{
  auto c = makeConfigured(
  {
    rclcpp::Parameter("FollowPath.max_int_iter_qp", 1),
    rclcpp::Parameter("FollowPath.max_ext_iter_qp", 1),
    rclcpp::Parameter("FollowPath.max_iter_sqp", 1),
    rclcpp::Parameter("FollowPath.max_solver_failures", 3),
  });
  c->activate();
  c->setPlan(makeStraightPlan(31, 0.2));
  ASSERT_NEAR(c->lastCmdV(), 0.0, kTol);   // last command is zero right after activate

  geometry_msgs::msg::Twist measured;
  measured.linear.x = 0.40;                // the robot is actually moving
  const auto cmd = c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), measured, nullptr);

  ASSERT_NE(c->mpc()->qp_info.status, proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED);
  // Ramp from the measured 0.40, not an abrupt 0 off the stale last command.
  EXPECT_NEAR(cmd.twist.linear.x, 0.40 - kModelDecel * 0.1, 1e-6);
}

// A non-finite measured velocity must never survive the brake ramp: +/-inf
// compares as moving, so an unguarded ramp would subtract a finite step from
// infinity and publish an infinite command on the safety path. NaN, +inf and
// -inf all yield exactly zero, on both the linear and the angular channel.
TEST_F(ProxMpcControllerTest, SolverFailureNeutralizesNonFiniteMeasuredVelocity)
{
  auto c = makeConfigured(
  {
    rclcpp::Parameter("FollowPath.max_int_iter_qp", 1),
    rclcpp::Parameter("FollowPath.max_ext_iter_qp", 1),
    rclcpp::Parameter("FollowPath.max_iter_sqp", 1),
    rclcpp::Parameter("FollowPath.max_solver_failures", 10),
  });
  c->activate();
  c->setPlan(makeStraightPlan(31, 0.2));

  const double inf = std::numeric_limits<double>::infinity();
  for (const double bad : {inf, -inf, std::numeric_limits<double>::quiet_NaN()}) {
    geometry_msgs::msg::Twist measured;
    measured.linear.x = bad;
    measured.angular.z = bad;
    const auto cmd = c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), measured, nullptr);
    ASSERT_NE(c->mpc()->qp_info.status, proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED);
    EXPECT_DOUBLE_EQ(cmd.twist.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(cmd.twist.angular.z, 0.0);
  }
}

// Structurally invalid horizon sizing (np or nc < 1, or dt <= 0) fails configure
// with a ControllerException rather than wrapping into an astronomical size_t
// allocation or dividing by zero.
TEST_F(ProxMpcControllerTest, ConfigureThrowsOnInvalidSizing)
{
  {
    auto c = makeUnconfigured({rclcpp::Parameter("FollowPath.np", 0)});
    EXPECT_THROW(
      c->configure(node_, "FollowPath", tf_, costmap_ros_), nav2_core::ControllerException);
  }
  {
    auto c = makeUnconfigured({rclcpp::Parameter("FollowPath.nc", -1)});
    EXPECT_THROW(
      c->configure(node_, "FollowPath", tf_, costmap_ros_), nav2_core::ControllerException);
  }
  {
    auto c = makeUnconfigured({rclcpp::Parameter("FollowPath.dt", 0.0)});
    EXPECT_THROW(
      c->configure(node_, "FollowPath", tf_, costmap_ros_), nav2_core::ControllerException);
  }
}

// Out-of-range tuning parameters are clamped (not fatal): the controller still
// configures, and an observable clamp (max_obstacles < 0 -> 0) is applied.
TEST_F(ProxMpcControllerTest, ConfigureClampsOutOfRangeTuning)
{
  std::shared_ptr<TestableProxMpcController> c;
  ASSERT_NO_THROW(
    c = makeConfigured(
  {
    rclcpp::Parameter("FollowPath.q_pos", -1.0),                    // negative weight -> floored
    rclcpp::Parameter("FollowPath.w_weight", -5.0),
    rclcpp::Parameter("FollowPath.cbf_gamma", 2.0),                 // > 1 -> clamped into (0, 1]
    rclcpp::Parameter("FollowPath.costmap_cost_threshold", 999),    // > 254 -> clamped
    rclcpp::Parameter("FollowPath.safety_margin", -0.2),            // < 0 -> floored
    rclcpp::Parameter("FollowPath.robot_radius", -0.5),
    rclcpp::Parameter("FollowPath.obstacle_cluster_radius", -0.3),
    rclcpp::Parameter("FollowPath.max_solve_time", -1.0),           // < 0 -> 0 (disabled)
    rclcpp::Parameter("FollowPath.max_obstacles", -3),              // < 0 -> 0 (observable)
    }));
  ASSERT_NE(c, nullptr);
  EXPECT_EQ(c->mpc()->getMaxObs(), 0u);   // max_obstacles clamped to 0
}

// The solver iteration caps reach the core as size_t, so a negative value wraps
// to an astronomical bound (an effectively unbounded SQP loop), and ProxQP
// rejects a zero cap outright. Both are structural, like the horizon sizing, so
// configure() fails with a ControllerException rather than degrading silently to
// a one-iteration QP that can never converge.
TEST_F(ProxMpcControllerTest, ConfigureThrowsOnInvalidSolverIterationCaps)
{
  for (const auto & bad : {
    rclcpp::Parameter("FollowPath.max_int_iter_qp", -1),
    rclcpp::Parameter("FollowPath.max_int_iter_qp", 0),
    rclcpp::Parameter("FollowPath.max_int_iter_qp", -100),
    rclcpp::Parameter("FollowPath.max_ext_iter_qp", -1),
    rclcpp::Parameter("FollowPath.max_ext_iter_qp", 0),
    rclcpp::Parameter("FollowPath.max_ext_iter_qp", -100),
    rclcpp::Parameter("FollowPath.max_iter_sqp", -1),
    rclcpp::Parameter("FollowPath.max_iter_sqp", 0),
    rclcpp::Parameter("FollowPath.max_iter_sqp", -100),
  })
  {
    auto c = makeUnconfigured({bad});
    EXPECT_THROW(
      c->configure(node_, "FollowPath", tf_, costmap_ros_), nav2_core::ControllerException);
  }
}

// Tracked obstacles with a non-finite field or a negative radius (untrusted input)
// are dropped before they reach the QP rows; the slots fall back to the
// costmap-only fill (here an empty costmap, so every slot stays at the sentinel).
TEST_F(ProxMpcControllerTest, NonFiniteObstacleIsDropped)
{
  auto c = makeConfigured(
  {
    rclcpp::Parameter("FollowPath.np", 2),
    rclcpp::Parameter("FollowPath.nc", 2),
    rclcpp::Parameter("FollowPath.max_obstacles", 1),
    rclcpp::Parameter("FollowPath.predict_obstacles", true),
    rclcpp::Parameter("FollowPath.dynamic_speed_threshold", 0.05),
  });
  const std::size_t k = 1;
  const rclcpp::Time now(1000, 0, RCL_ROS_TIME);

  auto msg = std::make_shared<prox_mpc_msgs::msg::ObstacleArray>();
  msg->header.frame_id = "map";
  msg->header.stamp = now;
  prox_mpc_msgs::msg::Obstacle bad_pos;        // NaN position: dropped
  bad_pos.id = 1;
  bad_pos.position.x = std::numeric_limits<double>::quiet_NaN();
  bad_pos.velocity.x = 1.0;
  bad_pos.radius = 0.2;
  prox_mpc_msgs::msg::Obstacle bad_radius;      // negative radius: dropped
  bad_radius.id = 2;
  bad_radius.position.x = 0.3;
  bad_radius.velocity.x = 1.0;
  bad_radius.radius = -1.0;
  msg->obstacles.push_back(bad_pos);
  msg->obstacles.push_back(bad_radius);
  c->injectObstacles(msg);

  MatrixXd reference = MatrixXd::Zero(3, c->nDim());
  reference(1, 0) = 0.2;
  reference(2, 0) = 0.4;

  MatrixXd obs(static_cast<Eigen::Index>(2 * k), 3);
  c->fillObstacles(reference, obs, now);

  for (Eigen::Index r = 0; r < obs.rows(); ++r) {
    EXPECT_NEAR(obs(r, 0), prox_mpc::MPC::kObsFarSentinel, kTol);
  }
}

// Tracker-sampled curved prediction drives the obs rows: with samples pinning
// the obstacle's future at x = 2 and advancing +y, the constraint positions
// follow the sampled arc while the straight ray (position + velocity * dt_k)
// would advance along +x at y = 0.
TEST_F(ProxMpcControllerTest, CurvedSamplesDriveConstraintRows)
{
  auto c = makeConfigured(
  {
    rclcpp::Parameter("FollowPath.np", 5),
    rclcpp::Parameter("FollowPath.nc", 5),
    rclcpp::Parameter("FollowPath.max_obstacles", 1),
    rclcpp::Parameter("FollowPath.predict_obstacles", true),
    rclcpp::Parameter("FollowPath.dynamic_speed_threshold", 0.05),
    rclcpp::Parameter("FollowPath.robot_radius", 0.5),
    rclcpp::Parameter("FollowPath.safety_margin", 0.1),
  });
  const std::size_t np = 5;
  const double dt = 0.1;
  const rclcpp::Time now(1000, 0, RCL_ROS_TIME);

  // Velocity says +x (passes the dynamic gate); the sampled prediction turns
  // the track to +y instead.
  auto msg = makeObstacleMsg(now, 2.0, 0.0, 1.0, 0.0, 0.2);
  std::vector<std::array<double, 2>> samples;
  for (int k = 1; k <= 25; ++k) {
    samples.push_back({2.0, 0.1 * static_cast<double>(k)});
  }
  addPredictedSamples(*msg, samples, 0.1);
  c->injectObstacles(msg);

  MatrixXd reference = MatrixXd::Zero(static_cast<Eigen::Index>(np + 1), c->nDim());
  for (std::size_t i = 0; i <= np; ++i) {
    reference(static_cast<Eigen::Index>(i), 0) = static_cast<double>(i) * 0.2;
  }
  MatrixXd obs(static_cast<Eigen::Index>(np), 3);
  c->fillObstacles(reference, obs, now);

  for (std::size_t node = 0; node < np; ++node) {
    const double dt_k = static_cast<double>(node + 1) * dt;
    const Eigen::Index row = static_cast<Eigen::Index>(node);
    EXPECT_NEAR(obs(row, 0), 2.0, 1e-9);                             // arc: x pinned
    EXPECT_NEAR(obs(row, 1), 0.1 * static_cast<double>(node + 1), 1e-9);   // arc: +y
    EXPECT_NEAR(obs(row, 2), 0.5 + 0.2 + 0.1, 1e-9);                 // growth default 0
    // Far from the straight ray (2 + dt_k, 0) the legacy fill would produce.
    EXPECT_GT(std::hypot(obs(row, 0) - (2.0 + dt_k), obs(row, 1)), 0.05);
  }
}

// The sampled polyline is interpolated between samples (anchored on the
// current position before the first sample) and extrapolated along the last
// segment's direction past the span; a single-sample polyline extrapolates
// along the position-to-sample direction.
TEST_F(ProxMpcControllerTest, SampledPredictionInterpolatesAndExtrapolates)
{
  auto c = makeConfigured(
  {
    rclcpp::Parameter("FollowPath.np", 5),
    rclcpp::Parameter("FollowPath.nc", 5),
    rclcpp::Parameter("FollowPath.max_obstacles", 1),
    rclcpp::Parameter("FollowPath.predict_obstacles", true),
    rclcpp::Parameter("FollowPath.dynamic_speed_threshold", 0.05),
  });
  const std::size_t np = 5;
  const rclcpp::Time now(1000, 0, RCL_ROS_TIME);
  MatrixXd reference = MatrixXd::Zero(static_cast<Eigen::Index>(np + 1), c->nDim());
  MatrixXd obs(static_cast<Eigen::Index>(np), 3);

  // Two samples 0.2 s apart from an obstacle at (2, 0): with dt = 0.1 the
  // nodes fall mid-segment, on a sample, at the span end, and past it.
  auto msg = makeObstacleMsg(now, 2.0, 0.0, 1.0, 0.0, 0.2);
  addPredictedSamples(*msg, {{2.2, 0.1}, {2.4, 0.4}}, 0.2);
  c->injectObstacles(msg);
  c->fillObstacles(reference, obs, now);

  EXPECT_NEAR(obs(0, 0), 2.1, 1e-9);    // t = 0.1: midpoint of (2, 0) and sample 1
  EXPECT_NEAR(obs(0, 1), 0.05, 1e-9);
  EXPECT_NEAR(obs(1, 0), 2.2, 1e-9);    // t = 0.2: exactly sample 1
  EXPECT_NEAR(obs(1, 1), 0.1, 1e-9);
  EXPECT_NEAR(obs(2, 0), 2.3, 1e-9);    // t = 0.3: midpoint of samples 1 and 2
  EXPECT_NEAR(obs(2, 1), 0.25, 1e-9);
  EXPECT_NEAR(obs(3, 0), 2.4, 1e-9);    // t = 0.4 (span end): exactly sample 2
  EXPECT_NEAR(obs(3, 1), 0.4, 1e-9);
  EXPECT_NEAR(obs(4, 0), 2.5, 1e-9);    // t = 0.5: extrapolated along the last segment
  EXPECT_NEAR(obs(4, 1), 0.55, 1e-9);

  // Single sample: past the 0.2 s span the ray runs from the current position
  // through that sample.
  auto msg1 = makeObstacleMsg(now, 2.0, 0.0, 1.0, 0.0, 0.2);
  addPredictedSamples(*msg1, {{2.2, 0.1}}, 0.2);
  c->injectObstacles(msg1);
  c->fillObstacles(reference, obs, now);

  EXPECT_NEAR(obs(0, 0), 2.1, 1e-9);    // t = 0.1: midpoint of (2, 0) and the sample
  EXPECT_NEAR(obs(0, 1), 0.05, 1e-9);
  EXPECT_NEAR(obs(1, 0), 2.2, 1e-9);    // t = 0.2 (span end): the sample itself
  EXPECT_NEAR(obs(1, 1), 0.1, 1e-9);
  EXPECT_NEAR(obs(2, 0), 2.3, 1e-9);    // t = 0.3..0.5: extrapolated
  EXPECT_NEAR(obs(2, 1), 0.15, 1e-9);
  EXPECT_NEAR(obs(3, 0), 2.4, 1e-9);
  EXPECT_NEAR(obs(3, 1), 0.2, 1e-9);
  EXPECT_NEAR(obs(4, 0), 2.5, 1e-9);
  EXPECT_NEAR(obs(4, 1), 0.25, 1e-9);
}

// An empty predicted_positions (old publisher, prediction disabled, or a
// tracker with prediction_steps 0) reproduces the straight-ray fill
// bit-for-bit: exact double equality against p = position + velocity * dt_k
// and d_safe = robot_radius + radius + margin + growth * dt_k.
TEST_F(ProxMpcControllerTest, EmptyPredictedPositionsMatchStraightRayBitForBit)
{
  auto c = makeConfigured(
  {
    rclcpp::Parameter("FollowPath.np", 5),
    rclcpp::Parameter("FollowPath.nc", 5),
    rclcpp::Parameter("FollowPath.dt", 0.1),
    rclcpp::Parameter("FollowPath.max_obstacles", 1),
    rclcpp::Parameter("FollowPath.predict_obstacles", true),
    rclcpp::Parameter("FollowPath.dynamic_speed_threshold", 0.05),
    rclcpp::Parameter("FollowPath.robot_radius", 0.5),
    rclcpp::Parameter("FollowPath.safety_margin", 0.1),
    rclcpp::Parameter("FollowPath.prediction_uncertainty_growth", 0.1),
  });
  const std::size_t np = 5;
  const rclcpp::Time now(1000, 0, RCL_ROS_TIME);

  // makeObstacleMsg leaves predicted_positions empty and prediction_dt 0.0,
  // exactly what an old (pre-IMM) tracker publishes. Obstacle frame == costmap
  // global frame, so the rigid transform is the exact identity.
  c->injectObstacles(makeObstacleMsg(now, 2.0, -1.0, 0.7, 0.3, 0.2));

  MatrixXd reference = MatrixXd::Zero(static_cast<Eigen::Index>(np + 1), c->nDim());
  MatrixXd obs(static_cast<Eigen::Index>(np), 3);
  c->fillObstacles(reference, obs, now);

  for (std::size_t node = 0; node < np; ++node) {
    // Same expression as the fill: dt_k = (node + 1) * dt + age, age = 0 here.
    const double dt_k = static_cast<double>(node + 1) * 0.1 + 0.0;
    const Eigen::Index row = static_cast<Eigen::Index>(node);
    EXPECT_EQ(obs(row, 0), 2.0 + 0.7 * dt_k);
    EXPECT_EQ(obs(row, 1), -1.0 + 0.3 * dt_k);
    EXPECT_EQ(obs(row, 2), 0.5 + 0.2 + 0.1 + 0.1 * dt_k);
  }
}

// A non-finite sample or an invalid prediction_dt invalidates the whole
// sampled prediction (fail-closed): the fill falls back to the straight ray
// bit-for-bit instead of consuming a corrupt polyline.
TEST_F(ProxMpcControllerTest, NonFiniteSamplesFallBackToStraightRay)
{
  auto c = makeConfigured(
  {
    rclcpp::Parameter("FollowPath.np", 5),
    rclcpp::Parameter("FollowPath.nc", 5),
    rclcpp::Parameter("FollowPath.max_obstacles", 1),
    rclcpp::Parameter("FollowPath.predict_obstacles", true),
    rclcpp::Parameter("FollowPath.dynamic_speed_threshold", 0.05),
    rclcpp::Parameter("FollowPath.robot_radius", 0.5),
    rclcpp::Parameter("FollowPath.safety_margin", 0.1),
  });
  const std::size_t np = 5;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const rclcpp::Time now(1000, 0, RCL_ROS_TIME);
  MatrixXd reference = MatrixXd::Zero(static_cast<Eigen::Index>(np + 1), c->nDim());
  MatrixXd obs(static_cast<Eigen::Index>(np), 3);

  auto with_nan_sample = makeObstacleMsg(now, 2.0, 0.0, 1.0, 0.0, 0.2);
  addPredictedSamples(*with_nan_sample, {{2.2, 0.1}, {nan, 0.4}}, 0.1);
  auto with_nan_dt = makeObstacleMsg(now, 2.0, 0.0, 1.0, 0.0, 0.2);
  addPredictedSamples(*with_nan_dt, {{2.2, 0.1}, {2.4, 0.4}}, nan);
  auto with_negative_dt = makeObstacleMsg(now, 2.0, 0.0, 1.0, 0.0, 0.2);
  addPredictedSamples(*with_negative_dt, {{2.2, 0.1}, {2.4, 0.4}}, -0.1);

  for (const auto & msg : {with_nan_sample, with_nan_dt, with_negative_dt}) {
    c->injectObstacles(msg);
    c->fillObstacles(reference, obs, now);
    for (std::size_t node = 0; node < np; ++node) {
      const double dt_k = static_cast<double>(node + 1) * 0.1 + 0.0;
      const Eigen::Index row = static_cast<Eigen::Index>(node);
      EXPECT_EQ(obs(row, 0), 2.0 + 1.0 * dt_k);    // straight ray, exact
      EXPECT_EQ(obs(row, 1), 0.0 + 0.0 * dt_k);
      EXPECT_EQ(obs(row, 2), 0.5 + 0.2 + 0.1 + 0.0 * dt_k);
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
