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
#include <cstdarg>
#include <cstddef>
#include <functional>
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
#include <rcutils/logging.h>
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
constexpr double kBicycleWheelbase = 1.6;  // bundled bicycle wheelbase [m]
// The bundled bicycles are not the default model, so a test that needs one names it.
constexpr const char * kBicyclePlugin = "prox_mpc_core/BicycleFrontAxle";
constexpr double kSteerRateBound = 1.0;    // bundled Bicycle u[1] bound [rad/s], bicycle.hpp:41

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
  int maxSolverFailures() const {return max_solver_failures_;}
  double safetyMargin() const {return safety_margin_;}
  double robotRadius() const {return robot_radius_;}
  double cbfGamma() const {return cbf_gamma_;}
  double obstacleClusterRadius() const {return obstacle_cluster_radius_;}
  double obstacleTimeout() const {return obstacle_timeout_;}
  double dynamicSpeedThreshold() const {return dynamic_speed_threshold_;}
  double predictionUncertaintyGrowth() const {return prediction_uncertainty_growth_;}
  double maxDynamicObstacleRadius() const {return max_dynamic_obstacle_radius_;}
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

// A conforming model with exactly one control: a speed channel and nothing else.
// The planar contract asks for one speed control and no more, so this must be
// accepted rather than rejected for lacking a second control's rate bound.
class SingleControlModel : public prox_mpc::Model
{
public:
  SingleControlModel()
  {
    setName("single_control");
    setN(3);
    setM(1);
    setIneq("u", 0, -3.0, 3.0);
    setIneq("du", 0, -0.5, 0.5);
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
  // Isotropic tolerance: the shape every goal checker Nav2 ships actually
  // produces (SimpleGoalChecker writes the same scalar into both fields).
  StubGoalChecker(double xy_tol, bool valid)
  : StubGoalChecker(xy_tol, xy_tol, valid) {}
  // Anisotropic tolerance, for a custom goal checker whose x and y tolerances
  // differ; the pair equal tests above can never exercise this shape.
  StubGoalChecker(double x_tol, double y_tol, bool valid)
  : x_tol_(x_tol), y_tol_(y_tol), valid_(valid) {}
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
    pose_tolerance.position.x = x_tol_;
    pose_tolerance.position.y = y_tol_;
    return valid_;
  }

private:
  double x_tol_;
  double y_tol_;
  bool valid_;
};

// Installs a custom rcutils output handler for its lifetime and records every
// WARN-or-worse message's fully formatted text, so a test can assert on a log
// message's content directly instead of only inferring that some warning
// fired. Restores the previous handler on destruction (RAII), so a test that
// constructs one on the stack cannot leak it into a later test.
class LogCapture
{
public:
  LogCapture()
  : previous_(rcutils_logging_get_output_handler())
  {
    messages_.clear();
    rcutils_logging_set_output_handler(&LogCapture::handle);
  }
  ~LogCapture() {rcutils_logging_set_output_handler(previous_);}

  static const std::vector<std::string> & messages() {return messages_;}

private:
  static void handle(
    const rcutils_log_location_t *, int severity, const char *,
    rcutils_time_point_value_t, const char * format, va_list * args)
  {
    if (severity < RCUTILS_LOG_SEVERITY_WARN) {return;}
    char buf[512];
    va_list copy;
    va_copy(copy, *args);
    vsnprintf(buf, sizeof(buf), format, copy);
    va_end(copy);
    messages_.emplace_back(buf);
  }

  rcutils_logging_output_handler_t previous_;
  static std::vector<std::string> messages_;
};
std::vector<std::string> LogCapture::messages_;
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

// configure() loads the named model, sizes the MPC, and reads the model's speed
// bound from its declared constraints.
TEST_F(ProxMpcControllerTest, ConfigureLoadsModelAndSizesMpc)
{
  auto c = makeConfigured(
    {rclcpp::Parameter("FollowPath.model_plugin", std::string(kBicyclePlugin))});
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

// nc > np is well-formed dead weight in the core (surplus controls drive no
// state transition) but is rejected as a configuration error at the plugin
// boundary, matching the existing np < 1 / nc < 1 / dt <= 0 fatal checks.
TEST_F(ProxMpcControllerTest, ConfigureThrowsWhenNcExceedsNp)
{
  auto c = makeUnconfigured(
    {rclcpp::Parameter("FollowPath.np", 5), rclcpp::Parameter("FollowPath.nc", 10)});
  EXPECT_THROW(
    c->configure(node_, "FollowPath", tf_, costmap_ros_),
    nav2_core::ControllerException);
}

// dt <= 0.0 is false for NaN, so the structural horizon-sizing gate needs an
// explicit finiteness check to catch it.
TEST_F(ProxMpcControllerTest, ConfigureThrowsOnNonFiniteDt)
{
  auto c = makeUnconfigured(
    {rclcpp::Parameter("FollowPath.dt", std::numeric_limits<double>::quiet_NaN())});
  EXPECT_THROW(
    c->configure(node_, "FollowPath", tf_, costmap_ros_),
    nav2_core::ControllerException);
}

// A non-finite value at a clamped parameter site (bare "<"/">" comparisons pass
// NaN through every range check) falls back to the parameter's declared
// default, for every parameter configure() guards this way. One row per
// clamp_low/clamp_range call site; each row reads back the result through
// whichever accessor observes where that configure()-local variable ends up (a
// stored member, or the MPC weight matrix it seeds).
TEST_F(ProxMpcControllerTest, ConfigureUsesDefaultForNonFiniteClampedParameter)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  using Read = std::function<double (const std::shared_ptr<TestableProxMpcController> &)>;
  struct Row
  {
    const char * param;
    double expected_default;
    Read read;
  };
  const Row rows[] = {
    {"q_pos", 10.0, [](const auto & c) {return c->mpc()->getQ()(0, 0);}},
    {"q_theta", 1.0, [](const auto & c) {return c->mpc()->getQ()(2, 2);}},
    {"s_factor", 2.0,
      [](const auto & c) {return c->mpc()->getS()(0, 0) / c->mpc()->getQ()(0, 0);}},
    {"r_weight", 0.1, [](const auto & c) {return c->mpc()->getR()(0, 0);}},
    {"w_weight", 100.0, [](const auto & c) {return c->mpc()->getW()(0, 0);}},
    {"safety_margin", 0.1, [](const auto & c) {return c->safetyMargin();}},
    {"robot_radius", 0.5, [](const auto & c) {return c->robotRadius();}},
    {"cbf_gamma", 1.0, [](const auto & c) {return c->cbfGamma();}},
    {"obstacle_cluster_radius", 0.3, [](const auto & c) {return c->obstacleClusterRadius();}},
    {"obstacle_timeout", 0.5, [](const auto & c) {return c->obstacleTimeout();}},
    {"dynamic_speed_threshold", 0.1, [](const auto & c) {return c->dynamicSpeedThreshold();}},
    {"prediction_uncertainty_growth", 0.0,
      [](const auto & c) {return c->predictionUncertaintyGrowth();}},
    {"max_dynamic_obstacle_radius", 0.0,
      [](const auto & c) {return c->maxDynamicObstacleRadius();}},
  };
  for (const auto & row : rows) {
    SCOPED_TRACE(row.param);
    auto c = makeConfigured({rclcpp::Parameter(std::string("FollowPath.") + row.param, nan)});
    ASSERT_NE(c->mpc(), nullptr);
    EXPECT_NEAR(row.read(c), row.expected_default, kTol);
  }
}

// max_solve_time is a configure()-local wall-clock budget forwarded into the
// core's MPC, which exposes no getter for it, so its default fallback is not
// directly observable from the plugin's public surface; the check available
// here is that a non-finite value does not throw configure() and does not
// propagate into a non-finite command. (max_solve_time's clamp floor and
// declared default are both 0.0, so even a getter could not distinguish "fell
// back to the default" from "clamped to the floor" for this one parameter --
// both paths return the same number.)
TEST_F(ProxMpcControllerTest, ConfigureAcceptsNonFiniteMaxSolveTime)
{
  std::shared_ptr<TestableProxMpcController> c;
  EXPECT_NO_THROW(
    c = makeConfigured(
      {rclcpp::Parameter("FollowPath.max_solve_time", std::numeric_limits<double>::quiet_NaN())}));
  ASSERT_NE(c, nullptr);
  c->activate();
  c->setPlan(makeStraightPlan(31, 0.2));
  const auto cmd = c->computeVelocityCommands(
    makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_TRUE(std::isfinite(cmd.twist.linear.x));
  EXPECT_TRUE(std::isfinite(cmd.twist.angular.z));
}

// A negative max_solver_failures is clamped to 0, matching the max_obstacles
// clamp shape, rather than left as an arbitrary negative value.
TEST_F(ProxMpcControllerTest, ConfigureClampsNegativeMaxSolverFailuresToZero)
{
  auto c = makeConfigured({rclcpp::Parameter("FollowPath.max_solver_failures", -5)});
  EXPECT_EQ(c->maxSolverFailures(), 0);
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

// A model with one control declares no second control-rate bound, and must not
// be rejected for it: the second body-twist channel only exists to be ramped
// when there is a control to source a rate from. Rejecting it would also have
// named an "angular control" the model never declared.
TEST_F(ProxMpcControllerTest, SingleControlModelReadsBounds)
{
  auto c = makeUnconfigured();
  SingleControlModel one;
  EXPECT_NO_THROW(c->readModelBounds(one, "test/SingleControl"));
  EXPECT_NEAR(c->vMax(), 3.0, kTol);
}

// robot_radius below the costmap footprint's circumscribed radius is
// warn-and-continue (DECISIONS.md's chosen option): configure() must not
// throw, and robot_radius must not be silently overwritten to match the
// footprint -- an operator running a deliberately tighter disc keeps it. The
// fixture's default square footprint (half-extent 0.5 m, padded to 0.51 m)
// and default robot_radius (0.5 m) already trigger this case; the warning
// text is asserted directly, naming both values, not merely inferred.
TEST_F(ProxMpcControllerTest, RobotRadiusWarnsWhenBelowFootprintCircumscribedRadius)
{
  LogCapture log;
  std::shared_ptr<TestableProxMpcController> c;
  ASSERT_NO_THROW(c = makeConfigured());
  ASSERT_NE(c, nullptr);
  EXPECT_NEAR(c->robotRadius(), 0.5, kTol);   // not overwritten to the footprint's radius

  bool found = false;
  for (const auto & msg : LogCapture::messages()) {
    if (msg.find("robot_radius 0.500 m") != std::string::npos &&
      msg.find("circumscribed radius 0.721 m") != std::string::npos)
    {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "expected a robot_radius/circumscribed-radius warning naming both values";
}

// A footprint within robot_radius configures silently: no warning is emitted,
// and the parameter is (still) not touched.
TEST_F(ProxMpcControllerTest, RobotRadiusConfiguresSilentlyWhenWithinFootprint)
{
  costmap_ros_->setRobotFootprint(makeSquareFootprint(0.1));   // circumscribed ~0.156 m

  LogCapture log;
  std::shared_ptr<TestableProxMpcController> c;
  ASSERT_NO_THROW(c = makeConfigured({rclcpp::Parameter("FollowPath.robot_radius", 0.5)}));
  ASSERT_NE(c, nullptr);
  EXPECT_NEAR(c->robotRadius(), 0.5, kTol);

  for (const auto & msg : LogCapture::messages()) {
    EXPECT_EQ(msg.find("robot_radius"), std::string::npos) << msg;
  }
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
// curvature; on a straight plan it stays zero. Read back from the state
// reference the controller hands the MPC. The front-axle model's inverse is
// asin(L*kappa), the rear-axle one's is atan(L*kappa); both are far above the
// threshold asserted here.
TEST_F(ProxMpcControllerTest, CurvatureSetsBicycleSteeringReference)
{
  auto c = makeConfigured(
    {rclcpp::Parameter("FollowPath.model_plugin", std::string(kBicyclePlugin))});
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
  EXPECT_GT(max_delta_arc, 0.2);                   // asin(L*kappa) = asin(0.8) = 0.927 rad
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

// The curvature estimator seeds its first heading delta from the plan's own
// sampled tangent, not the robot's yaw: on a straight plan (zero true
// curvature) a robot heading that differs from the path tangent must not by
// itself taper the cruise speed.
TEST_F(ProxMpcControllerTest, CurvatureGainIgnoresInitialHeadingMismatchOnStraightPlan)
{
  auto c = makeConfigured({rclcpp::Parameter("FollowPath.curvature_gain", 2.0)});
  c->activate();
  c->setPlan(makeStraightPlan(31, 0.2));           // straight plan along +x
  c->computeVelocityCommands(
    makePose(0.0, 0.0, 1.0), geometry_msgs::msg::Twist(), nullptr);   // heading far off the path
  EXPECT_NEAR(c->mpc()->getGoalU()(0, 0), c->desiredLinearVel(), 1e-6);
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

// The taper radius is the tolerance itself (std::min of the two axes), not the
// diagonal upstream's hypot(x, y) would read: SimpleGoalChecker writes the same
// scalar into both fields, so hypot(T, T) = sqrt(2) * T would start the taper
// about 41% too far from the goal. StubGoalChecker(t, true) is isotropic
// (equal x and y), so this pins the sqrt(2) defect specifically, which an
// equal-tolerance stub run at only one remaining distance cannot: one point
// where the true tolerance is already satisfied (no easing) but hypot's
// inflated radius would still ease, and one where both ease, but by
// numerically different factors. A short horizon (np * dt) keeps the
// horizon-based cruise taper from binding at these near-goal poses, isolating
// the goal-checker term.
TEST_F(ProxMpcControllerTest, GoalCheckerToleranceIsRadialNotHypotExpanded)
{
  auto c = makeConfigured(
  {
    rclcpp::Parameter("FollowPath.np", 2),
    rclcpp::Parameter("FollowPath.nc", 2),
    rclcpp::Parameter("FollowPath.dt", 0.01),
  });
  c->activate();
  c->setPlan(makeStraightPlan(6, 0.2));            // 1 m plan

  const double t = 0.1;
  StubGoalChecker checker(t, true);

  // remaining = 0.12 m: inside hypot(t, t) = 0.1414 m (would still ease under
  // the old reading) but outside the true radial tolerance t = 0.1 m.
  c->computeVelocityCommands(
    makePose(0.88, 0.0, 0.0), geometry_msgs::msg::Twist(), &checker);
  EXPECT_NEAR(c->mpc()->getGoalU()(0, 0), 1.0, kTol);   // desired_linear_vel, unreduced

  // remaining = 0.05 m: inside t, so both readings ease, but the radial factor
  // (0.05 / 0.1 = 0.5) differs from hypot's (0.05 / 0.14142 = 0.3536).
  c->computeVelocityCommands(
    makePose(0.95, 0.0, 0.0), geometry_msgs::msg::Twist(), &checker);
  EXPECT_NEAR(c->mpc()->getGoalU()(0, 0), 0.5, 1e-6);
}

// An anisotropic custom goal checker (x and y tolerances differ) is read
// through std::min of the two axes: the conservative reading for a checker
// whose y tolerance is tighter than its x, and distinct from both RPP's
// x-only reading and a hypot diagonal, neither of which an equal-tolerance
// stub can separate from std::min.
TEST_F(ProxMpcControllerTest, GoalCheckerToleranceReadsAnisotropicAsMinimum)
{
  auto c = makeConfigured(
  {
    rclcpp::Parameter("FollowPath.np", 2),
    rclcpp::Parameter("FollowPath.nc", 2),
    rclcpp::Parameter("FollowPath.dt", 0.01),
  });
  c->activate();
  c->setPlan(makeStraightPlan(6, 0.2));

  // remaining = 0.08 m; x_tol = 0.3, y_tol = 0.1. std::min gives 0.08 / 0.1 =
  // 0.8, versus RPP's x-only 0.08 / 0.3 = 0.2667 or a hypot diagonal
  // 0.08 / hypot(0.3, 0.1) = 0.253 -- both far from 0.8.
  StubGoalChecker checker(0.3, 0.1, true);
  c->computeVelocityCommands(
    makePose(0.92, 0.0, 0.0), geometry_msgs::msg::Twist(), &checker);
  EXPECT_NEAR(c->mpc()->getGoalU()(0, 0), 0.8, 1e-6);
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

// A non-finite plan pose beyond the sampled horizon must still reject the whole
// cycle: the transform loop validates every plan pose up front, not only the
// ones the horizon happens to sample. Without that check this corrupted point
// is never read by sample() (it sits well past the ~2 m horizon reach) and the
// cycle would drive normally, oblivious to the corruption elsewhere in the plan.
TEST_F(ProxMpcControllerTest, ComputeDeceleratesOnNonFinitePlanPoseBeyondHorizon)
{
  auto c = makeConfigured();
  c->activate();
  nav_msgs::msg::Path path = makeStraightPlan(50, 0.2);   // 10 m plan; horizon reaches ~2 m
  path.poses[40].pose.position.x = std::numeric_limits<double>::quiet_NaN();
  c->setPlan(path);

  const auto cmd = c->computeVelocityCommands(
    makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_TRUE(std::isfinite(cmd.twist.linear.x));
  EXPECT_TRUE(std::isfinite(cmd.twist.angular.z));
  EXPECT_NEAR(cmd.twist.linear.x, 0.0, kTol);   // braked from rest, not a normal cruise command
  EXPECT_EQ(c->failureCount(), 1);
}

// A leading run of duplicate plan positions collapses the first segment to zero
// length; the heading reference at that node must come from the next segment
// with positive length, not from atan2(0, 0) on the degenerate one.
TEST_F(ProxMpcControllerTest, ComputeSkipsDuplicateLeadingPlanPointForHeading)
{
  auto c = makeConfigured();
  c->activate();

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  auto addPose = [&](double x, double y) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = "map";
      ps.pose.position.x = x;
      ps.pose.position.y = y;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    };
  addPose(0.0, 0.0);
  addPose(0.0, 0.0);   // duplicate of the first pose
  addPose(0.0, 1.0);   // the path actually heads in +y
  c->setPlan(path);

  c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  const MatrixXd gx = c->mpc()->getGoalX();
  EXPECT_NEAR(gx(0, 2), M_PI / 2.0, 1e-6);
}

// A sparse (coarsely spaced) plan is projected onto its true nearest segment
// point, not snapped to whichever vertex happens to be nearest: at the default
// 0.05 m costmap resolution, an 8 m vertex spacing is far too coarse for
// vertex snapping and true segment projection to agree. The pre-Wave-2 code
// compared distance to vertices only, so it could jump the tracked progress
// straight to the far end of a long segment.
TEST_F(ProxMpcControllerTest, PlanProjectionSparsePlanUsesTrueSegmentPoint)
{
  auto c = makeConfigured();
  c->activate();
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (double x : {-4.0, 4.0}) {   // one 8 m segment, within the 10 m x 10 m grid
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose.position.x = x;
    ps.pose.orientation.w = 1.0;
    path.poses.push_back(ps);
  }
  c->setPlan(path);

  // 5 m along the segment (from x = -4), offset toward the far vertex so
  // vertex-only snapping picks (4, 0) (distance 3.007) over (-4, 0) (5.004).
  c->computeVelocityCommands(makePose(1.0, 0.2, 0.0), geometry_msgs::msg::Twist(), nullptr);

  // True segment projection: s0 = 5.0 m of 8.0 m, remaining = 3.0 m, well
  // inside the horizon (np * dt = 2.0 s at the default 1.0 m/s cruise), so the
  // cruise reference is not tapered. Vertex snapping to (4, 0) (s0 = 8.0 m)
  // would leave 0 m remaining and collapse the reference to zero.
  EXPECT_EQ(c->planIndex(), 0u);
  EXPECT_NEAR(c->mpc()->getGoalU()(0, 0), 1.0, 1e-6);
}

// A self-intersecting (looping) plan does not jump the tracked progress
// forward onto a distant later branch merely because that branch happens to
// have a VERTEX geometrically close to the robot: the true continuous
// projection onto the segment the robot is actually on stays closer, because
// it is not restricted to vertices.
TEST_F(ProxMpcControllerTest, PlanProjectionSelfIntersectingPlanAvoidsForwardJump)
{
  auto c = makeConfigured();
  c->activate();

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  auto addPose = [&](double x, double y) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = "map";
      ps.pose.position.x = x;
      ps.pose.position.y = y;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    };
  addPose(-4.0, 0.0);   // P0
  addPose(4.0, 0.0);    // P1: the segment the robot is actually on
  addPose(4.0, 4.0);    // P2
  addPose(0.0, 1.2);    // P3: a later branch looping back near the robot
  addPose(0.0, 4.0);    // P4
  c->setPlan(path);

  // On segment P0-P1, offset 0.05 m: the nearest VERTEX is P3 (distance
  // 1.524 m), closer than either P0 (5.0 m) or P1 (3.0 m), but the nearest
  // point on any segment is on P0-P1 itself, 0.05 m away.
  c->computeVelocityCommands(makePose(1.0, 0.05, 0.0), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_EQ(c->planIndex(), 0u);   // stays on segment 0, not the P2-P3 branch (index 2)
}

// A non-converged solve decelerates the last command at the model deceleration
// limit, then escalates to NoValidControl once the failure budget is spent.
//
// The brake ramps the model's own controls through toTwist() rather than
// ramping the measured twist directly, so the bicycle's angular.z is not a
// twist-space ramp of the measured yaw rate: the second control is a steering
// rate, and the yaw rate the command carries is derived from the (decayed)
// steering state as v * sin(delta) / L. steeringState() is seeded non-zero so
// the decay (bounded by the model's u[1] steering-rate limit, not the du[1]
// steering-acceleration bound a twist-space ramp would have used) is what
// supplies the angle that mapping reads.
TEST_F(ProxMpcControllerTest, ComputeSolverFailureRampsThenEscalates)
{
  auto c = makeConfigured(
  {
    rclcpp::Parameter("FollowPath.model_plugin", std::string(kBicyclePlugin)),
    rclcpp::Parameter("FollowPath.max_int_iter_qp", 1),
    rclcpp::Parameter("FollowPath.max_ext_iter_qp", 1),
    rclcpp::Parameter("FollowPath.max_iter_sqp", 1),
    rclcpp::Parameter("FollowPath.max_solver_failures", 1),
  });
  c->activate();
  c->setPlan(makeStraightPlan(31, 0.2));
  c->steeringState() = 0.2;   // non-zero steering belief for the decay to move

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

  // The ramped control is v = max(0, v_meas - a_dec * dt); a_dec = 0.5, dt = 0.1
  // -> step 0.05. delta decays toward zero at the u[1] rate bound:
  // 0.2 - 1.0 * 0.1 = 0.1 rad. The front-axle model's twist is the base_link one,
  // so linear.x is v cos(delta) rather than the front-wheel speed v, and
  // angular.z is v sin(delta) / L rather than a ramp of the measured yaw rate.
  const double expected_delta = 0.2 - kSteerRateBound * 0.1;
  const double expected_v = 0.30 - kModelDecel * 0.1;
  EXPECT_NEAR(cmd.twist.linear.x, expected_v * std::cos(expected_delta), 1e-9);
  EXPECT_NEAR(
    cmd.twist.angular.z, expected_v * std::sin(expected_delta) / kBicycleWheelbase, 1e-9);
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

// The veto threshold matches upstream Nav2 (RPP collision_checker.cpp:150-154,
// MPPI cost_critic.hpp:71-78): INSCRIBED_INFLATED_OBSTACLE alone no longer
// vetoes, because a real polygon check has already been performed and an
// inflated cell is not by itself a collision; only LETHAL_OBSTACLE does.
TEST_F(ProxMpcControllerTest, FootprintVetoMatchesUpstreamLethalThreshold)
{
  {
    auto c = makeConfigured({rclcpp::Parameter("FollowPath.max_obstacles", 0)});
    c->activate();
    c->setPlan(makeStraightPlan(31, 0.2));
    fillCost(-0.6, -0.6, 0.6, 0.6, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    const auto cmd = c->computeVelocityCommands(
      makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
    EXPECT_GT(cmd.twist.linear.x, 0.0);          // no veto: command passes through
  }
  {
    auto c = makeConfigured({rclcpp::Parameter("FollowPath.max_obstacles", 0)});
    c->activate();
    c->setPlan(makeStraightPlan(31, 0.2));
    fillCost(-0.6, -0.6, 0.6, 0.6, nav2_costmap_2d::LETHAL_OBSTACLE);
    const auto cmd = c->computeVelocityCommands(
      makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
    EXPECT_NEAR(cmd.twist.linear.x, 0.0, kTol);  // vetoed: brakes
  }
}

// NO_INFORMATION does not veto when the layered costmap reports
// isTrackingUnknown() (RPP collision_checker.cpp:150-154, MPPI
// cost_critic.hpp:71-78): the robot is allowed to plan into and through
// genuinely unmeasured space rather than braking at its boundary.
// LayeredCostmap::isTrackingUnknown() reads the costmap's stored default value
// directly, so setting it is enough to flip the policy; the fixture otherwise
// starts every cell at FREE_SPACE (SetUp()).
TEST_F(ProxMpcControllerTest, FootprintVetoIgnoresUnknownWhenCostmapTracksUnknown)
{
  auto c = makeConfigured({rclcpp::Parameter("FollowPath.max_obstacles", 0)});
  c->activate();
  c->setPlan(makeStraightPlan(31, 0.2));
  costmap_ros_->getCostmap()->setDefaultValue(nav2_costmap_2d::NO_INFORMATION);
  fillCost(-0.6, -0.6, 0.6, 0.6, nav2_costmap_2d::NO_INFORMATION);

  const auto cmd = c->computeVelocityCommands(
    makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_GT(cmd.twist.linear.x, 0.0);            // no veto: command passes through
}

// The controller's veto comment (prox_mpc_controller.cpp, the footprint-veto
// block) and DECISIONS.md's node 9 refinement both describe an inherited
// upstream masking property: footprintCostAtPose takes the maximum cost under
// the footprint's perimeter, and since NO_INFORMATION (255) is numerically
// larger than LETHAL_OBSTACLE (254), a footprint spanning both was expected to
// read NO_INFORMATION and be reported clear. Measured directly against this
// nav2_costmap_2d release, that does not reproduce: lineCost does not let an
// unmeasured cell out-rank a lethal one on the same edge, in either traversal
// order, so a footprint spanning both is reported at LETHAL_OBSTACLE and still
// vetoes. This test pins the actually observed behaviour (see the stage
// report's findings for the discrepancy) rather than the documented-but-
// unverified one, so a future nav2 release that reintroduces the masking shows
// up as a failing test here instead of a silent divergence.
TEST_F(ProxMpcControllerTest, FootprintVetoStillFiresWhenLethalAdjoinsUnknown)
{
  auto c = makeConfigured({rclcpp::Parameter("FollowPath.max_obstacles", 0)});
  c->activate();
  c->setPlan(makeStraightPlan(31, 0.2));
  costmap_ros_->getCostmap()->setDefaultValue(nav2_costmap_2d::NO_INFORMATION);
  // Bottom half of the one-step-ahead footprint lethal, top half unknown: both
  // values are present under the perimeter regardless of the exact predicted
  // pose within it.
  fillCost(-0.6, -0.6, 0.6, 0.6, nav2_costmap_2d::LETHAL_OBSTACLE);
  fillCost(-0.6, 0.0, 0.6, 0.6, nav2_costmap_2d::NO_INFORMATION);

  const auto cmd = c->computeVelocityCommands(
    makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_NEAR(cmd.twist.linear.x, 0.0, kTol);    // vetoed: the lethal half still fires
}

// The threshold change (INSCRIBED_INFLATED_OBSTACLE -> LETHAL_OBSTACLE) still
// takes effect next to unknown space: since the checker resolves a mixed
// perimeter to the highest non-unknown cost present (the previous test's
// finding), an INSCRIBED_INFLATED_OBSTACLE cell mixed with NO_INFORMATION
// resolves to 253, which the old >= 253 threshold still vetoed but the new
// >= 254 one does not, independent of the masking property itself.
TEST_F(ProxMpcControllerTest, FootprintVetoThresholdChangeAppliesNextToUnknown)
{
  auto c = makeConfigured({rclcpp::Parameter("FollowPath.max_obstacles", 0)});
  c->activate();
  c->setPlan(makeStraightPlan(31, 0.2));
  costmap_ros_->getCostmap()->setDefaultValue(nav2_costmap_2d::NO_INFORMATION);
  fillCost(-0.6, -0.6, 0.6, 0.6, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  fillCost(-0.6, 0.0, 0.6, 0.6, nav2_costmap_2d::NO_INFORMATION);

  const auto cmd = c->computeVelocityCommands(
    makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_GT(cmd.twist.linear.x, 0.0);   // no veto: 253 no longer meets the threshold
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

// A Nav2 speed limit must never broaden the feasible set: it narrows the
// model's own bounds, it does not replace them. AsymmetricBounds declares an
// asymmetric reverse limit (v_min = -0.3, tighter than v_max = 2.0), which
// neither bundled model can express, so this is the only fixture that can show
// the defect: the old code applied every limit as model_->updateIneq("u", 0,
// -v_lim, v_lim), replacing v_min outright. A non-negative v_min is not
// reachable through the controller's own parameters (v_min is always < 0 for
// a model that can stop), so an asymmetric negative v_min is the strongest
// reachable form, together with the NO_SPEED_LIMIT restore path.
TEST_F(ProxMpcControllerTest, SpeedLimitPreservesAsymmetricLowerBound)
{
  auto c = makeRunning(
    {rclcpp::Parameter(
      "FollowPath.model_plugin", std::string("prox_mpc_test_models/AsymmetricBounds"))});

  c->setSpeedLimit(1.0, false);
  runCycle(c);
  EXPECT_NEAR(c->maxLinearVel(), 1.0, kTol);
  EXPECT_NEAR(c->model()->getIneq("u").at(0)[1], -0.3, kTol);   // lower bound untouched
  EXPECT_NEAR(c->model()->getIneq("u").at(0)[2], 1.0, kTol);    // upper bound narrowed

  // NO_SPEED_LIMIT restores the model's own upper bound; the lower bound must
  // stay at the model's -0.3, not widen to -v_max (-2.0).
  c->setSpeedLimit(0.0, false);
  runCycle(c);
  EXPECT_NEAR(c->maxLinearVel(), 2.0, kTol);
  EXPECT_NEAR(c->model()->getIneq("u").at(0)[1], -0.3, kTol);
  EXPECT_NEAR(c->model()->getIneq("u").at(0)[2], 2.0, kTol);
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
//
// The scan is centered on the MPC's own nominal trajectory (mpc()->getX()); no
// plan reference is passed, because the fill does not use one.
// Node j's window is centered on nominal row min(j + 2, Np) (the fill runs
// before solve() shifts the trajectory, so this reads one node ahead to
// compensate for that staleness). Np = 3 so nodes 0 and 1 read distinct rows
// (2 and 3); node 2 clamps to the same row as node 1.
TEST_F(ProxMpcControllerTest, ReduceCostmapClustersAndSentinels)
{
  // Np = 3, K = 2; a large robot radius forces the scan window to clamp.
  auto c = makeConfigured(
  {
    rclcpp::Parameter("FollowPath.np", 3),
    rclcpp::Parameter("FollowPath.nc", 3),
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

  const std::size_t np = 3;
  const std::size_t k = 2;
  MatrixXd nominal = MatrixXd::Zero(static_cast<Eigen::Index>(np + 1), c->nDim());
  nominal(2, 0) = 1.15;    // row read by node 0: inside the grid, near both clusters
  nominal(2, 1) = 1.0;
  nominal(3, 0) = 100.0;   // row read by nodes 1 and 2: outside the grid
  nominal(3, 1) = 100.0;
  c->mpc()->setX(nominal);

  MatrixXd obs(static_cast<Eigen::Index>(np * k), 3);
  c->reduceCostmap(obs);

  // Node 0: two distinct representatives, each carrying d_safe = radius + margin.
  EXPECT_LT(obs(0, 0), prox_mpc::MPC::kObsFarSentinel);
  EXPECT_LT(obs(1, 0), prox_mpc::MPC::kObsFarSentinel);
  EXPECT_NEAR(obs(0, 2), 3.0 + 0.1, kTol);
  EXPECT_NEAR(obs(1, 2), 3.0 + 0.1, kTol);
  const double sep = std::hypot(obs(0, 0) - obs(1, 0), obs(0, 1) - obs(1, 1));
  EXPECT_GE(sep, 0.3);             // representatives at least a cluster radius apart

  // Nodes 1 and 2: out of grid (nominal row 3), every slot stays at the sentinel.
  EXPECT_NEAR(obs(2, 0), prox_mpc::MPC::kObsFarSentinel, kTol);
  EXPECT_NEAR(obs(3, 0), prox_mpc::MPC::kObsFarSentinel, kTol);
  EXPECT_NEAR(obs(4, 0), prox_mpc::MPC::kObsFarSentinel, kTol);
  EXPECT_NEAR(obs(5, 0), prox_mpc::MPC::kObsFarSentinel, kTol);
}

// A static obstacle near the MPC's nominal predicted state, but well outside a
// reference-centered scan window, is picked up: the pre-Wave-2 code centered
// the search on reference(node+1), so a lethal cell beyond d_safe +
// obstacle_cluster_radius of the plan reference position never entered the
// window at all, no matter how close it was to where the solver actually
// predicted the robot would be.
TEST_F(ProxMpcControllerTest, ReduceCostmapCentersOnNominalNotReference)
{
  auto c = makeConfigured(
  {
    rclcpp::Parameter("FollowPath.np", 1),
    rclcpp::Parameter("FollowPath.nc", 1),
    rclcpp::Parameter("FollowPath.max_obstacles", 1),
    rclcpp::Parameter("FollowPath.robot_radius", 0.3),
    rclcpp::Parameter("FollowPath.safety_margin", 0.1),
    rclcpp::Parameter("FollowPath.obstacle_cluster_radius", 0.2),
  });
  // search_radius = d_safe + obstacle_cluster_radius = 0.4 + 0.2 = 0.6 m.
  fillCost(1.9, -0.1, 2.1, 0.1, nav2_costmap_2d::LETHAL_OBSTACLE);   // near (2, 0)

  MatrixXd nominal = MatrixXd::Zero(2, c->nDim());
  nominal(1, 0) = 2.0;   // node 0 reads row min(0 + 2, Np = 1) = 1: at the obstacle
  c->mpc()->setX(nominal);

  // reference stays at the origin: more than 0.6 m from the obstacle, so a
  // reference-centered window would never see it (2.0 m - 0.6 m margin).
  MatrixXd obs(1, 3);
  c->reduceCostmap(obs);

  EXPECT_LT(obs(0, 0), prox_mpc::MPC::kObsFarSentinel);
  EXPECT_NEAR(obs(0, 0), 2.0, 0.15);
}

// The obstacle-slot ranking keys on the earliest node an object is first seen
// at, ties broken by closest approach -- not on closest approach anywhere in
// the horizon. A long np is needed to separate the two rules: at the shipped
// np and scan radius, every candidate is first seen at the same node, so the
// tie-break alone reproduces the old (closest-approach) order and no fixture
// built at those defaults can tell the two rules apart. Object A is seen only
// at the early node 2, offset ~0.25 m from that node's scan center (moderate
// approach). Object B is seen only at the late node 15, offset ~0 m (the
// closer approach overall). Ranking on closest-approach-anywhere would give
// the single slot to B; ranking on earliest-encounter gives it to A.
TEST_F(ProxMpcControllerTest, ObstacleSlotRankingPrefersEarliestEncounter)
{
  auto c = makeConfigured(
  {
    rclcpp::Parameter("FollowPath.np", 20),
    rclcpp::Parameter("FollowPath.nc", 20),
    rclcpp::Parameter("FollowPath.max_obstacles", 1),
    rclcpp::Parameter("FollowPath.robot_radius", 0.3),
    rclcpp::Parameter("FollowPath.safety_margin", 0.1),
    rclcpp::Parameter("FollowPath.obstacle_cluster_radius", 0.3),
  });
  // search_radius = d_safe + obstacle_cluster_radius = 0.4 + 0.3 = 0.7 m. Both
  // clusters stay well inside the 10 m x 10 m grid (+/-5 m): a cluster placed
  // at the grid edge would silently fail to paint (worldToMap rejects it).
  fillCost(2.25, -0.05, 2.35, 0.05, nav2_costmap_2d::LETHAL_OBSTACLE);   // A: near (2.3, 0)
  fillCost(3.5, -0.05, 3.55, 0.05, nav2_costmap_2d::LETHAL_OBSTACLE);    // B: near (3.5, 0)

  const std::size_t np = 20;
  MatrixXd nominal = MatrixXd::Zero(static_cast<Eigen::Index>(np + 1), c->nDim());
  // node 2 reads row min(2 + 2, np) = 4: centered on A, ~0.25 m away.
  nominal(4, 0) = 2.0;
  // node 15 reads row min(15 + 2, np) = 17: centered on B, ~0 m away (closer).
  nominal(17, 0) = 3.5;
  // Every other node's row is left at (0, 0): more than 0.7 m from both A and
  // B, so no other node picks up either one.
  c->mpc()->setX(nominal);

  MatrixXd obs(static_cast<Eigen::Index>(np), 3);
  c->reduceCostmap(obs);

  EXPECT_LT(obs(2, 0), prox_mpc::MPC::kObsFarSentinel);      // node 2: A wins the slot
  EXPECT_NEAR(obs(2, 0), 2.3, 0.1);
  EXPECT_NEAR(obs(15, 0), prox_mpc::MPC::kObsFarSentinel, kTol);   // node 15: B did not
}

// The footprint the veto uses is read fresh each cycle rather than cached at
// configure() time -- it is the one "in force at the start of the cycle", the
// hoisted-out-of-the-lock read node 28 chose, and the same per-cycle read RPP
// and MPPI both perform to keep a runtime footprint update taking effect.
TEST_F(ProxMpcControllerTest, FootprintVetoUsesFootprintInForceAtCycleStart)
{
  auto c = makeConfigured({rclcpp::Parameter("FollowPath.max_obstacles", 0)});
  c->activate();
  c->setPlan(makeStraightPlan(31, 0.2));

  // A lethal block straddling the default footprint's right edge (half-extent
  // 0.5 m, padded to ~0.51 m): footprintCostAtPose checks the polygon
  // perimeter only, not its interior, so the block must cross an edge to be
  // seen at all. The much smaller footprint used below does not reach it.
  fillCost(0.4, -0.05, 0.6, 0.05, nav2_costmap_2d::LETHAL_OBSTACLE);

  const auto cmd_default = c->computeVelocityCommands(
    makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_NEAR(cmd_default.twist.linear.x, 0.0, kTol);   // default footprint reaches it: vetoed

  // Shrink the footprint before the next cycle; the veto must use this new
  // footprint, not the one configure() saw.
  costmap_ros_->setRobotFootprint(makeSquareFootprint(0.05));
  const auto cmd_small = c->computeVelocityCommands(
    makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_GT(cmd_small.twist.linear.x, 0.0);   // the shrunk footprint no longer reaches it
}

// The costmap-reduction path finds the same candidate obstacle set it did
// before the critical section was narrowed to the cell reads: the grid lock
// now closes and reopens once per node instead of once per fill, so this
// checks that the per-node scans it protects still each find their own
// cluster and nothing else. This is a regression guard on the invariant node
// 29 chose (the candidate set, not fill atomicity against a concurrent
// costmap write) rather than evidence the lock-scope change altered
// single-threaded behavior, which it does not: the same instructions run
// either way, just under a narrower held lock.
TEST_F(ProxMpcControllerTest, CostmapReductionCandidateSetUnaffectedByNarrowedLock)
{
  auto c = makeConfigured(
  {
    rclcpp::Parameter("FollowPath.np", 4),
    rclcpp::Parameter("FollowPath.nc", 4),
    rclcpp::Parameter("FollowPath.max_obstacles", 2),   // >= 2 slots: no cross-node competition
    rclcpp::Parameter("FollowPath.robot_radius", 0.3),
    rclcpp::Parameter("FollowPath.safety_margin", 0.1),
    rclcpp::Parameter("FollowPath.obstacle_cluster_radius", 0.2),
  });
  // search_radius = 0.4 + 0.2 = 0.6 m; each cluster sits inside exactly one
  // node's window and outside every other node's.
  fillCost(1.9, -0.05, 2.0, 0.05, nav2_costmap_2d::LETHAL_OBSTACLE);   // near (2, 0): node 0
  fillCost(3.9, -0.05, 4.0, 0.05, nav2_costmap_2d::LETHAL_OBSTACLE);   // near (4, 0): node 1

  const std::size_t np = 4;
  const std::size_t k = 2;
  MatrixXd nominal = MatrixXd::Zero(static_cast<Eigen::Index>(np + 1), c->nDim());
  nominal(2, 0) = 2.0;    // node 0 reads row min(0 + 2, np) = 2
  nominal(3, 0) = 4.0;    // node 1 reads row min(1 + 2, np) = 3
  nominal(4, 0) = -4.0;   // nodes 2 and 3 read row min(node + 2, np) = 4: no obstacle there
  c->mpc()->setX(nominal);

  MatrixXd obs(static_cast<Eigen::Index>(np * k), 3);
  c->reduceCostmap(obs);

  // The near-(2,0) object is seen only at node 0 and ranks first (earliest
  // first_node), winning slot 0; row = node * k + slot = 0 * 2 + 0 = 0.
  EXPECT_LT(obs(0, 0), prox_mpc::MPC::kObsFarSentinel);
  EXPECT_NEAR(obs(0, 0), 2.0, 0.1);
  // The near-(4,0) object is seen only at node 1 and ranks second, winning
  // slot 1; row = 1 * 2 + 1 = 3.
  EXPECT_LT(obs(3, 0), prox_mpc::MPC::kObsFarSentinel);
  EXPECT_NEAR(obs(3, 0), 4.0, 0.1);
  // Node 0's slot 1 and node 1's slot 0 were never assigned either object.
  EXPECT_NEAR(obs(1, 0), prox_mpc::MPC::kObsFarSentinel, kTol);
  EXPECT_NEAR(obs(2, 0), prox_mpc::MPC::kObsFarSentinel, kTol);
  // Nodes 2 and 3 (both centered far from either cluster) stay empty.
  for (Eigen::Index r = 4; r < obs.rows(); ++r) {
    EXPECT_NEAR(obs(r, 0), prox_mpc::MPC::kObsFarSentinel, kTol) << "row " << r;
  }
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
//
// The static scan is centered on the MPC's own nominal trajectory
// (mpc()->getX()), not on the reference argument: at Np = 2 both nodes read the
// same clamped nominal row (min(node + 2, Np) = 2 for node 0 and node 1 alike),
// so setting that one row near the static block reproduces both nodes seeing it.
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

  // A static lethal block near the nominal trajectory's node-0/node-1 row.
  fillCost(1.0, 0.8, 1.3, 1.2, nav2_costmap_2d::LETHAL_OBSTACLE);
  // A dynamic obstacle far away, so its exclusion disc does not cover the block.
  c->injectObstacles(makeObstacleMsg(now, 5.0, 5.0, 1.0, 0.0, 0.2));

  MatrixXd nominal = MatrixXd::Zero(3, c->nDim());
  nominal(2, 0) = 1.15; nominal(2, 1) = 1.0;   // row read by both node 0 and node 1
  c->mpc()->setX(nominal);

  MatrixXd reference = MatrixXd::Zero(3, c->nDim());
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
  c->reduceCostmap(obs_reduce);

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
  c->reduceCostmap(obs_reduce);

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

// The default (0.0) brake_period_s steps the ramp with the measured
// inter-cycle period, floored at dt_ = 0.1 s so a fast solver-failure cycle
// (the elapsed time since the diagnostics timestamp was last reset, here
// microseconds) never brakes faster than the configured design.
// AsymmetricBounds' asymmetric du[0] (-2.0, +0.5) makes the resulting step
// size, not just its sign, an observable proxy for which period was used.
TEST_F(ProxMpcControllerTest, BrakeMeasuredPeriodFloorsAtConfiguredDt)
{
  auto c = makeConfigured(
  {
    rclcpp::Parameter(
      "FollowPath.model_plugin", std::string("prox_mpc_test_models/AsymmetricBounds")),
    rclcpp::Parameter("FollowPath.max_int_iter_qp", 1),
    rclcpp::Parameter("FollowPath.max_ext_iter_qp", 1),
    rclcpp::Parameter("FollowPath.max_iter_sqp", 1),
    rclcpp::Parameter("FollowPath.max_solver_failures", 10),
  });
  c->activate();
  c->setPlan(makeStraightPlan(31, 0.2));

  geometry_msgs::msg::Twist measured;
  measured.linear.x = 1.0;
  const auto cmd = c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), measured, nullptr);
  ASSERT_NE(c->mpc()->qp_info.status, proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED);
  EXPECT_NEAR(cmd.twist.linear.x, 1.0 - 2.0 * 0.1, 1e-6);   // floor: |du_low| * dt_ = 2.0 * 0.1
}

// The measured period is capped at kMaxBrakePeriodFactor * dt_ = 0.2 s so a
// real stall does not collapse one ramp step into an abrupt stop. The
// non-finite-pose fail path is used deliberately: it calls make_brake()
// before this cycle's own diagnostics timestamp reset runs, so the elapsed
// time it measures is the real gap since the previous (converged) cycle,
// which the deliberate sleep below controls; the solver-failure path used by
// the floor test above cannot show this, because publishDiagnostics() resets
// the timestamp earlier in that same cycle, before make_brake() reads it.
TEST_F(ProxMpcControllerTest, BrakeMeasuredPeriodCapsAtTwiceConfiguredDt)
{
  auto c = makeConfigured(
    {rclcpp::Parameter(
      "FollowPath.model_plugin", std::string("prox_mpc_test_models/AsymmetricBounds"))});
  c->activate();
  c->setPlan(makeStraightPlan(31, 0.2));

  // A normal converged cycle, so publishDiagnostics() records this cycle's
  // wall-clock time as the reference point the next cycle's stall is measured
  // against.
  const auto ok = c->computeVelocityCommands(
    makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  ASSERT_TRUE(std::isfinite(ok.twist.linear.x));

  std::this_thread::sleep_for(std::chrono::milliseconds(500));   // far past 2 * dt_ = 0.2 s

  geometry_msgs::msg::Twist measured;
  measured.linear.x = 1.0;
  geometry_msgs::msg::PoseStamped bad = makePose(0.0, 0.0, 0.0);
  bad.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  const auto cmd = c->computeVelocityCommands(bad, measured, nullptr);

  // An unclamped ~0.5 s period would apply step = 2.0 * 0.5 = 1.0, saturating
  // the ramp at zero; the ceiling caps it at 2.0 * 0.2 = 0.4.
  EXPECT_NEAR(cmd.twist.linear.x, 1.0 - 2.0 * 0.2, 1e-6);
}

// A positive brake_period_s pins the ramp step and bypasses the measured
// period entirely, even across a real stall far longer than the pinned value.
TEST_F(ProxMpcControllerTest, BrakePinnedPeriodIgnoresElapsedTime)
{
  auto c = makeConfigured(
  {
    rclcpp::Parameter(
      "FollowPath.model_plugin", std::string("prox_mpc_test_models/AsymmetricBounds")),
    rclcpp::Parameter("FollowPath.brake_period_s", 0.05),
    rclcpp::Parameter("FollowPath.max_int_iter_qp", 1),
    rclcpp::Parameter("FollowPath.max_ext_iter_qp", 1),
    rclcpp::Parameter("FollowPath.max_iter_sqp", 1),
    rclcpp::Parameter("FollowPath.max_solver_failures", 10),
  });
  c->activate();
  c->setPlan(makeStraightPlan(31, 0.2));

  geometry_msgs::msg::Twist measured;
  measured.linear.x = 1.0;
  c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), measured, nullptr);

  std::this_thread::sleep_for(std::chrono::milliseconds(300));   // far past 0.05 s
  const auto cmd = c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), measured, nullptr);
  ASSERT_NE(c->mpc()->qp_info.status, proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED);
  EXPECT_NEAR(cmd.twist.linear.x, 1.0 - 2.0 * 0.05, 1e-6);
}

// The ramp saturates at zero rather than reversing: an over-large step (here,
// a deliberately oversized pinned period) shortens the stop instead of
// overshooting past zero into the opposite sign, which is what makes an
// uncapped or mis-tuned step benign rather than a new failure mode.
TEST_F(ProxMpcControllerTest, BrakeSaturatesAtZeroRatherThanReversing)
{
  auto c = makeConfigured(
  {
    rclcpp::Parameter(
      "FollowPath.model_plugin", std::string("prox_mpc_test_models/AsymmetricBounds")),
    rclcpp::Parameter("FollowPath.brake_period_s", 1.0),   // step = 2.0 * 1.0 = 2.0
    rclcpp::Parameter("FollowPath.max_int_iter_qp", 1),
    rclcpp::Parameter("FollowPath.max_ext_iter_qp", 1),
    rclcpp::Parameter("FollowPath.max_iter_sqp", 1),
    rclcpp::Parameter("FollowPath.max_solver_failures", 10),
  });
  c->activate();
  c->setPlan(makeStraightPlan(31, 0.2));

  geometry_msgs::msg::Twist measured;
  measured.linear.x = 0.5;      // step (2.0) is 4x the measured speed
  const auto cmd = c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), measured, nullptr);
  ASSERT_NE(c->mpc()->qp_info.status, proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED);
  EXPECT_DOUBLE_EQ(cmd.twist.linear.x, 0.0);    // saturates, never crosses to negative
}

// Each direction ramps at its own declared rate: braking a forward-moving
// robot uses du[0]'s lower bound magnitude (2.0), while returning to zero from
// a reversing robot uses its upper bound magnitude (0.5). brake_period_s is
// pinned so the comparison is not sensitive to test execution timing. Neither
// bundled model can show this because both declare symmetric du bounds.
TEST_F(ProxMpcControllerTest, BrakePreservesAsymmetricDecelerationBoundsPerDirection)
{
  auto c = makeConfigured(
  {
    rclcpp::Parameter(
      "FollowPath.model_plugin", std::string("prox_mpc_test_models/AsymmetricBounds")),
    rclcpp::Parameter("FollowPath.brake_period_s", 0.1),
    rclcpp::Parameter("FollowPath.max_int_iter_qp", 1),
    rclcpp::Parameter("FollowPath.max_ext_iter_qp", 1),
    rclcpp::Parameter("FollowPath.max_iter_sqp", 1),
    rclcpp::Parameter("FollowPath.max_solver_failures", 10),
  });
  c->activate();
  c->setPlan(makeStraightPlan(31, 0.2));

  geometry_msgs::msg::Twist forward;
  forward.linear.x = 1.0;
  const auto cmd_fwd = c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), forward, nullptr);
  ASSERT_NE(c->mpc()->qp_info.status, proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED);
  EXPECT_NEAR(cmd_fwd.twist.linear.x, 1.0 - 2.0 * 0.1, 1e-6);    // du[0] low = -2.0

  geometry_msgs::msg::Twist reverse;
  reverse.linear.x = -1.0;
  const auto cmd_rev = c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), reverse, nullptr);
  ASSERT_NE(c->mpc()->qp_info.status, proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED);
  EXPECT_NEAR(cmd_rev.twist.linear.x, -1.0 + 0.5 * 0.1, 1e-6);   // du[0] upp = +0.5
}

// The bicycle's brake maps through the model's own toTwist physics - for the
// front-axle model linear.x = v cos(delta) and angular.z = v sin(delta) / L, both
// the base_link twist - rather than a twist-space ramp of the measured yaw rate,
// and a pinned brake_period_s steps both the speed ramp and the steering decay by
// the same pinned amount rather than the measured period
// BrakeMeasuredPeriodClampedToConfiguredRange exercises.
TEST_F(ProxMpcControllerTest, BicycleBrakeMapsThroughToTwistWithPinnedPeriod)
{
  auto c = makeConfigured(
  {
    rclcpp::Parameter("FollowPath.model_plugin", std::string(kBicyclePlugin)),
    rclcpp::Parameter("FollowPath.brake_period_s", 0.2),
    rclcpp::Parameter("FollowPath.max_int_iter_qp", 1),
    rclcpp::Parameter("FollowPath.max_ext_iter_qp", 1),
    rclcpp::Parameter("FollowPath.max_iter_sqp", 1),
    rclcpp::Parameter("FollowPath.max_solver_failures", 10),
  });
  c->activate();
  c->setPlan(makeStraightPlan(31, 0.2));
  c->steeringState() = 0.5;

  geometry_msgs::msg::Twist measured;
  measured.linear.x = 1.0;
  const auto cmd = c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), measured, nullptr);
  ASSERT_NE(c->mpc()->qp_info.status, proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED);

  const double expected_v = 1.0 - kModelDecel * 0.2;             // du[0] bound, pinned period
  const double expected_delta = 0.5 - kSteerRateBound * 0.2;     // u[1] bound, pinned period
  EXPECT_NEAR(cmd.twist.linear.x, expected_v * std::cos(expected_delta), 1e-9);
  EXPECT_NEAR(
    cmd.twist.angular.z, expected_v * std::sin(expected_delta) / kBicycleWheelbase, 1e-9);
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
