// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Contract tests for the controller's Nav2-facing behaviour: the transactional
// solve/commit wiring at the plugin boundary, SolverDiagnostics' gate-aware
// `converged` field and its publisher, the model/Nav2 adapter contract
// (Model::getPlanarMapping(), read by readModelMapping()), and
// terminal-heading/reverse-travel tracking.
//
// This is a separate binary from test_prox_mpc_controller.cpp (which already
// covers configure(), the fail-safe branches, and the costmap/predictive
// obstacle fill in depth) rather than an addition to it, so the file can be
// built and run in isolation without disturbing that suite's own coverage.

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
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

#include <prox_mpc/proxqp.hpp>
#include <prox_mpc_msgs/msg/solver_diagnostics.hpp>

#include "prox_mpc_controller/prox_mpc_controller.hpp"

namespace
{
constexpr double kTol = 1e-9;
constexpr double kModelDecel = 0.5;   // bundled-model du bound [m/s^2, rad/s^2]
constexpr double kResolution = 0.05;
constexpr unsigned int kGridCells = 200u;
constexpr double kGridOrigin = -5.0;
constexpr const char * kFrontAxlePlugin = "prox_mpc_core/BicycleFrontAxle";
constexpr const char * kRearAxlePlugin = "prox_mpc_core/BicycleRearAxle";
constexpr double kBicycleWheelbase = 1.6;

// Exposes the protected surface these tests need to drive directly: the
// model/Nav2 adapter read (readModelMapping()), the cached mapping it
// produces, and the loaded model/mpc handles. White-box, no production change.
class TestableProxMpcController : public prox_mpc_controller::ProxMpcController
{
public:
  using ProxMpcController::readModelMapping;
  using ProxMpcController::keepOutShift;
  using ProxMpcController::fillStaticObstacles;
  using ProxMpcController::n_;
  using ProxMpcController::m_;

  std::shared_ptr<prox_mpc::MPC> mpc() const {return mpc_;}
  std::shared_ptr<prox_mpc::Model> model() const {return model_;}
  double wheelbase() const {return wheelbase_;}
  double refOffsetX() const {return ref_offset_x_;}
  double refOffsetY() const {return ref_offset_y_;}
  bool hasSteering() const {return has_steering_;}
  std::size_t idxX() const {return idx_x_;}
  std::size_t idxY() const {return idx_y_;}
  std::size_t idxYaw() const {return idx_yaw_;}
  std::size_t idxV() const {return idx_v_;}
  std::size_t idxSteer() const {return idx_steer_;}
  int failureCount() const {return failure_count_;}
  std::size_t maxObstacles() const {return static_cast<std::size_t>(max_obstacles_);}
};

// Local, unregistered models exercising readModelMapping()'s rejection paths.
// They never reach a solve, so their dynamics hooks are stubs -- matching the
// existing NoDuBoundModel/SingleControlModel pattern in
// test_prox_mpc_controller.cpp, which keeps a loader-path fixture in
// prox_mpc_test_models and a readModelMapping()-only one local to the test.

class TwoStateModel : public prox_mpc::Model
{
public:
  TwoStateModel()
  {
    setName("two_state");
    setN(2);
    setM(1);
  }
  void updatec(double, VectorXd) override {}
  void updateA(double) override {}
  void updateB() override {}
};

// Declares its state dimension and forgets its control dimension. Model's
// control count is a ModelInfo default member initialiser (0) and Model
// declares no constructor, so a subclass that never calls setM leaves it there;
// setM's own m == 0 rejection only covers an explicit zero.
class NoControlModel : public prox_mpc::Model
{
public:
  NoControlModel()
  {
    setName("no_control");
    setN(3);   // clears the n_ < 3 gate, so m_ < 1 is the one under test
  }
  void updatec(double, VectorXd) override {}
  void updateA(double) override {}
  void updateB() override {}
};

// Maps y to the same state index as x -- an internally inconsistent mapping.
class DuplicatePlanarIndexModel : public prox_mpc::Model
{
public:
  DuplicatePlanarIndexModel()
  {
    setName("duplicate_planar_index");
    setN(3);
    setM(1);
  }
  prox_mpc::PlanarMapping getPlanarMapping() const override
  {
    prox_mpc::PlanarMapping mapping;
    mapping.idx_y = mapping.idx_x;   // 0 == 0: collides
    return mapping;
  }
  void updatec(double, VectorXd) override {}
  void updateA(double) override {}
  void updateB() override {}
};

// Carries a steering state (n > 3) but declares no wheelbase.
class SteeringNoWheelbaseModel : public prox_mpc::Model
{
public:
  SteeringNoWheelbaseModel()
  {
    setName("steering_no_wheelbase");
    setN(4);
    setM(2);
  }
  void updatec(double, VectorXd) override {}
  void updateA(double) override {}
  void updateB() override {}
};

// Declares a steering state with a wheelbase, but a lateral reference offset
// -- the steering inverse is derived for a reference point on the body x axis.
class SteeringLateralOffsetModel : public prox_mpc::Model
{
public:
  SteeringLateralOffsetModel()
  {
    setName("steering_lateral_offset");
    setN(4);
    setM(2);
  }
  prox_mpc::PlanarMapping getPlanarMapping() const override
  {
    prox_mpc::PlanarMapping mapping;
    mapping.idx_steering = 3;
    mapping.wheelbase = 1.6;
    mapping.ref_offset_y = 0.2;   // lateral: unsupported
    return mapping;
  }
  void updatec(double, VectorXd) override {}
  void updateA(double) override {}
  void updateB() override {}
};

// A conforming three-state, two-control model (no steering channel) that
// overrides nothing: the default getPlanarMapping() must still let it
// configure and drive. n <= 3 is the case that actually "still drives" with
// no override -- a model with n > 3 and no override also reproduces the
// pre-hook convention by inferring a steering channel at state 3, but is then
// correctly rejected for declaring no wheelbase
// (ReadModelMappingRejectsSteeringWithoutWheelbase below, and
// migration.md: "rejects at configure() a model that carries a steering
// angle without declaring a usable wheelbase").
class BareThreeStateDrivableModel : public prox_mpc::Model
{
public:
  BareThreeStateDrivableModel()
  {
    setName("bare_three_state_drivable");
    setN(3);
    setM(2);
    setA(MatrixXd::Zero(getN(), getN()));
    setB(MatrixXd::Zero(getN(), getM()));
    setc(VectorXd::Zero(getN()));
    setIneq("u", 0, -3.0, 3.0);
    setIneq("u", 1, -1.0, 1.0);
    setIneq("du", 0, -0.5, 0.5);
    setIneq("du", 1, -0.5, 0.5);
    setObsAvoid(false);
  }
  void updatec(double dt, VectorXd x_next) override
  {
    c << getX()(0) - x_next(0) + dt * getU()(0) * std::cos(getX()(2)),
      getX()(1) - x_next(1) + dt * getU()(0) * std::sin(getX()(2)),
      getX()(2) - x_next(2) + dt * getU()(1);
  }
  void updateA(double dt) override
  {
    const double th = getX()(2);
    const double v = getU()(0);
    A << 1.0, 0.0, -dt * v * std::sin(th),
      0.0, 1.0, dt * v * std::cos(th),
      0.0, 0.0, 1.0;
  }
  void updateB() override
  {
    const double th = getX()(2);
    B << std::cos(th), 0.0,
      std::sin(th), 0.0,
      0.0, 1.0;
  }
};

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

std::vector<geometry_msgs::msg::Point> makeSquareFootprint(double half)
{
  std::vector<geometry_msgs::msg::Point> fp(4);
  fp[0].x = half; fp[0].y = half;
  fp[1].x = half; fp[1].y = -half;
  fp[2].x = -half; fp[2].y = -half;
  fp[3].x = -half; fp[3].y = half;
  return fp;
}

class StubGoalChecker : public nav2_core::GoalChecker
{
public:
  explicit StubGoalChecker(double yaw_tol)
  : yaw_tol_(yaw_tol) {}
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
    pose_tolerance.position.x = 1.0;
    pose_tolerance.position.y = 1.0;
    pose_tolerance.orientation.z = std::sin(yaw_tol_ / 2.0);
    pose_tolerance.orientation.w = std::cos(yaw_tol_ / 2.0);
    return true;
  }

private:
  double yaw_tol_;
};
}  // namespace

class ControllerContractsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions cm_opts;
    cm_opts.arguments({"--ros-args", "-r", "__node:=prox_mpc_contracts_costmap"});
    cm_opts.parameter_overrides(
    {
      rclcpp::Parameter("global_frame", std::string("map")),
      rclcpp::Parameter("robot_base_frame", std::string("base_link")),
      rclcpp::Parameter("use_sim_time", false),
      rclcpp::Parameter("plugins", std::vector<std::string>{}),
    });
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(cm_opts);
    costmap_ros_->on_configure(rclcpp_lifecycle::State());

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

  rclcpp_lifecycle::LifecycleNode::SharedPtr makeNode(
    const std::vector<rclcpp::Parameter> & overrides)
  {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides(overrides);
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "prox_mpc_contracts_controller_" + std::to_string(node_counter_++), opts);
    nodes_.push_back(node);
    return node;
  }

  std::shared_ptr<TestableProxMpcController> makeUnconfigured(
    const std::vector<rclcpp::Parameter> & overrides = {})
  {
    node_ = makeNode(overrides);
    controller_ = std::make_shared<TestableProxMpcController>();
    return controller_;
  }

  std::shared_ptr<TestableProxMpcController> makeConfigured(
    const std::vector<rclcpp::Parameter> & overrides = {})
  {
    auto c = makeUnconfigured(overrides);
    c->configure(node_, "FollowPath", tf_, costmap_ros_);
    return c;
  }

  std::shared_ptr<TestableProxMpcController> makeRunning(
    const std::vector<rclcpp::Parameter> & overrides = {})
  {
    auto c = makeConfigured(overrides);
    c->activate();
    c->setPlan(makeStraightPlan(61, 0.2));
    return c;
  }

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

// --- A rejected candidate does not anchor the next cycle's rate constraint --
//
// The vetoed-candidate half of the transactional solve. In 1.0.0 a
// solved-but-vetoed candidate still advanced MPC's retained u0
// (mpc.cpp:148-150 at that release), so the next cycle's du rate constraint was
// anchored on a command the robot never received. make_brake()'s own twist
// output is sourced directly from the measured velocity argument (not from
// u0), so it cannot show this by itself; what can is whether the SOLVER's own
// rate bound on the FOLLOWING accepted cycle is anchored on the value
// setU0() recorded. It is engineered to be observable: measured.linear.x is
// set far above the plan's desired speed, so the accepted cycle's command is
// bound-saturated at the deceleration limit if (and only if) it is really
// anchored on the applied brake.
TEST_F(ControllerContractsTest, VetoedCandidateDoesNotAnchorNextCycleRateConstraint)
{
  auto c = makeRunning({rclcpp::Parameter("FollowPath.max_obstacles", 0)});

  // Cycle 1: lethal block over the one-step-ahead footprint vetoes the
  // candidate. The measured velocity (2.0 m/s) is deliberately far above the
  // plan's desired speed (default 1.0 m/s), so the two scenarios below are
  // easy to tell apart.
  fillCost(-0.6, -0.6, 0.6, 0.6, nav2_costmap_2d::LETHAL_OBSTACLE);
  geometry_msgs::msg::Twist measured;
  measured.linear.x = 2.0;
  const auto cmd1 = c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), measured, nullptr);
  const double expected_applied = 2.0 - kModelDecel * 0.1;   // 1.95, first-cycle period = dt
  EXPECT_NEAR(cmd1.twist.linear.x, expected_applied, 1e-6);

  // Cycle 2: clear the veto. If the rate constraint is anchored on the value
  // actually applied (1.95), and the reference wants much less, the QP is
  // bound-saturated at the deceleration limit: exactly 1.95 - 0.05 = 1.90.
  // If it were instead anchored on the vetoed candidate's own (unsent) u0 --
  // close to the low reference speed already -- this cycle would show no such
  // saturation and could jump straight toward the reference.
  fillCost(-0.6, -0.6, 0.6, 0.6, nav2_costmap_2d::FREE_SPACE);
  const auto cmd2 = c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), measured, nullptr);
  const double expected_saturated = expected_applied - kModelDecel * 0.1;   // 1.90
  EXPECT_NEAR(cmd2.twist.linear.x, expected_saturated, 5e-3);
}

// --- Model/Nav2 adapter contract (Model::getPlanarMapping()) ----------------
// --- and the "a model declaring nothing still drives" pin -------------------
//
// readModelMapping() reads the controller's OWN cached n_/m_ (populated by
// configure() from model_->getN()/getM() immediately before it is called,
// prox_mpc_controller.cpp), not the model argument's dimensions directly, so
// a direct call must set them first -- otherwise every one of these throws on
// the unconditional "declares 0 states" check regardless of what is actually
// being tested.
void callReadModelMapping(
  const std::shared_ptr<TestableProxMpcController> & c, prox_mpc::Model & model,
  const std::string & name)
{
  c->n_ = model.getN();
  c->m_ = model.getM();
  c->readModelMapping(model, name);
}

TEST_F(ControllerContractsTest, ReadModelMappingRejectsFewerThanThreeStates)
{
  auto c = makeUnconfigured();
  TwoStateModel model;
  EXPECT_THROW(callReadModelMapping(c, model, "test/TwoState"), nav2_core::ControllerException);
}

// setM(0) throws, but a model that never calls setM at all does not: ModelInfo
// default-initialises m to 0 and Model declares no constructor of its own, so
// the omission survives construction and readModelMapping() -- which runs
// before readModelBounds() -- is the first gate it reaches.
//
// The message is asserted, not merely the exception type. Every later mapping
// check compares an index against m_, so all of them also fire at m_ == 0 and
// any of them would satisfy an EXPECT_THROW: the default speed index 0 is
// "outside" a zero-length control vector. Only the message distinguishes being
// told the model declares no control from being pointed at a speed-index
// mapping the author never wrote.
TEST_F(ControllerContractsTest, ReadModelMappingRejectsAModelDeclaringNoControl)
{
  auto c = makeUnconfigured();
  NoControlModel model;
  try {
    callReadModelMapping(c, model, "test/NoControl");
    FAIL() << "a model declaring no control must not configure";
  } catch (const nav2_core::ControllerException & ex) {
    const std::string what(ex.what());
    EXPECT_NE(what.find("declares no control input"), std::string::npos) << what;
    EXPECT_NE(what.find("test/NoControl"), std::string::npos) << what;
  }
}

TEST_F(ControllerContractsTest, ReadModelMappingRejectsDuplicatePlanarIndices)
{
  auto c = makeUnconfigured();
  DuplicatePlanarIndexModel model;
  EXPECT_THROW(
    callReadModelMapping(c, model, "test/DuplicatePlanarIndex"),
    nav2_core::ControllerException);
}

TEST_F(ControllerContractsTest, ReadModelMappingRejectsSteeringWithoutWheelbase)
{
  auto c = makeUnconfigured();
  SteeringNoWheelbaseModel model;
  EXPECT_THROW(
    callReadModelMapping(c, model, "test/SteeringNoWheelbase"),
    nav2_core::ControllerException);
}

TEST_F(ControllerContractsTest, ReadModelMappingRejectsSteeringWithLateralOffset)
{
  auto c = makeUnconfigured();
  SteeringLateralOffsetModel model;
  EXPECT_THROW(
    callReadModelMapping(c, model, "test/SteeringLateralOffset"),
    nav2_core::ControllerException);
}

// A model whose planar position is not at state columns 0 and 1 configures with
// the in-loop keep-out term active. It used to be rejected here, because the
// solver's obstacle rows indexed those two columns directly; they now index the
// same declared mapping the controller reads, so there is nothing left to
// reject and the model drives with avoidance on.
//
// The keep-out is then checked where it is assembled: the half-plane normal has
// to land in the two state columns the model declares, and the column it
// declares as heading must carry none of it. A misplaced normal still produces
// a plausible trajectory, just one constrained on the wrong axes, so the
// assembled matrix is read rather than the solution.
TEST_F(ControllerContractsTest, PermutedModelDrivesWithObstacleAvoidanceActive)
{
  std::shared_ptr<TestableProxMpcController> c;
  ASSERT_NO_THROW(
    c = makeConfigured(
      {rclcpp::Parameter(
          "FollowPath.model_plugin", std::string("prox_mpc_test_models/PermutedPlanarMapping")),
        rclcpp::Parameter("FollowPath.max_obstacles", 1)}));
  ASSERT_NE(c, nullptr);
  ASSERT_EQ(c->idxYaw(), 0u);
  ASSERT_EQ(c->idxX(), 1u);
  ASSERT_EQ(c->idxY(), 2u);
  ASSERT_EQ(c->maxObstacles(), 1u);

  c->activate();
  c->setPlan(makeStraightPlan(61, 0.2));
  const auto cmd =
    c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_TRUE(std::isfinite(cmd.twist.linear.x));

  auto solver = c->mpc()->getSolver();
  const MatrixXd & C = solver->getC();
  const std::vector<size_t> & ineq_idx = solver->getIneqIdx();
  const size_t obs_start = ineq_idx[ineq_idx.size() - 3];
  const size_t col = c->n_ * 1;   // x_start == 0; state block of node 1

  // The row is a unit normal spread over the two declared position columns.
  const double nx = C(obs_start, col + c->idxX());
  const double ny = C(obs_start, col + c->idxY());
  EXPECT_NEAR(nx * nx + ny * ny, 1.0, 1e-9);
  EXPECT_NEAR(C(obs_start, col + c->idxYaw()), 0.0, 1e-12);
}

// A model that overrides nothing still drives: the default getPlanarMapping()
// reproduces the pre-hook convention this controller assumed, so
// readModelMapping() accepts it and derives exactly that convention (no
// steering channel is inferred for a model with three states or fewer).
TEST_F(ControllerContractsTest, ReadModelMappingDefaultMappingLetsBareModelDrive)
{
  auto c = makeUnconfigured();
  BareThreeStateDrivableModel model;
  EXPECT_NO_THROW(callReadModelMapping(c, model, "test/BareThreeStateDrivable"));
  EXPECT_EQ(c->idxX(), 0u);
  EXPECT_EQ(c->idxY(), 1u);
  EXPECT_EQ(c->idxYaw(), 2u);
  EXPECT_EQ(c->idxV(), 0u);
  EXPECT_FALSE(c->hasSteering());
  EXPECT_NEAR(c->refOffsetX(), 0.0, kTol);
  EXPECT_NEAR(c->refOffsetY(), 0.0, kTol);
}

// The state-cost weights are placed through the declared mapping rather than at
// state columns 0 and 1. PermutedPlanarMapping orders its state [theta, x, y],
// so q_pos belongs on columns 1 and 2 and q_theta on column 0; indexing by
// position instead weights the heading as a position and the y position as a
// heading, which configure() accepts without a diagnostic because the model is
// otherwise legal (its obstacle avoidance is off, which is what lets a
// permuted position through readModelMapping() at all).
TEST_F(ControllerContractsTest, StateWeightsFollowTheDeclaredPlanarMapping)
{
  auto c = makeConfigured(
  {
    rclcpp::Parameter(
      "FollowPath.model_plugin", std::string("prox_mpc_test_models/PermutedPlanarMapping")),
    rclcpp::Parameter("FollowPath.q_pos", 10.0),
    rclcpp::Parameter("FollowPath.q_theta", 1.0),
  });
  ASSERT_EQ(c->idxYaw(), 0u);
  ASSERT_EQ(c->idxX(), 1u);
  ASSERT_EQ(c->idxY(), 2u);

  const MatrixXd Q = c->mpc()->getQ();
  ASSERT_EQ(Q.rows(), 3);
  ASSERT_EQ(Q.cols(), 3);
  EXPECT_NEAR(Q(0, 0), 1.0, kTol);    // yaw carries q_theta
  EXPECT_NEAR(Q(1, 1), 10.0, kTol);   // x carries q_pos
  EXPECT_NEAR(Q(2, 2), 10.0, kTol);   // y carries q_pos
}

// A full pluginlib-loaded control cycle against a model with no
// getPlanarMapping() override is already covered by
// test_prox_mpc_controller.cpp's ConfigureLoadsModelAndSizesMpc and
// ComputeWorksWithUnicycleModel: the bundled Unicycle itself declares no
// override, so every one of that suite's Unicycle-model cycles already
// exercises the default mapping end to end.

// --- The reference-point offset is explicitly transformed, never ------------
// --- silently mixed between a front- and rear-axle-referenced model ---------
//
// A Nav2 plan is always a base_link-referenced trajectory (there is no such
// thing as a "front-axle plan"); the question is whether the controller
// silently treats that trajectory as the model's own state (correct only for
// a model referenced to base_link, i.e. the rear axle) or explicitly
// transforms it by the model's declared offset. The state fed to the solver
// is read back through MPC::getPose() (public), so no white-box access is
// needed for this half.
TEST_F(ControllerContractsTest, ReferenceOffsetIsExplicitlyTransformedNotMixed)
{
  auto front = makeRunning(
    {rclcpp::Parameter("FollowPath.model_plugin", std::string(kFrontAxlePlugin)),
      rclcpp::Parameter("FollowPath.max_obstacles", 0)});
  front->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  const VectorXd pose_front = front->mpc()->getPose();

  auto rear = makeRunning(
    {rclcpp::Parameter("FollowPath.model_plugin", std::string(kRearAxlePlugin)),
      rclcpp::Parameter("FollowPath.max_obstacles", 0)});
  rear->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  const VectorXd pose_rear = rear->mpc()->getPose();

  // Rear axle: reference point is base_link itself, so the solver's pose IS
  // the robot's own pose (0, 0, 0).
  EXPECT_NEAR(pose_rear(rear->idxX()), 0.0, 1e-9);
  EXPECT_NEAR(pose_rear(rear->idxY()), 0.0, 1e-9);

  // Front axle: reference point is one wheelbase ahead on the body x axis, so
  // the solver's pose is explicitly shifted by that offset, not left at the
  // robot's own (0, 0) -- which is what "mixing" the two conventions would
  // silently do instead.
  EXPECT_NEAR(pose_front(front->idxX()), kBicycleWheelbase, 1e-9);
  EXPECT_NEAR(pose_front(front->idxY()), 0.0, 1e-9);
  EXPECT_NEAR(front->wheelbase(), kBicycleWheelbase, 1e-9);
  EXPECT_NEAR(front->refOffsetX(), kBicycleWheelbase, 1e-9);
}

// keepOutShift() carries an obstacle from base_link's frame (where the
// keep-out is meant to protect) into the model's reference-point frame (where
// the solver constrains it). Before the first solve the nominal trajectory is
// all-zero, so the shift is the offset itself rotated by yaw = 0.
TEST_F(ControllerContractsTest, KeepOutShiftCarriesFrontAxleOffsetBeforeFirstSolve)
{
  auto front = makeConfigured(
    {rclcpp::Parameter("FollowPath.model_plugin", std::string(kFrontAxlePlugin)),
      rclcpp::Parameter("FollowPath.max_obstacles", 1)});
  std::vector<double> shift_x;
  std::vector<double> shift_y;
  front->keepOutShift(shift_x, shift_y);
  ASSERT_FALSE(shift_x.empty());
  for (std::size_t i = 0; i < shift_x.size(); ++i) {
    EXPECT_NEAR(shift_x[i], kBicycleWheelbase, 1e-9) << "node " << i;
    EXPECT_NEAR(shift_y[i], 0.0, 1e-9) << "node " << i;
  }

  auto rear = makeConfigured(
    {rclcpp::Parameter("FollowPath.model_plugin", std::string(kRearAxlePlugin)),
      rclcpp::Parameter("FollowPath.max_obstacles", 1)});
  std::vector<double> shift_x_rear;
  std::vector<double> shift_y_rear;
  rear->keepOutShift(shift_x_rear, shift_y_rear);
  for (std::size_t i = 0; i < shift_x_rear.size(); ++i) {
    EXPECT_NEAR(shift_x_rear[i], 0.0, 1e-9) << "node " << i;
    EXPECT_NEAR(shift_y_rear[i], 0.0, 1e-9) << "node " << i;
  }
}

// fillStaticObstacles() writes a scanned cell in the SOLVER's frame (cell
// position + keepOutShift), so a front-axle model's keep-out disc and its
// footprint veto both protect the same physical (base_link) point: the disc
// is centred on base_link (the shift), and the veto transforms the predicted
// pose back to base_link before the check (computeVelocityCommands). This
// checks the disc half directly against the shift derived above.
TEST_F(ControllerContractsTest, FillStaticObstaclesWritesCellShiftedToSolverFrame)
{
  auto front = makeConfigured(
    {rclcpp::Parameter("FollowPath.model_plugin", std::string(kFrontAxlePlugin)),
      rclcpp::Parameter("FollowPath.max_obstacles", 1)});
  // Before the first solve the nominal predicted trajectory (mpc_->getX()) is
  // all-zero, so node 0's scan centres on the model-frame origin shifted BACK
  // to base_link: 0 - wheelbase = -wheelbase. Place the occupied cell there.
  fillCost(
    -kBicycleWheelbase - 0.05, -0.05, -kBicycleWheelbase + 0.05, 0.05,
    nav2_costmap_2d::LETHAL_OBSTACLE);

  MatrixXd obs(
    static_cast<Eigen::Index>(front->mpc()->getMaxObs() * front->mpc()->getNp()), 3);
  for (Eigen::Index r = 0; r < obs.rows(); ++r) {
    obs(r, 0) = prox_mpc::MPC::kObsFarSentinel;
    obs(r, 1) = prox_mpc::MPC::kObsFarSentinel;
  }
  front->fillStaticObstacles(obs, 0, {});

  // Node 0's slot must carry the cell's base_link-frame position shifted
  // forward by the wheelbase (base_link -> the front axle's own reference
  // frame), landing back near the model-frame origin, not the raw
  // base_link-frame position (-wheelbase, 0) the shift started from.
  EXPECT_NEAR(obs(0, 0), 0.0, kResolution);
  EXPECT_NEAR(obs(0, 1), 0.0, kResolution);
}

// The footprint veto operates in base_link terms regardless of the model's
// reference point (computeVelocityCommands transforms the predicted pose back
// before the check), so a lethal cell over the robot's own one-step-ahead
// base_link footprint vetoes for BOTH a front-axle and a rear-axle model, even
// though the two solve in different reference frames -- they protect the same
// physical point.
TEST_F(ControllerContractsTest, FootprintVetoFiresIdenticallyForFrontAndRearAxleModels)
{
  fillCost(-0.6, -0.6, 0.6, 0.6, nav2_costmap_2d::LETHAL_OBSTACLE);

  auto front = makeRunning(
    {rclcpp::Parameter("FollowPath.model_plugin", std::string(kFrontAxlePlugin)),
      rclcpp::Parameter("FollowPath.max_obstacles", 0)});
  const auto cmd_front = front->computeVelocityCommands(
    makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_NEAR(cmd_front.twist.linear.x, 0.0, kTol);   // ramp from rest: vetoed
  EXPECT_EQ(front->failureCount(), 0);                // veto is not a solver failure
}

TEST_F(ControllerContractsTest, FootprintVetoFiresForRearAxleModelAtSamePoint)
{
  fillCost(-0.6, -0.6, 0.6, 0.6, nav2_costmap_2d::LETHAL_OBSTACLE);

  auto rear = makeRunning(
    {rclcpp::Parameter("FollowPath.model_plugin", std::string(kRearAxlePlugin)),
      rclcpp::Parameter("FollowPath.max_obstacles", 0)});
  const auto cmd_rear = rear->computeVelocityCommands(
    makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_NEAR(cmd_rear.twist.linear.x, 0.0, kTol);
  EXPECT_EQ(rear->failureCount(), 0);
}

// --- Command validation: a non-finite Twist component OTHER than ------------
// --- linear.x is caught, not published unchecked ----------------------------

TEST_F(ControllerContractsTest, NonFiniteOtherAxesTwistRampsThenEscalates)
{
  auto c = makeRunning(
  {
    rclcpp::Parameter(
      "FollowPath.model_plugin", std::string("prox_mpc_test_models/NonFiniteTwistOtherAxes")),
    rclcpp::Parameter("FollowPath.max_solver_failures", 1),
  });

  geometry_msgs::msg::Twist measured;
  measured.linear.x = 0.30;
  measured.angular.z = -0.30;

  // The QP converges (finite dynamics) and linear.x/angular.z are finite, but
  // linear.y/angular.x are not: twist_is_finite() checks all six components,
  // so this is caught exactly like NonFiniteTwistModel's linear.x case is in
  // test_prox_mpc_controller.cpp's NonFiniteCommandRampsThenEscalates.
  const auto cmd = c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), measured, nullptr);
  EXPECT_NEAR(cmd.twist.linear.x, 0.30 - kModelDecel * 0.1, 1e-6);
  EXPECT_TRUE(std::isfinite(cmd.twist.linear.y));
  EXPECT_TRUE(std::isfinite(cmd.twist.angular.x));
  EXPECT_EQ(c->failureCount(), 1);

  EXPECT_THROW(
    c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), measured, nullptr),
    nav2_core::NoValidControl);
}

// --- SolverDiagnostics reports the applied outcome, not the QP's ------------
// --- outcome alone; the publisher is opt-in and only-when-subscribed --------

// A subscriber connects to the plugin's diagnostics topic and collects every
// message delivered, spinning between control cycles so DDS discovery has a
// chance to complete (matches PublishesPredictedTrajectory's technique in
// test_prox_mpc_controller.cpp).
class DiagnosticsCollector
{
public:
  DiagnosticsCollector(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & topic)
  {
    exec_.add_node(node->get_node_base_interface());
    sub_ = node->create_subscription<prox_mpc_msgs::msg::SolverDiagnostics>(
      topic, rclcpp::QoS(10).reliable(),
      [this](prox_mpc_msgs::msg::SolverDiagnostics::SharedPtr msg) {
        received_.push_back(*msg);
      });
  }

  // Runs one control cycle and spins until at least one new message is
  // delivered, or the budget is spent (which is itself an assertable result:
  // the caller checks the returned count against what it expects).
  std::size_t runCycleAndCollect(
    const std::function<void ()> & cycle, int max_spins = 100)
  {
    const std::size_t before = received_.size();
    cycle();
    for (int i = 0; i < max_spins && received_.size() == before; ++i) {
      exec_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return received_.size() - before;
  }

  const std::vector<prox_mpc_msgs::msg::SolverDiagnostics> & received() const {return received_;}

  // Spins this collector's own executor. A node may be added to only one
  // executor at a time, so callers that need to spin after runCycleAndCollect()
  // reuse this rather than constructing a second executor over the same node.
  void spinSome() {exec_.spin_some();}

private:
  rclcpp::executors::SingleThreadedExecutor exec_;
  rclcpp::Subscription<prox_mpc_msgs::msg::SolverDiagnostics>::SharedPtr sub_;
  std::vector<prox_mpc_msgs::msg::SolverDiagnostics> received_;
};

// publish_diagnostics defaulting false publishes nothing, even with an active
// subscriber -- the publisher itself is never created.
TEST_F(ControllerContractsTest, DiagnosticsPublisherOffPublishesNothing)
{
  auto c = makeRunning({rclcpp::Parameter("FollowPath.max_obstacles", 0)});
  DiagnosticsCollector collector(node_, "FollowPath/diagnostics");

  for (int i = 0; i < 5; ++i) {
    collector.runCycleAndCollect(
      [&]() {
        c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(),
        nullptr);
                 }, 20);
  }
  EXPECT_TRUE(collector.received().empty());
}

// publish_diagnostics true publishes exactly one message per control cycle,
// once a subscriber is connected.
TEST_F(ControllerContractsTest, DiagnosticsPublisherOnPublishesOncePerCycle)
{
  auto c = makeRunning(
    {rclcpp::Parameter("FollowPath.max_obstacles", 0),
      rclcpp::Parameter("FollowPath.publish_diagnostics", true)});
  DiagnosticsCollector collector(node_, "FollowPath/diagnostics");

  const std::size_t got1 = collector.runCycleAndCollect(
    [&]() {
      c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(),
      nullptr);
               });
  ASSERT_GE(got1, 1u);   // discovery may take more than one cycle the first time

  const std::size_t before = collector.received().size();
  c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  collector.spinSome();
  EXPECT_EQ(collector.received().size(), before + 1);   // exactly one per cycle
}

// A normal accepted cycle reports status == STATUS_SOLVED with converged ==
// true, and every field carries the value its comment in the .msg says it
// carries (SolverDiagnostics.msg): the header frame is the base frame, the
// residuals/objective/iteration counts mirror qp_info, max_obstacle_slack is
// 0 with obstacle avoidance off, control_period_ms is NaN on the first cycle
// of a task and a finite gap on the next, and deadline_missed follows
// solve_time_ms against the 1000*dt budget.
TEST_F(ControllerContractsTest, DiagnosticsFieldsMatchMessageContractOnAcceptedCycle)
{
  auto c = makeRunning(
    {rclcpp::Parameter("FollowPath.max_obstacles", 0),
      rclcpp::Parameter("FollowPath.publish_diagnostics", true)});
  DiagnosticsCollector collector(node_, "FollowPath/diagnostics");

  collector.runCycleAndCollect(
    [&]() {
      c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(),
      nullptr);
               });
  ASSERT_GE(collector.received().size(), 1u);
  const auto & d1 = collector.received().back();
  EXPECT_EQ(d1.header.frame_id, "base_link");
  EXPECT_EQ(d1.status, prox_mpc_msgs::msg::SolverDiagnostics::STATUS_SOLVED);
  EXPECT_TRUE(d1.converged);
  EXPECT_GE(d1.solve_time_ms, 0.0);
  EXPECT_GE(d1.qp_solve_time_ms, 0.0);
  EXPECT_GE(d1.sqp_iters, 1u);
  EXPECT_GE(d1.qp_iters_ext, 1u);
  EXPECT_NEAR(d1.max_obstacle_slack, 0.0, kTol);   // obstacle avoidance off
  EXPECT_TRUE(std::isnan(d1.control_period_ms));   // first cycle of the task
  EXPECT_EQ(d1.deadline_missed, d1.solve_time_ms > 1000.0 * 0.1);
  EXPECT_EQ(d1.num_active_obstacles, 0u);

  const std::size_t before = collector.received().size();
  c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  for (int i = 0; i < 20 && collector.received().size() == before; ++i) {
    collector.spinSome();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_GT(collector.received().size(), before);
  const auto & d2 = collector.received().back();
  EXPECT_FALSE(std::isnan(d2.control_period_ms));   // second cycle: a real gap
  EXPECT_GE(d2.control_period_ms, 0.0);
}

// A footprint-vetoed cycle reports the QP's own outcome (status ==
// STATUS_SOLVED, since the candidate itself converged) with converged ==
// false, because the command was never applied. In 1.0.0 converged was set
// from the QP status alone and published before the gates, so a vetoed and
// braked cycle was recorded as converged, contradicting the released message's
// own definition of the field.
TEST_F(ControllerContractsTest, DiagnosticsReportsSolvedButNotConvergedOnFootprintVeto)
{
  auto c = makeRunning(
    {rclcpp::Parameter("FollowPath.max_obstacles", 0),
      rclcpp::Parameter("FollowPath.publish_diagnostics", true)});
  DiagnosticsCollector collector(node_, "FollowPath/diagnostics");
  fillCost(-0.6, -0.6, 0.6, 0.6, nav2_costmap_2d::LETHAL_OBSTACLE);

  collector.runCycleAndCollect(
    [&]() {
      c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(),
      nullptr);
               });
  ASSERT_GE(collector.received().size(), 1u);
  const auto & d = collector.received().back();
  EXPECT_EQ(d.status, prox_mpc_msgs::msg::SolverDiagnostics::STATUS_SOLVED);
  EXPECT_FALSE(d.converged);
}

// The footprint-veto escalation publishes before it throws, on the same terms
// as the solver-failure escalation. A cycle that ends the task with
// NoValidControl is still a control cycle, and the diagnostics record of why it
// ended is the last message on the topic; publishing after the escalation test
// would leave that cycle unrecorded.
TEST_F(ControllerContractsTest, DiagnosticsPublishedOnTheEscalatingVetoCycle)
{
  auto c = makeRunning(
    {rclcpp::Parameter("FollowPath.max_obstacles", 0),
      rclcpp::Parameter("FollowPath.max_solver_failures", 1),
      rclcpp::Parameter("FollowPath.publish_diagnostics", true)});
  DiagnosticsCollector collector(node_, "FollowPath/diagnostics");
  fillCost(-0.6, -0.6, 0.6, 0.6, nav2_costmap_2d::LETHAL_OBSTACLE);

  // The first veto sits inside the budget and brakes.
  ASSERT_EQ(
    collector.runCycleAndCollect(
      [&]() {
        c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(),
        nullptr);
      }), 1u);

  // The second exceeds it and ends the task, and is recorded all the same.
  const std::size_t delivered = collector.runCycleAndCollect(
    [&]() {
      EXPECT_THROW(
        c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(),
        nullptr),
        nav2_core::NoValidControl);
    });
  EXPECT_EQ(delivered, 1u);
  EXPECT_FALSE(collector.received().back().converged);
}

// A non-finite-toTwist cycle reports the same shape: the QP itself converged
// (status == STATUS_SOLVED) but converged == false, because toTwist()'s
// output failed the finiteness gate.
TEST_F(ControllerContractsTest, DiagnosticsReportsSolvedButNotConvergedOnNonFiniteToTwist)
{
  auto c = makeRunning(
    {rclcpp::Parameter(
        "FollowPath.model_plugin", std::string("prox_mpc_test_models/NonFiniteTwist")),
      rclcpp::Parameter("FollowPath.publish_diagnostics", true)});
  DiagnosticsCollector collector(node_, "FollowPath/diagnostics");

  collector.runCycleAndCollect(
    [&]() {
      c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(),
      nullptr);
               });
  ASSERT_GE(collector.received().size(), 1u);
  const auto & d = collector.received().back();
  EXPECT_EQ(d.status, prox_mpc_msgs::msg::SolverDiagnostics::STATUS_SOLVED);
  EXPECT_FALSE(d.converged);
}

// --- Terminal heading and travel direction ----------------------------------

// The final pose's quaternion differs from the final segment's tangent (the
// plan runs straight along +x, but the goal pose faces +y): existing tests in
// test_prox_mpc_controller.cpp use plans where the two agree, so none of them
// can tell terminal-yaw tracking from tangent-holding. mpc()->getGoalX() (a
// public MPC accessor) is the state reference actually handed to the solver.
TEST_F(ControllerContractsTest, TerminalYawTracksGoalOrientationDivergentFromTangent)
{
  auto c = makeConfigured({rclcpp::Parameter("FollowPath.max_obstacles", 0)});
  c->activate();

  nav_msgs::msg::Path plan = makeStraightPlan(11, 0.2);   // straight along +x, tangent yaw = 0
  plan.poses.back().pose.orientation.z = std::sin(M_PI / 4.0);   // goal pose faces +y (yaw = pi/2)
  plan.poses.back().pose.orientation.w = std::cos(M_PI / 4.0);
  c->setPlan(plan);

  StubGoalChecker goal_checker(0.5);   // publishes a yaw tolerance -> terminal yaw engages
  // Pose already at the goal: every horizon node samples past the plan end.
  c->computeVelocityCommands(makePose(2.0, 0.0, 0.0), geometry_msgs::msg::Twist(), &goal_checker);

  const std::size_t np = c->mpc()->getNp();
  EXPECT_NEAR(
    c->mpc()->getGoalX()(static_cast<Eigen::Index>(np), c->idxYaw()), M_PI / 2.0, 1e-6);
}

// Without a published yaw tolerance, the reference instead holds the final
// segment's tangent (yaw = 0 here), exactly as it did before terminal-yaw
// tracking existed -- the other half of the same branch.
TEST_F(ControllerContractsTest, NoYawToleranceHoldsFinalSegmentTangent)
{
  auto c = makeConfigured({rclcpp::Parameter("FollowPath.max_obstacles", 0)});
  c->activate();

  nav_msgs::msg::Path plan = makeStraightPlan(11, 0.2);
  plan.poses.back().pose.orientation.z = std::sin(M_PI / 4.0);
  plan.poses.back().pose.orientation.w = std::cos(M_PI / 4.0);
  c->setPlan(plan);

  c->computeVelocityCommands(makePose(2.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);

  const std::size_t np = c->mpc()->getNp();
  EXPECT_NEAR(c->mpc()->getGoalX()(static_cast<Eigen::Index>(np), c->idxYaw()), 0.0, 1e-6);
}

// A reverse segment: the plan's own pose orientations face -x while the
// segment tangent points +x, so with allow_reversing the reference speed
// becomes negative (dir = -1); with it off (the default) the reference stays
// forward-only, unaffected by pose orientation.
TEST_F(ControllerContractsTest, AllowReversingSignsTheReferenceSpeedNegative)
{
  nav_msgs::msg::Path plan = makeStraightPlan(11, 0.2);
  for (auto & p : plan.poses) {
    p.pose.orientation.z = 1.0;   // yaw = pi: faces -x, opposite the +x tangent
    p.pose.orientation.w = 0.0;
  }

  auto c = makeConfigured(
    {rclcpp::Parameter("FollowPath.max_obstacles", 0),
      rclcpp::Parameter("FollowPath.allow_reversing", true),
      rclcpp::Parameter("FollowPath.reverse_from_plan_orientation", true)});
  c->activate();
  c->setPlan(plan);
  c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_LT(c->mpc()->getGoalU()(0, static_cast<Eigen::Index>(c->idxV())), 0.0);
}

// The same plan, with reversing allowed but the plan's orientations not trusted
// (the default): the reference stays forward-only. This is what keeps a planner
// that leaves its pose orientations at the identity - NavFn, Smac 2D - from
// being read as asking for a reverse traverse of every path running the other
// way, which drove the robot backwards down the whole path.
TEST_F(ControllerContractsTest, UntrustedPlanOrientationKeepsReferenceForward)
{
  nav_msgs::msg::Path plan = makeStraightPlan(11, 0.2);
  for (auto & p : plan.poses) {
    p.pose.orientation.z = 1.0;   // yaw = pi, opposite the +x tangent
    p.pose.orientation.w = 0.0;
  }

  auto c = makeConfigured(
    {rclcpp::Parameter("FollowPath.max_obstacles", 0),
      rclcpp::Parameter("FollowPath.allow_reversing", true)});
  c->activate();
  c->setPlan(plan);
  c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_GE(c->mpc()->getGoalU()(0, static_cast<Eigen::Index>(c->idxV())), 0.0);
}

// A steering (Ackermann) model on a reversing plan. Unlike a differential drive,
// a car-like platform cannot turn in place, so reverse is the only way through a
// cusp and the capability has to survive the orientation-trust split. `dir` also
// signs the steering inverse, so the reference steering angle turns the opposite
// way in reverse for the same path curvature.
TEST_F(ControllerContractsTest, SteeringModelTracksReverseOnADirectionalPlan)
{
  // An arc whose poses face backwards along it, which is how a reversing plan is
  // expressed: the body heading is the path tangent turned by pi.
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  const double radius = 8.0;
  const double dphi = 0.2 / radius;
  for (int i = 0; i < 21; ++i) {
    const double phi = static_cast<double>(i) * dphi;
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose.position.x = radius * std::sin(phi);
    ps.pose.position.y = radius * (1.0 - std::cos(phi));
    const double facing = phi + M_PI;
    ps.pose.orientation.z = std::sin(facing / 2.0);
    ps.pose.orientation.w = std::cos(facing / 2.0);
    plan.poses.push_back(ps);
  }

  auto c = makeConfigured(
    {rclcpp::Parameter("FollowPath.model_plugin", std::string(kFrontAxlePlugin)),
      rclcpp::Parameter("FollowPath.max_obstacles", 0),
      rclcpp::Parameter("FollowPath.allow_reversing", true),
      rclcpp::Parameter("FollowPath.reverse_from_plan_orientation", true)});
  c->activate();
  c->setPlan(plan);
  c->computeVelocityCommands(makePose(0.0, 0.0, M_PI), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_LT(c->mpc()->getGoalU()(0, static_cast<Eigen::Index>(c->idxV())), 0.0);

  // The same plan with the orientations untrusted keeps the reference forward,
  // so the default protects a steering model from an orientation-less planner
  // exactly as it protects a differential drive.
  auto fwd = makeConfigured(
      {rclcpp::Parameter("FollowPath.model_plugin", std::string(kFrontAxlePlugin)),
        rclcpp::Parameter("FollowPath.max_obstacles", 0),
        rclcpp::Parameter("FollowPath.allow_reversing", true)});
  fwd->activate();
  fwd->setPlan(plan);
  fwd->computeVelocityCommands(makePose(0.0, 0.0, M_PI), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_GE(fwd->mpc()->getGoalU()(0, static_cast<Eigen::Index>(fwd->idxV())), 0.0);
}

// Travel direction is a latched mode, not a per-cycle read of the plan, and a
// change is accepted only from rest: a drivetrain takes a gear shift at
// standstill, and the same gate is what stops the mode alternating cycle to
// cycle. While the change is pending the reference is pinned to the arc length
// under the robot, so the commanded speed falls to zero and the platform coasts
// down to the standstill the change is waiting on.
TEST_F(ControllerContractsTest, DirectionChangeIsHeldWhileThePlatformIsMoving)
{
  auto c = makeConfigured(
    {rclcpp::Parameter("FollowPath.max_obstacles", 0),
      rclcpp::Parameter("FollowPath.allow_reversing", true),
      rclcpp::Parameter("FollowPath.reverse_from_plan_orientation", true),
      rclcpp::Parameter("FollowPath.direction_switch_standstill_speed_mps", 1e-3),
      rclcpp::Parameter("FollowPath.direction_switch_dwell_s", 0.0)});
  c->activate();

  // Build forward speed first, so the platform is moving when the plan flips.
  c->setPlan(makeStraightPlan(61, 0.2));
  geometry_msgs::msg::TwistStamped cmd;
  for (int i = 0; i < 5; ++i) {
    cmd = c->computeVelocityCommands(
      makePose(0.2 * static_cast<double>(i), 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  }
  ASSERT_GT(cmd.twist.linear.x, 1e-3) << "the platform has to be moving for the gate to bite";

  nav_msgs::msg::Path reverse_plan = makeStraightPlan(61, 0.2);
  for (auto & p : reverse_plan.poses) {
    p.pose.orientation.z = 1.0;   // yaw = pi, opposite the +x tangent
    p.pose.orientation.w = 0.0;
  }
  c->setPlan(reverse_plan);
  c->computeVelocityCommands(makePose(1.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);

  // Held, not taken: the reference speed is zero rather than negative.
  EXPECT_NEAR(c->mpc()->getGoalU()(0, static_cast<Eigen::Index>(c->idxV())), 0.0, 1e-12);
}

// The second gate. With standstill satisfied throughout, a change still waits
// out the dwell, which is what bounds the switching rate: a mode re-chosen every
// cycle with no dwell is free to chatter however cheap each switch looks.
TEST_F(ControllerContractsTest, DirectionChangeIsHeldUntilTheDwellElapses)
{
  auto c = makeConfigured(
    {rclcpp::Parameter("FollowPath.max_obstacles", 0),
      rclcpp::Parameter("FollowPath.allow_reversing", true),
      rclcpp::Parameter("FollowPath.reverse_from_plan_orientation", true),
      rclcpp::Parameter("FollowPath.direction_switch_standstill_speed_mps", 100.0),
      rclcpp::Parameter("FollowPath.direction_switch_dwell_s", 100.0)});
  c->activate();

  nav_msgs::msg::Path reverse_plan = makeStraightPlan(61, 0.2);
  for (auto & p : reverse_plan.poses) {
    p.pose.orientation.z = 1.0;
    p.pose.orientation.w = 0.0;
  }
  // The first direction of a task is free: nothing has been switched away from.
  c->setPlan(reverse_plan);
  c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  ASSERT_LT(c->mpc()->getGoalU()(0, static_cast<Eigen::Index>(c->idxV())), 0.0);

  // Switching straight back is not: the dwell has just been spent.
  c->setPlan(makeStraightPlan(61, 0.2));
  c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_NEAR(c->mpc()->getGoalU()(0, static_cast<Eigen::Index>(c->idxV())), 0.0, 1e-12);
}

// Inside the goal-checker xy tolerance the reference is pinned to the goal pose
// instead of tracking the robot's own projection onto the plan. Composing the
// cruise taper with the sampling step makes the horizon's arc reach
// remaining^2 / xy_tol, shorter than `remaining` everywhere inside the
// tolerance, so the tracked reference would otherwise be a stub a few
// millimetres ahead of the projection that the robot carries along with it -- a
// reference with no fixed point, and one whose horizon never reaches the plan
// end, so the goal's own orientation never enters it. Pinning gives the last
// stretch a fixed setpoint and a standing yaw error to act on.
TEST_F(ControllerContractsTest, GoalRegionPinsTheReferenceToTheGoalPose)
{
  nav_msgs::msg::Path plan = makeStraightPlan(11, 0.2);   // goal at x = 2.0
  // A goal yaw the robot is already within tolerance of, so this exercises the
  // settle pin rather than the on-the-spot turn tested below, while still
  // differing from the plan's own tangent (0.0) that the reference held before.
  plan.poses.back().pose.orientation.z = std::sin(0.15);
  plan.poses.back().pose.orientation.w = std::cos(0.15);

  auto c = makeConfigured({rclcpp::Parameter("FollowPath.max_obstacles", 0)});
  c->activate();
  c->setPlan(plan);

  StubGoalChecker goal_checker(0.5);   // xy tolerance 1.0, yaw tolerance 0.5
  // 0.5 m short of the goal: inside the tolerance, so the settle latch engages.
  c->computeVelocityCommands(makePose(1.5, 0.0, 0.0), geometry_msgs::msg::Twist(), &goal_checker);

  // Node 0 carries the goal pose, not the projection at x = 1.5.
  EXPECT_NEAR(c->mpc()->getGoalX()(0, static_cast<Eigen::Index>(c->idxX())), 2.0, 1e-6);
  EXPECT_NEAR(c->mpc()->getGoalX()(0, static_cast<Eigen::Index>(c->idxY())), 0.0, 1e-6);
  EXPECT_NEAR(c->mpc()->getGoalX()(0, static_cast<Eigen::Index>(c->idxYaw())), 0.3, 1e-6);
}

// The latch is entered on the tolerance and released only past a wider band, so
// a remaining distance hovering about the tolerance cannot flip the reference
// mode to mode. Within one plan the projection is forward-only and the remaining
// distance is monotone, so the band is what a replan crosses: a goal that moves
// a little further away holds the latch, and one that moves well away releases
// it back to path tracking.
TEST_F(ControllerContractsTest, GoalRegionLatchReleasesOnlyPastTheHysteresisBand)
{
  auto c = makeConfigured(
    {rclcpp::Parameter("FollowPath.max_obstacles", 0),
      rclcpp::Parameter("FollowPath.goal_settle_hysteresis_m", 0.1)});
  c->activate();

  StubGoalChecker goal_checker(0.5);   // xy tolerance 1.0, so the band ends at 1.1
  const geometry_msgs::msg::Twist zero;

  // 0.5 m from the goal at x = 2.0: inside the tolerance, so the latch engages.
  c->setPlan(makeStraightPlan(11, 0.2));
  c->computeVelocityCommands(makePose(1.5, 0.0, 0.0), zero, &goal_checker);
  ASSERT_NEAR(c->mpc()->getGoalX()(0, static_cast<Eigen::Index>(c->idxX())), 2.0, 1e-6);

  // Replanned 1.05 m out: past the tolerance but inside the band, so the latch
  // holds and the reference stays pinned to the new goal rather than tracking.
  c->setPlan(makeStraightPlan(8, 0.15));   // goal at x = 1.05
  c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), zero, &goal_checker);
  EXPECT_NEAR(c->mpc()->getGoalX()(0, static_cast<Eigen::Index>(c->idxX())), 1.05, 1e-6);

  // Replanned 2.0 m out: past the band, so tracking resumes and node 0 is the
  // robot's own projection onto the plan again.
  c->setPlan(makeStraightPlan(11, 0.2));   // goal at x = 2.0
  c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), zero, &goal_checker);
  EXPECT_NEAR(c->mpc()->getGoalX()(0, static_cast<Eigen::Index>(c->idxX())), 0.0, 1e-6);
}

// Once the checker's own xy condition is met, only the heading is outstanding.
// Pinning the reference to the goal would leave the solver holding a position it
// is already close enough to, and reversing and re-advancing to hold it as the
// body sweeps round and the body-frame projection of the offset changes sign.
// The reference is therefore held at the robot's own reference point, leaving
// heading as the only standing error, and the platform turns on the spot.
TEST_F(ControllerContractsTest, TerminalHeadingTurnsOnTheSpot)
{
  nav_msgs::msg::Path plan = makeStraightPlan(11, 0.2);   // goal at x = 2.0
  plan.poses.back().pose.orientation.z = std::sin(M_PI / 4.0);   // goal yaw = pi/2
  plan.poses.back().pose.orientation.w = std::cos(M_PI / 4.0);

  auto c = makeConfigured({rclcpp::Parameter("FollowPath.max_obstacles", 0)});
  c->activate();
  c->setPlan(plan);

  StubGoalChecker goal_checker(0.5);   // xy tolerance 1.0, yaw tolerance 0.5
  // 0.1 m from the goal, so the translation is done, but pi/2 of heading out.
  c->computeVelocityCommands(makePose(1.9, 0.0, 0.0), geometry_msgs::msg::Twist(), &goal_checker);

  // The reference holds station at the robot, not at the goal 0.1 m ahead...
  EXPECT_NEAR(c->mpc()->getGoalX()(0, static_cast<Eigen::Index>(c->idxX())), 1.9, 1e-6);
  // ...and carries the goal heading, which is the error left to close.
  EXPECT_NEAR(c->mpc()->getGoalX()(0, static_cast<Eigen::Index>(c->idxYaw())), M_PI / 2.0, 1e-6);
  // No reference speed: the platform is asked to turn, not to translate.
  EXPECT_NEAR(c->mpc()->getGoalU()(0, static_cast<Eigen::Index>(c->idxV())), 0.0, 1e-12);
}

// The same geometry with the heading already inside the checker's yaw tolerance
// is an ordinary settle: nothing is outstanding but the last of the translation,
// so the reference stays pinned to the goal pose.
TEST_F(ControllerContractsTest, TerminalHeadingWithinToleranceStaysPinnedToTheGoal)
{
  auto c = makeConfigured({rclcpp::Parameter("FollowPath.max_obstacles", 0)});
  c->activate();
  c->setPlan(makeStraightPlan(11, 0.2));   // goal yaw 0, matching the robot's

  StubGoalChecker goal_checker(0.5);
  c->computeVelocityCommands(makePose(1.9, 0.0, 0.0), geometry_msgs::msg::Twist(), &goal_checker);

  EXPECT_NEAR(c->mpc()->getGoalX()(0, static_cast<Eigen::Index>(c->idxX())), 2.0, 1e-6);
}

// A steering model is excluded: a car-like platform cannot turn on the spot, and
// pinning its position would leave it no admissible way to correct its heading.
// It settles onto the goal pose and manoeuvres out of a heading error instead.
TEST_F(ControllerContractsTest, SteeringModelDoesNotTurnOnTheSpotAtTheGoal)
{
  nav_msgs::msg::Path plan = makeStraightPlan(11, 0.2);
  plan.poses.back().pose.orientation.z = std::sin(M_PI / 4.0);
  plan.poses.back().pose.orientation.w = std::cos(M_PI / 4.0);

  auto c = makeConfigured(
    {rclcpp::Parameter("FollowPath.model_plugin", std::string(kFrontAxlePlugin)),
      rclcpp::Parameter("FollowPath.max_obstacles", 0)});
  c->activate();
  c->setPlan(plan);

  StubGoalChecker goal_checker(0.5);
  c->computeVelocityCommands(makePose(1.9, 0.0, 0.0), geometry_msgs::msg::Twist(), &goal_checker);

  EXPECT_NEAR(c->mpc()->getGoalX()(0, static_cast<Eigen::Index>(c->idxX())), 2.0, 1e-6);
}

TEST_F(ControllerContractsTest, ReversingOffKeepsForwardOnlyReferenceDespiteOrientation)
{
  nav_msgs::msg::Path plan = makeStraightPlan(11, 0.2);
  for (auto & p : plan.poses) {
    p.pose.orientation.z = 1.0;
    p.pose.orientation.w = 0.0;
  }

  // allow_reversing: default false
  auto c = makeConfigured({rclcpp::Parameter("FollowPath.max_obstacles", 0)});
  c->activate();
  c->setPlan(plan);
  c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_GE(c->mpc()->getGoalU()(0, static_cast<Eigen::Index>(c->idxV())), 0.0);
}

// A direction-change cusp truncates the reference at the first reversal: the
// plan runs forward from x=0 to x=1.0, then reverses back toward x=0 with the
// SAME pose orientation throughout (yaw = 0), so the tangent (not the
// orientation) is what flips. The reference must hold at the cusp (x ~ 1.0)
// rather than continue past it back toward x=0 within one horizon.
TEST_F(ControllerContractsTest, DirectionChangeCuspTruncatesTheReference)
{
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  for (int i = 0; i <= 10; ++i) {   // 0.0 -> 1.0, forward
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose.position.x = static_cast<double>(i) * 0.1;
    ps.pose.orientation.w = 1.0;
    plan.poses.push_back(ps);
  }
  for (int i = 9; i >= 0; --i) {   // 0.9 -> 0.0, tangent reverses; orientation unchanged
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose.position.x = static_cast<double>(i) * 0.1;
    ps.pose.orientation.w = 1.0;
    plan.poses.push_back(ps);
  }

  auto c = makeConfigured(
    {rclcpp::Parameter("FollowPath.max_obstacles", 0),
      rclcpp::Parameter("FollowPath.allow_reversing", true),
      rclcpp::Parameter("FollowPath.reverse_from_plan_orientation", true)});
  c->activate();
  c->setPlan(plan);
  c->computeVelocityCommands(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist(), nullptr);

  const std::size_t np = c->mpc()->getNp();
  // Held at the cusp (~1.0), not continuing past it back toward 0.0 within
  // the horizon -- the horizon (Np * dt = 2 s at the defaults) comfortably
  // reaches and passes the 1 m cusp point at any ordinary cruise speed.
  EXPECT_GE(c->mpc()->getGoalX()(static_cast<Eigen::Index>(np), c->idxX()), 1.0 - 0.05);
}

// --- At the controller level, the released Bicycle name still loads ---------
// --- and resolves to the front-axle plugin ----------------------------------

TEST_F(ControllerContractsTest, DeprecatedBicycleNameResolvesToFrontAxlePhysics)
{
  auto c = makeConfigured(
    {rclcpp::Parameter("FollowPath.model_plugin", std::string("prox_mpc_core/Bicycle"))});
  ASSERT_NE(c->model(), nullptr);
  EXPECT_EQ(c->model()->getName(), "bicycle");
  EXPECT_NEAR(c->wheelbase(), kBicycleWheelbase, 1e-9);
  EXPECT_NEAR(c->refOffsetX(), kBicycleWheelbase, 1e-9);   // front-axle offset, not 0
  EXPECT_TRUE(c->hasSteering());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
