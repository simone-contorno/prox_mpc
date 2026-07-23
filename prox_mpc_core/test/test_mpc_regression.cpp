// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Regression test for the obstacle-off configuration. The MPC is configured as
// the demo node configures it, solved once from a fixed initial pose, and the
// first control input and a few trajectory samples are compared against recorded
// reference values. A divergence beyond the tolerance indicates a change in the
// obstacle-off solver path.

#include <memory>
#include <tuple>

#include <gtest/gtest.h>

#include <proxsuite/proxqp/status.hpp>

#include <prox_mpc/mpc.hpp>
#include <prox_mpc/models/bicycle.hpp>

using prox_mpc::Bicycle;
using prox_mpc::MPC;
using prox_mpc::Model;

namespace
{
// Regression tolerance. The obstacle-off path is reproduced bit-for-bit on the
// reference build; kTol leaves margin for floating-point/platform variation
// while staying far tighter than any real logic change would move the solution.
// The reference constants below were captured against the locally provisioned
// ProxSuite; the canonical reproducible dependency is the apt package
// ros-jazzy-proxsuite, which CI provisions via rosdep, so a divergence surfaces
// here rather than as a false green.
constexpr double kTol = 1e-6;

// Build the demo's obstacle-off configuration and run one solve() from pose 0.
std::tuple<MatrixXd, MatrixXd> solveDemoObstacleOff()
{
  auto model = std::make_shared<Bicycle>();   // demo default model
  // obstacle OFF: leave the capacity at 0 (do not call setMaxObs).

  const size_t n = model->getN();
  const size_t m = model->getM();

  VectorXd q_diag = VectorXd::Constant(n, 1.0);  // q_theta
  q_diag(0) = 10.0;                              // q_pos
  q_diag(1) = 10.0;                              // q_pos
  MatrixXd Q = q_diag.asDiagonal();
  MatrixXd S = 2.0 * Q;                          // s_factor
  MatrixXd R = 0.1 * MatrixXd::Identity(m, m);   // r_weight
  MatrixXd W = MatrixXd::Constant(1, 1, 100.0);  // w_weight

  auto mpc = std::make_shared<MPC>();
  mpc->setNp(20);
  mpc->setNc(20);
  mpc->setdt(0.1);
  mpc->setQ(Q);
  mpc->setS(S);
  mpc->setR(R);
  mpc->setW(W);
  mpc->init(model);

  MatrixXd goal_x = MatrixXd::Zero(21, n);
  for (size_t k = 0; k <= 20; k++) {
    goal_x(k, 0) = 5.0;
    goal_x(k, 1) = 0.0;
    goal_x(k, 2) = 0.0;
  }
  MatrixXd goal_u = MatrixXd::Zero(20, m);   // v_ref in column 0
  for (size_t k = 0; k < 20; k++) {
    goal_u(k, 0) = 1.0;
  }
  mpc->setGoalX(goal_x);
  mpc->setGoalU(goal_u);

  mpc->setPose(VectorXd::Zero(n));
  auto traj = mpc->solve();
  EXPECT_EQ(mpc->qp_info.status, proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED);
  return traj;
}

// A bicycle MPC whose QP iteration caps are too low to converge, so the SQP loop
// never sees PROXQP_SOLVED and runs until either max_iter_sqp or the optional
// wall-clock budget stops it. Used to exercise the wall-clock budget.
std::shared_ptr<MPC> makeNonConvergingMpc(double max_solve_time, size_t max_iter_sqp)
{
  auto model = std::make_shared<Bicycle>();
  const size_t n = model->getN();
  const size_t m = model->getM();

  VectorXd q_diag = VectorXd::Constant(n, 1.0);
  q_diag(0) = 10.0;
  q_diag(1) = 10.0;
  MatrixXd Q = q_diag.asDiagonal();

  auto mpc = std::make_shared<MPC>();
  mpc->setNp(20);
  mpc->setNc(20);
  mpc->setdt(0.1);
  mpc->setQ(Q);
  mpc->setS(2.0 * Q);
  mpc->setR(0.1 * MatrixXd::Identity(m, m));
  mpc->setW(MatrixXd::Constant(1, 1, 100.0));
  mpc->setMaxIntIterQP(1);   // too few QP iterations to converge, so the SQP loop
  mpc->setMaxExtIterQP(1);   // never reaches PROXQP_SOLVED and keeps iterating
  mpc->setMaxIterSQP(max_iter_sqp);
  mpc->setMaxSolveTime(max_solve_time);
  mpc->init(model);

  MatrixXd goal_x = MatrixXd::Zero(21, n);
  for (size_t k = 0; k <= 20; k++) {
    goal_x(k, 0) = 5.0;
  }
  MatrixXd goal_u = MatrixXd::Zero(20, m);
  for (size_t k = 0; k < 20; k++) {
    goal_u(k, 0) = 1.0;
  }
  mpc->setGoalX(goal_x);
  mpc->setGoalU(goal_u);
  mpc->setPose(VectorXd::Zero(n));
  return mpc;
}
}  // namespace

TEST(MpcRegression, ObstacleOffMatchesBaseline)
{
  auto [x, u] = solveDemoObstacleOff();

  ASSERT_EQ(x.rows(), 21);
  ASSERT_EQ(x.cols(), 4);
  ASSERT_EQ(u.rows(), 20);
  ASSERT_EQ(u.cols(), 2);

  // First control input (sent to the vehicle).
  EXPECT_NEAR(u(0, 0), 0.050000065264268402, kTol);
  EXPECT_NEAR(u(0, 1), 0.0, kTol);

  // Trajectory samples [x, y, theta, delta]; the path is straight (y/theta/delta 0).
  EXPECT_NEAR(x(1, 0), 0.0050000065692164045, kTol);
  EXPECT_NEAR(x(2, 0), 0.015000019220180322, kTol);
  EXPECT_NEAR(x(5, 0), 0.075000089563722935, kTol);
  EXPECT_NEAR(x(10, 0), 0.27500028980698932, kTol);
  EXPECT_NEAR(x(20, 0), 1.0500008362745861, kTol);
  for (int k : {1, 2, 5, 10, 20}) {
    EXPECT_NEAR(x(k, 1), 0.0, kTol);
    EXPECT_NEAR(x(k, 2), 0.0, kTol);
    EXPECT_NEAR(x(k, 3), 0.0, kTol);
  }

  // Last control input reaches the reference speed.
  EXPECT_NEAR(u(19, 0), 1.0000005738580449, kTol);
  EXPECT_NEAR(u(19, 1), 0.0, kTol);
}

// The optional wall-clock budget bounds the SQP loop: a sub-nanosecond budget
// stops it well before max_iter_sqp, while the disabled budget (0, the default)
// lets a non-converging problem run to the iteration cap. Either way the
// unconverged status routes the caller to its fail-safe, so a slow solve cannot
// overrun the control cycle.
TEST(SqpBudget, WallClockBudgetStopsLoopEarly)
{
  constexpr size_t kMaxSqp = 30;

  auto budgeted = makeNonConvergingMpc(1e-9, kMaxSqp);   // sub-nanosecond budget
  budgeted->solve();
  EXPECT_NE(budgeted->qp_info.status, proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED);
  EXPECT_LT(budgeted->sqp_iter, kMaxSqp);                // stopped early by the budget

  auto unbudgeted = makeNonConvergingMpc(0.0, kMaxSqp);  // budget disabled
  unbudgeted->solve();
  EXPECT_NE(unbudgeted->qp_info.status, proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED);
  EXPECT_EQ(unbudgeted->sqp_iter, kMaxSqp);              // ran to the SQP iteration cap
}

// Move-blocking (Nc < Np): the move-blocking clamp holds the control constant
// past the control horizon, so a configuration with fewer control nodes than
// prediction nodes still assembles a solvable QP and drives the bicycle forward.
TEST(MoveBlocking, NcLessThanNpSolvesAndAdvances)
{
  auto model = std::make_shared<Bicycle>();
  const size_t n = model->getN();
  const size_t m = model->getM();

  VectorXd q_diag = VectorXd::Constant(n, 1.0);
  q_diag(0) = 10.0;
  q_diag(1) = 10.0;
  MatrixXd Q = q_diag.asDiagonal();

  auto mpc = std::make_shared<MPC>();
  mpc->setNp(20);
  mpc->setNc(5);                 // move-blocking: fewer control nodes than prediction nodes
  mpc->setdt(0.1);
  mpc->setQ(Q);
  mpc->setS(2.0 * Q);
  mpc->setR(0.1 * MatrixXd::Identity(m, m));
  mpc->setW(MatrixXd::Constant(1, 1, 100.0));
  mpc->init(model);
  ASSERT_EQ(mpc->getNp(), 20u);
  ASSERT_EQ(mpc->getNc(), 5u);

  MatrixXd goal_x = MatrixXd::Zero(21, n);
  for (size_t k = 0; k <= 20; k++) {
    goal_x(k, 0) = 5.0;
  }
  MatrixXd goal_u = MatrixXd::Zero(5, m);
  for (size_t k = 0; k < 5; k++) {
    goal_u(k, 0) = 1.0;
  }
  mpc->setGoalX(goal_x);
  mpc->setGoalU(goal_u);
  mpc->setPose(VectorXd::Zero(n));

  auto [x, u] = mpc->solve();
  EXPECT_EQ(mpc->qp_info.status, proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED);
  EXPECT_EQ(u.rows(), 5);            // exactly Nc control rows
  EXPECT_GT(u(0, 0), 0.0);           // accelerates forward toward the goal
  EXPECT_GT(x(20, 0), x(0, 0));      // the predicted trajectory advances in +x
  EXPECT_TRUE(x.allFinite());
}
