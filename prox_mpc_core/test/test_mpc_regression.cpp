// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Regression test for the obstacle-off configuration. The MPC is configured as
// the demo node configures it, solved once from a fixed initial pose, and the
// first control input and a few trajectory samples are compared against recorded
// reference values. A divergence beyond the tolerance indicates a change in the
// obstacle-off solver path.

#include <memory>

#include <gtest/gtest.h>

#include <proxsuite/proxqp/status.hpp>

#include <prox_mpc/mpc.hpp>
#include <prox_mpc/models/bike.hpp>

using prox_mpc::Bicycle;
using prox_mpc::MPC;
using prox_mpc::Model;

namespace
{
// Regression tolerance. The obstacle-off path is reproduced bit-for-bit on the
// reference build; kTol leaves margin for floating-point/platform variation
// while staying far tighter than any real logic change would move the solution.
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
