// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// A custom single-integrator model exercised through the Model interface,
// confirming the solver accepts a model defined outside the library. The
// pure-virtual hooks mean a model that omits an override does not compile; an
// opt-in negative compile check demonstrating this is included at the bottom.

#include <cmath>
#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>

#include <prox_mpc/mpc.hpp>
#include <prox_mpc/model.hpp>
#include <prox_mpc/proxqp.hpp>

using prox_mpc::MPC;
using prox_mpc::Model;

namespace
{

// Single-integrator: x_{k+1} = x_k + dt * u_k, state [x, y], control [vx, vy].
class DummyLinear : public Model
{
public:
  DummyLinear()
  {
    setName("dummy_linear");
    setN(2);  // state: [x, y]
    setM(2);  // control: [vx, vy]
    setIneq("u", 0, -2.0, 2.0);
    setIneq("u", 1, -2.0, 2.0);
  }

  void updatec(double dt, VectorXd x_next) override
  {
    // residual c = x_k - x_{k+1} + dt * f(x_k, u_k), with f = u_k.
    c = getX() - x_next + dt * getU();
  }

  void updateA(double /*dt*/) override
  {
    // A = d(residual)/d x_k = I (f has no state dependence).
    A = MatrixXd::Identity(getN(), getN());
  }

  void updateB() override
  {
    // B = d f / d u_k = I (scaled by dt at the call site).
    B = MatrixXd::Identity(getN(), getM());
  }
};

// Same single integrator, but the only control-rate bound is declared for
// control index 1, so the first "du" entry does not sit at map key 1's index.
class DuOnSecondControl : public DummyLinear
{
public:
  DuOnSecondControl()
  {
    setName("du_on_second_control");
    setIneq("du", 1, -0.5, 0.5);
  }
};

bool isFinite(const MatrixXd & m)
{
  return m.allFinite();
}

// Build a solved MPC for the given model with a simple position goal.
std::shared_ptr<MPC> solveTowardGoal(const std::shared_ptr<Model> & model, size_t np)
{
  const size_t n = model->getN();
  const size_t m = model->getM();

  auto mpc = std::make_shared<MPC>();
  mpc->setNp(np);
  mpc->setNc(np);
  mpc->setdt(0.1);
  mpc->setQ(10.0 * MatrixXd::Identity(n, n));
  mpc->setS(20.0 * MatrixXd::Identity(n, n));
  mpc->setR(0.1 * MatrixXd::Identity(m, m));
  mpc->setW(MatrixXd::Constant(1, 1, 100.0));
  mpc->init(model);

  MatrixXd goal_x = MatrixXd::Zero(np + 1, n);
  for (size_t k = 0; k <= np; k++) {
    goal_x(k, 0) = 3.0;
    goal_x(k, 1) = 4.0;
  }
  mpc->setGoalX(goal_x);
  mpc->setGoalU(MatrixXd::Zero(np, m));
  mpc->setPose(VectorXd::Zero(n));
  mpc->solve();
  return mpc;
}
}  // namespace

TEST(CustomModel, DummyLinearDrivesTowardGoal)
{
  auto model = std::make_shared<DummyLinear>();
  const size_t n = model->getN();
  const size_t m = model->getM();
  const size_t np = 15;
  const size_t nc = 15;

  MatrixXd Q = 10.0 * MatrixXd::Identity(n, n);
  MatrixXd S = 2.0 * Q;
  MatrixXd R = 0.1 * MatrixXd::Identity(m, m);
  MatrixXd W = MatrixXd::Constant(1, 1, 100.0);

  auto mpc = std::make_shared<MPC>();
  mpc->setNp(np);
  mpc->setNc(nc);
  mpc->setdt(0.1);
  mpc->setQ(Q);
  mpc->setS(S);
  mpc->setR(R);
  mpc->setW(W);
  mpc->init(model);   // the solver accepts a model defined in the test

  const double goal_px = 3.0;
  const double goal_py = 4.0;
  MatrixXd goal_x = MatrixXd::Zero(np + 1, n);
  for (size_t k = 0; k <= np; k++) {
    goal_x(k, 0) = goal_px;
    goal_x(k, 1) = goal_py;
  }
  MatrixXd goal_u = MatrixXd::Zero(nc, m);
  mpc->setGoalX(goal_x);
  mpc->setGoalU(goal_u);

  mpc->setPose(VectorXd::Zero(n));
  auto [x, u] = mpc->solve();

  // Shapes: (Np+1) x n states, Nc x m controls.
  ASSERT_EQ(x.rows(), static_cast<Eigen::Index>(np + 1));
  ASSERT_EQ(x.cols(), static_cast<Eigen::Index>(n));
  ASSERT_EQ(u.rows(), static_cast<Eigen::Index>(nc));
  ASSERT_EQ(u.cols(), static_cast<Eigen::Index>(m));

  // Finite trajectories.
  EXPECT_TRUE(isFinite(x));
  EXPECT_TRUE(isFinite(u));

  // The terminal predicted state is closer to the goal than the start.
  const double d_start =
    std::hypot(x(0, 0) - goal_px, x(0, 1) - goal_py);
  const double d_end =
    std::hypot(x(np, 0) - goal_px, x(np, 1) - goal_py);
  EXPECT_LT(d_end, d_start);
  EXPECT_EQ(mpc->qp_info.status, proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED);
}

// Move-blocking: with Np > Nc the prediction nodes beyond the control horizon
// reuse the last control block (the setE column clamp). The QP must still size,
// solve, and drive the terminal state toward the goal.
TEST(CustomModel, MoveBlockingNpGreaterThanNc)
{
  auto model = std::make_shared<DummyLinear>();
  const size_t n = model->getN();
  const size_t m = model->getM();
  const size_t np = 20;
  const size_t nc = 5;   // Np > Nc

  MatrixXd Q = 10.0 * MatrixXd::Identity(n, n);
  MatrixXd S = 2.0 * Q;
  MatrixXd R = 0.1 * MatrixXd::Identity(m, m);
  MatrixXd W = MatrixXd::Constant(1, 1, 100.0);

  auto mpc = std::make_shared<MPC>();
  mpc->setNp(np);
  mpc->setNc(nc);
  mpc->setdt(0.1);
  mpc->setQ(Q);
  mpc->setS(S);
  mpc->setR(R);
  mpc->setW(W);
  mpc->init(model);   // must size and configure with Np != Nc

  const double goal_px = 3.0;
  const double goal_py = 4.0;
  MatrixXd goal_x = MatrixXd::Zero(np + 1, n);
  for (size_t k = 0; k <= np; k++) {
    goal_x(k, 0) = goal_px;
    goal_x(k, 1) = goal_py;
  }
  MatrixXd goal_u = MatrixXd::Zero(nc, m);
  mpc->setGoalX(goal_x);
  mpc->setGoalU(goal_u);

  mpc->setPose(VectorXd::Zero(n));
  auto [x, u] = mpc->solve();

  // Shapes follow Np (states) and Nc (controls), not a single horizon.
  ASSERT_EQ(x.rows(), static_cast<Eigen::Index>(np + 1));
  ASSERT_EQ(u.rows(), static_cast<Eigen::Index>(nc));
  EXPECT_TRUE(isFinite(x));
  EXPECT_TRUE(isFinite(u));
  EXPECT_EQ(mpc->qp_info.status, proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED);

  const double d_start = std::hypot(x(0, 0) - goal_px, x(0, 1) - goal_py);
  const double d_end = std::hypot(x(np, 0) - goal_px, x(np, 1) - goal_py);
  EXPECT_LT(d_end, d_start);
}

// The first-control-rate row applies the bound to the control component the
// "du" entry declares, not to the entry's position in the map. A model that
// rate-limits only its second control must constrain u[1], not u[0].
TEST(CustomModel, FirstControlRateRowUsesDeclaredControlIndex)
{
  const size_t np = 10;
  auto model = std::make_shared<DuOnSecondControl>();
  const size_t m = model->getM();
  auto mpc = solveTowardGoal(model, np);

  auto solver = mpc->getSolver();
  const MatrixXd & C = solver->getC();
  const size_t u_start = solver->getWStart() - np * m;   // Nc == np here

  // One "du" entry -> exactly one first-control-rate row (row 0).
  EXPECT_DOUBLE_EQ(C(0, u_start + 1), 1.0);   // acts on u[1], the rate-limited control
  EXPECT_DOUBLE_EQ(C(0, u_start + 0), 0.0);   // and not on u[0], which declares no rate bound
}

// Every one of the Nc control blocks carries R, matching the documented cost
// sum over k = 0..Nc-1. With the state weights zeroed the control tracking term
// is the only cost, so an unweighted terminal block would leave the last
// control sitting at its linearization point instead of on goal_u.
TEST(CustomModel, TerminalControlBlockCarriesTheControlWeight)
{
  auto model = std::make_shared<DummyLinear>();
  const size_t n = model->getN();
  const size_t m = model->getM();
  const size_t np = 5;
  constexpr double kGoalU = 1.0;

  auto mpc = std::make_shared<MPC>();
  mpc->setNp(np);
  mpc->setNc(np);
  mpc->setdt(0.1);
  mpc->setQ(MatrixXd::Zero(n, n));
  mpc->setS(MatrixXd::Zero(n, n));
  mpc->setR(MatrixXd::Identity(m, m));
  mpc->setW(MatrixXd::Constant(1, 1, 100.0));
  mpc->init(model);

  mpc->setGoalX(MatrixXd::Zero(np + 1, n));
  mpc->setGoalU(MatrixXd::Constant(np, m, kGoalU));
  mpc->setPose(VectorXd::Zero(n));

  const auto [x, u] = mpc->solve();
  ASSERT_EQ(static_cast<size_t>(u.rows()), np);
  for (size_t k = 0; k < np; k++) {
    for (size_t j = 0; j < m; j++) {
      EXPECT_NEAR(u(k, j), kGoalU, 1e-3) << "control row " << k << ", column " << j;
    }
  }
}

// setIneq() validates the bound index against the model's declared state/control
// dimension where it is known, instead of writing an out-of-range row when the
// QP is assembled.
TEST(CustomModel, SetIneqRejectsIndexOutOfRange)
{
  DummyLinear model;   // n = 2, m = 2
  EXPECT_THROW(model.setIneq("x", 5, -1.0, 1.0), std::invalid_argument);
  EXPECT_THROW(model.setIneq("u", 5, -1.0, 1.0), std::invalid_argument);
  EXPECT_NO_THROW(model.setIneq("x", 1, -1.0, 1.0));   // in range
}

// updateIneq() reports a bound it cannot find instead of silently doing nothing.
TEST(CustomModel, UpdateIneqReportsMissingBound)
{
  DummyLinear model;   // declares "u" bounds only, no "x" bound
  EXPECT_THROW(model.updateIneq("x", 0, -1.0, 1.0), std::invalid_argument);
  EXPECT_NO_THROW(model.updateIneq("u", 0, -1.5, 1.5));   // existing bound
}

// Negative compile check: a model that omits an override stays abstract and
// cannot be instantiated. Define PROX_MPC_NEGATIVE_COMPILE_CHECK to confirm the
// build fails with an abstract-type error.
#ifdef PROX_MPC_NEGATIVE_COMPILE_CHECK
namespace
{
class MissingOverride : public Model
{
public:
  MissingOverride() {setN(2); setM(2);}
  void updatec(double dt, VectorXd x_next) override {c = getX() - x_next + dt * getU();}
  void updateA(double) override {A = MatrixXd::Identity(getN(), getN());}
  // updateB() intentionally omitted -> MissingOverride stays abstract.
};
MissingOverride g_should_not_compile;  // error: abstract type
}  // namespace
#endif
