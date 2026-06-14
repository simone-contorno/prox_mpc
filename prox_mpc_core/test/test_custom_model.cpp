// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// A custom single-integrator model exercised through the Model interface,
// confirming the solver accepts a model defined outside the library. The
// pure-virtual hooks mean a model that omits an override does not compile; an
// opt-in negative compile check demonstrating this is included at the bottom.

#include <memory>

#include <gtest/gtest.h>

#include <prox_mpc/mpc.hpp>
#include <prox_mpc/model.hpp>

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

bool isFinite(const MatrixXd & m)
{
  return m.allFinite();
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
