// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Tests for the obstacle-avoidance constraints: the linearized signed-distance
// half-plane with K slots per node, consistent (node, slot) indexing, a slack
// block sized Np*K, and a far sentinel for empty slots. The cases cover:
//   - constraint geometry: a single obstacle forces a clearance detour;
//   - indexing: each obstacle row and its slack share a (node, slot), and the
//     slack block has exactly Np*K entries;
//   - disabling: K=0 and an all-sentinel layout reproduce the obstacle-off result;
//   - multiple obstacles: K=2 keeps clearance from both, and a sentinel slot
//     reduces the result to the single-obstacle case.

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include <prox_mpc/mpc.hpp>
#include <prox_mpc/models/bicycle.hpp>
#include <prox_mpc/models/unicycle.hpp>
#include <prox_mpc/proxqp.hpp>

using prox_mpc::Bicycle;
using prox_mpc::MPC;
using prox_mpc::Model;
using prox_mpc::Unicycle;

namespace
{
constexpr double kDt = 0.1;
constexpr size_t kNp = 30;
constexpr size_t kNc = 30;

// One obstacle triple per (node, slot); empty slots at the far sentinel.
MatrixXd makeObs(size_t np, size_t k)
{
  MatrixXd obs = MatrixXd::Zero(np * k, 3);
  for (Eigen::Index r = 0; r < obs.rows(); r++) {
    obs(r, 0) = MPC::kObsFarSentinel;
    obs(r, 1) = MPC::kObsFarSentinel;
    obs(r, 2) = 0.0;
  }
  return obs;
}

void fillSlot(MatrixXd & obs, size_t np, size_t k, size_t slot, double ox, double oy, double d)
{
  for (size_t node = 0; node < np; node++) {
    obs(node * k + slot, 0) = ox;
    obs(node * k + slot, 1) = oy;
    obs(node * k + slot, 2) = d;
  }
}

std::shared_ptr<MPC> makeUnicycleMpc(
  size_t k_obs, double w_weight = 1000.0, double cbf_gamma = 1.0)
{
  auto model = std::make_shared<Unicycle>();
  const size_t n = model->getN();
  const size_t m = model->getM();

  MatrixXd Q = MatrixXd::Identity(n, n);
  Q(0, 0) = 10.0;
  Q(1, 1) = 10.0;
  MatrixXd S = 2.0 * Q;
  MatrixXd R = 0.1 * MatrixXd::Identity(m, m);
  MatrixXd W = MatrixXd::Constant(1, 1, w_weight);

  auto mpc = std::make_shared<MPC>();
  mpc->setNp(kNp);
  mpc->setNc(kNc);
  mpc->setdt(kDt);
  mpc->setQ(Q);
  mpc->setS(S);
  mpc->setR(R);
  mpc->setW(W);
  mpc->setMaxObs(k_obs);
  mpc->setCbfGamma(cbf_gamma);
  mpc->init(model);

  MatrixXd goal_x = MatrixXd::Zero(kNp + 1, n);   // straight reference to (6, 0, 0)
  for (size_t i = 0; i <= kNp; i++) {
    goal_x(i, 0) = 6.0;
  }
  MatrixXd goal_u = MatrixXd::Zero(kNc, m);        // v_ref in column 0
  for (size_t i = 0; i < kNc; i++) {
    goal_u(i, 0) = 1.0;
  }
  mpc->setGoalX(goal_x);
  mpc->setGoalU(goal_u);
  return mpc;
}

// Closed-loop run (advance pose to the predicted next state, as the demo does).
// Tracks the closest approach to the given obstacle point and the max |y|.
struct LoopStats
{
  double min_dist;   // closest approach over the traversed path
  double max_abs_y;  // largest lateral deviation
  MatrixXd last_x;   // last predicted trajectory
  MatrixXd last_u;   // last predicted controls
};

LoopStats runLoop(
  const std::shared_ptr<MPC> & mpc, size_t n, size_t steps,
  double ox, double oy)
{
  LoopStats s{std::numeric_limits<double>::infinity(), 0.0, {}, {}};
  VectorXd pose = VectorXd::Zero(n);
  for (size_t step = 0; step < steps; step++) {
    mpc->setPose(pose);
    auto [x, u] = mpc->solve();
    s.last_x = x;
    s.last_u = u;
    pose = x.row(1);
    s.min_dist = std::min(s.min_dist, std::hypot(pose(0) - ox, pose(1) - oy));
    s.max_abs_y = std::max(s.max_abs_y, std::abs(pose(1)));
  }
  return s;
}
}  // namespace

// A single obstacle offset from the straight path forces the robot to keep
// approximately d_safe clearance and to detour laterally around it.
TEST(ObstacleK, ConstraintGeometryDetour)
{
  const double ox = 2.5;
  const double oy = 0.5;
  const double d_safe = 1.0;

  auto mpc = makeUnicycleMpc(1);
  MatrixXd obs = makeObs(kNp, 1);
  fillSlot(obs, kNp, 1, 0, ox, oy, d_safe);
  mpc->setObs(obs);

  auto s = runLoop(mpc, 3, 70, ox, oy);

  // The traversed path keeps clearance close to d_safe (small soft violation).
  EXPECT_GE(s.min_dist, d_safe - 0.15);
  // The robot detoured laterally to get around the obstacle.
  EXPECT_GT(s.max_abs_y, 0.3);
}

// The discrete-time CBF coupling (cbf_gamma < 1) is wired and active: it changes
// the avoidance trajectory relative to the pointwise term (cbf_gamma = 1) while
// both keep clearance and advance past the obstacle. (gamma = 1 must reproduce the
// pointwise term exactly; that regression is covered by ConstraintGeometryDetour,
// which runs at the default gamma = 1.)
TEST(ObstacleK, CbfGammaChangesAvoidanceTrajectory)
{
  const double ox = 2.5;
  const double oy = 0.5;
  const double d_safe = 1.0;

  auto build = [&](double gamma) {
      auto mpc = makeUnicycleMpc(1, 1000.0, gamma);
      MatrixXd obs = makeObs(kNp, 1);
      fillSlot(obs, kNp, 1, 0, ox, oy, d_safe);
      mpc->setObs(obs);
      return mpc;
    };

  auto s_pointwise = runLoop(build(1.0), 3, 70, ox, oy);   // gamma = 1 -> pointwise
  auto s_cbf = runLoop(build(0.3), 3, 70, ox, oy);          // gamma < 1 -> CBF coupling

  // Both keep clearance and advance past the obstacle in x.
  EXPECT_GE(s_pointwise.min_dist, d_safe - 0.15);
  EXPECT_GE(s_cbf.min_dist, d_safe - 0.15);
  EXPECT_GT(s_pointwise.last_x(1, 0), ox);
  EXPECT_GT(s_cbf.last_x(1, 0), ox);

  // The coupling demonstrably changes the realized trajectory.
  EXPECT_GT(std::abs(s_cbf.max_abs_y - s_pointwise.max_abs_y), 1e-3);
}

// Each obstacle constraint row and its slack share the same (node, slot), and
// the slack decision block has exactly Np*K entries with no dangling slack.
TEST(ObstacleK, IndexingConsistency)
{
  const size_t k = 2;
  auto mpc = makeUnicycleMpc(k);
  MatrixXd obs = makeObs(kNp, k);
  fillSlot(obs, kNp, k, 0, 2.5, 0.5, 1.0);
  mpc->setObs(obs);

  mpc->setPose(VectorXd::Zero(3));
  mpc->solve();

  auto solver = mpc->getSolver();
  const std::vector<size_t> & ineq_idx = solver->getIneqIdx();
  const MatrixXd & C = solver->getC();
  const size_t w_start = solver->getWStart();

  ASSERT_EQ(solver->getMaxObs(), k);

  // Slack decision block is exactly Np*K (no dangling slack).
  EXPECT_EQ(solver->getNDvars() - w_start, kNp * k);

  // The last two inequality blocks are the obstacle rows and the slack rows.
  const size_t nb = ineq_idx.size();
  ASSERT_GE(nb, 3u);
  const size_t obs_start = ineq_idx[nb - 3];
  const size_t obs_end = ineq_idx[nb - 2];
  const size_t slack_start = ineq_idx[nb - 2];
  const size_t slack_end = ineq_idx[nb - 1];

  EXPECT_EQ(obs_end - obs_start, kNp * k);      // Np*K obstacle rows
  EXPECT_EQ(slack_end - slack_start, kNp * k);  // Np*K slack rows

  // Obstacle row r (slot) and slack row r (same slot) both carry the +1 slack
  // coefficient at column w_start + slot.
  for (size_t r = obs_start; r < obs_end; r++) {
    const size_t slot = r - obs_start;
    EXPECT_DOUBLE_EQ(C(r, w_start + slot), 1.0);
  }
  for (size_t r = slack_start; r < slack_end; r++) {
    const size_t slot = r - slack_start;
    EXPECT_DOUBLE_EQ(C(r, w_start + slot), 1.0);
  }
}

// setObs rejects a wrong-shaped obstacle matrix up front; the per-step assembly
// indexes obs(slot, .) with EIGEN_NO_DEBUG, so an undersized matrix would
// otherwise read out of bounds on the hot path instead of being caught.
TEST(ObstacleK, SetObsRejectsWrongShape)
{
  auto mpc = makeUnicycleMpc(2);   // Np = 30, K = 2 -> expects (Np*K) x 3 = 60 x 3
  auto solver = mpc->getSolver();
  ASSERT_EQ(solver->getMaxObs(), 2u);

  EXPECT_THROW(solver->setObs(MatrixXd::Zero(10, 3)), std::invalid_argument);       // too few rows
  EXPECT_THROW(solver->setObs(MatrixXd::Zero(kNp * 2, 2)), std::invalid_argument);  // wrong cols
  EXPECT_NO_THROW(solver->setObs(makeObs(kNp, 2)));                                  // correct shape
}

// cbf_gamma outside (0, 1] is rejected at both layers. Outside that range the
// bound (1 - gamma) * h_prev - h - w turns positive for the far sentinel, so
// unused slots would become hard-binding at sentinel magnitude.
TEST(ObstacleK, CbfGammaRejectsOutOfRange)
{
  prox_mpc::MPC mpc;
  EXPECT_THROW(mpc.setCbfGamma(0.0), std::invalid_argument);
  EXPECT_THROW(mpc.setCbfGamma(-0.1), std::invalid_argument);
  EXPECT_THROW(mpc.setCbfGamma(1.5), std::invalid_argument);
  EXPECT_NO_THROW(mpc.setCbfGamma(1e-3));
  EXPECT_NO_THROW(mpc.setCbfGamma(1.0));

  prox_mpc::ProxQP solver;
  EXPECT_THROW(solver.setCbfGamma(0.0), std::invalid_argument);
  EXPECT_THROW(solver.setCbfGamma(1.5), std::invalid_argument);
  EXPECT_NO_THROW(solver.setCbfGamma(0.5));
}

// K=0 reproduces the obstacle-off result exactly, and K=1 with every slot at the
// far sentinel reproduces it within tolerance.
TEST(ObstacleK, DisabledAndSentinelReproduceObstacleOff)
{
  auto buildBike = [](size_t k_obs) {
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
      mpc->setMaxObs(k_obs);
      mpc->init(model);
      MatrixXd goal_x = MatrixXd::Zero(21, n);
      for (size_t i = 0; i <= 20; i++) {
        goal_x(i, 0) = 5.0;
      }
      MatrixXd goal_u = MatrixXd::Zero(20, m);
      for (size_t i = 0; i < 20; i++) {
        goal_u(i, 0) = 1.0;
      }
      mpc->setGoalX(goal_x);
      mpc->setGoalU(goal_u);
      mpc->setPose(VectorXd::Zero(n));
      return mpc;
    };

  // K=0 must reproduce the obstacle-off result exactly.
  auto [x0, u0] = buildBike(0)->solve();
  EXPECT_NEAR(u0(0, 0), 0.050000065264268402, 1e-6);

  // K=1 with all slots at the sentinel is provably non-binding -> same solution.
  auto mpc_sentinel = buildBike(1);
  mpc_sentinel->setObs(makeObs(20, 1));   // all far sentinel
  auto [xs, us] = mpc_sentinel->solve();
  EXPECT_NEAR(us(0, 0), u0(0, 0), 1e-4);
  EXPECT_NEAR(xs(20, 0), x0(20, 0), 1e-4);
}

// With K=2 the robot keeps clearance from both obstacles; pushing the second
// slot to the sentinel reduces the solution to the single-obstacle case.
TEST(ObstacleK, MultiObstacleAndSentinelReducesToK1)
{
  const double ax = 2.5;
  const double ay = 0.5;
  const double bx = 4.0;
  const double by = -0.5;
  const double d_safe = 1.0;

  // K=2 with both obstacles.
  auto mpc2 = makeUnicycleMpc(2);
  MatrixXd obs2 = makeObs(kNp, 2);
  fillSlot(obs2, kNp, 2, 0, ax, ay, d_safe);
  fillSlot(obs2, kNp, 2, 1, bx, by, d_safe);
  mpc2->setObs(obs2);
  auto s2a = runLoop(mpc2, 3, 70, ax, ay);
  // Re-run a fresh loop to measure clearance from the second obstacle.
  auto mpc2b = makeUnicycleMpc(2);
  MatrixXd obs2b = makeObs(kNp, 2);
  fillSlot(obs2b, kNp, 2, 0, ax, ay, d_safe);
  fillSlot(obs2b, kNp, 2, 1, bx, by, d_safe);
  mpc2b->setObs(obs2b);
  auto s2b = runLoop(mpc2b, 3, 70, bx, by);
  EXPECT_GE(s2a.min_dist, d_safe - 0.2);   // clearance from obstacle A
  EXPECT_GE(s2b.min_dist, d_safe - 0.2);   // clearance from obstacle B

  // K=2 with slot 1 at the sentinel must match K=1 with only obstacle A.
  auto mpc_k2_sent = makeUnicycleMpc(2);
  MatrixXd obs_k2 = makeObs(kNp, 2);
  fillSlot(obs_k2, kNp, 2, 0, ax, ay, d_safe);   // slot 1 left at sentinel
  mpc_k2_sent->setObs(obs_k2);
  auto s_k2 = runLoop(mpc_k2_sent, 3, 40, ax, ay);

  auto mpc_k1 = makeUnicycleMpc(1);
  MatrixXd obs_k1 = makeObs(kNp, 1);
  fillSlot(obs_k1, kNp, 1, 0, ax, ay, d_safe);
  mpc_k1->setObs(obs_k1);
  auto s_k1 = runLoop(mpc_k1, 3, 40, ax, ay);

  // Same realized trajectory (the extra sentinel slot is non-binding).
  for (Eigen::Index r = 0; r < s_k1.last_x.rows(); r++) {
    EXPECT_NEAR(s_k2.last_x(r, 0), s_k1.last_x(r, 0), 1e-3);
    EXPECT_NEAR(s_k2.last_x(r, 1), s_k1.last_x(r, 1), 1e-3);
  }
}
