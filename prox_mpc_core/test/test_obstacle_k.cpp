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

// Like fillSlot, but only over [node_lo, node_hi): fillSlot writes the same
// triple at every node, so no test built from it alone can produce a per-node
// slot discontinuity (e.g. the far sentinel at one node and a real obstacle at
// the next). Call it twice on the same slot with adjoining ranges to place that
// discontinuity at a chosen node boundary.
void fillSlotRange(
  MatrixXd & obs, size_t k, size_t slot, size_t node_lo, size_t node_hi,
  double ox, double oy, double d)
{
  for (size_t node = node_lo; node < node_hi; node++) {
    obs(node * k + slot, 0) = ox;
    obs(node * k + slot, 1) = oy;
    obs(node * k + slot, 2) = d;
  }
}

std::shared_ptr<MPC> makeUnicycleMpc(
  size_t k_obs, double w_weight = 1000.0, double cbf_gamma = 1.0, bool warm_start = true)
{
  auto model = std::make_shared<Unicycle>();
  const size_t n = model->getN();
  const size_t m = model->getM();

  MatrixXd Q = MatrixXd::Identity(n, n);
  // GCC's -Wnull-dereference misfires on this exact shape at -O3/NDEBUG (an
  // Identity()-initialized matrix, one element written, then passed by value
  // into a call GCC inlines): verified as a false positive by reproducing it
  // in isolation, where neither an explicit Eigen::Index cast nor splitting
  // the construction from the identity fill clears it. No rename, cast or
  // scope change is available here; the site is narrowed to this one write.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnull-dereference"
  Q(0, 0) = 10.0;
#pragma GCC diagnostic pop
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
  mpc->setWarmStart(warm_start);
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

// The cross-cycle warm start updates the solver workspace in place rather than
// rebuilding it, which ProxQP only honours while the sparsity structure stays
// put. A structure derived from the values would move as the half-plane normals
// pass through zero, and ProxQP would silently ignore the update - dropping the
// keep-out rows and letting the robot drive through the obstacle. This pins that
// the constraint survives a long run on the update path, and that turning the
// warm start off produces the same avoidance.
TEST(ObstacleK, WarmStartPreservesTheKeepOutRows)
{
  const double ox = 2.5;
  const double oy = 0.5;
  const double d_safe = 1.0;

  auto run = [&](bool warm_start) {
      auto mpc = makeUnicycleMpc(1, 1000.0, 1.0, warm_start);
      MatrixXd obs = makeObs(kNp, 1);
      fillSlot(obs, kNp, 1, 0, ox, oy, d_safe);
      mpc->setObs(obs);
      return runLoop(mpc, 3, 70, ox, oy);
    };

  const auto warm = run(true);
  const auto cold = run(false);

  // The keep-out holds on the update path, not only on the first (init) cycle.
  EXPECT_GE(warm.min_dist, d_safe - 0.15);
  EXPECT_GT(warm.max_abs_y, 0.3);

  // And the two paths agree: the warm start changes how the QP is solved, not
  // which problem is solved.
  EXPECT_NEAR(warm.min_dist, cold.min_dist, 0.05);
  EXPECT_NEAR(warm.max_abs_y, cold.max_abs_y, 0.05);
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

  // Both keep clearance and advance past the obstacle in x. The coupled bound is
  // a rate condition rather than a pointwise one: it permits the clearance to
  // decay toward the keep-out as gamma falls, so its tolerance is looser than the
  // pointwise one rather than equal to it.
  EXPECT_GE(s_pointwise.min_dist, d_safe - 0.15);
  EXPECT_GE(s_cbf.min_dist, d_safe - 0.25);
  EXPECT_GT(s_pointwise.last_x(1, 0), ox);
  EXPECT_GT(s_cbf.last_x(1, 0), ox);

  // The coupling demonstrably changes the realized trajectory, in the direction
  // the rate condition predicts: a closer, less lateral pass than the pointwise
  // term takes.
  EXPECT_GT(std::abs(s_cbf.max_abs_y - s_pointwise.max_abs_y), 1e-3);
  EXPECT_LT(s_cbf.max_abs_y, s_pointwise.max_abs_y);
  EXPECT_LT(s_cbf.min_dist, s_pointwise.min_dist);
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

// At exact coincidence (predicted position equals the obstacle center) the
// emitted constraint normal must still be unit length: dividing a zero delta
// by only a floored norm would otherwise emit (0, 0), leaving slack as the
// sole thing resisting the constraint.
TEST(ObstacleK, CoincidentObstacleNormalIsUnitLength)
{
  auto mpc = makeUnicycleMpc(1);   // K = 1

  const double ox = 1.0;
  const double oy = 0.0;
  MatrixXd obs = makeObs(kNp, 1);
  fillSlot(obs, kNp, 1, 0, ox, oy, 0.5);

  // setC() reads the solver's own obs member, populated only through
  // ProxQP::setObs() (normally forwarded by MPC::solve() each cycle); calling
  // setC() directly here needs it set the same way.
  auto solver = mpc->getSolver();
  solver->setObs(obs);
  MatrixXd x = MatrixXd::Zero(kNp + 1, 3);   // unicycle state [x, y, theta]
  x(1, 0) = ox;   // node 1 exactly coincides with the obstacle center
  x(1, 1) = oy;
  solver->setC(x);

  const MatrixXd & C = solver->getC();
  const std::vector<size_t> & ineq_idx = solver->getIneqIdx();
  const size_t obs_start = ineq_idx[ineq_idx.size() - 3];
  const size_t col = 3;   // position block of node 1 (x_start == 0, n == 3)

  const double nx = C(obs_start, col);
  const double ny = C(obs_start, col + 1);
  EXPECT_NEAR(nx * nx + ny * ny, 1.0, 1e-9);
}

// The default per-node static fill does not preserve slot identity across
// nodes; at cbf_gamma < 1 a slot holding the far sentinel at one node and a
// real obstacle at the next node must not blow up the coupling term. This uses
// a local fill (not the shared makeObs/fillSlot helpers, which fill every node
// identically and cannot express a per-node discontinuity) to construct that
// exact case and checks the realized slack stays bounded.
TEST(ObstacleK, SentinelDiscontinuityDoesNotBlowUpSlack)
{
  auto mpc = makeUnicycleMpc(1, 100.0, 0.3);   // K = 1, cbf_gamma < 1

  MatrixXd obs = MatrixXd::Zero(kNp, 3);
  for (Eigen::Index r = 0; r < obs.rows(); r++) {
    obs(r, 0) = MPC::kObsFarSentinel;
    obs(r, 1) = MPC::kObsFarSentinel;
    obs(r, 2) = 0.0;
  }
  // Node 0's slot stays at the sentinel; node 1's slot holds a real obstacle,
  // creating the (sentinel at node k, real obstacle at node k+1) discontinuity.
  obs(1, 0) = 2.5;
  obs(1, 1) = 0.0;
  obs(1, 2) = 1.0;
  mpc->setObs(obs);

  mpc->setPose(VectorXd::Zero(3));
  mpc->solve();
  EXPECT_EQ(mpc->qp_info.status, proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED);

  // Before the fix the coupling term (1 - gamma) * obs_h_prev reaches ~1e5 at
  // this transition, forcing the slack to absorb it; after the fix it stays at
  // an ordinary avoidance magnitude.
  EXPECT_LT(mpc->getMaxObstacleSlack(), 10.0);
}

// fillSlotRange places a real obstacle from one node onward while the same slot
// stays at the far sentinel before it -- the (sentinel at node k, real obstacle
// at node k+1) discontinuity fillSlot cannot express, since it writes the same
// triple at every node. The core guard at proxqp.cpp:433-436 already lands on
// this branch; this exercises it at cbf_gamma = 0.5 and records the observed QP
// status and largest obstacle slack as a characterization.
TEST(ObstacleK, SlotDiscontinuityAtHalfGammaStaysBounded)
{
  const size_t k = 1;
  const size_t node_k = 4;   // last sentinel node; node_k + 1 is the first real one

  auto mpc = makeUnicycleMpc(k, 100.0, 0.5);   // K = 1, cbf_gamma = 0.5
  MatrixXd obs = makeObs(kNp, k);              // every node starts at the far sentinel
  fillSlotRange(obs, k, 0, node_k + 1, kNp, 2.5, 0.5, 1.0);
  mpc->setObs(obs);

  mpc->setPose(VectorXd::Zero(3));
  mpc->solve();

  const auto status = mpc->qp_info.status;
  const double max_slack = mpc->getMaxObstacleSlack();
  RecordProperty("qp_status", static_cast<int>(status));
  RecordProperty("max_obstacle_slack", std::to_string(max_slack));

  EXPECT_EQ(status, proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED);
  EXPECT_LT(max_slack, 10.0);   // ordinary avoidance magnitude, not the ~1e5 blow-up
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

// --- The keep-out rows follow the model's declared planar mapping ------------
//
// The obstacle rows are the only part of the assembly that has to know where a
// model keeps its position. They read it from Model::getPlanarMapping(), so a
// model whose state is not ordered [x, y, ...] is constrained on the axes it
// declares rather than on state columns 0 and 1.

// A model whose state is ordered [theta, x, y] and which enables obstacle
// avoidance: the case the assembly used to index wrongly. Unicycle dynamics
// written against the permuted ordering, so it also solves.
class PermutedObstacleModel : public prox_mpc::Model
{
public:
  PermutedObstacleModel()
  {
    setName("permuted_obstacle");
    setN(3);   // state: [theta, x, y]
    setM(2);   // control: [v, omega]
    setA(MatrixXd::Zero(getN(), getN()));
    setB(MatrixXd::Zero(getN(), getM()));
    setc(VectorXd::Zero(getN()));
    setIneq("u", 0, -3.0, 3.0);
    setIneq("u", 1, -1.0, 1.0);
    setIneq("du", 0, -0.5, 0.5);
    setIneq("du", 1, -0.5, 0.5);
    setObsAvoid(true);
  }
  prox_mpc::PlanarMapping getPlanarMapping() const override
  {
    prox_mpc::PlanarMapping mapping;
    mapping.idx_yaw = 0;
    mapping.idx_x = 1;
    mapping.idx_y = 2;
    return mapping;
  }
  void updatec(double dt, VectorXd x_next) override
  {
    const double th = getX()(0);
    c << getX()(0) - x_next(0) + dt * getU()(1),
      getX()(1) - x_next(1) + dt * getU()(0) * std::cos(th),
      getX()(2) - x_next(2) + dt * getU()(0) * std::sin(th);
  }
  void updateA(double dt) override
  {
    const double th = getX()(0);
    const double v = getU()(0);
    A << 1.0, 0.0, 0.0,
      -dt * v * std::sin(th), 1.0, 0.0,
      dt * v * std::cos(th), 0.0, 1.0;
  }
  void updateB() override
  {
    const double th = getX()(0);
    B << 0.0, 1.0,
      std::cos(th), 0.0,
      std::sin(th), 0.0;
  }
};

// The half-plane normal lands in the two state columns the model declares as
// its position, and the column it declares as heading is left untouched. Read
// out of the assembled constraint matrix rather than inferred from a solution:
// a wrongly placed normal still produces a plausible trajectory, just one
// constrained on the wrong pair of axes.
TEST(ObstacleK, KeepOutRowsFollowTheDeclaredPlanarMapping)
{
  auto model = std::make_shared<PermutedObstacleModel>();
  const size_t n = 3;
  auto mpc = std::make_shared<MPC>();
  mpc->setNp(kNp);
  mpc->setNc(kNc);
  mpc->setdt(kDt);
  mpc->setQ(MatrixXd::Identity(n, n));
  mpc->setS(MatrixXd::Identity(n, n));
  mpc->setR(0.1 * MatrixXd::Identity(2, 2));
  mpc->setW(MatrixXd::Constant(1, 1, 100.0));
  mpc->setMaxObs(1);
  mpc->init(model);

  // One obstacle, and a node-1 position offset from it along +x only, so the
  // expected normal is exactly (1, 0) in the declared axes.
  const double ox = 1.0;
  const double oy = 0.0;
  MatrixXd obs = makeObs(kNp, 1);
  fillSlot(obs, kNp, 1, 0, ox, oy, 0.5);

  auto solver = mpc->getSolver();
  solver->setObs(obs);
  MatrixXd x = MatrixXd::Zero(kNp + 1, n);
  x(1, 1) = ox + 2.0;   // declared x position of node 1
  x(1, 2) = oy;         // declared y position of node 1
  solver->setC(x);

  const MatrixXd & C = solver->getC();
  const std::vector<size_t> & ineq_idx = solver->getIneqIdx();
  const size_t obs_start = ineq_idx[ineq_idx.size() - 3];
  const size_t col = n * 1;   // x_start == 0; state block of node 1

  EXPECT_NEAR(C(obs_start, col + 1), 1.0, 1e-12);   // declared x column carries n_x
  EXPECT_NEAR(C(obs_start, col + 2), 0.0, 1e-12);   // declared y column carries n_y
  // The heading column is not a position and must carry no keep-out gradient.
  EXPECT_NEAR(C(obs_start, col + 0), 0.0, 1e-12);
}

// --- Current-time obstacle reconstruction at node 0 -------------------------
//
// The matrix's row 0 (block 0) carries the obstacle's position one step ahead
// (paired with x_sol row 1's time), not now, so the first coupled constraint's
// "previous node" side -- which must compare against the current pose x(0) --
// reconstructs the current-time position as 2*block0 - block1 rather than using
// block0 directly (proxqp.cpp's node == 0 branch). A two-node constant-velocity
// fill exercises the reconstruction directly, something no existing test above
// does: every other fill in this file is either fully static (uniform across
// nodes) or discontinuous well past node 0.

// A two-node constant-velocity obstacle fill: node 0's row (block 0, paired
// with x_sol row 1) and node 1's row (block 1, paired with x_sol row 2) hold
// positions one dt apart; the reconstructed current-time position is
// 2*block0 - block1, one more step back along the same line. Every other node
// stays at the far sentinel (single transient obstacle, not a persistent one).
MatrixXd makeConstantVelocityAtStart(
  size_t np, size_t k, double ox1, double oy1, double vx, double dt, double d_safe)
{
  MatrixXd obs = makeObs(np, k);
  obs(0, 0) = ox1;
  obs(0, 1) = oy1;
  obs(0, 2) = d_safe;
  obs(1, 0) = ox1 + vx * dt;
  obs(1, 1) = oy1;
  obs(1, 2) = d_safe;
  return obs;
}

// The first coupled row is bounded against the obstacle's position NOW, which
// the matrix carries no block for and which is recovered as
// 2 * block0 - block1. The bound is read straight out of the assembled system
// rather than inferred from the solution: the coupled row is absorbed by its
// slack at any shipped slack weight, so no closed-loop quantity moves and an
// ordering between two scenarios' slacks is satisfied whether the
// reconstruction runs or not.
//
// setC() and setd() are called directly on a chosen iterate so every term of
// low(row 0) = (1 - gamma) * h_prev(0) - h(0) - w(0) is closed-form:
//   h(0)      = ||x(1) - block0|| - d_safe            (node 0 constrains x(1))
//   h_prev(0) = ||x(0) - (2 * block0 - block1)|| - d_safe
//   w(0)      = 0
// With the robot at the origin, a head-on obstacle closing at 0.5 m/s and
// gamma = 0.5, that is -1.175. Bounding against block0 as though it were the
// current position - which is what happens without the reconstruction - gives
// -1.200 instead: the reconstruction is worth exactly
// (1 - gamma) * v_obs * dt = +0.025 of clearance floor here.
TEST(ObstacleK, FirstCoupledConstraintUsesReconstructedCurrentTimeObstacle)
{
  const size_t k = 1;
  const double ox1 = 2.9;      // block 0: obstacle position one step ahead of now
  const double vx = -0.5;      // closing head-on [m/s], well within kMaxObsSpeed
  const double d_safe = 0.5;
  const double cbf_gamma = 0.5;

  auto mpc = makeUnicycleMpc(k, 100.0, cbf_gamma);
  auto solver = mpc->getSolver();
  solver->setObs(makeConstantVelocityAtStart(kNp, k, ox1, 0.0, vx, kDt, d_safe));

  // Robot at the origin, and node 1 there too: the iterate is what makes every
  // term closed-form, not a solution.
  const MatrixXd x = MatrixXd::Zero(kNp + 1, 3);
  solver->setC(x);
  solver->setd(x, MatrixXd::Zero(kNc, 2), VectorXd::Zero(2), VectorXd::Zero(kNp * k));

  const std::vector<size_t> & ineq_idx = solver->getIneqIdx();
  const size_t obs_start = ineq_idx[ineq_idx.size() - 3];
  const double reconstructed_x = ox1 - vx * kDt;   // 2 * block0 - block1
  const double h = ox1 - d_safe;
  const double h_prev = reconstructed_x - d_safe;
  const double expected_low = (1.0 - cbf_gamma) * h_prev - h;

  EXPECT_NEAR(solver->getLow()(static_cast<Eigen::Index>(obs_start)), expected_low, 1e-12);
  // The same row bounded against block0 as "now"; the two differ by the
  // reconstruction's whole worth.
  const double block0_as_now_low = (1.0 - cbf_gamma) * (ox1 - d_safe) - h;
  EXPECT_NEAR(expected_low - block0_as_now_low, (1.0 - cbf_gamma) * std::abs(vx) * kDt, 1e-12);
}

// An implied inter-block step no obstacle could have travelled in one dt is
// treated as noise, not motion: above kMaxObsSpeed the backward extrapolation
// is skipped and the first block is used as the current-time position. The
// fallback is silent, so the assembled bound is the only place it is
// observable. Same closed-form reading as the case above, with the implied
// speed raised past the bound: 20 m/s over dt = 0.1 s implies a 2 m step
// against a kMaxObsSpeed * dt ceiling of 1 m.
TEST(ObstacleK, ImplausibleObstacleStepFallsBackToTheFirstBlock)
{
  const size_t k = 1;
  const double ox1 = 2.9;
  const double vx = -20.0;     // implied speed past kMaxObsSpeed = 10.0 m/s
  const double d_safe = 0.5;
  const double cbf_gamma = 0.5;

  auto mpc = makeUnicycleMpc(k, 100.0, cbf_gamma);
  auto solver = mpc->getSolver();
  solver->setObs(makeConstantVelocityAtStart(kNp, k, ox1, 0.0, vx, kDt, d_safe));

  const MatrixXd x = MatrixXd::Zero(kNp + 1, 3);
  solver->setC(x);
  solver->setd(x, MatrixXd::Zero(kNc, 2), VectorXd::Zero(2), VectorXd::Zero(kNp * k));

  const std::vector<size_t> & ineq_idx = solver->getIneqIdx();
  const size_t obs_start = ineq_idx[ineq_idx.size() - 3];
  const double h = ox1 - d_safe;
  // Block 0 used unchanged as "now", so both sides read the same position.
  const double expected_low = (1.0 - cbf_gamma) * (ox1 - d_safe) - h;
  EXPECT_NEAR(solver->getLow()(static_cast<Eigen::Index>(obs_start)), expected_low, 1e-12);

  // What the extrapolation would have produced had the guard not fired, kept
  // here so a widened bound shows up as a failure rather than as no change.
  const double extrapolated = ox1 - vx * kDt;
  const double unguarded_low = (1.0 - cbf_gamma) * (extrapolated - d_safe) - h;
  EXPECT_GT(std::abs(unguarded_low - expected_low), 1.0);
}

// Finite-difference Jacobian check of the coupled constraint's gradient columns
// (node >= 1, where they are written -- node 0's are inert because x(0) is
// pinned, per the comment at proxqp.cpp). setC() writes -( 1 - gamma) * grad h_k
// into the position columns of the previous node; this compares that analytic
// coefficient against a central finite difference of the closed-form clearance
// h_k(x) = ||x - o|| - d_safe the code implements, at the same operating point.
TEST(ObstacleK, CoupledConstraintGradientMatchesFiniteDifference)
{
  const size_t k = 1;
  const double gamma = 0.5;
  const double d_safe = 0.5;
  const double eps = 1e-6;

  auto mpc = makeUnicycleMpc(k, 100.0, gamma);
  // Real obstacle data at node 0 (block 0, prev-side for node 1) and node 1
  // (block 1, own side for node 1), so node 1's coupled row's gradient columns
  // are written (node >= 1) against a non-reconstructed, ordinary prev slot.
  const double ox = 2.5;
  const double oy = 0.3;
  MatrixXd obs = makeConstantVelocityAtStart(kNp, k, ox, oy, 3.0, kDt, d_safe);
  auto solver = mpc->getSolver();
  solver->setObs(obs);

  MatrixXd x = MatrixXd::Zero(kNp + 1, 3);
  x(0, 0) = 0.0; x(0, 1) = 0.0; x(0, 2) = 0.0;
  x(1, 0) = 1.7; x(1, 1) = 0.6; x(1, 2) = 0.1;   // node 1's own position
  x(2, 0) = 2.6; x(2, 1) = 0.5; x(2, 2) = 0.2;
  solver->setC(x);

  const MatrixXd & C = solver->getC();
  const std::vector<size_t> & ineq_idx = solver->getIneqIdx();
  const size_t obs_start = ineq_idx[ineq_idx.size() - 3];
  const size_t row = obs_start + 1;   // node 1's row (slot 1 of Np*K, K=1)
  const size_t col_prev = 3;          // position block of node 1 (x_start=0, n=3)

  const double gx_code = C(row, col_prev);
  const double gy_code = C(row, col_prev + 1);

  // h_prev(x) = ||x(0:2) - o|| - d_safe, at the SAME obstacle block 0's setObs()
  // resolves node 1's prev side to (ordinary, non-reconstructed since node >= 1).
  auto h_prev = [&](double px, double py) {return std::hypot(px - ox, py - oy) - d_safe;};
  const double fd_gx =
    (h_prev(x(1, 0) + eps, x(1, 1)) - h_prev(x(1, 0) - eps, x(1, 1))) / (2.0 * eps);
  const double fd_gy =
    (h_prev(x(1, 0), x(1, 1) + eps) - h_prev(x(1, 0), x(1, 1) - eps)) / (2.0 * eps);
  const double gain = -(1.0 - gamma);

  EXPECT_NEAR(gx_code, gain * fd_gx, 1e-6);
  EXPECT_NEAR(gy_code, gain * fd_gy, 1e-6);
}
