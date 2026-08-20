// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Contract tests for the transactional solve/commit pair added to MPC
// (solveCandidate(), commitCandidate(), getCandidateFinite(), setU0()). Before
// this pair existed, MPC::solve() applied a QP's increments to x, u and w
// unconditionally, before checking whether the QP had converged, so a failed
// cycle silently corrupted the retained trajectories. Two cases:
//   - a genuinely failed QP (a state bound the current pose violates by
//     construction) leaves the previously retained x, u and w unchanged;
//   - a candidate carrying a non-finite value in a LATER horizon row (not just
//     row 0) is refused by commitCandidate()'s whole-horizon finiteness guard.
//     Whether proxsuite itself can produce that iterate through the public
//     solve path is not established, so this case exercises the guard
//     directly via a protected-member test subclass rather than depending on
//     an unreliable numerical trigger.
//
// Observability note: MPC's retained x has a public getter (getX()) and its
// retained u0 is a direct protected member, so both are read back directly
// below. The retained u and w trajectories have neither a public getter nor a
// protected member of their own (they live only in the privately-inherited
// MPCParams base, so even a derived class cannot name them) -- MPC exposes no
// way to read either back without calling solve()/solveCandidate() again,
// which would itself advance the state under test. Their preservation is
// therefore argued structurally rather than observed directly:
// commitCandidate() has a single early return before x, u, w and u0 are ever
// assigned (mpc.cpp), with no per-field branch inside that block, so a
// refused commit that leaves the two DIRECTLY observable members (x, u0)
// unchanged leaves u and w unchanged for the same reason.

#include <limits>
#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>

#include <proxsuite/proxqp/status.hpp>

#include <prox_mpc/mpc.hpp>
#include <prox_mpc/model.hpp>

using prox_mpc::MPC;
using prox_mpc::Model;

namespace
{

// Linear integrator: x_{k+1} = x_k + dt * u, state [x, y, theta], control
// [v, omega]. Declares a state bound on x[0] that the pose used for the
// "failed" cycle below sits far outside of, and the pose used for the
// "previous valid" cycle sits well inside of, so which cycle is feasible is
// controlled entirely by setPose() between two solves against the same fixed
// model, not by chasing solver iteration limits.
class BoundedLinearModel : public Model
{
public:
  BoundedLinearModel()
  {
    setName("bounded_linear");
    setN(3);
    setM(2);
    setA(MatrixXd::Zero(getN(), getN()));
    setB(MatrixXd::Zero(getN(), getM()));
    setc(VectorXd::Zero(getN()));
    setIneq("x", 0, 100.0, 200.0);   // only reachable from a pose already inside it
    setIneq("u", 0, -3.0, 3.0);
    setIneq("u", 1, -1.0, 1.0);
    setIneq("du", 0, -0.5, 0.5);
    setIneq("du", 1, -0.5, 0.5);
    setObsAvoid(true);
  }
  void updatec(double dt, VectorXd x_next) override
  {
    c << getX()(0) - x_next(0) + dt * getU()(0),
      getX()(1) - x_next(1),
      getX()(2) - x_next(2) + dt * getU()(1);
  }
  void updateA(double) override {A = MatrixXd::Identity(getN(), getN());}
  void updateB() override
  {
    B << 1.0, 0.0,
      0.0, 0.0,
      0.0, 1.0;
  }
};

// Exposes MPC's protected candidate-staging members (genuinely protected,
// direct members of MPC itself -- unlike the retained x/u/w, which live only
// in the privately-inherited MPCParams and so are not nameable from a derived
// class at all) and adds a way to exercise commitCandidate()'s finiteness
// guard directly, for the case where constructing a non-finite QP output
// through the public solve path is not established as reachable. White-box,
// no production change.
class TestableMpc : public MPC
{
public:
  using MPC::u0;
  using MPC::cand_x;
  using MPC::cand_u;
  using MPC::cand_w;
  using MPC::cand_solved;
  using MPC::cand_finite;
  using MPC::cand_u0;

  // Stage a candidate exactly the way solveCandidate() would have left one,
  // computing cand_finite with the identical whole-horizon allFinite()
  // reduction the production code uses, so commitCandidate() is exercised
  // against the real guard rather than a stand-in.
  void injectCandidate(const MatrixXd & cx, const MatrixXd & cu, const VectorXd & cw, bool solved)
  {
    cand_x = cx;
    cand_u = cu;
    cand_w = cw;
    cand_solved = solved;
    cand_finite = cand_x.allFinite() && cand_u.allFinite() && cand_w.allFinite();
    cand_u0 = cand_u.row(0);
  }
};

std::shared_ptr<TestableMpc> makeMpc()
{
  auto model = std::make_shared<BoundedLinearModel>();
  const size_t n = model->getN();
  const size_t m = model->getM();

  auto mpc = std::make_shared<TestableMpc>();
  mpc->setNp(6);
  mpc->setNc(6);
  mpc->setdt(0.1);
  mpc->setQ(10.0 * MatrixXd::Identity(n, n));
  mpc->setS(20.0 * MatrixXd::Identity(n, n));
  mpc->setR(0.1 * MatrixXd::Identity(m, m));
  mpc->setW(MatrixXd::Constant(1, 1, 100.0));
  mpc->setMaxObs(1);   // non-empty w, so its preservation argument is non-vacuous
  mpc->init(model);

  MatrixXd goal_x = MatrixXd::Zero(7, n);
  for (int i = 0; i <= 6; i++) {
    goal_x(i, 0) = 150.0;
  }
  MatrixXd goal_u = MatrixXd::Zero(6, m);
  mpc->setGoalX(goal_x);
  mpc->setGoalU(goal_u);

  MatrixXd obs = MatrixXd::Zero(6, 3);
  for (Eigen::Index r = 0; r < obs.rows(); r++) {
    obs(r, 0) = MPC::kObsFarSentinel;
    obs(r, 1) = MPC::kObsFarSentinel;
  }
  mpc->setObs(obs);
  return mpc;
}

void expectMatrixUnchanged(const MatrixXd & before, const MatrixXd & after, const char * what)
{
  ASSERT_EQ(before.rows(), after.rows()) << what;
  ASSERT_EQ(before.cols(), after.cols()) << what;
  EXPECT_LT((before - after).cwiseAbs().maxCoeff(), 1e-15) << what << " changed";
}
}  // namespace

// A genuinely failed QP (the pose lands outside the declared state bound, a
// hard infeasibility) leaves the previously retained x, u and w trajectories
// untouched: solveCandidate() computes into the candidate copies only, and
// commitCandidate() refuses to advance because the candidate did not converge.
TEST(TransactionalSolve, FailedSolveLeavesRetainedTrajectoriesUnchanged)
{
  auto mpc = makeMpc();

  // Cycle 1: pose inside the feasible band, trivial tracking error at the
  // goal -- converges immediately and becomes the retained baseline.
  mpc->setPose((VectorXd(3) << 150.0, 0.0, 0.0).finished());
  auto [x1, u1] = mpc->solveCandidate();
  (void)x1;
  (void)u1;
  ASSERT_EQ(mpc->qp_info.status, proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED);
  ASSERT_TRUE(mpc->getCandidateFinite());
  ASSERT_TRUE(mpc->commitCandidate());

  const MatrixXd baseline_x = mpc->getX();
  const VectorXd baseline_u0 = mpc->u0;

  // Cycle 2: pose moved (by the caller, between cycles -- exactly how the
  // controller calls setPose() every cycle) far outside the declared x[0]
  // bound of [100, 200], while the model's constraints stay fixed. x(0) is
  // pinned to the pose by an equality, so this is a hard, structural
  // infeasibility, not a numerically fragile near-miss.
  mpc->setPose((VectorXd(3) << 0.0, 0.0, 0.0).finished());
  mpc->solveCandidate();
  EXPECT_NE(mpc->qp_info.status, proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED);
  EXPECT_FALSE(mpc->commitCandidate());

  // x (public getter) and u0 (protected, direct MPC member) are both
  // observable and both confirmed unchanged; u and w are written in the same
  // unconditional block right after commitCandidate()'s single early return
  // (mpc.cpp), with no per-field guard, so their preservation follows from
  // the same refused commit -- see the file header for why they cannot be
  // read back directly to confirm this independently.
  expectMatrixUnchanged(baseline_x, mpc->getX(), "x");
  EXPECT_LT((baseline_u0 - mpc->u0).cwiseAbs().maxCoeff(), 1e-15) << "u0 changed";
}

// commitCandidate()'s finiteness guard checks the WHOLE horizon (cand_x,
// cand_u and cand_w all-finite), not only the first row: a non-finite value in
// a later row is refused exactly like one in row 0. Constructed by direct
// injection because whether proxsuite can itself return such an iterate
// through solveCandidate() is not established.
TEST(TransactionalSolve, NonFiniteLaterHorizonRowIsRejected)
{
  auto mpc = makeMpc();
  EXPECT_FALSE(mpc->getCandidateFinite());   // nothing solved yet

  mpc->setPose((VectorXd(3) << 150.0, 0.0, 0.0).finished());
  auto [x1, u1] = mpc->solveCandidate();
  ASSERT_TRUE(mpc->commitCandidate());

  const MatrixXd baseline_x = mpc->getX();
  const VectorXd baseline_u0 = mpc->u0;
  // A finite candidate the SAME shape as the real one, reused as the base for
  // each corrupted copy below.
  const MatrixXd good_x = x1;
  const MatrixXd good_u = u1;
  const VectorXd good_w = VectorXd::Zero(6);

  MatrixXd bad_x = good_x;
  ASSERT_GT(bad_x.rows(), 3);
  bad_x(3, 0) = std::numeric_limits<double>::quiet_NaN();   // a LATER row, not row 0
  mpc->injectCandidate(bad_x, good_u, good_w, /*solved=*/true);
  EXPECT_FALSE(mpc->getCandidateFinite());
  EXPECT_FALSE(mpc->commitCandidate());
  expectMatrixUnchanged(baseline_x, mpc->getX(), "x");
  EXPECT_LT((baseline_u0 - mpc->u0).cwiseAbs().maxCoeff(), 1e-15) << "u0 changed";

  // The same guard for a non-finite value in u, and in w.
  MatrixXd bad_u = good_u;
  bad_u(bad_u.rows() - 1, 0) = std::numeric_limits<double>::infinity();
  mpc->injectCandidate(good_x, bad_u, good_w, /*solved=*/true);
  EXPECT_FALSE(mpc->getCandidateFinite());
  EXPECT_FALSE(mpc->commitCandidate());

  VectorXd bad_w = good_w;
  bad_w(bad_w.size() - 1) = std::numeric_limits<double>::quiet_NaN();
  mpc->injectCandidate(good_x, good_u, bad_w, /*solved=*/true);
  EXPECT_FALSE(mpc->getCandidateFinite());
  EXPECT_FALSE(mpc->commitCandidate());

  // A fully finite candidate marked unsolved is refused too -- the guard is
  // an AND of "converged" and "finite", not finiteness alone.
  mpc->injectCandidate(good_x, good_u, good_w, /*solved=*/false);
  EXPECT_TRUE(mpc->getCandidateFinite());
  EXPECT_FALSE(mpc->commitCandidate());
}
