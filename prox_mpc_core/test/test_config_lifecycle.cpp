// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Contract tests for the post-init() configuration lifecycle (decision: reject
// after init, not a validated rebuild), the weight-matrix validation init()
// gained alongside it, the setGoalX()/setGoalU() undersized-matrix rejection,
// and MPC::setObs()'s single accepted shape.

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include <prox_mpc/mpc.hpp>
#include <prox_mpc/models/unicycle.hpp>

using prox_mpc::MPC;
using prox_mpc::Unicycle;

namespace
{
// A minimal initialized MPC (Unicycle, Np = Nc = 5), for tests that only need
// a post-init() object to call a structural setter against.
std::shared_ptr<MPC> makeInitialized()
{
  auto model = std::make_shared<Unicycle>();
  auto mpc = std::make_shared<MPC>();
  mpc->setNp(5);
  mpc->setNc(5);
  mpc->setdt(0.1);
  mpc->init(model);
  return mpc;
}
}  // namespace

// --- Structural setters reject a post-init() call ----------------------------
//
// Every setter whose value the sized buffers or the QP object are built from:
// setQ, setR, setS, setW, setNp, setNc, setdt, setT, setMaxIntIterQP,
// setMaxExtIterQP, setGuess, setQPtype, setCbfGamma and setMaxObs. Each throws
// std::logic_error rather than mutating a member the sized buffers and the QP
// object no longer agree with (decision: reject after init, not the pruned
// validated-rebuild alternative).

TEST(ConfigLifecycle, SetQRejectsAfterInit)
{
  auto mpc = makeInitialized();
  EXPECT_THROW(mpc->setQ(MatrixXd::Identity(3, 3)), std::logic_error);
}

TEST(ConfigLifecycle, SetRRejectsAfterInit)
{
  auto mpc = makeInitialized();
  EXPECT_THROW(mpc->setR(MatrixXd::Identity(2, 2)), std::logic_error);
}

TEST(ConfigLifecycle, SetSRejectsAfterInit)
{
  auto mpc = makeInitialized();
  EXPECT_THROW(mpc->setS(MatrixXd::Identity(3, 3)), std::logic_error);
}

TEST(ConfigLifecycle, SetWRejectsAfterInit)
{
  auto mpc = makeInitialized();
  EXPECT_THROW(mpc->setW(MatrixXd::Constant(1, 1, 1.0)), std::logic_error);
}

TEST(ConfigLifecycle, SetNpRejectsAfterInit)
{
  auto mpc = makeInitialized();
  EXPECT_THROW(mpc->setNp(6), std::logic_error);
}

TEST(ConfigLifecycle, SetNcRejectsAfterInit)
{
  auto mpc = makeInitialized();
  EXPECT_THROW(mpc->setNc(4), std::logic_error);
}

TEST(ConfigLifecycle, SetdtRejectsAfterInit)
{
  auto mpc = makeInitialized();
  EXPECT_THROW(mpc->setdt(0.2), std::logic_error);
}

TEST(ConfigLifecycle, SetTRejectsAfterInit)
{
  auto mpc = makeInitialized();
  EXPECT_THROW(mpc->setT(1.0), std::logic_error);
}

TEST(ConfigLifecycle, SetMaxIntIterQPRejectsAfterInit)
{
  auto mpc = makeInitialized();
  EXPECT_THROW(mpc->setMaxIntIterQP(100), std::logic_error);
}

TEST(ConfigLifecycle, SetMaxExtIterQPRejectsAfterInit)
{
  auto mpc = makeInitialized();
  EXPECT_THROW(mpc->setMaxExtIterQP(100), std::logic_error);
}

TEST(ConfigLifecycle, SetGuessRejectsAfterInit)
{
  auto mpc = makeInitialized();
  EXPECT_THROW(mpc->setGuess(false), std::logic_error);
}

TEST(ConfigLifecycle, SetQPtypeRejectsAfterInit)
{
  auto mpc = makeInitialized();
  EXPECT_THROW(mpc->setQPtype(true), std::logic_error);
}

// The CBF rate reaches the solver only through configProxQP(), which init()
// runs once, so a post-init() call used to be accepted and then have no effect
// on the QP it appears to configure.
TEST(ConfigLifecycle, SetCbfGammaRejectsAfterInit)
{
  auto mpc = makeInitialized();
  EXPECT_THROW(mpc->setCbfGamma(0.5), std::logic_error);
}

// The obstacle-slot capacity is as structural as the horizon: init() sizes the
// obstacle matrix and the QP's own slot count from it.
TEST(ConfigLifecycle, SetMaxObsRejectsAfterInit)
{
  auto mpc = makeInitialized();
  EXPECT_THROW(mpc->setMaxObs(4), std::logic_error);
}

// The exception names both the class and the setter, so a caller reading the
// log can tell which precondition it violated without a debugger.
TEST(ConfigLifecycle, RejectionMessageNamesSetter)
{
  auto mpc = makeInitialized();
  try {
    mpc->setNp(6);
    FAIL() << "setNp after init() must throw";
  } catch (const std::logic_error & ex) {
    const std::string what(ex.what());
    EXPECT_NE(what.find("setNp"), std::string::npos) << what;
  }
}

// --- Weight-matrix validation in init() --------------------------------------
//
// init() throws std::invalid_argument unless Q, S, R and W are finite,
// symmetric and positive semidefinite; 1.0.0 accepted an asymmetric or
// indefinite weight and silently solved the symmetrized problem instead.

TEST(ConfigLifecycle, InitRejectsAsymmetricQ)
{
  auto model = std::make_shared<Unicycle>();
  auto mpc = std::make_shared<MPC>();
  mpc->setNp(5);
  mpc->setNc(5);
  mpc->setdt(0.1);
  MatrixXd q = MatrixXd::Identity(3, 3);
  q(0, 1) = 1.0;   // q(1, 0) stays 0: asymmetric
  mpc->setQ(q);
  EXPECT_THROW(mpc->init(model), std::invalid_argument);
}

TEST(ConfigLifecycle, InitRejectsIndefiniteS)
{
  auto model = std::make_shared<Unicycle>();
  auto mpc = std::make_shared<MPC>();
  mpc->setNp(5);
  mpc->setNc(5);
  mpc->setdt(0.1);
  MatrixXd s = MatrixXd::Identity(3, 3);
  s(2, 2) = -1.0;   // symmetric but has a negative eigenvalue
  mpc->setS(s);
  EXPECT_THROW(mpc->init(model), std::invalid_argument);
}

TEST(ConfigLifecycle, InitRejectsNonFiniteR)
{
  auto model = std::make_shared<Unicycle>();
  auto mpc = std::make_shared<MPC>();
  mpc->setNp(5);
  mpc->setNc(5);
  mpc->setdt(0.1);
  MatrixXd r = MatrixXd::Identity(2, 2);
  r(0, 0) = std::numeric_limits<double>::quiet_NaN();
  mpc->setR(r);
  EXPECT_THROW(mpc->init(model), std::invalid_argument);
}

TEST(ConfigLifecycle, InitRejectsIndefiniteW)
{
  auto model = std::make_shared<Unicycle>();
  auto mpc = std::make_shared<MPC>();
  mpc->setNp(5);
  mpc->setNc(5);
  mpc->setdt(0.1);
  mpc->setW(MatrixXd::Constant(1, 1, -1.0));   // negative -> indefinite
  EXPECT_THROW(mpc->init(model), std::invalid_argument);
}

// A finite, symmetric, positive-semidefinite set of weights (including the
// diagonal-with-zero edge case, still PSD) still initializes.
TEST(ConfigLifecycle, InitAcceptsValidWeights)
{
  auto model = std::make_shared<Unicycle>();
  auto mpc = std::make_shared<MPC>();
  mpc->setNp(5);
  mpc->setNc(5);
  mpc->setdt(0.1);
  mpc->setQ(MatrixXd::Identity(3, 3));
  mpc->setS(2.0 * MatrixXd::Identity(3, 3));
  mpc->setR(0.1 * MatrixXd::Identity(2, 2));
  mpc->setW(MatrixXd::Constant(1, 1, 0.0));
  EXPECT_NO_THROW(mpc->init(model));
}

// --- setGoalX / setGoalU undersized-matrix rejection -------------------------
//
// setc() indexes goal_x.row(Np) and goal_u.row(Nc - 1) (proxqp.cpp); an
// undersized matrix from a direct core consumer used to be a silent
// out-of-bounds read there with EIGEN_NO_DEBUG on. Both setters now validate
// row and column counts once Np/Nc/n/m are known.

TEST(ConfigLifecycle, SetGoalXRejectsTooFewRows)
{
  auto mpc = makeInitialized();   // Np = 5, n = 3
  EXPECT_THROW(mpc->setGoalX(MatrixXd::Zero(5, 3)), std::invalid_argument);   // needs Np+1 = 6
  EXPECT_NO_THROW(mpc->setGoalX(MatrixXd::Zero(6, 3)));
}

TEST(ConfigLifecycle, SetGoalXRejectsWrongColumnCount)
{
  auto mpc = makeInitialized();   // n = 3
  EXPECT_THROW(mpc->setGoalX(MatrixXd::Zero(6, 2)), std::invalid_argument);
  EXPECT_THROW(mpc->setGoalX(MatrixXd::Zero(6, 4)), std::invalid_argument);
}

TEST(ConfigLifecycle, SetGoalURejectsTooFewRows)
{
  auto mpc = makeInitialized();   // Nc = 5, m = 2
  EXPECT_THROW(mpc->setGoalU(MatrixXd::Zero(4, 2)), std::invalid_argument);   // needs Nc = 5
  EXPECT_NO_THROW(mpc->setGoalU(MatrixXd::Zero(5, 2)));
}

TEST(ConfigLifecycle, SetGoalURejectsWrongColumnCount)
{
  auto mpc = makeInitialized();   // m = 2
  EXPECT_THROW(mpc->setGoalU(MatrixXd::Zero(5, 1)), std::invalid_argument);
  EXPECT_THROW(mpc->setGoalU(MatrixXd::Zero(5, 3)), std::invalid_argument);
}

// The precondition is checked before Np/Nc/n/m are known too (pre-init call):
// with none of them known yet, an arbitrarily-shaped matrix is accepted, and
// is then validated retroactively as soon as init() (or setNp()/setNc())
// establishes the missing dimension.
TEST(ConfigLifecycle, SetGoalXAcceptsAnyShapeBeforeHorizonKnown)
{
  MPC mpc;
  EXPECT_NO_THROW(mpc.setGoalX(MatrixXd::Zero(1, 1)));
}

// --- MPC::setObs: exactly Np*K rows, no other shape -------------------------
//
// A second accepted shape, (Np + 1) * K, was briefly implemented and
// deliberately retired; MPC::setObs() itself performs no validation (it is a
// raw assignment), but the mismatch is caught the first time the stored
// matrix reaches ProxQP::setObs() inside solveCandidate().

TEST(ConfigLifecycle, SetObsRejectsNpPlusOneRowsAtSolveTime)
{
  auto model = std::make_shared<Unicycle>();
  auto mpc = std::make_shared<MPC>();
  mpc->setNp(5);
  mpc->setNc(5);
  mpc->setdt(0.1);
  mpc->setMaxObs(1);
  mpc->init(model);   // Np = 5, K = 1 -> the solver expects exactly 5 rows

  mpc->setGoalX(MatrixXd::Zero(6, 3));
  mpc->setGoalU(MatrixXd::Zero(5, 2));
  mpc->setPose(VectorXd::Zero(3));

  // (Np + 1) * K = 6 rows: the retired second shape, must not be accepted.
  mpc->setObs(MatrixXd::Zero(6, 3));
  EXPECT_THROW(mpc->solveCandidate(), std::invalid_argument);

  // The single accepted shape, Np * K = 5 rows, still solves.
  MatrixXd obs_ok = MatrixXd::Zero(5, 3);
  for (Eigen::Index r = 0; r < obs_ok.rows(); r++) {
    obs_ok(r, 0) = MPC::kObsFarSentinel;
    obs_ok(r, 1) = MPC::kObsFarSentinel;
  }
  mpc->setObs(obs_ok);
  EXPECT_NO_THROW(mpc->solveCandidate());
}
