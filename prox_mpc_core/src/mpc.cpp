// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


#include <prox_mpc/mpc.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

namespace prox_mpc
{

namespace
{
/* Relative tolerance the weight-matrix symmetry and eigenvalue tests are run at.
 * Loose enough that a matrix assembled in floating point passes, tight enough
 * that a matrix the caller meant to be asymmetric or indefinite does not. */
constexpr double kWeightTol = 1e-8;

/* Reject a structural setter called after init(). The buffers, the QP object and
 * the solver's own sizing are fixed there, and none of these setters resizes
 * them, so a post-init call would leave the object describing one problem and
 * solving another. */
void rejectAfterInit(bool initialized, const char * setter)
{
  if (initialized == true) {
    throw std::logic_error(
            std::string("MPC::") + setter +
            ": structural setters must be called before init()");
  }
}

/* Throw unless `m` is finite, symmetric and positive semidefinite. proxsuite
 * validates sizes only, and the Hessian it is handed is 2 * m, so an asymmetric
 * or indefinite weight silently makes it solve a different problem than the
 * caller wrote. */
void requireSymmetricPSD(const MatrixXd & m, const char * name)
{
  const std::string prefix = std::string("MPC::init: ") + name;
  if (!m.allFinite()) {
    throw std::invalid_argument(prefix + " must be finite");
  }
  const double scale = std::max(1.0, m.cwiseAbs().maxCoeff());
  if ((m - m.transpose()).cwiseAbs().maxCoeff() > kWeightTol * scale) {
    throw std::invalid_argument(prefix + " must be symmetric");
  }
  const Eigen::SelfAdjointEigenSolver<MatrixXd> solver(m);
  if (solver.info() != Eigen::Success) {
    throw std::invalid_argument(prefix + " eigenvalue decomposition failed");
  }
  if (solver.eigenvalues().minCoeff() < -kWeightTol * scale) {
    throw std::invalid_argument(prefix + " must be positive semidefinite");
  }
}
}  // namespace

/*!
 * Inizialize the Model Predictive Control.
 * @param model model pointer.
 */
void MPC::init(std::shared_ptr<Model> model)
{
  /* Model */
  this->model = model;

  /* MPC */
  n = model->getN();
  m = model->getM();

  /* Default-initialize unset weight matrices and validate their dimensions */
  if (Q.size() == 0) {Q = MatrixXd::Identity(n, n);}
  if (S.size() == 0) {S = MatrixXd::Identity(n, n);}
  if (R.size() == 0) {R = MatrixXd::Identity(m, m);}
  if (W.size() == 0) {W = MatrixXd::Identity(1, 1);}
  if (Q.rows() != static_cast<Eigen::Index>(n) || Q.cols() != static_cast<Eigen::Index>(n)) {
    throw std::invalid_argument("MPC::init: Q must be n x n");
  }
  if (S.rows() != static_cast<Eigen::Index>(n) || S.cols() != static_cast<Eigen::Index>(n)) {
    throw std::invalid_argument("MPC::init: S must be n x n");
  }
  if (R.rows() != static_cast<Eigen::Index>(m) || R.cols() != static_cast<Eigen::Index>(m)) {
    throw std::invalid_argument("MPC::init: R must be m x m");
  }
  if (W.rows() != 1 || W.cols() != 1) {
    throw std::invalid_argument("MPC::init: W must be 1 x 1");
  }
  requireSymmetricPSD(Q, "Q");
  requireSymmetricPSD(S, "S");
  requireSymmetricPSD(R, "R");
  requireSymmetricPSD(W, "W");

  x = MatrixXd::Zero(Np + 1, n);
  u = MatrixXd::Zero(Nc, m);
  const bool obstacle_active = model->getObsFlag() == true && max_obs > 0;
  w = VectorXd::Zero(obstacle_active == true ? Np * max_obs : 0);
  u0 = u.row(0);

  n_eq = 0;
  n_ineq = model->getIneq("x").size() + model->getIneq("u").size() + model->getIneq("du").size();

  /* ProxQP */
  proxqp = std::make_shared<ProxQP>();
  configProxQP();

  /* Obstacle avoidance: capacity K of (o_x, o_y, d_safe) slots per predicted
   * node; unused slots default to the far sentinel so they stay non-binding. */
  obs = MatrixXd::Zero(Np * max_obs, 3);
  for (Eigen::Index r = 0; r < obs.rows(); r++) {
    obs(r, 0) = kObsFarSentinel;
    obs(r, 1) = kObsFarSentinel;
    obs(r, 2) = 0.0;
  }

  /* Every buffer and the QP object are sized from here on; the structural
   * setters reject a later call rather than mutating one of the two halves. */
  initialized = true;
}

/* Configure the ProxQP solver. */
void MPC::configProxQP()
{
  proxqp->setNp(Np);
  proxqp->setNc(Nc);
  proxqp->setNEq(n_eq);
  proxqp->setNIneq(n_ineq);
  proxqp->setdt(dt);
  proxqp->setQ(Q);
  proxqp->setS(S);
  proxqp->setR(R);
  proxqp->setW(W);
  proxqp->setMaxInIter(max_int_qp);
  proxqp->setMaxOutIter(max_ext_qp);
  proxqp->setQPType(qp_type);
  proxqp->setGuess(guess);
  proxqp->setCbfGamma(cbf_gamma);
  proxqp->setMaxObs(max_obs);
  proxqp->init(model);
}

/*!
 * Run one SQP cycle and return the predicted state and control trajectories,
 * without retaining any of it: x, u, w and u0 are untouched, and commitCandidate()
 * retains the result once the caller's own acceptance gates have passed.
 * Convergence must be checked by the caller through qp_info.status, which equals
 * PROXQP_SOLVED on success. On non-convergence this takes no safety action; the
 * returned first control is the last (non-converged) iterate and must not be
 * applied as is. The caller is responsible for the fallback, for example a
 * deceleration ramp toward zero that respects the robot's limits.
 */
std::tuple<MatrixXd, MatrixXd> MPC::solveCandidate()
{
  /* The whole cycle runs on the candidate copies. Nothing below writes x, u, w
   * or u0, so a cycle the caller rejects leaves the retained state exactly as the
   * last accepted cycle left it. */
  cand_x = x;
  cand_u = u;
  cand_w = w;
  cand_solved = false;
  cand_finite = false;

  /* Slide states and control by 1 position. The right-hand side is .eval()'d into
   * a temporary because source and destination overlap: Eigen assumes no aliasing
   * for block/row assignments, so an explicit temporary keeps the shift correct. */
  cand_x.topRows(cand_x.rows() - 1) = cand_x.bottomRows(cand_x.rows() - 1).eval();
  cand_x.row(cand_x.rows() - 1) = cand_x.row(cand_x.rows() - 2);

  /* With Nc == 1, u has a single row: there is no previous row to shift, and
   * u.row(u.rows() - 2) would read out of bounds. */
  if (cand_u.rows() > 1) {
    cand_u.topRows(cand_u.rows() - 1) = cand_u.bottomRows(cand_u.rows() - 1).eval();
    cand_u.row(cand_u.rows() - 1) = cand_u.row(cand_u.rows() - 2);
  }

  /* Update current predicted state with the current real pose */
  cand_x.row(0) = pose;
  cand_u.row(0) = u0;

  /* Set ProxQP */
  proxqp->setdt(dt);
  proxqp->setObs(obs);

  /* Start SQP */
  sqp_iter = 0;
  qp_iter_ext = 0;
  bool timed_out = false;
  const auto sqp_start = std::chrono::steady_clock::now();
  do{
    /* Solve the QP sub-problem */
    auto [x_sol, u_sol, w_sol, info] =
      proxqp->solve(cand_x, cand_u, u0, cand_w, goal_x, goal_u);

    /* Update */
    cand_x += x_sol;
    cand_u += u_sol;
    cand_w += w_sol;
    qp_info = info;
    qp_iter_ext += qp_info.iter_ext;
    sqp_iter++;

    /* Soft wall-clock budget (0 disables it), checked between SQP iterations:
     * it bounds how many further QP sub-problems start, not the one already in
     * flight, because proxsuite exposes no time-based stop (only max_iter and
     * max_iter_in). It is therefore not a bound on worst-case cycle latency in
     * either direction, and the loop still reports success when the QP that
     * overran the budget converged. The bound that does hold per cycle is the
     * iteration caps; max_iter_sqp = 1 gives a genuinely bounded real-time
     * iteration. */
    if (max_solve_time > 0.0) {
      const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - sqp_start).count();
      timed_out = elapsed >= max_solve_time;
    }
  } while (qp_info.status != proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED &&
    sqp_iter < max_iter_sqp && !timed_out);

  cand_solved = qp_info.status == proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED;
  /* Full-horizon finiteness, not only the first control: a non-finite tail would
   * otherwise be committed and then warm-start the next cycle. */
  cand_finite = cand_x.allFinite() && cand_u.allFinite() && cand_w.allFinite();
  cand_u0 = cand_u.row(0);

  return {cand_x, cand_u};
}

/*!
 * Retain the last candidate. The first control becomes the warm-start reference
 * for the next cycle and the anchor of its control-rate constraint, so it is
 * advanced only for a candidate that both converged and is finite.
 */
bool MPC::commitCandidate()
{
  if (cand_solved == false || cand_finite == false) {return false;}
  x = cand_x;
  u = cand_u;
  w = cand_w;
  u0 = cand_u0;
  return true;
}

/* Whether the last candidate is finite over the whole horizon. */
bool MPC::getCandidateFinite() {return cand_finite;}

/*!
 * Run one SQP cycle and commit it. This is the propose-and-commit entry point:
 * the increments are retained whatever the QP reported, and the first control
 * advances only on a converged solve. Callers that must not advance on a cycle
 * their own gates reject use solveCandidate()/commitCandidate() instead.
 */
std::tuple<MatrixXd, MatrixXd> MPC::solve()
{
  solveCandidate();
  x = cand_x;
  u = cand_u;
  w = cand_w;
  if (cand_solved == true) {u0 = cand_u0;}
  return {x, u};
}

/* Get the states matrix. */
MatrixXd MPC::getX() {return x;}

/* Get the intermediate states weight matrix. */
MatrixXd MPC::getQ() {return Q;}

/* Get the control input weight matrix. */
MatrixXd MPC::getR() {return R;}

/* Get the final state weight matrix. */
MatrixXd MPC::getS() {return S;}

/* Get the slack variables weight matrix. */
MatrixXd MPC::getW() {return W;}

/* Get the number of shooting nodes (state). */
size_t MPC::getNp() {return Np;}

/* Get the number of shooting nodes (control). */
size_t MPC::getNc() {return Nc;}

/* Get the step size. */
double MPC::getdt() {return dt;}

/* Get the prediction horizon. */
double MPC::getT() {return T;}

/* Get the current pose. */
VectorXd MPC::getPose() {return pose;}

/* Get the desired state goals. */
MatrixXd MPC::getGoalX() {return goal_x;}

/* Get the desired control goals. */
MatrixXd MPC::getGoalU() {return goal_u;}

/* Get the maximum number of internal iterations for the QP solver. */
size_t MPC::getMaxIntIterQP() {return max_int_qp;}

/* Get the maximum number of external iterations for the QP solver. */
size_t MPC::getMaxExtIterQP() {return max_ext_qp;}

/* Get the maximum number of iterations for the SQP. */
size_t MPC::getMaxIterSQP() {return max_iter_sqp;}

/* Get the guess flag. */
bool MPC::getGuess() {return guess;}

/*!
 * Set the states matrix.
 * @param x matrix.
 */
void MPC::setX(MatrixXd x) {this->x = x;}

/*!
 * Set the previous control input the next cycle's rate constraint is anchored on.
 * @param u0 control actually applied (length m).
 */
void MPC::setU0(VectorXd u0) {this->u0 = u0;}

/*!
 * Set the intermediate states weight matrix. Pre-init only; init() validates it.
 * @param Q matrix.
 */
void MPC::setQ(MatrixXd Q)
{
  rejectAfterInit(initialized, "setQ");
  this->Q = Q;
}

/*!
 * Set the control input weight matrix. Pre-init only; init() validates it.
 * @param R matrix.
 */
void MPC::setR(MatrixXd R)
{
  rejectAfterInit(initialized, "setR");
  this->R = R;
}

/*!
 * Set the final state weight matrix. Pre-init only; init() validates it.
 * @param S matrix.
 */
void MPC::setS(MatrixXd S)
{
  rejectAfterInit(initialized, "setS");
  this->S = S;
}

/*!
 * Set the slack variables weight matrix. Pre-init only; init() validates it.
 * @param W matrix.
 */
void MPC::setW(MatrixXd W)
{
  rejectAfterInit(initialized, "setW");
  this->W = W;
}

/*!
 * Set the prediction horizon.
 * If T is set, dt is automatically updated.
 * @param Np number (> 0).
 */
void MPC::setNp(size_t Np)
{
  rejectAfterInit(initialized, "setNp");
  if (Np == 0) {throw std::invalid_argument("MPC::setNp: Np must be > 0");}
  this->Np = Np;
  if (T > 0.0) {this->dt = T / Np;}
}

/*!
 * Set the control horizon.
 * @param Nc number (> 0).
 */
void MPC::setNc(size_t Nc)
{
  rejectAfterInit(initialized, "setNc");
  if (Nc == 0) {throw std::invalid_argument("MPC::setNc: Nc must be > 0");}
  // Np may not be set yet (0 is its unset sentinel, matching setdt/setT below);
  // the comparison is skipped until it is known.
  if (Np > 0 && Nc > Np) {throw std::invalid_argument("MPC::setNc: Nc must be <= Np");}
  this->Nc = Nc;
}

/*!
 * Set the sample time.
 * If Np is set, T is automatically updated.
 * @param dt sample time (> 0).
 */
void MPC::setdt(double dt)
{
  if (!std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument("MPC::setdt: dt must be > 0");
  }
  this->dt = dt;
  if (Np > 0) {this->T = Np * dt;}
}

/*!
 * Set the prediction horizon time [s].
 * If Np is set, dt is automatically updated.
 * @param T time (> 0).
 */
void MPC::setT(double T)
{
  if (T <= 0.0) {throw std::invalid_argument("MPC::setT: T must be > 0");}
  this->T = T;
  if (Np > 0) {this->dt = T / Np;}
}

/*!
 * Set the current pose.
 * @param pose current pose.
 */
void MPC::setPose(VectorXd pose) {this->pose = pose;}

/*!
 * Set the desired state goals.
 * The assembly reads rows 0..Np and every state column, so an undersized matrix
 * is rejected here rather than indexed out of bounds on the control hot path,
 * where EIGEN_NO_DEBUG leaves the access unchecked. The column count is known
 * only once init() has read n from the model, so it is checked from then on.
 * @param goal_x goals ((Np + 1) x n).
 */
void MPC::setGoalX(MatrixXd goal_x)
{
  if (Np > 0 && goal_x.rows() < static_cast<Eigen::Index>(Np) + 1) {
    throw std::invalid_argument("MPC::setGoalX: goal_x must have at least Np + 1 rows");
  }
  if (n > 0 && goal_x.cols() != static_cast<Eigen::Index>(n)) {
    throw std::invalid_argument("MPC::setGoalX: goal_x must have n columns");
  }
  this->goal_x = goal_x;
}

/*!
 * Set the desired control goals.
 * The assembly reads rows 0..Nc-1 and every control column; same reasoning as
 * setGoalX.
 * @param goal_u goals (Nc x m).
 */
void MPC::setGoalU(MatrixXd goal_u)
{
  if (Nc > 0 && goal_u.rows() < static_cast<Eigen::Index>(Nc)) {
    throw std::invalid_argument("MPC::setGoalU: goal_u must have at least Nc rows");
  }
  if (m > 0 && goal_u.cols() != static_cast<Eigen::Index>(m)) {
    throw std::invalid_argument("MPC::setGoalU: goal_u must have m columns");
  }
  this->goal_u = goal_u;
}

/*!
 * Set the maximum number of internal iterations for the QP solver.
 * @param max_iter max. iterations (default = 1500).
 */
void MPC::setMaxIntIterQP(size_t max_iter)
{
  rejectAfterInit(initialized, "setMaxIntIterQP");
  this->max_int_qp = max_iter;
}

/*!
 * Set the maximum number of external iterations for the QP solver.
 * @param max_iter max. iterations (default = 10000).
 */
void MPC::setMaxExtIterQP(size_t max_iter)
{
  rejectAfterInit(initialized, "setMaxExtIterQP");
  this->max_ext_qp = max_iter;
}

/*!
 * Set the maximum number of iterations for the SQP solver.
 * @param max_iter max. iterations (default = 100).
 */
void MPC::setMaxIterSQP(size_t max_iter) {this->max_iter_sqp = max_iter;}

/*!
 * Set the wall-clock budget for the whole SQP loop.
 * @param seconds budget [s]; <= 0 disables it (iteration caps then bound the loop).
 */
void MPC::setMaxSolveTime(double seconds) {this->max_solve_time = seconds;}

/*!
 * Set if use initial guesses or not.
 * @param guess flag (default: true).
 */
void MPC::setGuess(bool guess)
{
  rejectAfterInit(initialized, "setGuess");
  this->guess = guess;
}

/*!
 * Set QP sub-problems type.
 * @param qp_type sparse (false) or dense (true) (default: false).
 */
void MPC::setQPtype(bool qp_type)
{
  rejectAfterInit(initialized, "setQPtype");
  this->qp_type = qp_type;
}

/*!
 * Set the discrete-time CBF rate for the obstacle coupling (forwarded to ProxQP).
 * Must be set before init()/configProxQP().
 * @param cbf_gamma rate in (0, 1]; 1.0 reduces to the pointwise constraint.
 */
void MPC::setCbfGamma(double cbf_gamma)
{
  // Validated here as well as in ProxQP so a bad value fails at configuration
  // time rather than on the first init().
  if (!(cbf_gamma > 0.0 && cbf_gamma <= 1.0)) {
    throw std::invalid_argument("MPC::setCbfGamma: cbf_gamma must be in (0, 1]");
  }
  this->cbf_gamma = cbf_gamma;
}

/*!
 * Set the obstacle-slot capacity K per predicted node (0 disables avoidance).
 * Must be set before init()/configProxQP() so the QP is sized once for K.
 * @param max_obs capacity K.
 */
void MPC::setMaxObs(size_t max_obs) {this->max_obs = max_obs;}

/* Get the obstacle-slot capacity K per predicted node. */
size_t MPC::getMaxObs() {return max_obs;}

/* Get the max obstacle soft-keep-out slack over the horizon (0 if avoidance off). */
double MPC::getMaxObstacleSlack() {return w.size() > 0 ? w.maxCoeff() : 0.0;}

/*!
 * Set the obstacle triples for the current cycle.
 *
 * Two block layouts are accepted, and nothing between them. In the (Np*K) x 3
 * form, block j holds the obstacle at predicted state j+1 and the position at
 * the current time is reconstructed from the first two blocks. In the
 * ((Np+1)*K) x 3 form, block j holds the obstacle at predicted state j, so the
 * leading block is the obstacle now and nothing is inferred; a caller that
 * knows where the obstacle is should supply it. Both are validated here rather
 * than indexed out of bounds during assembly, where EIGEN_NO_DEBUG leaves the
 * access unchecked.
 *
 * @param obs matrix of [o_x, o_y, d_safe] per (block, slot); empty slots should
 *   hold the far sentinel so their soft constraint is non-binding.
 */
void MPC::setObs(MatrixXd obs)
{
  if (max_obs > 0 && Np > 0) {
    const Eigen::Index rows_horizon = static_cast<Eigen::Index>(Np * max_obs);
    const Eigen::Index rows_with_now = static_cast<Eigen::Index>((Np + 1) * max_obs);
    if (obs.cols() != 3 || (obs.rows() != rows_horizon && obs.rows() != rows_with_now)) {
      throw std::invalid_argument(
              "MPC::setObs: obs must be (Np*max_obs) x 3 or ((Np+1)*max_obs) x 3");
    }
  }
  this->obs = obs;
}

}  // namespace prox_mpc
