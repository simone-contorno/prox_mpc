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
 * @param new_model model pointer.
 */
void MPC::init(std::shared_ptr<Model> new_model)
{
  /* Model */
  this->model = new_model;

  /* MPC */
  n = new_model->getN();
  m = new_model->getM();

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

  /* Last chance to catch the horizon bound: the setters skip their comparison
   * while the other horizon is still at its unset sentinel, and Np and Nc are
   * public ProbDim members a caller can assign past the setters entirely. */
  if (Nc > Np) {
    throw std::invalid_argument("MPC::init: Nc must be <= Np");
  }

  x = MatrixXd::Zero(Np + 1, n);
  u = MatrixXd::Zero(Nc, m);
  const bool obstacle_active = new_model->getObsFlag() == true && max_obs > 0;
  w = VectorXd::Zero(obstacle_active == true ? Np * max_obs : 0);
  u0 = u.row(0);

  n_eq = 0;
  n_ineq = new_model->getIneq("x").size() + new_model->getIneq("u").size() +
    new_model->getIneq("du").size();

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
  proxqp->setWarmStart(warm_start);
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

  /* Update current predicted state with the current real pose. The control warm
   * start keeps what the shift above produced: row 0 holds the previous plan's
   * second control, which is the one this cycle is about to decide. Writing u0
   * here instead would put the control already executed last cycle in row 0
   * while row 1 still held the plan's third, so the pair straddled two steps of
   * the rate limit and the warm start entered the QP violating its own
   * control-rate chain. u0 reaches the solver separately as the rate anchor. */
  cand_x.row(0) = pose;

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
 * @param new_x matrix.
 */
void MPC::setX(MatrixXd new_x) {this->x = new_x;}

/*!
 * Set the previous control input the next cycle's rate constraint is anchored on.
 * @param new_u0 control actually applied (length m).
 */
void MPC::setU0(VectorXd new_u0) {this->u0 = new_u0;}

/*!
 * Set the intermediate states weight matrix. Pre-init only; init() validates it.
 * @param new_Q matrix.
 */
void MPC::setQ(MatrixXd new_Q)
{
  rejectAfterInit(initialized, "setQ");
  this->Q = new_Q;
}

/*!
 * Set the control input weight matrix. Pre-init only; init() validates it.
 * @param new_R matrix.
 */
void MPC::setR(MatrixXd new_R)
{
  rejectAfterInit(initialized, "setR");
  this->R = new_R;
}

/*!
 * Set the final state weight matrix. Pre-init only; init() validates it.
 * @param new_S matrix.
 */
void MPC::setS(MatrixXd new_S)
{
  rejectAfterInit(initialized, "setS");
  this->S = new_S;
}

/*!
 * Set the slack variables weight matrix. Pre-init only; init() validates it.
 * @param new_W matrix.
 */
void MPC::setW(MatrixXd new_W)
{
  rejectAfterInit(initialized, "setW");
  this->W = new_W;
}

/*!
 * Set the prediction horizon.
 * If T is set, dt is automatically updated.
 * @param new_Np number (> 0).
 */
void MPC::setNp(size_t new_Np)
{
  rejectAfterInit(initialized, "setNp");
  if (new_Np == 0) {throw std::invalid_argument("MPC::setNp: Np must be > 0");}
  // The Nc <= Np bound holds whichever setter runs second, so it is mirrored
  // here: checking it in setNc alone let the caller reach it by ordering.
  // Nc may not be set yet (0 is its unset sentinel, as in setNc below).
  if (Nc > 0 && Nc > new_Np) {throw std::invalid_argument("MPC::setNp: Np must be >= Nc");}
  this->Np = new_Np;
  if (T > 0.0) {this->dt = T / new_Np;}
}

/*!
 * Set the control horizon.
 * @param new_Nc number (> 0).
 */
void MPC::setNc(size_t new_Nc)
{
  rejectAfterInit(initialized, "setNc");
  if (new_Nc == 0) {throw std::invalid_argument("MPC::setNc: Nc must be > 0");}
  // Np may not be set yet (0 is its unset sentinel, matching setdt/setT below);
  // the comparison is skipped until it is known.
  if (Np > 0 && new_Nc > Np) {throw std::invalid_argument("MPC::setNc: Nc must be <= Np");}
  this->Nc = new_Nc;
}

/*!
 * Set the sample time.
 * If Np is set, T is automatically updated.
 * @param new_dt sample time (> 0).
 */
void MPC::setdt(double new_dt)
{
  rejectAfterInit(initialized, "setdt");
  if (!std::isfinite(new_dt) || new_dt <= 0.0) {
    throw std::invalid_argument("MPC::setdt: dt must be > 0");
  }
  this->dt = new_dt;
  if (Np > 0) {this->T = Np * new_dt;}
}

/*!
 * Set the prediction horizon time [s].
 * If Np is set, dt is automatically updated.
 * @param new_T time (> 0).
 */
void MPC::setT(double new_T)
{
  rejectAfterInit(initialized, "setT");
  if (new_T <= 0.0) {throw std::invalid_argument("MPC::setT: T must be > 0");}
  this->T = new_T;
  if (Np > 0) {this->dt = new_T / Np;}
}

/*!
 * Set the current pose.
 * @param new_pose current pose.
 */
void MPC::setPose(VectorXd new_pose) {this->pose = new_pose;}

/*!
 * Set the desired state goals.
 * The assembly reads rows 0..Np and every state column, so an undersized matrix
 * is rejected here rather than indexed out of bounds on the control hot path,
 * where EIGEN_NO_DEBUG leaves the access unchecked. The column count is known
 * only once init() has read n from the model, so it is checked from then on.
 * @param new_goal_x goals ((Np + 1) x n).
 */
void MPC::setGoalX(MatrixXd new_goal_x)
{
  if (Np > 0 && new_goal_x.rows() < static_cast<Eigen::Index>(Np) + 1) {
    throw std::invalid_argument("MPC::setGoalX: goal_x must have at least Np + 1 rows");
  }
  if (n > 0 && new_goal_x.cols() != static_cast<Eigen::Index>(n)) {
    throw std::invalid_argument("MPC::setGoalX: goal_x must have n columns");
  }
  this->goal_x = new_goal_x;
}

/*!
 * Set the desired control goals.
 * The assembly reads rows 0..Nc-1 and every control column; same reasoning as
 * setGoalX.
 * @param new_goal_u goals (Nc x m).
 */
void MPC::setGoalU(MatrixXd new_goal_u)
{
  if (Nc > 0 && new_goal_u.rows() < static_cast<Eigen::Index>(Nc)) {
    throw std::invalid_argument("MPC::setGoalU: goal_u must have at least Nc rows");
  }
  if (m > 0 && new_goal_u.cols() != static_cast<Eigen::Index>(m)) {
    throw std::invalid_argument("MPC::setGoalU: goal_u must have m columns");
  }
  this->goal_u = new_goal_u;
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
 * @param new_guess flag (default: true).
 */
void MPC::setGuess(bool new_guess)
{
  rejectAfterInit(initialized, "setGuess");
  this->guess = new_guess;
}

/*!
 * Enable the cross-cycle QP warm start: the solver workspace is built once and
 * updated in place, so the factorization and the previous primal/dual iterate
 * carry over between solves. Structural, because it decides how the workspace is
 * built, so it is rejected after init() like the other structural setters.
 * @param new_warm_start flag (default: true).
 */
void MPC::setWarmStart(bool new_warm_start)
{
  rejectAfterInit(initialized, "setWarmStart");
  this->warm_start = new_warm_start;
}

/*!
 * Set QP sub-problems type.
 * @param new_qp_type sparse (false) or dense (true) (default: false).
 */
void MPC::setQPtype(bool new_qp_type)
{
  rejectAfterInit(initialized, "setQPtype");
  this->qp_type = new_qp_type;
}

/*!
 * Set the discrete-time CBF rate for the obstacle coupling (forwarded to ProxQP).
 * Must be set before init()/configProxQP().
 * @param new_cbf_gamma rate in (0, 1]; 1.0 reduces to the pointwise constraint.
 */
void MPC::setCbfGamma(double new_cbf_gamma)
{
  // Pre-init only, like every other setter the QP is configured from: the rate
  // reaches the solver through configProxQP(), which init() runs once, so a
  // later call would change this object's own member and nothing else.
  rejectAfterInit(initialized, "setCbfGamma");
  // Validated here as well as in ProxQP so a bad value fails at configuration
  // time rather than on the first init().
  if (!(new_cbf_gamma > 0.0 && new_cbf_gamma <= 1.0)) {
    throw std::invalid_argument("MPC::setCbfGamma: cbf_gamma must be in (0, 1]");
  }
  this->cbf_gamma = new_cbf_gamma;
}

/*!
 * Set the obstacle-slot capacity K per predicted node (0 disables avoidance).
 * Must be set before init()/configProxQP() so the QP is sized once for K.
 * @param new_max_obs capacity K.
 */
void MPC::setMaxObs(size_t new_max_obs)
{
  rejectAfterInit(initialized, "setMaxObs");
  this->max_obs = new_max_obs;
}

/* Get the obstacle-slot capacity K per predicted node. */
size_t MPC::getMaxObs() {return max_obs;}

/* Get the max obstacle soft-keep-out slack over the horizon (0 if avoidance off). */
double MPC::getMaxObstacleSlack() {return w.size() > 0 ? w.maxCoeff() : 0.0;}

/*!
 * Set the obstacle triples for the current cycle.
 * @param new_obs (Np*K) x 3 matrix of [o_x, o_y, d_safe] per (node, slot); empty
 *   slots should hold the far sentinel so their soft constraint is non-binding.
 */
void MPC::setObs(MatrixXd new_obs) {this->obs = new_obs;}

}  // namespace prox_mpc
