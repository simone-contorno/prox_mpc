// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


#include <prox_mpc/mpc.hpp>

#include <chrono>
#include <memory>
#include <stdexcept>

namespace prox_mpc
{

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
 * Run one SQP cycle and return the predicted state and control trajectories.
 * Convergence must be checked by the caller through qp_info.status, which equals
 * PROXQP_SOLVED on success. On non-convergence solve() takes no safety action;
 * the returned first control is the last (non-converged) iterate and must not be
 * applied as is. The caller is responsible for the fallback, for example a
 * deceleration ramp toward zero that respects the robot's limits.
 */
std::tuple<MatrixXd, MatrixXd> MPC::solve()
{
  /* Slide states and control by 1 position. The right-hand side is .eval()'d into
   * a temporary because source and destination overlap: Eigen assumes no aliasing
   * for block/row assignments, so an explicit temporary keeps the shift correct. */
  x.topRows(x.rows() - 1) = x.bottomRows(x.rows() - 1).eval();
  x.row(x.rows() - 1) = x.row(x.rows() - 2);

  u.topRows(u.rows() - 1) = u.bottomRows(u.rows() - 1).eval();
  u.row(u.rows() - 1) = u.row(u.rows() - 2);

  /* Update current predicted state with the current real pose */
  x.row(0) = pose;
  u.row(0) = u0;

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
    auto [x_sol, u_sol, w_sol, info] = proxqp->solve(x, u, u0, w, goal_x, goal_u);

    /* Update */
    x += x_sol;
    u += u_sol;
    w += w_sol;
    qp_info = info;
    qp_iter_ext += qp_info.iter_ext;
    sqp_iter++;

    /* Wall-clock budget (0 disables it): bound the worst-case solve so a slow
     * SQP cannot overrun the control cycle. On timeout the loop exits with
     * status != PROXQP_SOLVED, routing the caller to its fail-safe. */
    if (max_solve_time > 0.0) {
      const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - sqp_start).count();
      timed_out = elapsed >= max_solve_time;
    }
  } while (qp_info.status != proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED &&
    sqp_iter < max_iter_sqp && !timed_out);

  /* On a converged solve the first control becomes the command sent to the robot
   * and the warm-start reference for the next cycle. On non-convergence solve()
   * takes no safety action: it keeps the last good command and reports the
   * failure through qp_info.status, leaving the fallback policy to the caller. */
  if (qp_info.status == proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED) {
    u0 = u.row(0);
  }

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
 * Set the intermediate states weight matrix.
 * @param Q matrix.
 */
void MPC::setQ(MatrixXd Q) {this->Q = Q;}

/*!
 * Set the control input weight matrix.
 * @param R matrix.
 */
void MPC::setR(MatrixXd R) {this->R = R;}

/*!
 * Set the final state weight matrix.
 * @param S matrix.
 */
void MPC::setS(MatrixXd S) {this->S = S;}

/*!
 * Set the slack variables weight matrix.
 * @param W matrix.
 */
void MPC::setW(MatrixXd W) {this->W = W;}

/*!
 * Set the prediction horizon.
 * If T is set, dt is automatically updated.
 * @param Np number (> 0).
 */
void MPC::setNp(size_t Np)
{
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
  if (Nc == 0) {throw std::invalid_argument("MPC::setNc: Nc must be > 0");}
  this->Nc = Nc;
}

/*!
 * Set the sample time.
 * If Np is set, T is automatically updated.
 * @param dt sample time (> 0).
 */
void MPC::setdt(double dt)
{
  if (dt <= 0.0) {throw std::invalid_argument("MPC::setdt: dt must be > 0");}
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
 * @param goal_x goals.
 */
void MPC::setGoalX(MatrixXd goal_x) {this->goal_x = goal_x;}

/*!
 * Set the desired control goals.
 * @param goal_u goals.
 */
void MPC::setGoalU(MatrixXd goal_u) {this->goal_u = goal_u;}

/*!
 * Set the maximum number of internal iterations for the QP solver.
 * @param max_iter max. iterations (default = 1500).
 */
void MPC::setMaxIntIterQP(size_t max_iter) {this->max_int_qp = max_iter;}

/*!
 * Set the maximum number of external iterations for the QP solver.
 * @param max_iter max. iterations (default = 10000).
 */
void MPC::setMaxExtIterQP(size_t max_iter) {this->max_ext_qp = max_iter;}

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
void MPC::setGuess(bool guess) {this->guess = guess;}

/*!
 * Set QP sub-problems type.
 * @param qp_type sparse (false) or dense (true) (default: false).
 */
void MPC::setQPtype(bool qp_type) {this->qp_type = qp_type;}

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
 * @param obs (Np*K) x 3 matrix of [o_x, o_y, d_safe] per (node, slot); empty
 *   slots should hold the far sentinel so their soft constraint is non-binding.
 */
void MPC::setObs(MatrixXd obs) {this->obs = obs;}

}  // namespace prox_mpc
