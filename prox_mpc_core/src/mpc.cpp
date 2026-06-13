// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


#include <prox_mpc/mpc.hpp>

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
  w = VectorXd::Zero(Np + 1);
  u0 = u.row(0);

  n_eq = 0;
  n_ineq = model->getIneq("x").size() + model->getIneq("u").size() + model->getIneq("du").size() +
    model->getIneq("w").size();

    /* ProxQP */
  proxqp = std::make_shared<ProxQP>();
  configProxQP();

    /* Obstacle avoidance */
  obs = MatrixXd::Zero(Np + 1, 2);
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
  proxqp->init(model);
}

/*!
 * Compute the state and the control input evolution for the whole prediction horizon.
 */
std::tuple<MatrixXd, MatrixXd> MPC::solve()
{
    /* Slide states and control by 1 position */
  x.block(0, 0, x.rows() - 1, x.cols()) = x.block(1, 0, x.rows() - 1, x.cols());
  x.row(x.rows() - 1) = x.row(x.rows() - 2);

  u.block(0, 0, u.rows() - 1, u.cols()) = u.block(1, 0, u.rows() - 1, u.cols());
  u.row(u.rows() - 1) = u.row(u.rows() - 2);

    /* Update current predicted state with the current real pose */
  x.row(0) = pose;
  u.row(0) = u0;

    /* Set ProxQP */
  proxqp->setdt(dt);
  proxqp->setObs(obs);
  proxqp->setObsDim(obs_l, obs_w, obs_pose_l, obs_pose_w);

    /* Start SQP */
  sqp_iter = 0;
  qp_iter_ext = 0;
  do{
        /* Solve the QP sub-problem */
    auto [x_sol, u_sol, w_sol, info] = proxqp->solve(x, u, u0, w, goal_x, goal_u);

        /* Update */
    x += x_sol;                             // state
    u += u_sol;                             // control
    w += w_sol;                             // slack variable
    qp_info = info;                         // QP informations
    qp_iter_ext += qp_info.iter_ext;        // QP external total iterations
    sqp_iter++;                             // SQP iterations
  } while (qp_info.status != proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED &&
    sqp_iter < max_iter_sqp);

    /* Problem  NOT solved */
  if (sqp_iter >= max_iter_sqp) {
    std::cout << "[SQP] Fail: maximum number of iterations reached." << std::endl;

        /* TODO: Braking? */
    u.row(0) = VectorXd::Zero(u.cols());
  }

  u0 = u.row(0);   // update last control input sent to the robot

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
 * Set desired goal for obstacle avoidance.
 * @param obs goals matrix.
 */
void MPC::setObs(MatrixXd obs) {this->obs = obs;}

/*!
 * Set obstacle box dimensions.
 * @param obs_l obstacle box length (>= 0).
 * @param obs_w obstacle box width (>= 0).
 * @param obs_pose_l obstacle pose distance by back side (>= 0).
 * @param obs_pose_w obstacle pose distance by right side (>= 0).
 */
void MPC::setObsDim(double obs_l, double obs_w, double obs_pose_l, double obs_pose_w)
{
  if (obs_l < 0.) {throw std::invalid_argument("MPC::setObsDim: obs_l must be >= 0");}
  if (obs_w < 0.) {throw std::invalid_argument("MPC::setObsDim: obs_w must be >= 0");}
  if (obs_pose_l < 0.) {throw std::invalid_argument("MPC::setObsDim: obs_pose_l must be >= 0");}
  if (obs_pose_w < 0.) {throw std::invalid_argument("MPC::setObsDim: obs_pose_w must be >= 0");}

  this->obs_l = obs_l;
  this->obs_w = obs_w;
  this->obs_pose_l = obs_pose_l;
  this->obs_pose_w = obs_pose_w;
}

}  // namespace prox_mpc
