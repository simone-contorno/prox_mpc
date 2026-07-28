// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


#include <prox_mpc/proxqp.hpp>


#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

// Solver-local convenience for the proxsuite types (kept out of the header so it
// does not leak into downstream consumers).
using namespace proxsuite::proxqp;

namespace prox_mpc
{

namespace
{
// Guard for the obstacle-constraint normal: avoids 0/0 = NaN when the predicted
// robot position coincides with the obstacle.
constexpr double kObsNormalEps = 1e-9;
}  // namespace

/*!
 * Initialize ProxQP solver.
 * @param model model pointer.
 */
void ProxQP::init(std::shared_ptr<Model> model)
{
  /* Robot model. */
  this->model = model;

  /* Problem dimensions */
  this->n = model->getN();
  this->m = model->getM();

  /* Constraints */

  // Equalities
  n_eq += 2;                            // update the number of equalities
  eq_idx.assign(n_eq, 0);
  eq_idx[0] = n;                        // first state = initial pose
  eq_idx[1] = eq_idx[0] + Np * n;       // kinematics
  eq_tot = eq_idx.back();

  // Inequalities

  /* Obstacle avoidance is active only if the model declares it and a capacity
   * K > 0 was set; K = 0 (or the model flag off) reduces to the obstacle-off QP.
   * Cached so the per-iteration assembly (setH/setc/setC/setd) reuses it. */
  obstacle_active = model->getObsFlag() == true && max_obs > 0;

  /* Update the number of inequality blocks: the first control input, plus the
   * obstacle constraint block and the slack lower-bound block when active. */
  n_ineq += 1;                                  // first control input
  if (obstacle_active == true) {n_ineq += 2;}   // obstacle rows + slack lower-bound rows

  /* Build the inequality indices vector */
  ineq_idx.assign(n_ineq, 0);

  /* Set inequality constraint indices */
  size_t i = 0;

  ineq_idx[i] = model->getIneq("du").size();                   // first control-rate rows
  i++;

  for (size_t j = 0; j < model->getIneq("x").size(); j++) {     // state
    ineq_idx[i] = ineq_idx[i - 1] + (Np + 1);
    i++;
  }

  for (size_t j = 0; j < model->getIneq("u").size(); j++) {     // control input
    ineq_idx[i] = ineq_idx[i - 1] + Nc;
    i++;
  }

  for (size_t j = 0; j < model->getIneq("du").size(); j++) {    // control input derivative
    ineq_idx[i] = ineq_idx[i - 1] + (Nc - 1);
    i++;
  }

  if (obstacle_active == true) {
    ineq_idx[i] = ineq_idx[i - 1] + Np * max_obs;   // obstacle constraint rows (Np*K)
    i++;
    ineq_idx[i] = ineq_idx[i - 1] + Np * max_obs;   // slack lower-bound rows (Np*K, s >= 0)
    i++;
  }

  ineq_tot = ineq_idx.back();

  /* Decision variables */
  x_start = 0;
  u_start = x_start + (Np + 1) * n;
  w_start = u_start + Nc * m;
  const size_t n_slack = obstacle_active == true ? Np * max_obs : 0;
  n_dvars = w_start + n_slack;

  /* Per-slot signed-distance caches (populated each iteration in setC): obs_h at
   * the constrained node (k+1), obs_h_prev at the previous node (k) for the CBF
   * coupling. */
  obs_h = VectorXd::Zero(n_slack);
  obs_h_prev = VectorXd::Zero(n_slack);

  /* ProxQP */
  H = MatrixXd::Zero(n_dvars, n_dvars);
  E = MatrixXd::Zero(eq_tot, n_dvars);
  C = MatrixXd::Zero(ineq_tot, n_dvars);
  c = VectorXd::Zero(n_dvars);
  b = VectorXd::Zero(eq_tot);
  upp = VectorXd::Zero(ineq_tot);
  low = VectorXd::Zero(ineq_tot);

  /* Optimal results */
  result_x = VectorXd::Zero(H.cols());

  /* Settings */
  setH();   // Hessian matrix

  qp_dim = n_dvars;
  qp_eq = eq_tot;
  qp_ineq = ineq_tot;

  /* Sparse problem */
  if (qp_type == false) {
    // Sparse matrices
    H_sparse = H.sparseView();
    E_sparse = E.sparseView();
    C_sparse = C.sparseView();

    // QP settings
    qp_sparse = sparse::QP<double, isize>(qp_dim, qp_eq, qp_ineq);
    qp_sparse.settings.max_iter = max_out_iter;
    qp_sparse.settings.max_iter_in = max_inn_iter;
    qp_sparse.settings.compute_timings = true;
  }
  // Dense problem
  else {
    // QP settings
    qp_dense = dense::QP<double>(qp_dim, qp_eq, qp_ineq);
    qp_dense.settings.max_iter = max_out_iter;
    qp_dense.settings.max_iter_in = max_inn_iter;
    qp_dense.settings.compute_timings = true;
  }
}

/*!
 * @brief Initial-guess policy for the QP sub-problem.
 *
 * The sub-problem is rebuilt with fresh matrices each solve, so only ProxQP's own
 * cheap starts apply: the equality-constrained guess (the default) or no guess.
 * An external initial guess handed to a freshly initialized workspace mixes a
 * stale guess with a reset state, which proxsuite 0.6.5 answers by diverging.
 */
proxsuite::proxqp::InitialGuessStatus ProxQP::initialGuessPolicy() const
{
  return guess ?
         proxsuite::proxqp::InitialGuessStatus::EQUALITY_CONSTRAINED_INITIAL_GUESS :
         proxsuite::proxqp::InitialGuessStatus::NO_INITIAL_GUESS;
}

/*!
 * Solve the quadratic program using ProxQP solver and compute the optimal state and control input evolution,
 * resulting in an optimal trajectory.
 * @param x states matrix.
 * @param u control inputs matrix.
 * @param u_prev last control input sent to the vehicle.
 * @param w slack variables vector for obstacle avoidance.
 * @param goal_x state's goals vector.
 * @param goal_u control's goals vector.
*/
std::tuple<MatrixXd, MatrixXd, VectorXd, proxsuite::proxqp::Info<double>>
ProxQP::solve(
  MatrixXd x, MatrixXd u, const VectorXd & u_prev, VectorXd w, const MatrixXd & goal_x,
  const MatrixXd & goal_u)
{
  /* Configure QP problem */
  setc(x, u, w, goal_x, goal_u);    // c
  setE(x, u);                       // E
  setb(x, u);                       // b
  setC(x);                          // C
  setd(x, u, u_prev, w);            // d

  /* Sparse problem */
  if (qp_type == false) {
    // Make constraints matrices sparse
    E_sparse = E.sparseView();
    C_sparse = C.sparseView();

    qp_sparse.settings.initial_guess = initialGuessPolicy();
    qp_sparse.init(H_sparse, c, E_sparse, b, C_sparse, low, upp);

    qp_sparse.solve();

    // Take results
    result_x = qp_sparse.results.x;

    // Take QP info
    qp_info = qp_sparse.results.info;
  }
  /* Dense problem */
  else {
    qp_dense.settings.initial_guess = initialGuessPolicy();
    qp_dense.init(H, c, E, b, C, low, upp);

    qp_dense.solve();

    // Take results
    result_x = qp_dense.results.x;

    // Take QP info
    qp_info = qp_dense.results.info;
  }

  /* Update decision variables */
  for (size_t i = x_start; i < u_start; i += n) {
    x.row(i / n) = result_x.segment(i, n);
  }
  for (size_t i = u_start; i < w_start; i += m) {
    u.row((i - u_start) / m) = result_x.segment(i, m);
  }
  for (size_t i = w_start; i < n_dvars; i++) {
    w(i - w_start) = result_x(i);
  }

  return {x, u, w, qp_info};
}

/* Fill the objective function Hessian matrix H. */
void ProxQP::setH()
{
  // Intermediate states
  for (size_t i = x_start; i < u_start - n; i += n) {
    H.block(i, i, n, n) = 2 * Q;
  }

  // Final state
  H.block(u_start - n, u_start - n, n, n) = 2 * S;

  // Control inputs. All Nc blocks carry R; unlike the state path there is no
  // separate terminal-control weight to exclude the last one for.
  for (size_t i = u_start; i < w_start; i += m) {
    H.block(i, i, m, m) = 2 * R;
  }

  // Slack variable
  if (obstacle_active == true) {
    for (size_t i = w_start; i < n_dvars; i++) {
      H.block(i, i, 1, 1) = 2 * W;
    }
  }
}

/*!
 * Fill the objective function coefficients vector c.
 * @param x states matrix.
 * @param u controls matrix.
 * @param w slack variables vector.
 * @param goal_x state's goals matrix.
 * @param goal_u control's goals matrix.
 */
void ProxQP::setc(
  const MatrixXd & x, const MatrixXd & u, const VectorXd & w,
  const MatrixXd & goal_x, const MatrixXd & goal_u)
{
  size_t count = 0;

  // Intermediate states
  for (size_t i = x_start; i < u_start - n; i += n) {
    c.segment(i, n) = 2 * Q * (x.row(count) - goal_x.row(count)).transpose();
    count++;
  }

  // Final state
  c.segment(u_start - n, n) = 2 * S * (x.row(count) - goal_x.row(count)).transpose();

  // Control inputs. Matches setH: all Nc blocks, so goal_u's last row is read.
  count = 0;
  for (size_t i = u_start; i < w_start; i += m) {
    c.segment(i, m) = 2 * R * (u.row(count).transpose() - goal_u.row(count).transpose());
    count++;
  }

  // Slack variable
  if (obstacle_active == true) {
    count = 0;
    for (size_t i = w_start; i < n_dvars; i++) {
      c.segment(i, 1) = 2 * W * w(count);
      count++;
    }
  }
}

/*!
 * Fill the equality constraints coefficient matrix E.
 * @param x states matrix.
 * @param u controls matrix.
 */
void ProxQP::setE(const MatrixXd & x, const MatrixXd & u)
{
  /* Initial state */
  for (size_t i = x_start; i < eq_idx[0]; i += n) {
    E.block(i, i, n, n) = MatrixXd::Identity(n, n);
  }

  /* Model kinematics (Euler) */
  size_t count = 0;
  size_t col;
  for (size_t i = eq_idx[0]; i < eq_idx[1]; i += n) {
    // Update for the current step
    model->setX(x.row(count));
    model->setU(u.row(std::clamp(static_cast<int>(count), 0, static_cast<int>(u.rows() - 1))));

    model->updateA(dt);
    model->updateB();

    // State
    E.block(i, i - eq_idx[0], n, n) = model->getA();
    E.block(i, i - eq_idx[0] + n, n, n) = -MatrixXd::Identity(n, n);

    // Control input. Move-blocking convention: when Np > Nc the prediction nodes
    // beyond the control horizon reuse the last control block (col clamped to the
    // final control block at w_start - m), i.e. the control is held constant after
    // node Nc. With Np == Nc (the bundled config) the clamp is inert.
    col = u_start + (i - eq_idx[0]) / n * m;
    E.block(i, std::clamp(static_cast<int>(col), 0, static_cast<int>(w_start - m)), n,
        m) = model->getB() * dt;

    count++;
  }
}

/*!
 * Fill the equality constraints vector b.
 * @param x states matrix.
 * @param u controls matrix.
 */
void ProxQP::setb(const MatrixXd & x, const MatrixXd & u)
{
  size_t count = 0;

  /* Model kinematics */
  for (size_t i = eq_idx[0]; i < eq_idx[1]; i += n) {
    // Update for the current step
    model->setX(x.row(count));
    model->setU(u.row(std::clamp(static_cast<int>(count), 0, static_cast<int>(u.rows() - 1))));
    model->updatec(dt, x.row(count + 1));

    b.segment(i, n) = -model->getc();

    count++;
  }
}

/*!
 * Fill the inequality constraints coefficients matrix C.
 * @param x states matrix.
 */
void ProxQP::setC(const MatrixXd & x)
{
  size_t i = 0;
  size_t idx;
  size_t col;

  /* First control input's constraint. Row j carries the bound declared by entry j
   * of the "du" map, so it must select that entry's control component, which is
   * not the map key when a model declares its rate bounds out of index order. */
  for (size_t j = 0; j < model->getIneq("du").size(); j++) {
    idx = u_start + static_cast<size_t>(model->getIneq("du").at(j)[0]);
    C(j, idx) = 1;
  }

  /* States' constraints */
  for (size_t j = 0; j < model->getIneq("x").size(); j++) {
    for (size_t l = ineq_idx[i]; l < ineq_idx[i + 1]; l++) {
      idx = x_start + model->getIneq("x").at(j)[0] + n * (l - ineq_idx[i]);
      C(l, idx) = 1;
    }
    i++;
  }

  /* Control inputs' constraints */
  for (size_t j = 0; j < model->getIneq("u").size(); j++) {
    for (size_t l = ineq_idx[i]; l < ineq_idx[i + 1]; l++) {
      idx = u_start + model->getIneq("u").at(j)[0] + m * (l - ineq_idx[i]);
      C(l, idx) = 1;
    }
    i++;
  }

  /* Control inputs derivatives' constraints */
  for (size_t j = 0; j < model->getIneq("du").size(); j++) {
    for (size_t l = ineq_idx[i]; l < ineq_idx[i + 1]; l++) {
      idx = u_start + model->getIneq("du").at(j)[0] + m * (l - ineq_idx[i]);
      C(l, idx) = -1;
      C(l, idx + m) = 1;
    }
    i++;
  }

  /* Obstacle avoidance (linearized signed-distance half-plane, K slots/node) */
  if (obstacle_active == true) {
    /* Obstacle constraint rows: n^T (p_k - o) + s >= d_safe, recomputed each
     * SQP iteration. Row and slack share the same (node, slot) index. */
    for (size_t r = ineq_idx[i]; r < ineq_idx[i + 1]; r++) {
      const size_t slot = r - ineq_idx[i];          // 0 .. Np*K-1
      const size_t node = slot / max_obs;           // predicted-node index 0 .. Np-1
      col = x_start + n * (node + 1);               // position block of predicted node (node+1)

      const double dx = x(node + 1, 0) - obs(slot, 0);
      const double dy = x(node + 1, 1) - obs(slot, 1);
      double norm = sqrt(dx * dx + dy * dy);
      if (norm < kObsNormalEps) {norm = kObsNormalEps;}
      const double nx = dx / norm;
      const double ny = dy / norm;

      C(r, col) = nx;                               // x
      C(r, col + 1) = ny;                           // y
      C(r, w_start + slot) = 1;                     // slack

      obs_h(slot) = norm - obs(slot, 2);            // signed distance at node k+1

      /* Signed distance at the previous node k for the CBF coupling, against the
       * SAME obstacle slot at the previous node's time. A predictive (moving) fill
       * carries a different obstacle position per node, so node k uses obs[slot -
       * max_obs]; for node 0 (x(0) is the fixed pose) and for a static fill this
       * reduces to obs[slot]. Held constant per SQP iteration (its gradient is not
       * added to C); the re-linearization across iterations recovers the value. */
      const size_t prev_slot = (node >= 1) ? (slot - max_obs) : slot;
      const double dxp = x(node, 0) - obs(prev_slot, 0);
      const double dyp = x(node, 1) - obs(prev_slot, 1);
      obs_h_prev(slot) = sqrt(dxp * dxp + dyp * dyp) - obs(prev_slot, 2);
    }
    i++;

    /* Slack lower-bound rows (s >= 0), same (node, slot) index. */
    for (size_t r = ineq_idx[i]; r < ineq_idx[i + 1]; r++) {
      const size_t slot = r - ineq_idx[i];
      C(r, w_start + slot) = 1;
    }
    i++;
  }
}

/*!
 * Fill the inequality constraints vector d.
 * @param x states matrix.
 * @param u controls matrix.
 * @param u_prev last control input sent to the vehicle.
 * @param w slack variables vector.
 */
void ProxQP::setd(
  const MatrixXd & x, const MatrixXd & u, const VectorXd & u_prev,
  const VectorXd & w)
{
  size_t i = 0;
  size_t row;
  size_t col;
  size_t idx;

  /* First control input's bounds */
  for (size_t j = 0; j < model->getIneq("du").size(); j++) {
    idx = model->getIneq("du").at(j)[0];
    low(j) = model->getIneq("du").at(j)[1] * dt + u_prev(idx) - u(0, idx);
    upp(j) = model->getIneq("du").at(j)[2] * dt + u_prev(idx) - u(0, idx);
  }

  /* States' bounds */
  for (size_t j = 0; j < model->getIneq("x").size(); j++) {
    for (size_t l = ineq_idx[i]; l < ineq_idx[i + 1]; l++) {
      row = l - ineq_idx[i];
      col = model->getIneq("x").at(j)[0];
      low(l) = model->getIneq("x").at(j)[1] - x(row, col);
      upp(l) = model->getIneq("x").at(j)[2] - x(row, col);
    }
    i++;
  }

  /* Control inputs' bounds */
  for (size_t j = 0; j < model->getIneq("u").size(); j++) {
    for (size_t l = ineq_idx[i]; l < ineq_idx[i + 1]; l++) {
      row = l - ineq_idx[i];
      col = model->getIneq("u").at(j)[0];
      low(l) = model->getIneq("u").at(j)[1] - u(row, col);
      upp(l) = model->getIneq("u").at(j)[2] - u(row, col);
    }
    i++;
  }

  /* Control inputs derivatives' bounds */
  for (size_t j = 0; j < model->getIneq("du").size(); j++) {
    for (size_t l = ineq_idx[i]; l < ineq_idx[i + 1]; l++) {
      row = l - ineq_idx[i];
      col = model->getIneq("du").at(j)[0];

      low(l) = model->getIneq("du").at(j)[1] * dt - u(row + 1, col) + u(row, col);
      upp(l) = model->getIneq("du").at(j)[2] * dt - u(row + 1, col) + u(row, col);
    }
    i++;
  }

  /* Obstacle avoidance (linearized signed-distance half-plane, K slots/node) */
  if (obstacle_active == true) {
    /* Obstacle bounds with the discrete-time CBF coupling: enforce
     * h_{k+1} >= (1 - gamma) h_k - s, i.e. n^T dp + dw >= (1 - gamma) h_k - h_{k+1} - w,
     * using the signed distances cached in setC for this iterate. gamma = 1
     * recovers the pointwise bound -h_{k+1} - w. */
    for (size_t r = ineq_idx[i]; r < ineq_idx[i + 1]; r++) {
      const size_t slot = r - ineq_idx[i];
      low(r) = (1.0 - cbf_gamma) * obs_h_prev(slot) - obs_h(slot) - w(slot);
      upp(r) = std::numeric_limits<double>::infinity();
    }
    i++;

    /* Slack lower bounds: dw >= -w, i.e. s = w + dw >= 0. */
    for (size_t r = ineq_idx[i]; r < ineq_idx[i + 1]; r++) {
      const size_t slot = r - ineq_idx[i];
      low(r) = -w(slot);
      upp(r) = std::numeric_limits<double>::infinity();
    }
    i++;
  }
}

/*!
 * Set the intermediate states weight matrix.
 * @param Q weight matrix.
 */
void ProxQP::setQ(MatrixXd Q) {this->Q = Q;}

/*!
 * Set the control input weight matrix.
 * @param R weight matrix.
 */
void ProxQP::setR(MatrixXd R) {this->R = R;}

/*!
 * Set the final state weight matrix.
 * @param S weight matrix.
 */
void ProxQP::setS(MatrixXd S) {this->S = S;}

/*!
 * Set the slack variables weight matrix.
 * @param W weight matrix.
 */
void ProxQP::setW(MatrixXd W) {this->W = W;}

/*!
 * Set the number of shooting nodes (state).
 * If T is set, dt is automatically updated.
 * @param Np shooting nodes (> 0).
 */
void ProxQP::setNp(size_t Np)
{
  if (Np == 0) {throw std::invalid_argument("ProxQP::setNp: Np must be > 0");}
  this->Np = Np;
}

/*!
 * Set the number of shooting nodes (control).
 * @param Nc shooting nodes (> 0).
 */
void ProxQP::setNc(size_t Nc)
{
  if (Nc == 0) {throw std::invalid_argument("ProxQP::setNc: Nc must be > 0");}
  this->Nc = Nc;
}

/*!
 * Set the step size.
 * @param dt step size (> 0).
 */
void ProxQP::setdt(double dt)
{
  if (dt <= 0.0) {throw std::invalid_argument("ProxQP::setdt: dt must be > 0");}
  this->dt = dt;
}

/*!
 * Set the number of equalities.
 * @param n_eq equalities (>= 0).
 */
void ProxQP::setNEq(size_t n_eq)
{
  this->n_eq = n_eq;
}

/*!
 * Set the number of inequalities.
 * @param n_ineq inequalities (>= 0).
 */
void ProxQP::setNIneq(size_t n_ineq)
{
  this->n_ineq = n_ineq;
}

/*!
 * Set the maximum number of inner iterations.
 * @param max_inn_iter maximum iteration (> 0).
 */
void ProxQP::setMaxInIter(size_t max_inn_iter)
{
  if (max_inn_iter == 0) {
    throw std::invalid_argument("ProxQP::setMaxInIter: max_inn_iter must be > 0");
  }
  this->max_inn_iter = max_inn_iter;
}

/*!
 * Set the maximum number of outer iterations.
 * @param max_out_iter maximum iteration (> 0).
 */
void ProxQP::setMaxOutIter(size_t max_out_iter)
{
  if (max_out_iter == 0) {
    throw std::invalid_argument("ProxQP::setMaxOutIter: max_out_iter must be > 0");
  }
  this->max_out_iter = max_out_iter;
}

/*!
 * Set the QP type (sparse or dense).
 * @param qp_type sparse (false) or dense (true).
 */
void ProxQP::setQPType(bool qp_type) {this->qp_type = qp_type;}

/*!
 * Choose if using initial guesses for warm start.
 * @param guess no (false) or yes (true).
 */
void ProxQP::setGuess(bool guess) {this->guess = guess;}

/*!
 * Set the discrete-time CBF rate for the obstacle coupling.
 * @param cbf_gamma rate in (0, 1]; 1.0 reduces to the pointwise constraint.
 */
void ProxQP::setCbfGamma(double cbf_gamma)
{
  // Outside (0, 1] the bound (1 - gamma) * h_prev - h - w turns positive for the
  // far sentinel padding unused slots, making empty slots hard-binding at ~1e6
  // and destroying the solve.
  if (!(cbf_gamma > 0.0 && cbf_gamma <= 1.0)) {
    throw std::invalid_argument("ProxQP::setCbfGamma: cbf_gamma must be in (0, 1]");
  }
  this->cbf_gamma = cbf_gamma;
}

/*!
 * Set the obstacle-slot capacity K per predicted node (0 disables avoidance).
 * Must be set before init() so the QP is sized once.
 * @param max_obs capacity K.
 */
void ProxQP::setMaxObs(size_t max_obs) {this->max_obs = max_obs;}

/*!
 * Set the obstacle triples for the current cycle.
 * @param obs (Np*K) x 3 matrix of [o_x, o_y, d_safe] per (node, slot); empty
 *   slots should hold a far sentinel so their soft constraint is non-binding.
 */
void ProxQP::setObs(MatrixXd obs)
{
  // Shape check: setC indexes obs(slot, .) for slot in [0, Np*max_obs). With
  // EIGEN_NO_DEBUG the per-step access is unchecked, so reject an undersized
  // matrix here rather than read out of bounds on the control hot path.
  if (max_obs > 0) {
    const Eigen::Index expected_rows = static_cast<Eigen::Index>(Np * max_obs);
    if (obs.rows() != expected_rows || obs.cols() != 3) {
      throw std::invalid_argument("ProxQP::setObs: obs must be (Np*max_obs) x 3");
    }
  }
  this->obs = obs;
}

}  // namespace prox_mpc
