// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


#include <prox_mpc/proxqp.hpp>

#include <prox_mpc/mpc.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
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
// Far placeholder used only while probing the sparsity pattern at init.
constexpr double kObsProbeSentinel = 1e6;
// Fastest obstacle motion the backward extrapolation of the current-time
// obstacle position is trusted for [m/s]. The reconstruction 2 o_1 - o_2
// amplifies inter-block noise, so an implied inter-block displacement above
// kMaxObsSpeed * dt is treated as noise rather than motion and the extrapolation
// falls back to the first block. Nothing this controller plans around moves
// faster than this.
constexpr double kMaxObsSpeed = 10.0;
}  // namespace

/*!
 * Initialize ProxQP solver.
 * @param new_model model pointer.
 */
void ProxQP::init(std::shared_ptr<Model> new_model)
{
  /* Robot model. */
  this->model = new_model;

  /* Problem dimensions */
  this->n = new_model->getN();
  this->m = new_model->getM();

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
  obstacle_active = new_model->getObsFlag() == true && max_obs > 0;

  /* Where the model keeps its planar position. The obstacle rows are the only
   * part of the assembly that has to know, so the mapping is read here, with
   * obstacle_active, rather than per row or per cycle. Read only when the
   * obstacle term is active: a model that does not use it may declare whatever
   * mapping it likes, and rejecting one the solver never indexes would be this
   * layer overreaching.
   *
   * Validated here rather than left to the caller. A consumer that drives this
   * library directly - without the bundled Nav2 plugin's own model screening -
   * would otherwise index a state column that does not exist, which
   * EIGEN_NO_DEBUG turns into a silent out-of-range read rather than an abort. */
  if (obstacle_active == true) {
    const PlanarMapping mapping = new_model->getPlanarMapping();
    const std::string where = "ProxQP::init: model '" + new_model->getName() + "' ";
    if (mapping.idx_x >= n) {
      throw std::invalid_argument(
              where + "maps its x position to state index " +
              std::to_string(mapping.idx_x) + ", outside its " + std::to_string(n) +
              " states");
    }
    if (mapping.idx_y >= n) {
      throw std::invalid_argument(
              where + "maps its y position to state index " +
              std::to_string(mapping.idx_y) + ", outside its " + std::to_string(n) +
              " states");
    }
    if (mapping.idx_x == mapping.idx_y) {
      throw std::invalid_argument(
              where + "maps both planar positions to state index " +
              std::to_string(mapping.idx_x));
    }
    idx_pos_x = mapping.idx_x;
    idx_pos_y = mapping.idx_y;
  }

  /* Update the number of inequality blocks: the first control input, plus the
   * obstacle constraint block and the slack lower-bound block when active. */
  n_ineq += 1;                                  // first control input
  if (obstacle_active == true) {n_ineq += 2;}   // obstacle rows + slack lower-bound rows

  /* Build the inequality indices vector */
  ineq_idx.assign(n_ineq, 0);

  /* Set inequality constraint indices */
  size_t i = 0;

  ineq_idx[i] = new_model->getIneq("du").size();                   // first control-rate rows
  i++;

  for (size_t j = 0; j < new_model->getIneq("x").size(); j++) {     // state
    ineq_idx[i] = ineq_idx[i - 1] + (Np + 1);
    i++;
  }

  for (size_t j = 0; j < new_model->getIneq("u").size(); j++) {     // control input
    ineq_idx[i] = ineq_idx[i - 1] + Nc;
    i++;
  }

  for (size_t j = 0; j < new_model->getIneq("du").size(); j++) {    // control input derivative
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
    declarePattern();

    /* Constructed from the structural masks rather than the dimensions, so the
     * KKT layout is allocated once for the pattern every later update() reuses. */
    using BoolMat = sparse::SparseMat<bool, isize>;
    qp_sparse = sparse::QP<double, isize>(
      BoolMat(H_sparse.cast<bool>()), BoolMat(E_sparse.cast<bool>()),
      BoolMat(C_sparse.cast<bool>()));
    qp_sparse.settings.max_iter = max_out_iter;
    qp_sparse.settings.max_iter_in = max_inn_iter;
    qp_sparse.settings.compute_timings = true;
    qp_ready = false;
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
 * @brief Fixed-sparsity bookkeeping for ProxQP's in-place update.
 *
 * ProxQP's sparse backend only applies update() when the matrices carry the same
 * sparsity structure as the workspace was built with; a pattern that moves is
 * silently ignored, which drops the obstacle keep-out rows. Deriving the pattern
 * from sparseView() does exactly that, because the half-plane normals and the
 * model Jacobians pass through zero as the trajectory evolves.
 *
 * The structure is therefore declared once from the positions setE and setC
 * write - which depend only on the problem dimensions, never on the values - and
 * the values are written into it each cycle, structural zeros included.
 */
namespace
{
/// Structural pattern of `m`: every entry a filler touched, marked by non-NaN.
Eigen::SparseMatrix<double> writtenPattern(const MatrixXd & m)
{
  std::vector<Eigen::Triplet<double>> t;
  for (Eigen::Index j = 0; j < m.cols(); j++) {
    for (Eigen::Index i = 0; i < m.rows(); i++) {
      if (!std::isnan(m(i, j))) {
        t.emplace_back(static_cast<int>(i), static_cast<int>(j), 1.0);
      }
    }
  }
  Eigen::SparseMatrix<double> s(m.rows(), m.cols());
  s.setFromTriplets(t.begin(), t.end());
  s.makeCompressed();
  return s;
}

/// Copy the dense values into a fixed pattern, keeping structural zeros.
void writeInto(Eigen::SparseMatrix<double> & pat, const MatrixXd & dense)
{
  for (int k = 0; k < pat.outerSize(); k++) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(pat, k); it; ++it) {
      it.valueRef() = dense(it.row(), it.col());
    }
  }
}
}  // namespace

/*!
 * Declare the sparsity structure of the equality and inequality blocks.
 *
 * setE and setC touch a fixed set of positions on every call, so filling the
 * dense blocks with NaN and running them once reveals exactly which entries they
 * own, without duplicating their indexing here.
 */
void ProxQP::declarePattern()
{
  const MatrixXd x_probe = MatrixXd::Zero(Np + 1, n);
  const MatrixXd u_probe = MatrixXd::Zero(Nc, m);
  const MatrixXd obs_saved = obs;
  if (obstacle_active == true &&
    (obs.rows() != static_cast<Eigen::Index>(Np * max_obs) || obs.cols() != 3))
  {
    obs = MatrixXd::Constant(Np * max_obs, 3, kObsProbeSentinel);
  }

  const double marker = std::numeric_limits<double>::quiet_NaN();
  E.setConstant(marker);
  C.setConstant(marker);
  setE(x_probe, u_probe);
  setC(x_probe);
  E_sparse = writtenPattern(E);
  C_sparse = writtenPattern(C);
  E.setZero();
  C.setZero();
  obs = obs_saved;

  /* H is built once from the constant weights, so its own view is already fixed. */
  H_sparse = H.sparseView();
  H_sparse.makeCompressed();
}

/*!
 * @brief Initial-guess policy for the QP sub-problem.
 *
 * WARM_START rather than WARM_START_WITH_PREVIOUS_RESULT: the latter also carries
 * the proximal step sizes across solves, and unused obstacle slots are padded
 * with a far sentinel whose rows are ~1e6 in magnitude, so step sizes tuned
 * against that scaling cripple the next solve. WARM_START keeps the previous
 * primal/dual iterate and derives the active set from it, which is the part that
 * pays.
 *
 * Only meaningful while the workspace survives between solves. Requesting a warm
 * start right after a fresh init mixes a stale guess with a reset state, which
 * proxsuite 0.6.5 answers by diverging, so the caller selects this only on an
 * update() path.
 */
proxsuite::proxqp::InitialGuessStatus ProxQP::initialGuessPolicy() const
{
  return guess ?
         proxsuite::proxqp::InitialGuessStatus::WARM_START :
         proxsuite::proxqp::InitialGuessStatus::NO_INITIAL_GUESS;
}

/*!
 * @brief Initial-guess policy for a solve that (re)builds the workspace.
 *
 * A freshly initialized workspace has no previous iterate to warm start from, so
 * the cheap starts are all that apply here: the equality-constrained guess, or
 * none when the caller turned guessing off.
 */
proxsuite::proxqp::InitialGuessStatus ProxQP::coldGuessPolicy() const
{
  return guess ?
         proxsuite::proxqp::InitialGuessStatus::EQUALITY_CONSTRAINED_INITIAL_GUESS :
         proxsuite::proxqp::InitialGuessStatus::NO_INITIAL_GUESS;
}

/*!
 * Solve the quadratic program using ProxQP solver and compute the optimal state and control input evolution,
 * resulting in an optimal trajectory.
 * @param x_in states matrix.
 * @param u_in control inputs matrix.
 * @param u_prev last control input sent to the vehicle.
 * @param w_in slack variables vector for obstacle avoidance.
 * @param goal_x state's goals vector.
 * @param goal_u control's goals vector.
*/
std::tuple<MatrixXd, MatrixXd, VectorXd, proxsuite::proxqp::Info<double>>
ProxQP::solve(
  MatrixXd x_in, MatrixXd u_in, const VectorXd & u_prev, VectorXd w_in, const MatrixXd & goal_x,
  const MatrixXd & goal_u)
{
  /* Configure QP problem */
  setc(x_in, u_in, w_in, goal_x, goal_u);    // c
  setE(x_in, u_in);                          // E
  setb(x_in, u_in);                          // b
  setC(x_in);                                // C
  setd(x_in, u_in, u_prev, w_in);            // d

  /* Sparse problem */
  if (qp_type == false) {
    /* Values only: the structure was declared at init and must not move, or
     * ProxQP silently ignores the update and the keep-out rows go missing. */
    writeInto(E_sparse, E);
    writeInto(C_sparse, C);

    if (warm_start == true && qp_ready == true) {
      qp_sparse.settings.initial_guess = initialGuessPolicy();
      qp_sparse.update(H_sparse, c, E_sparse, b, C_sparse, low, upp);
    } else {
      qp_sparse.settings.initial_guess = coldGuessPolicy();
      qp_sparse.init(H_sparse, c, E_sparse, b, C_sparse, low, upp);
      qp_ready = true;
    }

    qp_sparse.solve();

    // Take results
    result_x = qp_sparse.results.x;

    // Take QP info
    qp_info = qp_sparse.results.info;
  }
  /* Dense problem */
  else {
    qp_dense.settings.initial_guess = coldGuessPolicy();
    qp_dense.init(H, c, E, b, C, low, upp);

    qp_dense.solve();

    // Take results
    result_x = qp_dense.results.x;

    // Take QP info
    qp_info = qp_dense.results.info;
  }

  /* Update decision variables */
  for (size_t i = x_start; i < u_start; i += n) {
    x_in.row(i / n) = result_x.segment(i, n);
  }
  for (size_t i = u_start; i < w_start; i += m) {
    u_in.row((i - u_start) / m) = result_x.segment(i, m);
  }
  for (size_t i = w_start; i < n_dvars; i++) {
    w_in(i - w_start) = result_x(i);
  }

  return {x_in, u_in, w_in, qp_info};
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
 * @param x_in states matrix.
 * @param u_in controls matrix.
 * @param w_in slack variables vector.
 * @param goal_x state's goals matrix.
 * @param goal_u control's goals matrix.
 */
void ProxQP::setc(
  const MatrixXd & x_in, const MatrixXd & u_in, const VectorXd & w_in,
  const MatrixXd & goal_x, const MatrixXd & goal_u)
{
  size_t count = 0;

  // Intermediate states
  for (size_t i = x_start; i < u_start - n; i += n) {
    c.segment(i, n) = 2 * Q * (x_in.row(count) - goal_x.row(count)).transpose();
    count++;
  }

  // Final state
  c.segment(u_start - n, n) = 2 * S * (x_in.row(count) - goal_x.row(count)).transpose();

  // Control inputs. Matches setH: all Nc blocks, so goal_u's last row is read.
  count = 0;
  for (size_t i = u_start; i < w_start; i += m) {
    c.segment(i, m) = 2 * R * (u_in.row(count).transpose() - goal_u.row(count).transpose());
    count++;
  }

  // Slack variable
  if (obstacle_active == true) {
    count = 0;
    for (size_t i = w_start; i < n_dvars; i++) {
      c.segment(i, 1) = 2 * W * w_in(count);
      count++;
    }
  }
}

/*!
 * Fill the equality constraints coefficient matrix E.
 * @param x_in states matrix.
 * @param u_in controls matrix.
 */
void ProxQP::setE(const MatrixXd & x_in, const MatrixXd & u_in)
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
    model->setX(x_in.row(count));
    model->setU(
      u_in.row(std::clamp(static_cast<int>(count), 0, static_cast<int>(u_in.rows() - 1))));

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
 * @param x_in states matrix.
 * @param u_in controls matrix.
 */
void ProxQP::setb(const MatrixXd & x_in, const MatrixXd & u_in)
{
  size_t count = 0;

  /* Model kinematics */
  for (size_t i = eq_idx[0]; i < eq_idx[1]; i += n) {
    // Update for the current step
    model->setX(x_in.row(count));
    model->setU(
      u_in.row(std::clamp(static_cast<int>(count), 0, static_cast<int>(u_in.rows() - 1))));
    model->updatec(dt, x_in.row(count + 1));

    b.segment(i, n) = -model->getc();

    count++;
  }
}

/*!
 * Fill the inequality constraints coefficients matrix C.
 * @param x_in states matrix.
 */
void ProxQP::setC(const MatrixXd & x_in)
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
      col = x_start + n * (node + 1);               // state block of predicted node (node+1)

      const double dx = x_in(node + 1, idx_pos_x) - obs(slot, 0);
      const double dy = x_in(node + 1, idx_pos_y) - obs(slot, 1);
      const double raw_norm = sqrt(dx * dx + dy * dy);
      double norm = raw_norm;
      if (norm < kObsNormalEps) {norm = kObsNormalEps;}
      double nx;
      double ny;
      if (raw_norm < kObsNormalEps) {
        /* At or near coincidence dx, dy carry no reliable escape direction (dividing
         * by the floored norm would emit a zero or non-unit vector); fall back to a
         * deterministic unit normal instead. */
        nx = 1.0;
        ny = 0.0;
      } else {
        nx = dx / norm;
        ny = dy / norm;
      }

      C(r, col + idx_pos_x) = nx;                   // x
      C(r, col + idx_pos_y) = ny;                   // y
      C(r, w_start + slot) = 1;                     // slack

      obs_h(slot) = norm - obs(slot, 2);            // signed distance at node k+1

      /* Signed distance at the previous node k for the CBF coupling, against the
       * SAME obstacle slot at the previous node's time. A predictive (moving) fill
       * carries a different obstacle position per node, so node k uses obs[slot -
       * max_obs]; for a static fill this reduces to obs[slot].
       *
       * Node 0 is the exception: the previous state is x(0), the current pose, and
       * the matrix's first block holds the obstacle at state 1, not now. Using it
       * evaluates the two sides of the constraint at different obstacle times and
       * does so anti-conservatively for a closing obstacle, at the first and most
       * actionable constraint in the horizon. The current-time position is
       * recovered below. */
      const size_t prev_slot = (node >= 1) ? (slot - max_obs) : slot;
      /* If either this slot or the previous node's slot is unfilled (holds the far
       * sentinel), the pair does not describe the same obstacle across two nodes;
       * skip the coupling term by leaving obs_h_prev at zero. It enters setd() only
       * as (1 - cbf_gamma) * obs_h_prev, which is already exactly zero at the
       * shipped cbf_gamma = 1.0, so the whole coupling - value and gradient alike -
       * is gated on gamma < 1 and adds nothing at the default. Leaving the gradient
       * columns unwritten there also keeps them out of the constraint matrix's
       * sparsity pattern, which is what costs factorization work per cycle. */
      if (cbf_gamma < 1.0) {
        double gx = 0.0;
        double gy = 0.0;
        if (obs(slot, 0) >= 0.5 * MPC::kObsFarSentinel ||
          obs(prev_slot, 0) >= 0.5 * MPC::kObsFarSentinel)
        {
          obs_h_prev(slot) = 0.0;
        } else {
          /* Obstacle position at the previous node's own time. For node 0 that is
           * the current time, which the matrix carries no block for, so it is
           * recovered by extending the first two blocks backward:
           * o_0 = 2 o(block 0) - o(block 1). That is exact for a static fill,
           * where the two blocks are equal and it returns o(block 0) unchanged,
           * and exact for a constant-velocity fill. The clearance radius is a
           * per-obstacle constant and is taken from the first block rather than
           * extrapolated. */
          double ox = obs(prev_slot, 0);
          double oy = obs(prev_slot, 1);
          if (node == 0 && Np >= 2) {
            const size_t next_slot = slot + max_obs;
            if (obs(next_slot, 0) < 0.5 * MPC::kObsFarSentinel) {
              const double step_x = obs(slot, 0) - obs(next_slot, 0);
              const double step_y = obs(slot, 1) - obs(next_slot, 1);
              /* Extrapolation amplifies inter-block noise as 2 e_1 - e_2, so an
               * implied displacement no obstacle could have travelled in one step
               * is treated as noise and the first block is used as it stands. */
              const double max_step = kMaxObsSpeed * dt;
              if (step_x * step_x + step_y * step_y <= max_step * max_step) {
                ox = obs(slot, 0) + step_x;
                oy = obs(slot, 1) + step_y;
              }
            }
          }
          const double dxp = x_in(node, idx_pos_x) - ox;
          const double dyp = x_in(node, idx_pos_y) - oy;
          const double norm_prev = sqrt(dxp * dxp + dyp * dyp);
          obs_h_prev(slot) = norm_prev - obs(prev_slot, 2);
          /* Second half of the linearization: the constraint is
           * h_{k+1}(p_{k+1}) >= (1 - gamma) h_k(p_k) - s, so the previous node's
           * clearance depends on that node's own position too. Carrying only the
           * value term made the coupled constraint first-order exact in p_{k+1}
           * alone; this row completes it with -(1 - gamma) grad h_k. The same
           * deterministic unit fallback as above covers exact coincidence. */
          const double gain = -(1.0 - cbf_gamma);
          if (norm_prev < kObsNormalEps) {
            gx = gain;
            gy = 0.0;
          } else {
            gx = gain * dxp / norm_prev;
            gy = gain * dyp / norm_prev;
          }
        }
        /* Node 0's gradient columns are inert: x(0) is pinned to the current
         * pose by the identity equality in setE, so an increment there is fixed
         * at zero and these coefficients cannot influence the solution. Writing
         * them would still enlarge the constraint matrix's sparsity pattern and
         * cost factorization work on every cycle, so they are skipped. Node 0's
         * VALUE term above is not inert - it is the constant right-hand side of
         * the first coupled constraint - and is always computed. */
        if (node >= 1) {
          const size_t col_prev = x_start + n * node;  // state block of node k
          C(r, col_prev + idx_pos_x) = gx;
          C(r, col_prev + idx_pos_y) = gy;
        }
      }
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
 * @param x_in states matrix.
 * @param u_in controls matrix.
 * @param u_prev last control input sent to the vehicle.
 * @param w_in slack variables vector.
 */
void ProxQP::setd(
  const MatrixXd & x_in, const MatrixXd & u_in, const VectorXd & u_prev,
  const VectorXd & w_in)
{
  size_t i = 0;
  size_t row;
  size_t col;
  size_t idx;

  /* First control input's bounds */
  for (size_t j = 0; j < model->getIneq("du").size(); j++) {
    idx = model->getIneq("du").at(j)[0];
    low(j) = model->getIneq("du").at(j)[1] * dt + u_prev(idx) - u_in(0, idx);
    upp(j) = model->getIneq("du").at(j)[2] * dt + u_prev(idx) - u_in(0, idx);
  }

  /* States' bounds */
  for (size_t j = 0; j < model->getIneq("x").size(); j++) {
    for (size_t l = ineq_idx[i]; l < ineq_idx[i + 1]; l++) {
      row = l - ineq_idx[i];
      col = model->getIneq("x").at(j)[0];
      low(l) = model->getIneq("x").at(j)[1] - x_in(row, col);
      upp(l) = model->getIneq("x").at(j)[2] - x_in(row, col);
    }
    i++;
  }

  /* Control inputs' bounds */
  for (size_t j = 0; j < model->getIneq("u").size(); j++) {
    for (size_t l = ineq_idx[i]; l < ineq_idx[i + 1]; l++) {
      row = l - ineq_idx[i];
      col = model->getIneq("u").at(j)[0];
      low(l) = model->getIneq("u").at(j)[1] - u_in(row, col);
      upp(l) = model->getIneq("u").at(j)[2] - u_in(row, col);
    }
    i++;
  }

  /* Control inputs derivatives' bounds */
  for (size_t j = 0; j < model->getIneq("du").size(); j++) {
    for (size_t l = ineq_idx[i]; l < ineq_idx[i + 1]; l++) {
      row = l - ineq_idx[i];
      col = model->getIneq("du").at(j)[0];

      low(l) = model->getIneq("du").at(j)[1] * dt - u_in(row + 1, col) + u_in(row, col);
      upp(l) = model->getIneq("du").at(j)[2] * dt - u_in(row + 1, col) + u_in(row, col);
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
      low(r) = (1.0 - cbf_gamma) * obs_h_prev(slot) - obs_h(slot) - w_in(slot);
      upp(r) = std::numeric_limits<double>::infinity();
    }
    i++;

    /* Slack lower bounds: dw >= -w, i.e. s = w + dw >= 0. */
    for (size_t r = ineq_idx[i]; r < ineq_idx[i + 1]; r++) {
      const size_t slot = r - ineq_idx[i];
      low(r) = -w_in(slot);
      upp(r) = std::numeric_limits<double>::infinity();
    }
    i++;
  }
}

/*!
 * Set the intermediate states weight matrix.
 * @param new_Q weight matrix.
 */
void ProxQP::setQ(MatrixXd new_Q) {this->Q = new_Q;}

/*!
 * Set the control input weight matrix.
 * @param new_R weight matrix.
 */
void ProxQP::setR(MatrixXd new_R) {this->R = new_R;}

/*!
 * Set the final state weight matrix.
 * @param new_S weight matrix.
 */
void ProxQP::setS(MatrixXd new_S) {this->S = new_S;}

/*!
 * Set the slack variables weight matrix.
 * @param new_W weight matrix.
 */
void ProxQP::setW(MatrixXd new_W) {this->W = new_W;}

/*!
 * Set the number of shooting nodes (state).
 * If T is set, dt is automatically updated.
 * @param new_Np shooting nodes (> 0).
 */
void ProxQP::setNp(size_t new_Np)
{
  if (new_Np == 0) {throw std::invalid_argument("ProxQP::setNp: Np must be > 0");}
  this->Np = new_Np;
}

/*!
 * Set the number of shooting nodes (control).
 * @param new_Nc shooting nodes (> 0).
 */
void ProxQP::setNc(size_t new_Nc)
{
  if (new_Nc == 0) {throw std::invalid_argument("ProxQP::setNc: Nc must be > 0");}
  this->Nc = new_Nc;
}

/*!
 * Set the step size.
 * @param new_dt step size (> 0).
 */
void ProxQP::setdt(double new_dt)
{
  if (new_dt <= 0.0) {throw std::invalid_argument("ProxQP::setdt: dt must be > 0");}
  this->dt = new_dt;
}

/*!
 * Set the number of equalities.
 * @param new_n_eq equalities (>= 0).
 */
void ProxQP::setNEq(size_t new_n_eq)
{
  this->n_eq = new_n_eq;
}

/*!
 * Set the number of inequalities.
 * @param new_n_ineq inequalities (>= 0).
 */
void ProxQP::setNIneq(size_t new_n_ineq)
{
  this->n_ineq = new_n_ineq;
}

/*!
 * Set the maximum number of inner iterations.
 * @param new_max_inn_iter maximum iteration (> 0).
 */
void ProxQP::setMaxInIter(size_t new_max_inn_iter)
{
  if (new_max_inn_iter == 0) {
    throw std::invalid_argument("ProxQP::setMaxInIter: max_inn_iter must be > 0");
  }
  this->max_inn_iter = new_max_inn_iter;
}

/*!
 * Set the maximum number of outer iterations.
 * @param new_max_out_iter maximum iteration (> 0).
 */
void ProxQP::setMaxOutIter(size_t new_max_out_iter)
{
  if (new_max_out_iter == 0) {
    throw std::invalid_argument("ProxQP::setMaxOutIter: max_out_iter must be > 0");
  }
  this->max_out_iter = new_max_out_iter;
}

/*!
 * Set the QP type (sparse or dense).
 * @param new_qp_type sparse (false) or dense (true).
 */
void ProxQP::setQPType(bool new_qp_type) {this->qp_type = new_qp_type;}

/*!
 * Choose if using initial guesses for warm start.
 * @param new_guess no (false) or yes (true).
 */
void ProxQP::setGuess(bool new_guess) {this->guess = new_guess;}

/*!
 * Enable the cross-cycle QP warm start.
 * @param new_warm_start reuse the workspace and the previous iterate.
 */
void ProxQP::setWarmStart(bool new_warm_start) {this->warm_start = new_warm_start;}

/*!
 * Set the discrete-time CBF rate for the obstacle coupling.
 * @param new_cbf_gamma rate in (0, 1]; 1.0 reduces to the pointwise constraint.
 */
void ProxQP::setCbfGamma(double new_cbf_gamma)
{
  // Outside (0, 1] the bound (1 - gamma) * h_prev - h - w turns positive for the
  // far sentinel padding unused slots, making empty slots hard-binding at ~1e6
  // and destroying the solve.
  if (!(new_cbf_gamma > 0.0 && new_cbf_gamma <= 1.0)) {
    throw std::invalid_argument("ProxQP::setCbfGamma: cbf_gamma must be in (0, 1]");
  }
  this->cbf_gamma = new_cbf_gamma;
}

/*!
 * Set the obstacle-slot capacity K per predicted node (0 disables avoidance).
 * Must be set before init() so the QP is sized once.
 * @param new_max_obs capacity K.
 */
void ProxQP::setMaxObs(size_t new_max_obs) {this->max_obs = new_max_obs;}

/*!
 * Set the obstacle triples for the current cycle.
 * @param new_obs (Np*K) x 3 matrix of [o_x, o_y, d_safe] per (node, slot); empty
 *   slots should hold a far sentinel so their soft constraint is non-binding.
 */
void ProxQP::setObs(MatrixXd new_obs)
{
  // Shape check: setC indexes obs(slot, .) for slot in [0, Np*max_obs). With
  // EIGEN_NO_DEBUG the per-step access is unchecked, so reject an undersized
  // matrix here rather than read out of bounds on the control hot path.
  if (max_obs > 0) {
    const Eigen::Index expected_rows = static_cast<Eigen::Index>(Np * max_obs);
    if (new_obs.rows() != expected_rows || new_obs.cols() != 3) {
      throw std::invalid_argument("ProxQP::setObs: obs must be (Np*max_obs) x 3");
    }
  }
  this->obs = new_obs;
}

}  // namespace prox_mpc
