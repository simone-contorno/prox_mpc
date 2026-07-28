// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0

#ifndef PROX_MPC__PROXQP_HPP_
#define PROX_MPC__PROXQP_HPP_

#include <memory>
#include <tuple>
#include <vector>

#include <prox_mpc/utils.hpp>
#include <prox_mpc/model.hpp>
#include <prox_mpc/structs.hpp>

#include <proxsuite/helpers/optional.hpp>
#include <proxsuite/proxqp/dense/dense.hpp>
#include <proxsuite/proxqp/sparse/sparse.hpp>

namespace prox_mpc
{

/* ProxQP solver class. */
class ProxQP : public MPCParams
{
public:
  /* Constructor. */
  ProxQP() {qp_info.status = proxsuite::proxqp::QPSolverOutput::PROXQP_NOT_RUN;}

  /* Initialization */

  void init(std::shared_ptr<Model> model);

  /* Solving */

  std::tuple<MatrixXd, MatrixXd, VectorXd, proxsuite::proxqp::Info<double>> solve(
    MatrixXd x, MatrixXd u, const VectorXd & u_prev, VectorXd w, const MatrixXd & goal_x,
    const MatrixXd & goal_u);

  /* Set */

  void setH();
  void setc(
    const MatrixXd & x, const MatrixXd & u, const VectorXd & w, const MatrixXd & goal_x,
    const MatrixXd & goal_u);
  void setE(const MatrixXd & x, const MatrixXd & u);
  void setb(const MatrixXd & x, const MatrixXd & u);
  void setC(const MatrixXd & x);
  void setd(const MatrixXd & x, const MatrixXd & u, const VectorXd & u_prev, const VectorXd & w);

  void setQ(MatrixXd Q);
  void setR(MatrixXd R);
  void setS(MatrixXd S);
  void setW(MatrixXd W);
  void setNp(size_t Np);
  void setNc(size_t Nc);
  void setdt(double dt);
  void setNEq(size_t n_eq);
  void setNIneq(size_t n_ineq);
  void setMaxInIter(size_t max_inn_iter);
  void setMaxOutIter(size_t max_out_iter);
  void setQPType(bool qp_type);
  void setGuess(bool guess);

  /// Initial-guess policy handed to proxsuite for each QP sub-problem.
  proxsuite::proxqp::InitialGuessStatus initialGuessPolicy() const;
  void setCbfGamma(double cbf_gamma);

  /* Obstacle avoidance */

  void setMaxObs(size_t max_obs);
  void setObs(MatrixXd obs);

  /* Accessors for the assembled inequality system (valid after a solve()). */
  const MatrixXd & getC() const {return C;}
  const std::vector<size_t> & getIneqIdx() const {return ineq_idx;}
  size_t getWStart() const {return w_start;}
  size_t getNDvars() const {return n_dvars;}
  size_t getMaxObs() const {return max_obs;}

private:
  /* Robot model. */
  std::shared_ptr<Model> model;

  /* ProxQP initialization */
  proxsuite::proxqp::isize qp_dim = 1;   // QP problem dimension (number of decision variables).
  proxsuite::proxqp::isize qp_eq = 0;    // QP equality constraints number.
  proxsuite::proxqp::isize qp_ineq = 0;  // QP inequality constraints number.

  // Sparse and dense solvers.
  proxsuite::proxqp::sparse::QP<double, proxsuite::proxqp::isize> qp_sparse =
    proxsuite::proxqp::sparse::QP<double, proxsuite::proxqp::isize>(qp_dim, qp_eq, qp_ineq);
  proxsuite::proxqp::dense::QP<double> qp_dense =
    proxsuite::proxqp::dense::QP<double>(qp_dim, qp_eq, qp_ineq);

  /* Problem configuration */
  MatrixXd H;   // Dense Hessian matrix.
  MatrixXd E;   // Dense equality constraints matrix.
  MatrixXd C;   // Dense inequality constraints matrix.

  Eigen::SparseMatrix<double> H_sparse;  // Sparse Hessian matrix.
  Eigen::SparseMatrix<double> E_sparse;  // Sparse equality constraints matrix.
  Eigen::SparseMatrix<double> C_sparse;  // Sparse inequality constraints matrix.

  VectorXd c;    // Coefficient vector.
  VectorXd b;    // Equality constraints vector.
  VectorXd upp;  // Inequality constraints upper bounds vector.
  VectorXd low;  // Inequality constraints lower bounds vector.

  /* Constraints */
  size_t n_eq = 0;                   // Number of equalities.
  size_t n_ineq = 0;                 // Number of inequalities.
  std::vector<size_t> eq_idx;        // Equalities indices.
  std::vector<size_t> ineq_idx;      // Inequalities indices.
  size_t eq_tot = 0;                 // Total number of equalities.
  size_t ineq_tot = 0;               // Total number of inequalities.

  /* Problem dimensions */
  size_t n = 0;     // State dimension.
  size_t m = 0;     // Control dimension.
  size_t Np = 0;    // Prediction horizon shooting nodes.
  size_t Nc = 0;    // Control horizon shooting nodes.
  double dt = 0.0;  // Step size.

  /* Decision variables */
  size_t n_dvars = 0;  // Total number of decision variables.
  size_t x_start = 0;  // State starting index.
  size_t u_start = 0;  // Control starting index.
  size_t w_start = 0;  // Slack variable starting index.

  /* Settings (mirror the MPC defaults; MPC overwrites them all before init()). */
  bool qp_type = false;         // Sparse (false) / dense (true) problem.
  bool guess = true;            // Use (true) / don't use (false) warm start for initial guesses.
  size_t max_out_iter = 10000;  // Maximum number of outer iterations.
  size_t max_inn_iter = 1500;   // Maximum number of inner iterations (proximal operator).

  /* Results */
  proxsuite::proxqp::sparse::Vec<double> result_x;       // Optimal decision variables.
  // Info is a plain aggregate with no default member initializers; value-initialize
  // it so a read before the first solve() is not indeterminate. The zero-valued
  // QPSolverOutput enumerator is PROXQP_SOLVED, so the constructor overrides the
  // status with PROXQP_NOT_RUN.
  proxsuite::proxqp::Info<double> qp_info{};

  /* Obstacle avoidance: linearized signed-distance half-plane, K slots per node. */
  size_t max_obs = 0;             // Capacity K of obstacle slots per predicted node (0 = disabled).
  bool obstacle_active = false;   // Cached in init(): model declares avoidance and max_obs > 0.
  MatrixXd obs;                   // Per (node, slot) obstacle triples (Np*K) x [o_x, o_y, d_safe].

  // Discrete-time control-barrier-function rate (Zeng et al., ACC 2021):
  // h(x_{k+1}) >= (1 - cbf_gamma) * h(x_k). cbf_gamma in (0, 1]; 1.0 reduces the
  // coupling to the pointwise constraint h(x_{k+1}) >= 0 (the bundled default).
  double cbf_gamma = 1.0;

  // Signed distance h = ||p_k - o|| - d_safe per (node, slot). obs_h is taken at
  // the constrained node (k+1) and obs_h_prev at the previous node (k); both are
  // computed in setC() and reused for the bounds in setd().
  VectorXd obs_h;
  VectorXd obs_h_prev;
};

}  // namespace prox_mpc

#endif  // PROX_MPC__PROXQP_HPP_
