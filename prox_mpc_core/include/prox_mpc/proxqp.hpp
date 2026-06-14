// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0

#ifndef PROX_MPC__PROXQP_HPP_
#define PROX_MPC__PROXQP_HPP_

#include <memory>
#include <tuple>

#include <prox_mpc/utils.hpp>
#include <prox_mpc/model.hpp>
#include <prox_mpc/structs.hpp>

#include <proxsuite/helpers/optional.hpp>
#include <proxsuite/proxqp/dense/dense.hpp>
#include <proxsuite/proxqp/sparse/sparse.hpp>
#include <proxsuite/proxqp/utils/random_qp_problems.hpp>

using namespace proxsuite::proxqp;

namespace prox_mpc
{

/* ProxQP solver class. */
class ProxQP : public MPCParams
{
public:
  /* Constructor. */
  ProxQP() {}

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

  /* Obstacle avoidance */

  void setMaxObs(size_t max_obs);
  void setObs(MatrixXd obs);

  /* Accessors for the assembled inequality system (valid after a solve()). */
  const MatrixXd & getC() const {return C;}
  const VectorXd & getIneqIdx() const {return ineq_idx;}
  size_t getWStart() const {return w_start;}
  size_t getNDvars() const {return n_dvars;}
  size_t getMaxObs() const {return max_obs;}

private:
  /* Robot model. */
  std::shared_ptr<Model> model;

  /* ProxQP initialization */
  isize qp_dim = 1;   // QP problem dimension (number of decision variables).
  isize qp_eq = 0;    // QP equality constraints number.
  isize qp_ineq = 0;  // QP inequality constraints number.

  // Sparse and dense solvers.
  sparse::QP<double, isize> qp_sparse = sparse::QP<double, isize>(qp_dim, qp_eq, qp_ineq);
  dense::QP<double> qp_dense = dense::QP<double>(qp_dim, qp_eq, qp_ineq);

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
  size_t n_eq;        // Number of equalities.
  size_t n_ineq;      // Number of inequalities.
  VectorXd eq_idx;    // Equalities indeces.
  VectorXd ineq_idx;  // Inequalities indeces.
  size_t eq_tot;      // Total number of equalities.
  size_t ineq_tot;    // Total number of inequalities.

  /* Problem dimensions */
  size_t n;   // State dimension.
  size_t m;   // Control dimension.
  size_t Np;  // Prediction horizon shooting nodes.
  size_t Nc;  // Control horizon shooting nodes.
  double dt;  // Step size.

  /* Decision variables */
  size_t n_dvars;  // Total number of decision variables.
  size_t x_start;  // State starting index.
  size_t u_start;  // Control starting index.
  size_t w_start;  // Slack variable starting index.

  /* Settings */
  bool qp_type;         // Sparse (false) / dense (true) problem.
  bool guess;           // Use (true) / don't use (false) warm start for initial guesses.
  size_t max_out_iter;  // Maximum number of outer iterations.
  size_t max_inn_iter;  // Maximum number of inner iterations (proximal operator).

  /* Results */
  proxsuite::proxqp::sparse::Vec<double> result_x;       // Optimal decision variables.
  proxsuite::proxqp::sparse::Vec<double> result_lambda;  // Optimal equality Lagrange multipliers.
  proxsuite::proxqp::sparse::Vec<double> result_mu;      // Optimal inequality Lagrange multipliers.
  proxsuite::proxqp::Info<double> qp_info;               // QP information.

  /* Obstacle avoidance: linearized signed-distance half-plane, K slots per node. */
  size_t max_obs = 0;  // Capacity K of obstacle slots per predicted node (0 = disabled).
  MatrixXd obs;        // Per (node, slot) obstacle triples (Np*K) x [o_x, o_y, d_safe].

  // Signed distance h = ||p_k - o|| - d_safe per (node, slot), computed while
  // filling the inequality matrix in setC() and reused for the bounds in setd().
  VectorXd obs_h;
};

}  // namespace prox_mpc

#endif  // PROX_MPC__PROXQP_HPP_
