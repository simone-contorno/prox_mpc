// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0

#ifndef PROX_MPC__STRUCTS_HPP_
#define PROX_MPC__STRUCTS_HPP_

#include <map>
#include <string>
#include <vector>

#include <prox_mpc/utils.hpp>

namespace prox_mpc
{

/* Problem dimensions. */
struct ProbDim
{
  size_t dvars{0};   // Number of decision variables.
  size_t Np{0};      // Prediction horizon shooting nodes.
  size_t Nc{0};      // Control horizon shooting nodes.
  size_t n{0};       // State variables number.
  size_t m{0};       // Control variables number.
  double dt{0.0};    // Step size.
  double T{0.0};     // Prediction horizon.
  size_t n_eq{0};    // Equalities number.
  size_t n_ineq{0};  // Inequalities number.
};

/* Model Predictive Control parameters. */
struct MPCParams
{
  MatrixXd x;   // States (along the whole prediction horizon).
  MatrixXd u;   // Controls (along the whole control horizon).
  VectorXd w;   // Slack variable.
  MatrixXd S;   // Final state weight matrix.
  MatrixXd Q;   // Intermediate states weight matrix.
  MatrixXd R;   // Controls weight matrix.
  MatrixXd W;   // Slack variables weight matrix.
};

/* Model information. */
struct ModelInfo
{
  std::string name;  // Model name.
  size_t n{0};       // State vector dimension.
  size_t m{0};       // Control vector dimension.
  VectorXd params;   // Model parameters.
  VectorXd x;        // State vector.
  VectorXd u;        // Control vector.
  MatrixXd A;        // State matrix.
  MatrixXd B;        // Control matrix.
  VectorXd c;        // Local function approximation.
};

/* Problem constraints. */
struct Constraints
{
  std::map<int, std::vector<double>> ineq_x;   // State inequality constraints.
  std::map<int, std::vector<double>> ineq_u;   // Control inequality constraints.
  std::map<int, std::vector<double>> ineq_du;  // Acceleration inequality constraints.
  std::map<int, std::vector<double>> ineq_w;   // Slack variable inequality constraints.

  size_t map_idx_x;
  size_t map_idx_u;
  size_t map_idx_du;
  size_t map_idx_w;

  /* Obstacle avoidance */
  bool obs_flag;  // Whether the model supports obstacle avoidance.
};

}  // namespace prox_mpc

#endif  // PROX_MPC__STRUCTS_HPP_
