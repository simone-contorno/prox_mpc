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

/* How a model's state and control vectors map onto the planar quantities a Nav2
 * consumer drives. Declared by Model::getPlanarMapping() and read once at the
 * consumer's configuration time, never on the control path.
 *
 * Validation is split by who indexes what. The core validates only the indices
 * it dereferences itself: today that is idx_x and idx_y, checked in
 * ProxQP::init() when obstacle avoidance is active. idx_yaw and the steering
 * indices are deliberately unchecked there, because no core path reads them -
 * a two-state holonomic model doing obstacle avoidance through the core
 * directly is legitimate, and its out-of-range idx_yaw default harms nothing.
 * The Nav2 controller validates all of them at readModelMapping(), for its own
 * use of them.
 *
 * Whoever first makes a core path index the heading or a steering channel must
 * extend the check in ProxQP::init() to cover it: EIGEN_NO_DEBUG turns an
 * out-of-range read into silent garbage rather than an abort. */
struct PlanarMapping
{
  /* Index value meaning "this model does not carry that quantity". */
  static constexpr size_t kNoIndex = static_cast<size_t>(-1);

  size_t idx_x{0};                // State index of the x position [m].
  size_t idx_y{1};                // State index of the y position [m].
  size_t idx_yaw{2};              // State index of the heading [rad].
  size_t idx_speed{0};            // Control index of the signed longitudinal speed [m/s].
  size_t idx_steering{kNoIndex};  // State index of the steering angle [rad], or kNoIndex.

  /* Control index of the steering-angle rate [rad/s], or kNoIndex. Meaningful
   * only alongside idx_steering: it is the channel whose declared bound sets how
   * fast a consumer may move its belief about the steering angle. */
  size_t idx_steer_rate{kNoIndex};

  /* Position of the point the state's x/y refer to, expressed in base_link
   * [m]: (0, 0) when the model is referenced to base_link itself, (L, 0) for a
   * model referenced to the front axle of a vehicle whose base_link sits on the
   * rear axle. A consumer transforms the model's pose by this offset before any
   * check defined about base_link, such as a footprint collision test. */
  double ref_offset_x{0.0};
  double ref_offset_y{0.0};

  /* Wheelbase L [m] the steering geometry is defined on. 0.0 means undeclared,
   * which a consumer must reject for a model that carries a steering angle. */
  double wheelbase{0.0};
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
