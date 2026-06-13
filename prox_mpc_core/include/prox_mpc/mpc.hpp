// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0

#ifndef PROX_MPC__MPC_HPP_
#define PROX_MPC__MPC_HPP_

#include <memory>
#include <tuple>

#include <prox_mpc/utils.hpp>
#include <prox_mpc/proxqp.hpp>
#include <prox_mpc/model.hpp>
#include <prox_mpc/structs.hpp>

namespace prox_mpc
{

/* Model Predictive Control class. */
class MPC : public ProbDim, MPCParams
{
public:
  /* Constructor. */
  MPC() {}

  /* Initialization */

  void init(std::shared_ptr<Model> model);

  /* ProxQP configuration */

  void configProxQP();

  /* Solving */

  std::tuple<MatrixXd, MatrixXd> solve();

  /* Set */

  void setX(MatrixXd x);
  void setQ(MatrixXd Q);
  void setR(MatrixXd R);
  void setS(MatrixXd S);
  void setW(MatrixXd W);
  void setNp(size_t Np);
  void setNc(size_t Nc);
  void setdt(double dt);
  void setT(double T);
  void setPose(VectorXd pose);
  void setGoalX(MatrixXd goal_x);
  void setGoalU(MatrixXd goal_u);
  void setMaxIntIterQP(size_t max_iter);
  void setMaxExtIterQP(size_t max_iter);
  void setMaxIterSQP(size_t max_iter);
  void setGuess(bool guess);
  void setQPtype(bool qp_type);

  /* Get */

  MatrixXd getX();
  MatrixXd getQ();
  MatrixXd getR();
  MatrixXd getS();
  MatrixXd getW();
  size_t getNp();
  size_t getNc();
  double getdt();
  double getT();
  VectorXd getPose();
  MatrixXd getGoalX();
  MatrixXd getGoalU();
  size_t getMaxIntIterQP();
  size_t getMaxExtIterQP();
  size_t getMaxIterSQP();
  bool getGuess();

  /* Obstacle avoidance */

  void setObs(MatrixXd obs);
  void setObsDim(double obs_l, double obs_w, double obs_pose_l, double obs_pose_w);

  /* Variables */
  proxsuite::proxqp::Info<double> qp_info;  // QP information.
  uint qp_iter_ext;                         // Total QP external iterations (summed over SQP).
  size_t sqp_iter;                          // SQP total iterations.

protected:
  /* Robot model. */
  std::shared_ptr<Model> model;

  /* MPC */
  VectorXd u0;        // First optimal control input (sent to the vehicle).
  VectorXd pose;      // Current vehicle pose.
  MatrixXd goal_x;    // State's goals.
  MatrixXd goal_u;    // Control's goals.
  bool guess = true;  // Use (true) / don't use (false) warm start for initial guesses.

  /* Default solver limits (centralized; override via the setters) */
  static constexpr size_t kDefaultMaxExtQP = 10000;  // Max QP external iterations.
  static constexpr size_t kDefaultMaxIntQP = 1500;   // Max QP internal iterations (proximal op).
  static constexpr size_t kDefaultMaxIterSQP = 100;  // Max SQP iterations.

  /* ProxQP */
  std::shared_ptr<ProxQP> proxqp;        // QP solver.
  size_t max_ext_qp = kDefaultMaxExtQP;  // Max QP external iterations.
  size_t max_int_qp = kDefaultMaxIntQP;  // Max QP internal iterations (proximal operator).
  bool qp_type = false;                  // Sparse (false) / dense (true) problem.

  /* SQP */
  size_t max_iter_sqp = kDefaultMaxIterSQP;  // Max SQP iterations.

  /* Obstacle avoidance goals. */
  MatrixXd obs;

  /* Obstacle dimensions */
  double obs_l;       // Obstacle box length.
  double obs_w;       // Obstacle box width.
  double obs_pose_l;  // Obstacle box distance between pose point and back side.
  double obs_pose_w;  // Obstacle box distance between pose point and right side.
};

}  // namespace prox_mpc

#endif  // PROX_MPC__MPC_HPP_
