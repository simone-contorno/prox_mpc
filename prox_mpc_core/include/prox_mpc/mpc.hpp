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
/* MPCParams is inherited privately on purpose: the horizon buffers and weight
 * matrices are solver internals, reconfigured through MPC's public interface. */
class MPC : public ProbDim, private MPCParams
{
public:
  /* Constructor. */
  MPC() {qp_info.status = proxsuite::proxqp::QPSolverOutput::PROXQP_NOT_RUN;}

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
  void setMaxSolveTime(double seconds);
  void setGuess(bool guess);
  void setQPtype(bool qp_type);
  void setCbfGamma(double cbf_gamma);

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
  size_t getMaxObs();
  bool getGuess();

  /* Max obstacle soft-keep-out slack over the horizon after the last solve();
   * 0 when avoidance is disabled. >0 means the keep-out was relaxed (a
   * safety-feasibility signal exposed for telemetry without re-deriving it). */
  double getMaxObstacleSlack();

  /* Obstacle avoidance */

  void setMaxObs(size_t max_obs);
  void setObs(MatrixXd obs);

  /* Access the underlying QP solver, exposing the assembled problem for inspection. */
  std::shared_ptr<ProxQP> getSolver() {return proxqp;}

  /* Far sentinel for unused obstacle slots: a slot placed at (kObsFarSentinel,
   * kObsFarSentinel) with zero clearance yields a non-binding soft constraint. */
  static constexpr double kObsFarSentinel = 1e6;

  /* Variables */
  // Info is a plain aggregate with no default member initializers; value-initialize
  // it so a read before the first solve() is not indeterminate. The zero-valued
  // QPSolverOutput enumerator is PROXQP_SOLVED, so the constructor overrides the
  // status with PROXQP_NOT_RUN.
  proxsuite::proxqp::Info<double> qp_info{};  // QP information.
  size_t qp_iter_ext = 0;                     // Total QP external iterations (summed over SQP).
  size_t sqp_iter = 0;                        // SQP total iterations.

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

  /* Optional wall-clock budget for the whole SQP loop [s]; 0 disables it (the
   * iteration caps are then the only bound). When exceeded the loop stops early,
   * leaving qp_info.status != PROXQP_SOLVED so the caller's fail-safe runs. */
  double max_solve_time = 0.0;

  /* Discrete-time CBF rate forwarded to ProxQP (1.0 = pointwise obstacle term). */
  double cbf_gamma = 1.0;

  /* Obstacle avoidance (linearized signed-distance, bounded K per node). */
  size_t max_obs = 0;  // Capacity K of obstacle slots per predicted node (0 = disabled).
  MatrixXd obs;        // Per (node, slot) obstacle triples (Np*K) x [o_x, o_y, d_safe].
};

}  // namespace prox_mpc

#endif  // PROX_MPC__MPC_HPP_
