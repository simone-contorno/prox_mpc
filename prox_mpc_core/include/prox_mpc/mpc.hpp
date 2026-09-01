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

  void init(std::shared_ptr<Model> new_model);

  /* ProxQP configuration */

  void configProxQP();

  /* Solving */

  /*!
   * Run one SQP cycle and commit its result, the entry point this class shipped
   * with. Equivalent to solveCandidate() followed by retaining the candidate
   * whatever its status, with u0 advanced only on a converged solve.
   */
  std::tuple<MatrixXd, MatrixXd> solve();

  /*!
   * Run one SQP cycle without retaining anything: x, u, w and u0 keep the values
   * they had on entry, and the caller commits with commitCandidate() once its own
   * acceptance gates have passed. This is the transactional entry point.
   */
  std::tuple<MatrixXd, MatrixXd> solveCandidate();

  /*!
   * Retain the candidate produced by the last solveCandidate(), advancing x, u, w
   * and u0. It is a no-op returning false unless that candidate both converged
   * and is finite over the whole horizon, so a caller that never commits simply
   * does not advance.
   */
  bool commitCandidate();

  /* Whether the last solveCandidate() produced finite x, u and w over the whole
   * horizon. False before the first solveCandidate(). */
  bool getCandidateFinite();

  /* Set */

  void setX(MatrixXd new_x);

  /*!
   * Set the previous control input the next cycle's rate constraint is anchored
   * on. A caller that overrides the command this class produced - a deceleration
   * ramp on a rejected cycle, say - sets it here so the anchor is the control
   * actually applied rather than one that was never sent.
   */
  void setU0(VectorXd new_u0);
  void setQ(MatrixXd new_Q);
  void setR(MatrixXd new_R);
  void setS(MatrixXd new_S);
  void setW(MatrixXd new_W);
  void setNp(size_t new_Np);
  void setNc(size_t new_Nc);
  void setdt(double new_dt);
  void setT(double new_T);
  void setPose(VectorXd new_pose);
  void setGoalX(MatrixXd new_goal_x);
  void setGoalU(MatrixXd new_goal_u);
  void setMaxIntIterQP(size_t max_iter);
  void setMaxExtIterQP(size_t max_iter);
  void setMaxIterSQP(size_t max_iter);
  void setMaxSolveTime(double seconds);
  void setGuess(bool new_guess);
  void setWarmStart(bool new_warm_start);
  void setQPtype(bool new_qp_type);
  void setCbfGamma(double new_cbf_gamma);

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

  void setMaxObs(size_t new_max_obs);
  void setObs(MatrixXd new_obs);

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
  bool guess = true;        // ProxQP cheap-start policy selector.
  bool warm_start = true;   // Cross-cycle QP warm start (workspace + iterate reuse).

  /* Staged result of the last solveCandidate(), retained only by commitCandidate(). */
  MatrixXd cand_x;              // Candidate states.
  MatrixXd cand_u;              // Candidate controls.
  VectorXd cand_w;              // Candidate slack.
  VectorXd cand_u0;             // Candidate first control input.
  bool cand_solved = false;     // Whether the candidate's last QP converged.
  bool cand_finite = false;     // Whether the candidate is finite over the whole horizon.

  /* Set by init(); the structural setters reject a call made after it. */
  bool initialized = false;

  /* Default solver limits (centralized; override via the setters) */
  static constexpr size_t kDefaultMaxExtQP = 10000;  // Max QP external iterations.
  static constexpr size_t kDefaultMaxIntQP = 1500;   // Max QP internal iterations (proximal op).
  static constexpr size_t kDefaultMaxIterSQP = 1;  // Max SQP iterations (real-time iteration).

  /* ProxQP */
  std::shared_ptr<ProxQP> proxqp;        // QP solver.
  size_t max_ext_qp = kDefaultMaxExtQP;  // Max QP external iterations.
  size_t max_int_qp = kDefaultMaxIntQP;  // Max QP internal iterations (proximal operator).
  bool qp_type = false;                  // Sparse (false) / dense (true) problem.

  /* SQP */
  size_t max_iter_sqp = kDefaultMaxIterSQP;  // Max SQP iterations.

  /* Optional soft wall-clock budget for the SQP loop [s]; 0 disables it. It is
   * tested between SQP iterations, so it caps how many further QP sub-problems
   * start and cannot interrupt one in flight - proxsuite offers no time-based
   * stop. A loop that exceeds it stops early with whatever status the last QP
   * returned, which is PROXQP_SOLVED when that QP converged, so this is not a
   * worst-case latency bound and does not by itself route the caller to its
   * fail-safe. The per-cycle bound is the iteration caps (max_iter_sqp = 1 for a
   * bounded real-time iteration). */
  double max_solve_time = 0.0;

  /* Discrete-time CBF rate forwarded to ProxQP (1.0 = pointwise obstacle term). */
  double cbf_gamma = 1.0;

  /* Obstacle avoidance (linearized signed-distance, bounded K per node). */
  size_t max_obs = 0;  // Capacity K of obstacle slots per predicted node (0 = disabled).
  MatrixXd obs;        // Per (node, slot) obstacle triples (Np*K) x [o_x, o_y, d_safe].
};

}  // namespace prox_mpc

#endif  // PROX_MPC__MPC_HPP_
