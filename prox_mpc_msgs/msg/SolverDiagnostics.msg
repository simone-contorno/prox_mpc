# Per-control-cycle NMPC/QP solver telemetry for benchmarking the three metric
# classes (real-time, feasibility, and the inputs to accuracy/precision). Filled
# from fields the core already computes (qp_info, sqp_iter, qp_iter_ext, slack w)
# and discards today. Publishers are opt-in: on by default in the standalone sim,
# off by default and only-when-subscribed in the production controller plugin.

# Control-cycle stamp.
std_msgs/Header header

# Wall solve time measured around MPC::solve() [ms].
float64 solve_time_ms

# QP-reported solve/run time from qp_info [ms].
float64 qp_solve_time_ms

# PROXQP QPSolverOutput of the last QP (see STATUS_* constants below).
uint8 status
uint8 STATUS_SOLVED=0
uint8 STATUS_MAX_ITER_REACHED=1
uint8 STATUS_PRIMAL_INFEASIBLE=2
uint8 STATUS_DUAL_INFEASIBLE=3
uint8 STATUS_NOT_RUN=4

# True when the cycle converged (status == STATUS_SOLVED and the applied iterate
# is finite); the caller's fail-safe runs otherwise.
bool converged

# SQP iterations and total external QP iterations summed over the SQP loop.
uint32 sqp_iters
uint32 qp_iters_ext

# Final QP primal / dual residuals (qp_info.pri_res / dua_res).
float64 primal_residual
float64 dual_residual

# Final QP objective value (qp_info.objValue).
float64 objective

# Max obstacle soft-keep-out slack w over the horizon [m]; >0 means the keep-out
# was relaxed (a safety-feasibility signal). 0 when avoidance is disabled.
float64 max_obstacle_slack

# Measured inter-cycle wall period [ms]; NaN on the first cycle.
float64 control_period_ms

# True when solve_time_ms or control_period_ms exceeds the 1000*dt budget.
bool deadline_missed

# Number of filled (non-sentinel) obstacle slots considered this cycle.
uint16 num_active_obstacles
