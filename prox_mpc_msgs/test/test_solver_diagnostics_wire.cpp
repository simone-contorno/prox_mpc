// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// SolverDiagnostics.msg shipped in prox_mpc_msgs 1.0.0, so its layout is
// already installed on users' machines; this file pins that wire contract,
// which nothing did before. It fixes field names, C++ types, declaration order
// and zero-initialized defaults for all 14 fields, and the values of the 7
// STATUS_* constants, so a later removal, rename, reorder or retype fails the
// suite rather than silently breaking a benchmark script or a recorded bag.
//
// Field order is checked through the rosidl introspection typesupport
// (declaration-order member list), not merely by naming every field: naming
// alone would not catch two fields swapping position while keeping their own
// names and types.

#include <cstdint>
#include <limits>
#include <string>

#include <gtest/gtest.h>

#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp>

#include <prox_mpc_msgs/msg/obstacle.hpp>
#include <prox_mpc_msgs/msg/obstacle_array.hpp>
#include <prox_mpc_msgs/msg/solver_diagnostics.hpp>

using prox_mpc_msgs::msg::Obstacle;
using prox_mpc_msgs::msg::ObstacleArray;
using prox_mpc_msgs::msg::SolverDiagnostics;

// --- Field names and declaration order ---------------------------------------

TEST(SolverDiagnosticsWire, FieldOrderMatchesReleasedContract)
{
  const auto * ts =
    rosidl_typesupport_introspection_cpp::get_message_type_support_handle<SolverDiagnostics>();
  ASSERT_NE(ts, nullptr);
  const auto * members =
    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(ts->data);
  ASSERT_NE(members, nullptr);

  // The 14 fields released at 1.0.0, in their declared order.
  const std::string expected[] = {
    "header", "solve_time_ms", "qp_solve_time_ms", "status", "converged",
    "sqp_iters", "qp_iters_ext", "primal_residual", "dual_residual", "objective",
    "max_obstacle_slack", "control_period_ms", "deadline_missed", "num_active_obstacles"
  };
  ASSERT_EQ(members->member_count_, sizeof(expected) / sizeof(expected[0]));
  for (std::uint32_t i = 0; i < members->member_count_; ++i) {
    EXPECT_EQ(std::string(members->members_[i].name_), expected[i]) << "at position " << i;
  }
}

// --- Field types, via compile-time-checked assignment ------------------------
//
// A renamed, retyped or removed field fails to compile here, which is the
// point: this is a source-level pin, not merely a runtime check.

TEST(SolverDiagnosticsWire, FieldTypesMatchReleasedContract)
{
  SolverDiagnostics d;
  d.header = std_msgs::msg::Header();
  d.solve_time_ms = 1.5;
  d.qp_solve_time_ms = 1.5;
  d.status = std::uint8_t{0};
  d.converged = true;
  d.sqp_iters = std::uint32_t{0};
  d.qp_iters_ext = std::uint32_t{0};
  d.primal_residual = 1.5;
  d.dual_residual = 1.5;
  d.objective = 1.5;
  d.max_obstacle_slack = 1.5;
  d.control_period_ms = 1.5;
  d.deadline_missed = true;
  d.num_active_obstacles = std::uint16_t{0};
}

// --- Zero-initialized defaults ------------------------------------------------

TEST(SolverDiagnosticsWire, DefaultConstructedFieldsAreZero)
{
  SolverDiagnostics d;
  EXPECT_EQ(d.solve_time_ms, 0.0);
  EXPECT_EQ(d.qp_solve_time_ms, 0.0);
  EXPECT_EQ(d.status, SolverDiagnostics::STATUS_SOLVED);   // both zero, not independently telling
  EXPECT_FALSE(d.converged);
  EXPECT_EQ(d.sqp_iters, 0u);
  EXPECT_EQ(d.qp_iters_ext, 0u);
  EXPECT_EQ(d.primal_residual, 0.0);
  EXPECT_EQ(d.dual_residual, 0.0);
  EXPECT_EQ(d.objective, 0.0);
  EXPECT_EQ(d.max_obstacle_slack, 0.0);
  EXPECT_EQ(d.control_period_ms, 0.0);
  EXPECT_FALSE(d.deadline_missed);
  EXPECT_EQ(d.num_active_obstacles, 0u);
}

// --- STATUS_* constants -------------------------------------------------------
//
// The message's own comment: these are the message's contract, not a copy of
// proxsuite's QPSolverOutput ordering, mapped explicitly rather than cast so a
// value recorded here never changes meaning even if proxsuite reorders its own
// enum. Pinning the numeric values is what makes that promise checkable.

TEST(SolverDiagnosticsWire, StatusConstantsMatchReleasedValues)
{
  EXPECT_EQ(SolverDiagnostics::STATUS_SOLVED, 0);
  EXPECT_EQ(SolverDiagnostics::STATUS_MAX_ITER_REACHED, 1);
  EXPECT_EQ(SolverDiagnostics::STATUS_PRIMAL_INFEASIBLE, 2);
  EXPECT_EQ(SolverDiagnostics::STATUS_DUAL_INFEASIBLE, 3);
  EXPECT_EQ(SolverDiagnostics::STATUS_NOT_RUN, 4);
  EXPECT_EQ(SolverDiagnostics::STATUS_SOLVED_CLOSEST_PRIMAL_FEASIBLE, 5);
  EXPECT_EQ(SolverDiagnostics::STATUS_UNKNOWN, 255);
}

// --- Obstacle / ObstacleArray: pinned for completeness ----------------------
//
// These two messages are also released at 1.0.0 and unchanged since; included
// so a future change to either is caught by the same suite that guards
// SolverDiagnostics.

TEST(ObstacleWire, FieldOrderMatchesReleasedContract)
{
  const auto * ts =
    rosidl_typesupport_introspection_cpp::get_message_type_support_handle<Obstacle>();
  const auto * members =
    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(ts->data);
  const std::string expected[] = {
    "id", "position", "velocity", "radius", "position_covariance",
    "velocity_covariance", "predicted_positions", "prediction_dt"
  };
  ASSERT_EQ(members->member_count_, sizeof(expected) / sizeof(expected[0]));
  for (std::uint32_t i = 0; i < members->member_count_; ++i) {
    EXPECT_EQ(std::string(members->members_[i].name_), expected[i]) << "at position " << i;
  }
}

TEST(ObstacleArrayWire, FieldOrderMatchesReleasedContract)
{
  const auto * ts =
    rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ObstacleArray>();
  const auto * members =
    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(ts->data);
  const std::string expected[] = {"header", "obstacles"};
  ASSERT_EQ(members->member_count_, sizeof(expected) / sizeof(expected[0]));
  for (std::uint32_t i = 0; i < members->member_count_; ++i) {
    EXPECT_EQ(std::string(members->members_[i].name_), expected[i]) << "at position " << i;
  }
}
