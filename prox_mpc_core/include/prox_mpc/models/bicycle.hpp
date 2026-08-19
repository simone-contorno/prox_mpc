// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0

#ifndef PROX_MPC__MODELS__BICYCLE_HPP_
#define PROX_MPC__MODELS__BICYCLE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <prox_mpc/models/bicycle_front_axle.hpp>

namespace prox_mpc
{

/// Deprecated alias for BicycleFrontAxle.
///
/// The bicycle ships as two plugins, one per reference point, so the geometry is
/// unambiguous at the point of selection. This name predates that split and
/// resolves to the front-axle model, which is what its dynamics always were. It
/// keeps a configuration written against the previous release loading, warns
/// once per construction, and is removed in the next major release.
///
/// Its emitted twist and the pose it presents for a collision check are the
/// front-axle model's corrected ones, not the previous release's; the two
/// differ, and the difference is deliberate.
class Bicycle : public BicycleFrontAxle
{
public:
  Bicycle()
  {
    setName("bicycle");
    RCLCPP_WARN(
      rclcpp::get_logger("prox_mpc_core"),
      "Model 'prox_mpc_core/Bicycle' is deprecated and resolves to "
      "'prox_mpc_core/BicycleFrontAxle'; select that name, or "
      "'prox_mpc_core/BicycleRearAxle' for a rear-axle-referenced vehicle. "
      "The alias is removed in the next major release.");
  }
};

}  // namespace prox_mpc

#endif  // PROX_MPC__MODELS__BICYCLE_HPP_
