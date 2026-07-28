// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// pluginlib registration of the bundled models, so they can be loaded by name
// through pluginlib::ClassLoader<prox_mpc::Model>. This lets model selection
// live in a separate package without modifying this library.

#include <pluginlib/class_list_macros.hpp>

#include <prox_mpc/model.hpp>
#include <prox_mpc/models/bicycle.hpp>
#include <prox_mpc/models/unicycle.hpp>

PLUGINLIB_EXPORT_CLASS(prox_mpc::Bicycle, prox_mpc::Model)
PLUGINLIB_EXPORT_CLASS(prox_mpc::Unicycle, prox_mpc::Model)
