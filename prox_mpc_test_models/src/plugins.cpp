// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// pluginlib registration of the fault-injection test models, so they can be
// loaded by name through pluginlib::ClassLoader<prox_mpc::Model> exactly as the
// bundled models are. Used only by the controller's unit tests.

#include <pluginlib/class_list_macros.hpp>

#include <prox_mpc/model.hpp>
#include <prox_mpc_test_models/non_finite_twist_model.hpp>

PLUGINLIB_EXPORT_CLASS(prox_mpc_test_models::NonFiniteTwistModel, prox_mpc::Model)
