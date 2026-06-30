// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Standalone driver for the self-contained closed-loop NMPC simulation node. The
// node logic lives in the header so it can also be unit-tested; see
// prox_mpc_demo/simulation_node.hpp.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "prox_mpc_demo/simulation_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulationNode>());
  rclcpp::shutdown();
  return 0;
}
