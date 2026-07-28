// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


// Standalone driver for the prox_mpc_obstacle_tracker lifecycle node. The node
// brings itself up (configure -> activate), spins, and tears itself down on
// SIGINT/SIGTERM through a single checked finalize() ladder (own signal
// handlers, async-signal-safe handler, checked transitions, one teardown
// path, clean spin cancel).

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <memory>
#include <thread>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "prox_mpc_obstacle_tracker/obstacle_tracker_node.hpp"

namespace
{

/// Lock-free stop flag set by the signal handler; a second signal force-quits.
std::atomic<bool> g_stop{false};
static_assert(
  std::atomic<bool>::is_always_lock_free, "stop flag must be lock-free for async-signal safety");

void on_signal(int /*sig*/)
{
  if (g_stop.load(std::memory_order_relaxed)) {
    std::_Exit(EXIT_FAILURE);  // second signal: force-quit, skip teardown
  }
  g_stop.store(true, std::memory_order_relaxed);
}

using prox_mpc_obstacle_tracker::ObstacleTrackerNode;

/// Single noexcept teardown ladder (deactivate -> cleanup -> shutdown), checked
/// and guarded so it runs on every post-construction exit and never throws.
void finalize(const std::shared_ptr<ObstacleTrackerNode> & node) noexcept
{
  try {
    using lifecycle_msgs::msg::State;
    if (!node || !rclcpp::ok()) {return;}
    if (node->get_current_state().id() == State::PRIMARY_STATE_ACTIVE) {
      node->deactivate();
    }
    if (node->get_current_state().id() == State::PRIMARY_STATE_INACTIVE) {
      // A failed cleanup lands in Finalized-via-error, which the shutdown step
      // below cannot distinguish from a clean finalize; report it here.
      const auto cleaned_state = node->cleanup();
      if (cleaned_state.id() != State::PRIMARY_STATE_UNCONFIGURED) {
        RCLCPP_ERROR(
          node->get_logger(), "finalize: cleanup() did not reach Unconfigured (state id %u).",
          cleaned_state.id());
      }
    }
    // shutdown() resolves to LifecycleNode::shutdown() (the node hides no such
    // name); it drives whatever state remains to Finalized.
    const auto final_state = node->shutdown();
    if (final_state.id() != State::PRIMARY_STATE_FINALIZED) {
      RCLCPP_ERROR(
        node->get_logger(), "finalize: node did not reach Finalized (state id %u).",
        final_state.id());
    }
  } catch (...) {
    // A teardown ladder must never propagate.
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  using lifecycle_msgs::msg::State;
  auto node = std::make_shared<ObstacleTrackerNode>();
  rclcpp::executors::SingleThreadedExecutor exec;

  bool brought_up = false;
  try {
    if (node->configure().id() != State::PRIMARY_STATE_INACTIVE) {
      RCLCPP_FATAL(node->get_logger(), "configure() did not reach Inactive; aborting.");
    } else if (g_stop.load(std::memory_order_relaxed)) {
      RCLCPP_WARN(node->get_logger(), "stop requested during bring-up; skipping activate.");
    } else if (node->activate().id() != State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_FATAL(node->get_logger(), "activate() did not reach Active; aborting.");
    } else {
      brought_up = true;
    }
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "bring-up exception: %s", e.what());
  }

  bool added = false;
  bool spin_failed = false;
  std::thread watcher;
  if (brought_up && !g_stop.load(std::memory_order_relaxed)) {
    exec.add_node(node->get_node_base_interface());
    added = true;
    watcher = std::thread(
      [&exec]() {
        while (!g_stop.load(std::memory_order_relaxed)) {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        // Best-effort guard against cancelling before the spin has started.
        for (int i = 0; i < 1000 && !exec.is_spinning(); ++i) {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        exec.cancel();
      });
    // An exception escaping a callback (publish, allocation) must still reach the
    // teardown ladder below instead of std::terminate()ing with the node active.
    try {
      exec.spin();
    } catch (const std::exception & e) {
      RCLCPP_FATAL(node->get_logger(), "executor spin terminated by exception: %s", e.what());
      spin_failed = true;
    }
  }

  g_stop.store(true, std::memory_order_relaxed);  // release the watcher
  if (watcher.joinable()) {watcher.join();}

  finalize(node);
  if (added) {exec.remove_node(node->get_node_base_interface());}
  node.reset();  // destroy the node before shutting the context down
  rclcpp::shutdown();
  return brought_up && !spin_failed ? EXIT_SUCCESS : EXIT_FAILURE;
}
