#!/usr/bin/env bash
# Copyright 2026 Simone Contorno
# SPDX-License-Identifier: Apache-2.0
#
# Reproducible build + single-scenario run for the ProxMPC benchmark (mode b1).
# Builds the affected packages in ~/ros2_ws, sources the overlay, and runs one
# scenario x model through the standalone sim + metrics node, printing the
# per-run summary JSON. No git operations.
#
# Usage: ./init.sh [scenario] [model]
#   scenario: static_box | dynamic_circle | dynamic_line_forward | dynamic_line_backward
#   model:    unicycle | bicycle
set -euo pipefail

SCENARIO="${1:-static_box}"
MODEL="${2:-bicycle}"
WS="${ROS2_WS:-$HOME/ros2_ws}"
PKGS="prox_mpc_msgs prox_mpc_core prox_mpc_controller prox_mpc_demo prox_mpc_benchmark"

echo "== sourcing ROS 2 Jazzy =="
source /opt/ros/jazzy/setup.bash

echo "== building ($PKGS) in $WS =="
( cd "$WS" && colcon build --symlink-install --packages-select $PKGS )

echo "== sourcing overlay =="
source "$WS/install/setup.bash"

OUT="$(ros2 pkg prefix prox_mpc_benchmark)/../../src/prox_mpc/prox_mpc_benchmark/results"
mkdir -p "$OUT/runs"
SUMMARY="$OUT/runs/init__${SCENARIO}__${MODEL}.json"
rm -f "$SUMMARY"

echo "== running b1: scenario=$SCENARIO model=$MODEL =="
ros2 launch prox_mpc_benchmark benchmark.launch.py \
  scenario:="$SCENARIO" model:="$MODEL" summary_json:="$SUMMARY" &
LAUNCH_PID=$!

# The metrics node self-terminates a settle after the goal; bound the wait.
for _ in $(seq 1 90); do
  [ -f "$SUMMARY" ] && break
  sleep 1
done
sleep 1
kill -INT "$LAUNCH_PID" 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true

echo "== summary =="
if [ -f "$SUMMARY" ]; then
  cat "$SUMMARY"
else
  echo "no summary written (goal not reached within the bound)"
fi
