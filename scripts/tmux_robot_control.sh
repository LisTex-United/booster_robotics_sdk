#!/usr/bin/env bash

# Simple tmux layout:
# - Left pane : move_robot_node.py
# - Right pane: gripper_control_node.py
#
# Usage:
#   ./scripts/tmux_robot_control.sh <network_interface>
# Example:
#   ./scripts/tmux_robot_control.sh enp6s0

set -e

SESSION_NAME="booster_control"
WORKDIR="/home/master/Workspace/booster_robotics_sdk"

IFACE="${1:-}"

if [ -z "$IFACE" ]; then
  echo "Usage: $0 <network_interface>"
  echo "Example: $0 enp6s0"
  exit 1
fi

# Kill existing session if it exists (useful for oh-my-tmux)
tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true

# Start new detached session with first command in left pane
tmux new-session -d -s "$SESSION_NAME" -c "$WORKDIR" \
  "python3 scripts/move_robot_node.py --network_interface \"$IFACE\"" \; \
  split-window -h -c "$WORKDIR" \
  "python3 scripts/gripper_control_node.py \"$IFACE\"" \; \
  select-pane -L

# Attach to the session
tmux attach -t "$SESSION_NAME"

