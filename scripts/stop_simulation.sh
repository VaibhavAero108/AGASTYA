#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Vaibhav (VaibhavAero108)
#
# Stop every process of a running AGASTYA session (Gazebo, RViz, bridge, pad).
# Patterns use [x] so this script never matches its own command line.
pkill -INT -f "ros2 launch agastya_[s]im" 2>/dev/null
sleep 3
for p in "gz[ ]sim" "rviz[2]" "parameter_[b]ridge" "robot_state_[p]ublisher" \
         "world_[m]arkers" "control_[p]ad" "acceptance_[t]est"; do
  pkill -9 -f "$p" 2>/dev/null
done
sleep 1
if pgrep -f "gz[ ]sim|rviz[2]|parameter_[b]ridge" >/dev/null; then
  echo "Some processes are still running:"; pgrep -fl "gz[ ]sim|rviz[2]|parameter_[b]ridge"
  exit 1
fi
echo "AGASTYA simulation stopped."
