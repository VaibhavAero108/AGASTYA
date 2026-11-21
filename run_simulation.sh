#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Vaibhav (VaibhavAero108)
# Build (if needed) and start the complete AGASTYA simulation:
# Gazebo + vehicle, ROS bridge, RViz with FPV, and the control pad window.
#
#   ./run_simulation.sh            build + launch
#   ./run_simulation.sh --deps     also install missing system deps with rosdep (needs sudo)
#   ./run_simulation.sh gui:=false (any extra args are passed to ros2 launch)
set -eo pipefail
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
if [[ "${1:-}" == "--deps" ]]; then
  shift
  rosdep install --from-paths src --ignore-src -r -y
fi
colcon build --symlink-install
source install/setup.bash
exec ros2 launch agastya_sim simulation.launch.py "$@"
