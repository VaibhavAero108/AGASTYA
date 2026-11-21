#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Vaibhav (VaibhavAero108)
#
# Check that this machine/shell is ready to run the AGASTYA simulation.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ok=0
pass() { echo "  [ OK ] $*"; }
fail() { echo "  [FAIL] $*"; ok=1; }
warn() { echo "  [WARN] $*"; }

echo "AGASTYA environment check (${WS})"
[ -f /opt/ros/jazzy/setup.bash ] && pass "ROS 2 Jazzy installed" || fail "ROS 2 Jazzy not found in /opt/ros/jazzy"
[ "${ROS_DISTRO:-}" = "jazzy" ] && pass "ROS 2 environment sourced" || fail "ROS not sourced (run: source ~/.bashrc)"
command -v gz >/dev/null && pass "Gazebo: $(gz sim --version 2>/dev/null | head -1)" || fail "gz command not found"
for p in ros_gz_sim ros_gz_bridge rviz2 robot_state_publisher xacro; do
  ros2 pkg prefix "$p" >/dev/null 2>&1 && pass "package $p" || fail "missing ROS package $p (sudo apt install ros-jazzy-${p//_/-})"
done
python3 -c "import tkinter" 2>/dev/null && pass "python3-tk (control pad)" || fail "python3-tk missing (sudo apt install python3-tk)"
if [ -f "${WS}/install/setup.bash" ]; then
  pass "workspace built"
  ros2 pkg prefix agastya_sim >/dev/null 2>&1 && pass "workspace sourced" || warn "workspace built but not sourced (source ~/.bashrc)"
else
  warn "workspace not built yet (run: agastya_build or ./run_simulation.sh)"
fi
for m in f450_frame_lite.stl a2212_motor_lite.stl; do
  [ -f "${WS}/src/agastya_description/meshes/$m" ] && pass "mesh $m" || fail "mesh $m missing"
done
if [ -e /dev/dxg ]; then
  pass "WSL GPU device (/dev/dxg) present"
  if [ -f /mnt/wslg/weston.log ] && grep -q "rdp_allocate_shared_memory: Failed" /mnt/wslg/weston.log; then
    fail "WSLg is in COPY MODE (windows may be black): run 'wsl --shutdown' in Windows PowerShell, then reopen Ubuntu"
  else
    pass "WSLg shared memory OK"
  fi
fi
[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ] && pass "display available" || fail "no DISPLAY/WAYLAND_DISPLAY (GUI cannot open)"
echo
[ $ok -eq 0 ] && echo "Ready. Start with:  agastya" || echo "Fix the [FAIL] items above, then re-run: agastya_check"
exit $ok
