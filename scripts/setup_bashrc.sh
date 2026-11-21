#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Vaibhav (VaibhavAero108)
#
# Make ~/.bashrc ready for the AGASTYA simulation (idempotent).
#
#   ./scripts/setup_bashrc.sh            install / update the AGASTYA block
#   ./scripts/setup_bashrc.sh --remove   remove it again
#
# The block is wrapped in markers so re-running replaces it instead of
# duplicating it. A timestamped backup of ~/.bashrc is written first.
set -euo pipefail

BASHRC="${HOME}/.bashrc"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BEGIN="# >>> AGASTYA simulation >>>"
END="# <<< AGASTYA simulation <<<"

touch "${BASHRC}"
cp "${BASHRC}" "${BASHRC}.bak.$(date +%Y%m%d_%H%M%S)"

# drop any previous AGASTYA block
tmp="$(mktemp)"
# (also drop a stand-alone "alias sl=..." for this project; it now lives in the block)
awk -v b="${BEGIN}" -v e="${END}" '$0==b{skip=1} !skip && $0 !~ /^alias sl=.*agastya_ws.*run_simulation\.sh/{print} $0==e{skip=0}' "${BASHRC}" > "${tmp}"
cat "${tmp}" > "${BASHRC}"
rm -f "${tmp}"

if [[ "${1:-}" == "--remove" ]]; then
  echo "AGASTYA block removed from ${BASHRC}"
  exit 0
fi

cat >> "${BASHRC}" <<EOF
${BEGIN}
# Added by ${WS}/scripts/setup_bashrc.sh - re-run that script to update, --remove to undo.
export AGASTYA_WS="${WS}"
export GZ_VERSION=harmonic
[ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash
[ -f "\${AGASTYA_WS}/install/setup.bash" ] && source "\${AGASTYA_WS}/install/setup.bash"
alias sl='cd "\${AGASTYA_WS}" && ./run_simulation.sh'                  # short launch alias
agastya()       { "\${AGASTYA_WS}/run_simulation.sh" "\$@"; }        # build + launch everything
agastya_stop()  { "\${AGASTYA_WS}/scripts/stop_simulation.sh"; }      # close Gazebo/RViz/pad
agastya_test()  { ros2 run agastya_sim acceptance_test; }             # automated mission (sim must be running)
agastya_build() { (cd "\${AGASTYA_WS}" && colcon build --symlink-install) && source "\${AGASTYA_WS}/install/setup.bash"; }
agastya_check() { "\${AGASTYA_WS}/scripts/check_environment.sh"; }   # verify ROS/Gazebo/GPU/WSLg
${END}
EOF

echo "AGASTYA block written to ${BASHRC} (backup saved next to it)."
echo "Open a new terminal or run:  source ~/.bashrc"
echo "Commands: sl | agastya | agastya_stop | agastya_test | agastya_build | agastya_check"
