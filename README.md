# AGASTYA – amphibious quadrotor simulation (ROS 2 Jazzy + Gazebo Harmonic)

AGASTYA is an F450-based quadrotor that flies, enters the water, swims at shallow depth, surfaces, takes off from the water and lands on a pad. One launch file starts everything:

- **Gazebo Sim 8**: a test pool (8 × 6 × 1.2 m) with an underwater gate and markers, a deck, a landing pad, and the vehicle.
- **RViz 2**: the CAD robot model, the pool and pad, a flight trail and the live **FPV camera** image.
- **AGASTYA control pad**: a small window that turns held keys into commands.

## Requirements

- Ubuntu 24.04, natively or in WSL 2 with WSLg.
- ROS 2 Jazzy (`ros-jazzy-desktop`).
- `ros-jazzy-ros-gz`, which brings Gazebo Harmonic / Gazebo Sim 8.
- `ros-jazzy-xacro` and `python3-tk`.

## Get the code and set up your shell (once)

```bash
git clone https://github.com/VaibhavAero108/AGASTYA.git ~/agastya_ws
cd ~/agastya_ws
./scripts/setup_bashrc.sh      # adds an AGASTYA block to ~/.bashrc (idempotent; backs it up first)
source ~/.bashrc
agastya_check                  # verifies ROS, Gazebo, packages, GPU and WSLg
```

The `.bashrc` block adds these commands:

| Command | What it does |
| --- | --- |
| `sl` or `agastya` | build if needed, then launch Gazebo, RViz (FPV) and the control pad |
| `agastya_stop` | close the whole simulation |
| `agastya_build` | `colcon build`, then re-source the workspace |
| `agastya_test` | run the automated acceptance mission (with the simulation running) |
| `agastya_check` | environment readiness check |

Run `./scripts/setup_bashrc.sh --remove` to remove the block again.

## Start

In an Ubuntu-24.04 (WSL) terminal, type `sl` (or `agastya`). Without the `.bashrc` setup, run:

```bash
cd ~/agastya_ws
./run_simulation.sh
```

Or do the steps by hand:

```bash
cd ~/agastya_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch agastya_sim simulation.launch.py
```

The Gazebo, RViz and control pad windows appear on Windows through WSLg. AGASTYA starts **on the landing pad, facing the pool**, with its motors idle.

## Fly it

Click the **AGASTYA control pad** window so it has keyboard focus, then hold keys:

| Key | Action |
| --- | --- |
| `R` / `↑` | climb (from the pad this takes off) |
| `F` / `↓` | descend (on the pad this lands; the motors stop automatically) |
| `W` / `S` | forward / backward |
| `A` / `D` | strafe left / right |
| `Q` / `E` / `←` / `→` | yaw left / right |
| `SPACE` | stop: hold the current position |
| `H` | dive to and hold 0.35 m depth (also works from the air) |
| `C` | cancel depth hold |
| `1` `2` `3` `0` | mode: auto / air / underwater / off |

When you release a key, the vehicle holds its position, altitude or depth. The sliders set the requested speed; the vehicle limits it to its air or water envelope.

**Suggested mission:**
1. Hold `R` to take off and climb to about 1 m.
2. Hold `W` to fly out over the pool.
3. Hold `F` to descend through the waterline. The vehicle switches to its underwater envelope as it submerges.
4. Press `H` for 0.35 m depth hold, then use `W` / `A` / `D` / `Q` / `E` to swim through the yellow-red gate.
5. Hold `R` to surface. It breaches level and climbs back into the air.
6. Fly back to the orange/black **H** pad and hold `F` until it lands.

### Modes

- **auto** (the default): the control profile blends from air to water with the measured submersion.
- **air** / **underwater**: force one control profile. The physics stays the same either way.
- **off**: all four motors stop immediately. In water the vehicle slowly sinks, because it is slightly heavier than the water it displaces.

### Other command sources

- **Topics:** anything that publishes `geometry_msgs/Twist` on `/agastya/cmd_vel` can fly it. The values are body-frame m/s, with `angular.z` in rad/s.
  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/agastya/cmd_vel
  ros2 topic pub --once /agastya/mode std_msgs/msg/String "{data: off}"
  ros2 topic pub --once /agastya/depth/setpoint std_msgs/msg/Float64 "{data: 0.35}"   # 0 cancels
  ros2 topic echo /agastya/status    # JSON: mode, state, domain, depth, rotor thrusts, ...
  ```
- **Launch options:** `rviz:=false`, `pad:=false`, `gui:=false` (headless server only), and the spawn pose `x:= y:= z:= yaw:=`.

## Troubleshooting (WSLg display)

- **Window titles start with `[WARN:COPY MODE]` and Gazebo/RViz are black or frozen.** This is a WSL problem: WSLg could not open its shared memory with Windows. `/mnt/wslg/weston.log` shows `rdp_allocate_shared_memory: Failed to open ... Input/output error`. Close the simulation, run `wsl --shutdown` in Windows PowerShell, and start again. If it keeps happening, run `wsl --update`.
- **GPU selection.** The launch file sets Mesa's D3D12 driver on the NVIDIA GPU (`gl:=nvidia`, the default). Without it, WSLg falls back to slow CPU rendering (llvmpipe). Other options are `gl:=intel`, `gl:=software` and `gl:=system`.
- **Window placement.** The control pad must have keyboard focus to fly. In RViz, scroll to zoom the follow camera. In Gazebo, right-click the vehicle and choose *Follow*.

## Automated acceptance test

With the simulation running (GUI or `gui:=false`):

```bash
ros2 run agastya_sim acceptance_test
```

The test flies the full mission and checks 11 criteria:
- idle on the pad for 10 s
- FPV image stream
- take-off and hover
- manual air response
- flight to the pool
- air → water transition
- 0.35 m depth hold
- underwater drive
- water → air breach while moving
- landing on the pad
- the `off` stop command

The latest result is 11/11, both headless and in the GUI session; see `PROJECT_STATUS.md`.

## Topics

| Topic | Type | Direction |
| --- | --- | --- |
| `/agastya/cmd_vel` | geometry_msgs/Twist | → vehicle |
| `/agastya/mode` | std_msgs/String (`auto`, `air`, `underwater`, `off`) | → vehicle |
| `/agastya/depth/setpoint` | std_msgs/Float64 (m below surface; 0 cancels) | → vehicle |
| `/agastya/status` | std_msgs/String (JSON) | ← vehicle |
| `/agastya/fpv/image`, `/agastya/fpv/camera_info` | sensor_msgs | ← 640×480, 30 Hz, 90° HFOV |
| `/agastya/imu` | sensor_msgs/Imu | ← 100 Hz (Pixhawk location) |
| `/agastya/odometry` | nav_msgs/Odometry | ← 50 Hz, frame `world` |
| `/tf`, `/tf_static`, `/robot_description` | | ← `world → base_link →` parts |

## Packages

- `agastya_description`: the single URDF/Xacro used by **both** RViz and Gazebo, the CAD meshes, the world and the mesh tools.
- `agastya_gz_plugins`: `AgastyaDynamics`, a Gazebo system plugin for rotor thrust, buoyancy, drag and flight/swim assist on simulation time.
- `agastya_sim`: the launch file, bridge configuration, RViz layout, control pad, world markers and acceptance test.

See `HARDWARE_ARCHITECTURE.md` for the parts list and model assumptions, and `PROJECT_STATUS.md` for verified results and limitations.

## Full-resolution F450 mesh

`src/agastya_description/meshes/f450_frame.stl` (about 404 MB) is **not in the repository**, because it exceeds GitHub's 100 MB file limit and the simulation does not use it. The simulation uses `f450_frame_lite.stl`, and `f450_frame_runtime.stl` is the 537k-triangle intermediate. To regenerate the full export, run `src/agastya_description/tools/export_cad_meshes.py` with FreeCAD and the F450 STEP file (see the instructions in that script).

## License

Copyright (C) 2026 Vaibhav (VaibhavAero108)

This program is free software: you can redistribute it and/or modify it under the terms of the **GNU General Public License** as published by the Free Software Foundation, either **version 3** of the License or (at your option) any later version. See [`LICENSE`](LICENSE) for the full text.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

Every source file carries an `SPDX-License-Identifier: GPL-3.0-or-later` header. Third-party CAD geometry is listed separately in [`NOTICE.md`](NOTICE.md).

This is a simulation. Nothing in it certifies flight readiness, waterproofing or the safety of any physical vehicle.
