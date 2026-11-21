# AGASTYA handover (updated by Claude Code, 2026-11-21)

## State

The project is **working**. `./run_simulation.sh` (or `ros2 launch agastya_sim simulation.launch.py`) opens Gazebo, RViz with the FPV view, and the control pad. Manual air ↔ water flight and landing on the pad are verified by `ros2 run agastya_sim acceptance_test`, which passes 11/11 both headless and with the GUI. See `PROJECT_STATUS.md`.

The previous blocker is resolved:
- **What it was:** runaway queued `ApplyLinkWrench` forces, ending in an ODE `aabbBound` assertion.
- **What replaced it:** the ROS force controller was removed in favour of a Gazebo system plugin. The plugin applies exactly one wrench per physics step, on simulation time, with no entity IDs.

## Runtime

- **Environment:** Windows → WSL `Ubuntu-24.04` (note: not `Ubuntu`). ROS 2 Jazzy; Gazebo Sim 8.15 through the vendor packages (`gz_sim_vendor`); WSLg for the GUI windows.
- **Workspace:** `/home/vaibhav/agastya_ws`, with packages `agastya_description`, `agastya_gz_plugins` (C++) and `agastya_sim` (Python).
- **Launch environment:** the launch file appends `GZ_SIM_RESOURCE_PATH` (for `package://` meshes) and `GZ_SIM_SYSTEM_PLUGIN_PATH` (for `libagastya_dynamics.so`) itself.

## Architecture

```text
control pad / teleop / tests ──/agastya/cmd_vel, /mode, /depth/setpoint──► ros_gz_bridge ──► Gazebo
                                                                                     │
  AgastyaDynamics (PreUpdate, 250 Hz sim time): buoyancy + drag + rotor thrust + flight/swim assist
                                                                                     │
RViz ◄── /tf (OdometryPublisher), /robot_description, /agastya/fpv/image, markers ◄──┘
```

Key files:
- **`src/agastya_description/urdf/agastya.urdf.xacro`:** the single vehicle description (parts, masses, sensors, plugin parameters).
- **`src/agastya_description/worlds/agastya_dual_domain.sdf`:** the pool world. The water box must match the plugin's `<water_min>`/`<water_max>`.
- **`src/agastya_gz_plugins/src/AgastyaDynamics.cc`:** all vehicle dynamics and control.
- **`src/agastya_sim/launch/simulation.launch.py`:** the single launch file.
- **`src/agastya_sim/agastya_sim/{control_pad,world_markers,acceptance_test}.py`:** the pad window, RViz markers and test mission.

## Gotchas learned

- The STEP-derived F450 STL is not in the robot frame. See the visual origin in the URDF and `HARDWARE_ARCHITECTURE.md`.
- Rendering the 537k-triangle F450 mesh in Gazebo, the FPV camera and RViz at once drops the real-time factor to 0.14 under WSLg. Use `*_lite.stl`, regenerated with `tools/cluster_simplify.py` and `tools/make_lite_meshes.py`. FreeCAD's `decimate` barely reduces the frame mesh.
- The world needs `gz-sim-imu-system`, or `/agastya/imu` publishes nothing.
- The control pad publishes only while keys are held, so other `/agastya/cmd_vel` publishers (tests, teleop) are not overridden.
- **`[WARN:COPY MODE]` in WSLg window titles:**
  - **Cause:** the WSLg shared memory failed (see `/mnt/wslg/weston.log`).
  - **Effect:** GL windows render black, and PrintWindow captures are black.
  - **Fix:** `wsl --shutdown`, then relaunch. Once it's healthy, `capture_windows`-style PrintWindow (flag 2) screenshots of the Gazebo and RViz windows work from Windows.
- On this laptop (Intel UHD + RTX 3050), Mesa defaults to llvmpipe. The launch file sets `GALLIUM_DRIVER=d3d12` and `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` (launch arg `gl`). RViz then reports GL 4.6.
- The Gazebo GUI `/gui/screenshot` service needs the `Screenshot` GUI plugin, which is now in the world's `<gui>` block.
- In scripts, do not `pkill -f "gz sim"` from a shell whose own command line contains that string, because it kills the shell. Use `pkill -f "gz[ ]sim"`.
- FreeCAD export still needs safe mode: `freecadcmd.exe --safe-mode -u <headless.cfg> <script>`. See `tools/export_cad_meshes.py` and `tools/make_lite_meshes.py`.

## Files that need special care

- Keep `f450_frame.stl` (the full source-derived mesh) and `f450_frame_runtime.stl`. Never use CAD meshes as collision geometry.
- If you move parts in the URDF, keep the plugin's `<rotor>` and `<buoyancy_*>` entries in step; they use the same xacro properties where possible.
- Keep `PROJECT_STATUS.md` current after each tested milestone, and rerun the acceptance test after any dynamics change.
