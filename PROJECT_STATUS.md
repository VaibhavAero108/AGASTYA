# AGASTYA project status

Updated: 2026-11-22 (fibre tether, link model, propeller RPM, splashdown and cavitation added)

## Summary

The simulation is **operational out of the box**. A single launch starts the following, and the vehicle can be flown by hand in air, into the water, underwater, back out of the water and onto the landing pad:
- Gazebo with the pool world
- the CAD-based vehicle
- the ROS bridge
- RViz with the FPV image
- the control pad

## Verified (2026-11-22): fibre tether / link / propeller update

Headless (`gui:=false`, isolated `GZ_PARTITION`) result: **15/15**.

| # | Check | Result |
| --- | --- | --- |
| 1–4 | Pad idle, FPV, take-off/hover, manual air control | pass (drift 0.000 m; 2.83 m forward in 2 s; 85° yaw) |
| 5 | Air → water **while moving forward** | peak IMU 15.9 m/s² |
| 5b | Splashdown prop spin-down | props entered at ≤ 1,065 RPM, **0 prop strikes** |
| 6 | 0.35 m depth hold | error < 0.01 m |
| 7 | Underwater drive | 1.56 m in 8 s |
| 7b | Fibre link underwater | link OK, video 100 %, tether 7.6/25 m |
| 7c | Cavitation | min σ = 144 (onset below 1); 89 RPM, 2.5 A maximum |
| 7d | Radio link underwater | lost (video 0 %), failsafe surfaced, regained at antenna depth 0.07 m |
| 8 | Water → air breach while moving | tilt 9.6° |
| 9–10 | Land on pad; `off` | (5.50, 0.00) LANDED; 0.000 N |

Regression case: flying to the pool and diving immediately used to give **2 prop strikes at 2,405 RPM**. After the speed-aware spin-down and the active-braking model, the props enter at 642–1,079 RPM with 0 strikes. The FPV frames were checked: clean over fibre at 0.41 m depth, snow in radio mode.

## Verified (2026-11-21): initial takeover

`ros2 run agastya_sim acceptance_test` results:

| # | Check | Headless (`gui:=false`) | GUI session (Gazebo + RViz + pad) |
| --- | --- | --- | --- |
| 1 | Idle 10 s on the pad | drift 0.000 m | drift 0.000 m |
| 2 | FPV image stream | 30 Hz sim | 30 Hz sim, 29.6 Hz wall, RViz subscribed |
| 3 | Take-off + 4 s hover | drift 0.010 m | drift 0.010 m |
| 4 | Manual air control | 2.9 m forward in 2 s, 85° yaw in 1.5 s | same |
| 5 | Air → water transition | peak IMU 11.9 m/s² (no impulse) | 11.8 m/s² |
| 6 | 0.35 m depth hold | error < 0.01 m | error < 0.01 m |
| 7 | Underwater drive | 1.3 m in 8 s, depth error 0.01 m | same |
| 8 | Water → air breach while moving | max tilt 14.6° | 15.5° |
| 9 | Fly back and land on the pad | (5.49, 0.00), LANDED | (5.58, 0.00), LANDED |
| 10 | `off` stops all thrust | 0.000 N | 0.000 N |

Overall: **11/11 in both**, including the flight to the pool centre.

Further checks:
- **Real-time factor:** about 1.0 with the full GUI stack (it was 0.14 before the lite meshes).
- **Control pad:** driven with real Tk key events. R takes off, W flies forward, 1/3 switch modes, and F descends into the pool.
- **FPV:** frames were inspected in air (pool, gate and props visible) and underwater (gate, markers and lane lines).
- **Build:** `colcon build --symlink-install` builds all three packages with no compiler warnings. `check_urdf` and `gz sdf -k` pass.

## Changes in this takeover

1. **Control path replaced.** The ROS-side wrench publisher (`controller.py`) was the cause of the runaway force accumulation and the ODE `aabbBound` crash. It is replaced by the `AgastyaDynamics` Gazebo system plugin, which applies one wrench per physics step on simulation time. This also removes the fixed entity-ID (18) workaround.
2. **CAD placement fixed.** The F450 mesh had been drawn on its side and offset by about 0.27 m. It is now centred and upright, with the A2212 motors on the arm tips.
3. **Real parts added.** The 1045 props, ESCs, Pixhawk 6C Mini, PM07, M10 GPS, FPV camera, 3S battery, sealed enclosure, ballast and landing gear are all included, with their masses feeding the lumped inertia.
4. **Single URDF for RViz and Gazebo.** The separate, divergent `model.sdf` is removed.
5. **New world.** It has a pool with walls, a floor and lane lines, a deck, an H landing pad, an underwater gate, markers, and the IMU system (the IMU previously published nothing).
6. **One launch file.** It starts Gazebo, the bridge (including `/tf`, camera_info and status), RViz, the control pad and the world markers.
7. **Lite visual meshes.** The originals are kept.
8. **Automated acceptance mission** added.
9. **(2026-11-22) Fibre tether and link model.** Radio (absorbed by water, with failsafe surfacing) and fibre (tether weight, drag and breaking load) links; FPV degradation through `link_sim`; the tether drawn in Gazebo and RViz; the shore reel; the media converter and Pi Zero 2 W in the URDF.
10. **(2026-11-22) Propeller physics.** RPM and current estimates, prop-strike detection, the automatic splashdown (slow approach, speed-aware spin-down, faster spin-down for ESC braking), and the cavitation number. The control pad shows the link, RPM, current, strikes and cavitation; `L` switches the link.

The pre-takeover sources are backed up at `~/agastya_backups/agastya_src_pre_takeover_*.tgz`.

## Known limitations

- The water thrust (3 N per rotor), drag coefficients, enclosure volume and ballast are **assumptions**; see `HARDWARE_ARCHITECTURE.md`.
- There is no added mass, wave, splash or current model. Water is a static volume, x −4…4, y −3…3, z −1.2…0, which must match the plugin parameters in the URDF.
- Underwater horizontal speed is low (about 0.2 m/s). Propulsion comes only from tilting the four rotors, and the vehicle is slightly negatively buoyant.
- Underwater, the FPV image is a clear-water rendering, with no turbidity or colour absorption.
- With motors `off` in water, the vehicle sinks slowly by design (net −1.76 N with the tether hardware).
- The propeller coefficients (C<sub>t</sub>, C<sub>q</sub>) are static textbook values, and the 1,500 RPM prop-strike threshold is an assumption. There is no ventilation model (air drawn down near the surface).
- The tether is a force model plus a drawn line. It is not a simulated flexible cable, so it can't snag on the gate or wrap on the props. The radio absorption figures are fresh-water estimates.
- The splashdown guard is automatic: hovering low over the water with the hull wet and not climbing lets the vehicle settle in. To lift off from the water, hold `R`.
- Surfacing directly under an obstacle (for example the gate crossbar) blocks the vehicle, as it would in reality.

## Next steps

- PX4 SITL integration (Pixhawk 6C Mini target) to replace the simulation flight assist.
- Thrust-stand data for the A2212 + 1045 in air and water, to replace the assumed thrust curves.
- Measured enclosure volume and mass, and a trimmed ballast.
- Battery and power model (PM07 sensing).
- An optional underwater fog or colour shader for the FPV camera.
