# AGASTYA hardware and simulation architecture

## One model, two viewers

`src/agastya_description/urdf/agastya.urdf.xacro` is the only vehicle description:

- RViz renders it through `robot_state_publisher`.
- Gazebo spawns the same robot description, so the two views match part for part.

sdformat merges the fixed-joint parts into one rigid body. Its mass, centre of gravity and inertia come from the per-part values below:

- **Total mass:** 1.623 kg.
- **Centre of gravity:** (−0.9, 0, +0.7) mm from the frame centre.

## Parts list (as modelled)

| Part | Selection | Size | Mass | Source of figures | Representation |
| --- | --- | --- | --- | --- | --- |
| Frame | DJI Flame Wheel F450 | 450 mm wheelbase | 282 g | DJI manual / retailer specs | **User STEP → FreeCAD mesh** |
| Motors ×4 | A2212 1000 KV | — | 47 g each | vendor listing (weights vary 47–70 g between sellers) | **User A2212 STL** |
| Propellers ×4 | 1045 (10 × 4.5 in) | 254 mm | ~10 g each | typical | two-blade bar; CCW orange, CW black |
| ESCs ×4 | 30 A, BLHeli_32 class (3D mode not required) | 45 × 24 × 9 mm | ~25 g each | typical | box on each arm |
| Flight controller | Holybro Pixhawk 6C Mini, Model A (current) | 54.3 × 39 × 17.5 mm | 42.4 g | Holybro technical specification | box in enclosure, IMU frame |
| Power module | Holybro PM07 | 68 × 50 × 8 mm | 36 g | PX4 docs | board in enclosure |
| GNSS | Holybro M10 | Ø50 × 14.4 mm | 32 g | Holybro / retailer | puck on mast |
| FPV camera | 19 mm micro FPV class (no specific model selected) | 19 mm cube | ~8 g | generic | front bulkhead; Gazebo RGB camera 640×480 @ 30 Hz, 90° HFOV |
| Battery | 3S 3300 mAh LiPo | 135 × 43 × 24 mm | ~275 g | typical | under the frame |
| Sealed enclosure | clear acrylic box (**design assumption**) | 140 × 100 × 70 mm | ~160 g | — | on the top plate |
| Trim ballast | lead plate (**design assumption**) | 100 × 50 × 7 mm | ~400 g | — | under the battery |
| Landing gear | skid type | 240 mm skids, 100 mm tall | ~60 g | typical | legs and skids |

The vendor's propulsion figure is about 885 g of pull for an A2212 1000 KV with a 1045 prop at 11.1 V. The model is derated to **8.0 N per motor**, which gives a thrust-to-weight ratio of about 2.0.

### Geometry facts established from the CAD

- **F450 STL:** it lies in the STEP's X–Z plane with +Y up, offset by (80.4, 140.4, −256.2) mm. The URDF visual origin `xyz=(-0.080449, -0.256228, -0.140425) rpy=(π/2, 0, 0)` re-centres it with +Z up. The arm top surface sits at z = +28.3 mm. Motor centres are at ±159 mm on both axes (450 mm / 2 / √2).
- **A2212 STL:** the axis is +Z, the base sits at z = 5.2 mm, and the shaft centre is at (14.5, 4.0) mm. The visual origin puts the motor base on the arm top.

## Meshes

| File | Triangles | Use |
| --- | --- | --- |
| `f450_frame.stl` (423 MB) | full | source export, kept unchanged |
| `f450_frame_runtime.stl` (27 MB) | 537 k | FreeCAD 98 % decimation of the export |
| `f450_frame_lite.stl` (0.95 MB) | 19 k | **used by URDF / RViz / Gazebo** (`tools/cluster_simplify.py`, 1.5 mm cells) |
| `a2212_motor.stl` | 36.5 k | source, kept unchanged |
| `a2212_motor_lite.stl` | 5.5 k | **used** (`tools/make_lite_meshes.py`, FreeCAD) |

Switching to the lite meshes raised the GUI real-time factor from 0.14 to about 1.0. With the 537k-triangle mesh, Gazebo, its FPV camera and RViz were each rendering it under WSLg.

## Dynamics model (`agastya_gz_plugins/AgastyaDynamics`)

The plugin runs inside Gazebo on simulation time, at every 4 ms physics step, and applies **one** summed wrench about the true centre of gravity. Nothing is queued over transport.

- **Rotors:**
  - per-rotor thrust with a 40 ms first-order motor lag
  - reaction torque from the PX4 quad-X spin directions
  - air limit of 8.0 N per rotor; the water limit of 3.0 N is an *assumption* (an air propeller underwater is torque-limited)
  - each rotor blends air ↔ water by its own hub depth over a 4 cm band
  - while the rotors are in mixed media, the allocator caps all four at the weakest rotor's limit
- **Buoyancy:** the enclosure volume is integrated over 36 cells, and solid-part volumes are lumped at their locations, for 1.495 L in total. Each piece is applied at its submerged centroid, which gives a correct righting moment. Fully submerged, the net is −1.26 N, so the vehicle is slightly negative.
- **Drag:** linear plus quadratic in the body frame, blended air → water by the submerged fraction, and clamped so a single step can never reverse the velocity.
- **Control:**
  - position-hold target driven by the sticks
  - velocity PI loop
  - geometric SO(3) attitude controller
  - tilt limits: 34° in air and water, 14° while crossing the surface
- **State machine:** the vehicle is LANDED (motors idle) or FLYING. Climb input arms it. Touchdown is detected from physical ground support: a descent command, thrust below 75 % of weight and no motion for 1 s. `off` cuts all thrust.

## Boundaries (not established by this simulation)

The simulation does **not** establish any of the following:
- waterproofing, pressure rating or sealing of the enclosure, ESCs, motors or connectors
- the underwater thrust of 1045 props on A2212 motors, or their motor and ESC thermal limits underwater
- added-mass effects, free-surface waves and splash loads
- validated drag coefficients

All of these need prototype measurements.

The controller is a simulation flight assist. It is not PX4 or ArduPilot firmware, and there is no SITL or hardware in the loop yet.
