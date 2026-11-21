# NOTICE

AGASTYA is Copyright (C) 2026 Vaibhav (VaibhavAero108). It is licensed under the GNU General Public License v3.0 or later; see `LICENSE`.

## Third-party material

The CAD-derived meshes in `src/agastya_description/meshes/` were generated from models obtained from third parties:

| Mesh | Derived from | Original source |
| --- | --- | --- |
| `f450_frame_runtime.stl`, `f450_frame_lite.stl` | `680d4b324366ea6baf6e8f56.step` (DJI Flame Wheel F450 frame model) | third-party CAD download |
| `a2212_motor.stl`, `a2212_motor_lite.stl` | `motor v7.stl` (A2212 1000 KV brushless motor model) | third-party CAD download |

Copyright in these models remains with their original authors, under the terms of the sites they were downloaded from. **The GPL licence of this project does not apply to them.** If you redistribute this repository, check that those terms allow it. If they don't, replace the meshes with your own geometry, using the tools in `src/agastya_description/tools/`.

"DJI", "Flame Wheel", "Pixhawk", "Holybro" and the other product names are trademarks of their respective owners. They are used only to identify the components that are modelled. This project is not affiliated with or endorsed by any of these companies.

Component dimensions and masses come from public datasheets and listings, as recorded in `HARDWARE_ARCHITECTURE.md`.
