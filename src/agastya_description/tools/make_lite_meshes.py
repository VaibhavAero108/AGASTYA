# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Vaibhav (VaibhavAero108)
"""Create real-time "lite" visual meshes from the CAD-derived STLs (FreeCAD).

Run with Windows FreeCADCmd in safe mode (see CLAUDE_CODE_HANDOVER.md):

  & 'C:\\Program Files\\FreeCAD 1.1\\bin\\freecadcmd.exe' --safe-mode -u <headless.cfg> \\
      '\\\\wsl.localhost\\Ubuntu-24.04\\home\\vaibhav\\agastya_ws\\src\\agastya_description\\tools\\make_lite_meshes.py'

The geometry, units and coordinate frame are unchanged; only the triangle
count is reduced so Gazebo, its FPV camera and RViz can all render the same
model in real time under WSLg. The source meshes are kept untouched.
"""
import os

import Mesh

HERE = os.path.dirname(os.path.abspath(__file__))
MESHES = os.path.normpath(os.path.join(HERE, '..', 'meshes'))

JOBS = [
    # source, output, tolerance [mm], reduction fraction
    ('f450_frame_runtime.stl', 'f450_frame_lite.stl', 2.0, 0.92),
    ('a2212_motor.stl', 'a2212_motor_lite.stl', 0.3, 0.85),
]

for src, dst, tol, red in JOBS:
    mesh = Mesh.Mesh(os.path.join(MESHES, src))
    before = mesh.CountFacets
    mesh.decimate(tol, red)
    mesh.write(os.path.join(MESHES, dst))
    print(f'{src}: {before} -> {mesh.CountFacets} facets -> {dst}')
