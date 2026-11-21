# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Vaibhav (VaibhavAero108)
"""Export the user-supplied F450 STEP assembly and motor STL for Gazebo/URDF.

Run this with the installed Windows FreeCADCmd executable. The source paths are
intentionally explicit: they are the CAD assets supplied with this project.
"""
from pathlib import Path
import FreeCAD
import Import
import Mesh

ROOT = Path(r"\\wsl.localhost\Ubuntu-24.04\home\vaibhav\agastya_ws\src\agastya_description\meshes")
FRAME_STEP = Path(r"C:\Users\Admin\Downloads\F450\680d4b324366ea6baf6e8f56.step")
MOTOR_STL = Path(r"C:\Users\Admin\Downloads\a2212-1000-kv-bldc-brushless-dc-motor-1.snapshot.2\motor v7.stl")

ROOT.mkdir(parents=True, exist_ok=True)

frame_doc = FreeCAD.newDocument("agastya_f450")
Import.insert(str(FRAME_STEP), frame_doc.Name)
frame_doc.recompute()
shapes = [obj for obj in frame_doc.Objects if hasattr(obj, "Shape") and not obj.Shape.isNull()]
if not shapes:
    raise RuntimeError("The F450 STEP import did not contain exportable shapes")
Mesh.export(shapes, str(ROOT / "f450_frame.stl"))

# Keep the original tessellation for traceability and make a rendering copy for
# the WSLg Gazebo session. The source mesh is around 200 MB.
runtime_frame = Mesh.Mesh(str(ROOT / "f450_frame.stl"))
runtime_frame.decimate(0.5, 0.98)
runtime_frame.write(str(ROOT / "f450_frame_runtime.stl"))

motor_mesh = Mesh.Mesh(str(MOTOR_STL))
motor_mesh.write(str(ROOT / "a2212_motor.stl"))

print(f"Exported {len(shapes)} F450 STEP shapes, runtime visual mesh, and A2212 motor mesh to {ROOT}")
