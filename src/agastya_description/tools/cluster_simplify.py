#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Vaibhav (VaibhavAero108)
"""Vertex-clustering simplification for binary STL meshes (numpy only).

    python3 cluster_simplify.py <in.stl> <out.stl> <cell_size_in_mesh_units>

Vertices are snapped to the centroid of a uniform grid cell; triangles that
collapse are dropped and duplicates removed. Units/frame are unchanged. Used
to make the real-time visual of the CAD-derived F450 frame (cell 1.5 mm) so
Gazebo, its FPV camera and RViz all render at interactive rates under WSLg.
"""
import struct
import sys

import numpy as np


def load(path):
    with open(path, 'rb') as f:
        f.read(80)
        n = struct.unpack('<I', f.read(4))[0]
        data = np.frombuffer(f.read(n * 50), dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return data['v'].reshape(-1, 3, 3).astype(np.float64)


def save(path, tris):
    e1, e2 = tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]
    n = np.cross(e1, e2)
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    out = np.zeros(len(tris), dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    out['n'] = n
    out['v'] = tris.reshape(-1, 9)
    with open(path, 'wb') as f:
        f.write(b'AGASTYA cluster-simplified mesh'.ljust(80, b' '))
        f.write(struct.pack('<I', len(tris)))
        f.write(out.tobytes())


def simplify(tris, cell):
    verts = tris.reshape(-1, 3)
    keys = np.floor((verts - verts.min(0)) / cell).astype(np.int64)
    uniq, inverse = np.unique(keys, axis=0, return_inverse=True)
    inverse = inverse.reshape(-1)
    sums = np.zeros((len(uniq), 3))
    np.add.at(sums, inverse, verts)
    counts = np.bincount(inverse, minlength=len(uniq))[:, None]
    reps = sums / counts
    idx = inverse.reshape(-1, 3)
    keep = (idx[:, 0] != idx[:, 1]) & (idx[:, 1] != idx[:, 2]) & (idx[:, 0] != idx[:, 2])
    idx = idx[keep]
    canon = np.sort(idx, axis=1)
    _, first = np.unique(canon, axis=0, return_index=True)
    idx = idx[np.sort(first)]
    return reps[idx]


if __name__ == '__main__':
    src, dst, cell = sys.argv[1], sys.argv[2], float(sys.argv[3])
    tris = load(src)
    out = simplify(tris, cell)
    save(dst, out)
    print(f'{src}: {len(tris)} -> {len(out)} triangles (cell {cell}) -> {dst}')
