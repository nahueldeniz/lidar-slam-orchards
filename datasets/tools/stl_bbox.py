#!/usr/bin/env python3
"""Bounding box of an STL mesh (binary or ASCII), optionally after a URDF origin.

Usage: python -I stl_bbox.py <mesh.stl> [roll pitch yaw x y z]
Prints min/max of the vertices after applying R(rpy) and translation (URDF
convention: R = Rz(yaw) Ry(pitch) Rx(roll)). Used only to interpret the visual
mesh offset in the Clearpath Jackal URDF.
"""
import struct
import sys

import numpy as np
from scipy.spatial.transform import Rotation


def load(path):
    data = open(path, "rb").read()
    if data[:5] == b"solid" and b"facet" in data[:1000]:
        v = [list(map(float, ln.split()[1:4])) for ln in data.decode(errors="ignore").splitlines()
             if ln.strip().startswith("vertex")]
        return np.array(v)
    n = struct.unpack("<I", data[80:84])[0]
    rec = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return rec["v"].reshape(-1, 3).astype(float)


def main():
    v = load(sys.argv[1])
    if len(sys.argv) >= 8:
        r, p, y, x, yy, z = map(float, sys.argv[2:8])
        R = Rotation.from_euler("xyz", [r, p, y]).as_matrix()  # extrinsic xyz == URDF rpy
        v = v @ R.T + np.array([x, yy, z])
    print(f"vertices={len(v)} min={v.min(0).round(4)} max={v.max(0).round(4)} extent={(v.max(0)-v.min(0)).round(4)}")


if __name__ == "__main__":
    main()
