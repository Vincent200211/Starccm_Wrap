"""Small binary-STL helpers used by the STAR wrapper workflow."""
from __future__ import annotations

import struct
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np


def read_binary_stl(path: str | Path) -> np.ndarray:
    path = Path(path)
    with path.open("rb") as fh:
        fh.read(80)
        count_raw = fh.read(4)
        if len(count_raw) != 4:
            raise ValueError(f"{path}: missing binary STL triangle count")
        count = struct.unpack("<I", count_raw)[0]
        raw = fh.read(count * 50)
    if len(raw) != count * 50:
        raise ValueError(f"{path}: truncated STL, expected {count} triangles")
    payload = np.frombuffer(raw, dtype=np.uint8).reshape(count, 50)[:, 12:48]
    return payload.copy().view("<f4").reshape(count, 3, 3).astype(np.float64)


def write_binary_stl(path: str | Path, triangles: np.ndarray, header: str = "starccm_wrap") -> None:
    path = Path(path)
    tri = np.asarray(triangles, dtype=np.float64)
    if tri.ndim != 3 or tri.shape[1:] != (3, 3):
        raise ValueError(f"triangles must have shape (n,3,3), got {tri.shape}")
    normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    nonzero = lengths > 0.0
    normals[nonzero] /= lengths[nonzero, None]
    normals[~nonzero] = 0.0

    dtype = np.dtype([
        ("normal", "<f4", (3,)),
        ("vertices", "<f4", (3, 3)),
        ("attribute", "<u2"),
    ])
    records = np.zeros(len(tri), dtype=dtype)
    records["normal"] = normals
    records["vertices"] = tri
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as fh:
        fh.write(header.encode("ascii", errors="replace")[:80].ljust(80, b" "))
        fh.write(struct.pack("<I", len(records)))
        fh.write(records.tobytes())


def merge_binary_stls(paths: Sequence[str | Path], output: str | Path) -> int:
    if not paths:
        raise ValueError("no STL files to merge")
    parts = [read_binary_stl(path) for path in paths]
    merged = np.concatenate(parts, axis=0)
    write_binary_stl(output, merged, "starccm_wrap merged reference")
    return int(len(merged))


def bbox(triangles: Iterable[np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    lo = np.full(3, np.inf, dtype=np.float64)
    hi = np.full(3, -np.inf, dtype=np.float64)
    found = False
    for tri in triangles:
        a = np.asarray(tri, dtype=np.float64)
        if a.size == 0:
            continue
        pts = a.reshape(-1, 3)
        lo = np.minimum(lo, pts.min(axis=0))
        hi = np.maximum(hi, pts.max(axis=0))
        found = True
    if not found:
        raise ValueError("cannot compute bbox from empty STL set")
    return lo, hi


def write_box_stl(path: str | Path, box: Sequence[float], name: str = "box") -> None:
    if len(box) != 6:
        raise ValueError("box must be [xmin,xmax,ymin,ymax,zmin,zmax]")
    xmin, xmax, ymin, ymax, zmin, zmax = map(float, box)
    if not (xmin < xmax and ymin < ymax and zmin < zmax):
        raise ValueError(f"invalid box: {box}")
    v = np.array([
        [xmin, ymin, zmin], [xmax, ymin, zmin], [xmax, ymax, zmin], [xmin, ymax, zmin],
        [xmin, ymin, zmax], [xmax, ymin, zmax], [xmax, ymax, zmax], [xmin, ymax, zmax],
    ], dtype=np.float64)
    faces = [
        (0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
        (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
        (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7),
    ]
    tri = np.array([[v[a], v[b], v[c]] for a, b, c in faces], dtype=np.float64)
    write_binary_stl(path, tri, name)
