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

def write_quad_stl(path: str | Path, rectangles: Sequence[Sequence[float]], z: float,
                   normal_sign: float, name: str = "surface") -> None:
    """Write one or more axis-aligned XY rectangles at height z."""
    tris = []
    for rect in rectangles:
        xmin, xmax, ymin, ymax = map(float, rect)
        if not (xmin < xmax and ymin < ymax):
            continue
        a = [xmin, ymin, z]
        b = [xmax, ymin, z]
        c = [xmax, ymax, z]
        d = [xmin, ymax, z]
        if normal_sign >= 0.0:
            tris.extend(([a, b, c], [a, c, d]))
        else:
            tris.extend(([a, c, b], [a, d, c]))
    if not tris:
        raise ValueError(f"{path}: no non-empty rectangles")
    write_binary_stl(path, np.asarray(tris, dtype=np.float64), name)


def write_box_faces(directory: str | Path, box: Sequence[float]) -> dict[str, Path]:
    """Create five separate outer-wall surfaces; ground is generated separately."""
    if len(box) != 6:
        raise ValueError("box must be [xmin,xmax,ymin,ymax,zmin,zmax]")
    xmin, xmax, ymin, ymax, zmin, zmax = map(float, box)
    out_dir = Path(directory)
    out_dir.mkdir(parents=True, exist_ok=True)

    def plane_mesh(path: Path, vertices: Sequence[Sequence[float]], faces: Sequence[Sequence[int]], name: str) -> Path:
        v = np.asarray(vertices, dtype=np.float64)
        write_binary_stl(path, np.asarray([[v[i] for i in face] for face in faces], dtype=np.float64), name)
        return path

    files = {
        "DomainInlet": plane_mesh(
            out_dir / "DomainInlet.stl",
            [[xmin, ymin, zmin], [xmin, ymax, zmin], [xmin, ymax, zmax], [xmin, ymin, zmax]],
            [(0, 2, 1), (0, 3, 2)], "inlet"),
        "DomainOutlet": plane_mesh(
            out_dir / "DomainOutlet.stl",
            [[xmax, ymin, zmin], [xmax, ymin, zmax], [xmax, ymax, zmax], [xmax, ymax, zmin]],
            [(0, 2, 1), (0, 3, 2)], "outlet"),
        "DomainSideMin": plane_mesh(
            out_dir / "DomainSideMin.stl",
            [[xmin, ymin, zmin], [xmin, ymin, zmax], [xmax, ymin, zmax], [xmax, ymin, zmin]],
            [(0, 2, 1), (0, 3, 2)], "side_min"),
        "DomainSideMax": plane_mesh(
            out_dir / "DomainSideMax.stl",
            [[xmin, ymax, zmin], [xmax, ymax, zmin], [xmax, ymax, zmax], [xmin, ymax, zmax]],
            [(0, 2, 1), (0, 3, 2)], "side_max"),
        "DomainTop": plane_mesh(
            out_dir / "DomainTop.stl",
            [[xmin, ymin, zmax], [xmax, ymin, zmax], [xmax, ymax, zmax], [xmin, ymax, zmax]],
            [(0, 1, 2), (0, 2, 3)], "top"),
    }
    return files


def write_ground_plane(directory: str | Path, domain_box: Sequence[float],
                       name: str = "DomainGround") -> dict:
    """Write one planar ground patch covering the full wind-tunnel floor."""
    if len(domain_box) != 6:
        raise ValueError("domain_box must contain six values")
    xmin, xmax, ymin, ymax, zmin, _ = map(float, domain_box)
    out_dir = Path(directory)
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"{name}.stl"
    write_quad_stl(path, [(xmin, xmax, ymin, ymax)], zmin, -1.0, name)
    return {"name": name, "path": str(path)}


def write_ground_bands(directory: str | Path, domain_box: Sequence[float],
                       plinth_lo: Sequence[float], plinth_hi: Sequence[float],
                       bands: Sequence[dict]) -> list[dict]:
    """Partition the z=zmin ground plane into nested rectangular size bands.

    Each band is emitted as one planar STL part.  Adjacent bands share edges, so
    the union remains the complete wind-tunnel ground plane.
    """
    if len(domain_box) != 6:
        raise ValueError("domain_box must contain six values")
    xmin, xmax, ymin, ymax, zmin, _ = map(float, domain_box)
    cx0, cx1 = float(plinth_lo[0]), float(plinth_hi[0])
    cy0, cy1 = float(plinth_lo[1]), float(plinth_hi[1])
    out_dir = Path(directory)
    out_dir.mkdir(parents=True, exist_ok=True)

    rectangles = []
    prev = None
    for band in bands:
        expand = band.get("outer_expand_from_plinth_m")
        if expand is None:
            outer = (xmin, xmax, ymin, ymax)
        else:
            e = float(expand)
            outer = (max(xmin, cx0 - e), min(xmax, cx1 + e),
                     max(ymin, cy0 - e), min(ymax, cy1 + e))
        if prev is None:
            rects = [outer]
        else:
            px0, px1, py0, py1 = prev
            ox0, ox1, oy0, oy1 = outer
            if not (ox0 <= px0 <= px1 <= ox1 and oy0 <= py0 <= py1 <= oy1):
                raise ValueError(f"ground bands are not nested: {prev} -> {outer}")
            rects = []
            if px0 > ox0:
                rects.append((ox0, px0, oy0, oy1))
            if px1 < ox1:
                rects.append((px1, ox1, oy0, oy1))
            if py0 > oy0:
                rects.append((px0, px1, oy0, py0))
            if py1 < oy1:
                rects.append((px0, px1, py1, oy1))
        size_mm = float(band["size_m"]) * 1000.0
        name = f"GroundBand_{len(rectangles):02d}_{size_mm:g}mm"
        path = out_dir / f"{name}.stl"
        write_quad_stl(path, rects, zmin, -1.0, name)
        rectangles.append({
            "name": name,
            "path": str(path),
            "size_m": float(band["size_m"]),
            "outer": list(outer),
        })
        prev = outer

    if prev is None or not (prev[0] <= xmin + 1e-12 and prev[1] >= xmax - 1e-12 and
                            prev[2] <= ymin + 1e-12 and prev[3] >= ymax - 1e-12):
        raise ValueError("last ground band must cover the full domain box")
    return rectangles
