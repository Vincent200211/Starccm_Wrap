# -*- coding: utf-8 -*-
"""Assign the six wind-tunnel patches on a labelled cavity OBJ.

Input is the OBJ written by label.py. A domain face is identified by all of its
vertices lying on one domain-box plane. This check is applied to every input group,
including the temporary domain-pending group: ground faces immediately around the plinth can otherwise inherit a
car/plinth label from the nearest-reference classifier and leave the ground patch open.
"""
import argparse
import json
import os
import sys

try:
    sys.stdout.reconfigure(errors="replace")
except Exception:
    pass


DOMAIN = ("inlet", "outlet", "side_min", "side_max", "top", "ground")
WALLS = (
    "DP_FW", "DP_RW", "DP_RW_Upper", "DP_Monocoque", "DP_Winglets",
    "DP_Diffuser", "DP_T_Tray", "DP_Radiator_Ducts", "DP_Susps",
    "DP_Uprights", "DP_UT", "DP_Tyre_Plinths", "DP_Wheels_Front",
    "DP_Wheels_Rear",
)


def parse_obj(path):
    vertices = []
    groups = {}
    current = "__DOMAIN_PENDING__"
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith("v "):
                p = line.split()
                vertices.append((float(p[1]), float(p[2]), float(p[3])))
            elif line.startswith("g "):
                current = line[2:].strip() or "__DOMAIN_PENDING__"
                groups.setdefault(current, [])
            elif line.startswith("f "):
                idx = [int(tok.split("/")[0]) - 1 for tok in line.split()[1:]]
                if len(idx) >= 3:
                    for i in range(1, len(idx) - 1):
                        groups.setdefault(current, []).append((idx[0], idx[i], idx[i + 1]))
    return vertices, groups


def centroid(verts, face):
    p = [verts[i] for i in face]
    return (sum(v[0] for v in p) / 3.0,
            sum(v[1] for v in p) / 3.0,
            sum(v[2] for v in p) / 3.0)


def domain_of(verts, face, box, tol):
    xmin, xmax, ymin, ymax, zmin, zmax = box
    p = [verts[i] for i in face]
    planes = [
        (max(abs(v[0] - xmin) for v in p), "inlet"),
        (max(abs(v[0] - xmax) for v in p), "outlet"),
        (max(abs(v[1] - ymin) for v in p), "side_min"),
        (max(abs(v[1] - ymax) for v in p), "side_max"),
        (max(abs(v[2] - zmin) for v in p), "ground"),
        (max(abs(v[2] - zmax) for v in p), "top"),
    ]
    distance, name = min(planes, key=lambda item: item[0])
    return name if distance <= tol else None


def main():
    ap = argparse.ArgumentParser(description="Assign wind-tunnel OBJ groups")
    ap.add_argument("--in", dest="src", required=True)
    ap.add_argument("--out", dest="dst", required=True)
    ap.add_argument("--box-mm", required=True,
                    help="xmin xmax ymin ymax zmin zmax in mm")
    ap.add_argument("--tol-mm", type=float, default=2.0)
    ap.add_argument("--report", default=None)
    args = ap.parse_args()

    box = tuple(float(v) / 1000.0 for v in args.box_mm.replace(",", " ").split())
    if len(box) != 6:
        raise SystemExit("--box-mm needs 6 values")
    tol = args.tol_mm / 1000.0
    vertices, groups = parse_obj(args.src)
    out_groups = {name: [] for name in groups}
    source_groups = {name: len(faces) for name, faces in groups.items() if faces}
    moved = {}
    for group, faces in groups.items():
        for face in faces:
            target = domain_of(vertices, face, box, tol)
            if target:
                out_groups.setdefault(target, []).append(face)
                moved[target] = moved.get(target, 0) + 1
            else:
                out_groups.setdefault(group, []).append(face)

    with open(args.dst, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("# cavity surface with car/plinth/domain groups\n")
        for v in vertices:
            fh.write("v %.9f %.9f %.9f\n" % v)
        for group in ("inlet", "outlet", "side_min", "side_max", "top", "ground"):
            faces = out_groups.get(group, [])
            if not faces:
                continue
            fh.write("g %s\n" % group)
            for face in faces:
                fh.write("f %d %d %d\n" % (face[0] + 1, face[1] + 1, face[2] + 1))
        for group, faces in out_groups.items():
            if group in ("inlet", "outlet", "side_min", "side_max", "top", "ground"):
                continue
            if not faces:
                continue
            fh.write("g %s\n" % group)
            for face in faces:
                fh.write("f %d %d %d\n" % (face[0] + 1, face[1] + 1, face[2] + 1))
    counts = {k: len(v) for k, v in out_groups.items() if v}
    source_total = sum(source_groups.values())
    output_total = sum(counts.values())
    missing_domain = [name for name in DOMAIN if not counts.get(name)]
    missing_walls = [name for name in WALLS if not counts.get(name)]
    unexpected_groups = sorted(set(counts) - set(DOMAIN) - set(WALLS))
    report = {
        "input": os.path.abspath(args.src),
        "output": os.path.abspath(args.dst),
        "box_m": list(box),
        "tol_mm": args.tol_mm,
        "source_groups": source_groups,
        "moved": moved,
        "groups": counts,
        "source_faces": source_total,
        "output_faces": output_total,
        "unassigned_faces": source_total - output_total,
        "unexpected_groups": unexpected_groups,
        "missing_domain": missing_domain,
        "missing_walls": missing_walls,
        "ok": (not missing_domain and not missing_walls
               and output_total == source_total and not unexpected_groups),
    }
    if args.report:
        with open(args.report, "w", encoding="utf-8") as fh:
            json.dump(report, fh, indent=1, ensure_ascii=False)
    print("domain moved:", json.dumps(moved, sort_keys=True))
    print("groups:", counts)
    if missing_domain:
        print("!! missing domain patches: %s" % ", ".join(missing_domain))
    if missing_walls:
        print("!! missing wall patches: %s" % ", ".join(missing_walls))
    if unexpected_groups:
        print("!! unexpected groups: %s" % ", ".join(unexpected_groups))
    if output_total != source_total:
        print("!! face count not conserved: input=%d output=%d" %
              (source_total, output_total))
    if not report["ok"]:
        raise SystemExit(3)


if __name__ == "__main__":
    main()
