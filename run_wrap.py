#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DP18 STAR-CCM+ 2206 headless surface-wrapper workflow.

Stages:
  1. PrepareGeometry.java: STEP import + machine transform + per-body STL export
  2. RunWrapper.java: domain box + variable-size Surface Wrapper + wrapped STL
  3. local Python labeler: 14 wall patches + 6 wind-tunnel patches -> OBJ

The STAR client is always launched with -batch. No GUI interaction is required.
"""
from __future__ import annotations

import argparse
import fnmatch
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np

from lib.stl_io import bbox, merge_binary_stls, read_binary_stl, write_box_stl

ROOT = Path(__file__).resolve().parent
DEFAULT_CONFIG = ROOT / "config" / "wrap_config.json"


def log(message: str) -> None:
    print(message, flush=True)


def resolve_path(value: str | Path) -> Path:
    path = Path(os.path.expandvars(str(value))).expanduser()
    if not path.is_absolute():
        path = ROOT / path
    return path.resolve()


def load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def read_setup_txt(path: Path) -> dict[str, float]:
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    result: dict[str, float] = {}
    for key, index in (
        ("FRH", 2), ("RRH", 3), ("roll", 4), ("yaw", 5),
        ("steer", 6), ("dZ", 7), ("AoA", 8),
    ):
        if len(lines) <= index or not lines[index].strip().startswith(key):
            raise RuntimeError(f"{path}: line {index + 1} is not {key}")
        result[key] = float(lines[index].split("=", 1)[1].strip())
    return result


def expand_patterns(directory: Path, patterns: Iterable[str]) -> list[Path]:
    found: dict[str, Path] = {}
    for pattern in patterns:
        for path in directory.glob(pattern):
            if path.is_file():
                found[str(path.resolve()).lower()] = path.resolve()
    return [found[key] for key in sorted(found)]


def machine_parameters(config: dict) -> dict[str, float | bool | list]:
    m = config["machine"]
    enabled = bool(m.get("enabled", True))
    if not enabled:
        return {
            "enabled": False,
            "xpitch": 0.0, "zbase": 0.0, "pitch_deg": 0.0, "zpitch": 0.0,
            "yaw_deg": 0.0, "yaw_pivot_x": 0.0, "yaw_pivot_y": 0.0,
            "pitch_sign": 1.0, "yaw_sign": 1.0,
        }
    setup = read_setup_txt(resolve_path(config["paths"]["setup_txt"]))
    if abs(setup["AoA"]) > 1.0e-9 or abs(setup["dZ"]) > 1.0e-9:
        raise RuntimeError("setup.txt AoA/dZ must be zero for this ANSA-compatible workflow")
    if abs(setup["roll"]) > 1.0e-9 or abs(setup["steer"]) > 1.0e-9:
        raise RuntimeError("setup.txt roll/steer must be zero for this workflow")
    xpitch = float(m["xpitch_m"])
    setup_step_mm = 1000.0
    pitch_deg = -math.degrees(math.atan(
        (setup["RRH"] - setup["FRH"]) / (xpitch * setup_step_mm)
    ))
    zpitch = setup["RRH"] / setup_step_mm - float(m["cad_ref_ride_height_m"])
    pivot = m.get("yaw_pivot_m", [0.0, 0.0])
    if len(pivot) != 2:
        raise RuntimeError("machine.yaw_pivot_m must contain two values")
    return {
        "enabled": True,
        "xpitch": xpitch,
        "zbase": float(m["wheel_radius_m"]),
        "pitch_deg": pitch_deg,
        "zpitch": zpitch,
        "yaw_deg": float(setup["yaw"]),
        "yaw_pivot_x": float(pivot[0]),
        "yaw_pivot_y": float(pivot[1]),
        "pitch_sign": float(m.get("pitch_sign", 1.0)),
        "yaw_sign": float(m.get("yaw_sign", 1.0)),
        "fixed_patterns": list(m.get("fixed_to_ground", [])),
    }


def prop_escape(value: object) -> str:
    text = str(value)
    return (
        text.replace("\\", "\\\\")
        .replace("\n", "\\n")
        .replace("\r", "\\r")
    )


def write_properties(path: Path, values: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as fh:
        for key in sorted(values):
            fh.write(f"{key}={prop_escape(values[key])}\n")


def star_executable(config: dict) -> Path:
    home = Path(os.path.expandvars(str(config["star"]["home"])))
    exe = home / "star" / "bin" / "starccm+.bat"
    if not exe.is_file():
        raise FileNotFoundError(f"STAR-CCM+ launcher not found: {exe}")
    return exe


def run_command(cmd: Sequence[str], log_path: Path, env: dict, cwd: Path, dry_run: bool) -> None:
    log(">>> " + " ".join(f'"{x}"' if " " in x else x for x in cmd))
    if dry_run:
        return
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8", errors="replace") as log_file:
        proc = subprocess.Popen(
            list(cmd), cwd=str(cwd), env=env, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace",
        )
        assert proc.stdout is not None
        for line in proc.stdout:
            print(line, end="")
            log_file.write(line)
        rc = proc.wait()
    if rc != 0:
        raise RuntimeError(f"Command failed with exit code {rc}; see {log_path}")


def match_any(value: str, patterns: Sequence[str] | None) -> bool:
    if not patterns:
        return False
    low = value.lower()
    return any(fnmatch.fnmatch(low, p.lower()) for p in patterns)


def rule_matches(rule: dict, source: Path, body: str) -> bool:
    sources = rule.get("sources", ["*"])
    if not match_any(source.name, sources):
        return False
    includes = rule.get("include")
    excludes = rule.get("exclude")
    if includes and not match_any(body, includes):
        return False
    if excludes and match_any(body, excludes):
        return False
    return True


def load_part_index(path: Path) -> list[dict[str, str]]:
    rows = []
    with path.open("r", encoding="utf-8") as fh:
        for line in fh:
            fields = line.rstrip("\n").split("\t")
            if len(fields) != 4:
                raise RuntimeError(f"Invalid parts index row in {path}: {line!r}")
            rows.append({
                "source": fields[0],
                "part": fields[1],
                "parent": fields[2],
                "stl": fields[3],
            })
    return rows


def build_references(config: dict, rows: list[dict[str, str]], ref_dir: Path) -> dict:
    ref_dir.mkdir(parents=True, exist_ok=True)
    grouped: dict[str, list[Path]] = {}
    unmatched: list[dict[str, str]] = []
    rules = config["label"]["reference_rules"]
    for row in rows:
        source = Path(row["source"])
        body = f"{row['part']} {row['parent']}"
        selected = None
        for rule in rules:
            if rule_matches(rule, source, body):
                selected = rule["patch"]
                break
        if selected is None:
            unmatched.append(row)
            continue
        grouped.setdefault(selected, []).append(Path(row["stl"]))

    required = [rule["patch"] for rule in rules]
    empty = [name for name in required if not grouped.get(name)]
    if empty:
        raise RuntimeError("Reference patch groups are empty: " + ", ".join(empty))

    counts = {}
    for patch, paths in grouped.items():
        out = ref_dir / f"{patch}.stl"
        counts[patch] = merge_binary_stls(paths, out)
    return {"patches": counts, "unmatched_parts": unmatched}


def car_bounds(rows: list[dict[str, str]]) -> tuple[np.ndarray, np.ndarray]:
    return bbox(read_binary_stl(row["stl"]) for row in rows)


def box_from_margin(lo: np.ndarray, hi: np.ndarray, margin: float) -> list[float]:
    return [
        float(lo[0] - margin), float(hi[0] + margin),
        float(lo[1] - margin), float(hi[1] + margin),
        float(lo[2] - margin), float(hi[2] + margin),
    ]


def write_stage_params(path: Path, values: dict[str, object]) -> None:
    write_properties(path, values)


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the STAR-CCM+ DP18 wrapper workflow")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--stage", choices=("all", "prepare", "wrap", "label"), default="all")
    parser.add_argument("--skip-prepare", action="store_true")
    parser.add_argument("--skip-wrap", action="store_true")
    parser.add_argument("--np", type=int, default=None)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    config = load_config(Path(args.config).resolve())
    star = star_executable(config)
    np_ranks = args.np if args.np is not None else int(config["star"].get("np", 1))

    output_dir = resolve_path(config["paths"]["output_dir"])
    work_dir = output_dir / "work"
    parts_dir = work_dir / "parts"
    markers_dir = work_dir / "markers"
    ref_dir = work_dir / "references"
    logs_dir = output_dir / "logs"
    for path in (output_dir, work_dir, parts_dir, markers_dir, ref_dir, logs_dir):
        path.mkdir(parents=True, exist_ok=True)

    machine = machine_parameters(config)
    geometry_dir = resolve_path(config["paths"]["geometry_dir"])
    car_files = expand_patterns(geometry_dir, config["car"]["files"])
    if not car_files:
        raise RuntimeError(f"No car STEP files matched in {geometry_dir}")
    marker_names = []
    marker_files = []
    for marker in config.get("markers", []):
        found = expand_patterns(geometry_dir, [marker["pattern"]])
        if len(found) != 1:
            raise RuntimeError(f"Marker pattern {marker['pattern']} matched {len(found)} files")
        marker_names.append(marker["name"])
        marker_files.append(found[0])

    stage_dir = Path(tempfile.gettempdir()) / "starccm_wrap" / f"run_{int(time.time())}"
    stage_dir.mkdir(parents=True, exist_ok=True)
    macro_prepare = stage_dir / "PrepareGeometry.java"
    macro_wrap = stage_dir / "RunWrapper.java"
    shutil.copy2(ROOT / "macros" / "PrepareGeometry.java", macro_prepare)
    shutil.copy2(ROOT / "macros" / "RunWrapper.java", macro_wrap)

    prepared_sim = work_dir / "prepared.sim"
    wrap_sim = work_dir / "wrapped.sim"
    wrap_stl = work_dir / "wrapped.stl"
    domain_box_stl = work_dir / "domain_box.stl"
    refine_box_stl = work_dir / "car_refine_box.stl"
    parts_index = work_dir / "parts_index.tsv"
    markers_index = work_dir / "markers_index.tsv"

    prepare_params = stage_dir / "prepare.properties"
    write_stage_params(prepare_params, {
        "prepared_sim": prepared_sim,
        "parts_dir": parts_dir,
        "parts_index": parts_index,
        "markers_dir": markers_dir,
        "markers_index": markers_index,
        "car_files": "|".join(str(p) for p in car_files),
        "marker_names": "|".join(marker_names),
        "marker_files": "|".join(str(p) for p in marker_files),
        "tessellation_density": int(config["star"].get("tessellation_density", 1)),
        "sharp_edge_angle_deg": float(config["star"]["sharp_edge_angle_deg"]),
        "machine_enabled": bool(machine["enabled"]),
        "machine_xpitch": float(machine["xpitch"]),
        "machine_zbase": float(machine["zbase"]),
        "machine_pitch_deg": float(machine["pitch_deg"]),
        "machine_zpitch": float(machine["zpitch"]),
        "machine_yaw_deg": float(machine["yaw_deg"]),
        "machine_yaw_pivot_x": float(machine["yaw_pivot_x"]),
        "machine_yaw_pivot_y": float(machine["yaw_pivot_y"]),
        "machine_pitch_sign": float(machine["pitch_sign"]),
        "machine_yaw_sign": float(machine["yaw_sign"]),
        "fixed_patterns": "|".join(machine.get("fixed_patterns", [])),
    })

    stage = args.stage
    do_prepare = stage in ("all", "prepare") and not args.skip_prepare
    do_wrap = stage in ("all", "wrap") and not args.skip_wrap
    do_label = stage in ("all", "label") or (do_wrap and stage == "all")

    if do_prepare:
        env = os.environ.copy()
        env["STAR_WRAP_PARAMS"] = str(prepare_params)
        run_command(
            [str(star), "-new", "-batch", str(macro_prepare), "-np", str(np_ranks)],
            logs_dir / "star_prepare.log", env, stage_dir, args.dry_run,
        )

    if not args.dry_run and do_wrap:
        if not prepared_sim.is_file() or not parts_index.is_file():
            raise FileNotFoundError("prepare stage outputs are missing; run without --skip-prepare")
        rows = load_part_index(parts_index)
        ref_report = build_references(config, rows, ref_dir)
        lo, hi = car_bounds(rows)
        margin = float(config["wrap"]["refine_margin_m"])
        refine_box = box_from_margin(lo, hi, margin)
        domain_box = [float(v) for v in config["domain"]["box_m"]]
        write_box_stl(domain_box_stl, domain_box, "DomainBox")
        write_box_stl(refine_box_stl, refine_box, "CarRefineBox")
        build_report = {
            "machine": machine,
            "car_files": [str(p) for p in car_files],
            "marker_files": [str(p) for p in marker_files],
            "car_bbox_m": [lo.tolist(), hi.tolist()],
            "domain_box_m": domain_box,
            "refine_box_m": refine_box,
            "references": ref_report,
        }
        (logs_dir / "build_report.json").write_text(
            json.dumps(build_report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )

        wrap_params = stage_dir / "wrap.properties"
        write_stage_params(wrap_params, {
            "prepared_sim": prepared_sim,
            "wrap_sim": wrap_sim,
            "wrap_stl": wrap_stl,
            "domain_box_stl": domain_box_stl,
            "refine_box_stl": refine_box_stl,
            "wrap_name": config["wrap"]["name"],
            "base_size_m": float(config["wrap"]["base_size_m"]),
            "minimum_surface_size_m": float(config["wrap"]["minimum_surface_size_m"]),
            "car_target_size_m": float(config["wrap"]["car_target_size_m"]),
            "refine_volume_size_m": float(config["wrap"]["refine_volume_size_m"]),
            "nth_largest": int(config["wrap"]["nth_largest"]),
            "smallest_relevant_volume_m3": float(config["wrap"]["smallest_relevant_volume_m3"]),
            "curvature_refinement": bool(config["wrap"]["curvature_refinement"]),
            "proximity_refinement": bool(config["wrap"]["proximity_refinement"]),
            "gap_closure": bool(config["wrap"]["gap_closure"]),
            "run_global_if_local_fails": bool(config["wrap"]["run_global_if_local_fails"]),
            "stl_binary": bool(config["wrap"].get("stl_binary", True)),
        })
        env = os.environ.copy()
        env["STAR_WRAP_PARAMS"] = str(wrap_params)
        run_command(
            [str(star), str(prepared_sim), "-batch", str(macro_wrap), "-np", str(np_ranks)],
            logs_dir / "star_wrap.log", env, stage_dir, False,
        )

    if do_label and not args.dry_run:
        if not wrap_stl.is_file():
            raise FileNotFoundError(f"wrapped STL missing: {wrap_stl}")
        domain_box = [float(v) for v in config["domain"]["box_m"]]
        box_mm = [v * 1000.0 for v in domain_box]
        raw_obj = output_dir / "CAR_cavity_named_raw.obj"
        final_obj = output_dir / "CAR_cavity_named.obj"
        wall_report = logs_dir / "label_wall.json"
        domain_report = logs_dir / "label_domain.json"
        py = sys.executable
        run_command([
            py, str(ROOT / "lib" / "label_surface.py"),
            "--stl", str(wrap_stl),
            "--ref-dir", str(ref_dir),
            "--out", str(raw_obj),
            "--report", str(wall_report),
            "--smooth", str(int(config["label"]["smooth_rounds"])),
            "--far", str(float(config["label"]["far_m"])),
            "--wheels-split-x", str(float(config["label"]["wheels_split_x_m"])),
            "--box-mm", " ".join(str(v) for v in box_mm),
        ], logs_dir / "label_wall.log", os.environ.copy(), ROOT, False)
        run_command([
            py, str(ROOT / "lib" / "label_domain.py"),
            "--in", str(raw_obj),
            "--out", str(final_obj),
            "--box-mm", " ".join(str(v) for v in box_mm),
            "--tol-mm", str(float(config["label"]["domain_tolerance_mm"])),
            "--report", str(domain_report),
        ], logs_dir / "label_domain.log", os.environ.copy(), ROOT, False)
        summary = {
            "ok": True,
            "prepared_sim": str(prepared_sim),
            "wrap_sim": str(wrap_sim),
            "wrapped_stl": str(wrap_stl),
            "final_obj": str(final_obj),
            "wall_report": str(wall_report),
            "domain_report": str(domain_report),
        }
        (logs_dir / "run_summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
        log(f"Surface workflow complete: {final_obj}")

    if args.dry_run:
        log("Dry run complete; no STAR or label commands were executed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
