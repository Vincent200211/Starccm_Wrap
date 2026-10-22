# -*- coding: utf-8 -*-
"""
DP18 - 整车包面单壳打标签 (ANSA 包面输出 -> cfMesh 命名的 OBJ)
==============================================================
输入:  output/CAR.stl                ANSA 整车包面, 二进制 STL, 米
参考:  无关性/DP18_W14_*/constant/triSurface_0deg/*.obj   CAD 面, 同一坐标系, 米
输出:  output/car_named.obj         一个顶点表 + 每部件一个 `g DP_xxx` 面块
       logs/label_CAR.json          统计报告

做法:
  1. 焊接 STL 顶点 (二进制 STL 每个三角形自带三个顶点, 同一点按位相同);
  2. 参考面全部顶点建一棵 cKDTree, 每个三角形质心找最近参考点 -> 该参考面的 patch 名;
  3. 有公共边的相邻三角形做 N 轮多数组投票, 压平接缝;
  4. 最近距离 > --far (默认 10 mm) 的三角形仍分配给最近参考面; --far 仅用于统计;
  5. DP_Wheels 按 x = 0.765 m (半轴距) 切成 DP_Wheels_Front / DP_Wheels_Rear,
     与参考算例的 SPLIT_BY_PLANE 同一个平面 (CAD 帧下
     当前算例帧前轮在 x 小的一侧);
  6. 按本模块 CAR_PATCHES 顺序分段写 OBJ。

顶点表只写一次、各 patch 共用 -> 接缝处拓扑连通, cfMesh 拿到的是一个闭合壳。

坐标系: 包面和参考面**必须已经在同一个坐标系/姿态里**, 这里不做任何变换。两条路:

  (a) 参考用算例 preProcessor 的产物 (推荐)
      --ref-dir <算例>/geometry/triSurface      (算例帧)
      车体侧由 wrap_group.py 的 machine 块在"表面离散之前"施加姿态, 输出已经是算例帧;
      参考目录里已经切好 Wheels_Front.obj / Wheels_Rear.obj。

  (b) 参考用原始 CAD 快照 (两条都还是 CAD 帧)
      --ref-dir <无关性>/.../triSurface_0deg --aero-frame
      这时只做 CAD->算例帧的 180 deg 旋转, 不含姿态 (pitch/离地间隙), 只适合
      FRH = RRH = 35 那种 CAD 姿态的对比。

无论哪条, 脚本都会打印包面/参考的 bbox 并给出中心差; 差半个车身或中位距离 > 20 mm
就是两边没对上。
"""

import argparse
import json
import os
import struct
import sys
import time

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.environ.get("DP_LOG_DIR") or os.path.join(HERE, "logs")

# 参考面文件 -> 输出 patch 名。顺序即 OBJ 里 g 块的顺序,
# 车体壁面 patch 顺序；Wheels 会在后面拆成前/后两个 patch。
WALLS = (
    ("FW", "DP_FW"),
    ("RW", "DP_RW"),
    ("RW_Upper", "DP_RW_Upper"),
    ("Monocoque", "DP_Monocoque"),      # 本几何里 CH 没拆 Kick/Duct, 整块算 Monocoque
    ("Winglets", "DP_Winglets"),
    ("Diffuser", "DP_Diffuser"),
    ("T_Tray", "DP_T_Tray"),
    ("Radiator_Ducts", "DP_Radiator_Ducts"),
    ("Susps", "DP_Susps"),
    ("Uprights", "DP_Uprights"),
    ("UT", "DP_UT"),
    ("Tyre_Plinths", "DP_Tyre_Plinths"),
    ("Wheels", "DP_Wheels"),
)
WHEELS_FRONT = "DP_Wheels_Front"
WHEELS_REAR = "DP_Wheels_Rear"
DOMAIN_PENDING = "__DOMAIN_PENDING__"

XPITCH = 1.53           # 轴距 (m)
WHEELS_SPLIT_X = 0.765  # xpitch/2
WELD_TOL = 1.0e-7       # 顶点焊接量化 (m)
N_QUERY = 32            # 每次查询的邻居数, 用来估计"次优部件"距离


def log(msg=""):
    print("[%s] %s" % (time.strftime("%H:%M:%S"), msg), flush=True)


# ------------------------------------------------------------------ 读写
def read_binary_stl(path):
    """二进制 STL -> (n,3,3) float64。每三角形 50 字节: 法向 12 + 顶点 36 + 属性 2。"""
    with open(path, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        raw = f.read(n * 50)
    if len(raw) != n * 50:
        raise SystemExit("%s: 被截断, 声明 %d 个三角形但只读到 %d 字节"
                         % (path, n, len(raw)))
    tri = np.frombuffer(raw, dtype=np.uint8).reshape(n, 50)[:, 12:48]
    return tri.copy().view("<f4").reshape(n, 3, 3).astype(np.float64)


def read_obj_points(path):
    """只取 OBJ 的顶点坐标, 面不用。"""
    rows = []
    with open(path, "rb") as f:
        for line in f:
            if line[:2] == b"v ":
                p = line.split()
                rows.append((float(p[1]), float(p[2]), float(p[3])))
    if not rows:
        raise SystemExit("%s: 没有顶点" % path)
    return np.array(rows, dtype=np.float64)


def read_reference_points(path):
    """读取 STAR/ANSA 导出的参考表面: .stl 或 .obj。"""
    if path.lower().endswith(".stl"):
        return read_binary_stl(path).reshape(-1, 3)
    return read_obj_points(path)


def find_reference_file(ref_dir, stem):
    for ext in (".obj", ".stl"):
        path = os.path.join(ref_dir, stem + ext)
        if os.path.isfile(path):
            return path
    return None


def write_obj(path, V, F, groups, header):
    """顶点表写一次, 然后每个 patch 一个 g 块; 索引全局共享 -> 接缝连通。

    groups 是 [(patch 名, 三角形下标数组), ...], 已按要写出的顺序排好。
    """
    chunk = 200000
    with open(path, "w", newline="\n", encoding="utf-8") as f:
        for h in header:
            f.write("# %s\n" % h)
        for k in range(0, len(V), chunk):
            b = V[k:k + chunk]
            fmt = "\n".join(["v %.7f %.7f %.7f"] * len(b))
            f.write(fmt % tuple(b.ravel()) + "\n")
        for name, idx in groups:
            if not len(idx):
                continue
            f.write("g %s\n" % name)
            for k in range(0, len(idx), chunk):
                b = F[idx[k:k + chunk]]
                fmt = "\n".join(["f %d %d %d"] * len(b))
                f.write(fmt % tuple(b.ravel() + 1) + "\n")


# ------------------------------------------------------------------ 拓扑
def weld(tri):
    """STL 的 3n 个顶点焊成唯一表, 返回 (V, F)。"""
    raw = tri.reshape(-1, 3).copy()
    k = np.round(raw / WELD_TOL).astype(np.int64)
    order = np.lexsort((k[:, 2], k[:, 1], k[:, 0]))
    ks = k[order]
    first = np.empty(len(ks), dtype=bool)
    first[0] = True
    np.any(ks[1:] != ks[:-1], axis=1, out=first[1:])
    ids = np.cumsum(first, dtype=np.int64) - 1
    # inv 按原始顺序给出每个顶点的簇号: 散列目标必须是 inv, 不是 ids
    # (ids 是按 ks 排序后的顺序, 直接拿来散列会把点写进错的簇)。
    inv = np.empty(len(k), dtype=np.int64)
    inv[order] = ids
    V = np.empty((int(ids[-1]) + 1, 3), dtype=np.float64)
    V[inv] = raw
    return V, inv.reshape(-1, 3)


def edge_groups(F, nv):
    """按无向边分组, 返回 (开放边数, 非流形边数, 相邻三角形对 a, b)。"""
    e = np.concatenate((F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]))
    lo = np.minimum(e[:, 0], e[:, 1]).astype(np.int64)
    hi = np.maximum(e[:, 0], e[:, 1]).astype(np.int64)
    order = np.argsort(lo * nv + hi, kind="stable")
    ks = (lo * nv + hi)[order]
    bnd = np.flatnonzero(ks[1:] != ks[:-1]) + 1
    starts = np.concatenate(([0], bnd))
    sizes = np.diff(np.concatenate((starts, [len(ks)])))
    face_of = order % len(F)
    two = starts[sizes == 2]
    return int((sizes == 1).sum()), int((sizes > 2).sum()), face_of[two], face_of[two + 1]


def shell_counts(F, nv):
    """三角形连通壳数 + 每壳三角形数 (scipy 连通分量)。"""
    e = np.concatenate((F[:, [0, 1]], F[:, [1, 2]]))
    adj = coo_matrix((np.ones(len(e), dtype=np.int8), (e[:, 0], e[:, 1])),
                     shape=(nv, nv))
    n, lab = connected_components(adj.tocsr(), directed=False)
    return n, np.sort(np.bincount(lab[F[:, 0]], minlength=n))[::-1]


# ------------------------------------------------------------------ 主流程
def build_names(split_wheels):
    base = [w[1] for w in WALLS]
    names = (base[:-1] + [WHEELS_FRONT, WHEELS_REAR]) if split_wheels else base
    return names + [DOMAIN_PENDING]


def smooth_labels(Lf, A, B, K, rounds):
    """相邻三角形多数组投票 (自我一票 + 三个邻居各一票)。"""
    counts = np.zeros((len(Lf), K), dtype=np.int16)
    flat = counts.reshape(-1)
    rng = np.arange(len(Lf), dtype=np.int64)
    for it in range(rounds):
        before = Lf.copy()
        flat[:] = 0
        for a, b in ((A, B), (B, A)):
            uk, ck = np.unique(a.astype(np.int64) * K + Lf[b], return_counts=True)
            flat[uk] = ck.astype(np.int16)
        counts[rng, Lf] += 1
        Lf = counts.argmax(axis=1).astype(np.int32)
        n_ch = int((Lf != before).sum())
        log("  第 %d 轮: %d 个三角形改判 (%.3f%%)" % (it + 1, n_ch, 100.0 * n_ch / len(Lf)))
    return Lf


def main():
    ap = argparse.ArgumentParser(description="给 ANSA 整车包面单壳打部件标签")
    ap.add_argument("--stl", default=os.path.join(HERE, "output", "CAR.stl"))
    ap.add_argument("--ref-dir", default=os.path.abspath(os.path.join(
        HERE, "geometry", "from_ansa", "reference")))
    ap.add_argument("--out", default=None, help="默认 <stl 同目录>/car_named.obj")
    ap.add_argument("--report", default=None, help="默认 logs/label_<out名>.json")
    ap.add_argument("--smooth", type=int, default=3, help="接缝多数组投票轮数")
    ap.add_argument("--far", type=float, default=0.010,
                    help="远距面诊断阈值 (m); 超过阈值的车体面仍归入最近 patch")
    ap.add_argument("--wheels-split-x", type=float, default=WHEELS_SPLIT_X)
    ap.add_argument("--no-wheels-split", action="store_true")
    ap.add_argument("--aero-frame", action="store_true",
                    help="输出前做 CAD->aero 帧变换 x_a = 1.53 - x_c, y_a = -y_c")
    ap.add_argument("--ref-stride", type=int, default=1, help="参考顶点抽稀, 调试用")
    ap.add_argument("--box-mm", default=None,
                    help="optional domain box; its faces are excluded before car-reference checks")
    args = ap.parse_args()
    box = None
    if args.box_mm:
        vals = [float(v) for v in args.box_mm.replace(",", " ").split()]
        if len(vals) != 6:
            raise SystemExit("--box-mm needs 6 values")
        box = tuple(v / 1000.0 for v in vals)

    t_all = time.time()
    stl = os.path.abspath(args.stl)
    ref_dir = os.path.abspath(args.ref_dir)
    out = os.path.abspath(args.out) if args.out else os.path.join(
        os.path.dirname(stl), "car_named.obj")
    report_path = os.path.abspath(args.report) if args.report else os.path.join(
        LOG_DIR, "label_%s.json" % os.path.splitext(os.path.basename(out))[0])
    for d in (os.path.dirname(out), os.path.dirname(report_path)):
        if not os.path.isdir(d):
            os.makedirs(d)

    split_wheels = not args.no_wheels_split
    names = build_names(split_wheels)
    K = len(names)
    report = {"stl": stl, "ref_dir": ref_dir, "out": out, "patches_out": names,
              "smooth_rounds": args.smooth, "far_m": args.far,
              "wheels_split_x": args.wheels_split_x if split_wheels else None,
              "aero_frame": bool(args.aero_frame)}

    log("STL    : %s (%.1f MB)" % (stl, os.path.getsize(stl) / 1e6))
    log("参考面 : %s" % ref_dir)
    tri = read_binary_stl(stl)
    log("三角形 : %d" % len(tri))

    # ---- 1. 焊接 + 拓扑 ----
    t0 = time.time()
    V, F = weld(tri)
    del tri
    log("焊接   : %d 唯一顶点 (%.1f s)" % (len(V), time.time() - t0))
    report["vertices"] = int(len(V))
    report["triangles"] = int(len(F))
    report["bbox"] = [V.min(0).round(6).tolist(), V.max(0).round(6).tolist()]

    domain_mask = np.zeros(len(F), dtype=bool)
    if box is not None:
        tol = 0.002
        pts = V[F]
        for axis, value in ((0, box[0]), (0, box[1]), (1, box[2]),
                            (1, box[3]), (2, box[4]), (2, box[5])):
            domain_mask |= (np.abs(pts[:, :, axis] - value).max(axis=1) <= tol)
    report["domain_triangles"] = int(domain_mask.sum())

    cen = V[F].mean(axis=1)
    nrm = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
    area = 0.5 * np.linalg.norm(nrm, axis=1)
    report["area_m2"] = round(float(area.sum()), 4)
    report["signed_volume"] = round(float((cen * nrm).sum() / 6.0), 6)

    t0 = time.time()
    n_open, n_nonman, A, B = edge_groups(F, len(V))
    log("拓扑   : 开放边 %d, 非流形边 %d (%.1f s)" % (n_open, n_nonman, time.time() - t0))
    report["open_edges"] = int(n_open)
    report["nonmanifold_edges"] = int(n_nonman)

    t0 = time.time()
    n_sh, sh = shell_counts(F, len(V))
    log("壳数   : %d  %s (%.1f s)" % (n_sh, sh[:5].tolist(), time.time() - t0))
    report["shells"] = int(n_sh)
    report["shell_tris"] = [int(c) for c in sh[:10]]
    log("面积   : %.3f m^2, 有向体积 %+.6f m^3 (闭合壳应 > 0, 法向朝外)"
        % (area.sum(), report["signed_volume"]))

    # 车身最低区: 包面若在车底封了底板, 这里会露出大块面积
    lowf = (V[:, 2] < 0.010)[F].all(axis=1)
    if lowf.any():
        pts = V[F[lowf]].reshape(-1, 3)
        report["below_10mm"] = {"tris": int(lowf.sum()),
                                "area_m2": round(float(area[lowf].sum()), 4),
                                "bbox": [pts.min(0).round(4).tolist(),
                                         pts.max(0).round(4).tolist()]}
        log("z<10mm : %.1f mm^2, 范围 %s" % (1e3 * report["below_10mm"]["area_m2"],
                                             report["below_10mm"]["bbox"]))

    # ---- 2. 参考面 + 一棵全局 KD-tree ----
    pts, lab = [], []
    missing = []
    report["reference"] = {}
    ref_lo = np.full(3, np.inf)
    ref_hi = np.full(3, -np.inf)
    for ref_name, patch in WALLS:
        # 参考面如果已经按 SPLIT_BY_PLANE 切好 (算例 preProcessor 的产物: 一个目录里
        # 直接是 Wheels_Front.obj / Wheels_Rear.obj), 就按文件读, 不再按平面切。
        if patch == "DP_Wheels" and split_wheels:
            pf = find_reference_file(ref_dir, "Wheels_Front")
            pr = find_reference_file(ref_dir, "Wheels_Rear")
            if pf and pr:
                for path, lab_name in ((pf, WHEELS_FRONT), (pr, WHEELS_REAR)):
                    t0 = time.time()
                    rv = read_reference_points(path)
                    if args.ref_stride > 1:
                        rv = rv[::args.ref_stride]
                    pts.append(rv)
                    lab.append(np.full(len(rv), names.index(lab_name)))
                    ref_lo = np.minimum(ref_lo, rv.min(0))
                    ref_hi = np.maximum(ref_hi, rv.max(0))
                    report["reference"][os.path.splitext(os.path.basename(path))[0]] = int(len(rv))
                    log("  参考 %-16s %9d 点 (%.1f s)" % (
                        os.path.splitext(os.path.basename(path))[0], len(rv), time.time() - t0))
                continue
        p = find_reference_file(ref_dir, ref_name)
        if p is None:
            # 每个算例的部件几何不同: CAD 里没有拆出这个件时这里就会缺。
            # 不直接死掉, 记下来最后一起报 —— 对应的 DP_* 那个 patch 就不会有面。
            missing.append(ref_name)
            continue
        t0 = time.time()
        rv = read_reference_points(p)
        if args.ref_stride > 1:
            rv = rv[::args.ref_stride]
        if patch == "DP_Wheels" and split_wheels:
            front = rv[:, 0] < args.wheels_split_x
            if not front.any() or front.all():
                raise SystemExit("Wheels 在 x = %.3f 切不开 (x 范围 %.3f .. %.3f)"
                                 % (args.wheels_split_x, rv[:, 0].min(), rv[:, 0].max()))
            pts.append(rv[front])
            lab.append(np.full(int(front.sum()), names.index(WHEELS_FRONT)))
            pts.append(rv[~front])
            lab.append(np.full(int((~front).sum()), names.index(WHEELS_REAR)))
        else:
            pts.append(rv)
            lab.append(np.full(len(rv), names.index(patch)))
        ref_lo = np.minimum(ref_lo, rv.min(0))
        ref_hi = np.maximum(ref_hi, rv.max(0))
        report["reference"][ref_name] = int(len(rv))
        log("  参考 %-16s %9d 点 (%.1f s)" % (ref_name, len(rv), time.time() - t0))

    if missing:
        log("!! 参考面里缺这些件: %s" % ", ".join(missing))
        log("   -> 对应的 DP_* patch 不会有面。要么在 CAD 里把它们单独导出来")
        log("      (见 wrap_config.json 的 reference_export.map), 要么把它们从算例的")
        log("      patch 表里去掉 (orig0/ + fvOptions)。")
        report["reference_missing"] = missing
    if not pts:
        raise SystemExit("参考面一个都没读到 (ref-dir=%s)" % ref_dir)
    P = np.vstack(pts)
    L = np.concatenate(lab).astype(np.int32)
    del pts, lab
    log("参考点 : %d, 候选 patch %d" % (len(P), K))

    t0 = time.time()
    tree = cKDTree(P)
    log("KD-tree: 建好 (%.1f s)" % (time.time() - t0))
    del P

    # ---- 3. 每个三角形质心找最近参考面, 同时估计"次优部件"距离 ----
    t0 = time.time()
    dist = np.empty(len(F))
    gap = np.empty(len(F))
    Lf = np.empty(len(F), dtype=np.int32)
    step = 200000
    for k in range(0, len(F), step):
        b = slice(k, k + step)
        d, i = tree.query(cen[b], k=N_QUERY, workers=-1)
        Lq = L[i]
        best = Lq[:, 0]
        dist[b] = d[:, 0]
        Lf[b] = best
        other = Lq != best[:, None]
        has = other.any(axis=1)
        first_other = np.argmax(other, axis=1)
        gap[b] = np.where(has, d[np.arange(len(d)), first_other] - d[:, 0], np.inf)
    del tree, L, Lq
    log("查询   : 最近参考面 (%.1f s)" % (time.time() - t0))
    nearest = Lf.copy()

    # 帧/姿态一致性自检: 包面应该套在参考外面 (包面外扩 ~2 mm), 中心差一点无所谓,
    # 差半个车身就是坐标系或姿态没对上 (比如算例的 FRH/RRH 和 ANSA 侧 machine 不一致)。
    car_dist = dist[~domain_mask]
    if not len(car_dist):
        raise SystemExit("no non-domain triangles found for car labelling")
    car_face_idx = np.flatnonzero((~domain_mask) & (dist <= args.far))
    car_verts = np.unique(F[car_face_idx]) if len(car_face_idx) else np.flatnonzero(~domain_mask)
    w_lo, w_hi = V[car_verts].min(0), V[car_verts].max(0)
    log("车体面 bbox : %s .. %s" % (np.round(w_lo, 3).tolist(), np.round(w_hi, 3).tolist()))
    log("参考 bbox : %s .. %s" % (np.round(ref_lo, 3).tolist(), np.round(ref_hi, 3).tolist()))
    report["bbox_wrap"] = [w_lo.round(4).tolist(), w_hi.round(4).tolist()]
    report["bbox_ref"] = [ref_lo.round(4).tolist(), ref_hi.round(4).tolist()]
    dcen = np.linalg.norm((w_lo + w_hi) / 2.0 - (ref_lo + ref_hi) / 2.0)
    if dcen > 0.10:
        log("!! 包面和参考的中心差 %.3f m: 两边坐标系/姿态不是同一个 (ANSA 侧 machine 块"
            " 与算例 setup.txt 的 FRH/RRH 必须一致)" % dcen)
    report["bbox_center_delta_m"] = round(float(dcen), 4)

    pct = np.percentile(car_dist, [50, 90, 99, 99.9])
    report["distance_mm"] = {"p50": round(1e3 * float(pct[0]), 3),
                             "p90": round(1e3 * float(pct[1]), 3),
                             "p99": round(1e3 * float(pct[2]), 3),
                             "p99.9": round(1e3 * float(pct[3]), 3),
                             "max": round(1e3 * float(car_dist.max()), 3)}
    log("距离   : p50 %.2f p90 %.2f p99 %.2f p99.9 %.2f max %.2f mm"
        % (1e3 * pct[0], 1e3 * pct[1], 1e3 * pct[2], 1e3 * pct[3], 1e3 * car_dist.max()))
    if pct[0] > 0.020:
        log("!! 中位距离 %.1f mm 太大: 包面和参考多半不在同一个坐标系/姿态" % (1e3 * pct[0]))
    g = gap[np.isfinite(gap) & ~domain_mask]
    report["ambiguity"] = {"pct_gap_lt_0.5mm": round(100.0 * float((g < 5e-4).mean()), 3),
                           "pct_gap_lt_1mm": round(100.0 * float((g < 1e-3).mean()), 3),
                           "pct_gap_lt_2mm": round(100.0 * float((g < 2e-3).mean()), 3)}
    log("模糊带 : 次优差 <0.5mm %.2f%%, <1mm %.2f%%, <2mm %.2f%%"
        % (report["ambiguity"]["pct_gap_lt_0.5mm"], report["ambiguity"]["pct_gap_lt_1mm"],
           report["ambiguity"]["pct_gap_lt_2mm"]))
    del gap

    car_far = (dist > args.far) & ~domain_mask
    report["far_faces"] = int(car_far.sum())
    report["far_faces_pct"] = round(
        100.0 * float(car_far.sum() / max(1, (~domain_mask).sum())), 4)
    report["domain_triangles"] = int(domain_mask.sum())
    log("远距面 : >%.0f mm 的车体三角形 %d 个 (%.4f%%), 全部归入最近 patch"
        % (1e3 * args.far, int(car_far.sum()), report["far_faces_pct"]))
    Lf[domain_mask] = names.index(DOMAIN_PENDING)

    report["raw_patch_pct"] = {names[i]: round(100.0 * float((Lf == i).mean()), 3)
                               for i in range(K - 1) if (Lf == i).any()}
    log("打标   (未平滑): " + ", ".join(
        "%s=%.1f%%" % (n, p) for n, p in report["raw_patch_pct"].items()))

    # ---- 4. 接缝多数组投票 ----
    if args.smooth > 0 and len(A):
        log("平滑   : %d 轮" % args.smooth)
        Lf = smooth_labels(Lf, A, B, K, args.smooth)
    # 远距面必须使用最近参考面本身，不能被邻域平滑改到次近部件。
    Lf[car_far] = nearest[car_far]
    # 风洞面在 domain_label.py 中最终分成六个边界；这里只做临时占位。
    Lf[domain_mask] = names.index(DOMAIN_PENDING)
    del A, B, nearest

    # 注: Wheels 的前后切分在参考面一侧就做完了 (见 --wheels-split-x),
    # 与参考算例 SPLIT_BY_PLANE 切的是同一个平面, 这里不再对标签做二次切分。

    # ---- 6. 报告 + 导出 ----
    log("最终 patch 分布:")
    table, groups = [], []
    empty_walls = []
    for i in range(K):
        sel = Lf == i
        n = int(sel.sum())
        if not n:
            if names[i] != DOMAIN_PENDING:
                log("  %-20s 0 个三角形  <<< 该 patch 会缺失" % names[i])
                empty_walls.append(names[i])
            continue
        table.append({"patch": names[i], "tris": n,
                      "pct": round(100.0 * n / len(F), 3),
                      "area_m2": round(float(area[sel].sum()), 4),
                      "dist_mm_mean": round(1e3 * float(dist[sel].mean()), 3),
                      "dist_mm_p99": round(1e3 * float(np.percentile(dist[sel], 99)), 3)})
        groups.append((names[i], np.flatnonzero(sel)))
        log("  %-20s %9d  %5.2f%%  %7.4f m^2  d_mean %.2f mm  d_p99 %.2f mm"
            % (names[i], n, table[-1]["pct"], table[-1]["area_m2"],
               table[-1]["dist_mm_mean"], table[-1]["dist_mm_p99"]))
    report["patches"] = table
    report["missing_patches"] = empty_walls
    pending_count = int((Lf == names.index(DOMAIN_PENDING)).sum())
    wall_count = int(len(F) - pending_count)
    report["assigned_wall_faces"] = wall_count
    report["pending_domain_faces"] = pending_count
    report["unassigned_faces"] = 0
    report["frame_warning"] = bool(dcen > 0.10)

    if args.aero_frame:
        V = V * np.array([-1.0, -1.0, 1.0]) + np.array([XPITCH, 0.0, 0.0])
        report["bbox_aero"] = [V.min(0).round(6).tolist(), V.max(0).round(6).tolist()]
        log("CAD->aero: bbox -> %s .. %s" % (report["bbox_aero"][0], report["bbox_aero"][1]))

    header = ["DP18 car_named.obj - ANSA 整车包面单壳 + 最近参考面打标",
              "source: %s" % stl,
              "reference: %s" % ref_dir,
              "smooth=%d  far=%.3f m%s" % (args.smooth, args.far,
                                           "  aero-frame" if args.aero_frame else "")]
    t0 = time.time()
    write_obj(out, V, F, groups, header)
    log("导出   : %s (%.1f MB, %.1f s)" % (out, os.path.getsize(out) / 1e6, time.time() - t0))
    report["out_bytes"] = os.path.getsize(out)
    report["seconds"] = round(time.time() - t_all, 1)
    with open(report_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=1, ensure_ascii=False)
    log("报告   : %s   总耗时 %.1f s" % (report_path, report["seconds"]))
    fatal = []
    if missing:
        fatal.append("reference files missing: %s" % ", ".join(missing))
    if empty_walls:
        fatal.append("zero-face patches: %s" % ", ".join(empty_walls))
    if wall_count + pending_count != len(F):
        fatal.append("face assignment does not conserve the input triangle count")
    if pending_count != int(domain_mask.sum()):
        fatal.append("non-tunnel faces were left in DOMAIN_PENDING")
    if report["frame_warning"]:
        fatal.append("wrap/reference bbox center delta %.3f m" % dcen)
    if fatal:
        raise SystemExit("labelling validation failed: " + "; ".join(fatal))


if __name__ == "__main__":
    main()
