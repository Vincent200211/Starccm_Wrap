# DP18 STAR-CCM+ Surface Wrapper

本仓库把现有 ANSA 包面流程改写为 **STAR-CCM+ 2206 无 GUI 自动化流程**。运行时通过
`starccm+.bat -batch` 执行 Java 宏，不使用交互界面。

当前实现覆盖：

- 读取 12 个车体 STEP 文件和 4 个 marker STEP 文件
- CAD -> 算例帧、整车俯仰、固定落地件处理、全局 yaw
- 按 `setup.txt` 的 FRH/RRH/yaw 自动计算姿态
- 创建固定坐标的外场 box
- 创建随车体 bbox 自动外扩的 refinement box
- STAR Surface Wrapper
- 最大内部体积 `LARGEST_INTERNAL`
- 车体/plinth 表面 2 mm、风洞五壁 1 m
- 使用单一风洞地面 `DomainGround`，不再生成 ground band
- 曲率加密、接近值加密、局部失败自动回退
- 导出二进制 STL
- 按最近参考面重新打标为 14 个车体 patch
- 按风洞 box 六个平面补 `inlet/outlet/side_min/side_max/top/ground`
- 输出与现有 CFMesh 流程兼容的 OBJ

## 环境

- Simcenter STAR-CCM+ `2206.0001 (17.04.008)`
- Windows Python 3.10+
- Python 包：`numpy`、`scipy`

```powershell
python -m pip install -r requirements.txt
```

## 配置

主配置：

```text
config/wrap_config.json
```

默认读取相邻 ANSA 工程：

```text
../ANSA_WRAP_CFMESH/geometry
../ANSA_WRAP_CFMESH/preProcessor/setup.txt
```

如果移动到其他位置，修改：

```json
"paths": {
  "geometry_dir": "...",
  "setup_txt": "...",
  "output_dir": "output"
}
```

`star.home` 指向 STAR-CCM+ 安装根目录，例如：

```json
"home": "D:\\Program Files\\Siemens\\17.04.008\\STAR-CCM+17.04.008"
```

## 运行

只检查命令和生成运行目录：

```powershell
python run_wrap.py --dry-run
```

完整运行：

```powershell
python run_wrap.py
```

指定进程数：

```powershell
python run_wrap.py --np 8
```

分阶段运行：

```powershell
python run_wrap.py --stage prepare
python run_wrap.py --stage wrap
python run_wrap.py --stage label
```

已有 `prepared.sim` 时跳过第一段：

```powershell
python run_wrap.py --skip-prepare
```

已有 `wrapped.stl` 时只重新打标：

```powershell
python run_wrap.py --stage label
```

## 输出

```text
output/
├─ CAR_cavity_named.obj       # 最终 14+6 patch OBJ
├─ CAR_cavity_named_raw.obj   # 风洞六面打标前的中间 OBJ
├─ logs/
│  ├─ star_prepare.log
│  ├─ star_wrap.log
│  ├─ build_report.json
│  ├─ label_wall.json
│  ├─ label_domain.json
│  └─ run_summary.json
└─ work/
   ├─ prepared.sim
   ├─ wrapped.sim
   ├─ wrapped.stl
   ├─ parts/                  # 已变换的车体 part 参考 STL
   ├─ markers/                # Porous/Fan marker STL
   ├─ references/             # 按 patch 合并的参考 STL
   └─ domain_faces/           # 五个 1 m 风洞壁 + 单一地面 STL
```

最终 OBJ 的 patch 名称：

```text
inlet
outlet
side_min
side_max
top
ground
DP_FW
DP_RW
DP_RW_Upper
DP_Monocoque
DP_Winglets
DP_Diffuser
DP_T_Tray
DP_Radiator_Ducts
DP_Susps
DP_Uprights
DP_UT
DP_Tyre_Plinths
DP_Wheels_Front
DP_Wheels_Rear
```

## 实现结构

```text
run_wrap.py
  ├─ 生成 stage properties / 外场 box / refinement box
  ├─ 调用 STAR PrepareGeometry.java
  ├─ 合并 patch reference STL
  ├─ 调用 STAR RunWrapper.java
  └─ 调用本地 label_surface.py + label_domain.py

macros/PrepareGeometry.java
  └─ CAD 导入、姿态变换、逐 body STL 导出

macros/RunWrapper.java
  └─ Surface Wrapper、变尺寸控制、最大内部体积、STL 导出

lib/stl_io.py
  └─ 二进制 STL 读写、合并、bbox、box STL

lib/label_surface.py
  └─ 最近参考面打标、接缝投票、车轮前后切分

lib/label_domain.py
  └─ 风洞 box 六个 patch 的最终归属
```

## 和 ANSA 的关系

本流程复刻的是 ANSA 流水线的**功能和输出契约**：

```text
STP -> CAD 姿态 -> 计算域 -> 变尺寸包面 -> 最大内部体积
    -> 包面单壳 -> patch 打标 -> 命名 OBJ
```

STAR 与 ANSA 的包面算法不是同一个实现，因此不能假设三角形数量、局部去特征结果
完全相同。第一次正式跑完整车后应至少检查：

1. `wrapped.stl` 为闭合流形，且连通壳数量为当前期望的 3；
2. 14 个车体 patch 均非空；
3. 6 个风洞 patch 均非空；
4. `build_report.json` 中的变换后 bbox 与 ANSA 参考 bbox 在 6 mm 内；
5. `label_wall.json` 中最近参考面距离和模糊带比例合理；
6. 最终 OBJ 能通过 CFMesh `checkMesh`。

当前版本已完成几何变换与最大内部体积包面验证；接触防接触当前关闭。

## 已修复：STAR 2206 装配层级变换

此前 `TransformPartsOperation` 对装配层级中的 lifted leaf parts 批量执行 pitch 时，
部分零件会按自身局部坐标解释旋转轴心，造成米级错位。当前实现已改为：

1. STEP 导入后立即按文件名生成稳定 part 名称；
2. 将全部 leaf parts 重挂到根 `GeometryPartManager`，消除装配局部坐标系影响；
3. 使用 `GeometryPartManager.rotateParts/translateParts` 施加全局变换；
4. pitch 和 yaw 通过显式原点的局部 Cartesian 坐标系指定旋转轴心；
5. `GeometryPartManager` 的旋转角单位是弧度，脚本内统一用 `Math.toRadians` 转换。

变换顺序为：

```text
Rz(180°) + Tx(+1.53 m)
簧上件：pitch + dZ
全局：yaw
```

当前配置已启用 pitch 和 dZ：

```json
"apply_pitch": true,
"apply_dz": true
```

已验证变换后主要 patch 的 bbox 与
`../ANSA_WRAP_CFMESH/geometry/from_ansa/reference` 对照在约 0–5 mm 内；
当前 `W42 - Wheels.stp` 已更新，其轮组 bbox 与旧 ANSA 参考约差 9 mm。
