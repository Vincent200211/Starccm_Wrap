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
- “第 N 个最大内部体积”，默认 `N=1`
- 车体表面 4 mm、远场 1 m、车体 refinement volume 30 mm 的变尺寸控制
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
   ├─ domain_box.stl
   └─ car_refine_box.stl
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

当前版本已经完成脚本落地和静态结构检查；整车质量和性能标定留待后续逐步测试。
