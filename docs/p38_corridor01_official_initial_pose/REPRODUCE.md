# 复现与实际执行

研究工作区：`/home/jian/livox_ws/p10_corridor01_oosm_r1_ws`。
本地缓存：`.p38_official_pose.NVzksJ`，不提交XYZ/bag/PCD或二进制。

实际命令如下；不表示授权自动重跑。复现须选新的空缓存，不能覆盖现存证据。

```bash
cd /home/jian/livox_ws/p10_corridor01_oosm_r1_ws
source /opt/ros/noetic/setup.bash
set -o pipefail
/usr/bin/python3 docs/p38_corridor01_official_initial_pose/test_initial.py 2>&1 | tee .p38_official_pose.NVzksJ/test_results.txt
/usr/bin/python3 docs/p38_corridor01_official_initial_pose/prepare_initial.py --output .p38_official_pose.NVzksJ/input
mkdir .p38_official_pose.NVzksJ/geometry
env LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/bin/time -v -o .p38_official_pose.NVzksJ/geometry/resources.txt .p34_gt_frame.XDwg1L/build/p34_geometry .p38_official_pose.NVzksJ/input '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/map/corridor01.pcd' .p38_official_pose.NVzksJ/geometry --NN-only 2>&1 | tee .p38_official_pose.NVzksJ/geometry/stdout.txt
/usr/bin/python3 docs/p38_corridor01_official_initial_pose/collect_geometry.py --cache .p38_official_pose.NVzksJ
```

实际准备阶段因审核标签更正执行过两次；首次输出重命名保留为
`.p38_official_pose.NVzksJ/input_original_audit_label`，没有删除。
新旧 `hypotheses.f64` 和 `selected_raw.xyz` 均用 `cmp` 验证完全相同。
这不是两次几何实验；NN探针只执行过一次，统计两候选；NDT/ICP/回放均0。
最终collector只改严格门槛字段名，没有改统计或重跑查询。

Python语法检查针对prepare/test/collector执行；未重编译定位系统。
二进制复用P34，其源代码及PCL辅助由归档manifest记录SHA。
进程局部LD_PRELOAD规避既有MVS/libusb冲突，没有修改系统动态库。

源码复用：

- P36 `prepare_hypotheses.py`：矩阵certificate、SHA、既有输入助手入口。
- P33 `prepare.py`：真实路径/布局、整数时间、nearest/header顺序校验；不执行它的main。
- `p10_corridor01_prospective/pack_causal_slice.py`：缓存uint32 ns→实际float秒的标准转换。
- `p32_gt_frame/prepare.py`：只用于冻结后独立GT诊断的合法TUM/SLERP插值。
- P34 `geometry_probe.cpp` 与 `p23_budgeted_coupled/frozen_ndt_rules.hpp`：相同Source/Target预处理和NN。

归档 `geometry_comparison.csv` 包含所有全精度统计、bbox和传入矩阵。
`input_freeze.json` 包含实际原文YAML、原始输入SHA和fresh/inherited区分。
`initial_pose_projection.png` 同时显示原地图与正/反向投影，三面板统一物理尺度。

只读校验（不执行几何查询）：

```bash
/usr/bin/python3 docs/p38_corridor01_official_initial_pose/verify_archive.py
git diff --check
```
