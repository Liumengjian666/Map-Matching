# 实际复现流程

工作区 `/home/jian/livox_ws/p10_corridor01_oosm_r1_ws`，实际本地缓存
`.p36_world_planar.gDJzEB`。缓存 XYZ、二进制和大数据不提交。
下述命令说明实际执行过程，不表示授权自动重跑；复现须使用新的空输出目录，
不能覆盖当前已执行证据。

```bash
cd /home/jian/livox_ws/p10_corridor01_oosm_r1_ws
source /opt/ros/noetic/setup.bash
/usr/bin/python3 docs/p36_corridor01_world_planar_test/test_preparation.py
/usr/bin/python3 -m py_compile docs/p36_corridor01_world_planar_test/prepare_hypotheses.py docs/p36_corridor01_world_planar_test/collect_results.py docs/p36_corridor01_world_planar_test/test_preparation.py
/usr/bin/python3 docs/p36_corridor01_world_planar_test/prepare_hypotheses.py --output .p36_world_planar.gDJzEB/input --p33-cache .p33_frame_relation.a9l5g3
```

下述固定循环表示三次执行；本轮实际由外部协调并行启动同样三个调用，每帧只调用一次：

```bash
set -o pipefail
for frame in 67 100 150; do
    folder=".p36_world_planar.gDJzEB/input/$frame"
    mkdir "$folder/geometry"
    env LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/bin/time -v -o "$folder/geometry/resources.txt" .p34_gt_frame.XDwg1L/build/p34_geometry "$folder" '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/map/corridor01.pcd' "$folder/geometry" --NN-only 2>&1 | tee "$folder/geometry/stdout.txt"
done
/usr/bin/python3 docs/p36_corridor01_world_planar_test/collect_results.py --cache .p36_world_planar.gDJzEB
```

`LD_PRELOAD` 只针对该进程规避已有 MVS/libusb 动态库冲突，不修改系统动态库。
`--NN-only` 是必需开关；禁止使用工具的 NDT 模式。

只读归档校验（不执行 NN/配准）：

```bash
/usr/bin/python3 docs/p36_corridor01_world_planar_test/verify_archive.py
git diff --check
```

## 复用源码范围

- 本目录 `prepare_hypotheses.py`：读取实际 Y、冻结左乘及完整逆、重读三帧并做输入合同检查。
- P33 `prepare.py`：继承路径、点布局、标准转换入口与整数时间辅助，不执行它的 main。
- `src/dog_prior_map_localization/scripts/p32_gt_frame/prepare.py`：既有 TUM/SLERP 插值。
- `src/dog_prior_map_localization/scripts/p10_corridor01_prospective/pack_causal_slice.py`：
  `uint32 offset_ns` 缓存转换为实际 PointCloud2 `float32 time_s` 的唯一复用入口。
- `docs/corridor01_gt_frame_initialization/geometry_probe.cpp` 与
  `src/dog_prior_map_localization/scripts/p23_budgeted_coupled/frozen_ndt_rules.hpp`：
  同一已有 PCL 预处理/目标/NN；不修改、不重编译。
- `collect_results.py` 仅收集/校验/绘图，未实现第二套 NN。

归档 `input_freeze.json` 保存 Y、逆矩阵和 A/B/C0/C1 全部每帧矩阵；
`nn_comparison.csv` 是全精度统计和传入矩阵；`source_parity.csv` 是同输入逐字节证据。
依赖与产物的真实 SHA 记录在 `artifact_hashes.json`，不把未新计算的大文件 SHA 标成 fresh。
