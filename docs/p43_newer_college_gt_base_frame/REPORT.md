# P43 Newer College GT Base-frame 验证

## 结论

在预先固定的 3 个 2020 `01_short_experiment` 扫描上，**Base 轴假设稳定且显著优于 optical 轴假设**。Base 假设的 NN median 为 `0.122–0.192 m`、P90 为 `0.404–0.455 m`、0.5 m overlap 为 `91.3–93.9%`；optical 假设 median 为 `8.392–19.108 m`、P90 为 `23.233–41.083 m`、0.5 m overlap 仅 `2.3–2.9%`。Base 解释也让变换后的 LiDAR +Z 与地图 +Z 保持约 `1.43–2.04°`，而 optical 解释产生约 `87.71–88.90°` 倾角。

正式分类：**`BASE_FRAME_GEOMETRICALLY_SUPPORTED`**。原始数据集论文与官方 URDF README 明确支持 GT 使用 Base frame；但是当前官网同时将 stereo GT 标为 `RS_C1`，而平台页把 `RS_C1` 称为 optical frame，并说明 Base 与其共点但采用机器人轴。此次未找到 `registered_poses.csv` 的 exporter/writer 源码，故保留“CSV 导出轴语义尚缺直接代码认证”的限定。几何匹配强烈支持 Base 解释，但不等同于 GT 格式源代码认证。

本轮没有运行任何 ICP/NDT/定位；P41 历史的一次真实 NDT 结果仅复核，不重跑。

## 冻结的帧与数据

三个时间点在查看几何结果前按 `t0`、`t0+100 s`、`t0+150 s` 规则冻结：

| 帧 | LiDAR PCD / 精确时间戳 | 相对 t0 | PCD SHA256 |
|---|---|---:|---|
| P41 existing first | `cloud_1583836591_182590976.pcd` / `1583836591182590976 ns` | 0 s | `7c2fc3a20ef266c2ac3196173c534a1acdb4436224c264bc8f2dc1b1909803a2` |
| nearest t0+100 s | `cloud_1583836691_180687360.pcd` / `1583836691180687360 ns` | 99.998096384 s | `07872fb6552630b50a56f6346a173f8e9b140ec32c3cb231ccbee1f41aa89c19` |
| nearest t0+150 s | `cloud_1583836741_180127744.pcd` / `1583836741180127744 ns` | 149.997536768 s | `4f2ec13a27fb787a360a0f971ace0958ff44299330afe8df67c5e6180106c336` |

每个 LiDAR 时间戳在 GT CSV 中均有逐纳秒精确行，没有插值或时间外推。每个源扫描 65,536 点；应用 P41 同一 source 合同（finite XYZ、range `[0.5,80] m`、z 不限、0.25 m VoxelGrid、确定性均匀 cap 1,400）后为 1,400 点。地图为 P41 的官方 5 cm PLY 无损 XYZ PCD（原 10,277,007 点），按 P41 配置的 map 0.15 m + target 0.15 m 两级 voxel 后为 3,142,751 点。A/B 使用相同 source 点和相同地图 KD-tree，唯一变量是 GT 姿态轴解释。

所有输入绝对路径和 SHA、GT 原始行、6 个完整变换矩阵及 PCD headers 见 [input_manifest.json](input_manifest.json)；三帧冻结规则见 [FRAME_SELECTION.md](FRAME_SELECTION.md)。

## 固定 A/B 结果

距离百分位沿用 P41：排序后 `floor(fraction × (N−1))`；overlap 表示最近邻距离不大于阈值的 source 点比例。

| 扫描 | 假设 | NN median (m) | NN P90 (m) | ≤0.5 m | ≤2 m | ≤8 m | LiDAR +Z 对地图 +Z 倾角 |
|---|---|---:|---:|---:|---:|---:|---:|
| t0 | A optical | 19.107738 | 41.083160 | 2.71% | 13.07% | 26.50% | 88.899° |
| t0 | B Base | 0.178342 | 0.454757 | 91.29% | 99.93% | 100.00% | 2.035° |
| t0+100 s | A optical | 8.669098 | 23.233490 | 2.93% | 11.93% | 47.07% | 87.709° |
| t0+100 s | B Base | 0.191877 | 0.421343 | 93.93% | 97.71% | 99.93% | 2.040° |
| t0+150 s | A optical | 8.392348 | 29.433235 | 2.29% | 13.93% | 48.36% | 88.779° |
| t0+150 s | B Base | 0.122049 | 0.403585 | 92.36% | 98.29% | 100.00% | 1.425° |

三帧均支持 B，并非只在 P41 首帧偶然靠近。按三帧等权均值，B median `0.164089 m`，A `12.056395 m`；B 0.5 m overlap 均值 `92.52%`，A `2.64%`。新增两帧仅按时间预注册，没有再挑选或扩展候选。

同尺度 XY/XZ/YZ 对照见 [geometry_projection.png](geometry_projection.png)；灰色为地图采样显示，彩色为相同预处理后的 A/B 投影。三个时间列共享同一坐标范围；没有通过图像配准或优化对齐。

## P41 历史单帧结果复核

P41 归档记录 A：median `19.298385 m`、P90 `40.775133 m`、0.5 m overlap `2.714%`，gate rejected、align 0；B：median `0.178773 m`、P90 `0.445447 m`、0.5 m overlap `92.5%`。B 的历史真实 NDT 为 converged、7 iterations、fitness `0.0206946358`、align `752.553341 ms`。本轮未再次调用 align。

本轮对 P41 首帧的固定 A/B 离线几何重算得到 `0.178342/0.454757 m` 与 `19.107738/41.083160 m`（median/P90）。与归档统计存在小的数值差异，故报告历史值与本轮配对值分开列出；结论和数量级一致，未声称跨运行逐点/逐浮点完全相同。

## 数学与数据检查

- Kalibr `cam0.T_cam_imu` 按 `T_C_I` 使用；Ouster `os1_lidar_to_os1_imu` 按 `T_I_L` 使用。反向 Ouster 字段核对为互逆，没有误用 `T_L_I`。
- GT CSV 四元数顺序为 qx/qy/qz/qw，使用前仅对接近单位范数的值归一化；三帧原范数偏差不超过 `2.81e-7`。
- Base/optical 共点仅使 `T_C_B` 平移为零，不省略两者坐标轴旋转。旋转合法，`T_C_B` 与 `T_B_C` 互逆误差为 0。
- 没有把 Base→optical 旋转重复应用；A/B 公式各自只含其定义所需的固定旋转。
- 点云都按同一 PCD XYZ 直接读取。本轮不做 IMU deskew，避免引入额外运动估计；P41 runtime config 中 `deskew_enable=false`。
- 变换后的 LiDAR +Z 在 B 下近似地图竖直，而 A 下近似水平，与几何距离差一致。

## 来源、代码和 Git

本轮新增仅为可复现诊断脚本与文档，没有改 CONTROL/NDT/EKF/标定/官方原始数据。数学链与源码检查见 [transform_chain.md](transform_chain.md) 和 [source_code_findings.md](source_code_findings.md)。

Git 起始 HEAD：`47c7da5d8c5c4f90513160423c8efd8d9041dfd6`，分支 `research/p9-r4-heldout-visual-evidence`。本次归档提交 SHA、最终 HEAD 与实际 origin HEAD 在执行交接中给出；未执行 push。P43 探针 binary SHA256：`b7f5f9cdb10b05dfec809d0956c85199f683a7f57d9b6ec0e152b14a28d8eb48`。运行成本：2.26 s wall、最大 RSS 744,256 kB；该值包含 PCL 加载约 3.14 M 目标点及 KD-tree 的诊断内存，不是定位系统部署 RSS。

## 正式判断与唯一 NEXT

- 官方资料：原论文和官方 URDF README 对 Base 姿态有直接支持；当前 GT 页面/平台命名对 RS_C1 optical 的表述与之存在语义张力。CSV exporter 级语义仍未闭合。
- 几何：Base 解释在三帧重复通过，Optical 解释在三帧明显失败。
- 本轮状态：`BASE_FRAME_GEOMETRICALLY_SUPPORTED`；不是完整轨迹定位精度验收，也不是连续运行/部署验收。
- 唯一 NEXT：由科研总控核查本报告、原始官方文献与数据后，决定是否批准以 Base 轴作为显式、可追溯的条件输入进入连续帧验证；在批准前不运行连续定位。
