# P64-R1 GenZ-ICP 用户机器狗先验地图定位

## 最终判定

**INCONCLUSIVE**

本轮没有进行正式 A/B 定位实验，也没有把公开 Long-Corridor、SubT-MRS 或
HILTI 数据作为实验数据。原因是用户数据的“建图数据”和“测试数据”没有形成
可验证的独立坐标合同：最新完整测试 bag 与同日全量 FAST-LIVO2 建图 PCD
同时存在，但没有独立建图 bag、扫描时间排除清单或建图输出来源证明可以排除
测试数据泄漏。继续运行会使定位结果不可归因于 GenZ-ICP。

因此本轮不能回答“GenZ-ICP 是否改善长走廊漂移”，也没有修改稳定机器狗工程、
没有接入 UKF/ESKF、没有上传原始 bag 或 PCD。

## 1. 运行身份与资产核查

### 最新候选用户数据

| 项目 | 实际资产 |
|---|---|
| 最新完整候选 bag | `/home/jian/rosbag/dog/mapping_20261031_160829/rect_record_2026-10-31-16-08-29.bag` |
| bag 时长 | 1121.128269 s |
| bag 消息 | 251991 |
| LiDAR | `/livox/lidar`, 11212 条 |
| IMU | `/livox/imu`, 224225 条 |
| 最新同日 PCD | `/home/jian/rosbag/dog/mapping_20261031_160829/pcd/mapping_20261031_160829_all_downsampled_points.pcd` |
| 同日 PCD SHA-256 | `a0d4d78172065151e6854a676d88d410cebd85c0e02607e4058002fd743c1c5d` |
| 历史定位使用 PCD | `/home/jian/rosbag/dog/rect_record_20261029_fastlivo2/rerun_20261102_182309/pcd/rerun_20261102_182309_all_downsampled_points.pcd` |
| 历史定位 PCD SHA-256 | `cd3fe063af0f8f3848055dd8c5fbe049ca6d909aa73b70ab0fd85699a45a4721` |

历史候选 loop2 bag 仍为：
`/home/jian/rosbag/loop2/raw_restore_20260914_stage0/loop2_raw.bag`，
时长 1090 s、LiDAR 10905 条、IMU 218101 条。其对应 PCD 的存在已确认，
但本轮没有找到足以证明建图/测试严格分离的完整来源合同，因此未将其偷偷
替换为正式输入。

### 当前 NDT 合同

当前机器狗 NDT 配置是
`/home/jian/dog_visual_loc_ws/src/dog_prior_map_localization/config/dog_prior_map_localization_ndt.yaml`
及 `dog_prior_map_localization_split.launch`：

* `map_frame=camera_init`
* `base_frame=livox_frame`
* 输入 `/livox/lidar`、`/livox/imu`
* `scan_reference_time=mid`
* `deskew_enable=false`
* source voxel `0.25 m`，target voxel `0.15 m`
* NDT 初值使用本地 IMU gyro rotation prior，不使用外部 EKF pose feedback
* PCD 没有 ROS frame 元数据，frame 由配置指定
* bag 不含 `/tf` 话题

FAST-LIVO2 用户配置
`/home/jian/FAST-LIVO2_project/FAST-LIVO2/config/ours_config.yaml` 中记录：

* `extrinsic_R=I`
* `extrinsic_T=[-0.011,-0.02329,0.04412]`

但本轮没有找到新的、独立且可验证的 LiDAR-IMU 标定原始文件。不能把该配置
未经复核地宣称为最新有效标定。

### 泄漏风险证据

`mapping_20261031_160829` 目录保存了与同名 bag 相关的 FAST-LIVO2 建图
资产、PCD 和运行记录；已有记录显示该日期 bag 被用于建图/回放链路。当前
没有以下任一证据：

1. 建图只使用测试 bag 之外的独立 bag；
2. 建图扫描时间与测试区间完全不重叠；
3. 建图 PCD 的输入 bag SHA 与测试 bag 不同；
4. 独立地图坐标系和 LiDAR/IMU 外参封存。

所以正式实验停止。

## 2. 官方 GenZ-ICP 复现

官方仓库已独立克隆到：
`/home/jian/livox_ws/genz_icp_prior_map_ws/src_genz_icp`

官方 HEAD：

```text
674ccce2705df012a29a99d3b8ef0e6c1cbc0129
```

环境核查：

| 依赖 | 结果 |
|---|---|
| Ubuntu | 20.04 系统工具链 |
| ROS | Noetic |
| C++ | GCC 9.4.0 |
| CMake | 3.16.3 |
| Eigen | 可用 |
| Sophus | 官方 CMake 依赖可用 |
| TBB | 系统库可用 |
| PCL | 1.10 系统库可用 |

构建命令：

```bash
cd /home/jian/livox_ws/genz_icp_prior_map_ws/src_genz_icp/cpp/genz_icp
cmake -B build -DCMAKE_CXX_STANDARD=20 -DUSE_CCACHE=OFF
cmake --build build -j2
```

结果：官方 C++ `core`、`metrics`、`pipeline` 静态库构建成功。

官方 Python 测试未执行成功，因为系统 Python 没有 `pytest`：

```text
/usr/bin/python3: No module named pytest
```

这不是算法测试通过；本轮没有安装新包，也没有把缺少 pytest 伪装成通过。

## 3. 源码审计摘要

### 配准入口

官方 `Registration::RegisterFrame()` 先把 source 变换到初值坐标，再循环：

1. `VoxelHashMap::GetCorrespondences()`
2. `BuildLinearSystem()`
3. 求解 6-DoF 增量并更新 `Sophus::SE3d`
4. 直到增量满足收敛条件或达到最大迭代次数

### 对应点与几何类型

`VoxelHashMap` 以固定 voxel hash 查询 27 个相邻 voxel。每个 query 使用最近
邻点；邻域点数至少为 5 时计算协方差并判断平面性：

\[
\lambda_3 / (\lambda_1+\lambda_2+\lambda_3) < \text{planarity\_threshold}
\]

平面点进入 point-to-plane；非平面点进入 point-to-point。算法不是把全图
写入当前帧，而是在标准 pipeline 中维护局部地图。

### 自适应权重与鲁棒核

`BuildLinearSystem()` 使用：

* point-to-plane 残差：`(source-target) dot normal`
* point-to-point 残差：三维 source-target
* 结构/平面权重：`alpha`
* 非结构/点权重：`1-alpha`
* 鲁棒权重：

\[
w(r^2)=\frac{k^2}{(k+r^2)^2}
\]

代码中的 `kernel` 和 `alpha` 共同作用于 `J^T W J` 与 `J^T W r`。

### 标准 pipeline 与本轮目标的差异

官方 `pipeline::GenZICP::RegisterFrame()` 会在每帧结束调用：

```cpp
local_map_.Update(frame_downsample, new_pose);
```

这适用于里程计局部地图，不满足本轮“固定 PCD 先验地图、当前帧不得写回”
的要求。若数据合同闭合，适配时必须直接构建一次冻结 `VoxelHashMap`，
禁止调用该 `Update()`，并显式记录 registration source、initial guess、
correspondence 统计及优化诊断。当前没有实施该适配，避免在泄漏数据上产生
不可解释的结果。

## 4. 未完成项目及原因

以下项目被有意停止，而不是遗漏：

* 固定用户 PCD 的 GenZ-ICP adapter：数据合同未闭合；
* A：原 NDT + ESKF 完整回放：无法定义无泄漏评价集；
* B：GenZ-ICP + 原 ESKF 完整回放：同上；
* 长走廊 RMSE/P95/Max：无独立 GT，且地图泄漏未排除；
* 轨迹折返/绕圈比较：不能把建图轨迹或 NDT 轨迹当真值；
* RViz A/B 截图：会制造未经验证的视觉结论；
* 正式接入、fallback launch、在线准入：不应在实验前置条件失败时实现。

## 5. Git 交付

本报告所在分支：

```text
research/p64-r1-genz-icp-prior-map
```

本轮只提交本报告，不复制第三方 GenZ-ICP 源码到研究仓库，官方源码保留在
独立工作区并带有其 MIT License。未上传用户 bag、PCD 或大体积运行数据。

SHA 字段在提交后填写：

* `START_SHA`: `c030980fba6f93e0f7f232c7ac756335ff5b727a`
* `CODE_SHA`: report commit SHA
* `END_SHA`: report commit SHA

## 6. 唯一下一步

用户需要提供或确认一组严格闭合的资产合同：

1. 独立建图 bag SHA；
2. 独立定位测试 bag SHA；
3. PCD SHA 及其建图 bag 来源；
4. 建图/测试时间排除区间；
5. 最新 LiDAR-IMU 外参原始文件及方向；
6. `map_frame`、`body_frame`、`lidar_frame` 的明确变换链；
7. 若无独立 GT，至少提供不参与建图的外部测量或独立轨迹。

在这些证据到位前，唯一正确结论是 **INCONCLUSIVE**，不应宣布 GenZ-ICP
改善或恶化，也不应继续参数搜索。
