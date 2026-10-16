# P50 Floor01 输入合同审计

审计日期：2026-10-16。工作区：`/home/jian/livox_ws/p10_corridor01_oosm_r1_ws`。

| 输入 | 任务/历史期望 | 实际检查 | 结论 |
|---|---|---|---|
| Floor01 canonical bag | `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/p3_r7_floor01_full_baseline/floor01_canonical_raw_inputs.bag`；R9B 声明 SHA `6383fdbdad0e3f8375069b33cc552ed069dafb91aec88f1673bc043e0f314e0b` | 文件不存在；`SuperLoc/Floor01` 目录不存在；历史材料未提供可核实的大小 | BLOCKED |
| Velodyne 原始输入 | VLP-16 `/input/velodyne_packets`；历史审计为 4,139 raw packet scans，76 packets/scan；必须解码以得到点级 firing time | canonical bag 不存在；没有可用 packet 数据 | BLOCKED |
| timed-point 数据 | P9/R6 记录的 `/tmp/p7_baseline_v2_floor01_raw_retry/raw_timed_points.bin`、`raw_timed_scans.csv`；期望 SHA 分别 `2c64e78c8b7faaf2b43f0e53fda95198de8472270bd9151acaafd12759bdf809`、`711ee35603a0026f2d04e65100dd8ba068b312e06ab0f992e191e6f55c0b8bd6` | 两个文件均不存在 | BLOCKED |
| P6 派生 LiDAR 资产 | 同序列历史恢复资产 | `request_xyz_f32.bin` 1,360,256,112 bytes，SHA `75e71ba365b9c2d0b247f4bf3944008994d398f8f5d87fcb2d49e9e7ffd2a09e`；manifest 声明 113,354,676 点、每点 XYZ 12 bytes、4,127 scans；`scans.csv` SHA `9371e593c0e625611f053e3ef0a581c52481ccaed392aa8d74d74313caa9938f` | 点坐标及逐扫描时间/偏移可读，但无每点时间与原始 packet | 仅派生资产，不满足本轮 raw/timed 输入合同 |
| IMU | `/input/imu` 或已核验真实等价输入 | `input/imu.csv` 83,342 行，stamp 范围 `1660857392515904000`–`1660857810012496000 ns`，SHA `faf83b341d6084df5a84e431d61a15f6680a7465d83e9fc7bba13a03961722c2` | 恢复副本存在；其 manifest 指向的 runtime-topic bag SHA `860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db` 不存在，无法在本轮直接从原始 bag 复核来源 |
| 地图 | frozen Floor01 H1 map；SHA `2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570` | P6 恢复副本 `/home/jian/livox_ws/p6_i1_recovery_assets_20260927/floor01_h1_map_p5_frozen.pcd` 实测 SHA 完全匹配，6,595,818 bytes | PASS（只读恢复副本） |
| 外参 | `floor01_extrinsics.yaml`；历史 SHA `fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414` | 数据目录/声明路径不存在；R9B 配置 SHA `7e4cd51280e9c478ce9f9d4f33e8188407aef78dde67438d585fc0a91e2fbfeb` 含冻结 `T_imu_lidar` 数值，但不能重验原标定文件 SHA | 需随输入恢复/在恢复时认证 |
| 初始化 | 已有合法 Floor01 起点；不得使用 GT 选择 | 现有配置和历史数据里有初始化记录，但未以当前原始序列合同复核；由于 raw bag 缺失，没有冻结 P50 起始帧 | 未验收 |

## 时间/点字段要点

历史 Floor01 审计明确原始 `VelodyneScan` 消息本身没有逐点时间字段；点 firing time 由 VLP-16 packet decoder 导出。P6 的 `request_xyz_f32.bin` 长度正好是 `113,354,676 × 12`，即每点仅 3 个 float32 XYZ；这份文件没有 packet bytes 或逐点 acquisition offset，因而无法核验原始 firing-time 顺序并按点时间执行 IMU Deskew。每帧 scan timestamp 不能替代点内时间。

因此，虽然地图、派生 IMU CSV 与 XYZ clouds 有部分恢复资产，仍无法满足真实连续 VLP-16 解码和 IMU Deskew 的 P50 输入合同。没有启动数据发布器或定位节点。

## 仓库状态

- Branch：`research/p9-r4-heldout-visual-evidence`
- START_SHA：`c28ecc95763e6f5bf7dacee80b68dd7a28b9a278`
- 检查时 REMOTE_HEAD：`c28ecc95763e6f5bf7dacee80b68dd7a28b9a278`
- 本轮没有改写以上任何历史资产，也没有复制大型文件。
