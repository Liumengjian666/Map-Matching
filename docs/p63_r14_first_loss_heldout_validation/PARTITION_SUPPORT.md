# 固定空间划分支持度预检

**结果：PASS（输入/划分支持度预检，0 次 NDT align）。** 对 raw5–raw2379 全部 2375 帧，重新按 R10 路径读取原始 PCD、复现 0.1 m VoxelGrid，并先逐帧匹配 R10 冻结的 Source 点数和 XYZ SHA，再应用 `PROTOTYPE_CONTRACT.md` 的固定 0.5 m 空间块哈希。

| 项目 | 结果 |
|---|---:|
| 帧数 | 2375 |
| 体素 Source 总点数 | 6,354,353 |
| 注册子集总点数 | 2,888,093（45.45%） |
| 验证子集总点数 | 3,466,260（54.55%） |
| 单帧 Source 点数范围 | 221–5,679 |
| 单帧注册点数范围 | 95–2,825 |
| 单帧验证点数范围 | 112–2,884 |
| 注册子集非空空间块数范围 | 8–193 |
| 验证子集非空空间块数范围 | 8–218 |
| 注册点比例范围 | 30.96%–58.77% |
| 帧内空注册集/空验证集 | 0 / 0 |
| 验证块少于 2 个的帧 | 0 |

每帧的注册/验证集合并后点数等于原 Source；实现按每个块唯一归属检查交集，交集为 0。固定分区没有按错误标签、轨迹结果或 NDT 质量调节。统计显示各帧都有多个验证空间块，可进行逐块残差描述；这并不证明块间统计独立，也不证明验证 Source 与 Target 地图来源独立。

## 可复现证据

- 源码：`src/spatial_split.hpp`，固定分块与哈希实现。
- 零 NDT 预检：`src/partition_preflight.cpp`。
- 逐帧输出：`research/p63_r14_first_loss_heldout_validation/data/preflight_output/PARTITION_SUPPORT.csv`。
- 构建身份与完整命令：`research/p63_r14_first_loss_heldout_validation/build/PARTITION_PREFLIGHT_BUILD.json`。
- 完整输出、`/usr/bin/time -v` 与资源：`research/p63_r14_first_loss_heldout_validation/logs/partition_preflight.log`。
- 本次分区预检墙钟 58.27 s，最大 RSS 63,312 KiB。该值只是输入/分块预检成本，不是 NDT 或在线定位耗时。
- 运行结果：`PASS fixed spatial split for 2375 R10 sources; registration=2888093 validation=3466260 align=0`。
