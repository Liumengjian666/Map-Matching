# 两份原始官方资产均完整验证

| 资产 | 字节 | SHA-256 | 状态 |
|---|---:|---|---|
| sheldonian_full.e57 | 5,447,128,064 | 46575d3eaf3169a24e24ba0013cc48610125ac842d6745e5e8156c6aecc59756 | PASS |
| exp18_corridor_lower_gallery_2.bag | 8,657,654,860 | 98373b52f207b9ace3914792b5604a6642f89441190cabca6acbaf1d17249280 | PASS |

保存于 /home/jian/p64_r3_q10_ws/run/assets。沿用合法分块，map、bag 分别约 19:06 和 19:19 落盘，原始文件之外保留历史块及元数据。正式 checksum 命令两文件均返回 OK。无完整 HILTI 其他序列下载。 [官方发布](https://huggingface.co/datasets/Hilti-Research/hilti-slam-challenge-2022/tree/main) [双文件校验](full_asset_sha256_check.txt)

原始 bag 实读 1,094 帧 LiDAR，索引含 43,656 个 IMU 消息和 3 个实际测针事件（P30/P7/P36）。录制时间约 1649856227.642634 到 1649856337.010822 s。已知初值从第 2 帧 LiDAR 起，全部 **1,093** 帧都有末回波后的 IMU 支撑，并制作了用于真实 ROS1 回放的派生 bag。 [原始录包审计](EXP18_FULL_BAG_AUDIT.json)
