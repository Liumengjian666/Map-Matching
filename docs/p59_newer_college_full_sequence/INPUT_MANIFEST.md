# P59 输入清单

## 计划输入

源根目录：`/media/jian/HIKVISION/paper rosbag/NewerCollege/01_short_experiment/`。扫描目录：`raw_format/ouster_scan/`。规范扫描数为15,301，另有一个带 `(1)` 后缀的重复文件，被明确排除。首帧 `1583836591182590976 ns`；末帧 `1583838121156355584 ns`。扫描间最大相邻时间间隔不超过150 ms。

计划转存：15,301 PCD + IMU CSV + GT CSV + time offsets + 74 P45地图 Tile及manifest + 12个P46冻结运行文件；共15,391项，合计 `29,230,076,611 bytes`。精确逐文件预期路径、大小和可用来源哈希由内盘 `transfer_plan.json` 保存。

## 实际内盘副本

位置：`/home/jian/livox_ws/p59_newer_college_full_sequence_data/`。

- 完整 PCD：170个，索引1–170，`323,788,800 bytes`；时间戳连续。每帧SHA256重新核对P46 `input_scan_sha256.csv`，全部一致。
- 逐帧核验记录：本目录 `partial_scan_hash_verification.csv`；可用 `audit_partial_p59_mirror.py` 仅从系统盘副本复算。
- 保留临时前缀：2个 `.p59part`，各1,048,576字节；其中一个来自最初被10秒超时停止的第4扫描，另一个为D-state安全停止前正在复制的下一段扫描。它们不算完整扫描，也没有进入任何定位输入。
- IMU副本：0；GT副本：0；地图Tiles副本：0；完整 P46 runtime副本：0。
- 所以当前目录不是可运行输入，不得作为全序列数据集使用。

外盘源的P40/P45/P46冻结清单路径及其SHA保存在工作区跟踪文档和 `transfer_plan.json` 中。外盘没有被写入。源文件未全量读取或全量哈希；不声称完整输入校验通过。
