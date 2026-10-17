# P59 — Newer College Short 全序列 Local NDT

## 结果

**`INPUT_OR_STORAGE_BLOCKED`**。本轮没有启动 ROS、没有执行 20 帧 Smoke、没有执行正式定位，也没有读取 GT 内容或计算精度。阻塞原因是受控外盘读取期间观察到 D-state 内核 worker；按冻结安全门，立即停止后续读取。转存停止后内盘元数据检查时又观察到第二个短暂D-state worker，关系未确认。最后一次相关内核日志检查没有 USB reset、I/O、UAS 或 exFAT 错误；没有证据证明磁盘已损坏。原因未确认，因此不继续试读。

## 已完成

- 在独立 worktree `/home/jian/livox_ws/p59_newer_college_full_sequence_ws`，分支 `research/p59-newer-college-full-sequence`，基于算法基线 `a93d41d46ac60adc04552d76a09861e08f7ed87d`。
- 只新增 P59 内盘转存工具及测试；工作区没有修改 NDT、EKF、Deskew、OOSM 或地图选择算法。
- 对源清单完成元数据检查：15,301 个规范 PCD，首末时间分别为 `1583836591182590976 ns` 和 `1583838121156355584 ns`，无超过150 ms的扫描间隔。包括 IMU、GT、时间偏移、P45 Tiles 和 P46 参考输出的计划转存量为 `29,230,076,611 bytes`，系统盘当时可用约135 GiB。
- 以2 MiB/s、单线程顺序方式开始向系统盘转存。因执行端曾误把实际复制命令置于10秒超时，首个短尝试被终止；三个已发布首帧和第4帧的1 MiB `.p59part` 被保留。修正为只识别这组精确状态续接后继续，没有重读三个已验证首帧，旧 partial 也未覆盖。
- 停止时内盘共有170个完整 PCD（`323,788,800 bytes`），以及两个各 `1,048,576 bytes` 的保留 partial。170个已完成 PCD 的时间戳连续、逐个重新计算的 SHA256 均与 P46 冻结清单第1–170帧一致。尚未复制 IMU、GT、地图 Tiles 或 P46运行输出；这些输入不构成可运行数据集。
- P59 包编译成功；launch 节点参数展开通过；转存器及partial-prefix审计7项单元测试通过。170帧逐帧哈希复核保存在 `partial_scan_hash_verification.csv`。编译/launch 检查不是定位验收。

## 未完成与停止边界

实际完整输入、P46前1800帧行为 parity、Smoke、完整回放、运行输出冻结、GT评价、分段误差、RPE、资源曲线均未完成。没有观察到定位失败，亦不能声称定位成功或漂移。P59正式NDT/EKF/Publisher二进制与P46冻结二进制哈希不同，未运行前无法判断数值差异；哈希见 `RUNTIME_AND_MEMORY.md`。

没有再读外盘、没有向外盘写入，没有运行磁盘修复。保留了内盘部分转存文件及两个 partial，不清理。任何后续外盘读取需要用户完成物理检查并获得科研总控新批准。

## Git

- `START_SHA`: `a93d41d46ac60adc04552d76a09861e08f7ed87d`
- `CODE_SHA`: 见 `results.json`（源码文件 SHA256；没有形成新 commit）
- `END_SHA`: `a93d41d46ac60adc04552d76a09861e08f7ed87d`
- `REMOTE_HEAD`: `a93d41d46ac60adc04552d76a09861e08f7ed87d`
- `PUSH_EXECUTED`: `false`

具体协议、存储事件、输入清单和未执行指标见本目录其他文件。
