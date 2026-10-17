# P59 执行协议与偏离记录

## 冻结条件

- 数据：Newer College 2020 `01_short_experiment`。
- 初始化：`CONDITIONAL_BASE_FRAME / ORACLE_START_ONLY`，只允许首帧起点；GT不得进入运行时。
- 算法：P46 Local CONTROL；PCL NDT、分辨率0.8 m、Source上限1400；P45 20 m XY Tile、10 m safety、30 m hysteresis；原 IMU Deskew、EKF、OOSM、初值、速度融合、步长限制和覆盖门。
- RViz关闭；不启用 Coupled、Recovery、Visual 或 Information-Aware。
- 输出目录固定在内盘，不向 HIKVISION 写入。

## 执行次序与实际状态

1. 核查独立 worktree、分支、HEAD、外盘UUID、系统盘容量、D-state及内核日志：通过；系统盘有约135 GiB可用，外盘UUID `F627-4133`。
2. 建立完整输入转存清单：15,301规范扫描、15,391项计划文件，计划总大小29,230,076,611字节。
3. 转存：以2 MiB/s上限运行，未完成。首次命令因执行端10秒命令超时而中断，保留其内盘成果；对该精确状态做受限续接后，系统观察到D-state worker，按安全门中止。没有重试。
4. 因本轮数据不完整，停止。Smoke帧数=0，正式运行次数=0，NDT align=0，GT运行时读取=0，GT离线读取=0。

## 安全停止规则

出现任意D-state时立即停止后续外盘数据读取，即使随后状态消失或没有对应I/O日志也不恢复。保存部分文件，不删除、不修复、不继续测试。最终状态为 `INPUT_OR_STORAGE_BLOCKED`，不是磁盘损坏判定。
