# 设备与挂载状态

所有查询均为系统设备/挂载元数据读取，不访问外盘目录中的数据文件。

| 项目 | 系统当前显示 |
|---|---|
| 块设备 | `/dev/sda`，约 `953.9G`，传输类型 `USB` |
| 分区 | `/dev/sda2` |
| 文件系统 | `exfat`（挂载表类型显示为 `fuseblk`） |
| UUID | `F627-4133` |
| 挂载点 | `/media/jian/HIKVISION` |
| 挂载选项 | 当前元数据显示 `rw` |
| USB 拓扑 | mass-storage，驱动 `uas`，链路报告 `10000M` |

`findmnt`、`lsblk` 和 `lsusb -t` 查询均在 5 秒超时内返回。系统中 `lsblk` 不支持任务建议的 `MOUNTPOINTS` 列名；改用受支持的单数 `MOUNTPOINT` 列后查询成功。这个列名兼容性问题不涉及外盘数据访问。

设备仍可枚举且仍显示挂载，不是健康证明。结合内核重复 UAS reset、写 I/O error、异步页写丢失和 D-state flush worker，当前结论仍为 **STORAGE_UNSAFE**。

没有检查容量余量、目录、partial 文件大小、哈希或 ZIP 内容；这些都属于本轮禁止的外盘文件访问。

最新一次（18:32）复查没有查询 `findmnt` 或 `lsblk`，因为内核日志刚记录 18:31:59 USB disconnect 和 cache synchronization failure。日志显示设备于 18:32:02 重新枚举，但当前分区是否挂载及 UUID 是否仍可读未核实。不要把重枚举当成挂载或健康确认。
