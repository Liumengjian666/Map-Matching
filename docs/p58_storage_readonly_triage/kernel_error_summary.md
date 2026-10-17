# 内核错误摘要

来源：当前启动实例的内核日志（`journalctl -k -b --no-pager`），只在内存/系统日志中筛选 USB reset、UAS、块 I/O、exFAT、offline、disconnect 和文件系统错误相关记录；没有读取外盘文件。

## 观察到的事件

- 日志在约 `16:21` 至 `16:46`（本机时区，2026-10-17）多次出现与 `/dev/sda` 相关的写入路径错误。
- UAS/SCSI 设备出现写命令超时、host reset（日志中有 `DID_RESET`）、UAS abort/reset 处理，以及 USB SuperSpeed Plus Gen 2x1 设备 reset。一次记录显示 UAS reset handler 成功，但之后仍有重复 reset/超时事件。
- `16:37:16`、`16:40:20`、`16:43:24`、`16:46:29` 附近可见重复的 `blk_update_request: I/O error, dev sda` 写操作以及 `Buffer I/O error on dev sda2 ... lost async page write`。
- 另一轮筛选还检出约 `16:21`、`16:24`、`16:28`、`16:31`、`16:34` 的相同故障类别，表明问题跨越数分钟反复出现。

## 解释边界

这些记录支持“USB/块存储写入链路持续异常”的判断；无法仅凭日志区分线缆、接口、供电、硬盘盒、设备本体或文件系统状态，也不能证明外盘上的数据是否完整。

在本次筛选结果中没有明确的 `device offline`、USB disconnect 或 exFAT 专属错误文本。由于这是按关键词筛选的日志摘要，不能将其解释为对所有内核消息的绝对不存在证明。无论这些项是否出现，已观察到的重复 reset 和写 I/O 错误足以判定当前状态不安全。

本轮没有运行 fsck、SMART 测试或任何写入/修复工具。

## 用户报告设备已检查后的再次复查

仅查看 P58 上次核查时间之后的系统内核日志。`16:55:11` 至 `17:00:49` 多次继续出现 UAS reset/abort 和 USB 设备 reset；期间有写命令 timeout、`blk_update_request: I/O error` 及 `lost async page write`。`17:01:04.593` 记录 `usb 2-2: USB disconnect, device number 2`。`17:01:08` 设备以新设备号重新枚举为 HIKVISION，内核重新识别 `/dev/sda` 及分区。重枚举只表示设备再次被发现，不表示存储健康。

检测到这些新错误后即停止后续外盘相关检查；没有查询目录、读取数据文件或执行任何恢复操作。

## 最新一次复查（2026-10-17 18:32）

查询 `17:01:08` 之后的内核日志，发现：

- `18:10:20.237`：USB 设备断开；紧接着 `Synchronize Cache(10) failed`，结果为 `hostbyte=DID_ERROR`。
- `18:10:23`：HIKVISION 设备重新枚举并重新附加为 SCSI disk。
- `18:31:59.925`：再次 USB disconnect；随后又出现 `Synchronize Cache(10) failed: hostbyte=DID_ERROR`。
- `18:32:02`：设备再次重新枚举。检查时刻为 `18:32:34`，即最近一次枚举后约 32 秒。

这是持续的 USB mass-storage/SCSI 通信异常证据，不只是 ZIP 下载或文件层面的错误。日志不能单独确定故障组件；不应仅凭重新枚举就继续读取大文件。此处记录后停止所有外盘查询。

## 降速尝试后的最新系统日志

在 `18:58:32.545`，`usb 2-2` 记录 disconnect；随后 `/dev/sda` 的 `Synchronize Cache(10)` 失败，错误为 `hostbyte=DID_ERROR`。`18:58:42.961`，HIKVISION 重新出现在 `usb 4-2`，日志报告 `SuperSpeed Plus Gen 2x1`，SCSI 驱动为 `uas`。因此日志没有证明 USB 链路已降到 USB 2.0；是否为用户主动移动连接导致这次 disconnect 无法从日志区分。

新错误出现后立即停止，没有继续读取任何盘上文件或查询分区/挂载信息。

## 系统重新启动后的低速读取窗口

本次用户确认后检查中，`journalctl -k -b --since 2026-10-17 19:24:12` 未检出 HIKVISION、UAS reset/disconnect、块 I/O error、exFAT 或 lost async write 相关记录。2 MiB/s 应用层限速顺序读取约27分40秒期间，读取完整结束；结束后进程状态筛选无 D-state。历史故障仍然保留，不能由本次短期稳定窗口推断为根因已修复。
