# P62-R3 — Hilti Exp18 有界资产恢复

正式结果：**ASSET_RECOVERY_PASS_BOTH_FIXED_REVISION_FILES**。

本轮只取得并核验固定版本的两个文件；不代表数据集 G0、坐标、时间、参考独立性或初始化合同通过，也不代表定位算法有效。

## Git 与执行身份

仓库：<https://github.com/Liumengjian666/Map-Matching>。

- 工作区：`/home/jian/livox_ws/p62_r3_bounded_exp18_asset_recovery_ws`
- 分支：`research/p62-r3-bounded-exp18-asset-recovery`
- START_SHA：`d383bfed52db638f54a9b208c1ec27f663d63eae`
- 实际恢复程序 CODE_SHA：`3d54bb08d9f712724c911cd5f8931e6063e90fb8`
- R2 本地 HEAD 与本次实际查询的远端 R2 HEAD 均为 START_SHA；R2 工作树干净。未重写 R2 历史。
- R3 远端分支在开始及恢复结束后的实际查询均未返回引用；PUSH_EXECUTED=false。
- 最终归档提交 END_SHA 由本报告所属的本地归档提交和最终回执记录。恢复后的 `audit_receipts.py` 是离线回执复核工具，不冒充实际传输版本。

系统 Git 为 2.25.1。新工作树建立时遇到 sparse-checkout 与索引位交互，曾临时展开仅属于新工作树的历史受控文件副本，造成启动余量低于门槛。该阶段没有网络下载。通过 Git 稀疏检出移除了新工作树的冗余副本，未清理任何历史数据资产，重新满足门槛后才运行恢复程序。没有修改稳定机器狗工作区。

## 固定来源与完整身份

Hugging Face revision：`e62017f907007fdc5ab8c721842e4ae7359d7f49`。
完整官方 URL 见 `transfer_manifest.json`，所有请求均绑定该 revision，而非 `main`。

| 资产 | 实际完整字节数 | SHA-256 | 数据分段 |
| --- | ---: | --- | ---: |
| Exp18 bag | 8657654860 | `98373b52f207b9ace3914792b5604a6642f89441190cabca6acbaf1d17249280` | 123 |
| Sheldonian E57 | 5447128064 | `46575d3eaf3169a24e24ba0013cc48610125ac842d6745e5e8156c6aecc59756` | 82 |

发布目录：`/home/jian/livox_ws/p62_r3_exp18_recovery_data/assets/verified/`。

- bag：`rosbags/exp18_corridor_lower_gallery_2.bag`
- E57：`laser_scans/sheldonian_full.e57`

每个完整 staging 文件先经过大小、全文件 SHA、magic 检查，再以同文件系统硬链接发布；发布路径再次执行完整 SHA 和大小检查，并核对 inode。两份路径共用各自 inode，不产生第二份大文件拷贝。magic 分别是 `#ROSBAG V2.0\n`、`ASTM-E57`。

## 旧 partial 与前缀合同

完整前后清单：`asset_identity_before.json`、`asset_identity_after.json`。两个 JSON 内容逐字节相同，5 个旧 partial 的路径、大小、SHA、st_blocks 和分配空间全部保持不变。

| 旧文件 | 逻辑字节数 | st_blocks（512B） | 分配字节数 | 用途 |
| --- | ---: | ---: | ---: | --- |
| 线性 bag partial | 452984832 | 884744 | 452988928 | 经验证后复制为新恢复文件的起始前缀 |
| 稀疏 ranged bag partial | 7576092054 | 7160 | 3665920 | 仅保护和哈希；未作为连续前缀 |
| dense reference cached partial | 88743 | 176 | 90112 | 仅身份哈希，未解析 |
| sparse reference cached partial | 185 | 8 | 4096 | 仅身份哈希，未解析 |
| calibration cached partial | 747 | 8 | 4096 | 仅身份哈希，未解析 |

稀疏 ranged 文件的逻辑长度不能代表下载完成量；其 R2 历史有效响应约 3627186 字节，本轮未将其发布或混接。所有旧文件均为普通文件。

线性 partial 完整 SHA：`2aa6b290c20ec150107c91ed0c987ef025c180d7f1d89c18cb49754358bd83a6`，与 R2 冻结记录一致。先请求官方固定 revision 的 bytes 0–1048575，实际 HTTP 206、Content-Range `bytes 0-1048575/8657654860`、收到 1048576 字节，与本地前 1 MiB 逐字节相同。前缀 SHA 为 `2335571cd2cb5b6dca1da2b11a07e2751c80bb4995f46c399e400360fee7f5c4`。随后复制整个旧线性 partial 到全新的 staging 文件，再核验复制 SHA；未对旧文件追加或覆盖。

## 有界执行结果

- 实际程序运行：2026-10-19 09:17:37.342568 至 11:38:49.315190 UTC，即北京时间 17:17:37 至 19:38:49。
- 程序墙钟：8471.972622 秒（141.199544 分钟）。从本轮首次核查约 09:04 UTC 起计仍低于 180 分钟。
- 保守硬截止：12:00 UTC；传输截止：11:59 UTC，另受进程开始后 180 分钟上限约束。没有触发截止。
- 单个活动 HTTPS 请求，64 MiB 分段，末段为准确余量；socket 超时 90 秒，每段最多两次。
- 实际请求 206 次：1 次前缀 probe + 205 次数据请求。205 个数据分段全部提交。
- 失败请求 0；重试 0；拒绝响应 0；超时 0。
- 本轮网络接收共 13652846668 字节（含 1 MiB probe）；bag 续传数据 8204670028 字节，E57 数据 5447128064 字节。
- bag 数据请求耗时合计 5194.927673 秒，有效载荷/请求时间约 1.506197 MiB/s。
- E57 数据请求耗时合计 3208.080663 秒，约 1.619282 MiB/s。以上速率不含初始旧文件哈希、复制、发布哈希等开销，不冒称全任务速率。

每段真实偏移、尝试号、HTTP 状态、Content-Range、实际长度、SHA、请求耗时和有效前缀提交位置均见 `chunk_manifest.csv`。离线独立回执检查验证全部区间连续、末段边界正确、响应与提交逐项一致。未将单个响应通过解释为整文件通过。

## 存储与测试

开始可用：48201134080 字节，满足 48000000000 字节启动门槛。
结束可用：34094829568 字节，仍大于 21474836480 字节（20 GiB）安全预留。每次写入检查安全余量，未触发存储停止。转换工作区使用量 0；未安装依赖，未访问 HIKVISION，未删除旧资产。

19 项恢复程序合成单元测试在运行前通过、结束后重跑再次通过。覆盖 206/range/长度拒绝、半截超时不追加、同段最多两次、前缀冲突、缺失冻结条目拒绝、存储门、路径/symlink、deadline、发布不覆盖和收尾错误不伪报成功。测试不是 ROS/定位有效性证据。

本地反证复核修正了预检未纳入异常收尾、旧条目缺失时开放式验证、截止被收尾捕获吞掉以及回执失败可能返回 PASS 等问题。全部修正在 CODE_SHA 中完成后才运行。未执行外部模型 CLI。

## 边界、未决事项与唯一 NEXT

本轮 rosbag play=0、完整 bag 遍历=0、完整 E57 解码=0、ROS 定位节点=0、NDT align=0、EKF=0、轨迹生成=0、ATE/RPE=0、时差搜索=0、gyro bias 拟合=0、生产算法修改=0、push=0。

旧的两个参考轨迹 cached partial 仅按要求计算文件身份哈希，未解析 GT 数值，未用于任何精度评价。不得将这种文件身份读取模糊记成“完全没有读取任何 GT 文件字节”。

唯一 NEXT：将完整文件身份和回执交科研总控验收，申请下一轮有界 G0 元数据审计，检查 bag/E57 完整可解析性、地图—参考坐标关系、点级时间、传感器标定及参考独立性；仍不自动启动定位或算法实现。

复核命令见 `reproduction_commands.md`。本轮恢复程序是一次性执行入口，已有输出时会拒绝重跑；历史下载命令不是再次下载授权。
