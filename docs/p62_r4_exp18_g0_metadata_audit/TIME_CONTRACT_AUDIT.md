# 时间与 IMU 覆盖合同

正式状态：`TIME_CONTRACT_NOT_ESTABLISHED`（完整 bag 融合输入尚不准入）。
分开记录：字段格式/编码支持、原始 ROS 时间支持、实际记录版本/配置、物理同步证据。

## 全段实际字段

完整遍历 66633 条消息、1094 帧 PointCloud2、43656 条 IMU。全部点云共 64227729 点，全部扫描均检查点时间；timestamp 和 XYZ 未发现非有限值。未只检查首帧。

PointCloud2：小端，point_step=48；x/y/z 为 FLOAT32@0/4/8，intensity 为 FLOAT32@16，timestamp 为 FLOAT64@24，ring 为 UINT16@32。schema 全段一致。
实际话题 `/hesai/pandar`、frame_id `PandarXT-32`；IMU `/alphasense/imu`、frame_id `imu_sensor_frame`。

header 和 bag record 使用原始整数 ns。每个带 header 的话题，两者之差全段都是 0；这只是存储字段关系，不证明物理时钟偏移为 0。没有按 NDT 残差搜索时差。
每话题时间范围、间隔、重复、回退和相同序列化消息计数见 `topic_inventory.csv`；本次这些重复/倒退计数均为 0。

## 点时间编码证据及边界

[官方 FAQ 的 PointXYZIT 说明](https://github.com/Hilti-Research/hilti-slam-challenge-2022/blob/e4aacf7c111ab832a0bdffe3f069255454068540/README.md) 与实际字段类型、偏移布局一致。
查阅固定 Hesai 源码 commit `f574bb8a680e215a6ade9bf35e0898932b2a2234`，快照 SHA 见 parser_environment：

- point_types.h 声明 double timestamp，但此声明本身不证明单位。
- hesai_internal.cc 中 packet timestamp 用 mktime 加微秒/1e6；XT 点计算函数中 device 分支再加微秒级通道补偿/1e6。
- 同函数另有 realtime 分支。因此不能宣称已证明本 bag 使用哪一条录制配置。
- EmitBackMessege 用点列表首点 timestamp 作为回调 timestamp；它不是“所有扫描结束时间等于 header”的依据。

原数据数值约 1.649856e9、扫描时长约 0.1；在绝对秒解释下，每帧最早点比 header 晚 0–999ns，中位 861.5ns。固定源与全段数据共同支持“absolute epoch seconds”的编码解释，但录制时驱动 commit、timestampType、时区配置并未从本会话认证。未把所查版本冒充实际录制版本。

raw FLOAT64 的 spacing 为 2.384185791015625e-7 原编码单位；若按秒解释约 238.419ns。因此从 double 得到整数 ns 不能创造原始数据没有的亚纳秒精度。端点转换采用 Decimal.from_float 后乘1e9并舍入，避免直接浮点乘法再放大精度损失。

`scan_time_audit.csv` 保存全部原始编码范围，换算列明确以 hypothetical_ 命名。`imu_coverage.csv` 的正式 status 仍为 TIME_UNIT_NOT_CERTIFIED，没有把假设性包络判为正式准入 PASS。

## IMU 与全扫描包络

IMU 43656 条时间严格递增、无重复/回退、加速度和角速度有限。相邻间隔：min 2473200ns、median 2505000ns、P95 2510100ns、max 2535600ns。分位数算法为 NumPy linear。

按官方格式所支持的绝对秒解释计算，但不修改原始数据：1093 帧有完整 IMU 前后包围，1 帧缺起点包围。

首帧：

- header：1649856227623466000ns
- 实际点时间编码换算起点：1649856227623466969ns
- 换算终点：1649856227723469019ns
- 第一条 IMU：1649856227670770220ns
- 起点缺口：47303251ns，约47.303251ms

首帧没有被裁剪、丢弃、外推或靠下一帧替换。末帧按同解释有合法 IMU 包围。每帧区间、包围样本数和最大间隔均在 CSV 中，不根据结果改变扫描准入规则。

## 物理同步资格

[官方平台说明](https://hilti-challenge.com/dataset-2022) 给出 PTP/硬件同步和1ms以内对齐说明，是平台级支持证据，不是本次由 header=record 独立测得的同步误差。本轮未把设备时钟直接改写为 ROS epoch，未重建点时间，也未拟合 bias。

该原文已有R1冻结缓存：`/home/jian/livox_ws/p62_r1_source_cache/hilti_page.html`，41298B，SHA256 `9dad17cabe011d5014987f3150154e0b806210eb011ee26fdbbad1e4b74497bc`，与R1 source_manifest.json条目一致。`platform_sync_evidence.json`记录该身份与短原文片段，避免只依赖可变网页链接。它仍不是Exp18实际会话的独立误差证明。

即使以后补齐编码/记录版本说明，首帧缺前置 IMU 仍须通过新合同明确处理，不能把本轮完整 bag 直接认定为连续 Deskew 输入已通过。
