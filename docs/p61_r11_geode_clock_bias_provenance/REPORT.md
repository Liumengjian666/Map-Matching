# P61-R11：GEODE时钟与bias来源只读审计

## 正式结果

- H11-A：**CLOCK_SYNC_NOT_CERTIFIED**。
- H11-B：**BIAS_STABILITY_NOT_EVALUABLE**。
- 综合：**R10_CONTRADICTION_REMAINS_CONDITIONAL**。

未认证不等于已证明不同步或bias漂移。R10的ANGULAR_CONSISTENCY_CONTRADICTED
在其固定时间、外参、bias合同下保留；本轮既没有推翻数值计算，也没有证明Raw NDT错误。

## Git及执行身份

Repository：https://github.com/Liumengjian666/Map-Matching。
Worktree：`/home/jian/livox_ws/p61_r11_geode_clock_bias_provenance_ws`。
Branch：`research/p61-r11-geode-clock-bias-provenance`。
START_SHA：a0275decc7862030c3581038ab87d09efe5aaa4e。
任务起点实查R10本地/远端均为此提交，本地干净；未依赖旧回执。

实际单次传感器消息核查CODE_SHA：8fe5d08241be44c821b55f9d773e0eb11656eea8。
元数据JSON修复/已保存CSV归档CODE_SHA：fe652728f（完整SHA见analysis_manifest.json）。
R8实际算法17922f2b63dae1f2af34f353287728d50a312f73、准备工具
1367e1d9587d1f90a47bf6f44405239a7b65e4f6仅作源码审计，未执行。
没有将R11修复后的脚本冒称第一次实际读取版本；序列化失败原件保留，见EXECUTION_NOTES.md。
本轮本地提交，不push；最终END_SHA与实时远端状态见最终回执，不自嵌文件哈希。

## 输入身份与范围

原始bag：/home/jian/geode/Shield_tunnel1_gamma/Shield_tunnel1_gamma.bag，
4500527001B，SHA162ed09ab6b262f89ac46c45bac36ad2335a6f869b360b80d80ddf59d0b22091。
R8 bag：/home/jian/livox_ws/p61_r8_geode_runs/input/full_return.bag，
1416133574B，SHAa138b5ceb0e8ee4a2e5203e70b3943596811ff15d7a45495909934d3a1494331。
R10输入冻结摘要a2f3b61260e727addc935fff38cdfee587a88ba806d4a3992e094af9bb8af690。
地图、初始化、协方差、数值配置、完整精度YAML、运行二进制、全部声明R8输出及R9/R10
归档逐项核验；详细绝对路径、大小、SHA和冻结来源见asset_gate_before/after.json。
额外来源资料单独pin；没有历史SHA的既有日志明确标为首次R11快照，不假称历史已冻结。

目标窗口固定1706671804799723787–1706671805899593546ns（1.099869759s）。
12个姿态端点5154–5165不等于12条窗口内LiDAR消息：header准入实际为5155–5165共11条。
窗口内110条Xsens消息及单独R10 causal-head逐字节核验；不重新积分。
历史seed仅查源码/真实stage记录指定的10条原始及R6 bridge消息。
时间查询用既有R2 record元数据精确定位后检查header，不假设两个时间相等。
只分析所批窗口，没有读取其他topic的payload。完整IMU旧CSV只使用topic/header/record
字段作查找，没有对全段gyro作统计或扩大分析范围。

## H11-A：实际记录时差与同步证据

|输入|话题|样本数|header-record min/median/P95/max(ns)|
|---|---|---:|---|
|原始bag|/livox/lidar|11|0/0/0/0|
|原始bag|/imu/data|110|0/0/0/0|
|R8 bag|/geode/raw_points|11|0/0/0/0|
|R8 bag|/imu/data|110|0/0/0/0|

P95为线性经验分位数；0是实际值，不是缺失替代值。
R8记录时间由prepare_inputs.py显式设成header；原始bag也观察为0，但未认证实际采集
writer/驱动来源，不能假设是独立到达时间。共同epoch、零记录时差均不足以证明物理同步。
原始连接元数据缺callerid；有限topic名检查没有同步/温度记录。设备说明支持gamma为
Livox AVIA+Xsens MTi-30，不能混入Ground-Challenge的MTi-300身份。
具体录制SDK/驱动版本、PPS/PTP/共同触发配置及同步误差日志缺失。
逐环来源、单位、源码点位与未知项见clock_source_chain.md。

## H11-B：偏置来源和独立适用性

10条历史初始化样本1706671289500390052–1706671289591141939ns的gyro均值
与实际初始seed记录一致，最大算术误差2.168404344971009e-19rad/s。
初始化只按计数退出，没有可核验独立静止合同。随后2984扫描/29844 IMU连续LIO运行，
2981次MAPPED更新允许native bg状态更新。冻结R6导出bias为末端`s.bg`，不是首10条seed。
其值与R10固定bias完全一致；到目标窗口已有216.969883664s。
没有目标时刻独立静止或bias复核，不能用运动中的gyro均值或未来EKF bias代替。
详细seed、最终值、源码点位、单位/坐标系及日志见bias_provenance.md。
stationarity_evidence.csv仅有表头，含义NOT_EVALUABLE，不是静止次数0的物理断言。

## 反证与结论边界

作者声明有同步方案、实际header/record一致、算法offset配置0，均是支持性背景；
不足以补全该序列物理时钟链。设备可能有有效内部同步/温补，但现有证据不能认证。
来源可追溯的LIO估计bias也可能适用；本轮无法独立证明或否定其稳定性。
没有证据足以将R10残差唯一归因于Raw NDT候选、错误盆地、时间偏移或bias漂移。
没有重新解释R10门限，没有调整其bias，没有生成任何新定位轨迹。

## 测试、资产及边界

9项纯数据合同测试通过；独立本地反证复核发现的时间域边界和覆盖声明问题已修复，
具体在PROTOCOL.md和record_selection.py。另保留并修复ROS元数据bytes序列化问题，
用已保存CSV完成单独JSON，无传感器消息重读；未隐藏失败或覆盖旧文件。
code-review-and-quality按正确性、可读性、架构、安全、成本审查，仅改R11研究目录。

分析前后资产核验及一致性统计见analysis_manifest.json；任何失败都不能作为通过。
正式计数：GT reads0；ROS localization nodes0；rosbag play0；new NDT align0；
EKF runs0；new trajectory estimates0；production code changes0；外部CLI0；
clock-offset search0；gyro-bias fit0。未访问HIKVISION。

## 唯一NEXT

向数据作者索取并绑定到Shield_tunnel1_gamma采集会话的**原始传感器溯源包**：
实际驱动/SDK/固件版本及时间映射代码、同步接线/配置与验证日志，以及独立静止/温度/bias
校验记录。此建议是一次证据补齐请求，不是批准新实验或时间/bias拟合。
验收看能否闭合每段时间转换并提供独立同步误差界、是否存在不依赖本次NDT/LIO的bias证据；
缺少任一项仍保留NOT_CERTIFIED/NOT_EVALUABLE。完成后停止，等待科研总控验收。
