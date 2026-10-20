# P62-R5 — Exp18 坐标闭合或切换

正式结果：**G0_BLOCKED_COORDINATE_CONTRACT**。决策建议：**PIVOT_RECOMMENDED**，不是数据集切换已经执行。

**决定性阻塞：固定发布E57根系M与参考世界W之间的T_M_W没有闭合。** 此外GT的I/base导出姿态歧义未消除，不能把局部标定语义得到支持写成整体准入。

## 身份与边界

- START_SHA `49a6733fb63b60fff06f001c4efc1e9c5441cbea`，R4本地与远端实查一致，R4工作树干净。
- 分支 `research/p62-r5-exp18-coordinate-closure-or-pivot`。
- 工作区 `/home/jian/livox_ws/p62_r5_exp18_coordinate_closure_ws`。
- 有界审计开始2026-10-20 07:56:35 UTC，硬截止09:26:35 UTC；实际完成时间见analysis_manifest。
- 第一阶段源码/证据归档 `dbd9ca37b9b5082d794167e21184157b9693f31a`。数值脚本首次运行时尚未提交，以脚本实际SHA256为精确身份，不冒称当时已有commit。
- 原R4及R1创新合同不改，R9–R12没有重复执行。稳定工作区未操作。
- 不push。END_SHA由最终本地commit外部回执提供，避免伪造自引用。

## 六项执行结论

| 项目 | 实际取得的结果 | 不能推导 |
|---|---|---|
| 官方Issue #4/URDF | 作者修正的雷达原始轴与参数化帧名均核实 | 不代表整个标定/GT世界系都认证 |
| MOLA消费源码 | 固定profile→launch→Rosbag1Dataset.cpp链已核实 | 当前第三方实现不是录制时驱动或GT导出器 |
| T_I_L | xyzw与LiDAR→IMU方向有交叉来源支持，见下矩阵 | 没有重新实测外参；CAD与YAML非逐元素相同 |
| GT I/base与T_M_W | FAQ/MOLA支持I，URDF仍写base；T_M_W未知 | 不选择pi翻转，不假设世界系恒等 |
| 首扫描缺IMU | 提出保留原始首帧、融合从第2帧开始的有界规则 | 本轮未过滤bag，未放宽时间认证 |
| 非GT初始化 | 给出有限现成全局粗配准方案；未执行 | 实际初值仍INITIALIZATION_NOT_ESTABLISHED |

## 已缩小的标定缺口

发布747B YAML按xyzw及MOLA消费者得到T_I_L（LiDAR→IMU）：

```text
 0 -1  0 -0.001
-1  0  0 -0.00855
 0  0 -1  0.055
 0  0  0  1
```

显示为精确轴约定；完整浮点矩阵保存于numerical_contract_checks.json。官方URDF链给出同一旋转、X平移-0.000747m，两者差0.253mm。保留不同来源，未替换任何标定。
GT姿态方面，I和URDF base同原点但相差Rx(pi)，无法仅凭名字合并。tip位置同样有0.253mm差异、坐标轴相差180°，其位置参考消费仍须分开。

详细证据和固定commit源码行在[COORDINATE_CLOSURE.md](COORDINATE_CLOSURE.md)。官方Issue #4的作者回复支持原始轴处理；MOLA硬编码消费发布YAML，不代表动态加载该文件，更不是对发布GT姿态的独立测量认证。

## 地图与参考：为什么仍然阻塞

复用R4已完整解码的结果：E57有一个合并扫描、285455240点，保存非单位T_M_S，coordinateMetadata为空。论文说明TLS用于生成稠密参考，但不能据此确认固定发布文件的根坐标等于GT世界系。
补查的官方Issue #20/#21/#23没有提供该跨文件变换。#20的construction文件更换不能被误用为Exp18出错证据；用户对Sheldonian重叠的描述也不能充当官方T_M_W。
本轮没有以GT配准、外观或误差反推变换。**未知不是单位阵，也不是数据被证明错误。**

稠密参考继续为CONDITIONAL_REGISTERED_REFERENCE，不是独立稠密真值；三个稀疏标签不能代替独立6DoF。没有计算ATE/RPE或任何定位误差。

## 时间与初始化

冻结记录给出首扫描缺47303251ns前置IMU。拟议规则为原始1094帧不删、首帧融合不可用、扫描2–1094共1093帧才可按完整包络准入。秒编码和实际会话配置的认证层次不变，不靠外推补点时间。
初始化建议使用有限的现成FPFH/SAC prerejective工程粗定位，不接入论文Recovery模块，不用GT选择；预算耗尽或重复结构无法区分时拒绝。参数与验收须由下一合同批准，本轮不输出占位初值。详见[IMU_BOUNDARY_AND_INITIALIZATION.md](IMU_BOUNDARY_AND_INITIALIZATION.md)。

## 核验与限制

最终10项纯数学/冻结元数据与读取守卫测试通过（首轮7项，加3项守卫测试）。R4原归档47项checksums通过；本轮前后快照对小文件重算SHA，对大型输入仅stat，**没有新全量SHA读取，不能冒称本轮重验大文件内容**。沿用R4已认证的完整SHA，实际stat及来源见asset_identity_before/after.json。无大型点云导出或新依赖。

本轮GT原文件只做2个文件的SHA读取，未读取/处理其数值；GT格式/参考语义使用R4归档。大型bag/E57内容读取=0；ROS定位节点=0、回放=0、NDT align=0、EKF=0、定位轨迹=0、定位误差=0、时差搜索=0、bias拟合=0、生产修改=0、HIKVISION访问=0。

网络限流、错误源码目录404、无必要的MOLA总仓库clone90秒超时、checksum错误cwd均保留于PROTOCOL/TESTS_AND_REVIEW；没有用这些技术错误假称数据合同失败，也没有重试大数据。

## 唯一 NEXT

请总控验收后决定是否切换R1唯一备用NCLT的跨会话验证平台合同，不再延长Exp18坐标审计。不自动下载、运行或宣称NCLT准入；尤其R1已指出其发布稠密参考的跨会话依赖，不能降低独立性标准来通过。具体约束见[PIVOT_DECISION.md](PIVOT_DECISION.md)。

创新主线仍为Budgeted Selective NDT Recovery with Structural Holdout Evidence，原创性NOT_ESTABLISHED。本轮不实现或验证它，完成后停止。
