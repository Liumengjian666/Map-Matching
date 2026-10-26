# P63-R2：冻结地图坐标与来源核查

**终止结果 B：FRAME_CONTRACT_PARTIALLY_CLOSED。** 已解释两套源码的世界坐标和重力处理，并重建38.021059°的数值来源；但指定PCD的生产参数／二进制绑定、实际地图重力状态和物理LiDAR–IMU外参仍未全部认证。**不批准据此复测400帧，不宣称定位基线有效。** 本轮没有运行NDT、ICP、ESKF或ROS，也没有应用任何新变换。

| 身份 | 实际值 |
|---|---|
| START | `2a2f202c01efe6a6452436bd1eff084ae1ddfc14`，P63-R1最终归档 |
| branch / worktree | `research/p63-r2-frame-provenance-closure` / `/home/jian/livox_ws/p63_r2_frame_provenance_ws` |
| 离线计算 CODE | `b8339e277971c87c71d3a0be664e650120f0f722` |
| END | 包含本归档的最终commit，完整SHA见最终交接／`git rev-parse HEAD`；本地提交、未push |
| Super-LIO源码 | R1固定ros1 `60b57aaac8dc397f80c56364e7ccb008c300cc29`，未改 |
| FAST-LIVO2可读源码 | `/home/jian/livox_ws/src/FAST-LIVO2`解析到`/home/jian/FAST-LIVO2_project/FAST-LIVO2`；HEAD `0d2c0346107b75b59934975adec9a6eeeb913c64`，已有未提交改动，未覆盖；具体文件SHA及代码摘录冻结 |

证据等级含义：CONFIRMED=现有独立记录／源码可直接验证该有限命题；SUPPORTED=证据相符但缺少完整绑定；UNVERIFIED=证据不足；CONTRADICTED=明确反证。源码和代数的CONFIRMED不等于物理标定或独立GT通过。

## Q1：地图是否重力对齐？

- **CONFIRMED（软件语义）**：FAST-LIVO2先设`R_F_I0=I`、`g_F=-mean_acc/||mean_acc||*9.81`。`gravity_align_en=false`跳过将状态旋转到重力世界的`gravityAlignment()`，并非自动将地图Z定义为竖直。PCD保存的是已经变换到世界的点；保存时无额外重力旋转。
- **SUPPORTED（指定地图）**：同目录首条轨迹姿态距单位旋转仅0.214653°，结合首段IMU方向，符合“地图继承初始IMU轴、未重力对齐”的解释。较早运行日志也明确输出倾斜重力。但这些文件尚缺指定PCD的生产绑定，不能把支持升级为最终认证。
- **UNVERIFIED**：指定8月12日PCD真实生效的gravity_align_en、保存前重力向量及任何外部后处理矩阵。

## Q2：38.02°来自哪里？

| 数量 | 离线重建值 | 等级／范围 |
|---|---:|---|
| R1首段平均比力与IMU +Z |23.617764682°|CONFIRMED：封存向量的代数计算；物理解释要求近静止 |
| Super-LIO重力对齐初值将该比力转至+Z的残差 |1.24e-16|CONFIRMED：复现源码公式，double计算，不冒充实际float矩阵逐位日志 |
| 记录的ICP终值作用后，比力与固定world +Z |38.021059232°|CONFIRMED：封存矩阵／均值的数值关系 |
| 较早建图日志重力与-Z |23.625887976°|CONFIRMED：该较早运行，不自动属于指定PCD |
| 较早日志负重力与R1比力方向差 |0.008784920°|SUPPORTED：相同起步方向线索，不认证同一次生产 |
| 终值比力与较早日志负重力方向差 |30.645083473°|条件性比较，不可当指定地图真值误差 |
| 原框架固定重力下隐含静止净加速度模长 |6.381023483m/s²|SUPPORTED：近静止、当前身份及日志舍入前提下的兼容性诊断 |

23.62°和38.02°不是可以相加／相减分摊的同一轴旋转。前者在原始IMU轴中，后者在配准输出采用的地图数值轴中。原始加速度计均值是**比力**，静止时与重力相反。即使使用正确符号，38.02°仍存在。NDT中间矩阵未记录，无法分别归因NDT和ICP；本轮没有为补日志重跑。

**SUPPORTED的解释**：Super-LIO启动先把IMU转至水平世界，再对非必然水平的地图做配准；最终位姿被赋给ESKF，而其固定`g_world=-9.7946 Z`没有随地图坐标语义重新表达。这足以提出输入世界坐标不兼容问题；无法独立证明地图生产、ICP错误分支或物理安装旋转各占多少责任，也不把全部R1漂移归于唯一原因。

## Q3：零／单位外参来源？

配置和历史快照确实写T_I_L=(I,0)，等级CONFIRMED限于“文件配置”。原注释明确是缺少单独标定时的假设，`own_loop2.yaml`明确UNAVAILABLE_NOT_PHYSICALLY_CALIBRATED。没有适用设备标定或安装证明，**物理外参UNVERIFIED**。相机—LiDAR标定、driver点云外部变换参数、相同frame_id、作者其他设备杆臂均不能补齐这一证据。

## Q4：8月8日能否证明8月12日？

不能。较早日志明确保存`loop2_all_downsampled_points.pcd`、468495点；目标为`loop2_simtime_rebuild_2026_08_12_001_all_downsampled_points.pcd`、459154点。**“较早日志直接就是指定PCD生产记录”被CONTRADICTED**；这不证明两次配置一定不同。较早YAML可解释历史假设，不能证明8月12日所有有效参数及命令行覆盖。

同目录PCD、6615行轨迹、mat_pre/out和metrics的时间／命名关联为SUPPORTED。mat_pre/out在核查源码中不记录gravity，尾部`1 0 0`是曝光状态与占位，不能读成重力。现存ROS精确名称命中均为9月使用地图的消费者；shell命中为pcl_viewer命令，均不填补生产身份。

## 缺口和唯一NEXT

仍缺：(1)8月12日有效参数／启动命令及精确代码或二进制绑定；(2)该次初始化／保存重力状态及是否有保存后坐标变换；(3)可追溯到这台设备的T_I_L来源。当前不允许用某个定位误差更小的变换补齐。

唯一建议：将上述三项精确证据需求交给数据生产者／用户确认；若无法提供，由科研总控决定停止这组地图＋传感器组合或切换已有可追溯平台。不是先调角度复测，也不自动切换框架。详细变换、数值与来源分别见下列五份文件。

- [变换链](FRAME_TRANSFORM_CHAIN.md)、[重力数值](GRAVITY_CONSISTENCY.md)
- [地图生产来源](MAP_PROVENANCE.md)、[外参证据](EXTRINSIC_EVIDENCE.md)、[结果JSON](results.json)
- `evidence/INPUT_IDENTITIES.json`、带行号`SOURCE_EXCERPTS.txt`、原始摘录与`FRAME_NUMERICS.json`

所有历史资产保留，R1未覆盖；长期台账只追加本轮事实。本轮结束，不自动进入P63-R3。
