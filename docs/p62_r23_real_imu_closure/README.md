# R23：真实 IMU / NDT / EKF 闭环复现

本目录保存研究工作区的真实ROS闭环对照。入口复用已有Newer College发布器、P45局部地图、原生PCL NDT及现有IMU/EKF/OOSM。默认策略为`MULTISTART_C`，实验性D默认关闭。数据不是rosbag；发布器实际读取本地Ouster PCD和原始Ouster IMU CSV。

## 一条命令启动

在具备桌面显示的VS Code终端直接复制以下**单行**命令。它使用本R23的构建结果和专用本地ROS master，自动加载地图与RViz显示配置，默认运行W1/C0的60个评价扫描：

```bash
source /home/jian/livox_ws/p62_r23_real_imu_closure_ws/devel/setup.bash && ROS_MASTER_URI=http://127.0.0.1:11383 ROS_IP=127.0.0.1 roslaunch dog_prior_map_localization p62_r23_integrated_localization.launch strategy:=MULTISTART_C rviz:=true
```

地图、局部目标、当前Deskew点云、NDT对齐点云、NDT位姿/轨迹以及EKF高频/校正轨迹均已预配置；固定坐标系为`newer_college_map`。无需手动加载PCD、添加Display或修改Fixed Frame。交互入口使用匿名新输出目录，位于`/home/jian/livox_ws/p62_r23_real_imu_data/`。数据发布结束后可继续查看RViz，查看结束按Ctrl-C关闭本次launch。复现命令会产生新的真实计算，文档提供命令不表示执行端将自动再运行。

真实GUI验收记录为W1/C0、`BASELINE`、20扫描，详见[RVIZ_ACCEPTANCE.md](RVIZ_ACCEPTANCE.md)。默认C入口和三模式共享同一已验证可视化配置，GUI验收本身没有另行运行C/D精度比较。

## 三个显式策略

|`strategy`|地图观测处理|默认状态|
|---|---|---|
|`BASELINE`|继承原生有限/收敛输出及现有限步、IMU/EKF/OOSM路径；原始NDT与最终发布观测分别记录|须显式选择|
|`MULTISTART_C`|冻结R20基本质量和Coverage Consistency Mean≤0.30；事件时地图yaw±5°各一次Full NDT；按原Native Score/距离/ID排序|默认|
|`RELATIVE_D_EXPERIMENTAL`|在C规则上保留冻结R21严格相对支配例外；风险已知，结果属于实验性消融|默认关闭，须显式选择|

C/D每帧最多2次额外Full NDT、每条60帧轨迹最多12次。无合法候选时不发布独立NDT测量；EKF继续消费真实IMU传播。选中候选与现有限步后实际发布观测不能混为一条记录。`heldout_mean`历史命名在这里仅表示**Coverage Consistency Mean**，Full优化已使用相关点，不能称独立留出证据。

切换策略时只将命令中的`strategy:=MULTISTART_C`改为上述完整模式名。各模式必须从新进程、独立状态和新目录运行；在C结果上事后重选D不是D闭环实验。R23没有将D写入机器狗默认配置。

## 窗口和初始化

唯一输入合同为`input/INITIALIZATION_AND_TIME_CONTRACT.json`。运行时仅使用冻结锚点位姿，后续参考轨迹不参与预测。初始化状态为`T_map_imu=T_map_lidar·inverse(T_imu_lidar)`；C3先对锚点LiDAR姿态施加地图Z轴+8°、保持LiDAR平移，再重算IMU初态。零速度和零bias是继承的初态先验，不能称锚点真实静止或bias已估计。

|窗口|锚点/IMU重放起始ns|首评价LiDAR `start_stamp_ns`|真实 `expected_boundary_imu_stamp_ns`|初始化文件|
|---|---:|---:|---:|---|
|W1 raw2600–2660|1583836851078989056|1583836851179232512|1583836851078961159|`config/w1_c0_initialization.yaml`或`w1_c3_initialization.yaml`|
|W3 raw6500–6560|1583837241072465152|1583837241172360960|1583837241068926621|`config/w3_c0_initialization.yaml`或`w3_c3_initialization.yaml`|

不运行锚点NDT；从锚点边界开始逐条重放IMU，到首评价扫描才发布LiDAR。`initialization_config`、`start_stamp_ns`、`imu_replay_start_stamp_ns`和边界样本必须成套切换，不能沿用最初序列的初值或漏掉锚点到首评价扫描的IMU。

W3/C0、C策略的一条完整命令：

```bash
source /home/jian/livox_ws/p62_r23_real_imu_closure_ws/devel/setup.bash && ROS_MASTER_URI=http://127.0.0.1:11383 ROS_IP=127.0.0.1 roslaunch dog_prior_map_localization p62_r23_integrated_localization.launch strategy:=MULTISTART_C rviz:=true initialization_config:=/home/jian/livox_ws/p62_r23_real_imu_closure_ws/docs/p62_r23_real_imu_closure/config/w3_c0_initialization.yaml start_stamp_ns:=1583837241172360960 imu_replay_start_stamp_ns:=1583837241072465152 expected_boundary_imu_stamp_ns:=1583837241068926621
```

C3必须同时改用对应窗口的`*_c3_initialization.yaml`；其中EKF和NDT种子已经一致扰动。其他窗口、扰动幅度或门限不属于本轮复现范围。

## 日志与验收

`r23_ndt_lineage.csv`记录真实EKF预测样本、查询时间、原生优化、实际地图观测发布时间及目标地图身份；`r23_admission.csv`、`r23_candidates.csv`保存全部质量/候选决定。`publisher_receipts.csv`以显式地图观测发布状态确认融合回执，拒绝观测不会伪造NDT测量。

`ekf_prediction_diagnostics.csv`记录真实IMU传播和状态修订；`deskew_diagnostics.csv`记录实际Deskew；`oosm_statistics.csv`记录历史应用时间、对齐误差、真实IMU重放以及APPLIED/拒绝状态。`local_map_diagnostics.csv`记录实际tiles、目标点数与身份。若各策略轨迹导致局部地图不同，必须按这些身份解释比较。完整数据指标以REPORT及封存后的评价为准，节点启动和RViz显示都不能单独证明精度或闭环通过。

G1已有延迟预测限制：历史高频EKF样本因OOSM修订后未重新发布历史样本，后续TP可能落后最新地图校正一个修订；记录仍然因果，但不能称最新后验状态同步无延迟。参考为条件性Base解释下的地图锚定Benchmark，不能称独立动捕真值。

正式对照10条×60输入扫描已单独完成，不由本README重复启动。实际formal原生调用432次（420名义+12额外）；部分输入扫描在继承的优化前几何门被拒绝，因此不能写成600次真实align。另有原基准Smoke20次和GUI验收20次BASELINE，总计472次Full align，extra仍为12。GUI回放不用于正式精度统计。

## 机器狗rosbag输入合同（尚未验收）

本入口固定Newer College资产；R23没有执行机器狗rosbag回放，也没有验收机器狗C/D闭环。机器狗输入至少必须先满足以下已有接口合同，不能直接套用Newer College外参或初始化文件。

|项目|要求及对应参数/接口|
|---|---|
|真实LiDAR|`topics/lidar`与`topics/lidar_msg_type`匹配实际`livox` CustomMsg或`pointcloud2`类型；若走外置Deskew，原始点云进入`deskew/input_topic`，Deskew输出与NDT订阅一致|
|真实IMU|`topics/imu`匹配原始IMU话题；角速度rad/s、加速度m/s²、时间严格递增，保留真实测量|
|点时间|明确header是Scan Start还是End；点内时间字段/单位和`deskew/point_time_field`、`lidar_update/offset_time_scale`、参考时刻一致，不以接收墙钟替代采样时间|
|外参|有来源的`T_imu_lidar`，将LiDAR坐标转换到IMU；Deskew、预测、输出及EKF观测逆变换使用同一完整SE(3)，不能假设共原点或单位旋转|
|初始化|与实际bag开始时刻对应的状态、IMU边界及合法预热；不得沿用本序列anchor或未来参考生成预测|
|地图|真实机器狗固定先验地图或合法tile manifest，地图坐标系/加载参数可追溯；Newer College Leica tiles不能作为机器狗地图|
|回放时钟|bag传感器时间与ROS时钟配置一致；不得混合原采样时间与当前墙钟；实际NDT观测时间保留并经原OOSM机制处理|
|输出|NDT为地图系LiDAR观测；拒绝时仅继续真实IMU传播，不将Prediction Fallback包装成地图测量|

满足输入合同后仍须另行运行与验收；本轮不据此启动实物或新的bag实验。
