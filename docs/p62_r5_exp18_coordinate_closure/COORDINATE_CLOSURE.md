# 坐标与标定：已闭合部分和硬缺口

约定：T_A_B 把 B 中的点变到 A。M 是应用 E57 保存的扫描 pose 后的根坐标；S 是该扫描局部系；W 是发布轨迹的世界系；I 是 Alphasense IMU；L 是原始 PandarXT-32；P 是 tip。

## 官方 Issue #4 与修正 URDF

[Issue #4](https://github.com/Hilti-Research/hilti-slam-challenge-2022/issues/4) 中作者 mcamurri 于2022-05-04确认原始雷达轴采用制造商约定，不额外旋转消息数据；链接修正 commit `64633725c701c2763c5787257f4e32b7bc3067d9`。
固定 `sources/hesai_issue4_fix.xacro:55–61` 把 hesai_base→pandar 定为 z=0.0464m、yaw=pi/2，明确注释X向左。2022-05-17 参数化修正 `ad175515d1d1ad4f5a0905b4fb57770d635928c5` 保留数值并允许 `PandarXT-32` 帧名，`sources/hesai_parametric.xacro:55–61`。

结合 R4 已冻结 phasma commit `79002026e9b229cf9b4e95a403eaec4f00acfd79` 和官方 Alphasense commit `80c1b8675b3b9e03f9de9d1b5f420e5225f67a50`：

- T_P_mount：[-0.065253,0,0.179]m，单位旋转，phasma.xacro:33–35。
- T_mount_I：[0.006,-0.00855,0.0174]m，Rx(pi)，alphasense.xacro:61–64。
- T_P_hesai：[-0.06,0,0.095]m，单位旋转，phasma.xacro:69–71。
- T_hesai_L：[0,0,0.0464]m，Rz(pi/2)，hesai_parametric.xacro:55–61。
- T_I_L = inverse(T_P_mount*T_mount_I) * T_P_hesai*T_hesai_L。

纯代数得 URDF 的 R_I_L = [[0,-1,0],[-1,0,0],[0,0,-1]]，t_I_L=[-0.000747,-0.00855,0.055]m。

## MOLA 实际消费链，不是执行结果

固定 MOLA lidar_odometry commit `6e905f0dcec1221be94b747850fba09435db2605` 的 `sources/mola_hilti2022.sh`：

- 17–24行注释明确按发布 YAML 解释四元数 xyzw、T_imu_pandar。
- 53–55行选择 `/hesai/pandar`、`/alphasense/imu`；71–90行默认IMU为基准、固定雷达位姿，不依赖 bag TF 别名。
- 78–83行使用 [-0.001,-0.00855,0.055]m，yaw=-90°、pitch=0°、roll=180°。
- 93–106行的 LiDAR 基准分支使用其逆；没有把原数组按wxyz套用。
- 128–132行是里程计首位姿规范固定，不是 TLS 地图中的真实初值。

同 commit `sources/mola_actual_rosbag1_launch.yaml:108–114,127–133` 将这些值传给 fixed_sensor_pose，字段顺序为x y z yaw_deg pitch_deg roll_deg。
固定 loader commit `d1079e427343b5eff108dce4e9ff34008aadda15` 的 `sources/mola_Rosbag1Dataset.cpp:605–612` 将其解析为 CPose3D；1218–1238行在固定值存在时直接采用，否则 lookupTransform(referenceFrame,frame)。1278–1282和1489–1495行分别设置点云和IMU时间及sensorPose。

**重要区别**：profile是第三方当前固定消费者，硬编码了发布标定值，并不是本数据录制时驱动，也没有在此处动态读取原始 lidar_calibration.yaml。这里只读源码，从未 source/执行这些shell、launch或二进制。

YAML 的 xyzw 得到同一 R_I_L，平移为发布值 [-0.001,-0.00855,0.055]m；MOLA与YAML数值一致。URDF 与 YAML 的X平移差0.000253m（0.253mm），不能冒称逐元素相同。可能是几何尺寸/取整差异，物理根因没有认证。不得把URDF数值无说明替代发布标定。

因此，**T_I_L的方向、四元数排列和雷达轴语义有交叉来源支持，局部标定语义缺口显著缩小**；不是新的实物外参精度认证，也不解决地图世界系。

## GT I / base 姿态仍未完全闭合

官方 R4 FAQ明确说GT在I；MOLA亦按I消费。两项是支持I语义的证据。
但固定 phasma.xacro:58–66仍注释GT在base，且T_I_base=Rx(-pi)、零平移。因此I与base不是可互换的姿态。

两种分支必须保留：若文件是T_W_I，不追加Rx(pi)；若是T_W_base，则T_W_I=T_W_base*inverse(T_I_base)。本轮没有生产GT文件的确切导出代码或权威更正来选择这一分支；MOLA的解释不是发布端文件生成证明。不得按轨迹外观、误差或重力选择。

tip的情况还应区分**参考点位置**与**tip坐标轴**：URDF推得t_I_P=[0.059253,-0.00855,0.1964]，发布YAML为[0.059,-0.00855,0.1964]，位置相差0.253mm；URDF tip姿态相对I为Rx(pi)，YAML measurement_tip姿态为单位阵，相差180°。单个物理测量尖端的位置与其人为坐标轴不能混用。稀疏文件是否已扣除杆臂，仍无发布消费者证明。

## T_M_W：本轮唯一决定性坐标阻塞

R4已解码E57：单合并扫描、285455240点；coordinateMetadata为空，CloudCompare导出，保存非单位T_M_S（约112.23m/21.41m平移及非零yaw）。这些是文件内部S→M，不是W→M。
论文Sec.V-A/V-C说明TLS地图用于生成稠密参考，支持两者存在注册关系；未给出该固定发布E57根系与导出GT世界的数值等同声明、导出程序或T_M_W。

补查官方Issue #20、#21、#23：#20的作者回复承认曾更换construction场景地图；用户称Sheldonian视觉上可用，**不是作者认证固定文件T_M_W的证据**，不能把不同场景的问题套到Exp18。#21未给出世界系合同；#23仅询问单站文件，没有可用发布变换。MOLA上述profile/loader只设置传感器相对基准，不加载这个TLS或认证其与GT的注册关系。

所查范围内仍没有可引用的T_M_W数值或同系声明。不能从“同一场景”“GT由先验地图生成”“E57 bounds看似包围轨迹”推导T_M_W=I；也不尝试令T_M_W=T_M_S或其逆。
这不是证明数据错误，而是证明本轮准入证据未闭合。停止使用Exp18做正式先验地图绝对误差对照。

## 核心链状态

| 链 | 结果 | 不可升级的结论 |
|---|---|---|
| T_I_L | 发布YAML方向/xyzw有官方几何及第三方消费者支持 | 未测得物理外参误差 |
| T_I_base | 官方URDF明确Rx(-pi) | 不代表GT已在base或I |
| T_W_I | 官方FAQ/MOLA支持I，发布导出与URDF冲突未消除 | 不按数值拟合选分支 |
| T_M_S | R4文件内部pose已知 | 不等于T_M_W |
| T_M_W | NOT_ESTABLISHED | 无恒等占位，无拟合 |
| T_I_P | 位置/轴差异分别保留，稀疏参考消费仍开放 | 不用占位姿态作真值 |
