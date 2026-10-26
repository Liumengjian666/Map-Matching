# LiDAR–IMU外参来源与等级

T_A_B从B到A。FAST-LIVO2 `extrinsic_T/R`与Super-LIO `g_lidar_imu`方向均为**T_I_L**：p_I=R_IL*p_L+t_IL；方向由实际源码确认，**CONFIRMED（软件接口）**。

| 材料 | 实际内容 | 证据等级 |
|---|---|---|
| 用户提供原目录loop2_config.yaml／R25快照 |零平移／单位旋转；注释明确缺少单独标定时使用此假设 |CONFIRMED配置内容；物理外参UNVERIFIED |
| own_loop2.yaml |T_imu_lidar.status=UNAVAILABLE_NOT_PHYSICALLY_CALIBRATED |CONFIRMED历史缺口声明 |
| P63-R1用户授权 |允许与历史一致的工程Smoke，不允许视为标定精度实验 |CONFIRMED授权范围，非标定证据 |
| 同一YAML的Rcl/Pcl及perm8文件路径 |相机–LiDAR转换，不是IMU–LiDAR标定 |CONFIRMED参数角色；不能补齐T_I_L |
| 当前driver2 MID360_config.json |零roll/pitch/yaw、x/y/z的extrinsic_parameter配置 |CONFIRMED文件内容；没有设备物理IMU杆臂标定证明 |
| driver2 Lddc::InitImuMsg |原gyro/acc复制到ROS消息，frame写livox_frame |CONFIRMED该函数行为；相同frame字符串不认证物理同轴／同原点 |
| 原Super-LIO示例 |非零杆臂[-.011,-.02329,.04412] |CONFIRMED官方配置，未绑定此设备，未采用 |
| 安装／标定文件只读核查 |当前具名标定目录不存在；稳定src具名imu/extrinsic/calib/URDF搜索未找到适用记录 |UNVERIFIED物理标定；不声称全磁盘绝对不存在 |

零/I的来源可以追溯到“历史手工运行假设”，无法追溯到物理测量、厂家该设备定义、安装CAD或已验证标定。当前唯一合理物理等级是**UNVERIFIED**。

FAST-LIVO2配置将R按行解释；Super-LIO Eigen M3(data)按列主序解释。单位阵不受影响，但一般矩阵需要同一方向与存储顺序核查。平移方向和单位不能仅靠配置名称推断，本轮已由p_I=R*p_L+t确认。原配置相机/激光旋转不能挪作IMU/激光旋转。

没有以38.02°、ICP终值、轨迹误差或旧地图几何拟合外参；没有采用其他设备示例。要升级验证等级，需可对应这台设备与实际消息轴的安装／工厂／独立标定依据及其方向、单位、版本。即使坐标数学自洽，也不能替代这个来源条件。
