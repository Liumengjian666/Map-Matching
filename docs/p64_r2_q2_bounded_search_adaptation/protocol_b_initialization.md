# Protocol B · 独立启动与因果短窗

**参考独立的短窗协议已建立并运行。** 以独立 Nav A Diff 测次的先验地图和 Omni 原始传感器为输入，完成 30 帧参考独立因果跟踪。光学动作捕捉参考在独立封存后才读取。可评价的 28/30 帧全部通过 0.15 m 与 2° 的联合门槛。窗口起始两帧缺参考包络，因此初始位姿及第一次校正本身的绝对误差仍未测。该窗口是同一公开测试序列上额外的资格检查，用于确认链路，与 Q2 的 12 帧比较集分开。没有建立未接触场景的留出集，也没有现场机器狗运行。

## 历史预测来源与协方差边界

- Hard 源为 P63-R10 的 NATIVE_NDT.csv.gz，Omni 源为 EXP-116 FORMAL_A 的 NATIVE_NDT.csv。两次有效配置均为 use_imu=true、enable_robot_odometry_prediction=false。hdl_localization_nodelet.cpp 226–265 先按扫描头时间消费可用 IMU（惯性测量单元）再执行校正。pose_estimator.cpp 80–100 对控制输入进行 UKF（无迹卡尔曼滤波）传播，143–156、189–190 将未启用车轮里程计融合时的 matrix() 直接传给 NDT（正态分布变换）。205–207 执行地图观测校正。由此确定 T_pred 是先前地图观测校正过的状态，经真实 IMU 因果传播后的本帧匹配初值。
- 对 Q1 12 帧，把每帧猜测与最后一条原生 IMU_PREDICT 状态均值按纳秒连接。12/12 平移逐数值一致，四元数到旋转矩阵差的最大转角小于 3.65×10⁻⁶°。历史初始化在更早帧继承参考辅助。车轮里程计没有进入该分支。 [逐帧来源复核](historical_prediction_provenance.csv)
- PoseEstimator 内部有 16×16 状态协方差。位置、速度、四元数及偏置的组合协方差依赖过程模型和每次更新。源码不存在可直接读取的公开时间戳配套协方差；publish_odometry 的普通输出只填位姿与部分速度字段。IMU_PREDICT、NATIVE_NDT 和 Q1 输入均未记录本帧先验协方差。模型常量、消息默认零矩阵、最终状态协方差及 NDT Hessian 的逆无法替代它。本轮正式搜索用冻结的确定性扰动。后续如引出内部矩阵，应在地图更新前按时间戳记录，明确将四元数扰动雅可比映射到选定 SE(3) 切空间，再用独立数据校验一致性。

## 输入、绝对启动和因果协议

- Nav A Diff 独立地图由该建图测次轨迹制成，地图 SHA-256 为 c72c5d6f19b622759f421f05c01c3ee09433ed9967a4b2df0f313129c744f158；并未读取 Omni 光学参考来选择窗口或求种子。Omni 原始传感器包 SHA-256 为 7e10c5d139bc435c2066585c2f5d27726c40b13961a26e0236920ae62a3a1fd2。官方数据提供独立的激光雷达、IMU、车轮里程计和 OptiTrack 参考轨迹；车轮里程计主题提供相对运动信息，未作为地图绝对启动来源。 [IILABS 传感器清单](https://jorgedfr.github.io/3d_lidar_slam_benchmark_at_iilab/content/dataset/) · [建图和查询测次说明](https://jorgedfr.github.io/3d_lidar_slam_benchmark_at_iilab/content/dataset/sequences/benchmark_sequences/)
- 窗口选择规则是从原信号包第一个雷达头时间起过 1 秒的下一扫描开始。实际窗口为雷达序号 932–961，纳秒 1738773034359456539–1738773037258863449，约 2.899 s。只读取 /eve/lidar3d 和 /eve/livox/imu，未执行整包回放。对原始传感器包做了文件哈希核验。这不读取或处理全序列点云。按冻结规则去掉半径 0.2 m 内的返回与非法点，不做额外去畸变；原始包已有的点时间字段未用于矫正。
- 以首个扫描在三层平面占据栅格中作参考独立绝对粗定位。起点 (0.125, 0.125, 0.6862) m，偏航 90°，主峰 0.94089，分离对手 0.67502，间隔 0.26587，通过固定 0.55 与 0.05 门槛。所用栅格为 0.25 m、72 个全周偏航角、每角 254×256 平移单元，共 4,681,728 个平面栅格相关值。计算采用 FFT（快速傅里叶变换）批量相关，未在此过程调用六自由度搜索或 NDT 评分。粗高程来自地图平面加标定传感器高度，且不作为测量协方差。光学参考与历史锚点均未参与。 [冻结规则](protocol_b_preregistration.md) · [初始化收据](protocol_b_input_receipt.json)
- 用上述种子和首扫描启动归档 HDL PoseEstimator。种子与首次 NDT 校正使用同一扫描，应按启动处理。之后每帧仅消费不晚于本帧头时间的真实 IMU 数据，并执行一次原生 NDT。使用原生 DIRECT7、1 m、转换步长阈值 0.01 和 0.1 m 体素点云预处理。按共同 LiDAR/IMU 安装坐标输入，加速度从重力倍率转换为 m/s²，陀螺以 rad/s 输入。沿用历史 2 秒冷启动设置和起始速度零。30 次 NDT 全部报告收敛，迭代为 2–5 次，DIRECT7 最低有效点比例 0.99970，使用 580 条真实 IMU 观测，其中 180 条在冷启动期后执行主动传播。单线程本地离线时间 1.37 s，末端包括状态校正。本运行与 S1–S3 分别记账，没有启动 Q2 网格、多候选选择或任何滤波器修复。 [在线状态 CSV](protocol_b_trajectory.csv) · [运行日志](protocol_b_runtime.log)
- 通过独立封存后才读取同一光学动作捕捉参考的必要头部窗口。按照既有 Diff 地图原点和装配杠杆臂转为 T_Map_Lidar，不再拟合轨迹。首两个扫描缺少两侧参考配对，其余 28 帧中，预测、原生 NDT 终值和 UKF 校正状态分别为 28/28、28/28、28/28 正确。校正状态中位数为 0.07810 m / 0.47767°，最大 0.12166 m / 0.91763°。预测最大平移 0.14910 m，接近本轮工程容差。 [封存哈希](protocol_b_online_seal.json) · [离线评估](protocol_b_reference_evaluation.csv) · [参考传感器说明](https://jorgedfr.github.io/3d_lidar_slam_benchmark_at_iilab/content/dataset/ground_truth/)

## 复现与发布范围

在 Q2 根目录运行，输出路径须为尚不存在的绝对路径。第一条命令结束时已经写入封存。执行第二条命令才读取光学参考。

```bash
bash docs/p64_r2_q2_bounded_search_adaptation/protocol_b_run.sh /home/jian/p64_protocol_b_repro_01
/usr/bin/python3 -B docs/p64_r2_q2_bounded_search_adaptation/protocol_b_evaluate.py /home/jian/p64_protocol_b_repro_01/frozen /home/jian/p64_protocol_b_repro_01/seal.json /home/jian/p64_protocol_b_repro_01/reference_eval.csv
```

地图尾部含有对齐零填充，解析器已根据 PCD 头部的计数读取有效点并校验余额全零。首次脚本因 NumPy 整数没有转换为 JSON 原生整数而未能落盘，随后只修复序列化类型，初始化门槛及评分保持不变。首次链接尝试引入了未被窗口使用的 CUDA 注册接口，最终直接使用归档 PoseEstimator 对象和原生 NDT_OMP 库。通过后的因果运行和参考评估各执行一次。

归档 CSV 对无参考的首两帧补齐空字段，参考标签、有效帧数和数值与离线运行原表一致。
