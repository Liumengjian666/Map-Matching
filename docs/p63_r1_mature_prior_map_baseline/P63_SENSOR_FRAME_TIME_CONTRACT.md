# 传感器、外参和时间合同：条件性工程 Smoke

## 原始资产和验证等级

原 bag 为 `/home/jian/rosbag/loop2/raw_restore_20260914_stage0/loop2_raw.bag`，19677401800 bytes，历史冻结 SHA256 `d17bee3f8697dcb01cb4b9f6ea2483aa42a1a31ae277659c8a7869d9c69f13ae`。本轮只读索引及有限原始消息，不重复全 19.7 GB 哈希；这是归档身份加本轮前缀检查，不冒充新的全文件认证。

bag 有 10905 LiDAR、218101 IMU，记录约 1090.511421 s。预检首 600 扫描／12200 IMU，双方 header 分别严格递增；最大 IMU 间隙 8.149385 ms；加速度模长中位数 0.9937207894（原始约 g 尺度）。详见 `evidence/LOOP2_READ_ONLY_INPUT.json`。

实际原始建图配置存在：

`/media/jian/HIKVISION/Jian_FASTLIVO_Data/dogrosbag/loop2_map_match_experiment_2026_08_08_17_36_19/config/loop2_config.yaml`

与 Git 历史快照 `docs/p62_r25_results_to_paper_closure/dog/ACTIVE_MAPPING_CONFIG.yaml` SHA 相同：`1cde331d3b092629e1c2b06ebf218a9c51122fe4f8feeef3ea6e9ef2fafae76c`。值为 extrinsic_T=0、extrinsic_R=I，gravity_align_en=false。**这是历史运行假设，不是物理标定。** 该配置在 8 月 8 日目录，而指定 PCD 是 8 月 12 日 simtime rebuild；现有配置不能构成此 PCD 生产参数的唯一文件级证明。没有为弥补这个空缺拟合地图变换。

用户已明确确认缺少独立标定，并授权同历史零/I的工程 Smoke。只读搜索稳定工程 src、现有标定目录及已知 FAST-LIVO2 配置，没有找到适用的设备验证结果。`own_loop2.yaml` 的 UNAVAILABLE_NOT_PHYSICALLY_CALIBRATED 保留；作者 MID360 示例杆臂及其他 Avia/NTU 示例不用于此设备。

## 方向合同

T_A_B 将 B 中点转换到 A。L=Livox LiDAR，I=IMU，M=原始 FAST-LIVO2 camera_init 地图。

- FAST-LIVO2 `RGBpointBodyLidarToIMU`：`p_I = extR * p_L + extT`。
- Super-LIO `g_lidar_imu`／Propagation_Undistort 同样是 **T_I_L**；名称不是取逆要求。
- FAST-LIVO2 YAML R 按行解释；Super-LIO `M3(data+3)` 按 Eigen 列主序解释。单位旋转两者相同，一般非对称 R 不能不检查便直接复制。
- 本轮唯一运行假设 `T_I_L=I4`，米；`T_M_L=T_M_I*T_I_L`。不是另用作者非零平移。
- 原生 `/lio/odom` 给 **T_M_I**，不是 T_M_L；child frame 原本为空。独立 TF 桥仅将同一数值发布 world→p63_imu，再以固定 T_I_L 发布 p63_imu→livox_frame，无反馈。
- M 数值坐标原样别名为 world；没有 SE3 拟合、轨迹对齐或新地图。物理重力轴是否对应原作者 world -Z **未验收**。
- 初值是固定人工地图原点和 yaw=0，roll/pitch 由原作者平均加速度处理，再经过原生 NDT＋ICP；没有读历史／未来位姿作为在线修正。

## 原始消息与时间

`/livox/lidar` 是 driver2/CustomMsg，上游接收 driver1/CustomMsg；两者 MD5 均为 `e4d6829bdfe657cb6c21a746c86b21a6`，真实 TCPROS 连接和运行证明兼容；不改消息点坐标或时间字段。

原生逻辑：header.stamp 为 scan start；保留点 offset_time uint32 纳秒乘 1e-9；原作者用**最后一个保留点**时间构造 scan end，并非主动求最大 offset。首消息 header=1786179479409757376 ns，timebase=1786179479409757444 ns，相差 68 ns；本轮未将 header 替换为 timebase，也未估计时差。点序的 offset 不是全局单调排序，沿用原作者逻辑，不宣称重新认证了所有点的绝对采样时刻。

重放按 bag record 顺序及节奏原样发布，有界前缀；不发表任何 pose 输入。IMU 原始加速度与角速度原样传递，作者初始化学习 `imu_scale=g/||mean_acc||`，不是本轮外部乘 9.81。官方 driver2 直接复制设备 acc/gyr 数值；作者以 rad/s 解释 gyro，并把初始化均值约1的 acc 用 imu_scale 转为其配置的 m/s²。模长及源码行为支持这次接口使用方式，但不构成独立传感器校准。不能因同为 livox_frame 而假定硬件无杆臂。

同步等待最新 IMU 覆盖 scan end；提取不晚于 end 的 IMU。Propagation_Undistort 使用原作者状态列表和 T_I_L；GetNavState.timestamp 为 ESKF current_time_，**不能据消息频率直接当作严格 scan end**。本次未重写插值、末端外推或 Deskew，未改 IMU/ESKF 数学。95 输出时间单调且有限，只证明这次输出数值顺序，不证明每个原生 deskew 时刻已独立验收。

## 失败后只读线索

`initialization_consistency.py` 从首 10 原扫描／185 IMU 重建 58407 个初始化点，数量与原日志完全一致。平均 acc 为 [-0.39737848,0.02562227,0.91068241]；用日志六位精度 ICP 旋转转换后，归一化比力在地图中为 [-0.37781924,-0.48646515,0.78778441]，距 +Z 38.021059°。在近似静止、作者固定重力 -9.7946 Z 的前提下，隐含净加速度约 [-3.7006,-4.7647,-2.0786] m/s²。

这是**初始化／地图重力合同存在明显分歧的诊断**，不是独立标定、真值或唯一根因。可能涉及地图不重力对齐、ICP 错分支或物理旋转假设，尚不能区分。没有据此旋转地图、改滤波重力或换初值重跑。ATE/RPE=`NOT_EVALUABLE`；历史同系建图轨迹不能当独立 GT。
