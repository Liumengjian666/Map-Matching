# R10完整Hard输入和预注册合同（零NDT后、正式运行前冻结）

START 6e2b67a6bffa81eb4d70667fdf6e604c56ae4959；已核对远端research/p63-r9-hdl-real-imu-prediction同SHA。独立分支research/p63-r10-hdl-imu-full-hard。

## 完整输入

Koide indoor_hard_01，Azure Kinect depth_camera_link瞬时深度点云＋imu_link真实测量。重读原ROS2全部2379条点云、44398条IMU；原topic CDR聚合SHA与R29一致，全部点云XYZ/PCD字节、原header/bagstamp/frame以及所有IMU六个值与整数时间戳均匹配原转换。没有新下载、重新转换数据、外参估计或地图拟合。详细身份见PREFLIGHT/INPUT_FREEZE和RAW_IMU_VERIFIED。

全序列raw1首stamp=1690354429463785391ns，raw2379末stamp=1690354593400954808ns。保持raw4=1690354429663743949ns作为唯一一次Oracle Start，沿用R9完整T_M_L与零初速。正式raw5–2379共2375帧，时间跨度163.670545635s，不使用前4帧配准暖启动，也不重置UKF。

完整IMU从1690354429603963391至1690354593481012582ns，严格递增；anchor之后且≤末扫描共有44267条拟入原生缓存，预期41000次实际传播、3267次cool-time/首次跳过。预检只模拟时间判定，并非实际调用证明；后续必须以原库trace认证。完整IMU间隔Mean3.691174/P9514.722/Max102.216553ms，47个>50ms间隔；扫描最后IMU年龄Mean6.617535/P9522.004312/Max83.388545ms，无新IMU的扫描raw68和1451，不删除/补造。IMU覆盖扫描整个目标范围。

## 冻结物理和框架

完全继承R9 IMU_INPUT_CONTRACT、HDL_NATIVE_IMU_SOURCE_ANALYSIS，不重新定义模型。T_A_B将B变换至A，官方T_L_I=[.003,.004,-.051,-.476,.474,.524,.525] xyz/xyzw、四元数只归一化。ω_L=R_L_Iω_I；角加速度由相邻真实gyro后向差分，f_L=R_L_I f_I−α_L×p_L_I−ω_L×(ω_L×p_L_I)。tools/imu_geometry.py与R9逐字节一致，SHA 19adb542534ebff22ba08ee98a84854dbc809d2295f601e049e49fe9f6570739。

SI比力及角速度、不翻号、不额外去重力、无插值/时间平移/额外滤波。原HDL状态与Bias/重力假设、2s冷启动、只传播至最后IMU以及校正不重设prev_stamp的时间近似原样保留。此前用户已接受，本轮合同继续冻结，不声称精确预积分/OOSM。全段差分杆臂项最大23.486314m/s²是有噪声离散转换量，不能改平滑或阈值追求效果。

地图仍为R8/9相同Hard Tile合并PCD96324点，输入SHA50bba09fde204a55be8261e69ea539ae69c65dd44ec7ca1eeed5b066ff8da098。复用原.1m VoxelGrid：实测93223点，Target XYZ SHA166c4d2a4f5d24a4c79d4c90a8a275e0fe905fcc9650d559aabb775780e81654。全2375 Source用相同原生预处理做零align身份冻结；前160现场预检身份与R9完全一致。每帧正式trace再次核对，不在线更新地图。

官方HDL de2dc7769aa2412876d05ae6161408d3295b6a3c。原版NDT_OMP/UKF、R9只读trace及六份原库复用，不重编译运行库。resolution1m/DIRECT7/epsilon.01/max35/step.1/outlier.55、scan/map voxel.1m全部冻结。IMU开启；global、robot odom、C+G、八方向搜索及新增门控全部关闭。

## 唯一运行与停止

full_01独占目录，一次2375帧上限，每帧一次align；同一nodelet/UKF实例一直存活，无initialpose回调或全局重置。原IMU按header升序发布，原imu_callback返回入缓存回执后才发当前扫描。只读trace必须证明实际消费/传播、NDT guess、终值和UKF输出的对应关系。质量低/未收敛/高误差都不能触发新质量门或提前结束；在线不读取测试参考位姿。

仅非有限核心、输入/地图身份违约、原生程序不可继续、不可恢复ROS/监控故障或磁盘<20GiB停止。未完成时保留实际调用、输出和最后有效状态；缺失输出不填零，不重跑。完整日志追加写入，避免全序列逐帧重写既有CSV；这只是记录效率调整，不改变任何传感器时间或算法状态。

## 参考与RPE（事前冻结）

官方Hard TUM为T_M_L，地图锚定离线GICP+IMU，非独立动捕。继承R9参考SHA3696eddcbe884af197b13b8205952793b55c37d9093ab95b96a781ae56c6eeea。运行前只读参考第一列时间及文件字节SHA，不解析测试位姿。

延用原Reference.at：位置线性/旋转SLERP；bracket≤150ms、最近参考端点≤50ms；边界附近保留R9实现的端点规则。无SE3拟合或时间偏移。预检2372/2375可评价（99.873684%），raw748/1691/1692不满足原插值合同，不能因近似同stamp就事后放宽门限。这3帧仍必须处理并保留输出，参考误差留空。失锁判定对参考缺失帧中断连续证据计数，不以缺失证明恢复。

1秒RPE：对每个原输出扫描t_i，固定目标t_j=t_i+1,000,000,000ns；不选择按误差有利的终点。若t_j超过末扫描不配对。估计终点在原输出扫描中位置线性/旋转SLERP，原扫描端点bracket≤150ms且最近端点≤50ms；参考在t_i/t_j均使用相同原Reference.at。只在封存后执行，估计/参考均无对齐。E_ij=(T_ref,i^-1 T_ref,j)^-1 (T_est,i^-1 T_est,j)，报告其平移范数m、旋转角deg的Mean/RMSE/P95/Max，不混成绝对位置误差或m/s。候选配对已仅用时间戳冻结到RPE_PAIRS.csv：2281对，其中预计2278对参考合法。缺失/不合法端点报告原因，不能补造姿态或替换配对。

## 判定和诊断边界

完整、核心有限、参考覆盖≥95%、位置RMSE≤.30m、旋转RMSE≤5°且无持续失锁、无泄漏/重置/调参，才为FULL_SEQUENCE_BASELINE_PASS。运行不完整优先FULL_SEQUENCE_EXECUTION_INCOMPLETE；完整但参考不足/不合法为FULL_SEQUENCE_REFERENCE_LIMITED；合同成立且完整精度/持续失锁不通过为FULL_SEQUENCE_LOCALIZATION_FAIL。原高误差>.5m或>5°，连续3有效高帧确认，连续3低帧恢复，不短暂关闭事件。

首个持续事件出现时，仅分析其前10帧和从起点起20帧已有记录，不新增NDT/Oracle、调参或候选。约150MiB定位进程RSS单独判定，不与精度混合。握手回放及callback计时不认证10Hz。保留真实在线RViz和完整Path，不拼接参考或候选轨迹。
