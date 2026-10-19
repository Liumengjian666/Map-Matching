# 时间来源链及物理同步证据缺口

## 判定

**CLOCK_SYNC_NOT_CERTIFIED**。这是证据不足，不是已证明不同步。
没有用角运动残差拟合时间差，也没有重算R10。

固定窗口：1706671804799723787–1706671805899593546ns。
时差统计按header准入，实际检查5155–5165共11条LiDAR、110条外置IMU。
5154仅为R10前驱姿态，不声称复查了它的LiDAR消息。
前驱IMU1706671804791549444单独核验原始/R8序列化SHA，排除在时差统计之外。

## 来源链

T_A_B表示B到A；本审计不产生任何新位姿。下列R8路径相对仓库根目录。
所有引用文件的绝对路径、大小、SHA见三个before输入清单及analysis_manifest.json。

|环节|实际证据与代码|单位及规则|已核验事实|仍未知事项|
|---|---|---|---|---|
|Xsens采集设备|归档GEODE_README.md:38–41；原始bag /imu/data|作者gamma说明为Xsens MTi-30；sensor_msgs/Imu.angular_velocity按ROS合同rad/s，imu_link|实际类型、frame、消息SHA与R10冻结样本一致；不是/livox/imu|具体序列号、固件、录制驱动版本、设备时钟字段和主机转换实现未绑定|
|Xsens设备→ROS时间|原始bag消息header；连接元数据record_audit_complete.json|ROS sec+nsec，整数ns|选定110条及单独前驱header与R10一致|只有最终header，没有独立原始设备时钟；无法恢复实际同步过程|
|Livox采集设备|归档GEODE_README.md:38–41；/livox/lidar|Livox AVIA，livox_ros_driver/CustomMsg，livox_frame|选定11条原始序列化SHA、timebase、uint32 offset及scan_end均匹配冻结扫描清单|录制时SDK/驱动/固件、同步接线及会话配置未绑定|
|Livox设备→ROS时间|原始CustomMsg的timebase和header；lidar_timebase_audit.csv|二者原点不同；点时间合同header_ns+offset_time_ns|header-timebase差范围1706671002200386209–1706671002200386399ns，共190ns变化|该原点转换由何录制程序实施、误差界及跨传感器物理偏移未认证；此差不是跨传感器偏移估计|
|参考驱动，不是录制驱动证明|归档official/livox_lddc.cpp:411–430、470；上游3d240d5666129e1a3052e78ee8487a04b08fdda3|GetStoragePacketTimestamp→首包timebase；同一timestamp/1e9→header；后包差→ns offset；bag.write传timestamp|参考实现源码内容可复核；SHA345b85c22536c56ddc64685f1d68e2f261f08c6222851283e4326118977d64df|不能证明实际录制用了此版本；它不能单独解释实际bag中header/timebase不同原点。没有用网上最新版替代录制版本|
|ROS header→原始bag record|原始bag选定topic/header/record；timestamp_records.csv|header_ns-record_ns，精确整数减法|11 LiDAR、110 IMU的min/median/P95/max均0ns|无法证明record是物理到达时间或未经过重打包；零记录时差不等于物理同步|
|原始bag→R8适配输入|实际1367e1d9587d1f90a47bf6f44405239a7b65e4f6版本prepare_inputs.py:58–83|IMU消息原样；dest.write时间显式用header；CustomMsg→PointCloud2保留uint32 offset，time=float32秒|IMU序列化SHA一致；适配点data SHA与R8冻结adapter SHA一致；header/frame未变|适配程序不构成硬件同步校准；prepared record时差0为构造结果|
|R8适配时间→scan-end|同程序:39、74–83；R8 manifest与R10 interval CSV|end_ns=header_ns+max(offset_ns)，整数ns；输出time字段ns×1e-9秒|11条全部整数scan_end与冻结清单一致；未使用设备timebase当epoch|R11没有重新执行Deskew、OOSM或R10积分；继承其冻结结果而非新认证物理同步|
|R6算法时间配置|实际R6 bridge_run/config.yaml:4–5；laserMapping.cpp:312–360|time_sync_en=false；time_offset_lidar_to_imu=0.0；回调ROS double秒往返|内部自同步分支未启用，未配置人为时间移位；native double表示可有ns舍入|关闭软件自同步/配置0不代表硬件已同步，也不代表硬件未同步|

## 独立证据及负面搜索范围

原始和R8连接信息没有callerid和latching（明确为NOT_RECORDED），不足以绑定录制驱动。
仅在这两个bag的topic元数据中检查pps/ptp/time_ref/sync/temperature关键词，没有匹配项。
未读取其他topic payload。关键词为空不证明设备没有内部同步或温补。

作者README:77存在同步方案的笼统说明，涉及多设备采集说明；没有随本序列绑定的
PPS/PTP/共同触发配置、误差测量日志、时钟映射代码版本或同步会话记录。
gamma_drive.yaml/fastlio_carol.yaml提供话题、外参与算法时间设置，但不提供上述证据。
已有本机Livox驱动目录不能追溯到2024录制会话，未冒充采集版本。

因此只确认：格式/ROS时间表达一致、指定消息身份一致、可观测记录时差为零。
物理时钟同步精度没有获得独立认证，不能使用零时差把这一缺口填平。
