# 实际输入合同

实际文件／哈希见bag_identity.json；全部记录计数和时间见bag_inspection.json、rosbag_info.yaml。
不是根据父目录SuperLoc或单独文件名判定数据身份。
大小4500527001、Avia CustomMsg家族、双相机、双IMU和542.741秒持续时间与官方GEODE gamma描述一致。
没有取得作者checksum，不能宣布通过官方SHA核验。

## 原始字段

bag内CustomMsg schema：Header、uint64 timebase、uint32 point_num、uint8 lidar_id、uint8[3] rsvd、CustomPoint[]。
CustomPoint严格为offset_time(uint32)、xyz(float32)、reflectivity/tag/line(uint8)，ROS packed序列化19bytes。
根据bag内嵌定义核对字段顺序和类型后解析，不假设PCL内存对齐布局。
57个真实消息与生成的ROS反序列化类交叉核对header/timebase/首末点所有字段。

原bag始终不改；选中帧完整字段保存在内盘smoke_inputs／screen_inputs/raw_scan_*.npy。
XYZ PCD只给纯几何NDT使用，不表示它携带去畸变时间。对应header、timebase和全部导出SHA在export_manifest.json。
NDT过滤只复用P60原有range/voxel/cap，没有FoV或tag/line方向裁剪。

## 双时钟

官方Livox驱动参照（不是采集二进制身份证明）：
<https://github.com/Livox-SDK/livox_ros_driver/blob/3d240d5666129e1a3052e78ee8487a04b08fdda3/livox_ros_driver/livox_ros_driver/lddc.cpp>

本地official/livox_lddc.cpp第405行标注packet_offset_time单位ns，第425–430行用首包timebase和ns差；
第428行用timestamp/1e9设置ROS时间。原始引用文件完整保留CRLF，SHA为
345b85c22536c56ddc64685f1d68e2f261f08c6222851283e4326118977d64df。

本数据timebase是287–829秒的设备时钟，而header是ROS epoch。
header−timebase全段变化238ns，不应直接用timebase+offset去匹配epoch IMU。
候选header+offset(ns)的覆盖只作独立诊断，没有用于本轮Deskew；GEODE发布重定时源码尚未认证。
原始值均保持不动。原inspection里0条raw-timebase覆盖不能解释为IMU传感器坏或全段丢失。

## 实际IMU与准入

- /imu/data：54276条，imu_link，严格递增；中位dt10.399580ms、最大15.870094ms。
- /livox/imu：109966条，livox_frame，严格递增；中位dt4.997254ms、最大5.109549ms。
- 不能把中位周期倒数与整段平均采样率混为一谈；完整起止时间和数量已经归档。
- header锚定条件下完整扫描覆盖分别5427／5425；内置IMU首末扫描边界不完全包围。
- 这不授权选择某IMU做Deskew。所选IMU外参、精确点时间参照和地图必须全部通过。

本轮选择的Deskew IMU：**无**。未估计新外参、未默认单位阵、未修改IMU数值或执行融合。
