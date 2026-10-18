# Sensor input contract — NOT established

本轮完整bag=0，实际可回放扫描=0。文件预期大小来自公开Drive元数据，不是本地校验结果。

| 项目 | 官方文档声明 | bag实测 |
|---|---|---|
| gamma LiDAR | Livox Avia | UNCONFIRMED |
| 点云话题／类型 | /livox/lidar；livox_ros_driver/CustomMsg；10Hz | UNCONFIRMED |
| 外置IMU | Xsens MTi-30；/imu/data；sensor_msgs/Imu；100Hz | UNCONFIRMED |
| LiDAR内置IMU | BMI088；/livox/imu；sensor_msgs/Imu；200Hz | UNCONFIRMED |
| LiDAR／IMU frame_id | 未由所读文字给出 | UNCONFIRMED |
| 逐点时间字段、单位与reference time | 必须由实际CustomMsg和驱动定义核实 | UNCONFIRMED |
| 扫描／IMU起止时间、覆盖、同步偏移 | 官网公布542秒仅为预期时长 | UNCONFIRMED |

不把raw bin的[x,y,z,intensity,tag,line]摘要当作已具有合法点时间的证明。
不选择Livox内置IMU来套用未知是否属于Xsens的外参，也不把二者视为同一坐标系。

访问探针：初次下载返回大文件确认HTML；一次正常确认后HTTP200、2040bytes、
text/html、Quota exceeded，未通过`#ROSBAG V2.0` magic。
没有运行rosbag info，因为不存在完整bag。没有从错误HTML重建数据。

允许后续Smoke的必要条件仍是：完整bag身份、真实消息及逐点时间、明确IMU选择和外参、
IMU合法覆盖、冻结可用地图及初始化全部成立；本轮没有满足。
