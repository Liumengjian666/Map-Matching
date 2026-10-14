# 一次有界资产清点

检查日期：2026-10-14（Asia/Shanghai）。存在性不等于全数据完整性通过。
原始文件均保持原位；未复制或删除大型数据。

## Floor01（优先检查）

根目录：`/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/`。

|资产|本轮实际证据|限制|
|---|---|---|
|raw shard0|3,422,674,923 bytes，存在|没有重新全文件哈希或全覆盖审计|
|raw shard1|3,442,739,076 bytes，存在|同上|
|raw shard2|2,783,923,015 bytes，存在|同上|
|LiDAR/IMU/Camera|已有 R7D 审计：VLP16 packets / Epson IMU /640×480 bgr8|本轮是历史元数据证据，不冒充新全段传感器验证|
|原地图 floor01.pcd|19,791,028 bytes，定位前已有|本轮仅核查存在性；不是新建评价轨迹 Oracle 地图|
|SSD H1 冻结地图|6,595,818 bytes，SHA 核实为2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570|方向/参考点合同未因此闭合|
|GT|floor01_gt.txt，178,532 bytes，存在；旧审计2061/2061与官方 SubT-MRS 同序列|原点 IMU 为历史官方 lineage；本轮未读取 GT 轨迹文件|
|相机内参/外参|MEI、LiDAR→IMU、Camera→IMU 文件存在且 SHA 重检一致|仅文件/既有语义证据，不声称已完成新视觉输入适配|
|初始矩阵|multi-floor01.yaml，1144 bytes，SHA 重检一致|H1 是实际非 GT 工程初始化；完整运行body/初始化/地图坐标链尚未闭合，不自动排除工程证据|

raw 完整路径：

```text
raw/extracted/Multi_Floor_Rosbag/raw_data_core_2022-08-18-17-16-31_0.bag
raw/extracted/Multi_Floor_Rosbag/raw_data_core_2022-08-18-17-19-00_1.bag
raw/extracted/Multi_Floor_Rosbag/raw_data_core_2022-08-18-17-21-29_2.bag
```

SSD 地图：`/home/jian/livox_ws/p6_i1_recovery_assets_20260927/floor01_h1_map_p5_frozen.pcd`。
旧 XYZ-only/状态相关 Deskew 及旧 PnP 产品不作为本轮新视觉或 Source。
原始 packet 可供未来合法解码；不因旧产品缺点否认原始数据存在。

独立性：Floor01 曾用于 R7/P6/P9 等历史研究。没有发现本轮 C 已在
Floor01 完整开发回放的证据，但也没有完成其全部开发 lineage 排除，
因此不称其严格未见 held-out。当前停止点不是凭历史使用单独排除它。

## Corridor02（备选，旧“缺 bag”结论已过时）

根目录：`/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02/`。
本轮直接 `rosbag info --yaml` 和首条真实消息读取成功：

- raw/corridor02.bag：24,494,797,838 bytes，ROS bag2，indexed，未压缩。
- record duration893.404680s，record start1724621084.660316，end1724621978.064996。
- 208,697条消息：8859 PointCloud2、178499 IMU、21339 Image。
- /velodyne_points：frame cmu_rc1_velodyne；首header1645999726278098106ns；
  28131点，含float32 XYZ/intensity/time及uint16 ring。
- /imu/data：frame epson；首header1645999726442864000ns。
- /camera_1/image_raw：frame d；首header1645999730701529211ns；
  640×480 bgr8，921600字节。

record 时间是2024年、sensor header 是2022年。历史 GT 时间范围
1645999726.984117–1646000619.653331 与 header 轴相容，不能与 record
轴直接比较。本轮未验证全段 header 时间覆盖或完整 bag SHA。

已有原地图66,056,109 bytes、GT475,805 bytes、内参478 bytes、外参860 bytes、
初始矩阵1011 bytes。三个标定/初始化小文件的 SHA 已重新核实。
地图/GT 的全文件哈希仅有旧审计记录，不标成本轮重新核实。

仍未闭合：发布 GT 的传感器参考点，以及初始 YAML 的source/target、
传感器/body关系和首扫描关联。无需字面时间戳或官方唯一证明方式。
不得继续把 Google Drive quota 当成当前最小阻塞。

## 其他现有目录

对 FusionPortable、M3DGR、ENWIDE、GEODE、NTNU_LiDAR_Degeneracy 现有根
目录进行了 maxdepth2 的 .bag 元数据搜索：未找到该范围内的 raw bag。
多个目录仅有官方 metadata checkout。本结论限于这一浅层范围，
不声称磁盘任何位置都没有其他数据。不为寻找结果无限扩大搜索。

官方页面本轮访问：[SuperLoc](https://superodometry.com/superloc.html)。
它提供地图、轨迹、标定与初始化配置链接；TUM字段格式本身不能确认
上述发布轨迹的传感器原点或初始矩阵的具体方向。

结论：传感器资产不是全部缺失；未发现已通过全部准入合同的一个场景。
`selected_scene=null`；`FINAL_RESULT=DATA_NOT_READY`。
