# 地图与融合准入状态

## 地图

复用R1已冻结的官方公开目录证据：
<https://drive.google.com/drive/folders/1WjJ5RpZg1IsiYpbwBvn0wBVXsPuUoSdX>。
当次只列stairs.las；本轮没有重新下载、不把它当盾构隧道地图。
实际bag只有LiDAR、两路IMU、两路相机，无map、tf、odom或GT轨迹话题。

没有可认证的独立同场景地图、地图实际XYZ或合法T_map_lidar起点。
局部100个相邻扫描匹配不能组成覆盖全段的可信全局轨迹；未在空缺处串联位姿。
因此不能认证可分离建图段与测试段的全局空间重叠。
没有读取测试GT选段／建图／拟合世界变换，也没有生成同序列自地图。

## 外参

实际LiDAR和内置IMU均声明livox_frame，不能仅凭字符串相同认定T_I_L=I。
官方Livox参照源第487行也直接给IMU填写livox_frame，这不是硬件外参认证。
外置Xsens为imu_link，bag没有TF给出其与LiDAR关系。
R1 gamma_config.yaml第48/59/65行的话题／线数与本包实测不匹配，未应用原矩阵。
没有任意外参搜索或采用机器狗／其他数据集外参。

| 准入项 | 当前状态 |
|---|---|
| 原始LiDAR完整与schema合法 | PASS |
| 原始offset_time保存、单位ns | PASS；ROS epoch锚定另需认证 |
| 所选IMU及T_imu_lidar | UNCONFIRMED；未选择用于Deskew |
| header锚定的IMU边界覆盖 | 已测，见time_contract_audit.json；不是完整融合批准 |
| 独立隧道先验地图和XYZ | UNCONFIRMED |
| T_map_lidar／T_map_imu初始关系 | UNCONFIRMED |
| Deskew+NDT+EKF/OOSM Smoke | NOT_RUN |

所有矩阵方向仍采用p_A=T_A_B p_B。若各项将来认证，才可使用
T_map_lidar=T_map_imu T_imu_lidar。当前不填任何无来源矩阵。

正式完成项是RAW_NATIVE_LIDAR_GEOMETRY_DIAGNOSTIC；不是先验地图定位或独立精度实验。
