# GT参考点证据与反证边界

| 证据 | 本轮复核结果 | 能证明 | 不能证明 |
| --- | --- | --- | --- |
| 本地`corridor01_gt.txt`与官方发布ZIP中的`ground_truth_path.csv` | 1385条p/q数值逐项相同，时间最大差239ns | 数值来源和轨迹身份 | `b`等于IMU、LiDAR或另一个Body安装点 |
| 官方[ICCV多传感器挑战要求](https://superodometry.com/iccv23_challenge_Mul) | 估计轨迹按IMU位姿提交 | 评测参考约定，支持IMU解释 | GT导出代码实际使用哪个杆臂/姿态外参 |
| 官方[SuperLoc发布页](https://superodometry.com/superloc.html) | Corridor01来源SubT-MRS/Hawkins/RC2；提供TUM字段顺序 | xyzw格式和数据集关联 | 地图XYZ世界与GT世界同一性、YAML时间及source/target定义 |
| 官方`laser_to_imu`文件 | RC2的数值外参存在 | LiDAR→IMU的标定方向标签、杆臂 | 不能据此把GT的`b`直接命名IMU |
| 已公开SuperLoc/SuperOdom定位读取代码 | 读取`start_pose.txt`的xyz/rpy构造`T_w_lidar` | 该工程启动文件预期LiDAR位姿 | `start_pose.txt`就是TUM GT，或GT首行可以直接使用 |

GT原始发布CSV列名：

```text
timestamp,p_w_b_x,p_w_b_y,p_w_b_z,q_w_b_x,q_w_b_y,q_w_b_z,q_w_b_w
```

当前并未取得直接绑定此CSV到物理传感器安装点的GT导出实现。
没有把字段名`b`、轨迹第一行p=0、首时刻恰好匹配一个LiDAR header，作为参考点认证依据。
第一行四元数按xyzw归一化；非单位首姿态也不能单独证明或否定LiDAR/IMU身份。

公开源码本地副本：
`/media/jian/HIKVISION/comparison algorithm/SuperLoc_SuperOdom`，
commit `f10e65cd50007767b22e4c401689665e20d827d6`。
`laserMapping.cpp:223`读取启动文件，`:226`取第一条xyz/rpy，`:305`构造`T_w_lidar`；
`superodom_utils.cpp:43`指定`start_pose.txt`，`:64`解析的是duration/xyz/rpy，不是TUM GT。
来源：[SuperOdom](https://github.com/superxslam/SuperOdom)。该URL由本地只读副本的origin核实。

旧P3文档的IMU参考点是“官方评测约定+精确轨迹谱系”的条件性工程判断。
本轮保留旧档案，不将此条件性判断升级成已认证官方GT导出定义。
