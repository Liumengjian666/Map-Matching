# 官方初始化用途与未闭合语义

## 已有直接证据

1. 官方 [SuperLoc 发布页](https://superodometry.com/superloc.html) 的 Dataset 节说明：
   为支持可配置初值的定位模块，提供各数据集的 initialization poses。它支持
   “本文件是初始化资产”，不支持直接将其中Y认证成任意时刻的固定世界变换。
2. 本机真实文件为 `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/initial_pose/corridor01.yaml`。
   文件完整原文保存在 `input_freeze.json`。只有 `%YAML:1.0`、`# s 67` 及
   两个生效 OpenCV R/t 字段，没有时间戳、传感器名、单位或变换方向的其他定义。
3. 所查官方 SuperOdom 固定版本 `f10e65cd50007767b22e4c401689665e20d827d6`
   的 [LOCALIZATION.md](https://github.com/superxslam/SuperOdom/blob/f10e65cd50007767b22e4c401689665e20d827d6/doc/LOCALIZATION.md)
   使用 `init_x/y/z/roll/pitch/yaw` 和可选 `read_pose_file`。
   本地只读源码为 `/media/jian/HIKVISION/comparison algorithm/SuperLoc_SuperOdom/`：
   `super_odometry/src/utils/superodom_utils.cpp:43` 寻找 `start_pose.txt`，`:64`
   解析 duration/xyz/rpy；`super_odometry/src/LaserMapping/laserMapping.cpp:223`
   读取该接口，`:305` 附近写 `T_w_lidar`。这些代码不读取本YAML字段，不能借它们
   证明本Y的source/target。对所查repo的cpp/h/hpp/md/yaml精确字段搜索没有命中。
   不将有界搜索无命中解释为所有版本官方实现都不存在读取器。

## 本轮数值定义

```text
Y =
[ 0.135990 -0.990409 -0.024406  1.968147
  0.990705  0.136027  0.000140 -6.879292
  0.003181 -0.024198  0.999702 -0.896125
  0         0         0         1        ]
```

采用 `p_A=T_A_B*p_B`：

- A：仅假设 `Y=T_M_L(t0)`，直接 `p_M=Y*p_L`。
- B：仅假设 `Y=T_L_M(t0)`，直接 `p_M=inverse(Y)*p_L`。
- `t0` 本轮按原始首LiDAR header之后67秒的唯一最近帧确定；不宣称官方已经定义此零点。
- 没有 `Y*GT`、`inverse(Y)*GT`、世界Yaw、24轴、平移网格或附加外参。
- LiDAR到IMU运行外参 E 存在，但没有证据表明DARPA是IMU或Body，所以不增加 `Y*E`。
- 不用归一化地图、不应用PCD VIEWPOINT、不用GT拟合世界变换。

Y的正交残差 `1.2379704056665196e-6`、det `0.999999216169924`，是保留文件舍入数值的
近似刚体矩阵，符合既有探针1e-5容差，不是严格数学上精确的SE(3)。未正交化或优化它。
完整数值逆及其合法性保存在冻结JSON。实验按地图/点云的米尺度解释平移；
原YAML本身没有明示单位，不把实验单位约定伪装成官方注释。

## 时间关系

P33已真实核查的记录时钟关系由 `time_binding.csv` 引用并明确标注 inherited evidence。
本轮重新从2777帧整数header索引验证最近帧，并读取真实解码消息核对header/record身份。

- 原始首LiDAR header：`1517157219088119030 ns`。
- 传感器header相对+67s目标：`1517157286088119030 ns`。
- 最近帧raw665：`1517157286055073023 ns`，相对+66.966953993s。
- 原bag record起点+67s后首LiDAR为raw666，相对header+67.067813873s。
- 重打包完整解码bag record起点+67s后首LiDAR为raw664，相对header+66.866163969s。

三个定义不是同一帧。`# s 67` 未说明其中哪一种，也未说明是否另有采集裁剪零点。
本轮不据此追加其他扫描的几何测试，不用GT挑选时间。

## GT的角色

候选只由 `direct_poses(Y)` 生成，函数没有GT或外参参数。
准备程序复用的P33冻结JSON包含历史GT矩阵；它们不是本轮初值输入。
本轮GT文本解析发生于Y/inverse(Y)与Source身份持久化之后，仅为该header生成
合法插值和数值关系诊断。Y和GT之间的数值差不是已认证坐标系中的定位误差。

官方用途确认、几何支持及物理语义是三项不同状态；按真实结果分别报告。
