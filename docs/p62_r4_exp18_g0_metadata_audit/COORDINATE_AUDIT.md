# 坐标与标定审计

正式状态：`COORDINATE_CONTRACT_NOT_ESTABLISHED`；并存 `CALIBRATION_CONTRACT_NOT_ESTABLISHED`。
约定 `T_A_B` 将 B 坐标点转换到 A，故 `T_M_W` 为 **W→M**，不是 M→W。
理论定位链 `T_M_L(t)=T_M_W T_W_I(t) T_I_L` 只是所需合同，不能据公式认定矩阵已知。

## 1. E57 的实际坐标

完整解码得到一个合并扫描、285455240 点。定义 S 为该扫描点数据所在局部系，M 为应用文件所存 pose 后的 E57 根坐标系。
`e57_structure.json` 保存了全部文件/扫描结构，`e57_scan_metadata.csv` 保存实际解码范围。

文件 pose 的旋转有具名 w/x/y/z 字段，顺序无需猜测：

`[0.5997538229970393, -0.00006330763397267112, 0.0000006879051607181, 0.8001845710785146]`。

平移为 `[112.229971,21.414423,0]` 米。该矩阵是 `T_M_S`，**不是 T_M_W，也不是任何传感器外参**。
具名字段按读入值构造 SO(3)，归一化仅去除浮点舍入；原范数 1，行列式约 1.0000000000000002。
纯数学往返最大矩阵误差 2.842170943040401e-14，正交性误差 2.220446049250313e-16。
这些只检验代数，不检验物理坐标精度。

E57 的 `coordinateMetadata` 是空字符串，创建时间为 0，未提供地图原点、测量基准或 W→M 数值变换。
格式坐标采用 E57 的笛卡尔长度语义；[libE57 官方 Translation 文档](https://asmaloney.github.io/libE57Format-docs/d6/dad/structe57_1_1_translation.html) 定义平移单位为米。这不提供 GT 与地图的跨文件注册变换。

额外的实际范围现象：文件 `cartesianBounds` 与应用 pose 后的根坐标范围相符，最大差约 1.76e-5m，而不是与 raw 局部范围相同。`e57_bounds_scope.csv` 逐轴保存了二者。不能只读 bounds 就给原始点坐标指定同一含义，也不能据此认证 W=M。未宣称此导出习惯必然构成格式损坏。

## 2. GT 的世界系与姿态系

[固定官方 FAQ](https://github.com/Hilti-Research/hilti-slam-challenge-2022/blob/e4aacf7c111ab832a0bdffe3f069255454068540/README.md) 称 GT 使用 Alphasense IMU I。
但[固定官方 xacro](https://github.com/Hilti-Research/phasma_description/blob/79002026e9b229cf9b4e95a403eaec4f00acfd79/urdf/phasma.urdf.xacro) 注释称 GT 在 `base`，并定义 parent `alphasense_imu` → child `base` 的零平移和 `rpy=(-pi,0,0)`。
同原点不等于同姿态。若文件是 `T_W_base`，转换到 I 需要右乘 `inverse(T_I_base)`；若文件已经是 `T_W_I`，再次乘会造成错误。没有发布 GT 的确切导出/消费者说明，不能选择其中一支。

同 commit 的 `phasma.urdf` 是不同的旧生成树：根节点 base、IMU 名 imu_link、安装参数也不同。不能拿它冒充当前 xacro 的展开或本 bag 实际 TF。
本 bag 没有提供认证该别名链的 TF。也没有借 GT 全段拟合 T_M_W。

## 3. 标定 YAML 与 frame_id

正式 747B YAML 的 parent/值见原文件 SHA 和 `coordinate_transform_chain.csv`：

- PandarXT-32 的 parent 为 imu；四元数数组 `[0.7071068,-0.7071068,0,0]`，平移数组 `[-0.001,-0.00855,0.055]`。
- measurement_tip 的 parent 为 imu；四元数数组 `[0,0,0,1]`，平移数组 `[0.059,-0.00855,0.1964]`。
- bag 的实际 LiDAR frame_id 为 PandarXT-32，IMU 为 imu_sensor_frame。

没有找到实际消费这一 YAML 的固定官方代码来认证四元数排列、主动/被动方向及矩阵方向。名称 imu、imu_sensor_frame、alphasense_imu、base、base_link 不能仅凭相似性互认。
未用“某种排列看起来合理”、四元数范数、CAD 大致尺寸或轨迹外观选择外参。
因此 T_I_L 和 T_I_P 的完整数值语义仍未认证，未将原数组擅自转为运行外参。

## 4. 有界审计结论

已建立的是 E57 存储扫描→根的内部代数关系，未建立的是地图根 M→参考世界 W 的物理/发布关系、GT 姿态参考系和正式 YAML 消费语义。
所有原始数组、参考与地图未改变。未执行 ICP、Umeyama、轴序搜索、GT 拟合或定位误差计算。
缺口和精确来源分别写入 `coordinate_transform_chain.csv`、`parser_environment.json`；应定向索取发布变换与消费者说明，而不是用测试数据估计缺失合同。
