# Corridor02坐标与初始化合同（失败如实保留）

采用T_target_source：p_target=T_target_source p_source。
M：发布corridor02.pcd中实际XYZ存储坐标；官方map世界语义未闭合。
L：cmu_rc1_velodyne原始PointCloud2的LiDAR坐标。
I：epson IMU坐标；C：标定rgb_camera光学坐标，图像frame_id为d。

## 已确认的传感器链

标定laser_to_imu给出T_I_L；rgb_camera_to_imu给出T_I_C。
按已有官方外参约定与Corridor01相同传感器处理约定：

```
p_I = T_I_L p_L
p_I = T_I_C p_C
T_C_L = inverse(T_I_C) T_I_L
T_M_I = T_M_L inverse(T_I_L)       # 只有T_M_L成立才可用于运行
```

T_I_L平移为[0,0,0]，旋转约Y轴0.0095rad；T_I_C平移
[0.17313616,0.0476788,-0.01933398]m。完整proper矩阵在sensor_probe.json。
相机MEI与原始畸变参数沿用RC1文件，不使用RC2；没有修改噪声。

## world_darpa的实际使用

Corridor01旧initializer实际将生效R/t作为数值先验，再估计首扫描
T_map_lidar；map_normalizer对原PCD施加该估计的逆。当前C31运行
不加载world_darpa，而是归一化地图首帧identity。相同字段只能支持
复用解析，不能证明Corridor02矩阵的source/target或某个传感器原点。

本轮唯一R/t直接数值种子见sensor_probe.json。它仅在有界非GT几何
检查中充当初始猜测；没有官方语义认证，也没有以它更新滤波器。
没有试逆方向、注释矩阵、轴置换或GT候选。

按IMU实际覆盖预定选第3扫描，1645999726479818106ns。前两帧缺头部
IMU，不以未来IMU回填。本检查为未补偿raw scan，不把它作为运动
补偿后Source。首帧关联仅绑定这次检查，不宣称官方YAML的历史时刻。

## 实际失败与未建立的链

唯一检查NN median293.606m/P90305.380m、0.5m覆盖0%，NDT0迭代。
工程初值FAIL；没有第一帧运行位姿。种子及Raw不是本轮可准入的T_M_L3。

缺少：可追溯非GT T_M_L3（第3帧header1645999726479818106ns），
或将发布初始数值的坐标定义接到M与L3的工程转换。
L3特指第一eligible扫描，不是原始第1扫描。若提供第1帧位姿，仍需
有依据的时间传播到第3帧。不能先假设world_darpa是运动LiDAR位姿或固定
世界间变换；不能靠自动求逆补齐。

若未来T_M_L3合法：N=第3帧参考点归一化地图，
T_N_M=inverse(T_M_L3)，T_N_L3=I，T_N_I3=inverse(T_I_L)。
这是已理解但**本轮未执行**的Corridor01地图归一化链；本轮未写新PCD。

## GT证据等级与边界

Corridor02发布GT参考点仍UNRESOLVED。现有官方SuperLoc页面只给TUM
字段，既有SuperOdom审计不包含此序列GT exporter，不能继承C01 IMU
lineage。未读取GT位姿或以其初始化；绝对map→GT未知不是唯一阻塞。

若将来运行合法但原点仍不确定，T_I_L零平移可支持IMU/LiDAR位置
原点重合的条件说明，但旋转表达仍不同；Camera有非零lever arm。
这些可以预注册有限参考点敏感性分析，不是本轮已认证精度。
当前正式GT评价NOT_RUN，初始化失败独立于GT参考点问题。

## 时间证据限制

第3帧max offset=0.10083921253681183s，整数end-start=100839213ns。
50帧前缀max=0.10083945095539093s，不能混用成第3帧offset。
探针将header视为scan_start、time视为seconds offset，cadence/范围与
该解释相容。IMU括号检查只证明该解释下的覆盖，不认证驱动时间语义
或相机/IMU/LiDAR同步；未运行Deskew和滤波，不宣称运行因果合同通过。
