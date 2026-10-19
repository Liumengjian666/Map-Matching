# R6陀螺仪偏置来源与稳定性审计

## 判定

**BIAS_STABILITY_NOT_EVALUABLE**。没有证明漂移，也没有证明稳定。
没有从目标窗口均值估计新bias，没有用Raw NDT或未来EKF状态拟合bias。

## 真实版本和状态来源

R6实际作者源码根：
`/home/jian/livox_ws/p61_r6_geode_runs/author_ws/src/FAST_LIO`。
作者上游1f008a7249e36393a1752622de50660b77b5b7f4，加冻结R6补丁；
R6工程CODE_SHA0532c3b2d882327c7f22f57a53add6c7ceb11983。
实际源码逐文件与R6 build_manifest核验，二进制只算SHA、没有启动。
源码、配置、日志的逐项路径/大小/SHA在asset_gate_before.json和evidence_inputs_frozen.json。

1. `laserMapping.cpp:895–912`第一扫描FIRST_SCAN被跳过，下一组进入ImuProcess。
2. `IMU_Processing.hpp:30,164–218,377–399`：MAX_INI_COUNT=10；N从1开始，
   对真实IMU角速度做均值，`init_state.bg=mean_gyr`；N>10退出初始化。
3. 真实mapping_trajectory.csv第二组明确记录IMU_INITIALIZATION和imu_count=10。
   replay.csv将该组限定到第一扫描raw end之后、第二扫描raw end之前。
   初始化使用10条原始/bridge均字节一致的Xsens消息：
   **1706671289500390052–1706671289591141939ns**，原始序号10596–10605，
   /imu/data，imu_link，angular_velocity按ROS合同rad/s。
4. 仅为复核历史seed算术，重新计算这10条的均值与当时记录相差最大
   **2.168404344971009e-19rad/s**。此历史均值不是新的真实bias估计：
   `[-0.001874310302082449,0.0036424783291295173,-0.008585996209876611]`。
5. `include/use-ikfom.hpp:12–20,49–83`把bg作为状态，传播使用gyro-bg，允许
   bias过程噪声；IMU_Processing.hpp:271配置其过程噪声。laserMapping.cpp:987调用
   原生迭代状态更新，esekfom.hpp:1813对状态施加boxplus。bg并非固定常量。
6. bridge总输入2984扫描、29844条IMU，记录2981次MAPPED更新；原始IMU输入范围
   1706671289400330543–1706671587824846982ns。记录最后bg与导出bg完全相同。
   这说明最终值来自连续非GT LIO状态；不能将29844条输入各自对最终bias的贡献
   当作已独立识别，也没有重跑滤波器以进行这种分解。
7. `src/r6_terminal.hpp:23–48`在native末端IMU传播后直接导出`s.bg`。
   initialization.json中的最终值是：

```
[-0.0008725237314739192,
  0.0023757483405765214,
 -0.010976615256704694] rad/s, IMU坐标系I
```

导出状态时刻1706671587829840123ns；目标窗口起点晚
**216.969883664s**。最后原始IMU时间与native double秒往返记录的1ns差异在既有
initialization.json分别记录，不能解释为独立物理时钟校准。
R10使用此冻结值，不是上述开机10条seed，也不是R8未来bias。

## 静止与温补证据

实际初始化代码使用样本计数退出，没有明确静止门。
mapper.log的“IMU Initial Done”只证明该分支执行，不证明机器人静止。
author_logs/imu.txt与dbg.txt均0字节；mat_pre/mat_out/pos_log属于同一估计器输出，
不是独立静止或真偏置证据。stationarity_evidence.csv仅有表头，明确NOT_EVALUABLE。

作者gamma_drive.yaml描述IMU噪声/随机游走，不是这次会话的温度补偿记录或静止验证。
未找到与该序列、目标时间绑定的独立静止记录、温度校准日志或真实bias复核。
这不是宣称Xsens设备内部没有出厂校准或温补，而是本次资料不能核验其适用性。
没有用“gyro小”推断静止，没有把目标gyro均值叫bias，也没有搜索更有利静止段。

## 对R10解释的限制

偏置来源可追溯，但来源可追溯不等于216.97秒后的稳定性已独立证明。
不能把同一LIO内部bias状态作为独立真值。R10在固定bias下得到的数值结果保留，
不因此判为计算错误；也不能把其残差唯一归因于Raw NDT。
