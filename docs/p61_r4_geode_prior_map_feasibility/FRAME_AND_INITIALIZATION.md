# 坐标、参考点与初始化

约定p_A=T_A_B*p_B。
L=Livox Avia，I=外置Xsens（/imu/data），M=未来非GT建图地图系，G=官方Leica参考世界系。
同名livox_frame不用于推断内置IMU外参。

## 已获得官方数值与方向消费证据

```text
T_I_L =
[ 0.999620  0.027463  0.002445  0.049258 ]
[-0.027517  0.999299  0.025398 -0.012500 ]
[-0.001746 -0.025456  0.999674  0.026946 ]
[ 0         0         0         1        ]
```

来源：官方Drive gamma.yaml，88–95行；作者FAST_LIO carol.yaml的extrinsic_R/T数值一致。
laserMapping.cpp:803–804读取参数，827–829传入Lidar_R/T_wrt_IMU，
181/193行将LiDAR点先转IMU再乘世界IMU状态，证明消费方向。
||R^T R-I||F=1.6552926776829896e-6，detR=1.0000001183073217。
保留官方舍入值，不投影改写标定。完整数值审计和逆矩阵见calibration_audit.json。
这是来源及方向证据，不是本轮已通过运行时精度/时间/去畸变验证。

未来若有合法非GT建图状态：
T_M_L(t)=T_M_I(t)*T_I_L。
地图点p_M=T_M_I(t)*T_I_L*p_L。
这些是合同公式，不表示本轮有T_M_I轨迹或地图。

## GT转换与世界注册不能混淆

官方GT的四元数全为0，无T_G_A完整姿态。
README把Metro_Tunnels评价参考归于alpha；官方gamma2GT_leica.py生成T_script后右乘其逆。
只有输入确为T_M_L时，按该脚本参考点合同可写：
T_M_A=T_M_L*inverse(T_script)。
若先输出的是T_M_I，应先右乘T_I_L，再作参考点转换，不可直接把IMU状态当LiDAR状态。
T_script的物理输入参考点还必须随具体保存输出确认，不能一段脚本套所有算法。

真正无对齐位置比较还要求独立已知的T_G_M：
predicted_p_G_A=translation(T_G_M*T_M_L*inverse(T_script))。
当前T_G_M不存在，不能从全段测试GT拟合后称为绝对地图定位精度。
官方rmse.py的-va含全段Umeyama对齐，与该无对齐协议不同，本轮未运行。

## 合法初始化边界

- 可以由非GT建图轨迹选择M的坐标规范；例如对全部地图与轨迹施加同一常量变换，
  使首个建图LiDAR位姿为I。这只是地图gauge，不意味着返程测试首帧也为I。
- 测试初值需来自已验证的同系因果状态、测试前合法预热/独立测量或用户指定物理起点。
- 如使用中间隔离段取得起始状态，需另行冻结其输入依赖，不能读取未来测试帧或全段平滑结果。
- 官方GT位置没有姿态，不能构造完整Oracle初值；更不能生成GT建图轨迹。
- 当前没有数值T_M_L(t_test0)或T_M_I(t_test0)，不填单位阵占位，不自动运行Smoke。

R2的header/timebase双时钟记录保持原结论。作者LIO使用header作为scan开始时刻的证据
不能替代实际Deskew时间合同验收；本轮没有重写或尝试新的运动补偿。
