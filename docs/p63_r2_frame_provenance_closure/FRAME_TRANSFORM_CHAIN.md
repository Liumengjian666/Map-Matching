# 坐标变换链及方向

约定：T_A_B将B中点变换到A；R用于方向向量，平移不作用于重力。所有数值使用米、弧度内部计算；报告角度为度。以下软件关系为CONFIRMED，实际设备和指定地图的绑定另标等级。

| 符号 | 定义 |
|---|---|
| L_t | 该时刻原始Livox点坐标 |
| I_t | 该时刻IMU测量轴 |
| F | FAST-LIVO2在未额外重力对齐时的初始世界轴，初始化时与I0同向 |
| G | 从F重力方向构造的水平世界；不是已经认证的指定PCD坐标 |
| M | 指定PCD保存坐标，生产合同尚未完整认证 |
| S | Super-LIO初始化期望的水平world轴，重力固定在-Z |

## FAST-LIVO2：LiDAR→IMU→地图

`ImuProcess::set_extrinsic`和`LIVMapper::RGBpointBodyLidarToIMU`：

\[
E=T_{I_tL_t}=\begin{bmatrix}R_{IL}&t_{IL}\\0&1\end{bmatrix},\quad p_I=R_{IL}p_L+t_{IL}.
\]

`_state.rot_end/pos_end`是IMU状态，不因轨迹文件叫lidar_poses就变成LiDAR原点。保存点：

\[
p_M=R_{MI}(R_{IL}p_L+t_{IL})+p_{MI},\quad T_{ML}=T_{MI}E.
\]

必要时反向链：\(T_{LM}=E^{-1}T_{MI}^{-1}\)。原始YAML extrinsic_R按行构造；Super-LIO Eigen M3(data)默认列主序，非单位矩阵不能未经检查直接复制。本轮E只有配置假设I4，**物理E为UNVERIFIED**。

FAST IMU_init设置R_F_I0=I、p=0、g_F=-9.81 u_I0。若开关true，`gravityAlignment`从g_F到-Z构造R_G_F，左乘位置、旋转、速度和重力；若false，则不做该世界基变换。重力估计仍可更新g分量，不能把它等同于世界坐标轴自动旋转。保存时PCD直接写世界点，无额外姿态变换；VIEWPOINT=I也不是world竖直的证据。

因此，只有生产参数可绑定时才能选择：false→M=F；true→M=G。指定PCD当前偏向M=F的证据为SUPPORTED，不能把条件分支认定为已获生产认证。

## Super-LIO：IMU初始化→NDT＋ICP

设封存平均比力单位方向为u_I，则近静止下g_I=-9.7946u_I。原生构造：

\[
R_a=\operatorname{FromTwoVectors}(g_I,-g e_z),\quad
\psi=\operatorname{atan2}((R_a)_{21},(R_a)_{11}),\quad
R_{SI}^{pre}=R_z(-\psi)R_a.
\]

R1人工初值为零位置和单位旋转，所以传给NDT的数值矩阵就是该R与零平移。旋转去yaw不改变沿Z的重力。源码将这一水平初态数值当作目标地图M中的初值；这隐含S与M重力语义兼容，不能只把camera_init改名world就证明成立。

初始化点先通过E变为IMU点，NDT＋ICP目标是PCD中的点，因此其最终输出数值为T_M_I，不是未经外参转换的T_M_L：

\[
p_M=T_{MI}^{ICP} E p_L.
\]

R1记录的最终矩阵（六位打印精度）：

```text
T_M_I_ICP =
[ 0.976824  -0.213127   0.0198769  -0.103246 ]
[ 0.197364   0.860845  -0.469037    0.414065 ]
[ 0.0828536  0.462091   0.882955    0.024467 ]
[ 0          0          0          1        ]
```

重建的pre矩阵及组合\(\Delta=T^{ICP}(T^{pre})^{-1}\)见FRAME_NUMERICS.json。Delta只是封存前后数值的关系，**不是独立认证的T_M_S，更不是拟合／建议应用的地图旋转**。NDT中间终值未封存，标NR，不能拆分两个优化器的各自影响。

`SetInitialConditions(...,ref_gravity)`设g_S=-9.7946Z；随后SetX赋值ICP的R、p，SetX不修改g。因此如果M并非水平世界，状态姿态和固定重力就可能属于不兼容的世界约定。源码行为CONFIRMED；该条件能否完整解释R1全部失败为UNVERIFIED。

**独立闭合所需但缺失的T_M_S**不能由这次配准结果直接升级得到。如果有生产者认证的R_G_M，重力应按g_G=R_G_M g_M表达，位姿也须用同一左乘变换。此处只给方向法则，不产生／应用适配参数，不改变地图或初始化角度。
