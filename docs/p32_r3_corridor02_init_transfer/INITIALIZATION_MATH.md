# 实际首段初始化数学与坐标边界

## 完整复用的旧程序

真实来源为 `superloc_adapter_ws/src/superloc_corridor_adapter/src/`。
本轮 `legacy/superloc_first_segment_initializer.cpp` 与原文件逐字一致，SHA256：
`deb58aeb3bd76f802568469f50d33f91b8026924b3fe299fa93b8fa8f9dc266d`。
诊断包装器继承同一 PCL ICP，调用原 `Base::align`，不改 source、target、
初值、优化参数或原 main。没有用单种子 NDT 代替首段程序。

## RC1 输入桥接

采用记号 `T_target_source`。原标定 `laser_to_imu` 为 `T_I_L`。
原始标定文件不改；派生旋转采用既有 adapter 的 SVD 舍入处理，本轮
RC1 输入 det>0，不需反射修正。桥接是 P2B 旋转去畸变输入适配，
不是 P31 的完整 Scan-Local runtime，也不声称重现 P2B 历史二进制。

真实 gyro 以梯形平均积累右乘 SO(3)，相邻真实时间节点间 slerp：

`q_(k+1) = normalize(q_k * Exp((omega_k+omega_(k+1))/2 * dt))`。

扫描内 `R_I0_It = R_W_I0^T R_W_It`，补偿至扫描起点 LiDAR：

`R_L0_Lt = R_I_L^T R_I0_It R_I_L`

`t_L0_Lt = R_I_L^T (R_I0_It t_I_L - t_I_L)`。

本轮 RC1 杆臂为零。上述并不引入逐点平移速度模型；首段完整帧间
平移由原 scan-to-scan ICP 估计。没有伪造 odometry、GT 或恒速反馈。

保留整数 header ns、原 float 秒点时间和原 XYZ 顺序。截断的是输入
窗口而非跨边界扫描：L0 为原 scan3，1645999726479818106 ns；
cutoff 为 1645999731479818106 ns。49 个完整扫描 end 均不晚于 cutoff；
1001 个真实 IMU 含 L0 前最近边界样本，最大间隔不超过 20 ms。
插值不用 cutoff 之后 IMU，不外插。窗口内后续数据属于已声明的离线
启动预热，不能声称在 L0 采集时实时可用。

## 局部子地图与 24 候选

每扫描 voxel=0.20 m。scan-to-previous ICP：最大对应距离 2.5 m、
最多 40 次迭代、epsilon=1e-6、identity 初值。
原接受规则：收敛、位移<3 m、旋转<45°、finite fitness。
累积 `T_L0_Li = T_L0_Lprevious * T_Lprevious_Li`，子地图 voxel=0.25 m。

官方 YAML 的生效 `R_yaml,t_yaml` 由原 loader 读取；测试核验实际文件
解析结果与生效字段一致，不读取注释矩阵。保留原 float 舍入值，
本轮正交残差约 2.09e-6，合法性工程容差预先固定为 1e-5。

`A_i` 是原程序枚举的 24 个 signed permutation，`det(A_i)=+1`。

**原真实生成规则：`R_seed_i=R_yaml*A_i`、`t_seed_i=t_yaml`。**

它等价于 `T_seed_i=T_yaml*[A_i,0]`，只改变旋转、平移不动。
它不包含世界坐标整体左乘 `[Rz90°,0]`。
后者会把 t 转到 [176.217148,39.929523,-5.225427]，但不在本轮授权
复用的原候选集合内。本轮不直接采纳 90°，不扩展搜索。

目标地图 voxel=0.20 m；每候选原 ICP 最大对应距离 8 m、最多60次、
transformation epsilon=1e-7、fitness epsilon=1e-6。
原排序保持：converged 优先，随后 fitness 从小到大；不增加 GT、
候选编号 tie-break、新接受策略或第二轮 ICP。

## 准入、地图归一化与时间

原程序无条件打印 `FIRST_SEGMENT_INITIALIZER_PASS` 并写
`corridor01_init.yaml`；即使所有候选失败也如此，退出码才为7。
本轮保留该原始证据，但不把日志/YAML 当作合法运行初值。
几何准入使用执行前 PROTOCOL 的固定阈值；本次未通过。

若合法 `T_M_L0` 已建立，旧 normalizer 才施加
`T_N_M=inverse(T_M_L0)`，得到 `T_N_L0=Identity`。
本次没有获准位姿，故没有实际归一化，没有可交付的 `T_N_M`。
旧 normalizer 与 holdout 副本只用于来源和接口归档，未运行。
5–10秒验证、F/G 短段、GT评价均没有执行。

这次证据仅表明原固定平移的24候选在当前 Corridor02 数值种子与
PCD下无法建立几何对应；不证明官方 YAML/PCD 错误，也不评价 P31
成熟融合算法精度，不证明需要哪一种具体左乘变换。
