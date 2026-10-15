# P34 Corridor01 GT 参考点与初始化定向验证

正式结果：`INITIALIZATION_FRAME_UNRESOLVED`。

没有证据把发布GT严格认证为LiDAR位姿；官方评测约定支持IMU解释，但GT导出代码中的
物理参考点仍未认证。没有找到“GT误认为IMU、重复应用外参后进入NDT初始化”的运行代码。
本轮不修改定位源码，不执行ICP、完整定位回放或NDT优化。

## 已经确认的事实

1. **完整解码输入真实有效**。本轮重新读取全部已解码bag，逐扫描比较有序XYZ/time，
   重新对照原始bag中的IMU/图像序列化payload，并重算输入SHA：2777扫描、79,932,911点、
   55,957 IMU、6720图像，全部通过；不是引用旧报告的PASS。
2. **当前NDT初始化不读取GT**。`p_=0,R_=I`作为归一化地图中的第一帧LiDAR种子；
   后续初值来自上一帧Used和历史增量，旋转可使用实际IMU先验。GT显示器是独立旁路。
3. **历史地图做了真实XYZ变换**，不是仅更改PCD存储格式。对338210个同序点复算
   `p_N=inverse(T_M_L0)*p_M`，最大误差0.0000355391m，低于预设0.00005m。
   原图和归一化图保留了相同VIEWPOINT元数据；NDT的`loadMap()`不把它再次施加到XYZ。
4. **时间不能混用**。首个完整IMU覆盖扫描是原始第2帧，但比GT首时刻早605161190ns。
   本轮几何检查固定改用首个同时有IMU和GT支持的原始第8帧，时间恰好等于GT第一行，
   没有插值误差或GT外推。`# s 67`的零点及语义仍未认证。

## GT 证据等级

本地1385条GT的全部平移和四元数数值，与缓存的官方SubT-MRS Hawkins Long_Corridor
GT发布文件逐项相同；1284行时间完全相同，其余101行最大序列化差239ns。
这证明了数值来源，**不证明CSV中`b`的物理安装参考点**。

[官方挑战页面](https://superodometry.com/iccv23_challenge_Mul)要求提交的估计轨迹使用IMU参考点。
这是支持H2的评测约定，不是已取得的GT导出程序。
[SuperLoc页面](https://superodometry.com/superloc.html)给出了Corridor01来源及TUM字段顺序，
没有闭合PCD与GT世界变换。历史P3报告把这一证据称为`OFFICIAL_LINEAGE`；
本轮不覆盖旧报告，但不将该推论提升成物理参考点已认证。

## 两个固定条件性候选

记`T_target_source`。`M`为原始PCD的实际XYZ系，`N`为归一化地图系，`W`为GT世界系，
`E=T_I_L`为冻结runtime的proper RC2外参。

只有在额外条件`W=M`下，才有本轮计算的：

- H1：GT为LiDAR，`T_N_L=T_N_M*T_W_L_GT`。
- H2：GT为IMU，`T_N_L=T_N_M*T_W_I_GT*E`。

`W=M`不是既成事实。H3没有独立Body外参，H4没有已认证`T_M_W`，均未构造假矩阵。
H1/H2位姿相差0.09022749m、0.23651110°；这一量级本身不能解释观察到的近似反向旋转。

同一raw扫描、同一0.5–80m筛选、0.25m source voxel、1400点上限；同一归一化地图
按生产`loadMap()`两次0.15m voxel后得到226164目标点。不使用GT逐点去畸变，
不伪称这些点是历史OOSM回放中的实际NDT Source。

| 条件性候选 | NN median (m) | NN P90 (m) | ≤0.5m | ≤2m | ≤8m |
| --- | ---: | ---: | ---: | ---: | ---: |
| H1 GT=LiDAR，W=M | 1.819515 | 5.893917 | 18.1429% | 55.4286% | 96.0714% |
| H2 GT=IMU，W=M | 1.812433 | 5.930974 | 18.1429% | 55.9286% | 96.0000% |

结果相近且近距离覆盖有限，不能唯一认证参考点或世界变换，也不能作为可靠初始化已通过。
没有按NN分数切换候选、拟合世界变换或扩大搜索。

## NDT准入与源码问题判断

原合同只允许在参考点等证据成立时，对预先选定的H2最多运行一次NDT。
执行NN之前即因参考点、`T_M_W`未认证冻结为`--NN-only`。
因此本轮完整NDT调用0，ICP调用0，定位回放0；`ndt_test.json`记录真实未执行原因，
不存在可报告的final pose、convergence或fitness。

未确认初始化变换代码错误：

- NDT与EKF不从GT构造初值，也不在运行时直接加载`world_darpa`作为首帧位姿。
- NDT LiDAR观测转内部IMU状态使用`T_N_I=T_N_L*E^-1`；输出LiDAR使用`T_N_L=T_N_I*E`。
  这两个参考点转换相互抵消，不是重复乘外参。
- GT显示/离线评价中曾使用IMU解释和外参；即使该解释最终被推翻，先影响的是显示和评价，
  不能据此倒推出它污染了NDT初值。
- TUM字段按`qx qy qz qw`解析、归一化后构造旋转；未发现xyzw/wxyz混用。
- 官方原始`laser_to_imu`旋转并非proper SO(3)：正交误差0.00221397，det=0.99844268。
  运行配置已使用既有SVD极分解旋转，t不变；本轮仅核实并复用，未改标定。

最小建议不是立即改乘法：先取得GT导出时`b`参考点及固定`T_M_W`的可追溯定义。
在此之前不得把GT直接输入正式初始化器，或在RViz中把未知坐标关系伪装成已对齐。

## 时间与当前入口限制

历史首段初始化器使用输入bag第一个合法点云起点后的5秒；只读取YAML的R/t，
不解析`# s 67`，不跳转到当前bag的+67秒。历史成功记录为第2帧附近首5秒、50扫描，
`GT_used=false`。这属于离线/启动预热结果，不代表采集第2帧时已经在线获得全部首5秒信息。

当前`corridor01_localization.launch`是P18 CONTROL LiDAR/IMU入口，明确关闭成熟视觉、
Square-Root和Scan-Local实验开关；不能把它称为P31 C完整成熟融合配置。
P19和P30归档首条NDT诊断都从`1517157219.1889789`开始，initial为Identity。
CSV使用double文本，本轮准确整数时刻来自原始索引和显式初始化参数，未从double反推ns。
本轮未启动NDT，因此没有新增“当前实际首条NDT消息”的运行观察。

## 成本、复现与交付

全输入离线复核109.520s，峰值RSS685724KiB；两次NN诊断合计wall 0.13s，CPU 0.11s，
峰值RSS55484KiB。不能将其解释为在线算法性能。离线工具统一编译通过，4项定向测试通过。

参见`GT_REFERENCE_EVIDENCE.md`、`SOURCE_CALL_CHAIN.md`、`time_correspondence.csv`、
`transforms.json`、`initial_geometry.csv`、`results.json`及`REPRODUCE.md`。
只新增本目录的离线工具和轻量证据；未修改`src/`、原始数据或两个受保护工作区。
所有大文件和临时XYZ留在本地缓存，不提交。没有执行push。

唯一NEXT：取得官方GT导出参考点和PCD—GT世界坐标转换的绑定证据；未获批准不再做初始化搜索。
