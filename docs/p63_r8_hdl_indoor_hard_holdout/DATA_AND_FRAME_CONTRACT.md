# P63-R8 Hard独立序列合同与预注册

START `cf5a45e47819c3bbbc0834bcfc2ab132aeac6818`。本轮只确认已冻结成熟八方向平移多初值与Gaussian排序，不研发新算法，不把离线候选送入UKF。

## 零NDT资产准入

Koide indoor_hard_01，官方Zenodo 10122133，Azure Kinect瞬时深度点云，**非MID360**。原始数据、官方map_indoor_hard.ply、GT和2379帧已转换PCD均在`/home/jian/rosbag/public_datasets/koide_p62_r29/`，无下载。R29此前只准备Hard，没有正式定位结果；本轮raw5–164未参与Easy R5–R7种子开发，但使用同一数据集家族及相关地图环境，不作跨平台泛化声明。

官方发布记录及历史下载回执直接从R29归档读取。三载荷本轮流式重算MD5/SHA与官方/历史一致：

- indoor_hard_01.zip：1,951,298,635B，官方MD5 `c5eca239e8635656af1ec5c5bb6532b7`。
- map_indoor_hard.ply：6,023,877B，MD5 `0ee5c1f2adc5c1a380fbd9a44c377c68`。
- gt.zip：1,036,441B，MD5 `ab2a80dc1d06767b7c3ce1893f3020b0`。

具体SHA和回执见ASSET_PREFLIGHT.json。对本轮raw4–164从原始ROS2 Reader重新解码161条：原header整数ns、frame、bag时刻及全XYZ float32字节均与PCD一致；没有修改原数据。原始DB历史整文件SHA保留在R29，本轮不再全量扫描5GB数据库，而直接核对实际使用CDR/转换点。原metadata SHA与R29一致。

## 坐标和时间

`T_A_B`将B变换到A。M为对应官方Hard先验图数字坐标；L为原深度点云`depth_camera_link`。官方traj_lidar_indoor_hard_01.txt为`T_M_L`；HDL父map/子depth_camera_link直接同义，不乘IMU外参，不拟合SE3、时差或地图坐标。来源沿用R29 KOIDE_DATASET_CONTRACT、Zenodo说明及官方GLIM方向快照。

IMU关闭，没有传播/Deskew/外参在线应用；已有零点时间是瞬时深度快照声明，不是假造LiDAR去畸变。原UKF预测以原header间隔工作，无未来参考反馈。

raw4 anchor=1690354429663743949ns，唯一Oracle Start位置约[11.444583,-6.156952,-.252192]m，完整SE3/四元数及合法参考插值bracket已冻结到INPUT_FREEZE.json。准备阶段只解析raw4所需参考两端位姿，不解析后160帧参考；参考文件字节校验不等于将测试位姿输入算法。

raw5=1690354429730409173ns，raw164=1690354440663856072ns。参考评价继承R29线性位置/SLERP，最大bracket150ms、最近端点≤50ms。GT文件与发布ZIP成员逐字节一致。参考属于同图锚定的离线GICP+IMU估计，非独立动捕或独立物理真值。

## 新地图和Source身份

复用R29已生成Hard全部8个Tile，来自官方Hard PLY的原两次0.15m PCL处理；逐Tile SHA核对。按manifest顺序合并96324点、XYZ逐bit不变，仅补零intensity生成独立PCD，原Tile/PLY不改。该既有地图预处理与R5协议一致，不依据本轮输出改图。

HDL原0.1m VoxelGrid实测Target **93223点**，XYZ SHA **166c4d2a4f5d24a4c79d4c90a8a275e0fe905fcc9650d559aabb775780e81654**。本轮不使用Easy的89656点身份。合并输入图路径/SHA在INPUT_FREEZE，预处理点数/目标SHA在TARGET.json。

全部160帧以R7同一PCD→PointCloud2→fromROSMsg→同帧复制→0.1m VoxelGrid预检，Source范围1699–3404点。SOURCES.csv和INPUT_FREEZE保存逐帧Source SHA；正式trace将保存实际Source SHA并逐帧核对，避免历史R5现场SHA缺失。原生map server、NDT目标和Source身份任一不符即停止。

## 原版160帧阶段

仅raw5–164，一次原版HDL，不添加候选。官方HDL `de2dc7769aa2412876d05ae6161408d3295b6a3c`、NDT_OMP原库与UKF不改；1m/DIRECT7/.01/max35/.1/.55、Source/Map .1m，IMU/global/robot odom/UTM均关闭，原cool-time2s（从首实际predict起）不改。

沿用R5逐帧离线握手；每帧等待输入XYZ、同stamp原生NDT、callback、status和UKF输出。只读trace唯一增加实际Source SHA，不增加align或改变滤波。原生质量低或未收敛不添加门控；仅非有限核心、身份/时序违约、程序/监控故障、30s回执超时或空间<20GiB停止。没有连续低inlier/跳变停止规则，不剔除失败帧。

baseline_01独占。先关闭并封存全部160帧原始结果，再从其实际guess生成候选输入；直到候选与排序也封存，才评价基线与候选参考误差。不重跑名义NDT或读取后续GT补状态。

## 六帧48次阶段

固定raw30/55/80/105/130/155，不因正常/失效改选。Candidate0复用基线原生终值；1–8为相同地图XY八方向0/45/90/135/180/225/270/315°、r=.75m，保持Z/旋转，输入初值仅来自封存guess。48实际种子另提交冻结后才运行，最多48次align，无候选UKF或ROS回放。

选择照搬R7：收敛、位姿有限、Gaussian分数有限；Gaussian降序，完全同分ID升序，0优先。NN fitness、原生inlier/matching_error仅记录，后者仍是地图原点25m裁剪统计。无新质量阈值、去重影响排序、联合分数或GT选择。

## 冻结评价和判定

基线报告全部160帧；>.5m或>5°连续3个有效参考帧计持续高误差，连续3个低误差计恢复。拒绝/丢失/参考缺失不填零。

六帧按Candidate0分组：正常≤.15m且≤2°；高误差>.5m或>5°；其余中间。候选恢复目标仍为≤.15m/2°且位置改善≥.30m。正常安全：位置误差增量≤.10m、旋转增量≤1°且不转为高误差。所有分组只在选择封存后完成，不改变在线决策。

判定优先级：合同/执行异常先报告；正常帧任一安全违约为HELDOUT_SAFETY_COUNTEREXAMPLE；高误差机会≥2且均生成/选中合格结果、正常≥2且全安全时为HELDOUT_RECOVERY_AND_SAFETY_SUPPORTED；有高误差机会且实际恢复失败为HELDOUT_RECOVERY_NOT_CONFIRMED；缺少足够机会且无上述反例时为HELDOUT_INSUFFICIENT_COVERAGE。没有高误差机会另记NO_HELDOUT_RECOVERY_OPPORTUNITY，不补选扫描。

分别报告生成和排序失败、改善/恶化、成本。GT最优仅离线上界，候选不拼成虚假在线轨迹。独立序列确认不等于创新成功或跨场景可靠性。

资源：160次名义+最多48额外，原版160次UKF正常执行；新增候选UKF0/ROS0。地图、单次NDT、完整callback、八次事件墙钟、排序、进程RSS分开；离线握手不证明10Hz或在线最终性能。不自动跑2379帧、不换序列、不下载/删除、不写稳定机器狗工程。
