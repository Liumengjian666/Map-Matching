# P63-R5预注册：Koide indoor_easy_01原版HDL短段

运行范围在任何新align前冻结：raw4只作一次Oracle Start，raw5–164完整160扫描评价，包含raw62/76/131。唯一一次正式运行，不调参重跑，不扩大2023帧，不启用IMU/C+G/全局重定位/robot odometry。

## 数据与坐标

复用R29已转换的官方Koide indoor_easy_01。传感器是**Azure Kinect瞬时深度点云**，不是MID360。ROS2源为`/points2/decompressed`，原frame为`depth_camera_link`，原始XYZ float32、header整数ns保留。现有PCD的附加`t=0`是深度快照约定，不是实际测得的扫描内时间；本轮不Deskew、不合成扫描时长。

`T_A_B`将B变换到A。M是官方先验地图数字坐标；L是原始深度点云刚体`depth_camera_link`；I是官方IMU刚体。R29官方来源确认参考TUM为`T_M_L`，官方外参为`T_L_I`，xyz/xyzw=`[.003,.004,-.051,-.476,.474,.524,.525]`，仅按发布舍入归一化：

\[
T_{M,I}=T_{M,L}T_{L,I},\quad T_{I,L}=T_{L,I}^{-1}.
\]

本轮HDL直接估计`T_M_L`，不需要把L=I，也不使用此外参传播。输出父`map`、子`depth_camera_link`；P62导出父`benchmark_map`、子`benchmark_lidar`已是同一L的位姿，不能因其内部EKF使用IMU状态再对导出位姿重复乘外参。两个父标签只换名，不对数字地图做平移/旋转。

官方参考来源为地图锚定的离线GICP＋IMU优化，不是独立动捕。原地图是否独立测量不额外推断。继承证据见R29 `KOIDE_DATASET_CONTRACT.md`、`input/DATASETS.json`及官方GLIM方向快照。本轮不拟合SE3、时差或外参。

## 固定地图和预处理

直接复用R29实际使用的8个Tile；合计93146点，来自官方`map_indoor_easy.ply`的既有两次0.15m PCL体素处理。逐Tile SHA已核对，按原manifest顺序合并到新独立PCD；XYZ逐bit不变，仅补零intensity以适配PointXYZI，不覆盖旧地图。

派生文件：`/home/jian/livox_ws/p63_r5_hdl_public_baseline_data/r29_all_tiles_exact_xyzi.pcd`。其SHA、全部Tile SHA及点数在`INPUT_FREEZE.json`。HDL map server按作者0.1m VoxelGrid再处理，实际目标点数/XYZ SHA由运行时记录，必须全程相同；不能假定仍为93146点。UTM转换关闭，无重力对齐、动态裁图、扫描插入或地图更新请求。

HDL扫描预处理保持原作者0.1m VoxelGrid，无P62的0.25m／1400点上限及Coverage门。NDT_OMP resolution1.0m、DIRECT7、epsilon.01、构造器max35/step.1/outlier.55；原UKF与cool_time=2s均继承R3，不改数学。

## 初始化与时间

从R29已冻结的raw4 `T_M_L`读取一次初值，不打开评价段参考文件。完整矩阵在`INPUT_FREEZE.json`；平移约[-25.922294009,3.261844004,-.396398000]m。native specify_init_pose直接设置此姿态；不发布raw4扫描，不做锚点NDT。

anchor header=1693718239138125590ns；首个评价1693718239204891574ns；最后1693718249954357686ns。PointCloud2 header使用原整数时间，不使用ROS2 bag record时间或墙钟。无IMU预热、无未来GT。

作者PoseEstimator原生在首个实际`predict`（raw5）建立cool-time起点；前2s可能抑制运动预测。没有为了对齐旧EKF而修改该行为，也没有假造raw4滤波更新。这是系统差异之一。

离线逐帧握手：每条扫描等待同stamp的原生NDT日志、完整callback、status、odom及输入XYZ回执后再发送下一条；保留传感器dt，不声称实时回放。

## 单次运行与停止

160帧即使inlier<.50、未收敛、fitness异常或发生大步进也继续记录；这些均不新加到原UKF准入。只在程序/输入/目标身份违约、监控或存储无法继续、30秒回执超时、或非有限核心位姿/实际初值时停止。质量诊断NaN单独记录，不当作核心非有限。

不沿用R4连续20次外部无效或3次大跳变停止规则。作者原生每帧UKF correct不因研究质量筛选而改写；外部低inlier计数只用于报告。

## 封存后评价及判定

先关闭运行并对在线原始文件SHA封存，再打开官方参考。使用原R29插值合同：最大参考bracket150ms、最近端点≤50ms，平移线性、旋转SLERP；不跨大缺口、对齐轨迹或拟合时差。

位置误差`||p_est-p_ref||`，旋转`angle(R_ref^T R_est)`。报告位置Mean/RMSE/P95/Max、旋转RMSE及完整覆盖；全部高误差帧保留。持续失锁沿用R29定义：连续3个合法参考输出>.5m或>5°；连续3个均低于界限表示恢复。无效参考不证明恢复、也不填零。

通过必须160帧完成、全部有限、评价合同成立、没有持续参考相对失锁且RMSE≤.5m/5°。否则按实际定位失败、数据/坐标阻塞或工程执行失败分别报告。无论成功/失败都不重跑。

P62同窗重算D（原生无前门）、B（G-only）、C（C+G）。使用其已封存地图系L位姿和**实际状态时间**求参考，而非拿当前状态强配到较早扫描；列出相对scan时间差。这保持相同时间匹配规则，但不伪称两系统输出时刻完全一样。HDL无IMU，P62有IMU、不同源/目标处理和EKF/OOSM/限步，是系统级比较，不是C+G的一因素消融。

## 只读测量与运行身份

原版HDL SHA `de2dc7769aa2412876d05ae6161408d3295b6a3c`及原算法Release库全部复用。新trace ELF只在R3只读记录库上追加实际const guess矩阵，调用原native函数仍恰好一次；没有新优化/诊断评分调用或滤波决策。BUILD.json记录编译命令和身份。原生inlier是对齐后**距地图原点25m**内点的统计，不称全点Overlap。

RUN目录独占`formal_01`；原始UKF输出、原NDT终值、预测、点云转换、完整callback/额外诊断、进程RSS都保留。callback含仪表成本；RViz/输入/记录器与定位manager的内存分开。至少20GiB剩余空间，无历史删除。
