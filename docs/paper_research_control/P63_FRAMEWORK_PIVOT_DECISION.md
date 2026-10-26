# P63 框架迁移总决策：继承已发表的先验地图定位实现（2026-10-26）

## 批准边界
过去R29–R31在自行维护IMU/EKF/OOSM/NDT/Tile接口上产生大量工程开销，Azure Kinect 2023帧完整失败，STRICT导致2023帧仅4次观测；短窗口NC成绩不能弥补完整基准缺失。因此**暂停R32旧自研接口实验（仅计划，未执行），不再把现有自研NDT–EKF作为SCI投稿主干**。保留原Git分支、负结果、地图和实验工具供分析/参考，不删除、不覆写；不是否认既有知识投入。

主科研题目由固定“自研NDT定位算法”调整为“低资源 LiDAR–Inertial Prior-Map Localization（轻量级激光惯性先验地图定位）”；如果后续必须坚持NDT为连续优化内核，则应重新选择真正的NDT原生开源定位骨干（如HDL Localization），不能将Super-LIO后端称为原生连续NDT。

## 候选优先级与核实证据
|优先级|候选|版本/直接支持|已验证的代码事实|不能假设|
|---|---|---|---|---|
|1|Super-LIO，Liansheng-Wang/Super-LIO|2026 RA-L、ROS1 ros1分支、Ubuntu20.04/Noetic、Livox MID360|src/super_lio/src/lio/super_lio_reloc.cpp 有loadPCDFile和ivox_->insert先验地图；初始化使用NDT+ICP；SuperLIOReLoc从SuperLIO继承ESKF与数据管线；官方ROS1 relocation.launch|config/relocation.yaml 当前relocation.update_map=true，UpdateMap()可插入当前点云；必须将其设false并核实无其他map写路径才满足冻结地图；不能仅凭论文推断RSS<150MB或已有成功外部GT|
|2|JoCatW/FAST_LIO_Relocation|FAST-LIO2固定PCD定位，ROS Noetic，Mid360，IKFoM|提供mapping/localization状态机、map PCD、质量指标与跟踪；用成熟FAST-LIO更新核|GitHub说明带NDT版本可能不完整；不能把FAST-LIO点到面扫描注册硬称NDT；license GPL-3.0|
|3研究参考|superxslam/SuperOdom / SuperLoc|ICRA2025，ROS2 Humble官方、LiDAR/IMU、冻结地图模式|官方doc/LOCALIZATION.md有localization_mode、指定map_dir、initial pose、退化风险预测|目前与Ubuntu20.04/ROS Noetic不直接对应，GTSAM/Ceres/Sophus迁移开销高，不先搭两套OS|
|4研究参考|koide3/glim|2024–26持续维护、MID360、多雷达、因子图扩展|支持MID360与可插拔扩展|主README强调建图，不能未经接口核验就宣称现成外部PCD持续定位；系统依赖/资源开销需核查|

本表只反映GitHub官方文档/源码核查，不等于本地编译和定位测试已经完成。

官方链接：
- https://github.com/Liansheng-Wang/Super-LIO/tree/ros1
- https://github.com/JoCatW/FAST_LIO_Relocation
- https://github.com/superxslam/SuperOdom
- https://github.com/koide3/glim

注意引用原作者论文、LICENSE/GPL衍生代码义务，不直接将其已有滤波、紧凑地图或NDT初值称为自己的创新。

## 唯一下一阶段 P63-R1 ：成熟定位框架基线迁移
目标：在真实现有MID360数据+先验PCD上跑通**未改核心算法的开源定位系统**，避免重写时间同步/滤波/地图匹配。最先只试Super-LIO ROS1，不要同时复现一堆系统。

G0 审查真实代码：克隆浅深度完整源码（含必要子模块）、锁SHA与LICENSE，列表列出input lidar/imu msg定义、点内offset_time、IMU单位、LiDAR–IMU外参方向、初值T_map_lidar、PCD格式和世界/传感器帧语义；找所有地图insert/UpdateMap路径，确认`relocation.update_map=false`是否真能使所有先验地图在运行后不变。原仓库如有map更新风险，只允许*必要配置或一项隔离式冻结地图修复*，记录patch；不能修改原作者算法数学或自动合并旧C+G。
G1 独立工作树和环境：Ubuntu20.04/Noetic下构建Super-LIO ROS1原版。先用其自带/现有最小样例完成可执行正确性证明；已知本机真实bag及FAST-LIVO2地图可重用，优先已存在loop2一段，**不下载大数据**。如果需要转换PCD字段或消息类型，写独立可复用adapter，不改滤波数学；先保存hash，不覆盖原map/bag。实际磁盘剩余应保留≥20GiB。
G2 独立原版基线回放：使用已存在的一段真实MID360 scan+IMU+先验PCD；首先只跑短段（约300–600帧，若资产不足就停止为环境/资产阻塞），实时发布Map、PointCloud、TF、轨迹，RViz一键roslaunch验收；追踪是否每帧确实与先验地图对齐（不能仅凭odometry发布认定定位）。合法起点初值来自前一次已冻结地图/设定的人工初值，不利用未来GT进行在线预测。
G3 汇总是否值得作为主干：原版真实输入处理和轨迹（需可靠参考才能报RMSE）、匹配/失锁统计、RTF/mean/p95延迟、实际同时RSS、冻结地图检查、源码可修改插点（只需找到一个可独立验证的扩展点）。所有技术门通过则选择Super-LIO为后续主干；若无法合法冻结地图或运行失败/依赖不可解决，只记录明确阻塞和转换成本，再建议是否切到FAST_LIO_Relocation，**不得擅自重写第二套框架**。
G4 论文创新开始条件：主干original baseline至少一条完整真实LiDAR先验定位序列合法运行；之后才针对确实可复现的几何退化/重复结构/错误匹配选择*一个*模块创新。理论选择可考虑匹配可靠性/恢复，而不是先直接把旧NDT C+G强塞入Super-LIO原有ESKF；如果强制保持NDT主题，则重新决策换原生NDT系统。

## 之后仅作为规划，不自动执行
P63-R2 原版完整Koide Outdoor01→Outdoor02（若官方外部PCD与Super-LIO导入语义合法）；引用Koide2024 HDL/FAST-LIO Localization发表结果并保持ATE/重启/评分差异显式注释。若原版失败则先按新框架源码和传感器合同归因，不能用旧自研源码推测。
P63-R3 研究一个确凿短板（例如重复结构下错误scan-map更新或初值恢复），数学假设、与原版唯一模块不同的机制、有限研发/heldout协议、资源预算。
P63-R4 公开基线原版vs我们改进版（**原版需要亲自跑一次**，是修改框架的必需因果对照；其他独立经典算法优先引用同数据公开结果）、完整两场景、消融、论文。

## Git和存储
每轮更新 MASTER_RESEARCH_PLAN_AND_LOG.md；保存源分支、SDK、依赖、地图与数据唯一保留副本SHA。只允许清理Codex明确列出且用户逐项批准的重复/可再生文件；保留全部R29–R31负实验结果和提交。若与既有分支协作，创建独立worktree不要改机器狗稳定项目。
