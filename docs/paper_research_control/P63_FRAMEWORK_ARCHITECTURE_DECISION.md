# P63 研究主干结构比较与选型：2026-10-26
状态：**第一候选，不等于完成原版复现；P63-R1须做有界源代码及真实MID360/PCD证据验收**。

## 用户指定研究任务
冻结既有地图（可由FAST-LIVO2产生）＋已知地图系初始位姿＋连续LiDAR–IMU scan-to-prior-map跟踪，重点退化/错误候选/低算力恢复；不是从零位置做全球重定位/地点检索，也不是纯在线建图或仅FAST-LIO里程计＋偶发地图闭环。

## 体系结构区别
A 原始FAST_LIO_LOCALIZATION及SCANCONTEXT/Quatro/Nano-GICP系列：FAST-LIO运动估计为主，后台低频粗/精或关键帧地图匹配更新全局变换。参考官方https://github.com/engcang/FAST-LIO-Localization-SC-QN 、https://github.com/YasuChiba/FAST_LIO_LOCALIZATION_ROS2 。与**每帧冻结地图直接约束状态**研究问题不完全一致，不能误称所有名叫FAST-LIO Localization都属于该结构。

B 另一2026研究fork https://github.com/QuintinUmi/fastlio_localization：其README明确宣布直接加载PCD到固定ikd-Tree，并在每帧FAST-LIO2 iEKF中使用scan-to-prior-map点到面残差，ROS1和MID360 launch可用。但这是研究fork，不是直接引用为2026正式同行评审论文；正确性、冻结map/add_points与性能必须验收。若首选不能实际工作，这是最贴近逐帧prior-map的候选之一，但勿未经批准同时开发。

C Super-LIO，Liansheng-Wang/Super-LIO，2026 IEEE RA-L 11(3):2666–2673，ROS1 ros1 + Mid360：
- 官方README说明LIO主体论文创新在OctVox + HKNN，论文评估主要为里程计/建图，不能冒称prior-map模式已有同样公开定位精度。
- `src/super_lio/src/lio/super_lio_reloc.cpp` `map_init()` 加载外部PCD并插入ivox、一次NDT→ICP初始化；`src/super_lio/src/lio/super_lio.cpp` `stateProcess()` 每帧`Propagation_Undistort();DownSample();Observe();UpdateMap()`；`Observe()` 内 `kf_->UpdateObserve` 使用ivox点邻域匹配。**它的连续优化不是NDT，不能称其为“NDT改进框架”。**
- `relocation.yaml` 默认`relocation.update_map: true`，`SuperLIOReLoc::UpdateMap()` 首行`if(!g_update_map) return`，否则继续`ivox_->insert`；必须显式关掉并检查其他地图写入，设置哈希/容量不变量实际回放。继承主类的`SuperLIO::UpdateMap`确实会插点，但重定位派生类覆盖UpdateMap；ROS入口是否构造派生类也已核查，需要编译/动态调用再确认。
- 适合论文从严格NDT放宽至轻量级robust LiDAR–inertial prior-map定位，前提是原版冻结地图定位短段可跑，完整序列能成功。
- 作者源码 GPLv3，必须依许可证履行相关义务；复用论文引用其原创里程计/地图与滤波部件，不可以当作我们原创贡献。

D koide3/hdl_localization：老但**任务最纯粹的NDT**：全球点云＋已知初始位姿＋UKF/可选IMU预测＋每帧ndt_omp scan-to-map校正；ROS1 PCL，原作者公开程序状态和有效匹配比例，优化插件容易替换。如果用户要求论文严格保持NDT、尽量不改原研究题目，应优先它而非把Super-LIO的迭代点到面算法误写成NDT；缺点是旧、原版全地图目标可能不能满足100MiB，需新地图管理/低内存模块，是实质额外工程。
E superxslam/SuperOdom 2025 SuperLoc：更接近“退化的先验地图”科学问题，官方ROS2 Humble及map模式，但与用户Ubuntu20.04/Noetic运行环境隔离，会有更多适配工作，而且已有退化几何风险方法不应重复。适合作为论文研究先例/强对照。官方链接 https://github.com/superxslam/SuperOdom 。
F robotics-upo/G-EDF-Loc 2026 IROS：先验连续高斯距离场定位，直接scan-to-map，可选IMU，ROS2 Humble；**必须先生成G-EDF .bin地图，不能把FAST-LIVO2 PCD直接当作输入**，地图转换与底层表示偏离原研究。参考 https://github.com/robotics-upo/G-EDF-Loc 。
G M-Evanovic/LL-Localizer IEEE TIM 2025：论文确实研究先验地图定位；但官方README明确承认论文实车ROS2完整代码未开源，只公开基于Faster-LIO的ROS1简化demo。不能认为复现demo即复现论文完整系统。参考 https://github.com/M-Evanovic/LL-Localizer 。
H Koide ICRA2024论文，作者项目页明确“Closed-source code”；官方数据和结果可以作外部比较，不能选作实际代码母体。

## 最终有条件推荐（只批准一个候选）
优先 **Super-LIO ROS1**，理由：2026正式论文、ROS Noetic/MID360、原生紧凑地图和成熟ESKF、具有PCD重定位派生类，易避免自建整套融合。启动前明确论文名称转“prior-map LiDAR–Inertial localization”而不是严格NDT算法创新，新增算法只选连续冻结地图匹配/退化可靠性单一薄弱点。失败止损：如果Reloc经`update_map=false`仍写图、已有MID360数据无法合法工作、真实连续匹配输出不能稳定或满足资源预算，结束第一候选而不是无限修框架；随后对「FAST-LIO2直接prior-map iEKF研究fork」与「hdl_localization原生NDT」做一次选择，按用户是否坚持NDT优先。不能因为发布时间晚就认定稳定。

## P63-R1执行范围
只对Super-LIO原版：冻结源码/依赖提交、编译ROS Noetic、复用现有MID360+真实PCD和初始位姿、`relocation.update_map=false`、追踪ivox地图写入、300–600扫描的真实一键RViz先验地图定位短段、报告RSS/CPU/延迟/定位退化与错误；不下载大型Outdoor、不移植旧自研C+G、不建新EKF/NDT；必要最小传感器适配记录单独patch，不通过就STOP并回传。对原版完整真实轨迹比较之前**不开展创新开发**。只有证据通过才批准P63-R2完整公开序列。

该决定取代“仅因为Super-LIO新就直接作为最终主干”的表述：**它是第一候选而非已验证唯一选择**。原有R29–R31失败与R32未执行的历史事实不可删除。
