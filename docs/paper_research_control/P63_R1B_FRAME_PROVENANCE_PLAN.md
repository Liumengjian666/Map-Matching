# P63-R1B：外部先验PCD、重力轴与IMU/LiDAR坐标合同的有界闭合（计划，非已执行）
日期2026-10-26；科研总控决策；前提R1 END=2a2f202c01efe6a6452436bd1eff084ae1ddfc14。

## 唯一科研问题
原版Super-LIO ROS1在已存在loop2 MID360+FAST-LIVO2先验PCD上：代码Release、固定地图和输入接收PASS；连续跟踪在107扫描已发/95输出后失锁；初始匹配fitness=0.10959但估计9.4s位移225.114m（不是GT误差）；静态比力经初始化位姿变换后与map +Z差38.021059°。为何原地图的实际重力轴／地图生产坐标／T_I_L不能与原版Super-LIO当前世界重力和初始化合同相互认证？确认输入/坐标前不得评价原版定位能力。

## 源码和原始证据（禁止重新自研ESKF）
- 用户仓库docs/p63_r1_mature_prior_map_baseline/REPORT.md、P63_SENSOR_FRAME_TIME_CONTRACT.md、P63_MAP_FREEZE_CONTRACT.md、P63_ORIGINAL_BASELINE_RESULTS.md、results.json、evidence/LOOP2_READ_ONLY_INPUT.json；
- R1实际来源Super-LIO ros1 SHA60b57aaac8dc397f80c56364e7ccb008c300cc29；
- 作者`src/super_lio/src/lio/super_lio_reloc.cpp` kf_init NDT+ICP初始姿态、重力投影；`src/super_lio/src/lio/super_lio.cpp` IMU传播＋点面ESKF，`config/relocation.yaml` 更新地图false；
- 现PCD /home/jian/rosbag/loop2/loop2mapping/pcd/loop2_simtime_rebuild_2026_08_12_001_all_downsampled_points.pcd，SHA256 016b7ae3ae494ad5d462a66d888a96b96dda5fe29d6058059aa2fa6bdf6b09ca；
- bag /home/jian/rosbag/loop2/raw_restore_20260914_stage0/loop2_raw.bag，历史SHA d17bee3f8697dcb01cb4b9f6ea2483aa42a1a31ae277659c8a7869d9c69f13ae；不必重复全20GB哈希但不得冒称新验；
- 8月8日mapping.yaml extrinsic_T=(0,0,0)、R=I、gravity_align_en=false；**这组文件未完成8月12日simtime重建PCD唯一生产绑定**；
- 本机没有可用物理LiDAR–IMU标定；历史单位阵只能作为授权条件性Smoke，绝非通过校准。

## 一、只读来源闭合（先执行、0定位回放）
1. 查唯一生成目标simtime PCD的脚本、roslaunch、配置、地图生产运行receipt/log、建图轨迹源文件、原始PCD/生成前后的SHA及时间；判断能否建立“这一PCD确实由这一mapping版本／外参／gravity_align设置生成”的证据链。查本地资产即可，**不大范围扫盘/不下载**。输出PROVENANCE: PASS / CONDITIONAL / BLOCKED和各证据。
2. 按T_A_B表示B到A的物理旋转，画清`M(map), I(IMU), L(LiDAR), B(robot)`及FAST-LIVO2 camera_init与Super-LIO world的意义；厘清哪些是可由地图生成轨迹证明的初值(T_{Mleftarrow I}(t0))、哪些是历史人工假设、哪些未有物理标定。不得根据Super-LIO本次失败轨迹或GT选择旋转。
3. 如能合法追溯**独立于新定位输出**的地图生成轨迹与同时间静止IMU，则将特定加速度样本在与源数据同一时刻用(R_{Mleftarrow I}(t))转为地图坐标，判断静止比力正向和理论gravity方向（明确正负号和单位）；核查帧名、数值轴向是否已经有重力对齐。静止性要由IMU方差/gyro/时间连续性和环境证据支持，不能拿运动平均代替精确重力。若轨迹文件没有这一PCD的唯一生产身份，标CONDITIONAL，绝不伪造硬PASS。
4. 对照源码重现R1作者初始化由加速度推出roll/pitch再乘人工yaw与NDT→ICP输出的完整变换顺序，并审查其与地图系重力关系。如果38°只反映大地图frame任意倾角，不应直接断言外参物理差38°。列不超过三项有原始证据支持的假设和可证伪判据。

## 二、可选的一次**独立证明后**冻结基线重放
只有G0来源可追溯、G1 map/world重力方向定义认证、G2初值和T_I_L使用合规（如物理外参未知只能标CONDITIONAL而非PASS）且有明确无需GT优化的确定性坐标变换时，才允许在独立新工作树以**一组预注册、一次性**纯坐标适配重新运行**首400扫描**原版Super-LIO。所有变化写一份固定 T_adapter 及其来源，不得修改作者ESKF、地图几何（尤其不能对地图做优化拟合）、NDT/ICP门槛、OctVox、IMU单位/时间、运动参数；不得运行不同角度寻找最优结果。冻结对照旧R1，0次外部基线重跑。
- 成功：≥400/400输入有完整收据，连续跟踪≥380输出、尾100输出有≥4有效平面匹配的比例≥90%、不触发原20连败Stop、位姿有限且无显见百米级瞬移；真实响应、地图几何SHA保持，RViz地图/雷达/轨迹叠加验收。注意这只是基础可用性，不是正式GT精度。
- 失败：任一记录不真实、初始化身份不闭合、未达到上述标准，报告阻塞/负例并STOP。不准试第二旋转或调门限。
- 如果没有独立生产来源而无法合法建立适配，**不运行新Smoke，按BLOCKED停止**。提出下一个单一路线：使用官方地图/标定完整且预装磁盘资产的数据集做Super-LIO原版入门（例如有可信标定的公开MID360）、或者用户坚持NDT时有界核查hdl_localization；不自动下载或起另一个框架。

## 三、内存和时限
已知root free约25.22GiB，至少留下20GiB余量；未经批准不得删除老数据/拷贝19.7GB bag/下载新的Outdoor大载荷。优先只读本地已知生产配置和小型轨迹/日志。允许新少量报告、小于预先校验容量的编译cache/适配文件，不允许大范围重复索引或无界构建。禁止研发新质量门、恢复算法、C+G、EKF或改动稳定机器狗。

## 四、交付和科学判定
FRAME_PROVENANCE_CHAIN.md，GRAVITY_DIRECTION_AUDIT.md，EXTRINSIC_AND_INITIALIZATION_CONTRACT.md，RUN_ELIGIBILITY.json，若合法且实际回放则 ORIGINAL_SUPERLIO_CONDITIONAL_REPLAY.csv、MAP_FREEZE_AFTER.json、RUNTIME_MEMORY.csv、RVIZ_ACCEPTANCE.md，否则明确「无新定位运行」。R1B_FINAL_DECISION.md：SOURCE_PASS/CONDITIONAL/BLOCKED，GRAVITY_PASS/CONDITIONAL/BLOCKED，EXTRINSIC_PASS/CONDITIONAL/BLOCKED，BASELINE_ACCEPTED/FAILED/NOT_TESTED，及唯一NEXT。
更新 docs/paper_research_control/MASTER_RESEARCH_PLAN_AND_LOG.md 阶段、真实执行数值、原始失败、文件占用和待批删除目录；保存Git START/CODE/END及官方Super-LIO精确SHA，报告是否push与远端真实HEAD。未经用户逐项批准不得删除。避免修改现有R1归档结果或换数据集来隐瞒失败。