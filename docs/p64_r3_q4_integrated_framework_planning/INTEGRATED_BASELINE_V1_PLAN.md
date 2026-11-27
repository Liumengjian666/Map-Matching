# Integrated-Baseline-V1 实现计划

## 优先级与可关闭项

| 层 | 第一实现与依据 | 开关、回退 |
|---|---|---|
| 正常链 | 复用已核实的 HDL PoseEstimator 原生 NDT+IMU 短窗、读地图和姿态评分入口，以同一后端复现 A。行为对齐后在独立 Adapter 接口加 IKFoM，另建同后端 A/E 对照。原生状态向量由选定后端保有 | pipeline_enabled；关闭时只用原生链。backend=hdl_ukf/ikfom，单次运行只选一个 |
| 事件检测 | 先读原生迭代、关联、创新、局部正半定信息代理；并行保留 X-ICP/DCReg 参考记录 | local_degeneracy、support_innovation、periodic_probe 独立设置；探测或诊断失败保留原生准入逻辑 |
| 候选生成 | 复用 Q1 Park 水平离散评分流程和峰提取，Q2 双中心作为已有适配对照。工作站离线资格与后续平台预算分立。沿用单调预算，日志分辨好多峰/平坦/未知 | search_mode 关闭、Q1 冻结网格、Q2 冻结适配；候选不足或预算用完回退 |
| 预期视图和结构 | R3 首交/局部裁剪、双侧支持与三态。对照普通体素、同支持池空间均匀、最近邻差异。U4、M3C2 型法向纳入独立开关 | view_enabled、geometry_mode=support_only/nearest_neighbor/normal_proxy，默认影子。DeCOD 和 FE-LOAM 仅在独立可行后替换 |
| 评分 | 默认 Native DIRECT7，保留完整点集排名。多候选用同一掩码，可复用 M6 作直接对照 | score_id=native/max_held_neighbors/mean_held_neighbors/park_log，后 3 项只影子打分。禁止切换原生优化目标 |
| 验证与精修 | 同一帧空域外留出区域，检查双侧支持点数、占比与分布、模式是否判别、后验创新与全点一致性 | refine=off 初始；通过候选和可靠性门槛后，允许一次从首选启动的全云原生拟合及越界否决。真正有界求解后续单验 |
| 状态提交 | 原生有效、候选可信、部分约束、传播四种有日志的动作 | 初始 shadow；active 阶段先启 ACCEPT_NATIVE/ACCEPT_REFINED，再只对已校准的后端开 PARTIAL。时间超限与全错均通过回退入口处理 |

融合计划保留现有 EKF 工程和 P64 HDL UKF 为两种历史参照。实际稳定机器狗工程中的 EKF 延迟观测处理与 FAST-LIO 端 IKFoM 来自不同实现。IKFoM 接口适配需要独立的测量模型、切空间传递、可变秩噪声和同源去重。不能复用 FAST-LIO 内点面量测关系作为新的 NDT 位姿观测，也不把切换滤波器的收益计作几何模块收益。

## 分阶段接通

1. 以固定历史候选做接口冒烟，验证 point_id、source_id、目标分数、坐标以及回退。Q3 的 B25 稀少支持和 H307 掩码交互必须保留。此步可以纯离线读取归档。
2. 进行不超过预注册时长的因果离线短窗，所有扩展置于影子。要求正常链位姿和地图更新时序与同后端 A 一致，每帧只有一次提交。输出计划动作与实际动作的差异。
3. 单独复现成熟退化检测，先记录参考输出，再允许检测调度。固定候选和筛选证据都通过后，允许可撤销的只读候选判别并测试精修，最后才在受控子集开放融合。
4. 在独立 ROS1 Noetic 研究包实现运行与记录。软件在工作站达标后再部署 Jetson Orin Nano，记录 Jetson SKU、功耗模式、CPU 频率、线程数与 ROS 回放实际进速。

## 一键 roslaunch 验收规格

计划入口为 roslaunch p64_integrated_localization replay.launch bag:=ABS_BAG map:=ABS_PCD config:=ABS_YAML rviz:=true，以上包名与入口是未来实现合同。启动加载固定图 SHA、外参和算法开关，启动 rosbag 模拟时钟与播放、配置帧、发布先验地图和单主定位话题。RViz（ROS 可视化）显示实际点云、地图、定位轨迹、局部退化/非局部歧义、竞争位姿轴、未知及差异和回退状态。通过独立运行目录保存参数、日志、Git SHA、地图哈希、输入匹配时标以及近实时队列。正常轨迹与影子诊断可同时显示，影子状态不会广播主 TF（坐标变换树）。不修改稳定工程的历史 launch 或 YAML。
