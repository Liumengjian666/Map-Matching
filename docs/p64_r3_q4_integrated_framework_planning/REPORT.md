# R3-Q4 集成框架规划验收

建议批准进入 **参考模块资格与离线影子集成**。V1 正常路径复用原生 NDT 和因果 IMU 更新。扩展从事件调度、候选和共享证据入手，先观察后接纳。正式 R4、线上连续系统及复杂模块实现未启动。

## 证据复核与设计决策

- R3-Q3 远端基准 4e3cfc8d2eb1cfd173462cea6765a3d122ddbd87，Git 文件树 67a381d3f7d446c4f4c290e9ce28893fb19fdb8c。独立工作区 /home/jian/p64_r3_q4_ws，分支 research/p64-r3-q4-integrated-framework-planning。实际稳定工作区 /home/jian/dog_visual_loc_ws HEAD 41999ea700c66c4cadf0eca9e0c5d73caa2783fd，仅作只读核对。工作树摘要前后 d234fc2c2abd3099f10ec0994f1bbf39bb821c00191450bb82c7d88481dad753。历史路径 livox_ws 下缺失。
- H307/H309 全点最大单体素恢复正确首选，M3/M6 和最大聚合组合后再错。H515 保持更高难度，空间等配额对照接近几何排序，不能将开发候选的分差作为在线正确性。
- B25 在仅 34 个双侧点的门控下翻错，W2_32 的双侧支持仅 11/11,016 点。Q3 三个真实因果新触发中无强竞争、错误首选或错误收敛。这些证据要求稀少支持进入可追溯回退，优先验证真实独立困难事件。
- X-ICP/DCReg、Park、EAMC、M3C2/EP、DeCOD、FE-LOAM、RegScorer 与 IKFoM 的原文和实际可用源码已对应。Park、EAMC、DeCOD、FE-LOAM 的完整作者源码未核实；相应适配不能称为原版。本阶段未做原版运行。现有 IKFoM 位于 FAST-LIO，稳定机器狗 EKF 和此前 P64 HDL UKF 是不同后端，V1 先复用真实基线验证接口，之后单独验收 IKFoM 接入。
- 采用两个独立告警轴判断局部退化与非局部歧义。候选到地图证据、共享观测、候选排序、全云精修和融合交给一条带时间与来源的接口。影子模式只记录候选动作。所有接受路径在一个提交端口汇合。

## 最小验证

在工作区根目录运行：

~~~bash
python3 docs/p64_r3_q4_integrated_framework_planning/interface_smoke.py
~~~

探针读取封存 B25、W2_32 的真实候选，核对两个候选共享时标和源云哈希、4×4 位姿矩阵方向合同的数值可逆性，以及影子输出和重试在探针中只留下一个提交令牌。B25 复现门控排序翻转，令牌仍指向原生基线。相关 4 个输入哈希和 H307 聚合交互均通过核查。该探针对归档运行，未执行配准、滤波、ROS 播放或实物部署。 [冒烟收据](interface_smoke_result.json)

## 交付与下一轮前置条件

| 文档 | 内容 |
|---|---|
| [INTEGRATED_FRAMEWORK_PLAN](INTEGRATED_FRAMEWORK_PLAN.md) | 事件分流、默认与回退策略 |
| [REFERENCE_MODULE_MATRIX](REFERENCE_MODULE_MATRIX.md) | 原文、数学证据、源码和使用边界 |
| [MODULE_INTERFACE_DESIGN](MODULE_INTERFACE_DESIGN.md) | 坐标、时序、协方差、输入输出、失败状态 |
| [DATA_FLOW](DATA_FLOW.md) | 正常/歧义分支与单次状态提交 |
| [INTEGRATED_BASELINE_V1_PLAN](INTEGRATED_BASELINE_V1_PLAN.md) | 最小实现顺序、模块开关、后续 roslaunch/RViz 验收 |
| [EXPERIMENT_ROADMAP](EXPERIMENT_ROADMAP.md) | 数据分类、因果与持出实验、逐阶段资格 |
| [INNOVATION_CANDIDATE_MATRIX](INNOVATION_CANDIDATE_MATRIX.md) | 可证伪的两项优先创新和调度备选 |
| [PUBLIC_BASELINE_PROTOCOL](PUBLIC_BASELINE_PROTOCOL.md) | A–H、外部系统、指标与统计 |

下一轮应独立提交成熟模块复现和影子链验证，并同时审计真实走廊参考。完整直接组合 E 形成可复现证据后，才选择一至两个创新机制进入对照。当前没有主张定位提升、实时性或原创性。 [R3-Q3 结果](../p64_r3_q3_geometry_reward_attribution/REPORT.md)
