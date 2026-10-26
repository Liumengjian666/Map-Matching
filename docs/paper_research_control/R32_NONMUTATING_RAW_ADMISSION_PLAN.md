# R32 预注册：非改写式 NDT 地图观测与运动一致性证据（2026-10-26）

> 本文件为下一阶段**计划，不是已执行结果**。由科研总控批准计划、Codex在用户本地独立工作树实施；代码测试及运行结果须另外归档。不得把任何工程修复描述为已证实的SCI创新。

## 0. 输入身份与已证据

START: R31 archive END `65ed408fba0c7adb406c7ec487fd3aba6860ffab`（真实远端分支已核实），执行分支须从本R32规划分支HEAD衍生，留独立CODE/END提交。稳定机器狗工作区不动。

R31 Koide `indoor_easy_01` Azure Kinect＋IMU、raw4锚点/raw5–2027完整连续2023帧:
- LEGACY G-only/C+G平移RMSE630.506992m、旋转16.232952°，实际融合137次；两者完整位姿一致。
- STRICT G-only/C+G平移RMSE5334.892892m、旋转26.538893°，仅融合4次、最长1978帧/136.038151秒无地图观测；C仍多做96次NDT、无独立收益。
- R31 65条实际发布观测与原始质量认证终值同一（工程PASS），但这是「拒绝被限幅改写的观测」，不是定位性能成功。
- raw62: 旧基线NDT原生终值通过优化后质量门，限步把终值修改0.446248866m，参考位置误差0.425469771→0.755997115m；STRICT拒绝并导致后续饥饿。
- Newer College NC_W1/C3开发60帧STRICT与历史均0.209942842m/2.984149397°、57融合；不是新独立验证。Base/RS_C1参考解释仍有条件。
- R30 的global-PCD高斯网格构建顺序已经修复，但该BUG不解释R29 Tile模式失败。R29 Azure原始IMU存在52段>50ms未积分（共3.17164625秒），本轮不宣称已修复。

## 1. 本轮唯一科学问题
固定帧数/0.5m、5°的**上一发布位姿限步器**无视长时间没有地图观测这一事实，直接改写被认证的NDT终值；严格拒绝又带来观测饥饿。问：
**如果不改写已认证原始终值，而是保留几何质量门并将运动一致性按真实时间与因果预测作只读观察，是否能获得更多合法地图更新、同时避免明显错误接纳？**

此轮不预设数学创新或者必须提高RMSE。首先验证一个最小正确性假设；如果不够，不继续只靠加门限碰运气。

## 2. 修改与实现边界
源码真实点位（须本地再核查）：
`src/dog_prior_map_localization/include/dog_prior_map_localization/observation_identity.hpp` 决定发布策略；
`src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp` 的 `applyR23Admission()`、`limitNdtStep()`、`publishPose()` 和 `ekfPredictionAtOrBeforeLocked()`；
`src/dog_prior_map_localization/src/fusion/ndt_observation.cpp` 原有 OOSM/校正和速度融合，不在本轮修改。

唯一正式新策略：`CERTIFIED_RAW_DIAGNOSTIC`（研究专用、默认仍 LEGACY）：
- 保留 R31 原有质量门、有限/收敛/刚体检查、C+G搜索、外参、因果EKF预测合同；**仅在当前原始候选被认可时直接发布它的原始SE(3)**，不对该位姿套用上一发布值的 `limitNdtStep`；时间戳、坐标系和真实 `publishPose` 均保持原合同。
- 必须在同一时刻额外记录旧限步器「如果启用将改动多少」，以及 (dt=t_{scan}-t_{last_actual_map})、原始终值相对于同时间 `EKF prediction` 的平移/旋转 innovation（差异）、预测时间年龄、IMU gap资格、EKF correction magnitude/OOSM回执、Source/Target/质量值。**这些统计本轮只做shadow，不作为新门限，不按GT调参。**
- EKF内部现有限制和OOSM规划保持不变，不能声称这种模式无风险；此策略只用于判断 R31 观测饥饿是否来自旧限步器，**不得用于实物机器狗自动控制**。
- 遇到未收敛、质量失败、非SE(3)、未来EKF样本、不合法时间戳，保留现有拒绝；不可把「原始候选通过」当作「真实GT正确」。
- 所有实时发布地图观测的SE(3)必须对应已认证的同一个原始终值。发布失败帧不更新 previous_pose_/delta_pose_/Path/TF/地图观测历史。
- 不修改IMU max_dt、0.8m NDT分辨率、Coverage阈值0.30、±5°搜索、每60帧额外NDT预算、NDT优化器、EKF/OOSM数学、局部Tile与初始化，不复活早期D/E/F/Coupled。

## 3. 工程测试
单元测试要求：
1. LEGACY和STRICT的历史行为不变；新策略默认关闭。
2. 合格原始终值即使旧限步器会改写，仍只发布**同一个原始值**（不能偷偷发布限幅候选）。
3. 不合格/无收敛/数值或时间无效照旧拒绝。
4. 拒绝时所有导航发布计数、上一地图观测与时间历史不变。
5. 通过时新发布的位姿及时间标签与候选终值一致。
6. 旧限步器影子测量不触发第二次Native Full NDT，也不修改状态。
7. raw62 冻结原始矩阵：旧限步提议、STRICT拒绝、新研究模式选用原始终值；本例只用于语义复现，GT仅作为事后评价。

完成ROS-free单测、Release节点/CTest和运行二进制冻结后才能正式回放。

## 4. 一次有界实验，不重复无限审计
G1 开始前冻结实验合同、原始data+地图hash、初始状态、参数、输入顺序与可执行CODE_SHA。
G2 真实因果全序列：仍先用已有Azure `indoor_easy_01` 原2023帧，运行G-only＋CERTIFIED_RAW_DIAGNOSTIC、C+G＋CERTIFIED_RAW_DIAGNOSTIC各**一次**；历史LEGACY/STRICT结果复用，不重跑。完整保留失锁轨迹，不挑结果，不中途重置，不使用未来GT。最多2条新增完整轨迹。
G3 正例保护：已有NC W1/C3 60帧、C+G 新模式至多一次，条件性参考；历史60帧只用于回归，不当新独立证明。
G4 计算开销：完整NDT/EKF NDT+Deskew/输入至最终确认Mean/P95/Max、采样峰值RSS、额外NDT次数，必须披露非法或无资格扫描。
G5 不论G2成功或失败，须给出**下一步MID360真正推进所需清单**：已持有Koide Outdoor01/02原始文件吗？若没有，列出官方资产及所需大小、磁盘/外置盘路径、预先保留≥20GiB可用空间；无需在本轮下载全部Outdoor或开展正式Outdoor定位。若用户已批准并存在原始资产，允许做**只读**头/点内时间/IMU单位/外参/GT地图合同检查，不能凭元数据直接标PASS。
G6 立即评估两种可能：
- 新模式明显增加有效地图观测并显著优于STRICT：证明旧固定限步/拒绝链是观测饥饿因素，但不能说已在MID360或外部基线证明精度优势。
- 新模式接纳错误候选、轨迹仍失锁，或结果与G-only相同：保留所有负例。不要通过GT调Coverage或创新名词解决；下一步直接转真实MID360资产+基础链路而非继续Azure多轮扫参。

**事前安全界限**：本次新策略属于故障研究，禁止使用于真实机器人动作执行；所有注册版本只在离线/ROS回放运行。若检测到观测不变量违规、时间穿越、编译/输入身份不一致，停止。若定位继续持续失锁，允许完成一条正式负轨迹以保留完整结果，但取消冗余第二长跑需明确预注册规则及理由。旧R29/R31结果不可覆盖。

## 5. 科学验收
ENGINEERING_PASS：每一发布地图观测==同帧已认证原始终值；上游质量门保留；状态、时间、回执因果合法；额外shadow绝不改变真实轨迹；单元/Release通过。
PERFORMANCE_REPORT：完整全段位置/姿态RMSE/P95/Max、失败及恢复、实际NDT尝试和融合、错误接纳proxy、最长无地图观测、计算/RSS；不强制PASS，失败必须报告。
SCIENTIFIC_STATUS：仅Azure深度点云上的结果，不得将它升级成已有MID360成功或与公开HDL/FAST-LIO Localization公平领先证据。

## 6. 唯一下一决策
本轮结束必须提供：继续保留正确性接口的情况下，是否已有足够证据去设计单一 **Time-aware Non-mutating Admission**（基于真IMU时间、有效预测和运动不确定性）创新，还是更应直接移至 Outdoor01 MID360输入合同？严禁直接发明没有实测约束的NIS硬切换/新阈值。
同时更新长期总控 `docs/paper_research_control/MASTER_RESEARCH_PLAN_AND_LOG.md`：R31完整负结果及本轮真实输入、代码/数据hash、NDT/EKF实际调用、计算、存储、下一唯一阶段。不得自动删除任何文件、下载大型资产或启动新的未批准阶段。