# 科研总控 / 外部 AI 复核回传提示词

请复核 **PAPER-P6-ALG-INTEGRATION-A2D：WINDOW-OWNED MEASUREMENT GENERATION AND
LOW-COMPUTE SOLVER CLOSURE** 的源码与工程证据。你负责数学验收和阶段决策；
以下是电脑端执行 AI 的完成记录，不是请求你替执行 AI 运行本地程序。

## 1. Git 身份和授权范围

Repository: https://github.com/Liumengjian666/Map-Matching.git

Branch: `research/p6-i6d-full-algorithm`

START_SHA: `b04a1f7f3442266dd48bd2efea5baf09d905b0c8`

CODE_SHA: `14f0e965639ce7c0e022e77a750670024b5aff37`

END_SHA 是包含此报告的后续 documentation commit；执行 AI 最终消息给出完整 SHA
和实际远程状态。请先核验 GitHub 分支/该 commit 可读，再基于真实源码复核。
若用户尚未完成手动 push，本地“已提交”不等于 GitHub“已上传”。

工作区始终为 `/home/jian/livox_ws/dog_loc_p6_i6b_ws`，没有新建 worktree。
工程结论为本轮授权范围 PASS；**READY_FOR_FORMAL_EXPERIMENT=NO**。
本轮没有运行 Floor01/Corridor01 完整轨迹，没有 GT 调参，没有修改 frozen
baseline/旧 FULL 默认行为，没有重新设计估计器或重写 PCL NDT。

材料目录：
`src/dog_prior_map_fastlio2_frontend_exp/docs/p6_alg_integration_a2d/`。
该目录交付了 summary、来源审计、deskew 数学/测试、visual provenance、
dense waste、解析 Jacobian、dense/sparse 等价、scaling、Schur、构建/CTest、
源码 SHA、changed_files 和小型 scaling CSV。没有 bag、图像、PCD 或真实轨迹大文件。

## 2. 已确认的输入来源事实

Floor01 `p6_i1_prepare_inputs.py` 从旧 runtime `message.cloud_end_frame`
导出 `request_xyz_f32.bin`。SHA/hash 证明输入身份，不证明其不依赖旧状态。
`p4_i3_visual_increment.py` 的 depth association 使用 `old_req.cloud_end_frame`，
因此现有 Floor01 visual 是 **LEGACY_STATE_DERIVED_DEPTH**。

Corridor01 `p6_i6c_prepare_corridor01.py` 导出的点云语义为
**SENSOR_LOCAL_ROTATION_ONLY**，明确没有平移去畸变，也不应再错误地重复旋转去畸变。
以上两个 XYZ-only prepared bundle 都没有真实逐点时间：
**RAW_POINT_TIME_UNAVAILABLE / NOT_ELIGIBLE_FOR_WINDOW_OWNED_DESKEW**。
本轮没有给这些真实数据按 index 或均匀规律伪造 point time，也没有伪造新相机测量。

## 3. Window-owned deskew 与 V3 生产链

新增 `window_scan_processor.hpp/.cpp`，不要求 IKFoM candidate。
重构 `window_imu_factor.cpp`，使 `preintegrateImu()` 与 Window trajectory 调用
同一个 bounded integration loop。原有 midpoint-input / left-knot-R 离散模型、
bias Jacobian、噪声密度和 covariance transition 保持不变。
adapter 的节点预测和 trajectory 也共享同一个 endpoint propagation。

对重查后的 scan-start 15D 状态 `{R,p,v,bg,ba}` 和固定 g，积分生成 `T_M_I(t)`，
采用真实 `T_I_L`，点变换为：

`p_end = T_M_L(end)^(-1) T_M_L(t_point) p_sensor`。

纯几何从旧 ScanEndProcessor 提取为共享 helper；旧 processor 只作 legacy
regression，绝不进入 V3 runtime。相同轨迹下的几何结果已比较。

V3 新增 `LIDAR_SCAN_START` 和 `LIDAR_SCAN_END`。start 创建真实 Window 节点；
scan 内 visual event 可以联合优化该节点；end 时通过 `activeStateAt(start)`
重新查询当前优化状态，而不是使用开始时缓存的 snapshot。
若 start 已边缘化，明确报错 `WINDOW_SCAN_START_NOT_IN_ACTIVE_WINDOW`。
同 timestamp priority 是显式函数，不依赖 enum 编号：start → end/V2 scan →
visual current → visual reference；不同 timestamp 完全按 sensor time 排序。
IMU 只读取必要左右 bracket，不预取整个未来 scan。

新模式 **FULL_FIXED_LAG_V3_EXPERIMENTAL** 的 NDT source 必须走
RawTimedScanProvider → Window deskew → preprocessSource → 原 PCL NDT。
旧 V2 retained 为 **COMPATIBILITY_ONLY_NOT_FORMAL_INPUT**，两个路径复用同一
producer 的测量路由核心，没有复制完整定位算法。

## 4. Provenance 的拒绝合同

LiDAR enum 包含 raw timed / sensor-local rotation-only / window-owned SE3 /
legacy-state-derived SE3。V3 raw provider 只接受 RAW_TIMED_SENSOR；其它输入
直接拒绝，旧 XYZ provider 不被调用。形式上仍依赖 exporter 如实填写 lineage，
不能仅从 XYZ 字节密码学地识别故意谎报的 provenance。

FrozenVisualEvent 加入 raw sensor-local / window-owned / legacy / unknown
provenance。V3 adapter 关闭 compatibility；legacy 和 unknown 均在图修改之前
拒绝，不提交 visual factor。V2 兼容测试显式输出
VISUAL_PROVENANCE_COMPATIBILITY_ONLY；hash 正确不会自动改变语义。

格式及 reader 已实现：40-byte little-endian record，float64 XYZ/intensity +
uint64 absolute sensor point stamp；CSV 提供 tx/start/end/offset/count/provenance。
visual sidecar 绑定 ref/cur/provenance；缺省为 UNKNOWN。读入有字节范围、finite、
时间范围和原子输出检查。最终 CLI 报告 raw binary / catalog SHA256。
未来真实 exporter 仍必须补齐原始 bag、topic、time field、转换及标定 manifest。

## 5. 解析 Jacobian 与数学复核

IMU runtime 已切换解析 Jacobian，覆盖 r_R/r_p/r_v/r_bg/r_ba 和两个端点，
严格使用 right SO3 perturbation。旋转 bias correction 保留
`beta=J_R_bg delta_bg` 的非线性 `J_r(beta)`，没有近似丢掉它。
完整公式见 ANALYTIC_JACOBIAN_AUDIT.md。

LiDAR residual 为 `[p_measured-p, Log(R^T R_measured)]`；状态 Jacobian 使用
位置 -I 和旋转 -J_l(phi)^(-1)，再由当次冻结 basis Q 左投影。保持原 outer
basis relinearization 合同。visual 仍用已有解析 Jacobian。

旧中心 FD 函数保留为 TEST_ONLY_REFERENCE，不进入 runtime factor assembly；
固定 marginal prior 的 rotation chart 仍保留原有每节点三列 FD。
精确 principal Log branch cut 不存在唯一导数，因此明确报错；没有新增宽泛的
near-pi 拒绝带，也没有修改 objective 定义或科研接受阈值。

每列 FD 对照最大误差：IMU **3.95194e-9**，LiDAR **2.19452e-9**；测试门限
均为预先设置的 2e-7，包含非零 bias/pose/velocity、投影 basis 和近 pi 样本。
独立只读数学审查未发现符号、坐标或 bias correction 错误。

## 6. Block/sparse solver 与信息守恒

删除 IMU 循环 `H += Zero(D,D)`。完整 H eigensolve 默认关闭，仅显式 debug
诊断可选；rank 不影响优化决策。无步合法收敛会重置 solver 诊断标签。

新增 WindowLinearSystem，以 upper 15x15 state blocks 存储 H、gradient、cost。
IMU/visual 二元、LiDAR 一元；dense marginal prior 可以贡献 dense blocks。
独立 dense assembler 保留，不是只将 block 自己转 dense 来证明自己。
主求解为 Eigen SimplicialLDLT；LM diagonal、step clipping、成本下降判断、
rollback 和 optimized revision 合同不变。sparse 失败必须显式记录
SPARSE_SOLVER_FALLBACK_DENSE。

实测最大误差：H relative **7.52677e-19**，g relative **1.01248e-16**，
cost absolute **1.73472e-18**；同一 H/g 下 dense/sparse candidate step relative
**6.06171e-15**，最终优化状态 local-coordinate difference **3.86139e-16**。
分别低于 1e-10/1e-10/1e-12、1e-9 约束，没有扩大门限。

原 A1-R1 Schur 子图保持 existing prior + touching-oldest；retained-only
因子继续 active，不重复进入 prior。原信息守恒、噪声冲突、rotation chart、
jitter 和 10,000 次 ID 生命周期测试继续通过。新 block/dense 测试也覆盖连续
优化/边缘化，比较 H/g/cost 而不是只看最终位置。
协方差继续 H X=E，不形成完整 H^-1；P15→map-left P6 未改。

## 7. 测试和复杂度 sanity

Release build PASS，**CTest 21/21**；Debug targeted build PASS，**5/5**；
git diff --check PASS。原九节点联合测试保留原断言，只显式打开 debug rank。
旧 ScanEndProcessor 和 frontend runtime end-to-end regression 也通过。

六种 motion/lever arm/bias deskew synthetic 最大点误差 **3.30093e-15 m**，
包含 IMU knots 与半 knots。测试使用原积分离散模型，不宣称任意连续运动积分精确。
in-scan visual 联合优化确实改变 scan-start，cached snapshot 的 deskew 会被
测试检测；start 被边缘化时明确拒绝。

真实 PCL 的 V3 synthetic fixture 对四种固定 R2 policy 分别验证：10 events、
3 scans/deskews、3 LiDAR commits、1 probe pair、5 NDT calls；
post-handoff IKFoM calls=0。此 full-rank 地图 fixture 视觉提交为 0，这是正确的
条件路由，并非宣称视觉融合成功；另有退化方向 adapter 测试实际提交 visual factor。
legacy LiDAR 拒绝时 NDT calls=0；legacy visual 不入图；binary reader 的真实
非均匀时间字段保留、缺时间、越界和截断测试均通过。

8/16/24/32/48 节点 scaling 已记录。48 节点单次 Release run：
dense/block assembly **91.274/1.51257 ms**；dense/sparse solve
**17.7264/2.20351 ms**；latest marginal covariance **113.051 ms**。
两种优化器各 3 iterations，最终状态相符。
该数据仅为 synthetic complexity sanity，不是正式 CPU benchmark、WCET 或论文
低算力性能结论。小窗口 sparse 可能较慢。dense marginal covariance 和 dense
prior 仍是剩余成本。
内存列只包含线性系统 payload 及保守 component accounting，不是实测 malloc
peak 或进程 RSS；离线 input/catalog/events 仍随序列长度增长。

## 8. 结论边界

两个本轮结构性目标在授权工程范围内已关闭：新 V3 的测量生成由 Window
负责，主优化有 block/sparse 及已验证解析 IMU/LiDAR Jacobian。

但“下一步只剩短真实链路验证”不成立：现有 Floor01/Corridor01 prepared XYZ
还不能产生 Window-owned timed deskew；Floor01 旧视觉深度仍不具备 formal
独立性。还需真正 raw timed export、可信 provenance manifest 和合格视觉深度
产品，然后才有可验证的短真实链路。执行 AI 没有自行重导全 bag、重跑 frontend
或完整轨迹，也没有自行开启下一阶段。

请以源码和上述证据进行数学验收，重点核对积分同源性、SO3/bias Jacobian、
prior chart、selected basis、Schur retained-only 合同、provenance 默认拒绝与
V3 真实时序。不要将 synthetic PASS 扩展成公开数据集科学效果或 formal readiness。

**READY_FOR_FORMAL_EXPERIMENT = NO。执行完成后 STOP。**
