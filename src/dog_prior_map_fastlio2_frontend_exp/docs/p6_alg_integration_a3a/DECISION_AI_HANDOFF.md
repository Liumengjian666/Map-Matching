# 给科研总控 / 外部复核 AI 的 A3A 回传提示词

你是科研总控 / 外部审查 AI。以下是执行 AI 对 **PAPER-P6-ALG-INTEGRATION-A3A：REAL INPUT ELIGIBILITY CLOSURE AND SPARSE MARGINAL COVARIANCE HOTPATH** 的实际交付。请依据 Git 源码与报告复核数学和数据来源，不把 synthetic PASS 或输入资格解释为真实定位效果已经通过。

## 1. 固定身份、Git 和执行边界

Repository: https://github.com/Liumengjian666/Map-Matching.git

Branch: `research/p6-i6d-full-algorithm`

Workspace: `/home/jian/livox_ws/dog_loc_p6_i6b_ws`

START_SHA: `bf4e103e28588c103991cb3521d06cb1a0e120ee`

CODE_SHA: `9d8a794dac7f06b0e52dc2f8c4aadcbfc5138a0c`

代码提交顺序：

- `c87edf320713f2eaa5fd9a62ca83e26539fa83db`：sparse pre-measurement marginal；
- `09a812e01eeea5b579006569a9d6f270764e4779`：raw exporter、decoder 身份和 V3 输入 gate；
- `9d8a794dac7f06b0e52dc2f8c4aadcbfc5138a0c`：raw schedule 初始化期间 scan 的显式跳过与 synthetic 回归。

最终交付 END_SHA 和是否已 push，应使用执行 AI 最终消息中的完整 SHA，并核验上传分支 HEAD。本文不把自身最终 commit SHA 递归写入文件，也不在尚未上传时宣称 GitHub 已可读取。源代码可固定到 CODE_SHA；之后仅有报告/小 CSV/manifest 提交。

[本轮报告目录](https://github.com/Liumengjian666/Map-Matching/tree/research/p6-i6d-full-algorithm/src/dog_prior_map_fastlio2_frontend_exp/docs/p6_alg_integration_a3a)

本轮没有完整 Corridor01/Floor01 回放，没有真实数据 NDT align，没有 GT ATE/RPE 或调参，没有新的融合策略，没有改 frozen baseline，也没有将实验路径接入正式 FULL 默认行为。

## 2. Sparse marginal 数学和生产路径

原热路径为 dense joint H → dense balancing → dense LLT → 15 RHS。现在 `latestMarginalCovariance()` 使用同一 `blockLinearizedSystem()` 得到未阻尼真实联合信息 H，保持 SparseMatrix 进行坐标平衡。

令 D=15N，E 为最新状态的 15 列 selector，S=diag(1/sqrt(H_ii))：

`B=S H S; B Y=S E; X=S Y; P15=sym(X.bottomRows(15))`。

没有形成完整 H⁻¹，没有生产 dense fallback，没有 optimizer damping、epsilon I、jitter 或旧 IKFoM P 回退。SimplicialLLT 使用 NaturalOrdering，以保留与旧 dense Cholesky 相同的 balanced-pivot 可靠性门槛 `min(L_ii²)>1e-12`。固定 15D 和 6D PSD 检查仍存在，但没有完整 D×D eigensolve。

normalized backward error 为 `||H X-E||F/(||H||F||X||F+||E||F)`，必须 ≤1e-10。不可靠/不可观 H 明确返回 `WINDOW_MARGINAL_COVARIANCE_UNAVAILABLE`；不能以缺 P 为由让 P2/P3 selected NIS 通过。U_nonlocal 在 P 不可用时不 probe。

P15 的误差坐标是 right-body rotation、additive map position/velocity 和两类 bias。P_map6 使用原合同 J6：rotation block=R，position block=I，保留交叉项。这是 map-product pose covariance，不冒充完整 left-SE(3) 平移 retraction 或新的外参后验。

Dense reference 独立保留为 `latestMarginalCovarianceDenseReferenceForTest()`，只供测试/debug。producer 检查其 runtime request counter=0。Schur marginalization 没有重写；仍按 A1-R1 只把 existing prior+touching-oldest 因子边缘化，retained-only 原始因子保留，不重复计入。

## 3. 协方差时序和实测证据

LiDAR 顺序严格是：prepareStateAt(scan_end) → 预测状态+IMU factor → P15 → NDT/U_obs/U_nonlocal → selected NIS → 决定 current LiDAR factor → optimize。当前 LiDAR 测量没有提前进入 H。

scan-start / visual-reference / visual-current 不请求 P。对应诊断是 NOT_REQUESTED_NON_LIDAR_EVENT、sigma=NaN、marginal time=0。此 producer 每个 policy 都启用 U_nonlocal，因此 LiDAR terminal 均有潜在 P 用途。V2 fixture 3/7 events 请求，V3 fixture 3/10 events 请求，四个 P0/P1/P2/P3 policy 均覆盖。

Sparse vs independent dense oracle 覆盖初始非零 prior gradient、IMU-only、方向性 LiDAR、IMU+LiDAR+cross-state visual、执行 Schur 后和多次 Schur。P15/P_map6 门槛 1e-9；observed fixture 最大 P15 8.88821e-15、P_map6 3.3327e-15、backward error 4.55665e-17。Scaling 最大 P15 relative error 6.84464e-14。

48-node 本轮 same-graph dense oracle 117.091 ms，sparse marginal 4.16642 ms；A2D 历史 dense 约 113 ms。8/16/24/32/48 的完整输出见报告。这只是普通开发机负载下的 synthetic complexity sanity，不是正式 CPU benchmark 或 C++ WCET，不宣称真实轨迹速度已验证。

## 4. Corridor01 原始 LiDAR：实际导出成功

原始 bag：`/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/raw/Long_Corridor_Rosbag/raw_data_core_2023-07-25-03-01-44.bag`

SHA256：`c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811`

真实 topic `/velodyne_packets`；类型 velodyne_msgs/VelodyneScan；frame `cmu_rc2_velodyne`。ROS reflection 确认 Header+VelodynePacket[]；packet 是 ROS time stamp + uint8[1206]。2777 scans，VLP16/strongest single return；导出逐 scan 检查模型/返回模式、block、header==first packet 和 packet monotonicity。

时间语义不是根据字段名猜测。官方 driver 固定提交 `29abd0e1361cb7f5eda451d2b51c35eeca45e0d5` 的 `buildTimings()`/`unpack_vlp16()` 给出 firing offset：`(block*2+firing)*55.296 us + laser*2.304 us`。调用 unpack 时以当前 packet.stamp 为小时间原点，避免大 epoch 进入 float。

`point_ns=recorded packet.stamp.toNSec()+llround(driver_firing_offset_seconds*1e9)`。

Scan start=min(valid decoded point stamps)；scan end=last packet stamp+1306368 ns（最后一个 scheduled firing，即使该 return 无有效 range）。未使用 point index 均匀赋时、header 复制赋时、bag record time、legacy deskew 或任何 estimator pose。

重要限制：源 driver 支持 host receive/GPS 时间，但本 bag 没记录采集模式。我们证明的是记录的 packet stamp+固定 firing schedule，不是独立硬件时钟来源或 LiDAR/IMU 同步精度。manifest 强制 `hardware_clock_or_sync_accuracy_proven=false`。

新持久化目录：`.../Corridor01/results/p6_a3a_v3_input/`。导出和独立 validate-only 均 PASS：2777 scans、79,932,911 points、3,197,316,440 bytes。

每点 40-byte little-endian：float64 x/y/z/intensity + uint64 sensor ns。官方解码浮点坐标提升为 double，0.1–200m 固定 range selection；没有 TF、map/NDT/EKF/Window transform 或 SE3 deskew。

原始点 binary SHA：`4ba09d8ae7004056dcc4e9d63d2ab09915748c7e68bc29d0dc8bd44a24fc95ff`。

catalog SHA：`fdaf9607bc1933269f5f999ee044d37aa1eefabedc054f522da5078ce708123f`。

filter schedule SHA：`41d0b2040a5de8a8bd428c382a7b6dc18fabcaa8d3e7d2cc331018e0585edf1d`。

Manifest SHA：`fb20125c63aa4c7110851b8bb08ec33e8938d94be18e28e9737d207001377575`；Git 中 manifest 副本与持久化原件字节一致。大 binary/bag 不提交 Git。

真实 terminal 与旧 assets **0/2777 exact match**。新 schedule 来自 raw sensor sequence，不 snapping 以迁就旧 runtime。V3 用 raw catalog 替代旧 scans.csv；保留初始化期原始记录，但 skip 无 Window scan-start 状态的 pre-handoff scans，并报告数量，不改物理时间/交易 ID。这仅做了 synthetic 测试，未运行真实链路。

Decoder 从固定官方源码重新构建；已加载库本身导出源码身份，检查实际 RawData/Calibration symbol binding，而非只检查 /proc/maps。LD_* interposition 环境被移除；旧库 preload 或同名 sibling 替换都拒绝。结果输出采用 exclusive inode/fd 创建，不能覆盖输入别名。Manifest/source pins 是受控构建+完整性证据，不是对恶意伪造二进制的密码学 attestation。

## 5. Corridor 视觉：不做错误升级

旧 CSV `corridor01_metric_visual_causal_v2.csv` 的数值完全保留，482 valid pairs，SHA `9aea15ce31411c08b61d878dbc5ccd76df2c9ebed33d112c9119bd430e18bc67`。

Depth 来源是 v2 derived bag 的 `/superloc_adapter/points_rot_only`。shared adapter source SHA `58c407…` 与冻结 meta.yaml 匹配。坐标公式本身只是 raw IMU rotation+固定外参；但发布前等待历史 `/dog_livo/ndt_odom`，且 full-SE3 分支的有效性拒绝会同时阻止 rot 输出。

因此没有证明整个已保存 depth 产品独立于旧 estimator。结果标记 CORRIDOR_VISUAL_FORMAL_INPUT_BLOCKED；逐条 sidecar 全为 UNKNOWN，缺行也不自动升级 RAW；formal V3 不加入这些因子。既未重跑视觉，也未把 sidecar 作为科学来源“洗白”。MAP_POSE_USED=true 特指 generation control flow，不声称 map pose 数值进入 rot_point。

原 provenance 没记录 exact generation script hash；报告区分当前 audited script 和 first-artifact commit 中的 script SHA，不冒充执行历史证明。

## 6. Floor01 和验证状态

旧 Floor XYZ-only request 来自 legacy cloud_end_frame，标记 LEGACY_STATE_DERIVED_SE3_DESKEW / RAW_POINT_TIME_UNAVAILABLE（在旧 prepared bundle 中）。旧 visual depth 是 LEGACY_STATE_DERIVED_DEPTH。它们仍不可 formal。

只读原始首 fragment 发现 `/cmu_sp1/velodyne_packets`、`/cmu_sp1/imu/data`、`/cmu_sp1/camera_1/image_raw`，可能可从原始 packets 恢复逐点时间；本轮没导出 Floor，也没有据此升级旧产品。

Release build PASS；原 21 个 CTest 未删/未弱化，加两项共 23/23 PASS；Debug targeted 7/7 PASS；Python 数据合同 15/15 PASS；git diff --check PASS。Release 覆盖实际 PCL synthetic V2/V3 producer、reader 与 warmup schedule；Debug 定向覆盖 covariance/Schur/deskew/provenance 数学核心，不宣称 Debug 跑过真实数据。

A2C/A2D 合同保留：23D→15D fixed-gravity prior，P15→map P6，U_nonlocal no-EKF-P，P0/P1/P2/P3，selected pre-measurement NIS，stale A_exact closure，directional/cross-state visual，single Window owner，post-handoff IKFoM calls=0，Schur 信息守恒。旧 FULL 源码 byte parity SHA `498ec598db3aaec84b74391292d9db7928b2337944be01287621502135abed34`。

## 7. 本轮结论及不可扩大解释的边界

READY_FOR_SHORT_REAL_LINK_TEST=YES：只表示已核验 raw LiDAR 输入+manifest+算法 build/tests 满足授权门槛，**不是短真实回放已经 PASS**；当前视觉输入仍不合法。

READY_FOR_FORMAL_EXPERIMENT=NO。

GT_USED=false；真实数据 NDT align=0；完整定位实验=0；本轮没有 RMSE 改善结论。

内部反向审查已经修复数值排序门槛、实际动态库来源、覆盖保护、manifest 计数/时间语义和 scan-start minimum 的问题。你正在进行的跨模型外部复核此前尚未执行，不要根据执行报告误写成已外部验收。

执行 AI 在 A3A 结束后停止，未进入 A3B。本回传只交付结果、证据及局限，不替科研总控提出下一阶段启动决定。
