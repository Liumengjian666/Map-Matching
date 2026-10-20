# P62-R8：结构准入与唯一窗口Shadow

正式结果：**REAL_SHADOW_NO_NATURAL_TRIGGER**。

独立判别字段：**HOLDOUT_DISCRIMINATION_NOT_EVALUABLE**，不是已证明留出证据无效，也不是恢复成功。20帧Smoke及100帧工程链路通过；自然触发0，4帧强制诊断的8个候选均在冻结物理去重范围内回到名义结果。

## 1. 冻结身份与执行范围

- 仓库：https://github.com/Liumengjian666/Map-Matching。
- 工作区：`/home/jian/livox_ws/p62_r8_structural_admission_ws`；分支`research/p62-r8-structural-admission`。
- START_SHA：b6c3bebb686697161cd645426d5d54ffe2927728；启动时R7本地/远端一致，干净。
- 全段预检实际CODE_SHA：15e2bf1d8de8dd6877cf45c141eaa6f773ebdb90。
- 选窗/首次Shadow前冻结CODE_SHA：463816a63038948cac29ae45925cfa4030a88da4。
- 最终本地归档提交由`GIT_HANDOFF.json`及Git HEAD给出；运行后报告/分析代码不冒充参与运行的算法。
- 原生二进制`/home/jian/livox_ws/p62_r8_structural_admission_build/shadow`，301616字节；SHA256=bd94da1878e696f5511ed862b55bd40a12a59ce0d978fab8e8a8726eab49189f。
- 实际编译未改的R7 shadow.cpp/recovery.hpp，调用未改R6 native_kernel.hpp/contract.hpp及系统PCL1.10；真实Gaussian Grid每次align检查0.8m。方向矩阵G是归一化PSD几何近似，**不是原生PCL Hessian**。
- 不push，不修改生产或历史归档。完整历史依赖相对START_SHA diff为空，输入、源码、二进制、PCL、R6输出与稳定工作区状态前后通过守卫。

原始bag：`/home/jian/rosbag/loop2/raw_restore_20260914_stage0/loop2_raw.bag`，19677401800字节，SHA256=d17bee3f8697dcb01cb4b9f6ea2483aa42a1a31ae277659c8a7869d9c69f13ae。

地图：`/home/jian/rosbag/loop2/loop2mapping/pcd/loop2_simtime_rebuild_2026_08_12_001_all_downsampled_points.pcd`，7346648字节，SHA256=016b7ae3ae494ad5d462a66d888a96b96dda5fe29d6058059aa2fa6bdf6b09ca。459154点，沿用两次0.15m体素后仍459154点；同记录FAST-LIVO2地图，不是独立跨会话真值。

共同历史因果初值：`/home/jian/rosbag/loop2/final_delivery_run_A_20260922/ndt_determinism.csv`，20638557字节，SHA256=e5f749cbbc7fc514326cc61626945f4ac6347722e4ad769e676984e1d214102a。仅投影身份/Source/initial_guess字段；未使用历史Raw/Used质量选窗。100帧完整精度initial逐项重新核实，当前Shadow输出不改变后续初值。

## 2. 全段结构资格

详见STRUCTURAL_ADMISSION_REPORT.md及两份完整逐帧CSV。10905帧原始身份有效；10900历史Source点数/FNV全部匹配。3338帧（30.609812%）满足训练与全部8块门槛；7567帧不可评价。原10800运动合法窗口仅402（3.722222%）留下，筛除10398；前1000名滚动高运动窗口全部被筛除。单帧资格随gyro强度非单调，不能推断运动造成稀疏。

唯一窗口原始4982–5081、seq9717–9816、原运动排名1039，P90=0.5323903214110374rad/s。完整点时间1786179977510337353–1786179987510793986ns。预注册SHA256=a5086e9eb4813eb46289638495bb8c0d92b759159686178c17abf35c14f921c1。规则与源码先冻结，不跳到第二名，不使用GT或候选结果排名。

## 3. 时间与算法合同

真实输入/livox/lidar CustomMsg及/livox/imu，frame_id=livox_frame；点时间header整数ns+原uint32 offset_ns。继承R6/R7 **raw、Deskew=false、midpoint参考**，并非本轮完成连续Deskew/EKF。历史double时间身份容差512ns；不修改点时间、不用设备timebase替代ROS epoch。

NDT .8m/.08/.001/40/.55，Source .25m、.5–80m、cap1400，偶数16扇区训练、奇数8块验证。训练≥300、每块≥10。β=0，无IMU候选运动评分。触发、G、种子、去重、.8m/5°输出准入、overlap及成对共同权重规则均保持冻结版本。

FULL使用完整Source，SPLIT使用Training Source；PROPOSED复用SPLIT。FULL的验证块也参与了其优化，因此FULL块损失不具有SPLIT的留出语义，不能作为公平泛化评分直接比较。验证块不是IID独立真值。

## 4. 实际运行与预算

同一进程先20帧Smoke通过，再继续80帧。100帧全部可评价、FULL/SPLIT各100次真实align；自然事件0/100，正式额外align0，普通帧额外0。SPLIT最高迭代7，验证块损失中位数的全窗最大0.059960546445029766，未达到0.25，未修改阈值。

100帧结束后，仅固定局部20/40/60/80（原始5001/5021/5041/5061）执行DIAGNOSTIC_FORCED。每帧G有2/2/3/2个合法弱方向，正负种子各一次NDT，总8次额外align，未重跑名义值，缓存位姿逐项一致。共享20预算仅用8。

| 局部帧 | λmin/λmax（约） | 额外align | 不同候选 | 结果 |
|---|---:|---:|---:|---|
| 20 | 0.005950 | 2 | 0 | COLLAPSE_TO_NOMINAL |
| 40 | 0.006969 | 2 | 0 | COLLAPSE_TO_NOMINAL |
| 60 | 0.006182 | 2 | 0 | COLLAPSE_TO_NOMINAL |
| 80 | 0.008080 | 2 | 0 | COLLAPSE_TO_NOMINAL |

总208次align全部原生报告converged；不能当作正确定位。8个恢复结果均duplicate_of=0。相对T0最大分离0.08283019585736584m、0.5547650915876962°，小于0.10m/1°物理去重门。最大种子平移0.30000000000000365m（浮点舍入）、旋转2.5764846838965876°，满足冻结限制。未观察到物理不同候选；不声称存在竞争盆地或恢复错误分支。

## 5. 三选择器及弃权

104个正式/诊断决策点的去重池均仅T0。Native和held-out mean各104次KEEP_NOMINAL；robust paired各104次ABSTAIN。三者pool_ids完全一致。所有无对手margin=NA，PAIRWISE_ADVANTAGES.csv合法表头无数据——没有可计算的真实成对优势。

这只是单候选下的既定弃权语义，不是非平凡判别差异。未观察到真实不同候选，因此结构留出判别能力为NOT_EVALUABLE，而非已成功或已证明无效。正式自然事件与强制诊断严格分开，诊断不能计入在线触发率/恢复率。

## 6. 资源（ms，P95采用线性插值）

| 项目 | Mean | P95 | Max |
|---|---:|---:|---:|
| Source预处理（预先缓存，单独计） | 2.282878 | 2.360869 | 6.784200 |
| CONTROL_FULL align | 8.388947 | 11.802400 | 69.077009 |
| SPLIT名义align | 3.943559 | 5.140667 | 12.733445 |
| 两臂验证合计 | 1.389195 | 1.471851 | 2.647834 |
| 选择器及记录 | 0.100207 | 0.112442 | 0.300458 |
| 双臂正式逐帧计算（不含预缓存Source） | 15.853122 | 20.331881 | 77.542434 |
| 双臂计算+Source预处理之和 | 18.136001 | 23.368879 | 80.081943 |
| 额外恢复align（8次） | 9.065278 | 9.607137 | 9.642117 |
| 强制诊断整帧（4帧） | 22.116571 | 23.117635 | 23.233984 |

原生进程墙钟2.121103249s；地图加载/Target/Grid/KD构建141.397529ms单列。峰值ru_maxrss=108884KiB（106.332031MiB），是本进程峰值，不是各进程RSS相加。完整输入前后哈希、200夹具与全段预检不包含于该2.12s；预检流处理墙钟36.744847s也不含其哈希门。

FULL计算显式计费，不能隐藏。逐帧计时含日志/记录开销，离线预缓存运行不是机器人持续实时性认证，不宣称10Hz或100MB目标通过。

## 7. 门、限制及唯一NEXT

输入/时间身份、真实原生NDT、同池、缓存、单窗口、预算及前后资产门全部通过。GT reads=0，ROS定位节点=0，rosbag play=0，EKF=0，生产改动=0，反馈=0，外部CLI=0，额外窗口=0。没有ATE/RPE或独立精度评价。

结论：固定16扇区在loop2上有明显适用性不足，应仅保留为受限工程对照，不按原样作为通用主方法继续选窗搜索。预算化恢复工程机制可运行，但候选分离、留出判别、独立精度改善及创新性均未建立。

**唯一NEXT：请科研总控审批有限视场/稀疏验证块下结构留出数学修订合同；先解决可用性和选择偏差，再另行批准实现。** 本轮不改分区、不放宽阈值、不运行第二窗，完成后停止。

## 源码与表格点位

- 预检准入：tools/preflight.cpp与tools/preprocess.hpp；原始消息桥接和资产门：run_preflight.py。
- 选窗：select_window.py，保留原R7排名顺序，只加结构门；窗口摘要和全部100帧身份：WINDOW_PRE_REGISTRATION.json。
- 原生参数及实际网格检查：../p62_r6_dog_shadow_prototype/tools/native_kernel.hpp:43–44；G:45起；成对最坏权重：contract.hpp:38起；种子:72起。
- 实际运行驱动：../p62_r7_motion_selected_shadow/tools/shadow.cpp:108起的100帧循环，119起的条件性缓存诊断；恢复与预算：recovery.hpp:15起；单候选NA：44起。
- 原始结果：NATURAL_SHADOW_RESULTS.csv、DIAGNOSTIC_PROBE_RESULTS.csv、NATIVE_ALIGN_CALLS.csv、CANDIDATE_POOL.csv、HELDOUT_BLOCK_LOSSES.csv、SELECTOR_COMPARISON.csv。完整seed/final矩阵均为T_M_L：将原始scanner-frame点转换至冻结地图系，列名seed_ij/final_ij是4×4矩阵第i行j列；空间差单位m/deg，时间ns或明确标注ms。
- 运行输入时间支持、原始SHA/FNV、历史initial来源：REAL_INPUT_MANIFEST.json。native/preflight/Shadow各阶段冻结与结束回执独立保留，未将分析后代码当作运行代码。
- 离线复算：analyze_results.py；独立结果/数学反例测试：test_result_audit.py；最终资产门/分析版本/校验和：seal_archive.py、analysis_manifest.json、asset_identity_after.json。
