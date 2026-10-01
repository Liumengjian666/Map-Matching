【执行 AI → 科研总控 / 决策 AI】

PAPER-P6-ALG-INTEGRATION-A3E-R2 执行回报

请以真实 Git 对象、源码与本目录 CSV 为依据复核。本报告只交付已授权的
production 数值架构修复及唯一 P3-200 工程验证，不将通过解释成定位精度
或正式实验准入，也不自行安排下一阶段。

## Git 身份

Repository=https://github.com/Liumengjian666/Map-Matching.git

Branch=research/p6-i6d-full-algorithm

START_SHA=e74735956ea82d6fef9c9eb9616c33cbf4443dc1

CODE_SHA=0ec016677bec6bf4301447ebcd01da353e74ad21

END_SHA=包含本报告的最终 analysis 提交；交付消息提供完整 SHA 和 push 后的远端核验结果。

worktree=/home/jian/livox_ws/dog_loc_p6_i6b_ws

本轮复用现有工作区，没有新建 worktree，没有重写历史或 force push。
冻结结果未改；第二个提交仅新增分析、证据与结果报告，不再修改估计器。

## Backend 与 prior 权威性

production marginalization backend=SQUARE_ROOT_QR

square-root prior authoritative=YES

legacy Schur fallback=NONE

initial prior square-root factorization=PASS_LLT_NO_REGULARIZATION

prior representation=(A_prior,b_prior,reference_states,valid)

prior quadratic=||A_prior d+b_prior||²

H/g=由 A/b 派生的 optimizer/covariance/diagnostics compatibility cache

下一次边缘化读取 A/b，绝不把 cache H/g 当作 authoritative 输入。
初始 H/g cache 保留现有数值，LLT 重建精度为 machine-level，用于保持首次
边缘化之前的初始化/优化路径。后续 cache 直接由新 prior A/b 派生。

whitening convention=R=L Lᵀ；Jw=L⁻¹J，rw=L⁻¹r，triangular solve，无显式逆。

oldest elimination=column-pivoted Householder QR，对 Am 及 [Ar,b] 使用相同 Q。

rank rule=eps_machine*max(rows,cols)*maximum QR pivot，必须 rank(Am)=15。

rank failure=square_root_marginalized_block_rank_deficient，FAIL CLOSED。

row compression=对 retained rows 再做 column-pivoted Householder QR；不经 normal equations。

原始 prior + touching-oldest factors 消耗；retained-only 因子继续 active，
可重线性化，没有重复计入 prior 和 active factor。

LiDAR 当前 selected/reliable subspace 与 Bᵀr、BᵀRB 保持原合同；rank5 没有补成 rank6。
Visual 架构保持 cross-state selected factor 白化，但真实运行 visual=NONE。
没有 initial epsilon-I、prior eigen clamp、伪逆、旧 Schur fallback 或噪声改动。

prior 零参考常数约定保留：row cost 与旧 quadratic 差 bᵀb；compression 可
丢 constant-only row。这些不是累计绝对 likelihood，不影响当前 iteration
的严格 candidate 比较和 H/g。非线性等价仅对应约定的 reference/chart。

## 数学回归与构建

Release build=PASS

full Release CTest=35/35 PASS，原测试未删、未降容差。

Debug targeted=13/13 PASS。

A1-R1 information conservation=PASS。

clean H/g errors=1.24607e-10 /7.85293e-13。

noisy H/g errors=1.24607e-10 /7.85317e-13。

新增 joint IMU+rank5 LiDAR+visual 图：相对 H/g error=3.24986e-13 /4.82723e-14。

nonzero prior reference/current rotation、gradient、chart C=PASS。

correlated covariance whitening 和 initial H/g orientation=PASS。

stacked objective 最小化 oldest 后的等价及常数偏移=PASS。

rank5 LiDAR weak direction 没有注入信息=PASS。

1005-cycle bounded-row lifecycle=PASS，max rows15，active IDs3，历史 ID 不重入。

rank-deficient window=FAIL CLOSED，并验证 states/prior/factors/IDs 未改变。

multi-removal 正常成功路径=PASS；later-attempt failure batch rollback 本轮未新增。

diagnostics ON/OFF state/prior/lifecycle parity=PASS，包括共享计数器 basis callback。

审查发现 std::function 复制不能隔离共享捕获状态；已在唯一 replay 前关闭：
shadow 使用生产已冻结的 incident projection，不再调用原外部 callback。

A3B-R2 frozen LiDAR=PASS。

A3C-R2 alias-safe=PASS。

A3D-R1 small-step=PASS，clipped-zero / nonfinite negative controls 保留。

A3E-R1 high-precision oracle self-test=PASS。

git diff --check=PASS。

threshold/damping/NIS/noise/window/NDT/rank/basis changes=NONE。

冻存 TX155 capsule SHA=
b44fe28654437ff1cd944945cb436f62368c7ff315e9e920686836950aed58a8。

legacy frozen Schur min=-97.63543657869423，证据未改写。该 capsule 没有 raw
J/R，因此没有伪造 QR graph；新 QR 证据来自本轮真实 raw-factor replay。

## 唯一 P3-200 输入与 identity

Corridor01 / FULL_FIXED_LAG_V3_EXPERIMENTAL / ADAPTIVE_SELECTED_NIS。

frame_limit=200，visual=NONE，visual provenance=NONE。

initialization stamp=1517157224188979000，官方外参，原 map、IMU、raw schedule。

REAL_REPLAY_COUNT=1，RETRY_COUNT=0，GT_USED=false。

七项 input hash 与上一轮一致；production manifest gate 实际 PASS，未删除校验。
详细 SHA 见 RUN_P3_200/input_identity.json。

首次边缘化 tx71/stamp1517157226248733355/enforcement40。
此前39条 completed events 的 frozen pose/cost/结构身份检查 PASS。
首次观测到 backend 数值分歧：tx72/stamp1517157226349593474，发生在 QR prior
生效之后。未出现不可解释的 pre-backend 漂移，不要求后续数值与 legacy bit-identical。

## tx90 / tx115 / tx155

tx90：ACCEPTED_UPDATE；两次 QR removal SUCCESS；最终39 nodes /1.916218042 s。
没有 alias symmetry failure。

tx115：ACCEPTED_UPDATE；两次 removal SUCCESS；最终39 nodes /1.916217089 s。
新 prior 导致当前测量/优化路径不同，未强制套用旧 tx115 数字。

tx155：transaction155/stamp1517157234720485283/enforcement208。

optimizer=CONVERGED_WITHOUT_STEP。

natural raw step=1.8246071048892027e-11，未 clipped，strict candidate rejected。

termination=NATURAL_SMALL_STEP_NO_ACCEPTED_UPDATE，candidate rollback difference=0。

pre-removal window=41 nodes /2.017077923 s。

attempt1 stack=30×615；Am rank15；threshold1.023059255910697e-5。

R diag min/max=2749.4364922149352 /1535816427.8991172。

retained rows before/after compression=15/15。

QR production result=SUCCESS；derived H asymmetry=0；lambda_min=0；
lambda_max=348827.40920818585。0 对应其他 retained 节点的零列/块，不能把它
当作 LiDAR rank6 或整张图不可观的单独证明。

同一 attempt 的 legacy shadow=FAIL:marginalized_prior_not_finite_psd，
lambda_min=-24.231923242581125。shadow 不影响 production。

attempt2：30×600，rank15，rows15/15，SUCCESS。

最终 window=39 nodes /1.916217804 s。

这是同一当前事件的 shadow-vs-QR 证据；旧轨迹 -97.6 不应强制逐值复现。
当前 tx155 stack 是 prior+IMU，当前 LiDAR 因 P 不可用没有提交；不能将该
对照误写成和旧 capsule 完全相同的 prior/factor graph。

## P3-200 完整结果

exit code=0；200 raw scans 中51 pre-handoff skip，149实际 Window deskew/terminals。

completed events=298；ACCEPTED_UPDATE267，CONVERGED_WITHOUT_STEP31。

QR marginalization attempts=260，全部 SUCCESS，Am rank 全部15。

LiDAR committed91 /not committed58，NDT converged149，U_obs valid149。

U_nonlocal probe104；NDT calls357（149 nominal+208 probe）。

post-handoff IKFoM calls=0；visual events/factors=0；candidate basis callbacks=0。

sparse solver dense fallback=0；non-LiDAR marginal-covariance time=0。

completed states finite/SO3=PASS；completed max nodes40，max span1.958914906 s。

first new failure=NONE。只跑到200，没有400/2777/Floor01/visual/GT/ATE/RPE。

## 不应隐藏的剩余证据

pre-measurement P available=129/149。

P UNAVAILABLE=20 terminals，tx136–155。

这些事件原 fail-closed 行为保持：不执行 probe，不让 selected NIS 因缺 P
通过，不提交该 LiDAR factor；没有 IKFoM-P fallback，也没有虚假信息补偿。

本轮修复 marginalization authoritative prior chain，并未把 optimizer 或
covariance hotpath 全部改成 square-root。这一限制应由科研总控基于证据处理。
工程 PASS 不代表可直接生成论文精度表。

## 资源

max authoritative prior rows=15，max extended columns=615，max A/b bytes=73920。

derived H/g cache 和临时 stack 另占内存，未计入上述 A/b byte number。

QR-path time（assembly/whitening/QR/compression/cache，排除 spectral/shadow）：
mean4.9049784115 ms，max6.049293 ms。

wall168.08 s，user155.78 s，system11.74 s，maxRSS112772 KiB。

diagnostics=ON；这些只是 engineering observation，不是正式 CPU benchmark。

## 交付与状态

summary.md /SQUARE_ROOT_PRIOR_CONTRACT.md /BUILD_AND_CTEST_RESULTS.txt。

RUN_P3_200 含小型 trajectory、全260次 QR history、tx90/115/155重点 optimizer/
preopt/marginalization、input SHA、command、resource、console 和 JSON summary。
完整外部日志路径及 SHA/大小见 EXTERNAL_RUN_FILES.json；未上传大点云或 bag。

A3E_R2_SQUARE_ROOT_MARGINALIZATION_CODE_CONTRACT=PASS

A3E_R2_TX155_NUMERICAL_REPAIR_GATE=PASS

A3E_R2_P3_200_REAL_LINK_GATE=PASS

READY_FOR_FORMAL_EXPERIMENT=NO

执行已停止。未自行进入任何下一阶段，最终数学验收及阶段裁定由科研总控负责。
