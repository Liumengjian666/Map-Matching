# 【执行 AI → 科研总控 / 决策 AI】

## PAPER-P6-ALG-INTEGRATION-A3D-R1 执行回报

本轮执行已经结束。以下为实际源码、测试和唯一真实 replay 的结果。
不申请自行启动下一阶段，不把工程结构验证包装成精度结论。

### 一、Git 与交付身份

Repository: https://github.com/Liumengjian666/Map-Matching.git
Branch: research/p6-i6d-full-algorithm
Worktree: /home/jian/livox_ws/dog_loc_p6_i6b_ws
START_SHA: 5e90b309c2612146a4ecbfdaff5becd3f6fcf7c8
CODE_SHA: 2ecbe50552fa463fb59cb2250720325c87b829b6
END_SHA: 包含本文件的 analysis 交付提交；完整字面 SHA 见最终 Codex 回报。
remote HEAD / push: 以最终执行回报的实际网络核验为准，不从本地 commit 推断。

开始时 HEAD 与指定 START_SHA 完全一致、branch 正确、worktree clean。
普通 commit，保留此前历史，未 squash、force push、reset 或 rebase。
代码提交信息：fix: classify natural small optimizer step as convergence。
结果提交信息：analysis: validate A3D-R1 optimizer termination repair。

### 二、production 修复边界

唯一估计行为改动是：严格拒绝 candidate 并恢复 iteration backup 后，
finite candidate cost + 0 < raw_step_norm < 1e-8 + 未被 clipping，
将已有 small-step 终止分类为自然收敛。
无更差 candidate 写入，无 objective tolerance / ULP acceptance。

gradient_convergence_tolerance changed=NO (1e-9)
step termination threshold changed=NO (1e-8)
candidate acceptance changed=NO (finite && candidate_cost < current_cost)
initial damping changed=NO (1e-6)
damping multiplier/schedule changed=NO
iterations/max step/backend changed=NO
Window/Schur/PSD/jitter/alias-safe evaluation changed=NO
NDT/U_obs/U_nonlocal/NIS/noise/reliable rank/visual policy changed=NO

Raw solver norm 与 policy clipping 独立于 diagnostics 保存。新增纯诊断
termination reason、small-step flag、rollback state difference。
Frozen LiDAR snapshot 同时用于 H/g 和 candidate cost，candidate basis 回调为0。
预测反馈沿已有成功收敛/revision 认证路径，没有虚构新 posterior 或测量。

### 三、先 RED 再修复及回归

Production 未改前，新 deterministic fixture：两个互相冲突的有限 LiDAR
观测使 cost=200，微小非零 prior gradient 产生 raw=3.3333302758042944e-9。
未 clipping、finite candidate cost=200、严格下降不成立。旧逻辑返回
FAILED_ALL_CANDIDATES，新测试正确失败。

修复后同 fixture：CONVERGED_WITHOUT_STEP；状态严格不变、feedback ready。
Trace OFF/ON 结果一致。

Negative controls：
1. maximum_step_norm=0：FAILED_ALL_CANDIDATES，旧 A3B-R1 测试保留并 PASS。
2. clip 到 1e-10：FAILED_ALL_CANDIDATES，拒绝状态回滚，OFF/ON 一致。
3. candidate NaN/+Inf/-Inf：精确 production 判据均 NOT_CONVERGED。
4. raw=0、raw=1e-8 边界、clipped=true：不能走自然小步分支。

覆盖局限如实记录：nonfinite 控制是实际 production predicate 层测试，
没有为了测试给 production 加 candidate-objective injection hook；
不是声称完整 optimizer 产生 NaN 后的端到端注入测试。

Release build=PASS；full CTest=32/32 PASS，原31项全部保留。
Debug targeted=10/10 PASS；guard standalone self-test=PASS；diff-check=PASS。
A1-R1 information conservation / A2D Schur / A3B-R1 / A3B-R2 / A3C-R1 /
A3C-R2 alias 与 tx90 capsule 全部继续 PASS，未放宽任何旧测试 tolerance。

A1-R1：clean H=1.24607e-10，clean g=7.85293e-13；
noisy H=1.24607e-10，noisy g=7.85317e-13；nonzero prior gradient error=0。
既有 jitter regression 保持原值1e-7，H/g delta=1.71782e-7/1.07995e-8。

### 四、唯一真实运行与身份

只运行 ONE Corridor01 P3-200：ADAPTIVE_SELECTED_NIS、visual NONE、
frame_limit200；没有先跑 P3-100、没有第二次200、没有其他真实 replay。
七项 raw/catalog/filter/manifest/map/official params/IMU SHA 与冻结值相同，
实际 V3 requireRawTimedInputManifest PASS。
初始化1517157224188979000；profile corridor01；library path 和 diagnostics
与旧 run 一致。完整路径/hash/argv 在 RUN_P3_200/ 中。

Live identity guard 的 tx115 preopt 和 first-candidate 两项都 PASS。
进一步 prefix 逐字段对比：64条 preopt、982条 optimizer，与 START run
完全一致（只排除 NDT runtime 和新增 diagnostics），差异条数0。
因此没有靠早期状态漂移绕过 tx115。
Guard 没有 retry，输出目录独立，不修改输入或算法参数。

### 五、tx115 完成修复

transaction=115
stamp_ns=1517157230686328484
preopt nodes=41
preopt span=2.0170772080000003 s
selected NIS=15.237396865140097
threshold=15.086
current LiDAR committed=0，仍按原逻辑拒绝。

optimizer current cost=81.345724589884568
candidate cost=81.345724589884583
gradient_inf=2.8457479913868156e-5
raw step=7.0568167407415232e-9
applied step=同 raw
clipped=NO
strict candidate accepted=NO
candidate committed=NO
rollback=YES
rollback max localDifference=0
candidate basis relinearization calls=0
termination=NATURAL_SMALL_STEP_NO_ACCEPTED_UPDATE
optimizer status=CONVERGED_WITHOUT_STEP
optimizer success/feedback path=TRUE
optimizer current/final cost 不变。

随后继续完整 duration enforcement：
attempt1 SUCCESS，41→40 nodes，span2.017055584 s，prior symmetry0，min eig0，jitter0。
attempt2 SUCCESS，40→39 nodes，span1.9162170890000001 s，prior symmetry0，min eig0，jitter0。
最终 Window 满足2 s；没有把 rejected candidate 接受后拼接轨迹。
tx90 原修复同样保持，两次 removal 都成功。

### 六、P3-200 新首次失败，立即 STOP

P3-200 未完成。首次新失败：
transaction=155
stamp_ns=1517157234720485283
event=LIDAR_SCAN_END
reason=producer_optimizer:marginalized_prior_not_finite_psd
实际阶段=MARGINALIZATION_STAGE，而非 optimizer candidate rejection。

tx155 optimizer 已 ACCEPTED_UPDATE，8 iterations，
cost2.5682974886605456→2.5682916669943747。
enforcement208，attempt1 FAIL，same-event earlier successful removal=0。
pre/failure window=41 nodes / 2.017077923 s。
first_bad_stage=M7_SYMMETRIZED_SCHUR（现有 diagnostics 标签）。
new prior candidate finite=YES；gradient finite=YES；symmetry max=0；
lambda_min=-97.635436582562278；lambda_max=349971.31159981759；
relative_negative=0.00027898125745291281；jitter=0。

本轮只保存这些直接证据，不裁定新 PSD 问题的根因。
没有把它误报为旧 tx90 aliasing 复发，也没有修改 solver、阈值、jitter。
失败 candidate prior 未存储，prior hash/state stamps/factor counts 未变，
当前 attempt 没有删除 oldest；没有自然触发 later-attempt partial commit。
没有重新 replay、没有修第二个问题、没有额外 root-cause oracle 审计。

### 七、完成前缀与限制

last completed terminal=154
completed LiDAR terminals=103
Window deskews=104（含失败tx155）
completed events=207
LiDAR committed=75
successful oldest removals=168（84个成功双-removal enforcement）
failed attempts=1
max completed nodes=40
max completed span=1.9589149060000002 s
failed tx155 Window 尚未完成2 s enforcement，不能用 prefix bound 冒充整体PASS。

post-handoff IKFoM calls=0
visual events/factors=0
sparse fallback=0
candidate basis callbacks=0
all terminal source provenance=WINDOW_OWNED_SE3_DESKEW
non-LiDAR P requests=0 / NOT_REQUESTED
observed marginalization jitter=0
successfully stored prior symmetry max=0 / min eig=0
失败 candidate 的 negative eigenvalue 另列，未标成 stored prior。

covariance available84 / unavailable19；首个 unavailable stamp=1517157232804267479。
没有借用 IKFoM P，缺P时 probe/NIS 按既有 fail-closed 逻辑处理。
这是一项真实链路限制，本轮不扩展修复。

221是完成 runtime ledger 中的 NDT调用数，不包括未完成 tx155 terminal 的
全部调用，不能宣称总调用数221。
启用 forensic diagnostics 的 wall85.66s / user81.89s / system3.42s /
maxRSS106424KiB，只是 SHORT_REAL_LINK_ENGINEERING_SANITY，非正式 benchmark。
记录状态 finite、SO3合法、时间戳单调；无精度结论。

### 八、产物与最终裁定项

docs/p6_alg_integration_a3d_r1/ 包含 summary、TERMINATION_CONTRACT、输入身份、
Release/Debug日志、逐项test stdout、SOURCE_SHA、changed_files、完整小型CSV、
tx115 preopt/optimizer/event/marginalization、tx155 first-failure trace和summary。

新 failure capsule 压缩为 TX155_FAILURE_CAPSULE.npz：39405 bytes，
SHA256=b44fe28654437ff1cd944945cb436f62368c7ff315e9e920686836950aed58a8。
615×615 consumed/prior等float64矩阵及production correction配套metadata已保存。
原18MBbinary保留外部，Git不提交bag/PCD/大型重复矩阵。

A3D_R1_SMALL_STEP_TERMINATION_CODE_CONTRACT=PASS
A3D_R1_TX115_REPAIR_GATE=PASS
A3D_R1_P3_200_REAL_LINK_GATE=FAIL
READY_FOR_FORMAL_EXPERIMENT=NO

GT_USED=false。未读取GT，未ATE/RPE，未调参，未正式精度实验，
未P3-400/full2777/Floor01/其他dataset/visual。
任务已在新失败后停止；下一轮数学验收与阶段决策属于科研总控。
