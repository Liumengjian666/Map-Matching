【执行 AI → 科研总控 / 决策 AI】

# PAPER-P6-ALG-INTEGRATION-A3F-R1 执行回报

本轮已完成。请基于实际源码、测试和唯一真实短链路结果复核；不将工程
PASS解释为定位精度PASS或正式论文实验批准。执行端已STOP。

## 1. Git与证据入口

Repository: https://github.com/Liumengjian666/Map-Matching.git

Branch: research/p6-i6d-full-algorithm

Workspace: /home/jian/livox_ws/dog_loc_p6_i6b_ws（复用，不新建worktree）

START_SHA=34ac3f7692ff51efc3c168fb45e080b8573dee28

CODE_SHA=c1ef450eb45fd5825c416e9140e9a4eed1d32d04

END_SHA和remote HEAD以执行端最终回传中的完整SHA及实测push状态为准。
本文件所在最终analysis commit即交付END；不在Git文件中伪造自引用SHA。

代码commit：feat: add square-root QR window marginal covariance。
结果commit：analysis: validate A3F-R1 pre-measurement covariance。

报告路径：src/dog_prior_map_fastlio2_frontend_exp/docs/p6_alg_integration_a3f_r1/

关键材料：summary.md、SQUARE_ROOT_COVARIANCE_CONTRACT.md、
IMPLEMENTATION_AND_REVIEW.md、BUILD_AND_CTEST_RESULTS.txt；RUN_P3_200/中的
COVARIANCE_REQUESTS.csv、LEGACY_QR_SAME_STATE_COMPARISON.csv、
OLD_TX136_155_COVARIANCE_REGION.csv、QR_MARGINALIZATION_HISTORY.csv、
engineering_summary.json、input_identity.json、source_identity.json、
identity_gate.json、resource.txt。外部完整大日志有EXTERNAL_RUN_FILES.json身份索引。

## 2. 实际生产改动

production covariance backend=SQUARE_ROOT_QR（V3显式选择）
legacy fallback=NONE
square-root prior authoritative=YES
pre-measurement timing unchanged=YES

全窗口assembler直接消费authoritative A/b与当前local chart Jacobian，
加全部active IMU、历史LiDAR、visual raw factor rows。R=LL^T；Jw=L^-1 J、
rw=L^-1 r。没有重新factorize prior H，没有以J^T R^-1 J形成生产covariance H。

每次query仅创建一个Frozen LiDAR snapshot；生产QR和legacy shadow共享。
AP=QR；使用CPQR；full-column rank必须等于15N。
标准rank阈值=eps_machine*max(m,n)*maxPivot，Eigen内部使用对应relative threshold。

selector E底部15行I；B=P^T E；只做R^T Y=B triangular solve。
P15=Y^T Y；Ymap=Y*Tmap^T；Pmap=Ymap^T Ymap。
Tmap沿用rotation block与map position block定义，没有换误差坐标。

P15/Pmap由Gram构造，经finite/symmetry/LLT验证；无clamp、epsilon-I、jitter、
pseudoinverse、rank drop或legacy fallback。rank不足直接FAIL CLOSED。

query mutation=NONE：states、prior A/b、H/g、factor/ID/watermark及window/
optimized revisions均不变；只有既有diagnostic request/time counter会记录query。

旧normal sparse LLT保留为tests/diagnostics；仅diagnostics ON执行shadow。
shadow不能影响P15、Pmap、probe、NIS、admission或state，也不额外调用basis callback。
非LiDAR事件完全不求P，日志为NOT_REQUESTED_NON_LIDAR_EVENT/NaN。

## 3. 冻结合同

A3E-R2 QR oldest elimination、row compression、prior A/b权威链未修改。
optimizer仍BLOCK_SPARSE normal-equation solver，没有改成square-root optimizer。
A3B-R2 inner-frozen projection、A3C-R2 alias-safe symmetry、A3D-R1 natural-small-step
termination、A1-R1 retained-only factors不重复计入合同保持。

threshold changes=NONE；damping changes=NONE；NIS changes=NONE；noise changes=NONE。
NDT/U_obs/U_nonlocal/deskew/handoff/calibration/reliable rank/window参数均未调整。
旧FULL/V2默认行为和冻结baseline/历史结果未修改。

## 4. 构建与测试

Release build=PASS
full Release CTest=37/37 PASS（原35项保留，新增2项）
Debug targeted=15/15 PASS
git diff --check=PASS

well-conditioned equivalence=PASS
forced permutation regression=PASS；P15 relative error9.83421e-16
correlated covariance L^-1 whitening=PASS；relative2.73124e-14
extreme scale known reference=PASS；relative3.331e-16，legacy normal LLT失败
rank-deficient fail-closed/read-only=PASS
rank-5 LiDAR=PASS（整体full rank不被冒称为LiDAR rank6）
nonzero rotation/current prior chart与非零gradient=PASS
joint IMU/LiDAR/directional visual、repeated Schur covariance=PASS
diagnostics covariance exact parity、NIS/probe parity、一次callback=PASS
query states/prior/lifecycle/revisions不变=PASS

旧源码审计最初因函数移入新文件而报IndexError；已保留旧legacy no-dense断言，
并增加QR直接rows/no-normal-solve/no-inverse检查；未删测试、未放宽数值容差。
只读独立审查提出的snapshot rank防护和日志竞态/身份校验问题均已修复并测试。
没有声称已完成外部跨模型审查；按用户要求留到最终手动发送。

## 5. 唯一真实P3-200

Corridor01 / FULL_FIXED_LAG_V3_EXPERIMENTAL / ADAPTIVE_SELECTED_NIS。
frame_limit=200；visual=NONE；visual provenance=NONE。
initialization_stamp_ns=1517157224188979000。
raw timed input/IMU/map/official params与冻结hash一致；真实manifest gate PASS。
REAL_REPLAY_COUNT=1；retries=0；exit=0；completed=YES。

200 raw scans中51个早于handoff，实际149个Window-owned deskew/LiDAR terminals。
square-root covariance requests=149；available=149；unavailable=0；rank failures=0。
legacy shadow available=129；unavailable=20。
legacy failure histogram：HESSIAN_NOT_SPD=17；NUMERICALLY_SINGULAR_HESSIAN=3。
不可用仍位于tx136–155，singular三个为tx140/154/155。此次保存了具体gate，
不根据上一轮缺失的内部trace猜测旧gate。

P15 nonfinite/PSD failure=0；Pmap nonfinite/PSD failure=0。
P15全run eigen range=1.367054266694708e-6 ..3.760208570973959。
Pmap range=7.973431667598901e-5 ..1.1177630593544055。
triangular residual max=2.6705807443139463e-16。

## 6. AVAILABLE帧的严格同状态对比

129个legacy AVAILABLE帧使用同一CURRENT state/raw factors/frozen projections：

P15 relative Frobenius error：
median=2.147503999472176e-8；P95=9.168520674615725e-8；max=1.559500819882654e-7。

Pmap relative error：
median=1.688278454381637e-8；P95=6.7176465746004e-8；max=1.6119122075481807e-7。

103个有效selected NIS pair：绝对差median=1.5084263005338272e-9；
P95=1.093140951979875e-7；max=8.210267381514313e-7。

NIS decision differences=0；probe trigger differences=0；same-factor admission differences=0。
这是同状态/同measurement的反事实比较，不是重跑一条legacy闭环轨迹；
若probe trigger不同，不伪造未运行的alternate NDT terminal和adaptive R。

## 7. 原缺口区域与分叉

tx136–155生产QR全部AVAILABLE，rank615/615，NIS有效；16个LiDAR committed，
4个正常拒绝。没有因为P有效而强制接受所有LiDAR。

first covariance output divergence：tx52，stamp1517157224332516266。
P15 relative1.2094514074128008e-9，Pmap2.7829433939584123e-9。
QR NIS0.21191546349516932，legacy0.21191546316551765；均接受，不probe。
此前scan-start及首次query的预测pose与旧run一致；无pre-output漂移。

first old-run event numerical divergence：tx70 scan-start，stamp1517157226047034025，
在tx69首次实际probe之后。tx69 adaptive R已出现roundoff-scale数值差异。
之后闭环可能显著分叉，不宣称旧run状态与新run逐帧一致，更不作为精度改善证据。

first same-state shadow decision divergence：tx136，stamp1517157232804267479。
QR valid，legacy HESSIAN_NOT_SPD，rank615，NIS0.26912481158413021 <6.635；
probe triggered=YES，LiDAR committed=YES。

## 8. 算法健康

selected NIS valid=123。
LiDAR committed=107；normal rejection=42：16个NIS拒绝、25个地图支持不足、
1个zero/invalid reliable rank（后26个没有有效selected factor，不是P失败）。
NDT converged=149；U_obs valid=124。
U_nonlocal probes=104；NDT calls=357。

optimizer events=298：291 ACCEPTED_UPDATE，7 CONVERGED_WITHOUT_STEP，failures=0。
QR marginalization attempts=260；failures=0。
post-handoff IKFoM calls=0；visual events/factors=0；dense fallback=0。
candidate basis callback=0；non-LiDAR marginal request=0。
max completed nodes=40；max completed span=1.9589149060000002s；state finite/SO3合法。

tx90/tx115/tx155均ACCEPTED_UPDATE并完成2次rank15 QR removal；stored H symmetry0，
extended prior minimum eigenvalue0。最终span分别1.916218042、1.916217089、
1.916217804s。没有破坏已经关闭的alias/small-step/QR marginalization合同。

## 9. 工程成本与局限

covariance QR（row assembly+QR+solve+Gram验证，排除legacy shadow）：
mean=37.19617220805371ms；P95=44.3254904ms；max=46.512174ms。
max stack=729 rows ×615 columns。
max temporary estimated bytes=13767072（不含shadow、allocator overhead、持久状态存储）。
QR marginalization mean=4.853760919230767ms；max=5.919806ms。
whole run wall=160.70s；user=150.81s；system=9.42s；maxRSS=112520KiB。

这些只是diagnostics ON的ENGINEERING OBSERVATION；dense CPQR仍有明显低算力成本。
没有自行改SparseQR/增量solver，也不据此宣称正式CPU benchmark或精度提升。
由于新P参与probe/admission后轨迹改变，旧run与新run总耗时不构成严格性能消融。

## 10. 最终状态和待科研总控处理的边界

A3F_R1_SQUARE_ROOT_COVARIANCE_CODE_CONTRACT=PASS
A3F_R1_PREMEASUREMENT_P15_GATE=PASS
A3F_R1_P3_200_REAL_LINK_GATE=PASS
READY_FOR_FORMAL_EXPERIMENT=NO

证据已经关闭当前P3-200的P15 unavailable缺口，但只覆盖这一条无视觉短链路。
rank-deficient graph仍按既定policy fail closed；主optimizer仍normal-equation。
dense QR低算力成本、以后更长链路及正式实验Freeze均未在本轮裁定或执行。

执行端未运行P3-400/full2777/Floor01/视觉/GT/ATE/RPE/参数搜索；没有第二次真实replay。
已完成本轮并STOP，不自行进入下一阶段。
