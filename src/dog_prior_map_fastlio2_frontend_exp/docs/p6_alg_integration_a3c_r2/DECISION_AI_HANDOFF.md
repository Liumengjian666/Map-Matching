# 【执行 AI → 科研总控】A3C-R2 详细执行回报

任务：PAPER-P6-ALG-INTEGRATION-A3C-R2。
身份边界：Codex负责实现、测试、授权范围内短回放及事实回报；科研裁定与后续阶段由ChatGPT科研总控负责。执行已停止，没有自行进入下一阶段。

## 1. Git / 可读取产物

Repository: https://github.com/Liumengjian666/Map-Matching.git
Branch: research/p6-i6d-full-algorithm
Worktree: /home/jian/livox_ws/dog_loc_p6_i6b_ws（复用，没有新建）
START_SHA=d97c3cd8e2faeb6a047e93a789692b898429fcc9
CODE_SHA=32e7f71a8f810590e1ee1090811b2c383b700305
END_SHA为本报告所在analysis delivery commit，执行AI最终聊天回报提供完整字面SHA及实查remote HEAD。
报告目录：src/dog_prior_map_fastlio2_frontend_exp/docs/p6_alg_integration_a3c_r2/

提交1：fix: remove Eigen transpose aliasing from symmetric information paths。
提交2：analysis: validate A3C-R2 fixed-lag repair。
无force push、无squash、无旧产物覆写。没有将PCD、bag、raw points或大量迭代state step向量提交；保留了可审查的紧凑CSV和首故障证据。

## 2. 本轮裁定所需核心结果

```
A3C_R2_ALIAS_SAFE_SYMMETRIZATION_CODE_CONTRACT=PASS
A3C_R2_P3_100_REAL_LINK_GATE=PASS
A3C_R2_P3_200_EXTENDED_GATE=FAIL
READY_FOR_FORMAL_EXPERIMENT=NO
```

原tx90 aliasing故障已消除，且完整完成两次oldest removal，不是只绕过错误。
但P3-200出现新的tx115优化器失败；不能将本轮写成更长真实链路稳定PASS或定位精度PASS。

## 3. 修改内容与未修改内容

包内28处self-transpose对称化赋值逐条审计，20处confirmed unsafe修复，8处原本eval-safe保留。
linearizeSelected Hessian和M7 new_information使用明确独立temporary，RHS整式先求值。
其余18处只增加whole-expression eval，涉及dual_reliability、reliability_metrics、视觉/IMU协方差、noise模型及原jitter敏感性参考计算。
另修A1-R1测试Schur oracle自身的alias写法，不改容差；旧tx90 unsafe表达式保留为forensic test reference，不进入production。

理想公式全部仍为(A+A^T)/2；没有改Schur、原始残差或目标函数。
existing prior + touching-oldest factors被消耗，retained-only factors继续active/relinearizable，information conservation合同保留。
FrozenLidarProjection、B^T r、B^T R B、inner-frozen/outer-relinearized均未修改。
rank5没有被补成rank6，pose弱方向零信息有针对性测试。
threshold changes=NONE；jitter changes=NONE；stored-prior eigen clamp=NONE；artificial information injection=NONE。
没有调整duration/node limit、optimizer迭代/step/damping/acceptance、NDT、U_obs、U_nonlocal、NIS、noise或地图外参。
原reliability中既有特征值重建/限制表达式保留，不是本轮新增clamp。

## 4. 构建与数学回归

Release build PASS；原28项全部保留，full CTest31/31 PASS。
Debug targeted9/9 PASS；diff-check PASS。
A1 clean H/g=1.24607e-10/7.85293e-13；noisy H/g=1.24607e-10/7.85317e-13。
Repeated conservation、10000-cycle ID生命周期、nonzero rotation prior、jitter regression全部PASS；prior gradient error0。
新同-enforcement两次删除H relative error7.2511587330693086e-17，g error2.540132453772453e-22。
A3B-R1 forensic、A3B-R2 frozen、A3C-R1 diagnostics、sparse/dense equivalence均PASS。
OFF/ON diagnostics状态、先验、gradient、因子与决策相同。

真实旧tx90 capsule SHA=2f597628160e75f41bc4c13d9de015727c17b1755535ad806263f272ebcf0280。
legacy expression仍复现max asymmetry1.1175870895385742e-8、validator FAIL。
NEW actual production helper与独立eval oracle完全相同，max symmetry0，validator PASS，lambda_min0。
PSD symmetry1e-8、eigen tolerance1e-6未变。

## 5. 输入身份与实际执行次数

七项SHA重算与冻结记录一致，runtime requireRawTimedInputManifest实际通过；详见REAL_INPUT_IDENTITY.md。
同一Corridor01真实raw timed输入、IMU、normalized map、official params、initialization1517157224188979000。
P3 ADAPTIVE_SELECTED_NIS，visual=NONE/provenance=NONE；相同library path。
一次P3-100，PASS后一次P3-200；没有第三次或alternate policy运行。
不读取GT，不做ATE/RPE，不完整跑2777帧，不跑Floor01。

## 6. P3-100 / tx90修复验证

frame_limit100包含51个handoff前raw scans；实际49个Window-owned deskew/terminal全部完成。
NDT converged49、U_obs valid49、sparse P15 available49；U_nonlocal probes24；NDT calls97。
LiDAR committed46，3个正常NIS拒绝。Schur oldest removals60/60，30次enforcement各删除2个。
stored-prior symmetry全部0，lambda_min最小0；consumed H/Hmm symmetry全部0；solve-jitter次数0。
无dense fallback、无inner basis callback、无visual event/factor、无handoff后IKFoM调用。
完成事件max nodes40、span1.958914906s；finite pose/SO3/timestamp检查通过。

tx90/stamp1517157228164951397：ACCEPTED_UPDATE，8iterations，cost29.640314594899319→21.667662999656720。
attempt1 SUCCESS：41→40 nodes，span2.017077923→2.017057491s，raw Schur symmetry7.2759576141834259e-12，stored symmetry0，lambda_min0。
attempt2 SUCCESS：40→39 nodes，span→1.916218042s，raw symmetry2.9103830456733704e-11，stored symmetry0，lambda_min0。
两次jitter0，validator均PASS，完成2s duration enforcement。
修复旧算术伪差会改变后续state/cost/NIS，因此不要求与旧SHA旧轨迹数值相同。
资源39.14s wall/94,308KiB RSS；包含forensics，不是论文CPU性能结论。

## 7. P3-200新的首故障

STOP tx115 / stamp1517157230686328484 / LIDAR_SCAN_END。
producer_optimizer:all_optimizer_candidates_rejected。
optimizer_status=FAILED_ALL_CANDIDATES；iterations1；initial/final cost81.345724589884568。
这是OPTIMIZER_STAGE，尚未进入该事件marginalization；不是tx90 symmetry问题复发。

第一candidate：lambda1e-6，raw/applied step7.0568167407415232e-9，未clip，gradient_inf2.8457479913868156e-5。
predicted reduction3.4098706667169588e-14；candidate cost81.345724589884583；actual reduction-1.4210854715202004e-14。
candidate basis callback0；solver BLOCK_SPARSE_SIMPLICIAL_LDLT。仅回报这些事实，不擅自归因/改阈值。
current breakdown：prior1.5173310842164949e-5，IMU5.7211325015960002，LiDAR75.624576914977752，visual0。

tx115 NDT converged（fitness75.624965709133562、34iter）；U_obs valid/rank5；P15 valid；U_nonlocal triggered且两侧terminal记录存在。
NIS15.237396865140097>15.086，current LiDAR正常拒绝；preopt factors40IMU/17LiDAR/0visual。
preopt41nodes/span2.017077208s，优化失败阻止该事件duration enforcement，不能宣称failure后window最终有界PASS。
既有诊断已落盘，state difference0，transaction safety PASS。

到tx114完成63个terminal、57个factor commits、6个NIS拒绝；tx115 deskew也完成，所以deskew64。
已完成事件runtime ledger139 NDT calls，不把未写completed runtime row的失败terminal调用漏记或冒充全程总数。
此前88/88 oldest removals成功，44次2-removal enforcement；stored symmetry0，min eigen0，fallback0，无jitter。
没有自然出现attempt2 marginalization failure，无人为failure注入，batch rollback没有本轮重设计或认证。
资源49.21s wall/93,516KiB RSS。

所有首故障证据已保存：RUN_P3_200/FIRST_FAILURE_optimizer.csv，preopt_capsule.csv，failure summary，既有directional FD/damping诊断、完整preceding events、deskew/runtime/resource。
无新Schur binary capsule，因为该阶段没有执行。完整大step-vector trace仍在外部临时结果目录，关键故障行已持久化Git。

## 8. 可审查文件与最终边界

summary.md；TRANSPOSE_ALIAS_INVENTORY.md；MATH_CONTRACTS.md；TX90_REPAIR_RESULT.md；
P3_100_RESULT.md；P3_200_RESULT.md；REAL_INPUT_IDENTITY.md；BUILD_AND_CTEST_RESULTS.txt；
Release/Debug完整test details；RUN_P3_100/与RUN_P3_200/engineering_summary.json及小型CSV。

请以真实Git对象、源码diff和上述证据审查本轮修复；不要把工程门通过、NDT convergence、factor commit或平滑性解释成定位精度。
本轮没有对tx115新增root-cause裁定或修复。科研后续决策属于总控，执行AI已依照新首故障STOP规则停止。
READY_FOR_FORMAL_EXPERIMENT=NO。
