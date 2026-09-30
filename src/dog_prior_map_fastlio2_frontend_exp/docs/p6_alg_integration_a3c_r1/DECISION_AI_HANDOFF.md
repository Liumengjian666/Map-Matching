# Copy-ready review prompt for the research decision AI

```text
【执行 AI → 科研总控 / 决策 AI】

PAPER-P6-ALG-INTEGRATION-A3C-R1 执行回报与根因复核请求

本轮性质：FORENSIC ROOT-CAUSE ATTRIBUTION ONLY。
没有实施 production fix，没有正式精度实验，没有读取 GT。

Git：
Repository: https://github.com/Liumengjian666/Map-Matching.git
Branch: research/p6-i6d-full-algorithm
START_SHA: 8f66c1d6d4cc7e1c250488c1f118b7a31b2006e1
CODE_SHA: f0de80a4a60d69efb03e32bf2332a585fe473dbd
END_SHA: 由最终执行回报提供（不在包含本报告的commit中自引用）
push: 待第二个分析提交完成后执行一次普通 push

冻结合同：A3B-R2 OUTER-RELINEARIZED / INNER-FROZEN LiDAR SUBSPACE 未修改。
没有修改 FrozenLidarProjection、B^T r、B^T R B、NDT、adaptive R、U_obs、
U_nonlocal、NIS、window、jitter、PSD阈值、optimizer 或 Schur 控制策略。

唯一一次获准的 Corridor01 P3-100 replay 使用 A3B-R2 相同输入、官方标定、
handoff=1517157224188979000、ADAPTIVE_SELECTED_NIS、visual=NONE、frame_limit=100。
所有 raw/map/params/IMU SHA 与 A3B-R2 记录一致，raw manifest gate 通过。
未重跑 P3-100；未运行 P3-200/full Corridor01/Floor01；未读 GT。

确定性结果：
- 复现 transaction 90, stamp 1517157228164951397，reason
  marginalized_prior_not_finite_psd。
- optimizer 先 ACCEPTED_UPDATE，8 iterations，cost
  17.972697968746459 -> 13.976734288274939。
- failed enforcement index 78，nodes=41，span=2.017077923 s；duration trigger
  true，node trigger false。
- attempt_index_within_enforcement=1；同次 enforcement 前没有成功 removal。
- failure 前/后 state stamps、prior hash 和 40/17/0 IMU/LiDAR/visual factor counts
  完全一致；multi-removal transactionality 对失败路径是 NOT_EXERCISED。

重复边缘化历史：
- 首次成功是 tx71 / event stamp 1517157226248733355。
- tx90 前 19 个 enforcement episode、38 个 successful oldest removals；前 19 个
  enforcement 每次各删除两个 oldest states。
- 38 次成功后 incoming/new prior lambda_min 和 relative-negative ratio 都为 0；
  没有发现 tolerated negative prior mode 逐渐累积。
- 历史 H_mm condition proxy 最大约1.6044e9、最小 pivot ratio约6.2486e-10，但全部
  仍为 primary rank 15/15，LDLT pivots无负值、无 near-zero、从未触发 jitter。

tx90 矩阵证据：
- incoming prior/ charted prior 有限且 PSD；lambda_min=0。
- H_touch 最小特征值 -2.7247e-7，谱尺度2.0000e9，relative-negative 1.36e-16，
  不是 material indefiniteness。1 IMU + 1 rank-5 LiDAR + 0 visual。
- H_c finite PSD，rank=30/615。
- H_mm finite SPD/full rank15；lambda_min=7.58949e6、lambda_max=1.000015e9，
  condition proxy131.76。
- LDLT D有15正/0负/0 nearzero；min/max abs D=7.589491e6/1.000015116e9，
  relative pivot ratio=0.00758938；jitter=0。
- solve backward error：H_mr约1.36e-16，g_m约3.97e-17。
- primary及1e-14/1e-12/1e-10三种相对阈值下 Hmm均rank15/nullity0；H_mr/g_m range
  residual均0。
- new gradient finite，norm40.7850，maxabs37.39894。

FIRST_BAD_STAGE = M7_SYMMETRIZED_SCHUR。
PRIMARY_ROOT_CAUSE_CLASS = A3C-R1-G / OTHER_IDENTIFIED_CAUSE。
具体原因：用真实capsule重演 production Eigen 原位表达式
  new_information = 0.5*(new_information + new_information.transpose())
后，max(abs(S-S^T))=1.1175870895385742e-8，超过production现有1e-8对称性门槛；
production PSD validator因此在M7拒绝。原位结果的symmetry Fro为1.58188368e-8。
同一raw Schur out-of-place对称化后max asymmetry=0，并通过完全相同的production
validator。独立谱分析lambda_min=0，故不是负特征值超容差。
这里的lambda_min=0来自独立计算的对称 forensic matrix spectrum；production
在对称性检查处短路，没有对原位候选矩阵执行后续特征值检查。

一般Schur/pseudoinverse oracle：
- production correction Schur vs generalized pseudoinverse：H relative Fro error
  1.2413e-10（absolute Fro diff4.1494e-6），gradient relative3.4027e-19
  （absolute1.3878e-17）。
- 独立计算的对称 production-correction matrix与generalized-Schur oracle均
  lambda_min=0、relative-negative=0；production本身在M7对称性检查处停止，未计算该谱。
- tx90 jitter=NO；direct solve Schur relative H error1.3812e-11。
- pseudoinverse只作为 forensic oracle，没有替代 production solver。

重要取证记录：捕获CSV中的原始 heuristic `first_bad_stage` 字段仍为
M2_TOUCHING_FACTORS，因为首版诊断误将production的绝对1e-8 symmetry门槛套在了
中间大尺度Hessian上。其relative symmetry defect仅4.47e-17。之后修正了诊断阶段分类：
中间阶段做尺度相对判读，M7精确对production validator；没有重跑真实数据，且该改动
不进入production控制流。原始CSV保持不变，单行键控文件
FIRST_BAD_STAGE_ADJUDICATION.csv并列记录raw和adjudicated stage。请以
capsule-based C++ exact-expression emulation 和 TX90_STAGE_ANALYSIS.md 中的校正
FIRST_BAD_STAGE 为准，同时审阅该差异是否已透明披露。

建议的未来 MINIMAL_REPAIR_TARGET（本轮未实施）：
在Schur矩阵上先构造独立/求值后的对称矩阵，再赋回 stored prior；保持当前
1e-8 / 1e-6 validator阈值、无jitter、无eigen clamp、无人工信息注入。任何修复都需
保留 Schur conservation、rank-deficient LiDAR方向和A1-R1/A3B-R2回归。

构建/测试：
- Release runner和新诊断测试目标构建PASS。
- Release全量CTest 28/28 PASS，包括A1-R1、A2D、A3B-R1、A3B-R2和新增A3C测试。
- Debug定向5/5 PASS（A1-R1、A3B-R1、A3B-R2、A3C C++、A3C Python oracle）。
- capsule C++ exact-expression emulation PASS；git diff --check PASS。
- 唯一真实replay停在tx90预期失败，因此本轮结论是root cause isolated，而非算法通过。

请独立复核：
1. 是否接受 A3C-R1-G / M7 的根因归属，还是认为仍需更多证据？
2. intermediate factor/Hc/Hmm的相对负值是否足以排除H1/H2/H3？
3. `H_mm` generalized Schur oracle和production correction的比较是否正确解释？
4. 当前multi-removal transactional status是否应保持 NOT_EXERCISED？
5. 上述最小修复候选是否严格不注入弱方向信息，并且足以进入后续修复审查？

本请求仅请对本轮证据和根因分类做复核；执行 AI 未实施下一轮修复或实验。
READY_FOR_FORMAL_EXPERIMENT = NO。
```
