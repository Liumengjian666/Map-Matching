# P10-R2：低预算残差修正耦合 NDT 与 P7 单帧影子验证

## 判定与交付范围

TASK = PAPER-P10-R2-BUDGETED-COUPLED-NDT。

执行结果标签：`BUDGETED_COUPLED_NDT_SHADOW_IMPLEMENTED_WITH_LIMITATIONS`。

NEXT（唯一）= `EVENT_TRIGGERED_COUPLED_NDT_SHADOW_SAFETY_GATE`。

残差修正、覆盖式 8→16 搜索、最多两次额外 PCL NDT、确定性非 Oracle 排序和真实 P7 shadow 均已实现并运行。工程硬要求通过；连续 200 帧 mean/P95 100/150 ms 目标未达到。结果只支持算法原型可运行，不支持整体定位精度提升、安全位姿替换或实时部署。

分支：`research/p9-r4-heldout-visual-evidence`；START_SHA：`69d84d9ccef0776e2fdff9e88bb2783b34af3f1d`。实际提交工作副本为 `/tmp/dog_loc_paper_r4_ws.Fq21k2`，原论文工作区 `.git` 只读。END_SHA 由最终 Git commit 回执给出，不在被该 commit 包含的文件内制造循环引用。未 push；远端查询因网络连接失败，不能声称远端已有本轮成果。

## 实际源码及架构

- 新 API：`include/dog_prior_map_fastlio2_frontend_exp/coupled_ndt_shadow.hpp`；核心：`src/coupled_ndt_shadow.cpp` 中 `runCoupledNdtShadow()`、`jointPullback()`、`pullNativeJet()`、`strongStep()`、`coveringProposals()`、`diverseQualityPreviews()`。
- `CurrentFrameNdtRegistration::shadow()` 在 `src/current_frame_ndt.cpp` 共享现有 prepared source、同一 PCL NDT target/voxel grid。`nativeJet()` 和 `dynamicScore()` 提供原始动态 PCL score/gradient/Hessian；完整精修仍调用 PCL `align()`，不重写优化器。
- `scripts/p10/budgeted/p10_r2_single.cpp` 使用归档 SAME_OBJECTIVE 源；`p10_r2_replay.cpp` 复用真正 P7 初始化、`prepareScanWindow`、scan-end deskew、nominal NDT 和 IKFoM 更新。推荐位姿仅记录，状态更新始终使用原 nominal 结果。
- `shadow_logging.hpp` 记录每个候选初始/纯耦合预测/残差修正/精修后的完整位姿、强坐标、梯度、score、rank、迭代和时间。`final_candidate_ranking.csv` 在不读取 GT/Oracle 的后处理里重现最终排名，并逐帧断言其第一名等于冻结推荐。
- `run_budgeted.py` 在每轮运行前冻结输入、源码、二进制；`evaluate_budgeted.py` 先核验 blind output hashes，再进行 Canonical/GT post-hoc 评价；`finalize_archive.py` 汇总审计；`verify_value_reuse.py` 对两轮所有非计时值做 exact 比较。

没有改生产 ROS 协议、IKFoM 更新设计、R1 历史文件、P9 历史结果或稳定机器狗 workspace。核心模块不持有第二份地图；后端始终使用现有注册对象。

## 数学和冻结预算

沿用 P9 map-product chart：

```text
eta = W*u + S*v
p = p0 + 0.8*eta_translation
R = Exp(eta_rotation)*R0
E = -raw_score/N
g = J^T*g_native
H = J^T*H_native*J + sum_a g_native[a]*K_a
(Hvv + lambda*I)*delta_v = -(g_v + Hvu*delta_u)
v_new = v_previous + delta_v
```

`jointPullback()` 在当前非零 previous endpoint 计算完整 J/K，包含混合二阶项及 `g*K`；不能用 nominal 的对角 eigencurvature 伪造非零位置的耦合。弱维数继续使用 R1 的一维/二维判据。每候选只应用一次残差修正；另做 R1 预测供诊断/回退，所以 C 实际执行 R1 与 R2 两个 `strongStep()`/LDLT factorization，而非仅一次矩阵分解。SPD/条件数/残差守卫；必要阻尼使最小特征值达到 `1e-4*max(1,max(abs(diag(Hvv))))`，combined increment 范数上限 0.10。异常退回 R1，继而 warm-start；不运行 Newton20。纯 R1 预测、残差项和修正后 v 均单独记录，不能把 predictor-only 称为条件驻点。

覆盖来自原可行 17 点线或 9×9 网格，范围 2 m/15 deg；非零点按归一化 FPS 选最多 16 个，不是截取网格前 16 个。第一阶段 8 个，若不足两个互相分离的 near-quality previews，扩展至 16；两帧消融强制同一 16 点池与顺序。

```text
preview_quality: E <= E_nom + 0.05*max(1,abs(E_nom))
merit = E/max(1,abs(E_nom))
        + 0.05*((dt_to_IMU_prediction/2m)^2 + (dr_to_prediction/15deg)^2)
diversity: dt > 0.2m OR dr > 2deg
recommend: successful AND converged AND finite
           AND raw_score >= nominal_reevaluated_raw + 2.747604276e-4
           AND refined_merit < nominal_merit
```

preview 按 finite、merit、原 node ID 确定性排序，最多两个 diverse previews 进入完整 NDT。最终等 merit 保留原 incumbent，nominal 默认输出。运动尺度统一且冻结；没有协方差、GT、canonical pose/ID 或学习分类器参与排序。

每帧 full align ≤3（nominal 1 + extra ≤2），distinct preview candidates/value evaluations ≤16。导数查询不是免费工作：最终 C 连续帧每帧 17 次 jet（nominal + 16 transport），16 次 preview value，另有精修后的 terminal value，分别计账并包含在总耗时；不能将 ≤16 previews 解释为最多 16 次所有 objective 查询。

## 真实数据和冻结顺序

复用 Floor01 冻结 SAME_OBJECTIVE raw sources、prepared source hashes、地图及原 P7 IMU/raw timed inputs；不用已被否决的 topic-bag source。32 个离散事务是集成测试，不冒充连续因果轨迹。连续段为事前固定的原 scan index TX1–200，约前 20 秒；control/A/B/C 全段执行，未依 GT 挑帧。

每轮完整 builder 输出先 SHA256 freeze，再读 canonical/GT。GT 使用历史已冻结 Floor01 anchor 和 IMU/LiDAR 外参，不重新拟合；32 帧历史 nominal GT error parity PASS。canonical 七邻域只用于离线评价，非独立定位事件。

32 帧 source count/hash exact；同一归档 objective mean 32/32 exact。历史 W/S 最大 eigenvalue relative 差约 `1.2328434e-7`，active projector Frobenius 差约 `1.0100694e-6`，在原冻结容差内。PCL 运行期 probability 与标准化 pose carrier 的重评价分数不保证 byte parity：最大 mean gap `2.2045462e-7`，raw gap `3.08636465e-4`；推荐比较使用同一重评价 carrier 下 nominal/alternative，不能声称跨 carrier raw-score exact。

## R2-A：两帧 A/B/C 对照

|方法|完整 NDT 次数（两帧合计）|预评价/帧|精修七 ID 恢复|推荐七 ID 恢复|mean/P95 frame ms|
|---|---:|---:|---:|---:|---:|
|A Weak-only|5|16|1/7|0/7|92.371 / 138.030|
|B R1 predictor|3|16|1/7|1/7|98.934 / 144.868|
|C R2 residual|5|16|2/7|1/7|124.188 / 162.458|

三臂预算上限和搜索池相同，实际花费不同；不能声称是同实际算力下的因果效率优势。n=2 的 P95 仅描述这两帧，不是稳定性能估计。各臂 preview 七 ID 恢复均为 0/7。

|参考候选（离线）|A refined|B refined|C refined|
|---|---|---|---|
|TX616 P02|否|否|否|
|TX616 P03|否|否|否|
|TX616 P10|是|是|是|
|TX616 P12|否|否|否|
|TX616 P13|否|否|否|
|TX616 P17|否|否|否|
|TX2226 P09|否|否|是|

C TX616：推荐 node 3，相对 nominal `0.4764598207 m / 9.3635801337 deg`；score `1095.8621619004`，13 iterations；另一精修 node 21 score `1082.3176353183`，9 iterations。C TX2226：node 42 score `1597.2584324639`，17 iterations，虽恢复 P09，但不满足 nominal score improvement，因此推荐保持 nominal。两帧 C full align 分别 3/2。

三个进入精修的 C 候选，其求解前 previous endpoint 的强梯度范数分别 `1.4760 / 0.8825 / 2.3157`，确实非零；不是 preview/refined 终点的梯度。node 3 的 previous v 约 `(−1.03e-6,1.49e-7,−5.96e-8,3.49e-7)`，纯预测 v 为 `(0.003092,0.010301,−0.006622,−0.008571)`，修正 v 为 `(−0.043209,0.028251,0.001824,−0.029461)`。精修再改变其 `0.264747 m / 6.430084 deg`，说明不能把单步修正等同于完整局部收敛。

## R2-B：32 帧集成与连续 200 帧

最终 32 帧 C：45 align，max 3/frame；mean previews 15.25，29 帧扩展；mean/P95 `106.892 / 148.778 ms`。5 帧推荐 alternative，其中 3 个 nonlocal；平移 GT improved/same/worse=`2/27/3`，变差帧 `2350/3094/3962`。平移 RMSE `0.9771888434→0.9834726815 m`（变差），旋转 `4.06592645→3.30559145 deg`。该反例不被删除。

最终连续 200 帧结果：

|方法|完整 NDT calls|mean/P95 frame ms|推荐 alternative|平移 GT improved/same/worse|
|---|---:|---:|---:|---|
|普通 nominal control|200|27.144 / 38.384|0|—|
|A Weak-only|269|87.985 / 156.961|28|22 / 172 / 6|
|B R1 predictor|233|111.828 / 187.576|16|12 / 184 / 4|
|C R2 residual|296|138.547 / 197.021|33|27 / 167 / 6|

三方法每帧 preview 16、全部 200 帧扩展；C 33 个推荐全部是小于 nominal center contract 的局部变化，nonlocal recommendation=0。不能将该连续早段结果外推为长走廊退化恢复证据。C 变差帧为 `5/28/73/133/163/183`；改善帧为 `17/19/36/41/52/62/83/102/112/116/125/128/132/136/151/152/153/157/158/161/165/168/172/175/178/179/180`。

GT 推荐位置是每帧 counterfactual/shadow，不反馈给下一帧状态。C translation RMSE `0.0433095484→0.0422489085 m`，约 1.06 mm 降幅；rotation RMSE `1.47810575→1.53294832 deg`（变差）。实际 nominal trajectory 完全不变，不能宣称实现了真实轨迹精度提升。A/B translation RMSE 分别 `0.0423823415/0.0432026843 m`，rotation `1.50151950/1.51035029 deg`。

三轮共 3600/3600 registration/trajectory 非计时行 exact parity。所有候选与推荐 nonfinite pose=0；shadow 不改变 source、nominal、score、iterations 或后续状态。明显跳变按事前 `.2m OR 2deg` 记录，两帧 C=1、32 帧 C=3、连续 200 帧 C=0；未发生真实状态切换。

## 时间、内存与两次针对性改进

连续 replay 的 frame_total 是源读取、scan-end deskew、nominal NDT、shadow、状态更新的实测时间，结束于 CSV logging 前；一次地图加载/初始化/日志另由 process wall 记录。32 single-frame 测试还计入源准入校验的预处理开销，并不等同生产连续帧管线。C 200 帧 process wall `28.222365265 s`，摊销 `141.111826325 ms/frame`，不能只报告预测求解耗时。C incremental CPU mean/P95=`111.163/164.097 ms`；increment wall=`111.236/164.204 ms`。

C 平均阶段：native jet `43.101 ms`、preview `30.035 ms`、额外 refine `37.392 ms`、model/chart `0.501 ms`、solve `0.109 ms`；nominal align `19.467 ms`。主要瓶颈是动态 objective/导数及额外完整精修，不是小矩阵线性求解。

C peak RSS `111816 KiB`（约 109.2 MiB）；control `111736 KiB`，进程 peak 差 `80 KiB` 只是 paired RSS proxy，非精确组件堆占用。结构上没有复制第二份地图。

|轮次|变化|C 200 帧 mean/P95 ms|完整 calls|七 ID refined recovery|
|---|---|---:|---:|---:|
|attempt 0|原全局 FPS|138.284 / 198.174|296|2/7|
|attempt 1|4 near-center + 4 FPS 首阶段|153.453 / 210.152|588|1/7|
|attempt 2（最终）|恢复原 FPS，仅 score 邻居缓冲复用|138.547 / 197.021|296|2/7|

第一次改进虽将平均 preview 减少到 8.48，却增加完整 NDT 到 588，且恢复变差，因此回退。第二次仅减少 value kernel 分配；attempt 0/2 非计时 **32 tables / 13460 rows exact PASS**，没有可声称的整体性能改善。首阶段 zero-quality 就停止扩展的提案在真实运行前被否决：attempt 0 的全部 200 帧首 8 点无 admissible preview，但第二阶段产生 96 extra align 和 33 推荐，zero-quality 不能据此推断后续无价值。未执行提案的 diff 单独保存，不伪造试验成绩。

两次改进预算已用完，停止进一步调参或追加数据。最终整套矩阵 1056 次 NDT（三两帧臂 13 + 集成 45 + control 200 + A269 + B233 + C296）；三轮真实实验总计 4156。synthetic test 调用不混入真实数据计账。没有新 oracle、B12、视觉、Corridor bootstrap 或 source recovery。

## 验证及使用边界

Release build PASS；P7/P10 tests 3/3，P9 tests 41/41，原 P10-R1 tests 2/2。覆盖非单位/非零 chart、混合 K、非零 gv、阻尼/步长/回退、覆盖与预算、非精修历史、失败保留 nominal、stale source 拒绝及 shadow 后下一次 align parity。三轮输出 freeze、源码快照、GT post-hoc、CSV/JSON/hash、最终排序重现、`git diff --check` 均通过。最终归档核验 331 项 SHA256、67 JSON（拒绝 NaN/Inf 常量）、162 CSV / 48506 数据行列宽均 PASS。当前单模型独立只读审查已完成，未追加外部 CLI。

提交前仅移除 live replay 源码的额外 EOF 空行，未改变可执行逻辑或重跑实验。冻结 source snapshots 和原始测试/构建日志保留 byte-exact SHA；归档内 `.gitattributes` 仅对这些 receipts 允许 EOF 空行，live code 仍执行正常 whitespace 检查，不能为 diff-check 改写 frozen receipts。

构建/测试阶段曾遇 PCL 参数 const 接口和库搜索路径问题；按真实签名/运行环境修复，不改科学参数。最终 Release single binary SHA=`2023721f764830522a18096c06910599e30309a11c8ec7e1266a8235821c44d1`；replay SHA=`e1ab3528c13b322420c75e653ddaf918b66a1a00fbc424e2f995c9410c496ba9`。外部大数据和运行二进制快照放缓存，不提交 Git。

本轮创新增量是 **非零强梯度的单步残差修正 + 覆盖/分阶段有限候选 + 共享 NDT 后端的真实单帧 shadow**，不是新目标函数或融合算法。尚未解决总成本与推荐安全性，也未证明耦合方法有总体因果效率优势。

下一阶段建议仅开展 event-triggered coupled shadow safety gate：先事前冻结不依 GT 的触发/保留 nominal 规则，在指定连续段验证总成本和推荐风险，仍保留 nominal 的真实更新。该 NEXT 是建议，未自行实施；不直接切换生产位姿、不进入 EKF/fusion。
