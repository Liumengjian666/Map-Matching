# P9-R2B：非 oracle competing-terminal evidence gate

## 正式结果

`FINAL_RESULT = ORACLE_DISCOVERY_WITHOUT_USABLE_ONLINE_EVIDENCE`

`NEXT = VISUAL_INDEPENDENT_NONLOCAL_EVIDENCE_GATE`

B12 达到三个基本数值门槛，但没有通过在加载 label 前预声明的删一帧分类稳定性门槛。这里的 FAIL 不等于没有任何区分信号：B12 AUC 明显高于随机，且所有删帧 AUC 都高于 .80；未闭合的是接近门槛的 LOFO balanced accuracy 稳定性。不得事后降低稳定性要求、换 feature、用 B16 救主结论，或继续纯 NDT cost optimization。

DUAL-U 主线、H1 和 R2A 的有限 discovery 结论保持不变。本轮不证明 conditioning 因果效率、不证明 objective-best pose 安全、不证明多个 stationary dynamic basins、不声称在线实时。

## Git、隔离与归档

- Branch：`research/p9-r2b-nonoracle-evidence`
- Start SHA：`2712573a8d3be1768ba59f6aa92f184d0949d19f`
- End SHA：包含本报告的最终提交；执行完成时聊天回传完整 SHA。
- Worktree：`/home/jian/livox_ws/dog_loc_paper_ws`
- Stable workspace 和 production localization core：未修改。
- 原有 `.vscode/` 和两个异常名称文件：未修改、未删除、不暂存、不提交。`Testing/` 保持未跟踪且不提交；第一次误入口 CTest 附带更新了其 Temporary 下的两个测试日志，见验证章节，未删除或擅自恢复它们。
- `NEW_NDT_CALLS = 0`；不重跑任何 R2A alignment。
- `GT_USED_FOR_EVIDENCE = NO`；`GT_USED_FOR_GATE = NO`。

第一阶段为独立进程，仅打开 start commit 中的 R2A `ndt_runs.csv`、`predictor_parity.csv` 和 `score_carrier_sensitivity.csv`。构造模块没有导入旧 evaluation 模块，不加载 label、canonical pose/ID 或 GT。这里的 blindness 指计算流程的信息隔离，不声称作者不知道任务给出的 cohort。

构造得到 566 个 budget-scoped clusters、69 条 competitive representatives 和 128 行 frame/budget evidence。冻结 `nonoracle_evidence.csv`：

`SHA256 = ae2d72cb76f289300bd69c5c763c2e11d662e3567a60d79747e66c671f4e7c4b`

`evidence_freeze.json` 保存输入、构造源码、预声明 THEORY、数学共享库和三个证据文件的 hash。冻结后才打开 H1 frame table 的9个 frame labels；其余23帧为 NO_MAJOR。初始 evaluation snapshot 在读取 archived GT annotation 前形成。随后仅修正未执行的可选 directional reader 字段名并补 synthetic regression test；`evaluation_snapshot_initial.json` 保留初始版本，source revision 强制验证所有 statistic、threshold、CSV 与 gate 完全不变。盲构造源码和证据 hash 没有变化。

## Non-oracle 合同

`EPS_SCORE = 2.747604276e-4 = 2 * 1.373802138e-4`，来源为既有 numerical carrier audit，不来自 label。

只用 COND_WEAK2 nested prefixes B4/8/12，B16 为 secondary。converged==1 才进入聚类；iteration-limit 且 converged==1 仍保留。128/256/384/512 个 prefix returns 全部 converged，相应 iteration-limit 为6/9/13/17；这些是复用的 returns，不是新 calls。

Complete-link：每个 member pair 均需 translation<=.2m AND rotation<=2deg；全局最小 complete-link distance 合并，member-rank lexicographic tie break。T0 所在 group 为 NOMINAL。每个 cluster representative 取最高 raw score，score tie 取最低 probe rank。局部 cluster ID 不代表 oracle basin ID。

竞争集合：非 NOMINAL group 且 representative `S>=S0+EPS_SCORE`。分数越大越好，energy=-S。分数只决定 competition，不授权替换 T0。

`xi = [(t_k-t0)/.8; Log_spatial(R_k R0^T)]`，精确保留历史 P9 Matrix4f carrier。`A_comp=sum xi xi^T`；`U_comp=sqrt(lambda_max(A_comp))`。空集合为零。没有除以 B、概率权重或 covariance 解释。

Complete-link 的非 nominal group 代表点不一定在 T0 center ball 外，不能把两种 separation 混为一谈。按指定规则保留此类点，并额外标记其 center distance；不增加未经授权的筛选条件。B4/8/12/16 分别有0/1/7/8条这种 competitive representative，共16条 budget-scoped records。尤其2722、3796在B12只有 inside-center representative，不能写成几何严格分离的 local-minimum certificate。

## Frame-level 统计

独立单位32 frames，major9 / no-major23。ROC exact ties 取平均 rank。one-sided AUC permutation：10000次 frame labels shuffle，保持9 positives，PCG64 seed20261011，`p=(1+#null>=observed)/10001`。四预算共用 permutation manifest。

LOFO threshold 仅在31 training frames 上最大化 balanced accuracy，exact tie 取更高 threshold；报告的是 pooled held-out balanced accuracy，不是普通 accuracy。预声明稳定性为删除任意一帧后，剩余31帧 AUC 和 nested LOFO（各30 training frames）balanced accuracy 都保持>=.80。这比仅展示 AUC 范围更保守，但在读 label 前已经冻结，不能因看到结果而改动。

| B | major U mean / median | no-major U mean / median | ROC-AUC | permutation p | LOFO BA | sensitivity / specificity | ordinary accuracy |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 4 | .364407 / 0 | .083051 / 0 | .584541 | .183382 | .589372 | .222222 / .956522 | .750000 |
| 8 | .764665 / .045737 | .093908 / 0 | .734300 | .006499 | .678744 | .444444 / .913043 | .781250 |
| 12 | .906683 / .072112 | .111180 / 0 | .857488 | .000300 | .801932 | .777778 / .826087 | .812500 |
| 16 secondary | .907531 / .071903 | .112447 / 0 | .850242 | .000600 | .780193 | .777778 / .782609 | .781250 |

B12 的 p 为3/10001（2个 null draws 至少达到观察值）。TP=7/9、TN=19/23，FP=4、FN=2。三个基本条件同时 PASS；formal gate 因稳定性 FAIL。

| B | omission AUC range | nested omission BA range | omission stability |
|---:|---:|---:|---|
| 4 | .538043–.611111 | .478261–.611111 | FAIL |
| 8 | .701087–.769022 | .644022–.706522 | FAIL |
| 12 | .839674–.913043 | .788043–.876263 | FAIL：27/32 omissions BA<.80 |
| 16 | .831522–.907609 | .766304–.828804 | FAIL |

B12 删除8个非零证据 major frame 中任意一个时，BA=.788043；删除19个零证据 no-major frame 中任意一个时，BA=.797980。不是某一个巨大 U 的 frame 独占了 AUC，而是总体分类裕量很薄。完整逐 fold 和 omission 数据在 `lofo.csv`、`omission_stability.csv`。

这些是 nominal budget-specific p-values；不宣称经过多预算 multiplicity correction 的普适结论。B16 的 LOFO BA 本身也没有达标，因此不是“只有B16有效”。

## Key frames，B12

Score advantage 为竞争集合最佳 `(S-S0)/N_source`。norm、trace 在同一 dimensionless chart。

| frame | clusters | competitive | U_comp | trace | max displacement | best score advantage/source | inside-center reps |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 368 | 4 | 1 | .054360843 | .002955101 | .054360843 | .00595549609 | 0 |
| 2226 | 7 | 1 | .122689359 | .015052679 | .122689359 | .0160655965 | 0 |
| 2350 | 10 | 7 | 4.048115334 | 16.607699949 | 2.547498842 | .307863668 | 0 |
| 2722 | 5 | 1 | .024712999 | .000610732 | .024712999 | .00447296299 | 1 |
| 3796 | 7 | 1 | .046913885 | .002200913 | .046913885 | .000879746728 | 1 |

2350 无需 P01/P05、canonical pose、GT 或任何 oracle admission 就产生强证据：7个 objective-better 非 nominal groups，U=4.0481。2226有较小非零证据；2846在B12为零，即 known major label 不能保证本 descriptor 检出。

## 全23个 NO_MAJOR controls

这里不叫 absolute healthy truth；NO_MAJOR 只意味着旧 frozen oracle 没有 major canonical basin。C12 是 competitive group 数；out12 是其 representative 真正在 T0 center neighborhood 外的数量，不是额外 evidence filter。

| frame | U B4 | U B8 | U B12 | U B16 secondary | C12 | out12 |
|---:|---:|---:|---:|---:|---:|---:|
| 120 | 1.910178 | 1.910178 | 1.910178 | 1.910178 | 1 | 1 |
| 244 | 0 | 0 | .045306 | .045306 | 1 | 0 |
| 740 | 0 | 0 | .105764 | .108533 | 1 | 0 |
| 838 | 0 | 0 | 0 | 0 | 0 | 0 |
| 839 | 0 | 0 | 0 | 0 | 0 | 0 |
| 864 | 0 | 0 | 0 | .026364 | 0 | 0 |
| 924 | 0 | 0 | 0 | 0 | 0 | 0 |
| 925 | 0 | 0 | 0 | 0 | 0 | 0 |
| 1111 | 0 | 0 | 0 | 0 | 0 | 0 |
| 1235 | 0 | 0 | 0 | 0 | 0 | 0 |
| 1359 | 0 | 0 | 0 | 0 | 0 | 0 |
| 1497 | 0 | 0 | 0 | 0 | 0 | 0 |
| 1498 | 0 | 0 | 0 | 0 | 0 | 0 |
| 1556 | 0 | 0 | 0 | 0 | 0 | 0 |
| 1557 | 0 | 0 | 0 | 0 | 0 | 0 |
| 1606 | 0 | 0 | 0 | 0 | 0 | 0 |
| 1730 | 0 | 0 | 0 | 0 | 0 | 0 |
| 1854 | 0 | 0 | 0 | 0 | 0 | 0 |
| 2102 | 0 | 0 | 0 | 0 | 0 | 0 |
| 2598 | 0 | 0 | 0 | 0 | 0 | 0 |
| 3094 | 0 | .249708 | .495888 | .495888 | 4 | 3 |
| 3217 | 0 | 0 | 0 | 0 | 0 | 0 |
| 3631 | 0 | 0 | 0 | 0 | 0 | 0 |

B4/8/12/16 分别有1/2/4/5个 NO_MAJOR frame 存在 competitive groups，总 group 数1/3/7/8。B12 的244/740只有 inside-center competitive representative，应保留其 complete-link 描述限制。

分类结果冻结后才读取 R2A 的 archived `objective_selection.csv` GT annotation，确认两个 genuinely departing/worse controls 为120和3094。其 B12 U=1.910178/.495888，两者 evidence 均非零，属于 potential optimizer ambiguity/risk，不是“应切换的正确 pose”。不重新读取 GT trajectory，不改变 evidence 或 gate。两者 separation 风险并不会因为旧 oracle NO_MAJOR label 而被忽略。

## Directional secondary

`ATTEMPTED = NO`，原因是 PRIMARY formal gate 没有 PASS。没有为该分析打开 U_obs archive，没有计算 principal angles，更没有通过角度调整证据。

## 验证与反证审查

Release build PASS。P9 25/25 tests PASS，包括 math-only P9 chart、non-oracle construction、frame statistics 三项新增自检。额外保持 R1C2 numerical self-tests。证据/CSV/JSON/input/code/library hash 审计 PASS；`git diff --check` PASS。

第一次 CTest 使用本机3.16不支持的 `--test-dir` 因而没有发现测试；不计为测试通过。该命令在仓库 cwd 运行，附带更新了未跟踪的 `Testing/Temporary/LastTest.log`、`CTestCostData.txt`，属于操作偏差，已告知用户；不删除、暂存或提交这些日志。随后所有测试在 `/tmp/p9_r2_build` 运行，暴露两个既有 executable 的 MVS libusb 符号冲突：实际加载 `/opt/MVS/lib/64/libusb-1.0.so.0`。仅为测试进程设置 `LD_LIBRARY_PATH=/lib/x86_64-linux-gnu` 后25/25通过；不改 NDT、实验输入、系统配置或 production。

独立只读构造审查发现原 Python fixture 没覆盖非单位旋转；在 freeze 前加入 spatial-left rotation regression，能拒绝错误 body chart。另加入 complete-link inside-center fixture。第二个独立 reviewer 重算全部 permutation、128个 LOFO thresholds/predictions、128个 nested omission outcomes 和 tensors；无数值差异，empty93行均为零，U 重算最大误差8.9e-16。没有未解决的实质 review finding。用户选择仅当前独立审查，未调用外部 CLI。

复现命令（均不会运行 NDT align）：

```bash
python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/p9_r2b_nonoracle_evidence.py audit
python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/evaluate_r2b_nonoracle_evidence.py audit
```

从空 R2B output 复现时先 `construct`，确认打印冻结 SHA，再 `evaluate`，最后 `finalize`；不可在已有 freeze 上覆盖构造。数学共享库的 Release source/binary hash 随本轮归档。

## 决策边界

保持当前 formal FAIL 和唯一 NEXT，不开始任何 visual experiment。B12 在当前 cohort 的基础 discrimination 值得如实保留，但不足以绕过预声明稳定性证书进入纯 NDT cost reduction。证据描述器不是 measurement covariance，不提供 pose switching authority，也不接 EKF。
