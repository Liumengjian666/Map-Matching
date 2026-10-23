# 三策略合同与运行边界

三策略从 R16 ordinal 7552 的同一 `T_M_L` 锚点开始，在 7553–7582 的同一冻结 PCD 和 Leica ROI 上各自维护 R14B 右乘恒速度预测器。每帧 `TP` 只由该策略过去正式接受的 Full 位姿构造；拒绝后将当前 `TP` 作为该帧安全回退输出，并在下一帧停止外推、保持最近一次已接受位姿。任何评价帧参考值均未进入预测、NDT 或决策。三策略因果状态从 raw7571 起确实分歧。

- **BASELINE_5DEG**：R10 `safeNative`、收敛、Coverage V2、Overlap≥0.50、与 `TP` 平移≤0.8m、旋转≤5°时接纳 Full。
- **RELAXED_6DEG**：仅将该旋转界限改为≤6°；不运行 Split。
- **EVENT_CONSENSUS**：正常时完全沿用 5°基线。仅当 Full 其余门全部通过且相对 `TP` 旋转在 (5°,6°] 时，额外运行一次同帧、同 `TP`、同地图、Training Source 的原生 Split NDT。Split 必须有限、收敛、Coverage有效且 Overlap≥0.50；Full–Split 平移≤0.10m且旋转≤1°时，条件性接纳 Full，否则回退。预算最多3次额外 Split；耗尽后拒绝。

这种接纳是**5°规则的明确例外**，并非原正式5°合同下的合法观测。6°由已知 R16 案例决定，该窗口是开发集。Full 和 Split 共享大部分点云结构、目标地图、初始化与优化器；其一致性不构成正确性证明。

先用 R16 冻结 Full/Split 和共同 `TP` 进行不含 GT 的静态规则回放，文件 `STATIC_RULE_REPLAY.csv` 明标 `STATIC_COMMON_TP_NOT_CAUSAL_TRAJECTORY`。随后仅一次 Release 实验，三策略各运行30次原生 Full；一致性策略仅在符合门时运行额外 Split。三臂顺序执行，共用地图和 Source，不共享优化结果，因此实际 Full 调用90次。原生结果、可追溯源码和二进制身份写入 `NATIVE_OUTPUT_FREEZE.json` 后，才读取评价参考。

原始 `native_run_01/ADMISSION_DECISIONS.csv` 中的 `confirmation_requested` 实为“Full 满足有界旋转事件谓词”；三策略都记录该谓词。实际是否发起 Split 以 `split_executed` 和 `ALIGN_CALLS.csv` 为准，基线和6°臂均没有发起 Split。该字段名在执行源码中不够准确，已在此披露，未事后改写冻结原生输出。
