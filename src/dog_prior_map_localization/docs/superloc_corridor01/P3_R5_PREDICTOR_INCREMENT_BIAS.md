# PAPER-P3-R5 targeted closure：预测增量偏差与误差归因

状态：`PAPER-P3-R5-PARTIAL`。本阶段的 frame semantics、递归 constant-motion、bootstrap、事件边界和 cutoff sensitivity 已闭环；但 0–3.068 s 的初始退化不能归因到一个已证实的唯一机制，因此整体机制选择保持 `G — UNRESOLVED`。3.068–5.590 s 的 previous-NDT-delta reuse / insufficient-correction 传播模式有描述性支持，但不等同于物理根因或 unstable feedback。

`P4_ALLOWED = NO`

## 输入与保护

- Frozen baseline：`41999ea700c66c4cadf0eca9e0c5d73caa2783fd`。
- Paper 分支分析起点：`91bf76bb7b65fa794464e548203a1568c346bd53`。
- Derived bag：`corridor01_adapted_full_se3_v2.bag`，SHA-256：`7c52b3703f2f5f9b7fe291e579187c67c0181e015df6a9a8398cf4795b547ba0`。
- Frozen NDT source SHA-256：`40cdf26f9dddb9d06dc16c0bfd89fc88a9dd33e42e5bbd4e9bc3b453f1e4277c`。
- 本阶段未重放 bag，未修改 baseline、paper runtime、predictor、NDT、EKF、limiter、deskew、YAML、地图或 bag。
- 所有大文件和离线 CSV/PNG 均留在结果目录；Git 只允许提交本文档。

结果目录：`/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p3_predictor_bias/`。

## Predictor provenance

`previous_pose_` 是最近一次成功且经过 step-limit 后接受的 `final_used` NDT LiDAR pose。源码没有 `previous_previous_pose_` 成员；解释中的 `T_final(k-2)` 是更新前保存的 accepted pose。

成功处理第 `k-1` 帧后：

```text
delta_pose_(k-1) = inverse(T_final(k-2)) * T_final(k-1)
previous_pose_    = T_final(k-1)
```

下一帧平移/旋转先验为：

```text
T_baseline_guess(k) = T_final(k-1) * delta_pose_(k-1)
p_guess = p_final(k-1) + R_final(k-1) * t_delta
```

这是右乘、body/local increment；不是 world-frame 直接相加，也不是 `delta_pose * previous_pose`。如果 local IMU gyro history 完整覆盖相邻 LiDAR reference stamp，源码只用梯形积分 gyro 替换旋转块：

```text
R_guess = R_final(k-1) * R_gyro
p_guess = p_final(k-1) + R_final(k-1) * t_delta
```

因此有效 IMU prior 时，previous accepted NDT 的平移增量被复用，旋转增量由局部 gyro 替代；没有 accelerometer double integration、EKF velocity 或当前 scan final pose 参与预测。首帧使用 identity/current internal pose，成功后 `delta_pose=I`；点数不足或 NDT 不收敛不会推进历史状态；没有发现进程内 reset/reinitialize 路径。

## Predictor reconstruction gate

使用 previous `final_used`、源码 composition 和 derived-bag gyro history 独立重建 `initial_guess`：

| 项目 | 结果 |
|---|---:|
| NDT rows | 2,776 |
| later-frame reconstruction | 2,775 / 2,775 |
| translation diff mean / P95 / max | `9.37e-9 / 3.03e-8 / 7.32e-8 m` |
| rotation diff mean / P95 / max | `2.30e-8 / 8.11e-8 / 4.92e-6 deg` |
| gate | `PASS` |

五个 synthetic SE(3) composition tests 全部通过。残差处于 CSV 数值精度范围；PCL float 转换发生在下游，不能作为日志 `initial_guess` 残差的直接解释。

## Frame semantics closure

P3-R4 的 `final_rel_*` 与 `gt_rel_*` 是首个有效配对归一化后的 relative-from-start pose。当前 predictor translation 在 previous estimate LiDAR body frame 中。实际离线转换为：

```text
C(k) = transpose(R_gt_rel(k-1)) * R_est_rel(k-1)
t_pred_gt_body = C(k) * t_pred_est_body
e_cross_eval = R_gt_rel(k-1) * e_cross_gt_body
```

这里没有使用 raw absolute map/world `R_GT^T R_final`，也没有假设未确认的 `T_map_GT`。从可用 R4 relative pose 字段独立重建历史输出：2,724 个 predictor-frame 匹配行的转换残差最大 `8.53e-14 m`；2,334 个有限 common-frame cross 行最大 `1.12e-12 m`。因此：

```text
DIRECTIONAL_FRAME_VALID = YES  (emitted-data level)
```

原始 P3-R5 计算脚本未在仓库、结果目录或临时路径中找到，因此上述结论是对已输出字段的独立复核，而不是对原脚本逐行复核；缺失脚本已记录于 `p3_r5_frame_semantics_audit.md`。

## Window and event semantics

时间是配置 evaluation start 后的秒数；首个有效 pose 为 `0.042698145 s`，首个 increment row 为 `0.143558025 s`。增量窗口采用：

| 窗口 | 规则 | increment rows |
|---|---|---:|
| W0 | `(0, 3.0683140754699707)` | 29 |
| W1 | `[3.0683140754699707, 5.589692115783691]` | 26 |
| primary combined | `(0, 5.589692115783691]` | 55 |
| secondary | `(5.589692115783691, 10.329807043075562)` | 46 |

Pose-row summaries分别为 30、26、56、46；这解释了 pose rows 与 increment rows 的差一。`10.329807043 s` fitness-cluster boundary 单独报告，不纳入 secondary summary。另一个 LiDAR-origin evaluator 的 0.5 m first/persistent crossing 是 `5.488831997 s`，所以 primary combined 是“截至正式 task anchor”，不能称作整个 pre-crossing。

事件预算（数值来自 LiDAR-origin relative evaluator；事件名称保留官方 IMU-origin anchor 语义）如下：

| 事件 | previous final | actual prediction | carry-only | oracle increment-only | raw/final NDT |
|---|---:|---:|---:|---:|---:|
| official 0.25 m anchor `3.068314075` | 0.250021 | 0.272093 | 0.261091 | 0.023831 | 0.266069 / 0.266069 |
| official 0.5 m anchor `5.589692116` | 0.500117 | 0.508456 | 0.506294 | 0.044545 | 0.509988 / 0.509988 |
| separate LiDAR 0.5 m crossing `5.488831997` | 0.485595 | 0.513820 | 0.494897 | 0.029115 | 0.500117 / 0.500117 |

完整逐事件表在 `p3_r5_event_budget.csv`。

## Prediction error decomposition

```text
Delta_GT   = inverse(T_GT(k-1))   * T_GT(k)
Delta_pred = inverse(T_final(k-1)) * T_pred(k)
T_oracle   = T_GT(k-1)             * Delta_pred
T_carry    = T_final(k-1)          * Delta_GT
```

`actual prediction`、`oracle-rebased increment-only` 和 `carry-only` 的标量范数不作线性相加；SE(3) 非交换。primary combined mean / P95（m）为 `0.2402/0.4619`、`0.0250/0.0464`、`0.2323/0.4498`；secondary 为 `0.8545/0.9584`、`0.0300/0.0654`、`0.8456/0.9515`。0.5 m anchor 的 actual/carry/oracle 分别为 `0.508456/0.506294/0.044545 m`，说明瞬时误差量级主要已在 previous accepted state 中，但本帧 increment 并非零误差。

## Increment bias and bootstrap

低运动 cutoff 为 `0.0185748338586 m`，来自 doubled-interval leave-one-out interpolation residual P95 `0.0742993354343 m` 按正常间隔/LOO span 的平方比例 `0.25` 缩放。该 dt² 是局部平滑插值误差的诊断假设，不是运行时模型。使用 `0.01/0.0185748338586/0.03 m` 三档重算后，两个 primary 窗口行数和结论完全不变：`LOW_MOTION_THRESHOLD_NOT_RESULT-SENSITIVE = YES`。

`p3_r5_bootstrap_statistics.csv` 使用 paired circular moving-block percentile bootstrap，10,000 replicates；current seeds 为 `20260924–20260927`，previous-final seeds 为 `20261024–20261027`，均写入 CSV。下表给出 current predictor 的 estimate [95% CI]；angle/cosine 为 median，rotation 的 P95 是 descriptive-only。

| 窗口 | along bias (m) | cross x / y / z (m) | cross magnitude (m) | magnitude ratio | direction angle (deg) | direction cosine | rotation mean / median / P95 (deg) |
|---|---|---|---|---|---|---|---|
| W0 | `-0.006342 [-0.013586,0.001915]` | `-0.003248 [-0.008038,0.000584]` / `0.007065 [-0.000742,0.016289]` / `0.000409 [-0.002473,0.003350]` | `0.016528 [0.009432,0.025241]` | `0.960540 [0.862406,0.999469]` | `5.296 [3.023,8.201]` | `0.995731 [0.989773,0.998608]` | `0.869 / 0.850 / 2.170` |
| W1 | `0.001966 [-0.006220,0.010137]` | `-0.004120 [-0.006948,-0.001146]` / `0.009887 [0.002627,0.016484]` / `-0.000963 [-0.002640,0.000698]` | `0.017048 [0.013167,0.021390]` | `0.991777 [0.939505,1.092745]` | `3.774 [2.779,7.792]` | `0.997831 [0.990767,0.998824]` | `0.929 / 0.614 / 2.513` |
| secondary | `-0.003138 [-0.008932,0.002399]` | `-0.002215 [-0.006906,0.000808]` / `0.007069 [-0.002230,0.021478]` / `-0.000746 [-0.003201,0.001953]` | `0.019269 [0.008257,0.036540]` | `0.965120 [0.938879,1.014362]` | `2.388 [1.865,4.758]` | `0.999131 [0.996492,0.999470]` | `0.257 / 0.175 / 0.670` |

W0/W1 signed along CIs cross zero; magnitude-ratio CIs include 1. W1 common-frame x/y CIs exclude zero, while W0 does not; this supports a W1 directional maintenance pattern, not a proven fixed whole-trajectory bias. Secondary has full uncertainty coverage and does not show the same statistically supported x/y direction.

Previous accepted final increment is numerically reused (translation residual about `1e-8 m`; valid gyro prior replaces rotation). In W1, previous-final cross x/y means are `-0.003622/+0.010178 m` with 95% CIs `[-0.005953,-0.001354]` and `[0.004964,0.015149]`; along bias CI crosses zero. In secondary, previous-final x/y CIs cross zero. This supports reuse in W1, not a universal directional bias.

## Recursive constant-motion oracle

The one-step GT oracle remains in the original increment tables. A new recursive diagnostic uses `T_cm(0)=T_GT(0)`, seeds the first observed pose, then recursively applies the previous true GT increment:

```text
T_cm(k) = T_cm(k-1) * inverse(T_GT(k-2)) * T_GT(k-1)
```

This is GT-only and does not use NDT. Results (pose rows, m / deg):

| 窗口 | translation mean / P95 / max (m) | rotation mean / P95 / max (deg) | endpoint recursive / actual prediction |
|---|---|---|---:|
| W0 before 0.25 m | `0.098838/0.161515/0.164652` | `1.467/4.568/5.209` | `0.6694` |
| W1 0.25–0.5 m | `0.134426/0.162758/0.165214` | `2.201/5.017/5.027` | `0.1765` |
| primary through 0.5 m | `0.115361/0.164639/0.165214` | `1.808/5.027/5.209` | `0.1765` |
| secondary before fitness boundary | `0.160676/0.219978/0.225483` | `0.315/1.567/1.759` | `0.2655` |

At the formal 0.25 m anchor, recursive CM error is `0.155390 m`, actual prediction `0.272093 m`, ratio `0.5711`; at the formal 0.5 m anchor, `0.089731 m` versus `0.508456 m`, ratio `0.1765`. Therefore constant-motion mismatch is a material contributor to initial degradation, but it is much smaller than the actual 0.5 m drift and does not explain the main later accumulation. Full rows and event ratios are in `p3_r5_constant_motion_recursive.csv`, `p3_r5_constant_motion_recursive_summary.csv`, and `p3_r5_constant_motion_recursive_events.csv`.

## Onset versus propagation

- `0–3.068 s`：along and cross x/y bootstrap CIs do not establish a stable directional increment bias; recursive constant-motion error is non-negligible. Initial degradation mechanism remains `UNRESOLVED`.
- `3.068–5.590 s`：previous accepted NDT translation delta is numerically reused; W1 previous-final and current predictor cross x/y CIs have matching signs. The mean raw NDT correction is only `0.02356 m` while the required correction is about `0.38 m`; mean prediction-error improvement is `0.00046 m` (median `-0.00011 m`). Final therefore retains the accepted error. This is a descriptive `ERROR_PROPAGATION / MAINTENANCE SUPPORTED` pattern.
- Because the onset and maintenance evidence are different, one E label cannot honestly cover the whole 0–5.59 s interval. Overall mechanism decision is `G — UNRESOLVED`; the W1 pattern remains a candidate maintenance chain only.

## Required scientific answers

1. **Prediction error有多少来自 carry？** At 0.5 m anchor, carry-only `0.506294 m` is close to actual `0.508456 m`, while oracle increment-only is `0.044545 m`; this is a comparison, not an additive percentage.
2. **本帧 increment 是否有系统性偏差？** No stable longitudinal scale bias; W1 has directional x/y evidence, W0 and secondary do not.
3. **偏差是幅值、方向还是旋转耦合？** Directional W1 component is supported; magnitude ratio includes 1; rotation errors are about `0.87/0.93 deg` mean in W0/W1. Coupling is a sensitivity term, not a complete causal attribution.
4. **previous NDT delta 是否复用？** Yes for translation to numerical precision; valid IMU prior replaces rotation. W1 shows matching previous/current cross-track signs.
5. **constant-motion 模型误差多大？** Recursive CM reaches `0.155 m` at the 0.25 m anchor and `0.090 m` at the 0.5 m anchor; it contributes to onset but is not the dominant later drift source.

## Decision and protection

```text
FRAME SEMANTICS: DIRECTIONAL_FRAME_VALID = YES
CONSTANT-MOTION: CONTRIBUTES TO ONSET, NOT DOMINANT AT 0.5 m
MECHANISM: G — UNRESOLVED (W1 propagation/maintenance pattern supported)
PHYSICAL ROOT CAUSE PROVEN: NO
UNSTABLE FEEDBACK PROVEN: NO
DESCRIPTIVE ERROR-PROPAGATION CHAIN: YES, W1 only
NEXT DECISION: NEEDS_MORE_VALIDATION
P4_ALLOWED = NO
```

Runtime algorithm/config/map/bag are unchanged. No P4 algorithm design is authorized by this report.
