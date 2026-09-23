# PAPER-P2B-R1 evaluator audit

## Scope

本阶段只审计离线评价和输入确定性，不修改 frozen baseline、NDT、EKF、OOSM 或参数。正式评估使用：

- derived input：`derived/corridor01_adapted_v1.bag`
- Run C：`results/frozen_baseline_derived_runC_20260923/result.bag`
- NDT diagnostic：Run C `ndt_determinism.csv`
- GT：`gt/corridor01_gt.txt`
- evaluator：`/home/jian/livox_ws/superloc_adapter_ws/src/superloc_corridor_adapter/scripts/evaluate_p2b_r1.py`

evaluator SHA-256：`fa18ebfc55273af0f6a1c0203f2d199e9b1b3b78fa21256e5bc23c7fe947de89`

评估起点为首个 NDT sensor stamp `1517157219.188979 + 5 s = 1517157224.188979`。估计位姿使用与旧 evaluator 相同的 Corridor01 `laser_to_imu` 固定外参转换；GT 只在离线评价阶段使用。

## Evaluator implementation

`evaluate_corridor01.py` 的 SE(3) 对齐逐行审计确认：

```text
p_aligned = R_align * p_est + t_align
R_aligned = R_align * R_est
scale = 1
```

人工测试保存在 `evaluator_unit_tests.txt`，结果为 `SE3_POSITION_PASS`、`SE3_ORIENTATION_PASS`、`NO_SCALE_PASS` 和 `FULL_ALIGNMENT_CONTAMINATION_PASS`。因此旧 evaluator 没有“只对平移对齐、旋转仍使用未对齐姿态”的 bug；旧的 full-alignment 指标没有因该类实现错误而失效。新增 evaluator 使用相同变换语义，并增加 FIRST_POSE、PREFIX_10S、PREFIX_30S 和 RAW。

## Run C alignment metrics

下面首先列 NDT（2725 samples，评估窗口从 +5 s 开始）。平移单位 m，旋转单位 deg。

| alignment | t mean | t RMSE | t median | t P95 | t max | r mean | r median | r P95 | r max |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| RAW | 100.9911 | 116.3891 | 104.4122 | 193.9074 | 195.8884 | 101.9383 | 113.1598 | 178.3695 | 179.9845 |
| FIRST_POSE | 87.2433 | 105.5530 | 89.3986 | 180.2948 | 182.7613 | 91.6430 | 104.0822 | 179.8513 | 179.9996 |
| PREFIX_10S | 88.4920 | 106.8062 | 91.2985 | 182.3346 | 184.6813 | 93.0907 | 105.5486 | 179.9063 | 179.9989 |
| PREFIX_30S | 88.3721 | 106.9247 | 91.4443 | 182.3853 | 184.8152 | 93.0629 | 104.7359 | 179.8978 | 179.9991 |
| FULL_SE3 | 62.9253 | 70.9838 | 51.1897 | 129.4261 | 137.9524 | 109.9604 | 96.1012 | 177.0828 | 179.8780 |

EKF corrected 的 2708-sample 指标和 NDT 数值接近，完整数值保存在 `eval_r1/alignment_metrics.json`。每个 NDT timestamp 的误差、fitness、迭代次数和 convergence 保存在 `eval_r1/failure_onset_alignment_comparison.csv`，图为 `eval_r1/translation_error_alignment_comparison.png`。

## Why full alignment is not used for onset

`FULL_SE3` 被后半段大幅错误轨迹污染，不能用于判断第一次失效。`PREFIX_10S` 的固定对齐在评估开始后的前 10 s 内为：translation mean `0.1424 m`、P95 `0.2929 m`、max `0.3490 m`；这提供了可信的短时 tracking 锚点。相同轨迹用 PREFIX_30S 对齐时，前 10 s 已为 mean `1.4995 m`，说明更长的 prefix 已受到后续轨迹/坐标语义污染。

## Old Smoke vs old Full A

在旧 live-adapter Smoke 与旧 Full A 的共同前 30 s 内，使用相同 sensor-time 窗口且不与 GT 比较：

- NDT：297 common samples；position mean/P95/max = `4.55497/7.92405/8.02462 m`；rotation mean/P95/max = `8.36528/23.42105/25.57758 deg`。
- EKF：采用 nearest timestamp，阈值 `0.03 s`，285 pairs，最大时间差 `0.02999997 s`；position mean/P95/max = `4.69562/7.93502/8.11370 m`；rotation mean/P95/max = `7.75869/22.64234/24.93500 deg`。

结论是旧 Smoke 与旧 Full 前 30 s 并不一致，不能把旧 Smoke 的结果直接外推为 Full A 的起始行为；这正是固定 derived input 的必要性。
