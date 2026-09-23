# PAPER-P3-R4：预测—配准—限幅误差分解

状态：`PAPER-P3-R4-PASS`（离线分析通过；不代表定位算法已修复）

机制分类：`C — PREDICTION_DRIFT_WITH_INSUFFICIENT_REGISTRATION_CORRECTION`
`P4_ALLOWED = NO`

本阶段只读取冻结 Run A/B、derived bag、GT/外参和 frozen NDT 源码；没有修改运行时定位逻辑、参数、launch、地图或 bag。所有大体积分析产物留在实验结果目录，没有纳入 Git。

## 输入与坐标语义

- Frozen baseline：`41999ea700c66c4cadf0eca9e0c5d73caa2783fd`。
- Paper workspace 分析基线：`3f00906064d5adee0674400e3007b469b85df815`。
- Derived bag：`/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/derived/corridor01_adapted_full_se3_v2.bag`。
- SHA-256：`7c52b3703f2f5f9b7fe291e579187c67c0181e015df6a9a8398cf4795b547ba0`。
- Baseline Run A：`.../results/baseline_full_se3_deskew_20260923_170936/runA/`；Run B 仅作重复性检查。
- GT 原点为 IMU，主分析按任务要求使用 `T_world_lidar = T_world_imu * T_imu_lidar`，并把 NDT 的 LiDAR-origin 位姿作为估计量。外参逆向往返最大平移残差 `1.42e-14 m`。
- 主评估为 relative-from-start：分别以首个有效配对的 `final_used` NDT LiDAR pose 与对应 GT LiDAR pose 作相对零点；GT 只在有效时间支撑内插值，不外推。时间轴仍以配置起点 `1517157224.188978910` 为零；首个有效配对 `1517157224.231677055` 晚 `0.042698145 s`。
- 为验证坐标变换，另将估计位姿转换为 IMU-origin 并与已闭合的 R3B 结果比对；共同 2,725 帧最大误差差值小于 `4.99e-7 m / 4.99e-7 deg`。因此 R4 的主 LiDAR-origin crossing 数字与历史 R3B IMU-origin anchors 不能不加说明地混为同一误差序列。

## 字段 provenance 与限幅语义

源文件 `/home/jian/livox_ws/dog_visual_loc_ws/src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp` 的 SHA-256 为 `40cdf26f9dddb9d06dc16c0bfd89fc88a9dd33e42e5bbd4e9bc3b453f1e4277c`；paper 副本字节一致。完整逐字段表见外部结果 `p3_r4_log_field_provenance.md`。

- `initial_guess`：在 `handleCloudLocked` 构造并传入 `ndt_.align()` 的 `T_pred`。除首帧初始化外为 `previous_pose_ * delta_pose_`。平移继承上一次已接受 NDT 增量；若 IMU gyro 积分有效，只替换旋转块，平移不变。它不读取 EKF velocity、加速度双积分或 full-SE(3) deskew velocity。
- `raw_ndt`：`ndt_.getFinalTransformation()` 的 PCL 原始优化输出，在 step limiter 之前记录。
- `final_used`：step limiter 处理后的 accepted pose，写回 `previous_pose_`、发布给 ROS。
- `fitness / iterations / converged`：分别是 PCL `getFitnessScore()`、`getFinalNumIteration()`、`hasConverged()`；fitness 不是 covariance 或经标定的不确定度。raw/final pose 仅在 `hasConverged()` 分支记录。
- 限幅开启；有效快照阈值为平移 `0.5 m`、旋转 `5 deg`。实现比较的是 `previous accepted final -> current raw NDT`：平移超限时将单次位移向量截到阈值，旋转超限时从 previous rotation 向 raw rotation 做 Slerp。它既不是 `initial_guess -> raw_ndt` correction 上限，也不是单个 NDT optimizer iteration 的上限。
- 因 PointCloud2 输入回调将内部 scan offsets 设为 0，NDT CSV 的 `scan_start/mid/end` 都退化为 header stamp；motion analysis 另从 deskew cloud 的 point `time` 字段（秒，自 scan start）读取最大 offset，重建 IMU 取样区间。两者未混用。

## 重复性与容差

Run A/B 有 2,776 个共同 NDT 帧；`initial_guess/raw_ndt/final_used` 最大平移及旋转差均为 0。独立 endpoint fresh-registration 检查的 P95 差异是 `0.03868 m / 1.4053 deg`。据此将逐帧 helpful/harmful/neutral 分类容差设为 `0.05 m / 1.5 deg`：高于观测重复波动，又小于本阶段的 `0.25 m` 初期阈值与后续米级漂移。低于容差的修正不称为有益或有害。

## Frozen anchors 与同原点复核

已冻结的官方 IMU-origin persistent anchors（阈值以上至少持续 5 s，帧间隔不超过 0.25 s）如下；fitness 首个 `>10` cluster 是 `10.329807 s`，晚于 0.25/0.5 m 失效信号。

| 阈值 | R3B IMU-origin persistent crossing（配置起点计时） |
|---|---:|
| 0.25 m | 3.068314 s |
| 0.5 m | 5.589692 s |
| 1 m | 34.131304 s |
| 2 m | 34.635604 s |
| 5 m | 35.341563 s |

将相同 persistent-crossing 规则应用到本阶段主分析的 LiDAR-origin 误差后：

| 误差序列 | 0.25 m 首次/持续 | 0.5 m 首次/持续 | 1 m 首次/持续 | 2 m 持续 | 5 m 持续 |
|---|---:|---:|---:|---:|---:|
| prediction | 2.665 / 3.068 s | 5.489 / 5.489 s | 6.598 / 34.030 s | 34.636 s | 35.342 s |
| raw NDT | 2.967 / 2.967 s | 5.489 / 5.489 s | 28.282 / 34.131 s | 34.636 s | 35.342 s |
| final accepted | 2.967 / 2.967 s | 5.489 / 5.489 s | 28.282 / 34.131 s | 34.636 s | 35.342 s |

Raw/final 在 `28.282 s` 首次越过 1 m，但该段不是持续失效；之后回落，持续 1 m crossing 到 `34.131 s` 才出现。图中垂直 anchor 标记明确标作历史 IMU-origin 参考，误差曲线为本阶段 LiDAR-origin 主分析。

## S0–S6 Core Statistics

All segment statistics below are copied directly from the formal `p3_r4_segment_statistics.csv`; values were not estimated from plots. The existing CSV's `start_s`/`end_s` boundaries are used (seconds from configured evaluation start), with S6 extending to the recording end. Translation errors/corrections are in meters; fitness is the PCL score and iterations are counts. Correction columns show median/P95 magnitudes.

**Table A — Error, correction, and NDT diagnostics**

| Segment | Time range (s) | Frames | Prediction error mean/P95 | Raw error mean/P95 | Final error mean/P95 | Raw correction median/P95 | Final correction median/P95 | Needed correction median/P95 | Fitness median/P95 | Iterations median/P95 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| S0 | 0–3.068314075 | 30 | 0.111 / 0.248 | 0.103 / 0.227 | 0.103 / 0.227 | 0.018 / 0.073 | 0.018 / 0.073 | 0.103 / 0.248 | 0.434 / 0.626 | 1 / 2 |
| S1 | 3.068314075–5.589692116 | 26 | 0.380 / 0.503 | 0.380 / 0.496 | 0.380 / 0.496 | 0.021 / 0.053 | 0.021 / 0.053 | 0.378 / 0.503 | 0.374 / 0.519 | 1 / 2 |
| S2 | 5.589692116–10.329807043 | 46 | 0.855 / 0.958 | 0.854 / 0.955 | 0.854 / 0.955 | 0.016 / 0.121 | 0.016 / 0.121 | 0.899 / 0.958 | 0.064 / 0.401 | 1 / 2.75 |
| S3 | 10.329807043–12.000 | 17 | 0.832 / 0.846 | 0.832 / 0.849 | 0.832 / 0.849 | 0.012 / 0.040 | 0.012 / 0.040 | 0.838 / 0.846 | 33.490 / 80.471 | 1 / 2 |
| S4 | 12.000–34.131304166 | 220 | 0.764 / 1.077 | 0.748 / 0.964 | 0.746 / 0.956 | 0.064 / 0.803 | 0.064 / 0.793 | 0.763 / 1.077 | 0.025 / 6.478 | 2 / 15 |
| S5 | 34.131304166–35.341563 | 12 | 3.043 / 5.163 | 2.978 / 5.163 | 2.965 / 5.146 | 0.982 / 1.083 | 0.953 / 0.995 | 2.682 / 5.163 | 0.056 / 0.158 | 13.5 / 15.45 |
| S6 | >35.341563 | 2374 | 57.852 / 151.112 | 57.849 / 151.116 | 57.849 / 151.116 | 0.076 / 0.911 | 0.073 / 0.696 | 52.686 / 151.112 | 0.112 / 68.820 | 2 / 19 |

**Table B — Correction classes and limiter activity**

Fractions are percentages of frames in the segment; helpful/harmful/beneficial classifications use the previously documented `0.05 m / 1.5 deg` neutrality tolerance. The class and translation-limited columns below are the translation-error classes/flag from `p3_r4_segment_statistics.csv`; rotation-limited activity is a separate flag fraction. Rotation-error limiter effects are inspected separately from the per-frame CSV and are not folded into these translation classes.

| Segment | RAW helpful | RAW harmful | FINAL helpful | FINAL harmful | Limiter beneficial | Limiter harmful | Translation limited | Rotation limited |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| S0 | 6.7% | 3.3% | 6.7% | 3.3% | 0.0% | 0.0% | 0.0% | 6.7% |
| S1 | 3.8% | 0.0% | 3.8% | 0.0% | 0.0% | 0.0% | 0.0% | 15.4% |
| S2 | 4.3% | 4.3% | 4.3% | 4.3% | 0.0% | 0.0% | 0.0% | 2.2% |
| S3 | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% |
| S4 | 23.6% | 20.9% | 24.1% | 20.0% | 1.8% | 0.0% | 3.6% | 8.6% |
| S5 | 33.3% | 25.0% | 50.0% | 8.3% | 0.0% | 0.0% | 83.3% | 0.0% |
| S6 | 10.1% | 8.6% | 9.6% | 8.2% | 1.8% | 2.1% | 10.0% | 11.2% |

**Broad windows A/B/C**

Errors are mean / sample std / P95, in meters. These windows are retained as the requested coarse comparison; they are separate from S0–S6.

| 区间 | 帧数 | Prediction | Raw NDT | Final used | T-limit / R-limit |
|---|---:|---:|---:|---:|---:|
| A：0–10 s | 99 | 0.505 / 0.343 / 0.950 | 0.502 / 0.343 / 0.951 | 0.502 / 0.343 / 0.951 | 0 / 7.1% |
| B：10–16 s | 60 | 0.796 / 0.036 / 0.850 | 0.795 / 0.036 / 0.850 | 0.795 / 0.036 / 0.850 | 0 / 0 |
| C：16–30 s | 139 | 0.771 / 0.125 / 1.006 | 0.765 / 0.102 / 0.982 | 0.765 / 0.102 / 0.982 | 0 / 2.9% |

关键窗口：

- `0.5 m` 持续 crossing 帧（`5.589692116 s`）：prediction/raw/final translation error 为 `0.508456/0.509988/0.509988 m`；needed/raw/final correction 为 `0.508456/0.008760/0.008760 m`；`translation_limited=false`、`rotation_limited=false`，raw/final 均为 neutral。误差在进入 NDT 优化前已存在，不是 raw NDT 从接近正确的 prediction 主动拉偏。
- `4–7 s`（0.5 m crossing 前后）：prediction/raw/final 均值 `0.555/0.556/0.556 m`；needed correction 均值 `0.555 m`，raw/final correction 仅 `0.048 m`，平均误差改善 `-0.0009 m`，无平移限幅。旋转限幅为 `3/29` 帧，旋转效果 1 beneficial / 0 harmful / 28 neutral。
- `9–12 s`（fitness 首个 >10 cluster）：first cluster 在 `10.329807 s`；该帧 prediction/raw/final error 为 `0.855266/0.849759/0.849759 m`，raw correction `0.008022 m`，limiter 两方向都未触发。prediction error 在 fitness cluster 之前已超过 `0.5 m` persistent threshold，因此该 cluster 分类为 `POST-DEGRADATION symptom`，不是 early predictor。整个 9–12 s 窗口 prediction/raw/final 均值约 `0.850/0.850/0.850 m`，修正均值 `0.014 m`。
- `33–36 s`：prediction/raw/final 均值 `3.059/2.972/2.952 m`，P95 `8.114/8.105/8.102 m`。needed/raw/final correction 均值 `3.059/0.871/0.832 m`。平移限幅 `19/30` 帧，旋转限幅 `3/30` 帧。

## 33–36 s 逐帧重点

- 最大 prediction 平移 increment 是 `0.500 m/frame`（多个帧到达此值）；最大 raw NDT increment 为 `0.815 m`，发生在 `33.123 s`；final increment 被截为 `0.500 m`。
- 同一帧 raw→final 平移差最大 `0.315 m`、旋转差 `2.543 deg`。平移误差 prediction/raw/final 为 `0.631/0.735/0.548 m`，且 `translation_limited=true`、`rotation_limited=true`，所以 limiter 改善了该帧平移；但旋转误差 raw/final 为 `1.501/3.141 deg`，该旋转限制反而使方向误差增大。故这不是“整个位姿全面变好”的例子。
- 在 33–36 s 的 30 帧中，按平移误差分类，raw helpful/harmful/neutral 为 `11/10/9`；final 为 `14/6/10`；translation limiter 为 `4 beneficial / 0 harmful / 26 neutral`（按 `0.05 m / 1.5 deg` 容差）。旋转 limiter flag 为 `3/30`，其中旋转误差分类有 `1` 帧 harmful、`29` 帧 neutral；因此需区分“平移保护”与“旋转变差”，不能把它们合并成整个位姿统一改善。尽管平移 flag 高频触发，仍没有证据说 limiter 主导造成整体错误。
- 单帧 limiter 保护案例（1 m anchor，相对时间 `34.131304 s`，sensor timestamp `1517157258.320283 s`）：prediction/raw/final 平移误差 `1.226/1.290/1.225 m`；`translation_limited=true`、`rotation_limited=false`；raw→final 平移改变量 `0.134 m`，旋转未被 limiter 修改。该帧 raw 平移误差比 final 大，支持“limiter can be beneficial on individual frames”，但不能据此判断 limiter 整体应保留或修改。2 m anchor：`2.388/2.132/2.132 m`，raw 有益且 raw→final 改动很小；5 m anchor：`5.443/5.501/5.476 m`，raw 略有害，final 相对 raw 的差异低于分类容差。
- Increment：33–36 s GT 位移均值 `0.237 m/frame`；prediction increment 均值 `0.463 m`，中位数/P95 `0.500/0.500 m`。prediction/GT increment 平移比中位数 `1.923`，方向 cosine 中位数 `0.002`。日志的 `previous_pose * delta_pose` 平移重构残差最大 `6.97e-8 m`，只证明 predictor 与日志字段代数一致，不证明预测运动符合 GT。
- 最终误差在此窗口主要沿 trajectory tangent/body-X 累积：along-track 均值 `-2.770 m`，末值 `-8.995 m`；horizontal cross-track 均值 `0.476 m`；vertical 误差绝对值均值 `0.215 m` 且正负变化。此处不是“corridor direction”的结论。

## Motion diagnostics

4–7 s、9–12 s、33–36 s 均未检出 high-acceleration 或 possible-collision flags；33–36 s 有 `2/30` high-angular flags。晚期估计 CV speed 均值/P95 `4.583/4.953 m/s`，scan translation deskew 均值/P95 `0.462/0.4995 m`，与误差突增时间相邻。因为 CV speed 来自近期 NDT 位姿，这只是 `MOTION_EVENT_TEMPORALLY_ASSOCIATED`，可能与定位漂移耦合；`CAUSAL CLAIM = NO`。

## 五个问题与结论

1. **最早误差增长在哪一级？** Prediction 首次瞬时越过 0.25 m 在 `2.665 s`；但按持续定义，raw/final 的 0.25 m crossing 比 prediction 早 `0.101 s`，0.5 m 持续 crossing 三者都是 `5.489 s`。因此不能宣称 prediction 对持久阈值有明确时间领先。可是到 0.5 m 时 prediction 已有 `0.508 m` 误差，raw correction 只有 `0.0088 m`，limiter 未触发；后续早期误差不主要由 limiter 引入。
2. **NDT raw 通常帮助还是伤害？** 4–7 s 有 24/29 neutral、3 helpful、2 harmful；9–12 s 30/30 neutral。33–36 s 则 help/harm 接近（11/10），所以 raw 不是一致的修复源或主要破坏源。
3. **limiter 通常帮助还是伤害？** 平移效果总体以 neutral 为主，33–36 s 按平移误差是 4 beneficial、0 harmful、26 neutral；旋转 limiter 有 1/30 帧使旋转误差变差。`34.131304 s` 是平移方向的明确保护案例；整体证据不满足 `STEP_LIMIT_CONTRIBUTES_STRONGLY` 的方向/最终误差/持续主导证据链。
4. **为何 0.5 m 后约 28 s 没有持续超过 1 m？** 误差大部分时间保持在约 `0.7–0.9 m` 带内；12–34 s final mean/P95 为 `0.746/0.956 m`。28.282 s 有过一次瞬时 >1 m，但之后回落，持续 crossing 到34.131 s。
5. **为何 34.13–35.34 s 从 1 m 快速到 5 m？** 两个 persistent anchors 相隔 `1.210259 s`。已观测到预测每帧增量接近 0.5 m 上限而 GT 平均每帧位移约 0.237 m，增量方向 cosine 中位数近 0；NDT raw 修正大但方向效果混合，未稳定补足 trajectory-tangent 方向的 needed correction。limiter 多次触发但多数中性或有益。该时间关联分解不证明具体物理原因。

**Mechanism decision：`C — PREDICTION_DRIFT_WITH_INSUFFICIENT_REGISTRATION_CORRECTION`。** 这是四个位姿关系支持的主导描述：持续误差已存在于 prediction；NDT raw 常只做厘米级或方向不稳定的修正，不能有效消除已有误差。并非证明错误 mode、退化、Hessian 问题或 limiter bug。

**Limiter overall role：`MOSTLY_NEUTRAL` for translation error.** 方向分量需要保留：部分旋转限幅帧对旋转误差有害；当前没有证据表明其成为整体 failure 的主导因素。

## Observed failure chain

`prediction` 已携带相当一部分相对误差；在 S1–S3（0.25 m 持续 crossing 至 fitness cluster 后的早期阶段），raw NDT correction 的中位数约 `0.012–0.021 m`，而 needed correction 中位数约 `0.378–0.899 m`，translation limiter 在这些分段均未触发；因此 `raw NDT` 通常没有把 prediction 拉回 GT，`final` 多数与 raw 基本相同。晚期 S5/S6 行为更复杂：raw 有 helpful 和 harmful 帧，limiter 既有中性也有少量正/负影响；当前数据不支持把晚期行为描述为早期链路的简单重复，也不支持认定有清晰的第二种主导机制。总体最符合当前证据的链路是：`prediction drift → early raw NDT correction insufficient → limiter mostly neutral (occasionally protective) → final retains much of the accumulated error`。

早期机制检查：S1 的 raw correction median/P95 为 `0.021/0.053 m`，needed correction 为 `0.378/0.503 m`；S2 分别为 `0.016/0.121 m` 与 `0.899/0.958 m`；S3 分别为 `0.012/0.040 m` 与 `0.838/0.846 m`。结合 S1–S3 raw helpful 占比 `3.8%/4.3%/0%`、translation limit 占比均为 `0%`，数据支持 prediction 已偏且 raw correction 通常不足，而不是从正确 prediction 出发后 raw NDT 主动拉偏。

晚期阶段检查：S5 的 raw helpful/harmful 占比为 `33.3%/25.0%`，final 为 `50.0%/8.3%`，translation limiter 触发 `83.3%`；S6 raw/final helpful/harmful 分别为 `10.1%/8.6%` 与 `9.6%/8.2%`，limiter 有害占比 `2.1%`。这是 mixed late-stage behavior，但没有显示 registration 突然成为一致的主动致错源，也没有满足将整个失效升级为 `MULTI_STAGE_FAILURE` 的清晰阶段机制切换条件。机制 C 仍然是最符合当前证据的描述，且不代表物理根因已被证明。

## NEXT SCIENTIFIC QUESTION

Does the previous_pose × delta_pose translation predictor exhibit a systematic directional increment bias before the 0.5 m persistent error crossing that explains the accumulated prediction drift?

中文：在 0.5 m 持续误差出现之前，`previous_pose × delta_pose` 的平移预测是否已经存在可重复的方向性增量偏差，并由此解释 prediction drift 的累积？本阶段不回答如何修改 predictor。

## 产物与保护状态

完整 CSV/PNG/provenance/mechanism summary 位于：`/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p3_prediction_correction/`。其中 CSV、PNG、bag 均不进入 Git。

唯一本阶段文档：`P3_R4_PREDICTION_CORRECTION_ANALYSIS.md`。没有进入 mode search/Hessian/P4、没有修改 frozen baseline 或 paper NDT/EKF/OOSM/deskew/config/map/bag。原有 staged/dirty 文件均保留。
