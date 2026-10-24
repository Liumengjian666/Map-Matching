# Lightweight Prior-Map NDT Localization with Geometry-Aware Admission and Budgeted Recovery

*Complete first manuscript draft for scientific review. Author and affiliation information will be supplied by the research team. The empirical scope and blocked robot deployment are stated explicitly; this document is not a claim of submission readiness.*

## Abstract

Prior-map localization must balance registration reliability, inertial continuity, and computational cost. A practical difficulty is that a pre-optimization geometry check evaluates the current prediction rather than the pose that an optimizer could recover. Consequently, rejecting a poorly aligned scan before registration can prevent a useful map correction. We study a lightweight system combining native Normal Distributions Transform (NDT), local map targets, real inertial propagation, post-registration coverage consistency, and a bounded two-start recovery procedure. The optimizer, filter model, and quality thresholds remain fixed. A controlled causal ablation on Newer College shows that bypassing the pre-optimization veto alone is insufficient: all 60 nominal registrations converge, but none passes the frozen post-registration quality gate, and the inertial trajectory continues to drift. Allowing the same scans to reach a fixed yaw-restart procedure reduces complete-window translation and rotation RMSE from 7.986 m and 13.053 deg to 0.210 m and 2.984 deg, using eight additional registrations. An earlier matched-segment map study records an 82.2% reduction in NDT-process high-water memory, with latency and maximum-error tradeoffs. We also preserve a failed relative-admission generalization test and an unconsumed OOSM-history synchronization attempt. The results support a narrowly defined engineering mechanism, not a new optimizer, universal recovery guarantee, or independent physical accuracy claim. Evaluation uses a conditionally interpreted, map-anchored benchmark reference; quadruped deployment remains blocked by an unverified LiDAR–IMU calibration chain.

**Keywords:** prior-map localization; normal distributions transform; inertial propagation; observation admission; bounded recovery; local maps.

## 1. Introduction

A robot operating in a previously mapped environment can avoid the cost of continuous map reconstruction by registering current LiDAR scans against a fixed prior. NDT is attractive for this setting because it represents target geometry statistically and estimates a rigid alignment from an initial pose. Its practical behavior nevertheless depends on initialization, map support, and the surrounding state-estimation architecture. Native convergence is an optimizer status, not a certificate that a pose is accurate. The original NDT formulation and its 3D development provide the registration foundation; our implementation retains the existing PCL solver rather than modifying its objective or derivatives. [1](https://www.diva-portal.org/smash/get/diva2:276162/FULLTEXT02.pdf), [2](https://pointclouds.org/documentation/tutorials/normal_distributions_transform.html)

A deployed prior-map system must make several different decisions. It must determine whether required map data are available, whether registration may be attempted, whether the resulting map observation is acceptable, and how the inertial estimator proceeds when an observation is rejected. These questions are related but not interchangeable. In particular, a nearest-map geometry statistic evaluated at a poor prediction may reject a scan that could support a useful alignment after optimization. Conversely, simply permitting registration does not guarantee that its output satisfies an unchanged observation-quality rule.

This paper investigates that distinction through a reproducible system study. The retained configuration combines an existing local-target manager, native Full NDT, causal IMU–EKF prediction, coverage-based post-registration admission, and a fixed event-triggered yaw-restart baseline. We call the removal of the *optimization veto* G and the bounded recovery procedure C. G preserves the original geometry thresholds as diagnostics and keeps all post-registration checks. It neither accepts an observation directly nor lowers a quality threshold.

Our principal contribution is empirical and implementation-oriented. First, we identify a concrete feedback failure in which a pre-optimization geometry veto prevents useful map corrections. Second, a new independently executed causal ablation on an exposed development window separates optimizer access from recovery: G-only produces 60 converged registrations but no accepted observation, whereas C+G admits a restart result and changes the subsequent fused trajectory. Third, we document resource behavior and failure boundaries, including a normal-initialization safety counterexample for relative-only admission. We do not claim that multistart search, threshold gating, map tiling, or degeneracy detection is new mathematics. The narrower claim is that their placement, budget, and interaction with a real filter must be evaluated causally rather than inferred from isolated registration scores.

## 2. Related Work

### 2.1. Registration and localizability

X-ICP analyzes correspondence-supported alignment strength along principal directions and incorporates localizability into constrained ICP updates. GenZ-ICP instead adapts the balance between point-to-point and point-to-plane residuals to improve behavior under changing geometric conditions. These approaches explicitly alter or constrain registration. Our retained method does not modify an optimizer, Hessian, or weak-direction update; it controls access to an existing NDT solver and admits its outputs using fixed quality rules. We provide no head-to-head performance claim against either system. [3](https://arxiv.org/abs/2211.16335), [4](https://arxiv.org/abs/2411.06766)

SuperLoc studies predictive alignment risk before optimization in map-based LiDAR localization. Its work motivates careful assessment of information available before matching. Our experiment addresses a narrower issue: whether one particular hard geometry veto prevents a useful optimization-and-admission sequence. The observed failure does not imply that pre-optimization risk assessment is inherently unsuitable, or that our baseline is superior to SuperLoc. [5](https://arxiv.org/abs/2412.02901)

### 2.2. Inertial localization and implementation scope

FAST-LIO2 is an established example of direct LiDAR–inertial odometry using raw-point registration and an efficient filtering architecture. We target localization against a fixed map and reuse an existing custom IMU/EKF implementation. We do not identify this custom filter as an unmodified FAST-LIO2 system. Likewise, the FAST-LIVO2-derived map available for our robot assets is not an independent reference trajectory. [6](https://arxiv.org/abs/2107.06829)

Spatially restricting a target, event-triggering expensive work, and evaluating alternative initializations are reusable engineering ideas. Their value here must therefore be demonstrated through measured memory, failure recovery, and a controlled ablation. An isolated successful candidate, or a formula with a new name, is insufficient evidence of either scientific originality or system reliability.

## 3. Methodology

### 3.1. Frames, propagation, and measurement time

We use T_A_B to denote a transform mapping coordinates from frame B into frame A. The internal filter state refers to the IMU, while the registration observation is T_M_L, the LiDAR pose in the map. With the fixed sensor transform T_I_L,

$$T_{M,L}=T_{M,I}T_{I,L},\qquad T_{M,I}=T_{M,L}T_{I,L}^{-1}.$$

Newer College uses the previously frozen extrinsic chain and conditional Base interpretation. Each trajectory is initialized once from its anchor reference. The C3 condition rotates the anchor LiDAR attitude by +8 deg about map Z, keeps its position fixed, and recomputes the complete IMU pose including the lever arm. Initial velocity and biases are inherited zero priors, not measured zero motion or independently estimated biases.

Subsequent NDT initial poses come from actual causal IMU–EKF output. The node selects a delivered state at or before the scan reference and uses the existing bounded body-twist interface for at most 20 ms of projection. It does not read current or future reference poses. All compared methods run independent state histories. Equal raw inputs do not force equal deskewed points after histories diverge; that dependence is part of the closed loop.

The Newer College adapter defines the scan header as scan start and preserves point-relative times. Existing IMU-based Deskew also references scan start. NDT observations retain this measurement time even when computation finishes later. The original OOSM path applies a historical update within its 20 ms alignment tolerance and replays real IMU samples to the current state. Errors for the fused output are evaluated at that output's actual current-state time, approximately 100 ms after scan start, rather than against an earlier scan reference. No propagation, rollback, replay, or covariance model is redesigned.

### 3.2. Local target construction

The P45 manager starts from a frozen target obtained by two global 0.15 m voxel passes and partitions it into 20 m horizontal tiles. For each scan it projects the preprocessed source using the same causal prediction supplied to NDT, requests intersecting tiles with a 10 m safety margin, and uses an expanded 30 m selection at updates to provide hysteresis. If the active set covers the requirement, the merged target and target model are reused. The current registration result and evaluation reference do not select that frame's map. Missing or corrupted required map assets remain hard failures even when G is enabled.

The live NDT process does not load the entire visualization map. The optional RViz map publisher is separate and can dominate total display memory. Actual target point counts, tile IDs, and fingerprints are logged because different causal trajectories could select different maps. In the principal W1/C3 ablation, all three conditions retain the same 2,297,615-point target fingerprint, eliminating a target-content explanation for the observed difference.

### 3.3. Pre-optimization geometry diagnostics

At prediction T_P, the existing geometry check computes nearest-map distances for the preprocessed source. Its original predicate is

$$\operatorname{median}(d)\le1.0\,\mathrm m,\quad P_{90}(d)\le3.0\,\mathrm m,\quad O_{0.5}(T_P)\ge0.50.$$

The original configuration returns before NDT if any condition fails. G records exactly the same metrics and decision but permits native registration to proceed. G does not declare the prediction accurate or force acceptance of an optimized pose. It changes which component is responsible for judging an observation: initial geometry becomes diagnostic, while post-registration quality remains authoritative. No geometry threshold is relaxed.

### 3.4. Native NDT and coverage consistency

Full Source preprocessing remains 0.25 m voxelization, 0.5–80 m range filtering, and a deterministic maximum of 1,400 points. The unchanged PCL NDT uses resolution 0.8 m, step 0.08, transformation epsilon 0.001, 40 maximum iterations, and outlier ratio 0.55. Every selected seed receives one native alignment; no post hoc candidate is presented as a converged native result.

The frozen Coverage V2 code divides scanner-frame azimuth into 16 sectors. Eight odd sectors define potential quality blocks. A block is active if its input count is at least 10; at least four active blocks and 300 even-sector points are required. Activity is fixed before optimizing candidates. For active block b,

$$L_b(T)=\frac{1}{|S_b|}\sum_{p\in S_b}\min\left(\frac{d(Tp,\mathcal M)^2}{(0.5\,\mathrm m)^2},1\right),\qquad Q(T)=\frac{1}{B}\sum_{b\in\mathcal A}L_b(T).$$

Overlap is the fraction of active-block points within 0.5 m of the target. Missing blocks are unavailable, not zero-loss observations. A candidate must be a valid finite native result, converge, have evaluable coverage, and satisfy overlap at least 0.50. The retained absolute quality gate additionally requires Q at most 0.30.

These blocks participate in Full NDT optimization. Q is therefore **Coverage Consistency Mean**, not independent holdout evidence. It is a deterministic spatial quality diagnostic, not a calibrated probability of correctness. The 0.30 threshold was developed on the earlier R19 observations and is not presented as universal.

### 3.5. Event-triggered bounded recovery

C first evaluates one nominal Full NDT result from T_P. If it passes quality, it is admitted without extra search. Otherwise, with remaining budget, C constructs two seeds:

$$R_{\pm}=R_z(\pm5^\circ)R_P,\qquad p_{\pm}=p_P.$$

Only attitude is changed; left-multiplying the entire transform and rotating its translation would implement a different operation. Each seed is optimized once using the same Full Source and target. Every resulting candidate must pass the same basic and absolute quality gates.

Among accepted candidates C selects the highest native transformation-probability score. An exact tie is resolved by smaller dimensionless prediction distance, then smaller fixed candidate ID. The distance uses translation normalized by 0.8 m and rotation by 5 deg. In the actual retained r20::accepted implementation these are tie-break scales, not hard candidate-to-prediction admission bounds. A separate inherited publication limiter bounds successive NDT observation steps by 0.5 m/5 deg, and the existing filter has its own update limits. Raw, selected, and published poses are logged separately.

Each event permits at most two extra calls; each approved 60-scan run permits 12 extras. This is a hard experimental per-run quota, not an evaluated indefinite sliding-budget scheduler. If no candidate passes, no map measurement is published and the existing IMU estimator continues. Prediction is never relabeled as an independent NDT observation. Relative-only admission, temporal confirmation, coupled refinement, and the robust paired selector are excluded from the retained method.

**Algorithm 1 — frozen C+G measurement path**

```text
used_extra = 0                         # quota belongs to this approved run
IMU propagation runs through the existing filter independently
for each scan in chronological order:
    TP = existing causal IMU/EKF prediction at scan reference
    target = existing map manager selection from source and TP
    if required map assets are invalid: publish no map measurement; continue
    record original pre-geometry decision                 # G does not veto
    candidates = [native Full NDT(source, target, TP)]
    if nominal fails unchanged quality and used_extra + 2 <= 12:
        add one Full alignment from each fixed map-yaw +/-5deg seed
        used_extra += 2
    select only candidates passing the unchanged absolute quality gate
    if none pass: publish no map measurement
    else: use the existing observation-limiter/publication/OOSM path
```

G-only executes the same path but never enters the extra-alignment branch.

### 3.6. G-only ablation

The new G-only condition enables optimization despite the same pre-gate failure but disables all extra search. It retains the identical post-quality predicate and selects only an eligible nominal result. It runs a new independent 60-frame causal state sequence. This separates “NDT was allowed to run” from “an additional initialization supplied an admissible map observation.” Historical original and C+G trajectories are cited without rerunning or reordering their per-frame outputs.

## 4. Experimental Setup

### 4.1. Data and reference limitations

Newer College `01_short_experiment` provides actual Ouster scans, inertial measurements, an independently acquired Leica prior map, and an official reference trajectory. The reference trajectory itself is generated by scan-to-map registration; it is not independent motion-capture truth. [7](https://arxiv.org/abs/2003.05691), [8](https://ori-drs.github.io/newer-college-dataset/ground-truth/)

The archived Base interpretation is supported by the dataset paper and pinned robot-model evidence, but a direct binding to the local CSV export convention remains unresolved. The official documentation also distinguishes stereo and multicamera reference frames. We retain the single previously approved interpretation, do not select axes by matching error, and report **reference-relative pose error** throughout. The map is independently surveyed; the trajectory reference is map anchored. These two independence claims must not be conflated. [8](https://ori-drs.github.io/newer-college-dataset/ground-truth/), [9](https://github.com/ori-drs/frontier_description/blob/0b039393334a7767e24a711bd05816bfc7f7485a/README.md)

Table 1 separates protocols that must not be pooled as one experiment.

|Cohort|Input and estimator|Scope|
|---|---|---|
|P45 resource study|Matched 300-scan segment; IMU/EKF and local target|Historical spatial-memory comparison; no live tile crossing|
|R19 basin test|Two windows, four initializations, 60 scans each; optimizer-only causal predictor|480 native outputs plus 6 stability checks|
|R20–R21 development|Same two windows, C0/C3; causal quality/recovery policies|Development evidence, not new holdout|
|R22 holdout|Four specified new windows; four policies and C0/C3|32 independent 60-frame causal trajectories; no IMU/EKF|
|R23–R24 closure|W1 anchor 2600 and W3 anchor 6500,60 tests each; true IMU/EKF|Known development windows with independent runs|
|R25 G-only|W1/C3, raw 2601–2660; true IMU/EKF|One new 60-frame ablation, zero extra calls|

R19/R20 use frozen 50 m static ROIs and a historical causal pose predictor, whereas the integrated experiments use real IMU/EKF, actual Deskew, and P45 target selection. Accuracy changes across these cohorts cannot be attributed to one algorithmic component.

### 4.2. Evaluation and reproducibility

All native outputs and online decisions are sealed before test reference values are read. There is no per-method trajectory alignment. Translation error is the Euclidean position difference; rotation error is the angle of R_ref^T R_est. We report RMSE, P95, maximum, and counts above 0.5 m or5 deg. Rejected scans retain their actual propagated output and are not assigned zero error. Reference matching uses the frozen bounded interpolation rule, not error-minimizing time shifts.

Runtime records distinguish native calls, node callbacks, input-to-fusion acknowledgment, first target construction, and process wall time. RSS for recent integrated runs is sampled every 50 ms; it is neither an exact instantaneous memory bound nor total machine consumption. Release builds and native parameters are frozen. The G-only NDT binary is rebuilt; the unchanged EKF and publisher binaries are reused with exact R24 hashes. Its native seal precedes evaluation in commit `0d4a56f55b6550a61d5c97fa6945d853cc8e49a5`.

The windows and initialization conditions are fixed. There are no repeated independent trials from which to infer confidence intervals or broad statistical significance; neighboring frames are temporally dependent. C3 evaluates recovery from a known perturbed initialization, not unknown-pose global relocalization.

## 5. Results

### 5.1. Local target resource behavior

The historical P45 matched 300-scan study reduces the active target from 3,142,751 points to 552,716 points. Recorded NDT-process high-water memory decreases from 775,308 KiB to 138,200 KiB, an 82.2% reduction. Translation RMSE changes from 0.067985 m to 0.061612 m. However, translation maximum increases from 0.119675 m to 0.216396 m, and rotation RMSE rises from 1.81802 deg to 1.87495 deg. Steady alignment mean increases from 7.086 ms to 8.079 ms. Input-to-final-acknowledgment wall time 33.82 s exceeds the 29.90 s sensor span.

![Historical local target memory](figures/local_map_memory.png)

**Figure 1.** Recorded per-process HWM in the historical matched-segment study. Different recording arrangements and profiles preclude an identical-binary microbenchmark interpretation. The live path did not cross a tile boundary; a separate replacement probe used a synthetic pose and zero NDT calls. No universal whole-system memory or real-time claim follows.

### 5.2. Initialization sensitivity and simple quality baselines

R19 observes persistent distinct native terminal neighborhoods in the strong-input-geometry C3 trajectory. Its translation/rotation RMSE is0.947759 m/7.788894 deg, versus 0.159179 m/2.033813 deg for C0. Six repeated optimizations near the earliest qualifying three-frame segment remain within 0.031275 m/0.197372 deg of their respective terminals. This supports stable distinct convergence behavior, not a proof of strict mathematical local minima.

The harmful branch has substantially lower native scores and degraded coverage. A threshold developed from these observations, Q>0.30, flags all 21 reference-relative suspect records among 480 original outputs, together with 14 other records. The latter records are additional alarms, not independently certified false positives. The apparent detection coverage is a development statistic.

R20 separates detection from causal recovery:

|Window/C3|Native baseline RMSE m/deg|Quality gate only|Quality-triggered multistart|
|---|---|---|---|
|WEAK|0.283121 /2.808664|4.164507 /7.016976|0.266655 /1.862490|
|STRONG|0.947759 /7.788894|0.351154 /4.143002|0.351154 /4.143002|

Gate-only rejects enough observations to damage the weak-window predictor; multistart restores useful measurements there. In the strong window multistart adds no trajectory benefit and misses the preset 0.35 m/3 deg dual target. Some physically different candidates improve reference error but remain above the fixed quality threshold. This prevents a claim that the absolute gate always admits beneficial corrections.

### 5.3. A rejected extension: relative-only admission

R21's single-frame Pareto admission produces a favorable strong-window development result: 0.158692 m/2.071567 deg versus R20 C's 0.351154 m/4.143002 deg. Two-frame confirmation is not better and adds a real confirmation call and delay. The subsequent frozen R22 test exposes an important counterexample. On W1/C0, C achieves 0.211401 m/2.422765 deg, while relative admission produces 0.242043 m/2.620034 deg and increases high-error frames from 2 to 5. The translation degradation 0.030641 m exceeds the prespecified safety boundary. A favorable W1/C3 result does not erase this normal-initialization failure; three other windows do not exercise the relative-only mechanism. We therefore exclude that extension rather than adding another fitted threshold.

### 5.4. True inertial closure and geometry responsibility

R23 verifies actual inertial propagation, native registration, and delayed map-observation fusion. Across ten formal trajectories it records 432 Full NDT calls, 6,110 original IMU propagations, and 418 applied map updates. Yet W1/C3 never enters NDT: the existing pre-gate rejects all 60 scans and the inertial state drifts. The result cannot be interpreted as an optimizer failure because no alignment was attempted.

R24 G changes only this optimization veto while retaining C and all post-quality criteria. In W1/C3, the new causal path violates the original geometry predicate on its first four scans. A restart result becomes admissible at raw 2603, after which the state history changes and the original geometry predicate eventually passes. The configuration publishes 57 map observations and uses eight extra registrations. G+F produces the same poses as G. The F-enabled and G+F runs receive 415 corrected messages and replace four exact same-time samples, but no subsequent NDT query consumes a replaced sample. The four replacements are later than the next scan start and are correctly excluded. This is not a demonstrated prediction-freshness fix.

### 5.5. New G-only ablation

The new R25 run executes 60 nominal Full registrations, all converged, with zero additional calls. None passes the retained post-quality gate. Its entire EKF and prediction pose sequences equal the original rejected-observation trajectory.

|W1/C3 method|Translation RMSE/P95/Max m|Rotation RMSE/P95/Max deg|Native nominal+extra|Accepted/rejected|High-error frames|
|---|---|---|---:|---:|---:|
|Original pre-gate, R23|7.986422 /16.421686 /18.755587|13.052649 /17.166246 /18.375466|0+0|0/60|60|
|G-only, R25|7.986422 /16.421686 /18.755587|13.052649 /17.166246 /18.375466|60+0|0/60|60|
|C+G, R24|0.209943 /0.300124 /0.444282|2.984149 /3.723287 /11.245245|60+8|57/3|2|

The optimizer-access change alone is insufficient in this case. The additional candidate from the fixed -5 deg seed at raw 2603 has Q=0.296356, passes the unchanged gate, and actually reaches the filter. Raw 2604 is rejected again, while raw 2605 onward admits nominal measurements. The longest no-map interval decreases from 60 scans to 2. Raw 2603 is both the first map update and the first output below the existing 0.5 m/5 deg reference-error criterion for the remainder of the approved window. That recovery-time description is post hoc and did not select a candidate.

![Complete causal ablation errors](figures/ablation_errors.png)

**Figure 2.** Complete 60-frame W1/C3 errors. Original and G-only curves coincide; the green line marks the first C+G map update. Horizontal position uses scan-start elapsed time, while each error is evaluated at its actual fused-state timestamp. The two initial large rotation errors remain visible.

![Actual quality decisions](figures/coverage_admission.png)

**Figure 3.** Frozen Coverage Consistency Mean and the 0.30 quality gate. Missing selected values are gaps, not zero losses. The plot concerns quality admission rather than independent correctness.

C+G reduces complete-window translation RMSE by 97.4% and rotation RMSE by 77.1% relative to the original/G-only output in this development instance. It does not remove the initial transient: maximum rotation error is11.245245 deg and two scans exceed 5 deg. On the normal W1/C0 and W3/C0 controls, R24 G and G+F preserve the original error sequences. These controls do not establish safety outside the tested windows.

### 5.6. Cost and robot-validation status

|W1/C3 method|Callback Mean/P95/Max ms|NDT sampled peak KiB|Simultaneous NDT+EKF peak KiB|
|---|---|---:|---:|
|Original pre-gate|30.352 /39.254 /602.224|259060|282196|
|G-only|110.496 /179.406 /977.302|369136|389380|
|C+G|55.304 /111.302 /1006.031|369096|392536|

The original path completes less work because it never calls NDT. C+G uses 13.3% more solver calls than G-only but follows a different subsequent optimization branch. Its lower observed callback mean is not a repeated-trial speed guarantee. Initialization and map construction affect maxima; neither this table nor a low mean proves 10 Hz operation. The larger active target in this window also explains why the earlier 135 MiB P45 process figure is not a universal memory budget.

The local quadruped loop 2 assets are present: a 19.68 GB bag with 10,905 Mid -360 scans, 218,101 IMU messages, and a matching FAST-LIVO2 prior. However, its actual mapping configuration uses identity/zero LiDAR–IMU extrinsics explicitly as a fallback assumption. No physically established transform chain was available. We therefore do not execute or invent a new DOG_BASELINE versus DOG_CG result. Existing FAST-LIVO2 mapping poses are a same-system reference, not independent truth. A visualization-only configuration is supplied but has not been accepted as a robot localization launch.

## 6. Discussion

### 6.1. What has been established

The principal evidence is a causal interaction. A hard pre-optimization veto prevents the optimizer and its recovery baseline from being exercised. Enabling optimization is necessary but, under a strict unchanged post-quality gate, not sufficient. A fixed additional initialization supplies the first admitted observation and changes the later inertial trajectory. This is stronger than reordering frozen candidates because both G-only and C+G histories are actual independent closed-loop executions. It remains one known development instance, not a general theorem about registration basins.

The result also distinguishes convergence, admission, and accuracy. All G-only nominal registrations converge, but no observation is admitted. An admitted C+G observation changes the trajectory, but the accuracy claim comes only from later reference-relative evaluation. The official reference and frame assumptions limit that claim; coverage and native scores do not replace physical truth.

### 6.2. Scope of originality

We do not claim a new NDT objective, a new degeneracy detector, a new Schur-complement solver, or the invention of multistart search. The defensible contribution is a specific, reproducible system design and a controlled demonstration of responsibility between map availability, initial geometry diagnostics, post-registration quality, and bounded recovery. Establishing sufficient novelty for a journal still requires editorial assessment against the closest systems. No unimplemented or unsuccessful development mechanism is folded into the retained contribution.

### 6.3. Remaining failures and limitations

The raw scan covariance ratio used for prescreening is not the native NDT Hessian or a proof of intrinsic degeneracy; the sustained failure occurred in the nominally stronger input-geometry window. The fixed absolute quality gate can reject a reference-improving result, as seen in the strong development window. The successful W1/C3 candidate is close to the threshold; no robustness sweep is performed or used to retune it. Relative-only admission has a concrete safety counterexample and is excluded. Initial rotation transients remain even when window RMSE improves. Per-run recovery budgets do not demonstrate indefinite operation.

Freshness is also unresolved. Publishing a corrected current state does not update a previously delivered earlier prediction sample. Exact timestamp matching frequently fails because of the inherited integer-header versus double-time publication path, and a legal latest-sample replacement can still lie after the next scan start. The tested F switch does not justify a claim that this history is synchronized. We retain the existing filter behavior and disclose the limitation rather than changing its mathematics during the ablation.

Finally, the experiments share one public sequence and several intentionally perturbed, reference-anchored short windows. They establish neither independent motion-capture precision nor cross-dataset generalization. The surveyed prior and map-registered reference share geometry. The conditional Base interpretation lacks direct CSV-export certification. Robot assets are real but a valid deployment comparison is blocked by physical calibration provenance. These limitations preclude a claim that the quadruped system is deployment-ready or that a universal 0.30 threshold has been validated.

## 7. Conclusion

We presented and causally evaluated a resource-conscious prior-map NDT system with geometry-aware optimizer access, fixed post-registration quality admission, real inertial continuity, and bounded yaw restarts. A new G-only ablation shows that merely permitting registration does not recover a rejected-observation trajectory:60 converged nominal results still yield no map updates. The frozen C+G combination admits a restart result, reduces complete-window RMSE to 0.210 m/2.984 deg, and retains most subsequent map observations using eight extra calls. Historical local-target measurements support a spatial memory benefit with explicit timing and tail-error tradeoffs. These results justify retaining the simple combination as an engineering baseline and manuscript contribution candidate. They do not establish new optimization theory, independent physical accuracy, universal reliability, or completed robot deployment.

## References

1. M. Magnusson. *The Three-Dimensional Normal-Distributions Transform—An Efficient Representation for Registration, Surface Analysis, and Loop Detection*. Doctoral thesis, Örebro University, 2009. [Institutional repository](https://www.diva-portal.org/smash/get/diva2:276162/FULLTEXT02.pdf).
2. Point Cloud Library. *How to use Normal Distributions Transform*. [Official documentation](https://pointclouds.org/documentation/tutorials/normal_distributions_transform.html). Runtime version in this work:PCL 1.10; accessed 2026-10-24.
3. T. Tuna, J. Nubert, Y. Nava, S. Khattak, and M. Hutter. *X-ICP: Localizability-Aware LiDAR Registration for Robust Localization in Extreme Environments*. arXiv:2211.16335,2022; revised 2024. [Primary manuscript](https://arxiv.org/abs/2211.16335).
4. D. Lee, H. Lim, and S. Han. *GenZ-ICP: Generalizable and Degeneracy-Robust LiDAR Odometry Using an Adaptive Weighting*. arXiv:2411.06766,2024; IEEE RA-L, DOI 10.1109/LRA.2024.3498779. [Primary manuscript](https://arxiv.org/abs/2411.06766).
5. S. Zhao, H. Zhu, Y. Gao, B. Kim, Y. Qiu, A. M. Johnson, and S. Scherer. *SuperLoc: The Key to Robust LiDAR-Inertial Localization Lies in Predicting Alignment Risks*. ICRA 2025; arXiv:2412.02901. [Primary manuscript](https://arxiv.org/abs/2412.02901).
6. W. Xu, Y. Cai, D. He, J. Lin, and F. Zhang. *FAST-LIO2: Fast Direct LiDAR-inertial Odometry*. arXiv:2107.06829,2021. [Primary manuscript](https://arxiv.org/abs/2107.06829).
7. M. Ramezani, Y. Wang, M. Camurri, D. Wisth, M. Mattamala, and M. Fallon. *The Newer College Dataset: Handheld LiDAR, Inertial and Vision with Ground Truth*. IROS 2020, pp. 4353–4360. DOI 10.1109/IROS 45743.2020.9340849. [Primary manuscript](https://arxiv.org/abs/2003.05691).
8. Oxford Robotics Institute. *Newer College Dataset—Ground Truth*. [Official reference-construction documentation](https://ori-drs.github.io/newer-college-dataset/ground-truth/). Accessed 2026-10-24.
9. Oxford Robotics Institute. *frontier_description*, newer-college-dataset branch, pinned commit 0b039393334a7767e24a711bd05816bfc7f7485a. [Official frame-model evidence](https://github.com/ori-drs/frontier_description/blob/0b039393334a7767e24a711bd05816bfc7f7485a/README.md).

## Reproducibility appendix

Internal evidence is archived by experiment, not pooled into one claim: `docs/p45_newer_college_local_map/`; `docs/p62_r19_native_ndt_basin_stress_test/`; `docs/p62_r20_quality_gated_recovery/`; `docs/p62_r21_relative_temporal_admission/`; `docs/p62_r22_frozen_relative_holdout/`; `docs/p62_r23_real_imu_closure/`; `docs/p62_r24_geometry_oosm_freshness/`; and the present R25 directory. R25 provides native/source identities, raw run logs, complete ablation tables, figure data and PNG/PDF/SVG exports. Historical outputs are not overwritten. The dog asset contract separately marks blocked versus measured fields. No new experiment is implied by the draft's completion.
