# PAPER-P4-I1: Sparse-probe NDT recovery prototype

## Scope and frozen inputs

Offline-only analysis of the captured R10B Floor01 scan requests. No runtime, FAST-LIO2/ESKF, protocol, map, bag, configuration, or NDT parameter was modified; the bag was read directly and never played.

- Window: 80–170 s from evaluator origin; selected 1 Hz events: 91.
- Candidate set: 13 body-horizontal XY translations (center plus ±0.5/±1/±2 m on each axis), then four yaw probes (±3/±6°) around the best-scoring XY translation; 17 score-only hypotheses total.
- Sparse branch: if any non-center probe score exceeds the center score, align the two highest-scoring non-center candidates. The offline full comparator aligns all 16 alternatives.
- Candidate selection uses probe score, final PCL fitness, convergence flag, and predictor-relative motion gate only. GT is not read by selection code; it is used afterward for evaluation.
- Predeclared motion gate: final aligned pose must remain within 2.0 m translation and 6.0° relative yaw of the predicted pose. This fixed analysis choice is not a tuned runtime parameter.
- Recovery thresholds: fixed relative fitness improvements of 10%, 20% (primary), and 30%.
- Reset-anchor results are an offline approximation: after an accepted recovery, left-multiply baseline corrected IMU poses by one fixed SE(3) correction until the next accepted recovery, and stop carrying resets beyond 170 s. This is not closed-loop replay.

Input SHA-256:

```text
bag: 860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db
map: 2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570
gt: b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f
extrinsics: fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414
```

## Score-only implementation and provenance

The candidate ranking adapts Autoware's MULTI_NDT_SCORE pattern: transform the source cloud at each proposed pose and evaluate nearest-voxel transformation likelihood without running the optimizer. Its XY offsets are rotated from the body-horizontal basis into map coordinates. The local comparison checkout did not contain the Autoware NDT-OMP implementation, so this is a PCL 1.10 semantic adapter over the protected NDT target-cell grid, not a bit-for-bit port. The adapter uses the maximum Gaussian cell contribution per point and averages over points that find a target neighborhood. [Autoware score-only implementation](https://github.com/autowarefoundation/autoware_core/blob/main/localization/autoware_ndt_scan_matcher/src/ndt_omp/estimate_covariance.cpp) and [NDT matcher documentation](https://autowarefoundation.github.io/autoware_core/pr-678/localization/autoware_ndt_scan_matcher/).

This stage does not claim novelty for multi-start NDT, NDT covariance, Hessian uncertainty, or multi-resolution NDT. It tests only low-frequency score probing followed by full optimization of a small number of alternatives for recovery.

## Replay reproduction gate

Across 91 selected requests, offline single-start NDT max pose difference versus captured raw NDT was 0.00015443214 m translation and 0.0010355508° rotation; max fitness difference was 1.2860725e-05. The predeclared numerical-equivalence limits were 0.001 m, 0.01°, and 0.0001 fitness, respectively.
Helper compile: `g++ -std=c++14 -O2 -Wall with PCL 1.10`.

## Baseline and recovery outcomes

Baseline 80–170 s translation deviation: mean 2.6215 m, median 1.5469 m, P95 11.3721 m, RMSE 3.9877 m.

Primary 20% threshold:

- Sparse: triggered 38/91, accepted 7; accepted-frame translation improved/worsened/unchanged post hoc: 4/3/0 (57.1% improved).
- Sparse accepted translation improvement: mean 0.2025 m, median 0.1249 m, P95 0.8101 m; rotation improvement mean/median 3.2560/3.3180°.
- Full multi-start comparator: accepted 39; translation improved/worsened/unchanged post hoc 21/18/0 (53.8% improved).

Threshold ablation (all choices use NDT fitness and predictor-relative motion only; GT is post hoc):

| Method | Fitness threshold | Accepted | GT improved | GT worsened | Improvement rate | Translation gain mean / median / P95 (m) |
|---|---:|---:|---:|---:|---:|---:|
| sparse_probe_top2 | 10% | 11 | 7 | 4 | 63.6% | 0.2457 / 0.1249 / 0.7886 |
| sparse_probe_top2 | 20% | 7 | 4 | 3 | 57.1% | 0.2025 / 0.1249 / 0.8101 |
| sparse_probe_top2 | 30% | 5 | 4 | 1 | 80.0% | 0.3596 / 0.2528 / 0.8528 |
| full_multi_start | 10% | 62 | 37 | 25 | 59.7% | 0.1195 / 0.0150 / 1.0663 |
| full_multi_start | 20% | 39 | 21 | 18 | 53.8% | 0.0663 / 0.0163 / 0.5540 |
| full_multi_start | 30% | 22 | 14 | 8 | 63.6% | 0.0661 / 0.0601 / 0.5113 |

The 10% and 30% groups are fixed sensitivity ablations, not GT-selected operating points.

## Persistent crossings in the 80–170 s evaluation window

Crossing means a threshold exceedance persisting at least 5 s, allowing at most 0.25 s between samples. A missing crossing means no persistent crossing within this window (right-censored at 170 s).

| Translation deviation | Baseline | Sparse reset-anchor (20%) | Full multi-start reset-anchor (20%) |
|---:|---:|---:|---:|
| 0.5 m | 84.9193 s | 84.9193 s | 83.5074 s |
| 1.0 m | 93.5928 s | 93.5928 s | 93.0885 s |
| 2.0 m | 151.4832 s | 122.6389 s | 153.6012 s |
| 5.0 m | 157.4336 s | 158.7447 s | 158.7447 s |

Primary-window translation deviation mean / RMSE / P95 (m): baseline 2.6215 / 3.9877 / 11.3721; sparse reset-anchor 2.6789 / 3.8898 / 10.9710; full multi-start 2.5602 / 3.7500 / 9.8586.

The R10B full-run persistent-crossing references are 84.919 s (0.5 m), 93.593 s (1 m), 151.483 s (2 m), and 157.434 s (5 m); this analysis recomputes crossings over the stated 80–170 s window and does not extrapolate reset corrections beyond it.

## Compute

- Single NDT mean: 6.950 ms.
- Score-only 17-candidate probe mean: 26.880 ms per 1 Hz probe event.
- Alternative full-align mean: sparse top-2 7.420 ms each; full comparator 16.594 ms each.
- Total event time at 1 Hz: sparse 40.027 ms; full multi-start 272.462 ms.
- Amortized additional compute: sparse 3.3782 ms/LiDAR frame (48.61%); full multi-start 27.1174 ms/frame (390.17%).
- Sparse/full extra-compute ratio: 12.5%.
- The full comparator is baseline align plus all 16 alternatives at each 1 Hz probe. Sparse is baseline align plus score probe and, only when triggered, two alternative aligns. Measurements are offline elapsed wall times and are not a runtime WCET guarantee.

## Verdict

**NOT_PROMISING.** Fewer than 70% of accepted sparse recoveries improved translation post hoc.

The predeclared operational bar for PROMISING was: at least 70% of accepted sparse recoveries improve translation post hoc; both 2 m and 5 m persistent crossings are delayed or absent within the window; sparse extra compute is below 30% of single-NDT cost and no more than half the full multi-start extra cost. Failure to meet this bar does not prove the concept impossible; it means this fixed first prototype did not meet its declared bar.

## Artifacts

- `probe_results.csv`: every score-only candidate, score rank, baseline replay values, and alternative alignment outputs/timing.
- `recovery_events.csv`: online-only acceptance decisions and separate post-hoc GT metrics for all methods/thresholds.
- `compute.csv`: per-probe timing and amortized quantities.
- `baseline_vs_reset_anchor_error.png`, `probe_candidate_score_landscape.png`, `compute_single_full_sparse.png`.
