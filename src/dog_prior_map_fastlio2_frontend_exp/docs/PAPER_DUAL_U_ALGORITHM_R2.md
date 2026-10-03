# Paper Dual-U Algorithm R2: rapid falsification

Date: 2026-10-03  
Branch: `research/dual-u-algorithm-r2`  
Base: R1 closure `9945c4f5c3d7759104de108a594bcaf2553fd78c`

## Prototype contract

The opt-in `--r2-sparse-probe` runner mode is diagnostic/shadow-only. Every
frame still commits the original prediction-centered NDT measurement through
the unchanged IKFoM update. Probe terminals are separately passed through a
clone of the pre-update frontend so that each candidate's counterfactual
measurement update can be evaluated offline. No probe or candidate selection
changes the recorded trajectory.

The GT-free direction trigger selects at most the first two ascending
branch-local curvature eigenvectors `q_i` satisfying
`lambda_i / lambda_5 <= 0.01`. For each selected direction, dynamic target-cell
membership is tested at chart radii
`[0.005, 0.01, 0.02, 0.04, 0.08, 0.16, 0.32, 0.5]`. The branch-exit radius
`h_exit` is the first sampled radius for which either sign changes at least
10% of source-point Gaussian-cell membership sets. A symmetric pair of NDT
initial guesses is then placed at
`T_probe = ApplyProductChart(T0, +/- (h_exit + 1.0) q_i)`.
The chart remains the R1 product chart
`[map translation / L, map-spatial rotation]` with actual target-grid scale
`L = 0.8 m`. A frame can therefore request at most four additional NDT aligns.
The support exit is a branch transition diagnostic, not proof of a distinct
registration basin.

Effective terminals are clustered with complete-link compatibility cutoffs
`0.20 m` and `2 deg`. The nominal cluster is supported by the nominal align;
a separated alternative is supported only when at least two independent
structured probe aligns converge into that cluster. The search is finite and
always reports unresolved candidate completeness. It never claims global
uniqueness.

Candidate comparators recorded are:

- raw PCL NDT score per source point, descending;
- existing frontend full-pose innovation NIS, ascending, evaluated before any
  update with `EXACT_LOG_RESIDUAL` and the frozen baseline pose noise;
- the same NIS ranking restricted to the nominal or probe-supported clusters;
- a fixed-coefficient pseudo-MAP diagnostic
  `log(PCL_score_per_source) - 0.5 * NIS`, restricted to supported clusters.

The last quantity is not a calibrated posterior: PCL's transformation score
is not a normalized log likelihood and no innovation-covariance determinant is
included. It is retained only as the requested minimum combined comparator,
not as a probability claim or proposed final policy.

All thresholds above were fixed before the post-hoc GT evaluation and were not
GT-tuned. They are prototype design constants, not literature-calibrated
probability thresholds. The 1.0 chart-unit post-exit margin is a particularly
coarse probe displacement and remains an engineering limitation.

## Rapid experiment

Dataset: SuperLoc Floor01, the frozen 32-transaction cohort
`[120, 244, 368, 616, 740, 838, 839, 864, 924, 925, 1111, 1235, 1359,
1497, 1498, 1556, 1557, 1606, 1730, 1854, 2102, 2226, 2350, 2598, 2722,
2846, 3094, 3217, 3341, 3631, 3796, 3962]`. The runner replayed the same
chronological prefix through transaction 3962 to preserve live filter state
and covariance. No GT path was given to the runner. The official Floor01 GT
and fixed common-anchor evaluator were used only after all candidate files
were written; GT SHA-256 was
`b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f`.

The full 3962-row shadow trajectory is byte-identical to the matching Run1
baseline prefix. All 32 nominal cloned updates equal the committed baseline
state exactly in the serialized position/quaternion components (maximum
absolute component difference `0`). The replay was finite and applied all
3962 nominal LiDAR updates.

| Diagnostic | Result |
|---|---:|
| U_obs spectrum trigger | 23 / 32 frames (71.9%) |
| Extra probe NDT aligns | 90 total; 3.91 per triggered frame; maximum 4/frame |
| Probe NDT convergence | 90 / 90 |
| Terminal cluster count | 1 cluster: 23 frames; 2: 7; 3: 2 |
| Frames with a repeated-probe-supported alternative | 1 / 32 |
| Frames with singleton alternative cluster(s) | 8 / 32 |
| Diagnostic time over cohort | mean 241.7 ms/frame; median 315.0; P95 415.1; max 446.5 |
| Total diagnostic time | 7.92 s |
| Prefix replay wall time / peak RSS | 104.82 s / 111,464 KiB (108.9 MiB) |

The R1 baseline cohort's 32-frame counterfactual translation errors under the
same P6-I6A fixed common anchor were: mean `0.444052 m`, RMSE `0.527909 m`,
median `0.446768 m`, P95 `0.858545 m`, max `1.086067 m`. Results for selector
counterfactual states were:

| Selector | Mean (m) | RMSE (m) | Median (m) | P95 (m) | Max (m) | Improved / worsened frames |
|---|---:|---:|---:|---:|---:|---:|
| Baseline nominal | 0.444052 | 0.527909 | 0.446768 | 0.858545 | 1.086067 | — |
| Raw PCL score | 0.442896 | 0.526501 | 0.446525 | 0.855372 | 1.086067 | 15 / 4 |
| Predictor NIS | 0.444030 | 0.527894 | 0.446768 | 0.858545 | 1.086067 | 4 / 1 |
| Supported-cluster NIS | 0.444030 | 0.527894 | 0.446768 | 0.858545 | 1.086067 | 4 / 1 |
| Supported-cluster pseudo-MAP | 0.443839 | 0.527578 | 0.446768 | 0.857064 | 1.086067 | 6 / 2 |

Among the 23 triggered frames, an offline oracle choosing the probe with the
lowest GT error had a median translation gain of only `0.00202 m` (mean
`0.01159 m`, max `0.09708 m`). That oracle is not an online policy. Even the
raw-score selector changed the cohort RMSE by only `0.27%`; its maximum error
did not improve and it worsened four frames. Rotation RMSE was `2.21856 deg`
for baseline and `2.21892 deg` for raw-score selection.

The 3962-frame run scheduled probes only at the frozen 32 cohort transactions,
so its measured whole-prefix average timing is not representative of deploying
the trigger on every frame. Baseline prefix mean/P95 frame time was
`22.98/43.15 ms`; the diagnostic replay was `26.32/46.97 ms`. At the observed
cohort trigger rate, triggered diagnostic computation averaged about `344 ms`.
This is incompatible with a <40 ms mean target if a substantial fraction of
all frames trigger. The exact full-sequence trigger rate was not measured in
this rapid run.

## Decision

The structured probes mostly converged back to the nominal basin; only one
frame had an alternative supported by two independent probes. The very small
candidate-level gains did not translate into a material selected-update
improvement, while the per-trigger diagnostic cost is high. The prespecified
32-frame positive-signal gate is therefore not met. No full 4127-frame
BASELINE / U_obs / U_nonlocal / Dual-U ablation was run, and no R2 candidate
update was committed to the main trajectory.

R2 outcome: `DUAL_U_COUPLING_NOT_SUPPORTED`. Specifically, the available data
do not support that branch-local weak directions, followed by this sparse
branch-exit probing rule, reliably reveal and select useful nonlocal basins
at low cost. This is a falsification result, not evidence that all possible
Dual-U formulations fail.

## Artifacts

- Runner diagnostics: external Floor01 results directory
  `results/dual_u_r2_sparse_probe_20261003_counterfactual/`
- Post-hoc per-frame and summary outputs:
  `posthoc/r2_sparse_32_v2.csv` and `.json`
- Post-hoc evaluator: `scripts/p7/evaluate_r2_sparse_probe_posthoc.py`
- R2 runner: `scripts/p7/p7_single_state_runner.cpp`
- Frozen thresholds/policy: `include/dog_prior_map_fastlio2_frontend_exp/dual_u_r2_sparse_probe.hpp`
