# P60-R4 Ground-Challenge Corridor2 NDT geometry screen

## Final result

**DEGENERACY_NOT_DEMONSTRATED** under the frozen descriptive screen.

100/100 frozen scan pairs have comparable finite, positive-definite same-pose
Hessians in all three arms; 300/300 one-pass registrations converged. This is
**raw adjacent-scan registration**, not prior-map localization or a motion-accuracy
experiment. No true errors, ATE/RPE, drift, wrong basins or recovery are certified.

Answers to the three research questions:

1. FULL_360 shows relative anisotropy, but not the preregistered severe weak-direction
   signature. All translation-block ratios are >=0.247692; minimum 6D ratio is approximately0.003476.
   No native candidate interval is emitted. This does not prove all 974 scans or
   a fixed prior-map localization chain are nondegenerate.
2. FOV_90 increases relative anisotropy significantly. All 100 pairs have lower
   6D min/max curvature ratio than the equal-point-count arm. However, zero pairs
   meet the combined conservative weak label. Do not label this as confirmed
   INDUCED_FOV_DEGENERACY or as native severe corridor degeneracy.
3. The effect is **not explained by point count alone**: B and C have exactly
   equal point counts in every pair. Their paired ratio-of-ratios median is
   0.335767, and per-point minimum curvature B/C median is 0.350349. C/A per-point
   minimum curvature median is 0.996305. The supported cause is the FoV-induced
   change in spatial sampling/structure, not simply fewer points. This design
   cannot further isolate pure azimuth loss from changed range/surface composition;
   raw motion distortion and azimuth-correlated point times are additional limitations.

## Frozen input, preprocessing and coordinates

Internal-only bag `/home/jian/ground-challenge/corridor2/corridor2.bag`:
2,871,922,599 bytes; SHA256 recomputed and verified:
`6ef85c76a36bdcef4811d411ac0f09219803377f5b7b9bd0cf2a5258cc826cea`.

Exactly 100 time-uniform source indices from original scans 2 through 974.
Source headers: `1658917771706929684` to `1658917869740242958` ns,
span **98.033313274 s**. Each target is strictly scan k-1; no source is added to its
own target. All 200 exported PCD files match the frozen manifest SHA256.

Source processing: finite/range [0.5,80]m, 0.25m VoxelGrid, production rounded
point cap1400. Target: finite XYZ, production 0.15m VoxelGrid **twice**, no target cap.
A is unmasked Source, not guaranteed angularly uniform. B keeps x>=0 and |y|<=x
AFTER shared Source preprocessing. C uses mt19937 seed6042026+k, shuffled unique
indices, then original-order restoration; exactly as many points as B.
Scanner `sensor` +X is not certified robot-forward. No IMU extrinsic is assumed.

All pairs/arms start at identity `T_(previous_scan)_(current_scan)` independently.
This is a zero-motion initial hypothesis, NOT synthetic truth. No inter-pair
trajectory state or post-match step limit is applied to this raw optimizer diagnostic.

## Primary same-pose results

Hessians are evaluated at the FULL_360 raw result, shared by A/B/C for each pair.
Energy is `-native_NDT_score_sum / source_point_count`. Chart:
`p = p0 + 0.8*eta_t` meters; `R = Exp(eta_r)*R0`, eta_r in radians.
The existing nonlinear Euler-carrier pullback, including the gradient-dependent
second-derivative term, is retained. These are objective Hessians, **not certified
Fisher information or pose covariance**. Translation block holds rotation fixed;
it is not marginalized translational information.

| Metric (median unless stated) | FULL_360 | FOV_90 | POINT_COUNT_MATCHED |
|---|---:|---:|---:|
| Pairs / converged / common SPD | 100/100/100 | 100/100/100 | 100/100/100 |
| Source points | 1389.5 | 680.5 | 680.5 |
| Source point range | 707–1400 | 278–1031 | 278–1031 |
| Target points (identical across arms) | 2393 | 2393 | 2393 |
| 6D lambda_min/lambda_max | 0.006827 | 0.002352 | 0.006745 |
| 6D condition number | 146.474 | 425.250 | 148.270 |
| Translation-block min/max ratio | 0.409906 | 0.358768 | 0.412699 |
| Minimum per-point curvature | 2.780995 | 1.017878 | 2.773024 |
| Minimum total-energy curvature (N*H) | 3553.602 | 622.941 | 1837.207 |
| Valid Gaussian-neighbor support | 85.571% | 84.329% | 85.547% |
| PCL Fitness, mean squared NN distance (m²) | 0.040325 | 0.029954 | 0.043230 |
| Align mean (ms) | 5.009 | 2.287 | 2.365 |
| Align P95 (ms) | 7.514 | 3.440 | 3.643 |
| Align max (ms) | 8.975 | 4.898 | 4.793 |
| Combined weak-screen labels | 0 | 0 | 0 |

FULL_360 6D ratio range: 0.003476–0.017789; maximum condition287.726.
FOV_90 range: 0.001233–0.010475; maximum condition810.808.
Equal-count range: 0.003661–0.020034; maximum condition273.144.
All are finite/positive here. No invalid/indefinite Hessians were discarded.

Paired C/A ratio6 median **1.003124**; B/C ratio6 median **0.335767**.
Paired C/A lambda_min-per-point median **0.996305**; B/C median **0.350349**.
These are medians of frame-wise paired ratios, not ratios of the table medians.
Dropping points still reduces total curvature: mean-energy normalization does not
make the loss of observations cost-free. Equal N for B/C controls that scalar effect.

Weakest normalized 6D axes have a dominant translation component. The translation
projection's largest component is scanner Z for 69 FULL pairs, 99 FoV pairs,
70 count-matched pairs; FULL has scanner X dominance in31 pairs. These are local
scanner axes, not certified corridor/world directions. Thus the screen does not
demonstrate the desired persistent corridor-axis translational drift mechanism.
Sign-invariant full/count weakest-6D-axis cosine median is0.999279; FoV/count0.946628.
Individual-axis changes can be sensitive to small eigenvalue gaps; no world-frame
direction is inferred by stitching these independent targets.

Secondary OWN_RESULT Hessian ratio medians: A0.006827, B0.002322, C0.006760,
with 100 positive-definite results each. They are reported separately, not pooled
with the common-pose primary comparison.

## Important frozen implementation finding: parameter vs actual grid

This screen faithfully reproduces **R2's actual Global-target lifecycle**.
Production `dog_prior_map_ndt_node.cpp:1646–1653` sets target before resolution.
PCL1.10 `ndt.h:setInputTarget()` builds cells immediately; `setResolution()` only
rebuilds if `input_` (Source) already exists. At this startup it does not.
Consequently parameter resolution is0.8m, but actual Gaussian grid leaf is
**1.0m in all300 rows**. Native score constants/query radius use the configured
0.8m. This is not silently repaired and not described as true0.8m Gaussian cells.

`tools/cloud_contract.hpp:69–74` preserves that order; a C++ regression verifies
the observed distinction. This does NOT certify parity with P45/P46 LocalMap
target rebuilding, which has a different lifecycle. Future scientific claims
must retain this implementation limitation or obtain a separately approved fix.

## Numerical parity and independent checks

First A, original scan2, matches frozen production R2:

- Source hash `2fe810b19aadb2d6`, source968 points, target1827 points: identical.
- Translation difference0m; rotation difference4.060854e-08°.
- Fitness difference0; both converged with1 iteration.
- Hessian eigenvalue relative difference0.
- Reconstructing the full R2 Hessian from archived eigenpairs gives maximum
  absolute matrix difference1.136868e-13.

All600 exported Hessians independently checked with NumPy eigenanalysis:
maximum eigenvalue absolute discrepancy1.250555e-12;
maximum weakest-eigenvector residual1.565533e-13.
`tools/metrics.hpp` explicitly sets internal `input_` for each arm before the
native derivative query; otherwise the explicit Source argument alone is not sufficient.

## Execution, tests and resources

One successful registered pass: **300 align, 600 native derivative jets**.
Explicit dynamicScore calls0; refinement/Coupled/Recovery calls0.
No production optimizer, EKF, launch or core source was modified.

One loader failure occurred before main, before output-directory creation and
before any align: MVS's old libusb lacked `libusb_set_option`. The error log is
preserved. Using the same process-only system-libusb preload already recorded
in R2 resolved it. No system USB/power configuration was modified.

Screen process wall2.74s, user2.14s, system0.59s; maximum RSS29552KiB
(28.859375MiB). This is a tiny-target offline probe process, NOT production
NDT/EKF deployment memory and NOT a10Hz fused-localization acceptance test.
Derivative diagnostics are additional research cost, not ordinary CONTROL cost.

Tests:9 Python unit tests,1 C++ contract test, Python compilation, independent
matrix verification and SHA checks passed. Old CTest ignored `--test-dir` and
initially found no tests; executing from its build directory then genuinely ran
and passed the C++ test. Logs were preserved rather than counted as a pass.

## Evidence and reproduction

`PROTOCOL.md` was committed before R4 align; `frozen_indices.json` and
`sample_manifest.csv` fix all100 pairs. `run_freeze.json` fixes raw run outputs,
binary, manifests and resource logs. `hessian_statistics.csv` contains600 labeled
records with all36 Hessian entries, six eigenvalues, weakest6D axis, translation
and rotation block eigenvalues, gradient norm, score, support and timing.
`alignments.csv` contains all300 raw poses/results; `group_comparison.csv` and
`paired_comparison.csv` retain every fixed pair. Figures show all100 pairs.

Raw exports/build/logs remain exclusively on internal disk:
`/home/jian/livox_ws/p60_r4_ground_challenge_degeneracy_runs/`.
No large PCD/bag/binary is committed. Reproduction commands are in `REPRODUCE.md`.

START_SHA: `3693ef524ade148e9b76b9ae1523ecdebb7e869a`.
CODE_SHA (actual probe run): `119be01e92f9efcc352cd58ad37d5099787d2b89`.
Analysis code: `cba2e7f13418d00a6485f584471ec75eca24d39c`.
Binary SHA256: `97850a02a185ae20e84afe8ecbb893a8628b3d8fe666d5b128771b9068ddbd0a`.
END_SHA: archive commit containing this report, reported explicitly at handoff.
Remote R3 remains START_SHA; R4 ref absent when checked. PUSH_EXECUTED=false.

## Unique NEXT, not executed

Request a separate bounded engineering review of the frozen Target-grid
initialization order (configured0.8m versus actual1.0m) before deciding whether
to authorize any corrected-grid repetition. Do not begin a new degeneracy
algorithm/Coupled experiment from this negative screen.
