# PAPER-P6-ALG-INTEGRATION-A3F-R1

**Square-root covariance code contract, pre-measurement P15 gate and the single
P3-200 engineering gate PASS.** `READY_FOR_FORMAL_EXPERIMENT=NO`.
No GT, accuracy evaluation, parameter change or second real replay.

START_SHA: `34ac3f7692ff51efc3c168fb45e080b8573dee28`

CODE_SHA: `c1ef450eb45fd5825c416e9140e9a4eed1d32d04`

The final analysis commit is END_SHA; the execution handoff supplies its full
SHA and observed remote HEAD. Existing branch/workspace reused.

## Production contract

V3 explicitly uses `MarginalCovarianceBackend::SQUARE_ROOT_QR` with no fallback.
Authoritative prior A/b, current reference-chart Jacobian, all active IMU,
historical selected LiDAR and visual factors directly form whitened rows.
One frozen LiDAR snapshot is shared with optional legacy diagnostics.
No production normal information matrix, prior-H refactorization, inverse,
jitter, epsilon-I, eigen clamp or rank drop is used to obtain P15.

AP=QR; full rank required under eps*max(m,n)*maxPivot. Solve R^T Y=P^T E.
P15=Y^T Y; unchanged map selector gives Pmap=(Y Tmap^T)^T(Y Tmap^T).
Query is estimator-read-only except diagnostic request/time counters. Timing
remains after prediction/IMU admission and before current LiDAR admission.
Non-LiDAR requests remain zero.

A3E-R2 prior/QR marginalization, optimizer, small-step termination, alias-safe
symmetry, NDT, deskew, handoff, U_obs/U_nonlocal, NIS thresholds, noise and
reliable-subspace semantics are unchanged. Legacy windows/V2 keep their backend.

## Verification

Release build PASS; full CTest **37/37** PASS; Debug targeted **15/15** PASS;
diff-check PASS. Original35 tests retained; no tolerance relaxed.
New tests cover forced permutation, correlated whitening, known inverse,
extreme-scale covariance, rank failure, rank5 factor, joint directional visual,
repeated Schur, moved-rotation/nonzero-gradient prior chart, exact ON/OFF
parity, NIS/probe parity, one callback and read-only lifecycle.
Fresh-context review findings were repaired and approved. See
[IMPLEMENTATION_AND_REVIEW.md](IMPLEMENTATION_AND_REVIEW.md).

## Exactly one real P3-200

Corridor01 V3 ADAPTIVE_SELECTED_NIS; visual/provenance NONE; raw limit200;
initialization1517157224188979000; unchanged official calibration/map/noise.
All seven frozen hashes match; actual production manifest gate passes.
51 raw scans precede handoff; **149 real Window-owned deskews/LiDAR terminals**.
Exit0, no retry. Full logs: `/tmp/p6_a3f_r1_corridor_p3_200`.

| Health metric | Result |
|---|---:|
| QR covariance requests / available / unavailable | 149 / 149 / 0 |
| Rank failures, nonfinite P15/Pmap, PSD failures | 0 |
| Legacy shadow available / unavailable | 129 / 20 |
| Selected NIS valid | 123 |
| LiDAR committed / normal rejection | 107 / 42 |
| QR marginalization attempts / failures | 260 / 0 |
| Optimizer events / failures | 298 / 0 |
| U_nonlocal probes / NDT calls | 104 / 357 |
| Post-handoff IKFoM, visual factors/events, dense fallback | 0 |
| Max completed nodes / span | 40 / 1.958914906 s |

NDT converged149/149; U_obs valid124/149. Twenty-six factors are not attempted:
25 map-support rejections and one zero/invalid reliable rank. Another16 are
normal NIS rejections. These are diagnostics, not accuracy. P15 availability
does not force admission. All completed states finite and SO3-valid.
Pre-measurement nodes may reach41/615 columns before duration enforcement;
completed bounds are checked afterwards. Candidate basis callbacks0.

## Closed old tx136–155 gap

All20 logical transactions have QR P AVAILABLE/full rank615, valid selected
NIS and finite SPD P15/Pmap. Sixteen commit; four are normally rejected.
Same-state legacy remains unavailable on these20: **HESSIAN_NOT_SPD17;
NUMERICALLY_SINGULAR_HESSIAN3**. Singular cases: tx140,154,155.
These are observed shadow reasons, not guesses about an unrecorded old gate.

tx136 /1517157232804267479: QR valid; legacy HESSIAN_NOT_SPD; rank615;
NIS0.26912481158413021 <6.635; probe triggered; LiDAR committed. First
same-state shadow decision difference occurs here. This is not an alternate
legacy trajectory or evidence of accuracy improvement.

tx90,115,155 all ACCEPTED_UPDATE, followed by two successful rank15 removals;
stored H symmetry defect0/minimum eigenvalue0 (extended zero blocks).
Final spans: 1.916218042 /1.916217089 /1.916217804 seconds respectively.
A3E-R2 marginalization implementation was not changed.

## Same-current-state available-frame comparison

All129 AVAILABLE legacy frames are compared on identical current state/raw
factors/frozen projections, not on old-run states.

| Relative Frobenius error | Median | P95 | Max |
|---|---:|---:|---:|
| P15 QR vs legacy | 2.147504e-8 | 9.168521e-8 | 1.559501e-7 |
| Pmap QR vs legacy | 1.688278e-8 | 6.717647e-8 | 1.611912e-7 |

103 valid selected-NIS pairs: absolute difference median1.5084263e-9,
P95 1.09314095e-7, max8.21026738e-7. NIS decisions, probe triggers and
same-factor admission differences are all0 in AVAILABLE pairs. Shadow never
executes alternate NDT probes; unknown alternate terminals are not fabricated.

First output difference: tx52 /1517157224332516266; P15 relative1.2094514e-9,
Pmap2.7829434e-9. Both accept NIS and do not probe. Preceding scan-start and
first changed-covariance predicted pose match frozen old data. First old-run
event numerical divergence under the strict guard: tx70 scan-start, after
first probe tx69; tx69 adaptive-R entries already differ at roundoff scale.
Later trajectories are not bit-identical and are not used as paired accuracy
evidence. No unexplained pre-output drift detected.

## Numerical health and engineering cost

P15 eigen range: 1.36705427e-6 ..3.76020857; Pmap: 7.97343167e-5 ..1.11776306.
Maximum triangular residual2.67058074e-16.
Covariance row assembly/QR/solve/Gram/validation, excluding shadow:
mean37.196172 ms, P95 44.325490 ms, max46.512174 ms.
Maximum stack729×615; estimated temporary13,767,072 bytes, excluding shadow,
allocator overhead and persistent estimator storage. Marginalization QR
mean4.853761/max5.919806 ms. Wall160.70s/user150.81s/system9.42s;
maxRSS112520KiB. Diagnostics ON. Dense CPQR is a material low-compute cost;
no SparseQR or incremental optimization added. These are engineering
observations, not CPU benchmarks. Changed states also prevent formal old/new
wall-time comparison.

## Artifacts and limits

Small committed data: all covariance/shadow rows, comparison/old-region rows,
short trajectory, thin QR history, selected event capsules, input/source SHA
and resource logs. Large optimizer logs stay external with SHA/size inventory.
Raw input and all frozen results are untouched. External review is left to
the user's manual final handoff. No complete replay, GT, ATE/RPE or visual run.

A3F_R1_SQUARE_ROOT_COVARIANCE_CODE_CONTRACT=PASS

A3F_R1_PREMEASUREMENT_P15_GATE=PASS

A3F_R1_P3_200_REAL_LINK_GATE=PASS

READY_FOR_FORMAL_EXPERIMENT=NO. Stage complete; STOP.
