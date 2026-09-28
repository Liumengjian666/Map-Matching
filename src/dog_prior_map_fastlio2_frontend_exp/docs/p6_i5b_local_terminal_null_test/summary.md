# PAPER-P6-I5B local terminal-response null test

Stage: `LOCAL_NULL_TEST_COMPLETE`; existing-data offline analysis only.

## Scope and integrity

- Base/I5A commit: `0a8d6fe6283b421813dd3c2eac4dbb46f6befceb`; parent/frozen base: `ec35ced556f0f1eba192b928bf07452543c5c0dc`.
- Frozen inputs checked: 11; changed hashes: 0; input gate: **PASS**.
- Principal/dense frames: 88/24; logical training probes: 288; held-out outcomes: 6144.
- Missing/conflicting required records: 0/0.
- New NDT calls: **0 by design**; no ROS/PCL/NDT runner or official GT was used. The P6-I4 post-hoc GT CSV was not opened.
- Original paper/I5A worktrees were not edited; this worktree is isolated from their dirty state.

## Frozen mathematical method

For each dense frame, the 6×6 empirical local terminal-response secant uses only its twelve principal `±0.25` probes: `J_w[:,j]=(e(+0.25 e_j)-e(-0.25 e_j))/0.5`. Rows are `[rotation_map, translation_map]`; rotation is `Log(R_terminal R_0^T)` (map-frame left tangent), translation is `p_terminal-p_0`. No covariance re-factorization or extra-ray fitting is used.

Synthetic quaternion left/right tangent unit test: **PASS**; left recovery error 7.41e-17 rad; wrong-side result is separately distinguished.
Primary local support: **22 strict**, **2 contaminated** of 24. Transaction 1 status: `LOCAL_SUPPORT_CONTAMINATED` (ONE_OR_MORE_REJECTED_AT_ALPHA_0P25); it was not excluded post hoc.

### Training ± symmetry residuals

Separate units are retained; rotation and translation residuals are never summed.

| Cohort | Rotation ||b_R|| (rad), mean / median / P95 / max | Translation ||b_t|| (m), mean / median / P95 / max |
|---|---:|---:|
| all 24 frames × 6 axes | 0.003165 / 0.001096 / 0.01369 / 0.05433 | 0.01376 / 0.002922 / 0.02474 / 0.4534 |
| strict support only | 0.00207 / 0.001074 / 0.006482 / 0.01678 | 0.0052 / 0.002866 / 0.01666 / 0.04406 |

## Held-out acceptance results

32 extra directions × both signs are held out from fitting. Aggregated probe counts are descriptive; the frame is the scene-level unit and per-frame metrics are also supplied. `ALWAYS_ACCEPTED` is the fixed negative control. `Balanced Accuracy` is undefined if a selected subset has only one actual class.

| Cohort | α | n | Actual accept rate | Secant accuracy / balanced accuracy | ALWAYS_ACCEPTED accuracy | TP / TN / FP / FN |
|---|---:|---:|---:|---:|---:|---:|
| STRICT_LOCAL_SUPPORT | 0.5 | 1408 | 0.9943 | 0.9943 / 0.5 | 0.9943 | 1400 / 0 / 8 / 0 |
| STRICT_LOCAL_SUPPORT | 1 | 1408 | 0.968 | 0.9496 / 0.6946 | 0.968 | 1318 / 19 / 26 / 45 |
| STRICT_LOCAL_SUPPORT | 2 | 1408 | 0.9048 | 0.7912 / 0.6609 | 0.9048 | 1047 / 67 / 67 / 227 |
| STRICT_LOCAL_SUPPORT | 3 | 1408 | 0.7521 | 0.6584 / 0.6471 | 0.7521 | 709 / 218 / 131 / 350 |
| LOCAL_SUPPORT_CONTAMINATED | 0.5 | 128 | 0.4922 | 0.7109 / 0.7063 | 0.4922 | 26 / 65 / 0 / 37 |
| LOCAL_SUPPORT_CONTAMINATED | 1 | 128 | 0.4219 | 0.6406 / 0.5766 | 0.4219 | 9 / 73 / 1 / 45 |
| LOCAL_SUPPORT_CONTAMINATED | 2 | 128 | 0.3203 | 0.6797 / 0.5193 | 0.3203 | 3 / 84 / 3 / 38 |
| LOCAL_SUPPORT_CONTAMINATED | 3 | 128 | 0.1328 | 0.8359 / 0.5069 | 0.1328 | 1 / 106 / 5 / 16 |
| ALL_DENSE_FRAMES | 0.5 | 1536 | 0.9525 | 0.9707 / 0.9326 | 0.9525 | 1426 / 65 / 8 / 37 |
| ALL_DENSE_FRAMES | 1 | 1536 | 0.9225 | 0.9238 / 0.8548 | 0.9225 | 1327 / 92 / 27 / 90 |
| ALL_DENSE_FRAMES | 2 | 1536 | 0.8561 | 0.7819 / 0.7409 | 0.8561 | 1050 / 151 / 70 / 265 |
| ALL_DENSE_FRAMES | 3 | 1536 | 0.7005 | 0.6732 / 0.6821 | 0.7005 | 710 / 324 / 136 / 366 |

Predictions are geometry-only: the local model does not predict the PCL convergence/termination decision. Therefore all-probe confusion includes non-converged actual probes as rejects; `CONVERGED_ONLY` and `PREDICTED_IN_LOCAL_DOMAIN` sensitivities are available in `heldout_acceptance_metrics.csv`.

## Terminal-pose response errors

Errors are predicted terminal versus actual terminal; rotation uses the SO(3) geodesic angle. All-probe sensitivity and converged-only results are separate. Values below are STRICT_LOCAL_SUPPORT.

| α | Scope | n | Translation mean / median / P95 / max (m) | Rotation mean / median / P95 / max (deg) |
|---:|---|---:|---:|---:|
| 0.5 | ALL_PROBES | 1408 | 0.01252 / 0.008659 / 0.0353 / 0.1019 | 0.2772 / 0.1586 / 0.8851 / 2.208 |
| 0.5 | CONVERGED_ONLY | 1408 | 0.01252 / 0.008659 / 0.0353 / 0.1019 | 0.2772 / 0.1586 / 0.8851 / 2.208 |
| 1 | ALL_PROBES | 1408 | 0.02774 / 0.02121 / 0.07009 / 0.4088 | 0.6249 / 0.4312 / 1.765 / 5.235 |
| 1 | CONVERGED_ONLY | 1408 | 0.02774 / 0.02121 / 0.07009 / 0.4088 | 0.6249 / 0.4312 / 1.765 / 5.235 |
| 2 | ALL_PROBES | 1408 | 0.06888 / 0.05042 / 0.168 / 0.9088 | 1.235 / 0.9029 / 3.352 / 10.32 |
| 2 | CONVERGED_ONLY | 1408 | 0.06888 / 0.05042 / 0.168 / 0.9088 | 1.235 / 0.9029 / 3.352 / 10.32 |
| 3 | ALL_PROBES | 1408 | 0.1131 / 0.08207 / 0.2964 / 1.291 | 1.896 / 1.304 / 5.318 / 17.06 |
| 3 | CONVERGED_ONLY | 1408 | 0.1131 / 0.08207 / 0.2964 / 1.291 | 1.896 / 1.304 / 5.318 / 17.06 |

## Predicted versus detected extra-ray exit radius

- Strict-support signed extra rays: 1408; finite detected first sampled exits: 364; right-censored through α=3: 1044.
- Predicted finite within α≤3 versus actual censored/finite status disagreements: 468.
- Finite-pair predicted/actual rejected-side endpoint α ratio (no fitted calibration): 2.233 / 1.204 / 7.587 / 32.49.
- I5A dense finite/censored winner cross-check rows: 20/4 passed.
- Censored directions are not converted into exact α=3 boundaries. The α=3 cap is right-censoring, and a local α estimate does not prove a continuous first-exit infimum.

## Exception/mismatch interpretation

`exception_frames.csv` has 567 descriptive rows, including every strict-support exit-status disagreement with its saved inside/outside endpoint, seed, convergence, iteration, objective, fitness, predicted terminal at the observed endpoint alpha, and raw terminal jump measurements. `abrupt_response_assessment` is not assigned a label because no frozen threshold exists; raw bracket displacement is retained. Rank-only terminal-error rows are descriptive, not thresholded.

Q4: For strict support, local finite-within-α≤3 predicted versus observed first-exit status has TP=232, TN=708, FP=336 (predicted finite, observed right-censored), FN=132 (predicted beyond cap, observed finite); 468/1408 signed rays disagree across 21 frames (P2F002, P2F003, P2F004, P2F005, P2F006, P2F011, P2F014, P2F024, P2F034, P2F041, P2F045, P2F048, P2F072, P2F073, P2F074, P2F075, P2F083, P2F084, P2F086, P2F087, P2F088). The IDs and each saved bracket endpoint are in `exception_frames.csv`; censored search results remain right-censored. This is a local-model mismatch, not evidence of a unique mechanism.
Q5: Evidence proving distinct optimizer local minima: **NOT DEMONSTRATED**. Local secant failure is compatible with higher-order smooth nonlinearity, finite iterations, correspondence changes, termination effects, or other non-smooth terminal mapping behavior.

## Scientific answers and frozen status

- Q1: Partially, not uniformly. At α=0.5 strict-support raw accuracy is 0.9943, exactly the ALWAYS_ACCEPTED control (0.9943), with balanced accuracy 0.5; at α=1 secant accuracy is 0.9496 below the 0.9680 control, although balanced accuracy rises to 0.6946. Terminal response error P95 at α=1 is 0.0701 m and 1.765° (max 0.4088 m, 5.235°). This is same-frame cross-direction validation, not independent-scene validation.
- Q2: At α=0.5 acceptance is not better explained than by the trivial always-accepted prediction. At α=1 the local secant adds some reject discrimination (balanced accuracy 0.6946 vs 0.5), but loses on overall accuracy to the fixed baseline; therefore actual acceptance is only partially captured, not ‘mainly explained’ without qualification. The two primary radii have 79 strict-support label disagreements across 7 frames (P2F006, P2F024, P2F072, P2F073, P2F074, P2F075, P2F083).
- Q3: Yes, terminal-response errors increase substantially with extrapolation: translation P95 grows from 0.0353 m (α=.5) to 0.0701 (α=1), 0.168 (α=2), 0.296 (α=3); rotation P95 grows 0.885°, 1.765°, 3.352°, 5.318°. Acceptance accuracy is also below ALWAYS_ACCEPTED at α=2/3 (0.7912/0.6584 vs 0.9048/0.7521). These are exploratory extrapolations.
- Q4: Strict-support exit-radius disagreement counts and frame IDs are summarized above; each disagreement has raw bracket endpoint data in `exception_frames.csv`. No abruptness label is inferred without a predeclared numeric threshold.
- Q4 detail: Disagreements are tabulated with frame, ray/sign, seed, actual terminal, predicted terminal at the observed finite endpoint or α=3 censoring cap, training symmetry, bracket endpoint iteration/objective/fitness/convergence, predicted-response error, and raw terminal jump measurements. They are not assigned a unique mechanism; abruptness is not classified without a predeclared threshold.
- Q5: Distinct real optimizer local minima: **NOT DEMONSTRATED**.
- Q6: Independent nonlocal semantic evidence for frozen `m_B^op`: **NOT ESTABLISHED by this experiment**; the local-sensitivity null is challenged, not logically eliminated by any model error.

Frozen status remains: `U_obs=PARTIAL`; `U_nonlocal=SUPPORTED MATHEMATICAL CANDIDATE` (semantics pending); `Wrong-basin detector=NOT DEMONSTRATED`; `Dual Reliability=INCOMPLETE`; `Novelty=UNVERIFIED`.

All claims are limited to the frozen 24-frame sequence, fixed local secant at α=0.25, saved finite-direction responses, and fixed P6-I4 acceptance. No probability or independent-dataset inference is made.
