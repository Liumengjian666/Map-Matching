# P62-R15 Coupled Subspace Refinement R1

## Result

**REAL_LOCAL_MODEL_NOT_ESTABLISHED; NO_MEANINGFUL_POSE_REFINEMENT.** The exact configured PCL Gaussian score was evaluated on the 25 frozen frames, but every frame failed the predeclared local-branch gate because a fixed finite-difference stencil changed at least one point's Gaussian-leaf association. The normalized geometric matrix had no direction at or below the frozen 0.01 weak-direction ratio on any frame. Consequently no real `Hss`, `Hsw`, `Hww`, `ds`, weak-only pose, or coupled pose was produced. The output candidates are unavailable and remain non-converged/non-eligible by construction.

The synthetic mathematical test suite passed. That verifies the coded formulas and safety gates on controlled smooth inputs; it is not evidence of a real coupled response or localization improvement.

## Identity and scope

- Repository/worktree: `/home/jian/livox_ws/p62_r15_coupled_subspace_ws`
- Branch: `research/p62-r15-coupled-subspace-refinement-r1`
- Parent baseline: `fae8937cd72e8272111186e572c346bb76f92a4e`
- R15 algorithm `CODE_SHA` (canonical SHA-256 over sorted `tools/CMakeLists.txt`, `coupled.hpp`, `run_coupled.cpp`, `test_coupled.cpp` path/hash lines): `322dd58ee84b7f25aae745e13e535e070cfa82ee5f7809c9f0e8c6b1f97dfe30`
- Binary SHA-256: `run_coupled` `f100c9d7e84a3d0aa0c615712d1841f59eb5f9b069297698cc258d7653258acc`; `test_coupled` `b5eaa859e7fe403c6d57665ec5ec60d892108e4d6fce338821014911deda2e89`.
- Inputs: five frozen R13 GEODE events (scan indices 60–64) and twenty frozen R14B Newer College evaluation frames (raw ordinals 768–787), with their existing T0, TP, processed Source, and maps. No new NDT alignment, rosbag replay, trajectory replay, or EKF run.
- The source/map identities, prior archives, and stable production worktree were snapshotted before and after the local model run. The after snapshot matched; both R13/R14B archived checksum lists verified. See `INPUT_FREEZE.json` and `ASSET_IDENTITY_AFTER.json`.

## Method and frozen model

The right/body perturbation is `T(z)=T0 Exp(Sz)`, with `S=diag(0.30 m,0.30 m,0.30 m,3°,3°,3°)` (angles represented in radians). The exact negative normalized PCL 1.10 Gaussian score is differentiated by fixed finite differences only while every training point retains the same ordered set of Gaussian-leaf associations. `G` is the normalized Gaussian-weighted Gauss–Newton information matrix, not the optimizer Hessian. The weak rule remains `lambda_i/lambda_max <= 0.01`; at most two directions could have been prediction-guided. The fixed coupled solve was `(Hss + 0.01 I) ds = -(gs + Hsw dw)`, with the same `dw` as the weak-only ablation and a unit normalized trust ball. Full formulas and gates are in `MATHEMATICAL_FORMULATION.md`.

## Real-input findings

- All 25 exact-score parity checks matched the PCL derivative score (`absolute difference = 0`).
- The PSD/orthogonality checks passed for the reported normalized `G` matrices; eigenvalue condition numbers ranged from `29.6649` to `69.0236`.
- Minimum-to-maximum normalized eigenvalue ratios ranged `0.0144878–0.0337099`; all 25 frames therefore had `weak_count=0` under the frozen 0.01 rule.
- All 25 derivative attempts returned `LOCAL_ASSOCIATION_SET_CHANGED`, starting at the first tested central-difference stencil. No gradient/Hessian passed; all real `Hsw` entries are `NA`.
- R13 scan 60 had a historical weak direction in the earlier unnormalized R11/R9 geometry (`7.991523/809.730786 ≈ 0.00987`). R15's physically scaled `G_z` ratio for the same frozen frame is `0.0980028/3.935038 ≈ 0.02491`. This is a unit/metric-dependent eigenvalue ratio, not proof that either calculation is numerically the same diagnostic. R15 retained the prescribed 0.01 rule in its new normalized coordinates.
- Direct versus R6-converted geometry support counts matched on all frames, but the relative matrix discrepancy exceeded the pre-run `1e-7` gate on 11/25 frames (maximum `2.27121e-7`). No tolerance was changed. The association failure already prevented refinement on every frame; the conversion discrepancy is an additional unresolved check on 11 frames.
- T0 was formally eligible on all 20 Newer College frames and on 0/5 R13 events. Since the refinements were unavailable, there was no alternative candidate to enter the R12 pool; no robust pairwise `D` was available and no observation was accepted as a refinement.

## Newer College reference-relative context

After the candidate/model outputs were frozen, the fixed 20 reference poses were evaluated against T0 and TP. This reference is `MAP_ANCHORED_BENCHMARK_REFERENCE`, under the conditional `PAPER_SUPPORTED_BASE_FRAME_ASSUMPTION`; it is not independent motion-capture truth, and file-level CSV axis binding remains uncertified. Results: TP translation RMSE/median/P95/max `0.12708/0.10040/0.17437/0.35023 m`, rotation `1.9991/2.0116/2.7220/3.3639°`; T0 translation `0.09562/0.08661/0.13886/0.15851 m`, rotation `1.7068/1.7292/2.0834/2.5175°`. There are no weak-only or coupled pose errors because neither pose was available.

**Non-blind limitation:** before R15 candidate freeze, three rows from the prior R14B `REFERENCE_ERROR.csv` had inadvertently been inspected for summary context. No raw registered pose values or R15 output were read then, and no frame, alpha, direction, gate, or result was selected from those rows. Nevertheless the R15 reference comparison is not a clean blind/confirmatory evaluation. The post-freeze reference calculation is reported transparently as reference-relative, exploratory context only.

## Cost and execution boundary

The single run completed in `1.54 s` wall time with peak RSS `79,040 KiB`; native NDT `align` calls `0`, EKF runs `0`, GT pose-value reads during the run `0`. Across 25 unique input frames, the per-frame local process measurement averaged `48.37 ms` (linear P95 `64.22 ms`, max `70.26 ms`); per-frame Gaussian-grid initialization averaged `36.20 ms`, while the local derivative check averaged `10.05 ms`. Refinement evaluation was not reached. For context only, the archived R14B timing measured full NDT at mean `11.22 ms` and split NDT at mean `5.14 ms`; this R15 precheck is not lower than two full alignments on that old timing, and does not establish a low-cost successful coupled method. Per-frame grid initialization may be reusable in a future implementation, but this run is not evidence for that optimization.

## Gate assessment

| Question | Result |
|---|---|
| H1: usable real strong/weak coupling information? | No. No frame had a qualifying weak direction; the exact local derivative branch also failed on all 25. |
| H2: bounded nonzero strong response? | Not evaluable. No real blocks or response were produced. |
| H3: actual training objective improvement over weak-only? | Not evaluable; neither correction was available. |
| H4: held-out improvement? | Not evaluable; all refinement losses and pairwise evidence are `NA`. |
| H5: reference-relative coupled error improvement? | No coupled pose exists. T0 is numerically better than TP in this 20-frame reference-relative comparison, but comparison is non-blind and the reference is map-anchored/conditional. |
| H6: cost below repeated NDT? | Not established; the measured unsuccessful precheck overhead is not below two archived full-alignment times, and successful refinement cost was not measured. |

The strongest supported claim is `MATHEMATICAL_PROTOTYPE_ESTABLISHED` on synthetic tests only. The real-data status is `MATHEMATICAL_MODEL_NOT_ESTABLISHED_ON_FROZEN_FRAMES`; `COUPLED_RESPONSE_OBSERVED` and `COUPLED_GEOMETRIC_BENEFIT_OBSERVED` are not supported. There is no evidence of accuracy improvement or scientific novelty.

## Files and review

CSV outputs are in `run_01/`; the reference-relative table is at the archive root. `CANDIDATE_OUTPUT_FREEZE.json` seals all candidate/model outputs before opening R14 reference pose values. `REFERENCE_ACCESS_ADMISSION.json` and `REFERENCE_EVALUATION_COMPLETE.json` record the later evaluation. `PROCESS_EXCEPTIONS.md` records two corrected procedural failures and the post-freeze reference-tool-only fix. `MATHEMATICAL_TESTS.md`, `IMPLEMENTATION_DIFF.md`, `RESULTS_AND_LIMITATIONS.md`, and `results.json` provide details.

**Unique NEXT:** pause this formulation and ask the scientific controller to decide whether the hard-truncated PCL Gaussian leaf association makes the proposed exact-score local derivative method unsuitable; if continued, a new approved contract must first specify a mathematically faithful treatment of association boundaries rather than loosening this run's frozen gates.
