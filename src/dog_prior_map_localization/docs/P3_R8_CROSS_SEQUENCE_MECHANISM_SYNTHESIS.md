# PAPER-P3-R8: Cross-sequence registration mechanism synthesis

Status: `PAPER-P3-R8-PARTIAL` — this is an `EXTERNAL-REVIEW SNAPSHOT`, not a final approved P3-R8 result. The report, CSVs, plots, and supporting Markdown are present in review snapshot commit `3c6532c58748f1c107297bbb1e3e2691f8184833`; this document records review corrections made after that snapshot.

`P4_ALLOWED = NO`

## Scope and input lineage

This is an offline synthesis of the formal Corridor01 R4/R5/R6 and Floor01 R7G/R7H/R7I evidence. No ROS bag was replayed. No NDT, predictor, limiter, deskew, visual, map, YAML, launch, or runtime source was changed. The frozen baseline remains `41999ea700c66c4cadf0eca9e0c5d73caa2783fd`.

The paper workspace was on branch `paper`, with both `HEAD` and `origin/paper` at `d6b985cac5830146e2613be8f8d38bd2d6c56a97` at task start. Existing paper dirty/deleted items were preserved. A status snapshot before the draft had SHA-256 `824170153ae12820f6692de74bd2623d88d492cc831169887b057b36e41ce1db`. The frozen-baseline worktree status snapshot at start was `1951ec111b6cf97e9f3d1b8409203f72b4143b17d4a138f391478d89674e58ff`.

The primary counterfactuals are neither phase-matched nor time-matched populations: Corridor01 R6 samples an early-failure window (W1+W2, 34 frames), while Floor01 R7I's primary comparison is a persistent/rapid-transition window union (W3+W4, 119 frames). They are different sequences and different failure regimes; they must not be described as a same-phase replication. R6 and R7I evaluate LiDAR-origin registration poses relative to their first eligible pair; the distinct R4/R7H full-run timeline origins are called out rather than silently mixed.

## Protocol audit

### Legacy R6 A–F classifier

The C branch called `C_ORACLE_MOVES_AWAY` actually means that the oracle endpoint lies outside the near-reference box. It does **not** mean oracle translation error is greater than baseline translation error. The E branch is shadowed by preceding B/C logic and is structurally unreachable. R8 therefore reports the A–F labels only as legacy descriptive output; primary comparisons use continuous endpoint errors, endpoint separation, and the explicitly named registration diagnostics.

### Local perturbation threshold

R6 narrative values `16/24, 0/24, 0/24` were reported alongside a `0.15 m / 2 deg` threshold, but the stored stable flag used `0.10 m / 1 deg`. Recomputing all raw endpoints under the common `0.15 m / 2 deg` rule gives Corridor01 `23/24, 22/24, 24/24` and Floor01 `24/24, 0/24, 0/24, 0/24` at its four anchors. The historical values are preserved; they are not mixed with the standardized counts. This is only local perturbation retention, not a basin, stationarity, Hessian, or mode-count test.

## Observation conditioning and counterfactual scope

Both fixed-cloud counterfactuals are conditional on translation deskew that uses prior NDT pose history:

- Corridor01's v2 adapter derives translation velocity from the two most recent completed prior `/dog_livo/ndt_odom` poses. The R6 per-frame comparisons hold the already-derived cloud fixed while changing only the NDT initial guess.
- Floor01 R7G/R7H use the two most recent completed prior NDT poses for causal full-SE(3) translation deskew. R7I likewise reuses the Run-A deskewed cloud for each independent initialization and does not regenerate later clouds under an oracle trajectory.

Thus both studies test **registration-level initialization sensitivity conditional on baseline/prior-NDT-conditioned observations**. Neither is an oracle closed-loop trajectory experiment. The counterfactuals do not establish what an oracle-initialized system would do to future deskew inputs.

## Phase mapping and evidence availability

| Sequence | Phase/window evidence | Registration counterfactual coverage |
|---|---|---|
| Corridor01 | Phase 0: R6 W0, 30 frames. Phase 1: R6 W1+W2, 34 frames. R4 additionally describes phase 2 (S4, 12–34.131 s; 220 reported frames) and phase 3 (S5+S6, from 34.131 s; 2,386 reported frames). | R6 tests only the reference/early interval through 6.5 s. Phase 2/3 have baseline prediction/raw/final timeline evidence but no oracle/perfect-increment counterfactual. |
| Floor01 | Phase 0: W0_PRE, 50. Phase 1: W1_TRANSIENT, 50. W2_P025, 60, straddles the persistent 0.25 m crossing at 38.223808 s. Phase 2: W3_P050, 70. W4_RAPID, 49, straddles the persistent 1 m crossing at 139.078133 s. | R7I has all three independent initializations in W0–W4; its fixed persistent union W3+W4 has 119 selected frames. |

The frozen Corridor01 phase/window table and all per-frame assignments are in external `p3_r8_phase_mapping.csv`. The reported phase-segment frame totals are retained with their source conventions and are not treated as exactly the same population as R6's selected counterfactual samples.

## Common evidence

1. **Repeatable baseline runs:** Corridor01 R4 reports 2,776 common Run-A/B NDT frames and zero maximum pose/fitness differences. Floor01 R7H reports 4,136 common outputs, zero translation-pose difference, `2.41483654e-6 deg` maximum rotation difference, and zero fitness difference. These repeatability gates establish repeatability of the tested runs, not physical cause.
2. **Sustained relative failure:** Corridor01 has persistent 0.5 m, 1 m, and 5 m crossings at 5.589692 s, 34.131304 s, and 35.341563 s (R3B/R4 conventions). Floor01 R7H has persistent 0.25 m, 0.5 m, 1 m, and 5 m crossings at 38.223808 s, 112.351752 s, 139.078133 s, and 140.590976 s. These origins and time axes are not directly aligned.
3. **Registration-level initialization sensitivity on tested fixed clouds:** Corridor01 R6 W1+W2 (early failure) has baseline/oracle/perfect-increment raw translation means `0.472037941176 / 0.138888064706 / 0.481060676471 m` (n=34); Floor01 R7I W3+W4 (persistent/rapid transition) has `1.535763 / 0.645010 / 1.493783 m` (n=119). Changing only each frame's initialization changes its raw NDT result and aggregate reference error in both tested windows. These are neither phase-matched nor time-matched populations.
4. **Oracle aggregate improvement is not endpoint preservation:** Corridor01 W1+W2 has paired baseline-minus-oracle mean/median `0.333149876471 / 0.32447305 m`; baseline endpoint translation error exceeds oracle on `34/34` frames. Oracle strong retention is `25/34`, recomputed from raw endpoint translation `<=0.15 m` AND rotation `<=2 deg`, not inferred from the legacy R6 B-label count. Floor01 W3+W4 baseline-minus-oracle paired mean is `0.890753 m` (95% paired-bootstrap CI `0.596239–1.207553 m`; median improvement `0.019740 m`).
5. **Perfect current increment alone does not recover the failure:** Corridor01 W1+W2 perfect-increment mean is `0.009023 m` worse than baseline. Floor01 W3+W4 improves by `0.041980 m` mean versus baseline, but its perfect-increment mean error remains `1.493783 m`. Therefore “not sufficient for recovery” is supported; “no effect” is not.
6. **Reported PCL registration scores are not interchangeable with reference-pose correctness:** in Floor01 W3+W4 all 119 runs converge. Aggregate means favor oracle in the PCL post-registration nearest-neighbor fitness score (`0.167364` vs `4.054451`, lower is better) and transformation probability (`1.603575` vs `1.437539`, higher is better), but paired per-frame wins are nearly balanced: fitness oracle/baseline `58/61`, probability `60/59`. PCL `getFitnessScore` is not the exact NDT optimized likelihood. R6 likewise shows score preferences that differ by metric. This does not establish a wrong mode or multimodality.
7. **Limiter activity is observable, not causal evidence:** Floor01 R7H logs translation limitation on `237/4136`, rotation limitation on `773/4136`, either on `845/4136`. Corridor01's early intervals mostly have no translation limiting; the R4 late interval has activity. No shared limiter ablation or causal-dominance result exists.
8. **The predictor evidence is asymmetric:** Corridor01 R4/R5 show that its baseline initial guess already carries substantial error at the formal 0.5 m crossing (`0.508456 m`) while raw NDT correction is `0.008760 m`; R5 supports a descriptive previous-NDT-delta reuse / insufficient-correction pattern in W1, but leaves initial onset unresolved and does not prove unstable feedback. Floor01 only has the logged initial-guess error in R7I, not a full R4/R5 predictor/carry decomposition.

## Sequence-specific conclusions

### Corridor01 adds

R6 reports `52/65` oracle-initialized endpoints and `20/65` baseline endpoints inside the joint `0.15 m / 2 deg` near-reference box. In W1+W2, oracle strong retention is `25/34`, computed directly by standardized raw endpoint recomputation (translation `<=0.15 m` AND rotation `<=2 deg`), not inferred from legacy B labels. Baseline endpoint translation error exceeds oracle on `34/34` frames; baseline/oracle/perfect raw translation means are `0.472037941176 / 0.138888064706 / 0.481060676471 m`, with paired baseline-minus-oracle mean/median `0.333149876471 / 0.32447305 m`. Standardized local perturbation counts are: Corridor formal 0.25 m anchor `23/24`; Corridor LiDAR 0.5 m anchor `22/24`; Corridor formal 0.5 m anchor `24/24`. These are early-window/local results only; no inference is made about the later persistent phase, which has no R6 counterfactual coverage.

### Floor01 limits generalization of the Corridor01 pattern

In the predeclared persistent/rapid-transition W3+W4 union, **none** of the 119 oracle endpoints is within `0.15 m / 2 deg`, despite a positive paired mean translation improvement. The improvement is concentrated in W4: W3's mean baseline-minus-oracle gain is `0.022210 m`, while W4's is `2.131530 m`. Floor standardized local perturbation counts are: A1 instantaneous 0.5 m `24/24`; A2 persistent 0.25 m `0/24`; A3 persistent 0.5 m `0/24`; A4 persistent 1.0 m `0/24`. This limits generalization of the Corridor01 early-window oracle-retention pattern to a different sequence and later failure regime; it does not show phase-matched populations or disprove a Corridor mechanism.

## Candidate hypotheses (not root-cause findings)

1. **H1 — Initialization-sensitive registration.** Supported as a registration-level description in both tested fixed-cloud windows. Contradiction/boundary: Corridor's early oracle retention does not replicate in Floor's persistent union. Missing discriminator: apply the same fixed-cloud three-initialization probe to predeclared Corridor persistent-phase frames.
2. **H2 — Registration limitation beyond initialization.** Floor's persistent oracle endpoints remain far from reference, and reported PCL registration scores are not uniformly aligned with reference error. Observation quality, geometry, objective landscape, and score behavior remain unresolved candidate interpretations. Boundary: Corridor often retains near-reference oracle endpoints early; no evidence isolates any candidate as the cause. Missing test: a phase-matched frozen-cloud comparison using existing online observables; no Hessian/mode claim is authorized. The current Corridor/Floor primary windows themselves are not phase- or time-matched.
3. **H3 — Prediction-correction coupling.** Corridor R4/R5 support a descriptive onset/maintenance chain in part of that sequence; Floor's perfect increment yields only a small persistent-union improvement. Boundary: Floor predictor/carry decomposition is absent, and Corridor onset remains unresolved. Missing test: offline decomposition of existing Floor records before proposing a predictor change.

Details and supporting/contradicting evidence are in external `p3_r8_candidate_hypotheses.md`.

## Online-observable audit and P4 gate

Fitness, iteration count, convergence, initial-to-raw NDT displacement, limiter flags, IMU motion, and source geometry are GT-independent signals available or derivable in runtime. Their reliability thresholds are not validated by R8. Transformation-probability and PCL registration-score samples were available in offline probes but are not consistently exposed by the frozen runtime (Floor R7H records them as unavailable). GT error, oracle initialization/retention, and perfect-increment error are strictly offline and cannot be runtime policy inputs.

No mechanism currently satisfies all three required conditions—support in both sequences, an online-observable signature, and GT independence. Physical cause remains `UNRESOLVED`; wrong mode, multimodality, and Hessian degeneracy remain `NOT_PROVEN`. “Fix the predictor” as the sole paper solution is `NOT_SUFFICIENTLY_SUPPORTED`.

## Decision and next recommendation

Recommend exactly **`P3-R8A-MINIMAL-DISCRIMINATIVE-EXPERIMENT`**. On already frozen Corridor01 observations, predeclare a small set of persistent-phase anchors and repeat the same per-frame baseline/oracle/perfect-increment comparison; retain Floor01 W3/W4 as the contrasting boundary. This tests whether Corridor01's early oracle-retention pattern persists into later failure without adding a dataset or changing runtime. It is a proposed offline experiment only, not algorithm implementation.

`P4_ALLOWED = NO`. No algorithm object is selected for modification in R8. The next experiment should distinguish initialization effect from observation/objective limitation before any predictor, NDT, or limiter change is proposed.

## Artifacts and verification

External numeric artifacts and plots are in:

`/media/jian/HIKVISION/paper rosbag/cross_sequence_analysis/p3_r8/`

The external-review snapshot commit `3c6532c58748f1c107297bbb1e3e2691f8184833` contains the evidence matrix, phase mapping, per-frame Corridor01/Floor01 metrics, oracle-retention ledger, NDT-displacement ledger, grouped cross-sequence summary, candidate hypotheses, online-observable audit, summary, and four plots. A supplementary CSV standardizes local perturbation counts. These are review artifacts, not final scientific approval.

Paper-workspace runtime paths `src/`, `include/`, `config/`, and `launch/` have no diff from the external-review snapshot. The frozen baseline HEAD and its existing user work are preserved. R8 remains diagnostic/offline; `P4_ALLOWED = NO`.
