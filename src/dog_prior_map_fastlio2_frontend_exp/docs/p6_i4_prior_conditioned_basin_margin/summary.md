# PAPER-P6-I4-PRIOR-CONDITIONED-BASIN-MARGIN-VIABILITY

Result: `PRIOR_CONDITIONED_MARGIN_SUPPORTED`

`SUPPORTED` here means numerical and operational viability under the frozen Floor01 finite-direction protocol only. It does not establish a topology-exact attraction basin, correctness/failure probability, calibrated uncertainty, cross-dataset validation, runtime readiness, or a complete `U_nonlocal` estimator.

## P6-I4-R1 claim/math closure

- R1: global equivalence-label notation removed; replaced by a nominal-reference operational acceptance set — **PASS**.
- R2: covariance wording corrected; the pose marginal retains rotation-position cross covariance only — **PASS**.
- R3: GT provenance narrowed; the estimator is GT-free, while prospective GT-independence of the historical cohort is not claimed — **PASS**.
- R4: Gate E labeled shared-ray internal semantic consistency; no held-out validation claim — **PASS**.
- R5: Gate D labeled implementation consistency — **PASS**.
- R6: the supported verdict is restricted to numerical/operational viability under the frozen Floor01 finite-direction protocol — **PASS**.

## Frozen inputs and estimator definition

- Workspace branch / start commit: `paper` / `1beb3863777df13b32c4117151fbd503598ab16f`.
- Frozen P5-I2 manifest: 88 frames; SHA-256 `c520bb0232f98b3e5eb65528d14ca5732ca62291e560ed9cac285fc24b83a291`. Dense cohort: 24 pre-frozen frames. Extra directions: NumPy PCG64 seed `20260928`, SHA-256 `9ab601004cfcbafe3fc54987b851b48b5143e469328ee17002b53d082f2fe118`.
- Exact frozen map SHA-256: `2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570`; raw bag SHA-256 `860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db`. Frozen map target preprocessing yields 549,606 points; PCL NDT resolution 0.8m, step 0.08, epsilon 0.001, maximum iterations 40. No map/bag/cloud asset is included in the repository.
- Nominal registration map: `registrationMap(T_minus)`, the terminal PCL NDT pose from the formal closed-loop baseline seed.
- Pose tangent: map product tangent `[dphi_map, dp_map]`, with `R_seed=Exp(dphi_map)R_pred`, `p_seed=p_pred+dp_map`; not a coupled SE(3) twist.
- Prediction covariance: the 6x6 pose covariance is projected from the full pre-NDT IKFoM prediction covariance using `J_pose P_state J_pose^T`, rather than formed from independently diagonalized rotation/translation blocks. `J_pose` has nonzero columns only for pose rotation and position, so the resulting marginal retains rotation-position cross covariance; velocity/bias/gravity/extrinsic-to-pose cross terms do not directly appear in the pose marginal. Right/body SO(3) state error is mapped by `R_est`; position error is map-additive.
- Nominal reference: `M_0=R_k(T^-)`, the terminal PCL NDT pose from the formal closed-loop baseline seed. `A_k(delta;M_0)=1` iff the probe converges and terminal translation/rotation separations from fixed `M_0` are `<=0.20m` / `<=2.0deg`; otherwise it is zero. This fixed-reference tolerance is not a transitive pairwise equivalence relation and induces no global mode label.
- Operational nominal-basin set: `B_0^op={delta | A_k(delta;M_0)=1}`. **Prior-Conditioned Operational Nominal-Basin Margin**: ideal margin `m_B^{op,*}=inf_(delta notin B_0^op) sqrt(delta^T P_pose^dagger delta)`. This is an operational stability margin and proxy for attraction-basin stability, not a strict attraction basin or topology-exact global boundary.
- Estimators: `m_principal` (principal operational margin estimator) and `m_dense` (dense directional reference for `m_B^{op,*}`) are finite-direction approximations only.
- Estimator search: principal ± covariance eigen-directions and 32 fixed extra whitened directions with both signs, coarse alpha step 0.25 over [0,3], first observed accepted-to-rejected bracket, bisection width <=0.01, report upper endpoint; high values are censored `>3`.

## Covariance audit and baseline replay

| Gate | Result |
|---|---|
| Error-state FD convention | PASS; 10 states / 60 columns; max abs error `2.97580537e-06` (limit `1e-5`) |
| Finite PSD covariance contexts | 88/88 = 100.00% (required >=99%) |
| Effective rank distribution | `{"6": 88}` |
| Closed-loop replay vs frozen P6-I2 full trajectory | PASS; max pose delta `0m`, `0deg` |
| Baseline post-hoc GT metrics | translation RMSE `21.936133m` (P6-I2 `21.936133`); rotation RMSE `9.906078deg` (P6-I2 `9.906078`) |

## Principal margin and dense directional reference

- Broad principal frames: 88; finite `75`; censored above 3: `13`; finite median / P10 / P90 = `1.7109` / `0.3844` / `2.5891` prior-metric units.
- Dense directional reference frames: 24; finite `20`; censored above 3: `4`; finite median `1.6211`.
- Gate D implementation consistency: the dense set contains the principal rays; violations of `m_dense <= m_principal + 0.01`: `0`.
- Principal-vs-dense fidelity on finite pairs: median ratio `1.0000`, P90 `1.2094`, missed finite dense boundaries while principal censored `0`.
- Principal estimator verdict: **GOOD** (separate from mathematical margin verdict; thresholds median<=1.25, P90<=1.50, misses<=2).
- Reproducibility: `10/10` audit frames pass; censored/censored repeats are explicitly counted as same censored state with equal reported cap, not as a detected boundary.

## Retention relationship and negative control

- Gate E — shared-ray internal semantic consistency: Spearman `m_dense` vs `S(1)` = `0.8500` (`n=20`); vs `S(2)` = `0.9697` (`n=20`). Required both >=0.50: **PASS**. Both use the same extra-ray sample; this is not held-out, independent, or external validation. Descriptive cap-at-3 sensitivity including four censored frames preserves the qualitative association (`S1≈0.804`, `S2≈0.962`); sensitivity only, not the gate.
- Principal margin vs retention is descriptive: Spearman with `S(1)` `0.8106` (`n=20`), with `S(2)` `0.9476` (`n=20`). The principal rays do not overlap the extra-ray retention sample, but this is not a fully independent experiment.
- Gate F — healthy multimodality counterexamples: `2` matched frozen controls with `K>=2`; both have `m_dense>=1` and `S(1)>=0.90`. This demonstrates existence of healthy multimodal counterexamples in these two controls only; it does not establish statistical independence or orthogonality. Raw multimodality need not imply nearby operational instability in these controls: **YES**.

## U_obs relation (descriptive only)

- P6-I3 BLOCK translation-block minimum eigenvalue vs principal margin Spearman `-0.5000` (`n=7`).
- BLOCK rotation-block minimum eigenvalue vs principal margin Spearman `0.1429` (`n=7`). See `margin_vs_uobs.csv` for min eigenvalues and condition numbers. This is descriptive; no orthogonality claim is made.
- `U_obs = PARTIAL`; `U_nonlocal = SUPPORTED MATHEMATICAL CANDIDATE`; dual reliability complete: **NO**.

## GT post-hoc diagnostics only

- Full trajectory persistent error crossings (5s persistence, sampled gaps <=0.25s): 0.5m `84.91932821273804`, 1m `93.5928385257721`, 2m `151.48321318626404`, 5m `157.43361377716064`.
- Strict GT overlap: 4126/4127 baseline scans; the final scan beyond official GT support is omitted, without extrapolation.
- Principal margin vs current corrected baseline translation error Spearman `-0.4563` (`n=75`); vs next-5s max error increase `DeltaE5` `-0.2445` (`n=75`). These are descriptive, not gates/classifiers.
- Pose NEES uses `e=[Log(R_GT R_pred^T),p_GT-p_pred]` and the unscaled pseudoinverse of `P_pose`; median `533.7583` over valid rows. Covariance was not rescaled using GT. Large NEES means the radius is only a filter-reported prior metric, not a calibrated probability.
- The first switched terminal pose's relative GT error is descriptive only. No GT pose/error values were read by the P6-I4 estimator, direction generation, NDT probing, operational boundary search, retention computation, or A--F verdict. Frame identities and strata were inherited from the frozen P5-I2 manifest; therefore this experiment does not claim prospective GT-independence of the historical cohort design. `preparation_manifest.json`'s `gt_read=false` means only that the P6-I4 preparation script did not itself open GT; the inherited manifest can contain historical GT-derived metadata. The frozen `margin_with_gt_posthoc.csv` is unchanged; its legacy `gt_use` cohort wording is superseded by this provenance statement.
- Official GT SHA `b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f`; calibration SHA `fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414`; frozen alignment provenance SHA `ff61f3fc72ec2b0c4c9e7a99f54e0866d696bb8999cf1f7a16001cacd3a26416`.

## Runtime and memory (not a realtime gate)

- Formal baseline replay: 4,127 NDT calls, replay `44075.932ms`, peak RSS `92.00MiB`.
- Principal/dense search made `32376` actual probe alignments over 88 frames: principal `13013`, extra margin `18546`, and retention `817`; repeatability made `1439` calls over 10 frames. The run log's legacy `principal_dense_calls=32464` included an extra nominal-seed count per frame; nominal M0 was already computed in the baseline's 4,127 calls. Corrected source now reports these categories separately. Principal/dense per-frame runtime mean / P95 / max `4389.4390` / `12475.3409` / `19911.4915ms`. `search_accounting_audit.csv` reconciles the per-probe rows and runtime counts.

## Prior-art and claim boundary

Mature prior art includes initialization-dependent registration uncertainty, multiple initial poses / multi-start NDT, uncertainty propagation through nonlinear registration, multi-NDT mode covariance, and Hessian-guided seed arrangements. The only candidate distinction under study is a **prior-conditioned operational nominal-basin margin as a reliability coordinate, explicitly separated from local observability**. `NOVELTY_UNVERIFIED`; no “first/novel” claim is made.

## Gates and verdict

| Gate | Outcome |
|---|---|
| A: covariance convention FD <=1e-5 | PASS |
| B: finite covariance >=99% | PASS |
| C: at least 10 repeatability frames stable within .01 | PASS |
| D: nested dense/principal rays agree within .01 — implementation consistency | PASS |
| E: dense margin/retention Spearman >=.50 — shared-ray internal semantic consistency | PASS |
| F: healthy K>=2 counterexample exists in two matched controls | PASS |

Final margin verdict: **PRIOR_CONDITIONED_MARGIN_SUPPORTED**. `SUPPORTED` means numerical and operational viability under the frozen Floor01 finite-direction protocol. This is a single-sequence result, not a correctness/failure probability, calibrated uncertainty, topology-exact attraction basin, cross-dataset validation, runtime risk predictor, mitigation, or complete `U_nonlocal` estimator.

## Scientific state after P6-I4

- `U_obs = PARTIAL`: local BLOCK curvature has synthetic directional feasibility, but no real degeneracy ground truth.
- `U_nonlocal = SUPPORTED MATHEMATICAL CANDIDATE`; primitive: `m_B^{op,*}`. Meaning: filter-prior-metric distance to the first detected exit from the nominal terminal-pose operational acceptance set (represented here by finite directional estimates).
- Dual Reliability: **INCOMPLETE**. Novelty: **NOVELTY_UNVERIFIED**.

## Implementation checks

- Release C++ experiment target: PASS.
- Product-tangent / mode-threshold / covariance-whitening math tests: PASS (`P6_I4_MATH_TEST_PASS`).
- Frozen P6-I4 Python report generation: PASS. R1 report wording/template syntax is checked separately; no estimator or NDT rerun is part of R1.
- `git diff --check`: PASS.

Limitations: operational mode tolerances; 0.25 alpha coarse grid can miss narrow switch-and-return regions; finite 32-direction reference; dense-margin/retention correlation reuses the same extra-ray set; alpha cap 3; filter covariance calibration unknown; Floor01 only; frozen deskew clouds; no visual, mitigation, or runtime integration.
