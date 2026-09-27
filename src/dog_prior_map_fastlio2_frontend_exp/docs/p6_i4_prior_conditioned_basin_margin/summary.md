# PAPER-P6-I4-PRIOR-CONDITIONED-BASIN-MARGIN-VIABILITY

Result: `PRIOR_CONDITIONED_MARGIN_SUPPORTED`

## Frozen inputs and estimator definition

- Workspace branch / start commit: `paper` / `1beb3863777df13b32c4117151fbd503598ab16f`.
- Frozen P5-I2 manifest: 88 frames; SHA-256 `c520bb0232f98b3e5eb65528d14ca5732ca62291e560ed9cac285fc24b83a291`. Dense cohort: 24 pre-frozen frames. Extra directions: NumPy PCG64 seed `20260928`, SHA-256 `9ab601004cfcbafe3fc54987b851b48b5143e469328ee17002b53d082f2fe118`.
- Exact frozen map SHA-256: `2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570`; raw bag SHA-256 `860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db`. Frozen map target preprocessing yields 549,606 points; PCL NDT resolution 0.8m, step 0.08, epsilon 0.001, maximum iterations 40. No map/bag/cloud asset is included in the repository.
- Nominal registration map: `registrationMap(T_minus)`, the terminal PCL NDT pose from the formal closed-loop baseline seed.
- Pose tangent: map product tangent `[dphi_map, dp_map]`, with `R_seed=Exp(dphi_map)R_pred`, `p_seed=p_pred+dp_map`; not a coupled SE(3) twist.
- Prediction covariance: full pre-NDT IKFoM state covariance projected by `J_pose P_state J_pose^T`, including orientation-position cross covariance; right/body SO(3) state error is mapped by `R_est`, position error is map-additive.
- Operational mode equivalence: both NDT runs converge and terminal translation separation `<=0.20m` AND rotation separation `<=2.0deg`; objective/fitness do not define a mode.
- Ideal object: `m_B*=inf_(delta:not in B0) sqrt(delta^T P_pose^dagger delta)`; computed estimates are finite directional operational approximations, not a topology-exact boundary or correctness likelihood.
- Estimator search: principal ± covariance eigen-directions and 32 fixed extra whitened directions with both signs, coarse alpha step 0.25 over [0,3], first observed same→different bracket, bisection width <=0.01, report upper endpoint; high values are censored `>3`.

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
- Dense/principal consistency: violations of `m_dense <= m_principal + 0.01`: `0`.
- Principal-vs-dense fidelity on finite pairs: median ratio `1.0000`, P90 `1.2094`, missed finite dense boundaries while principal censored `0`.
- Principal estimator verdict: **GOOD** (separate from mathematical margin verdict; thresholds median<=1.25, P90<=1.50, misses<=2).
- Reproducibility: `10/10` audit frames pass; censored/censored repeats are explicitly counted as same censored state with equal reported cap, not as a detected boundary.

## Retention relationship and negative control

- Dense margin vs extra-direction retention: Spearman `m_dense` vs `S(1)` = `0.8500` (`n=20`); vs `S(2)` = `0.9697` (`n=20`). Required both >=0.50: **PASS**.
- Independence caveat: retention uses extra directions rather than principal rays, but `m_dense` also searches those same extra rays. This is a protocol-defined consistency association, not a held-out independent validation; the specified numerical threshold is met, with this semantic limitation.
- Principal margin vs retention is descriptive: Spearman with `S(1)` `0.8106` (`n=20`), with `S(2)` `0.9476` (`n=20`).
- Healthy P5-I1 multimodal controls (K>=2) in the frozen healthy dense cohort: `2`. Controls with `m_dense>=1`: `2`; with `S(1)>=0.90`: `2`. H3 negative-control gate: **PASS**. Raw global multimodality does **not** automatically imply nearby prior-conditioned basin instability: **YES**.

## U_obs relation (descriptive only)

- P6-I3 BLOCK translation-block minimum eigenvalue vs principal margin Spearman `-0.5000` (`n=7`).
- BLOCK rotation-block minimum eigenvalue vs principal margin Spearman `0.1429` (`n=7`). See `margin_vs_uobs.csv` for min eigenvalues and condition numbers. This is descriptive; no orthogonality claim is made.
- `U_obs = PARTIAL`; `U_nonlocal = SUPPORTED CANDIDATE`; dual reliability complete: **NO**.

## GT post-hoc diagnostics only

- Full trajectory persistent error crossings (5s persistence, sampled gaps <=0.25s): 0.5m `84.91932821273804`, 1m `93.5928385257721`, 2m `151.48321318626404`, 5m `157.43361377716064`.
- Strict GT overlap: 4126/4127 baseline scans; the final scan beyond official GT support is omitted, without extrapolation.
- Principal margin vs current corrected baseline translation error Spearman `-0.4563` (`n=75`); vs next-5s max error increase `DeltaE5` `-0.2445` (`n=75`). These are descriptive, not gates/classifiers.
- Pose NEES uses `e=[Log(R_GT R_pred^T),p_GT-p_pred]` and the unscaled pseudoinverse of `P_pose`; median `533.7583` over valid rows. Covariance was not rescaled using GT. Large NEES means the radius is only a filter-reported prior metric, not a calibrated probability.
- The first switched terminal pose's relative GT error is recorded as descriptive only; mode/censoring/margin decisions were fully frozen before GT was read. **No GT was used in the estimator, cohort selection, direction generation, NDT mode search, or retention.**
- Official GT SHA `b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f`; calibration SHA `fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414`; frozen alignment provenance SHA `ff61f3fc72ec2b0c4c9e7a99f54e0866d696bb8999cf1f7a16001cacd3a26416`.

## Runtime and memory (not a realtime gate)

- Formal baseline replay: 4,127 NDT calls, replay `44075.932ms`, peak RSS `92.00MiB`.
- Principal/dense search made `32376` actual probe alignments over 88 frames: principal `13013`, extra margin `18546`, and independent retention `817`; repeatability made `1439` calls over 10 frames. The run log's legacy `principal_dense_calls=32464` included an extra nominal-seed count per frame; nominal M0 was already computed in the baseline's 4,127 calls. Corrected source now reports these categories separately. Principal/dense per-frame runtime mean / P95 / max `4389.4390` / `12475.3409` / `19911.4915ms`. `search_accounting_audit.csv` reconciles the per-probe rows and runtime counts.

## Prior-art and claim boundary

Mature prior art includes initialization-dependent registration uncertainty, multiple initial poses / multi-start NDT, uncertainty propagation through nonlinear registration, multi-NDT mode covariance, and Hessian-guided seed arrangements. The only candidate distinction under study is a **prior-conditioned nearest operational attraction-basin margin as a reliability coordinate, explicitly separated from local observability**. `NOVELTY_UNVERIFIED`; no “first/novel” claim is made.

## Gates and verdict

| Gate | Outcome |
|---|---|
| A: covariance convention FD <=1e-5 | PASS |
| B: finite covariance >=99% | PASS |
| C: at least 10 repeatability frames stable within .01 | PASS |
| D: dense <= principal + .01 | PASS |
| E: dense margin Spearman with S1 and S2 >=.50 | PASS |
| F: healthy K>=2 with m_dense>=1 or S1>=.90 | PASS |

Final margin verdict: **PRIOR_CONDITIONED_MARGIN_SUPPORTED**. This is a single-sequence, finite directional, operational-mode viability study—not a correctness probability, runtime router, mitigation, or calibrated uncertainty claim.

## Implementation checks

- Release C++ experiment target: PASS.
- Product-tangent / mode-threshold / covariance-whitening math tests: PASS (`P6_I4_MATH_TEST_PASS`).
- Python report syntax and complete report generation: PASS.
- `git diff --check`: PASS.

Limitations: operational mode tolerances; 0.25 alpha coarse grid can miss narrow switch-and-return regions; finite 32-direction reference; dense-margin/retention correlation reuses the same extra-ray set; alpha cap 3; filter covariance calibration unknown; Floor01 only; frozen deskew clouds; no visual, mitigation, or runtime integration.
