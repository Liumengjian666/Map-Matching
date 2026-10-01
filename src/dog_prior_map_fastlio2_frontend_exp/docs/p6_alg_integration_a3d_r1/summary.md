# PAPER-P6-ALG-INTEGRATION-A3D-R1

Targeted small-step status repair is correct, and real tx115 now completes.
The ONLY P3-200 replay stopped at a NEW first failure tx155 in marginalization.
No second issue was fixed and no retry/extra replay was performed.

| Gate | Result |
|---|---|
| A3D_R1_SMALL_STEP_TERMINATION_CODE_CONTRACT | PASS |
| A3D_R1_TX115_REPAIR_GATE | PASS |
| A3D_R1_P3_200_REAL_LINK_GATE | FAIL |
| READY_FOR_FORMAL_EXPERIMENT | NO |

## Git / validation

START_SHA `5e90b309c2612146a4ecbfdaff5becd3f6fcf7c8`; exact clean start on
`research/p6-i6d-full-algorithm`, workspace `/home/jian/livox_ws/dog_loc_p6_i6b_ws`.
CODE_SHA `2ecbe50552fa463fb59cb2250720325c87b829b6`.
END_SHA is the analysis delivery commit containing this document; full literal
SHA and actual push/remote state are in the final execution handoff.

Release build PASS, full CTest 32/32 PASS (original 31 retained), Debug targeted
10/10 PASS. A1-R1 information conservation, A2D Schur, A3B-R1 diagnostics and
forced clipped zero, A3B-R2 frozen projection, A3C-R1 diagnostics and A3C-R2
alias/real tx90 capsule remain PASS. `git diff --check` PASS. Detailed test
stdout and contract/negative controls are preserved in adjacent files.

No change to tolerances, strict candidate acceptance, initial damping, damping
updates, iterations, step clipping policy, sparse backend, NDT, IMU/noise,
U_obs/U_nonlocal, NIS, reliable rank, window size, Schur, PSD thresholds or
alias-safe symmetry. No artificial prior information/clamp was introduced.

## Determinism and real tx115

Seven real input hashes match frozen A3C-R2 identities. Actual V3 manifest gate
passed. Same raw input, IMU, normalized map, official params, initialization
1517157224188979000, profile corridor01, ADAPTIVE_SELECTED_NIS, visual NONE,
library path and diagnostic switches. Both live identity checks passed.
Additional complete prefix comparison found **zero differences**: 64 preopt
rows (excluding NDT runtime) and 982 optimizer rows including the first tx115
candidate (excluding only newly added fields) exactly match frozen A3C-R2.

tx115 stamp `1517157230686328484`; preopt 41 nodes, 2.0170772080000003 s.
NIS `15.237396865140097` > `15.086`; current LiDAR committed=0, unchanged.

Current cost `81.345724589884568`; candidate `81.345724589884583`.
Raw/applied norm `7.0568167407415232e-9`, clipped=false, accepted=false.
Termination `NATURAL_SMALL_STEP_NO_ACCEPTED_UPDATE`, optimizer
`CONVERGED_WITHOUT_STEP`; successful optimizer/feedback path reached.
Rejected candidate rollback max state difference **0**. Candidate was NOT
committed and candidate basis callback count stayed 0.

| tx115 removal | Nodes | Span after (s) | Stored symmetry | min eigenvalue | Jitter | Result |
|---|---|---|---|---|---|---|
| 1 | 41 → 40 | 2.017055584 | 0 | 0 | 0 | SUCCESS |
| 2 | 40 → 39 | 1.9162170890000001 | 0 | 0 | 0 | SUCCESS |

Duration enforcement completed, final span < 2 s. Existing tx90 also completed
both removals; its earlier alias repair was not changed or reopened.

## New failure: STOP, no second repair

tx155, `LIDAR_SCAN_END`, stamp `1517157234720485283`.
Wrapper error string: `producer_optimizer:marginalized_prior_not_finite_psd`.
Actual stage is **MARGINALIZATION_STAGE**, NOT optimizer rejection.
Optimizer `ACCEPTED_UPDATE`, 8 iterations, cost
`2.5682974886605456 → 2.5682916669943747` before enforcement failure.

Enforcement index 208, attempt1 FAIL, same-event previous successful removals0.
Recorded first bad stage `M7_SYMMETRIZED_SCHUR`; new prior candidate finite,
gradient finite, symmetry max defect0, minimum eigenvalue
`-97.635436582562278`, maximum `349971.31159981759`, recorded relative negative
`0.00027898125745291281`, jitter0. These are pre-existing diagnostics, NOT an
independent root-cause attribution or a newly approved solver change.

Rejected candidate prior was NOT stored. Prior hash, state stamp list and
factor counts stayed unchanged during failing enforcement, attempt1 removed
no state. There is no later-attempt partial commit in this observed failure.
Failed window remains 41 nodes / 2.017077923 s (exact raw row
in FIRST_FAILURE_marginalization.csv); completed-prefix bounds must not be
misrepresented as an enforced bound on this failed window.

Full matrix capsule saved independently of temporary run output:
`TX155_FAILURE_CAPSULE.npz`, 39,405 bytes, float64 matrices with metadata,
SHA256 `b44fe28654437ff1cd944945cb436f62368c7ff315e9e920686836950aed58a8`.
Its uncompressed source binary stays external; no large repeated matrix CSVs.
No follow-up oracle audit, extra real run, tuning or second-issue repair.

## Completed prefix / resources

Last completed terminal tx154: 103 LiDAR terminals, 104 window deskews including
tx155, 207 completed events. 75 LiDAR commits, 28 normal rejections/skips among
completed terminals. 168 successful oldest removals in 84 enforcement episodes,
then one failed attempt. Completed-event max nodes40 / span1.9589149060000002 s.
Recorded completed states finite and valid SO3; timestamps monotonic.

Pre-measurement covariance available84/unavailable19; first unavailable stamp
`1517157232804267479`. Those events retain the established fail-closed behavior:
no IKFoM P fallback and unavailable covariance does not enable selected NIS.
This is reported as an additional limitation, not repaired in this stage.

All terminal sources WINDOW_OWNED_SE3_DESKEW; post-handoff IKFoM calls0;
visual events/factors0; sparse fallback0; candidate basis callbacks0;
non-LiDAR covariance NOT_REQUESTED; production dense marginal reference not
used (existing source boundary/regressions, not a new counter claim).
No jitter in any observed marginalization attempt. Successfully stored prior
symmetry max0 and min eigenvalue0; failed candidate's negative eigenvalue is
reported separately, never labelled a stored prior value.

221 NDT calls in **completed runtime ledger rows**, not a grand total including
the failing tx155 terminal. Enabled diagnostics resource observation: wall
85.66 s, user81.89 s, system3.42 s, max RSS106424 KiB.
These are SHORT_REAL_LINK_ENGINEERING_SANITY, not a paper CPU benchmark.

GT used=false; no ATE/RPE, no accuracy claim, no P3-100/P3-400/full2777/Floor01,
no visual, no sweep. STOP condition triggered at new tx155 failure. All required
evidence is saved. Await scientific controller review; no next stage entered.
