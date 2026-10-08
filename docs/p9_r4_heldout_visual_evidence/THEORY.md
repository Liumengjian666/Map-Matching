# P9-R4 preregistered held-out ambiguity-evidence contract

This is **ambiguity-evidence held-out validation**, not visual-frontend held-out
validation. DUAL-U remains the scientific architecture: within-basin directional
observability plus inter-basin/competing-attractor ambiguity. No pose switching,
EKF or covariance fusion is authorized. The 32 historical targets are permanently
development-only; their target +/-8 transaction windows are excluded.

## Selection, provenance and gates

Baseline transaction IDs in [9,4118] are ranked by hexadecimal SHA256 of
`P9_R4_FLOOR01_HELDOUT_V1_20261008:` concatenated with the decimal ID. Greedy
acceptance requires target separation >=16 transactions. The first 160 accepted
IDs and their hash are frozen before oracle or visual execution. Cohort size is
the first sufficient prefix among 96,128,160, using **only** oracle label counts:
MAJOR >=12 and NO_MAJOR >=40. No availability/score/geometry/GT filtering is used.

Before new oracle calls, the original complete-link clusterer and P9-R1 major
definition must reproduce all 24 historical BASE263 frame labels (9/15) and all
22 major IDs. Historical seed functions, schedule, right/body SE(3) perturbation
and normalized numerical pose carriers are reused, not approximated. The oracle
has 263 calls/frame, .8 m resolution, .08 step, 1e-5 epsilon, 80 iterations, and
the original source/target preprocessing. Iteration-limit converged terminals
remain eligible; no early stop, seed filtering or new oracle clustering rule.

Oracle outputs are isolated from the candidate/visual evidence process. The
final evidence manifest carries IDs only, not labels or canonical poses.
Candidate generation is frozen R2A predictor-conditioned WEAK2, B12, first
seed122, deterministic farthest-point order (including the historical numerical
tie rule), fixed T0 U_obs weakest two eigenvectors. The complement remains that
of the real predictor. Neither oracle outcomes nor GT enter proposals.

## Frozen non-oracle evidence

Reuse R2B complete-link .2 m AND 2 deg clustering with nominal T0, converged
terminals only, maximum-score representatives, EPS_SCORE=2.747604276e-4, and
`S_cluster >= S0 + EPS_SCORE`. Visual alternatives additionally require true
center separation >.2 m OR >2 deg. Inside-center competing representatives are
archived but excluded from visual arbitration; R2B U_comp is retained as a
secondary auxiliary, with its original cluster contract unchanged.

Every visual frontend source and dependency carrier is pinned to
3a98a3cd1d64d7aba6888d3136295a0b41c38fa6. No PnP/depth/KLT/FB/MEI/sync changes,
no fallback and no new threshold. Run all lags 1/2/4/8, choose smallest VALID lag.
Use frozen nominal reference map_T_imu, never GT. PnP T_Ccur_Cref is converted by
`D_vis = T_IC inverse(T_Ccur_Cref) inverse(T_IC)`, representing
`inverse(map_T_imu_ref) map_T_imu_cur`. Convert NDT map_T_lidar to map_T_imu by
right multiplication with inverse(T_IL).

`E_k = inverse(D_vis) inverse(T_ref) T_k`;
primary residual is its translation norm in meters. The signed margin is
`G_visual = r_nom - min_alt r_alt`; `U_visual=max(0,G_visual)`. Available with no
strict alternative gives zero. Unavailable is missing, **never zero**. Rotation
and quality are descriptive only. No weights, learning, posterior or probability.

Freeze candidate runs, terminal clusters, all visual measurements, residuals and
non-oracle evidence with SHA256 before evaluation loads oracle labels. GT is not
loaded until the oracle/evidence evaluation receipt is frozen.

## Frame-level inference and immutable decision policy

Visual availability must reach >=60% in each label group and at least 8 available
MAJOR /24 available NO_MAJOR. Statistics use available frames only, frame as the
unit: ROC AUC, one-sided permutation test, LOFO balanced accuracy. Permutations
use PCG64 seed 20261008, 10,000 replicates, fixed label counts and the plus-one
p-value. AUC is the prespecified permutation statistic. In each LOFO training
fold choose maximum balanced-accuracy threshold, breaking ties by the numerical
smallest threshold; apply it only to the held-out frame. Ties in scores receive
half credit for AUC. Threshold candidate enumeration includes constant-label
solutions and all attainable score splits; the strict `score > threshold`
classification convention is fixed before running.

Success requires sufficient oracle cohort, coverage PASS, AUC >=.80, p<.05,
LOFO BA >=.75, and delete-one-available-MAJOR minimum AUC >=.70. Before attributing
a failed evidence gate to visual arbitration, report the frozen B12 generator's
strict competitive candidate coverage across all MAJOR frames; below60% is an
upstream candidate-generator failure, with priority over a discrimination failure.
Coverage failure is reported separately and never used to force a main claim.
Insufficient labels at160 stops all candidate/visual execution. No gate is lowered.

Post-freeze oracle recall is diagnostic only (macro frame and micro archive-ID
recall; overlapping admission balls are not strict local-minimum certificates).
Posthoc GT compares visual-best in the nominal-plus-strict-alternative candidate
set against nominal; no frame is removed and evidence is not recomputed. Secondary
U_comp discrimination uses all final frames, while Spearman uses the visual-
available intersection. Directionality is analyzed only after primary PASS.

## Delivery and scope

Original paper and stable workspaces remain unmodified. Work occurs in an
independent no-hardlinks local clone; all input data remain at their true absolute
paths. Release/P9 tests, selection, label parity, generator parity, frontend hash
guards, label/GT isolation and CSV/JSON/hash audit are mandatory. Deliver local
commit and a verified portable Git bundle; do not push.
