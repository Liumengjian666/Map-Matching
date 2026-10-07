# P9-R1C3B discrete support-transition evidence

## Scope and scientific boundary

This is an offline diagnostic for whether local PCL NDT support transitions
produce repeatable, numerically material fixed-support responses, and whether
those responses distinguish the frozen major-competitor cohort from the
pre-existing no-major-basin cohort. A material fixed-support response is not
evidence that the dynamic NDT objective has multiple self-consistent local
minima. Support transition is a candidate discrete evidence source for
`U_nonlocal`; it does not replace the DUAL-U contribution
`U_obs + U_nonlocal`.

No full NDT alignment, GT, trajectory error, posterior, Bayesian/Laplace
weighting, EKF, smooth support-crossing continuation, basin search, or
oracle-conditioned probe is used.

## Frozen inputs and cohorts

The event engine receives only the frozen P5 target map, the 32-row frozen
Floor01 cohort, the archived `dual_u.csv` U_obs records, and (in a separate
historical-only pass) the already-recorded R1C3A support-pair table. It does
not receive oracle basin labels, canonical basin poses, `u_b`, `v_b`, or GT.
Every event is constructed and evaluated before the analysis step joins
frame-level labels.

The deterministic-probe cohort is all 32 rows of `cohort_frozen.csv`.
Major-competitor labels are joined only after event construction from the
frozen H1 9-frame/22-basin table. All other 23 cohort frames form the
predeclared no-major-basin comparison group; this label means no frozen major
basin was recorded, not that the frame has no minor alternatives.

The historical R1C3A transitions (3341 shared ROOT rounds and TX616
alpha=0.68625 OLD/NEW support rounds) are re-evaluated as supplemental
mechanism examples. They are excluded from the cohort classifier because
those transitions were targeted in earlier work.

## Deterministic local support stencil

Use the frozen P9 map-product chart at each archived nominal terminal `T0`:

`eta = [delta_t_map / 0.8 m, delta_theta_map] = W2 u + S4 v`,

where `W2=[q1,q2]` and `S4=[q3,...,q6]` are the archived ascending-curvature
eigenvectors. The event probes themselves are the same full-chart coordinate
stencil in every frame, not oracle weak coordinates:

`eta_i(a) = a e_i`, `i=0..5`, `a in {-0.02,-0.01,0,+0.01,+0.02}`.

This gives six ordered 1-D stencils and exactly 24 adjacent edges per frame.
All 768 potential edges (32 x 24) are evaluated; an edge is a support event
only when its endpoint support signatures differ. Offsets, order, chart,
support test, and solver settings are fixed globally before cohort execution.

For an event edge with ordered endpoints `eta_A, eta_B`, use their midpoint
`eta_m=(eta_A+eta_B)/2`; set `u_m=W2^T eta_m`, `v_m=S4^T eta_m`. Both frozen
support hypotheses are minimized at this identical `u_m`, initialized from
the identical `v_m`. Thus the only changed objective is the frozen target
support. The local corrector is strong-coordinate only; it does not update
support and is not `ndt.align()`.

## Event quantities and numerical normalization

For support `s`, define the exact archived frozen PCL score in the dimensionless
chart, normalized by the prepared source point count:

`E_s(u,v) = -frozenScoreDouble(source, poseAtEta(T0, W2 u + S4 v), s) / N`.

The same PCL 1.10 grid, leaves, mean, inverse covariance, score constants and
derivative guard are used. Only the existing R1C2 continuous DOUBLE frozen
transform is used for stationary diagnostics; the dynamic support signature
continues to come from the production-compatible float PCL transform and
radius search. DOUBLE is not proposed as a production objective.

For each side, use the archived R1C2 `referenceMinimum` numerical certificate.
Let `rho_E` be its double energy resolution and `d_resolution` its implied
Newton-coordinate resolution. Freeze:

`EPS_DV_side = max(d_resolution, fine/coarse Newton-displacement difference)`,

`EPS_E_side = 64 eps_double max(1, |E_side|) (leaf_terms + 32)`,

`EPS_DV_PAIR = EPS_DV_A + EPS_DV_B`,
`EPS_E_PAIR = EPS_E_A + EPS_E_B`.

Let `v_A*`, `v_B*` be the fixed-support endpoints. Preserve components:

`I_v = ||v_A*-v_B*|| / EPS_DV_PAIR`,
`I_E = |E_A(v_A*)-E_B(v_B*)| / EPS_E_PAIR`,
`I_pose_t = translation_separation / (0.8 EPS_DV_PAIR)`,
`I_pose_r = rotation_separation_rad / EPS_DV_PAIR`.

Do not collapse these diagnostics into a `U_nonlocal` scalar. Also record both
cross energies `E_A(v_B*)`, `E_B(v_A*)`, each `H_vv` spectrum/condition number,
the signed energy difference, changed-point and membership fractions, and
whether each fixed-support endpoint's dynamic support equals its hypothesis.

An event is `UNRESOLVED` unless both reference minima report
`DOUBLE_RESOLUTION_STATIONARY` and both `H_vv` are SPD. Resolved class labels
are mutually exclusive, with the strong threshold checked first:

- `EVENT_STRONGLY_MATERIAL` when `I_v>3 OR I_E>3`;
- `EVENT_MATERIAL` (the moderate-only band) when at least one impact is `>1`
  and neither impact is `>3`;
- `EVENT_NUMERICALLY_NEGLIGIBLE` when `I_v<=1 AND I_E<=1`.

These boundaries are multiples of measured numerical envelopes, not thresholds
fit against frame labels.

## Repeatability rule

Repeatability is assessed only on the six predeclared four-edge stencils per
frame. It does not require exact support hashes to recur. An axis stencil is a
repeatable material event only if at least 3 of its 4 neighboring edges are
resolved and have the same material class (`MATERIAL` or `STRONGLY_MATERIAL`),
and one of these two frozen consistency checks holds:

1. for `I_v`, all pairwise cosines of the ordered strong-coordinate response
   vectors `(v_B*-v_A*)` are nonnegative and `max(I_v)/min(I_v)<=10`; or
2. for `I_E`, the signed differences `E_A(v_A*)-E_B(v_B*)` have the same sign
   and `max(I_E)/min(I_E)<=10`.

The factor-of-ten rule is the predeclared same-order-of-magnitude criterion;
the nonnegative ratios themselves have no sign. Record exact pair recurrence
separately. A frame's repeatable-event count is the number of qualifying axis
stencils (0..6), never the number of individual edges.

## Frame-level comparison and statistics

Only after all event features are written, attach the frozen major/no-major
labels and H1 `rho_W2`. Report per-frame event, material, strong-material and
repeatable-stencil counts; componentwise maxima and medians of `I_v` and `I_E`;
and `rho_W2` (N/A for no-major frames because no major displacement statistic
exists there).

The predeclared primary discriminator is `FRAME_REPEATABLE_EVENT_COUNT`.
`FRAME_MAX_Iv` and `FRAME_MAX_IE` are separate secondary discriminators; no
combined impact score is formed. For each feature report major-vs-no-major
ROC-AUC and a one-sided frame-label permutation test (100,000 deterministic
Monte Carlo permutations, PCG64 seed 20261016, with the plus-one correction).
Report leave-one-frame-out threshold accuracy: for each held-out frame, choose
the threshold maximizing balanced accuracy on the other 31 frames, with
deterministic lowest-threshold tie-breaking, then classify the held-out frame.
Also report the 32 leave-one-out AUC values/range and leave-one-out mean-gap
signs. This checks whether one frame drives the effect.

For the nine major frames only, report secondary Spearman association between
H1 frame `rho_W2` and each support-evidence feature, with a deterministic
frame-level permutation p-value. This is exploratory and not a basin-level
test.

`DISCRETE_SUPPORT_EVIDENCE_SUPPORTED` requires the primary repeatable-event
count to have one-sided permutation `p<0.05`, ROC-AUC `>=0.80`, leave-one-frame
out accuracy `>=0.80`, leave-one-frame out minimum AUC `>=0.70`, and positive
major-minus-no-major mean difference after every single-frame omission.
For deterministic result labeling when that gate fails: call events
`SUPPORT_EVENTS_NUMERICALLY_UNSTABLE` only when at least five major-frame
axis-stencils contain a material edge and fewer than half of those material
stencils are repeatable. If the aggregate primary feature has either AUC
`>=0.80` or one-sided permutation `p<0.05`, but any LOFO stability condition
fails, call it `SUPPORT_EVIDENCE_COHORT_DEPENDENT`. Otherwise call it
`DISCRETE_SUPPORT_EVIDENCE_NOT_DISCRIMINATIVE`. These fallback labels are
predeclared diagnostics, not alternative success gates; do not rescue a failed
primary result by post-hoc metric or threshold selection.
