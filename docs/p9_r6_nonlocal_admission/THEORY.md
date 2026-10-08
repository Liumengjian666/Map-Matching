# R6: nonlocal admission formulation development

Status: POSTHOC FORMULATION DEVELOPMENT, EXPLORATORY ONLY, NOT CONFIRMATORY.
Start: 5a95362861464d9b8756e444ea3dcf999e032d6a. The R4 FAIL and R5 MIXED
conclusions remain unchanged. The 96 frames are now development data.

## Frozen construction

Reuse the already frozen non-oracle complete-link clusters, without reclustering
or changing their maximum-score, lowest-rank-tie representatives. B12 and FULL263
are processed separately. FULL263 is an offline reference, not extra information
available to B12. Only converged terminals participate; converged iteration-limit
terminals remain. T0 is retained as a rank-zero member, even when a higher-scoring
member represents its nominal cluster.

S_star = max(S0, all observed cluster representative scores).
N is the unchanged prepared source point count. gamma is fixed at 0.05, inherited
from the historical oracle tolerance, not tuned here. Delta=(S_star-S_k)/N and
tau=0.05*max(1,abs(S_star/N)). Near-optimal admission is inclusive: Delta<=tau.
There is no calibrated likelihood or probability interpretation.

A separated alternative must be a non-nominal cluster whose representative has
translation>0.2m OR rotation>2deg from T0. Preserve the original pose/quaternion
carrier when checking separation. A cluster with both inside and outside members
is BOUNDARY_STRADDLING; flag it, do not replace its representative. Distinct
complete-link clusters are descriptive terminal groups, not certified attractors.

TYPE A: T0 itself is near-optimal and at least one separated alternative is
near-optimal. TYPE B: T0 itself is outside the band and at least one separated
alternative is near-optimal. Otherwise NONE. A and B are mutually exclusive.
T0's own S0, not the nominal cluster representative score, determines its band.

Use frozen P9 map-product displacement: [(t_k-t0)/0.8; Log_spatial(R_k R0^T)].
For A alternatives sum xi xi^T to A_near; for B alternatives sum to A_disfavored.
U is sqrt(lambda_max) of its own tensor. The inactive channel is exactly zero.
Do not add the channels or normalize by count. Neither tensor is a covariance.
The generator's archived U_obs/W2 is not recomputed or used to tune admission.

## Information and parity boundary

The builder's explicit data allowlist contains raw terminal pools, nominal data,
unlabeled non-oracle cluster tables, the frame-ID-only cohort, and hash receipts.
It must not open labels, canonical cluster/ID files, R5 labeled summaries or GT.
A runtime read guard enforces the allowlist for repository docs and external data.
The full terminal assignment, maximum-score representative and stored geometry
are verified against the raw pools. Reusing byte-pinned clusters retains the
historical deterministic complete-link merge order exactly.

Freeze all candidate, boundary, frame tensor and runtime artifacts with SHA256
before a separate evaluation process loads labels. That process first requires
original STRICT parity: B12 17/40 and 3/56; FULL263 30/40 and 5/56. Failure stops
interpretation. Original STRICT still uses S>=S0+2.747604276e-4 and original
representative geometry. No threshold is changed.

## Development evaluation

Report STRICT, A, B, and A-or-B event rates separately; sensitivity and specificity
are against the frozen oracle proxy, not absolute truth. Report U_near and
U_disfavored distributions and tie-aware ROC-AUC separately, without fitting a
threshold or fusing channels. No permutation significance claim or held-out PASS
is made. Report zero/one/multiple separated clusters and member support counts.

R6 specifies no numerical gate for PROMISING/NONDISCRIMINATIVE/DOMINANT. Final
classification will therefore be a transparent descriptive assessment of the
fixed B12 event rates and scalar distributions, not an invented confirmatory
acceptance test. A coverage increase alone is not success; widespread NO_MAJOR
positives must be treated as a false-alert tendency under the proxy. No score,
gamma, clustering, direction or budget may be adjusted after observing results.
Any promising formulation needs a new dataset. A nondiscriminative formulation
must not be rescued by further score-threshold tuning on these Floor01 frames.

## Cost and scope

NEW_NDT_CALLS=0; BASELINE_REPLAY_CALLS=0; VISUAL_EXTRACTION=0; GT_LOADED=NO.
Record hash/input, cached cluster verification, and new evidence calculation
times separately. Cached-cluster processing is not a fresh online clustering
benchmark. Historical B12 alignments cost about827ms/frame and are not accelerated
by this admission analysis. No pose switching, EKF or fusion is authorized.
