# P13 — mature-algorithm failure selection, not algorithm development

TASK: `P13-MATURE-ALGORITHM-FAILURE-SELECTION-R1`

FINAL_RESULT: `FAILURE_MODES_IDENTIFIED_NOVELTY_NOT_ESTABLISHED`

NEXT (one only): `REUSE_INLIER_AS_LABELLED_VERIFICATION_BASELINE`

## Scope and provenance

Repository `Liumengjian666/Map-Matching`, branch
`research/p9-r4-heldout-visual-evidence`; START_SHA and independently checked
remote HEAD `b71b9644a735c8299a31bba044a51a403d07daf1`.
Safe workspace `/home/jian/livox_ws/p10_corridor01_oosm_r1_ws`.
Dirty main paper workspace and stable robot workspace remain untouched.
Local commit SHA is supplied at handoff; this archive cannot contain its own
commit SHA. CODE_SHA means the commit of the descriptive analysis utility,
not a new localization algorithm. PUSH_EXECUTED=NO.

P11/P12/P12B and P10 V2 outputs are reused unchanged. The new script only
reads committed CSV/JSON/descriptor records and writes this new archive.
New registration calls=0; ROS replays=0; raw extractions=0; parameter changes=0.
No raw GT is loaded and no evaluation transform is refitted. Prior frozen
post-hoc metrics are explicitly reused. No new precision/recall experiment.

## What the mature implementations already provide

SOLiD creates usable compact descriptors and retrieves 389 independent Loop2
mapping references for 5816 valid Loop3 queries. Reference and query sessions
are distinct. This establishes functionality, not independent place accuracy.
Its author code also exposes `pose_estimation()` using the azimuth block;
our P11 adapter calls `loop_detection()` and does not call this yaw API.
Missing heading in this adapter is NOT a missing capability of SOLiD itself.

The original DCReg `RunRegistration()` executes on actual 1400-point sources
and the 459154-point prepared prior map. P12 produced 96 finite rigid poses
(24 local calls plus 72 candidate calls). P12B produced 18 finite rigid poses.
The solver's degeneracy handling is reused rather than reinvented. No output
is an independently verified place solely because it is finite or converged.

P10 V2 establishes real causal IMU/NDT/EKF, fixed-lag OOSM and Control/Shadow
determinism for its 396-scan Corridor01 window. This does not establish a
Corridor01 SOLiD database or DCReg experiment. Neither is fabricated here.
P11 itself did NOT certify full EKF parity: late fixed-lag events differed;
its NDT diagnostics were equal. Do not merge these distinct parity findings.

## A. Actual cost and what can be attributed

All times below are directly recorded offline-probe wall timers, not ROS
end-to-end latencies, and not sums of timings from separate experiments.

|Stage|P12 mean / P95 / max, ms|P12B mean / P95 / max, ms|
|---|---:|---:|
|SOLiD descriptor|0.148 / 0.227 / 0.232|1.809 / 7.701 / 10.202|
|SOLiD retrieval|0.088 / 0.103 / 0.132|0.111 / 0.200 / 0.238|
|One complete candidate DCReg API|143.203 / 182.767 / 188.001|151.111 / 194.916 / 196.771|
|Author iteration timer|85.327 / 124.053 / 131.356|91.386 / 134.716 / 138.599|
|API-minus-iteration setup proxy|57.876 / 61.888 / 67.723|59.725 / 67.688 / 73.062|
|Actual query → three candidate calls + logging|430.015 / 518.743 / 548.515|455.466 / 545.603 / 572.368|

P12B's first cold descriptor sample of 10.202ms is retained; no timing outlier
is removed. P11 full-query descriptor/search means are 0.372/0.195ms.
Cross-run timing differences do not prove an A/B speed improvement.

Source inspection: `RunRegistration()` constructs a new target cloud copy
and KD-tree before its author `total_timer` starts, on every call. The setup
proxy is 40.416% of summed P12 candidate API time and 39.524% in P12B. It
includes cloud copy/tree/wrapper work; it is NOT isolated KD-tree timing.
The remaining iteration timer contains nearest-neighbor queries, local plane
fits, robust correspondences, normal-equation assembly, degeneracy handling
and the small six-dimensional solve. Existing logs cannot separate those
components. Do not claim the PCG solver or correspondences alone dominate.

P12: 60/72 candidate calls hit the 30-iteration limit and consume
9124.950ms of 10310.648ms candidate API time (88.500%). P12B: 14/18 hit the
limit, consuming 2207.268/2719.989ms (81.150%). These are **nonconvergence
expenditures**, not proved avoidable waste or false-place expenditures.
P12 peak RSS=64664KiB; P12B=64540KiB. Both are offline probes, not whole ROS.

Engineering opportunity: share prepared target/tree if an author-supported
interface permits it. This removes repeated setup, not a paper innovation;
no speedup or author solver change is executed in P13.

## B. Location evidence versus suspected aliasing

### Loop2/Loop3 — no independent online place labels

5605/5816 valid queries have Top5 reference-state spread >5m. Top5 spread
alone is expected in retrieval lists and is NOT an ambiguity rate. There are
two distinct-reference pairs with range-block cosine >0.99 and separation >5m:

|Reference IDs|SOLiD range cosine|Mapping-state separation|
|---|---:|---:|
|37 / 381|0.994788|6.256m|
|41 / 191|0.990944|104.899m|

This is a descriptive collision warning; it proves neither online wrong
retrieval nor two independently optimized NDT/DCReg matching basins.
The full reference-origin/time conversion was not closed in P12B. No new
extrinsic investigation is performed. Approximate references remain marked.

Representative prior cases (all already frozen, no new registrations):

- TX16 IDs0/1 both author-converged, but terminals differ by about 1.33m;
  pre-update plane RMSEs 0.111229/0.306089m. Without an independent label,
  neither can be declared correct or a genuine nonlocal competing optimum.
- TX4105 ID265: correspondence fraction 0.997967, pre-update residual
  0.063872m, still reaches 30 iterations without convergence. A small native
  residual is not an acceptance certificate. That residual is not freshly
  evaluated at the terminal pose and is not comparable to native NDT fitness.
- P12B orientation retry changes stopping status 3/18 → 4/18, with far
  hypotheses 0/8 → 0/8. The only newly converged TX124/ID0 terminal changes
  only 0.000306738m / 0.002783627deg. It is not demonstrated place recovery.

### Corridor01 — independent GT-based relative drift, not place labels

Use only frozen V2: one full-pose SE(3) fit on Control first 0–3s, identical
transform for all methods, primary 3–35s with 6396 high-rate samples.
This is `PREFIX-ALIGNED RELATIVE DRIFT`, not certified absolute map-frame ATE.
Each method has 396 optimizer-converged NDT calls and 396 OOSM applications.

|Executed corrected-IMU trajectory|Translation RMSE / P95 / max, m|Rotation RMSE / P95 / max, deg|
|---|---:|---:|
|Control V2|1.887877 / 5.011039 / 5.667157|3.026246 / 3.705965 / 16.393782|
|Weak-only V2|1.064474 / 2.212340 / 4.529729|2.679232 / 3.646474 / 13.186984|
|Coupled V2|6.398821 / 15.039884 / 16.119987|3.315928 / 3.589677 / 17.536673|

Control first-10s translation RMSE=0.468620m. Persistent crossings (five-second
duration, historical evaluator gap rule unchanged): 0.5m at +6.449565s,
1m at +26.783325s, 2m at +29.823069s. Coupled reaches persistent 1/2m earlier,
at +22.283565/+24.298445s. This is a real negative trajectory result, not a
score-based interpretation. Histories differ, so it does not isolate one
particular candidate or prove local geometry ambiguity caused the drift.
Coupled rosbag was incomplete; complete synchronized publication ledgers,
not the truncated bag, underlie the frozen metrics. Peak ROS RSS unavailable.

## Two bounded problems, no established innovation claim

### Problem 1 (priority): location correctness cannot be inferred from convergence

1. Actual defect: successful local optimization can coexist with sustained
   executed-trajectory error; high descriptor similarity can span remote
   reference places. These are separate supported facts, not yet a joint
   labelled nonlocal false-match dataset.
2. Numbers: Corridor Control 396/396 converged but primary RMSE1.888m and
   persistent >2m drift; Loop2 reference pair 41/191 cosine0.990944 at104.90m.
3. Existing methods: DCReg improves local numerical conditioning; SOLiD
   retrieves places; RTR-Net already adds selective local/global reranking;
   InLiER already supplies yaw and token-guided 6DoF verification. Whether
   they leave this problem unsolved on our data is **NOT TESTED**.
4. Potential research mechanism: only if those mature verifiers still falsely
   accept independently labelled repeated-place alternatives, study a
   low-budget criterion for insufficient location-discriminating evidence
   and a defensible reject/abstain decision. No new formula is claimed here.
5. Low-compute potential: avoid expensive wrong/indistinguishable feedback
   rather than increase candidates. Actual benefit is NOT ESTABLISHED.
6. Fair ablation: identical candidate pool, coordinates, initialization and
   compute limits; author InLiER verification and existing local-score gates;
   measure independently labelled false accepts, retained correct candidates,
   abstention/coverage and measured CPU. Freeze rules without evaluation labels.
   Do not equate sparse retrieval lists or small Hessian eigenvalues with a
   proved nonlocal competitor. True query-place labels are required first.

### Problem 2 (secondary): candidate verification spends budget without certifying a place

1. Actual defect: three local geometric verifications cost430–455ms; repeated
   setup contributes about40%; most measured candidate-call time ends at
   max iterations, not a solver-converged result.
2. Numbers: P12 12/72 converged; 30 far coarse hypotheses 0/30; 88.50% call
   time nonconverged. P12B 3/18 →4/18 does not resolve the bottleneck.
3. Existing work already covers much of this: RTR-Net trigger-reranking,
   InLiER compact correspondence-guided verification, Autoware MULTI_NDT_SCORE
   instead of full convergence at every hypothesis. Generic selective
   validation, early stopping or KD-tree caching are not new algorithms.
4. Conditional research mechanism: risk-controlled verification allocation
   only if mature baselines cannot preserve correct-candidate recall at an
   equal budget. No untested scheduler or recall guarantee is proposed.
5. First remedy is mature reuse/ordinary caching, not original math. Most
   potential savings here are engineering until proved otherwise.
6. Fair ablation must give every arm the same caching, target, candidate IDs
   and input, keep a fixed whole-call budget, and measure recall loss as well
   as wall-time saved. Convergence-only scoring is inadequate.

## Decision and limits

The most valuable **question** is Problem1, but a reliable *new-method gap*
has not been demonstrated. InLiER is a directly relevant public MIT-licensed
implementation to reuse before inventing a verifier. Its benchmark claims
must not be transferred to our hardware or data. RTR-Net official code was
not verified in this bounded search; absence of a found repo is not proof
that no code exists. See LITERATURE.md for primary sources checked2026-10-11.

One next recommendation, requiring decision-AI authorization for a future
task: `REUSE_INLIER_AS_LABELLED_VERIFICATION_BASELINE`. A labelled test must
either use an existing independently referenced public protocol or actually
establish independent query-place labels; Loop3 NDT poses are not GT.
If the mature verifier resolves the failure within budget, reuse it and do
not manufacture a Dual-U innovation requirement. P13 ends with this report.
