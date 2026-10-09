# P10-R1: coupled subspace NDT prototype

FINAL_RESULT = COUPLED_SUBSPACE_NDT_PROTOTYPE_SUPPORTED

Development target met on the first frozen implementation: D recovered 7/7
GROUP A canonical IDs after complete NDT refinement. This means that the
prototype runs and reaches the prescribed development target. It does **not**
establish superior recovery over warm-start, online localization success,
independent generalization, or real-time feasibility. No targeted revision was
run; no additional experiment was launched after the target was reached.

## Git and scope

- Branch: research/p9-r4-heldout-visual-evidence.
- Task start: 1327c767f92ef8a85eb05a3ef049ff396657cd54.
- Tested implementation commit: e4cfa6e4fa1eca312e7158d588751c3da79fcf3f.
- Writable worktree: /tmp/dog_loc_paper_r4_ws.Fq21k2.
- Original paper workspace Git remains read-only in this environment.
- No push executed or new remote delivery asserted. The final archive commit
  is reported in the chat and delivery receipt rather than embedded in itself.
- New files only under scripts/p10 and docs/p10_r1_coupled_subspace_ndt; old
  P9 archives, production EKF/IKFoM, stable dog workspace remain unchanged.
- GT loaded: NO. Oracle initialization: NO. Visual: NO. Corridor bootstrap: NO.

## What was implemented

`scripts/p10/p10_coupled_subspace_ndt.cpp` is an independent executable.
`orderedGrid` generates original bounded weak nodes with deterministic
nearest-neighbor ordering from zero. `jointJet` computes the dynamic PCL NDT
gradient/Hessian in the original **global** map-product chart and joint basis
[W,S], including gradient-times-second-chart-derivative terms. `couplingStep`
solves Hvv*dv=-Hvu*du using LDLT, checks SPD/condition/finite values, caps the
predictor move at 0.10, and provides explicit warm fallback. `experiment`
maintains independent fixed-u histories, calls the unchanged P9 `iterativeNewton`
for B/D, and records one standard complete NDT refinement per proposal.
`evaluateArchive` is a separate post-freeze pose-proximity evaluator.

The actual addition is **Hvu-driven transport of strong coordinates between
weak proposals plus fixed-weak conditional correction**, not a new NDT
objective, chart, EKF update or global optimizer. E=-raw PCL score/N; eta=Wu+Sv;
v_pred=v_previous+dv. Full refined terminals do not feed the path history.

Historical R1A ZERO_V used oracle u_b, and R1B beta used oracle v_b. Those
initializations were not reused. Canonical poses were accepted only by the
separate evaluator after candidate CSVs and their SHA256 were frozen. The
builder receives map/cohort/U_obs, not a canonical target file. The wrapper
hashes the canonical file for immutable evaluation provenance but does not
parse it to build proposals.

## True inputs and common budget

Historical input byte hashes all match. Prepared source admission:
616=358 points / FNV64 16677765666605202002;
2226=554 / 2810802364734767265. Prepared target=549606.
Input file SHA256 and command/binary/source digests are in
attempt_0/execution_freeze.json. Inputs are the archived SAME_OBJECTIVE
scan-end LiDAR sources, not topic-bag clouds or newly reconstructed data.

For both frames k=2. The original 9x9 weak box plus existing combined 2m/15deg
weak-displacement bound retains 55 nodes for 616 and 51 for 2226. Every arm
uses those exact 106 nodes in the same order and makes 106 full NDT calls.
Two additional nominal-T0 refinements preserve the ordinary baseline.
Total full NDT calls=426; new oracle calls=0; baseline replay calls=0.

B and D use identical 20-iteration conditional caps, not 20 evaluator calls.
D spends an additional joint jet per node. Actual explicit evaluator counts:
A=424, B=16437, C=530, D=16646; plus two frame-setup evaluations and nominal
diagnostics included in their baseline rows. NDT's internal evaluator calls
are not included in these explicit counts. Source/target loading/setup time
is outside per-proposal timing and is included in complete process wall time.

## Recovery: seven frozen IDs

Recovery is successful full NDT terminal within 0.2m AND 2deg of the frozen
reference; this is candidate-pool discovery, not an online winner policy.

|Target|A weak-only|B warm|C predictor|D predictor-corrector|
|---|---|---|---|---|
|616/P02|YES|YES|YES|YES|
|616/P03|YES|YES|YES|YES|
|616/P10|YES|YES|YES|YES|
|616/P12|YES|YES|YES|YES|
|616/P13|YES|YES|YES|YES|
|616/P17|YES|YES|YES|YES|
|2226/P09|NO|YES|YES|YES|
|Total|6/7|7/7|7/7|7/7|

Ordinary nominal-T0 NDT reference: 0/7, two calls. Its output remains archived
in baseline_reference.csv and results.json; it is not a substitute historical
baseline trajectory. Before complete refinement, the four pools recover only
0/7, 1/7, 0/7, 1/7. For B/D, the sole pre-refine ID is 2226/P09 at other
nodes than its first post-refine hit. Per-case CSV therefore distinguishes
pool-level pre-refine recall from the pose of the first successful post-hit.

First successful D hit records (zero-based visit order):

|Target|Node|pre dt/dr (m/deg)|refined dt/dr (m/deg)|E pre/refined|conditional/full iterations|
|---|---:|---|---|---|---|
|616/P02|9|0.17579 / 3.54993|0.02291 / 1.55769|-3.02223 / -3.05753|20 / 11|
|616/P03|9|0.20771 / 3.79825|0.07186 / 0.72404|-3.02223 / -3.05753|20 / 11|
|616/P10|3|0.03218 / 2.32418|0.04409 / 1.99877|-3.06215 / -3.05148|20 / 4|
|616/P12|42|0.41925 / 11.60453|0.04319 / 1.83380|-2.96772 / -3.02479|20 / 13|
|616/P13|43|0.31604 / 8.09120|0.02409 / 0.67913|-2.97526 / -3.01953|19 / 15|
|616/P17|43|0.29795 / 7.92155|0.02293 / 0.80444|-2.97526 / -3.01953|19 / 15|
|2226/P09|38|0.37113 / 17.39381|0.00161 / 0.03687|-2.72038 / -2.87758|18 / 15|

The shared node9 and node43 demonstrate overlapping ID neighborhoods, not
seven independent certified attractors. All node u/v before/predicted/corrected,
four pose matrices, raw score and stage times are preserved in probes.csv;
case_results.csv provides first successful hits or explicitly offline closest
miss descriptions. Neither rule performs online pose selection.

Raw-score-best successful terminal selection was frozen before canonical
evaluation as a diagnostic only. Its recall is A=1/7, B=C=D=2/7. The latter
two IDs are P02/P03 reached by the **same** 616 winner; every 2226 winner misses
P09. No pose switching is authorized.

## Cost and numerical behavior

|Arm|mean/P95 per probe ms|mean/P95 per frame ms|full iteration mean/P95|
|---|---|---|---|
|A|28.488 / 76.849|1509.842 / 2107.727|21.264 / 52|
|B|168.586 / 252.434|8935.081 / 10653.428|23.528 / 52.75|
|C|31.444 / 79.851|1666.523 / 2328.888|22.717 / 56.25|
|D|169.216 / 246.731|8968.465 / 10578.901|22.075 / 48.75|

Frame quantiles use only two frames and are descriptive, not latency
generalization. Stage totals are in runtime_breakdown.csv. D: predictor
119.882ms, conditional Newton14662.149ms, diagnostics352.858ms, complete NDT
2794.017ms across106 nodes. Conditional Newton dominates cost. D does not
improve recall over B, nor establish a latency advantage. C's 7/7 with no
conditional iterations is an observed development result, not grounds to
drop correction without a separately declared comparison.

Entire four-arm process wall=42.69451s; peak RSS=106060KiB (103.57MiB).
NOT YET ONLINE-COMPETITIVE: D averages8.968s/frame at this full development
grid. No claim is made that the low-budget deployment problem is solved.

Both coupled methods have zero solve fallback and two capped predictor moves.
Maximum raw relative solve residuals: C2.885e-16, D2.696e-16;
maximum Hvv condition: C30.856, D27.019. No nonfinite endpoint or exception.
Full NDT statuses: A106 SUCCESS; B103 SUCCESS/3 ITERATION_LIMIT;
C105/1; D105/1. Limit endpoints remain archived but do not count as recovery.
D corrector termination: 70 MAX_ITERATIONS,31 TRUST_RADIUS_EXHAUSTED,
4 ENERGY_CHANGE_SMALL,1 STEP_SMALL. It is not generally a converged
conditional branch solver. Previous strong gradient mean C=1.838,D=0.222;
the predictor-only arm cannot be interpreted as stationary-minimizer tracking.

Full refinement leaves the initial 0.2m/2deg neighborhood for A/B/C=101/106
and D=100/106. Adjacent fixed-u endpoints leave this neighborhood103/106
times for D, with changing weak nodes; this is descriptive geometric movement,
not proof of conditional attractor switching. Mean support-membership change
pre-to-refine for D=.77023. The full refine can increase the explicitly
re-evaluated E (D5 nodes, including its P10 hit); conditional correction
energy-increase count is zero. Preserve this numerical behavior rather than
claiming every PCL refinement is monotonic or stays on the initial branch.

## Verification and handoff

Release build PASS; P10 CTest2/2 PASS (three Python test cases plus C++ self
tests); P9 CTest41/41 PASS with system LD_LIBRARY_PATH. The initial inherited
path prevented two P9 binaries from starting due to libusb; no algorithm
or threshold was changed. Independent read-only review found driver revision/
variant confusion and a duplicate/incomplete grid audit gap; both fixed.
The actual frozen candidate pool was already complete/unique before the audit
strengthening. Ordinary baseline aggregation was separately added without
rerunning NDT or changing frozen evidence. CSV/JSON/hash audit PASS and
git diff --check PASS. See audit.json and validation_before_run.json.

NEXT = P10_R2_BUDGETED_SINGLE_FRAME_COUPLED_NDT_INTEGRATION

Recommendation for the next contract, not performed here: add a bounded
single-frame **shadow** adapter around the existing nominal NDT, reuse its
current source/chart/U_obs, expose the predictor and a short conditional
correction budget, and compare with budget-matched warm-start. Preserve the
ordinary nominal output and do not connect candidate selection to EKF/IKFoM
until a separate selection/safety contract is accepted. The priority deficit
is correction cost and dependence on full refinement, not missing core code.
