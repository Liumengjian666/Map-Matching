# PAPER-P6-ALG-INTEGRATION-A3E-R2

Result: **square-root numerical-architecture repair and the single P3-200
engineering gate PASS**. This is not localization-accuracy or formal-experiment
acceptance. `READY_FOR_FORMAL_EXPERIMENT=NO`.

START_SHA: `e74735956ea82d6fef9c9eb9616c33cbf4443dc1`

CODE_SHA: `0ec016677bec6bf4301447ebcd01da353e74ad21`

Branch: `research/p6-i6d-full-algorithm`; existing workspace reused, no new worktree.
The final analysis commit containing this report is END_SHA; the delivery message
records its full SHA and observed remote status after the one normal push.

## Production repair

V3 explicitly selects `SQUARE_ROOT_QR`. Prior A/b and reference chart are
authoritative. Initial information uses unregularized LLT; subsequent prior
updates persist QR-retained/compressed rows. The marginalization input is the
prior plus incident raw whitened factor rows, never an aggregated normal Hc.
Derived H/g remain optimizer/covariance compatibility caches only. Default
legacy windows/V2 retain the old backend.

Oldest-column elimination and retained-row compression both use column-pivoted
Householder QR with standard eps/dimension/maximum-pivot rank rules. Oldest rank
must be 15; deficiency fails closed. No old-Schur fallback, pseudoinverse,
epsilon-I, eigen clamp, noise/weight/rank changes or threshold changes.
See [SQUARE_ROOT_PRIOR_CONTRACT.md](SQUARE_ROOT_PRIOR_CONTRACT.md).

The legacy Schur shadow runs only under diagnostics, on a copied window using
the same already-frozen incident LiDAR projections. Shared callbacks are not
reinvoked. Shadow success/failure does not affect production prior or lifecycle.
The reviewed callback-isolation issue was repaired before the real run.

## Tests

Release build PASS; full Release CTest 35/35 PASS; Debug targeted 13/13 PASS;
diff-check PASS. Old tests/tolerances unchanged. A1-R1, A3B-R1/R2, A3C-R1/R2,
A3D-R1 and A3E-R1 contracts remain PASS.

Joint IMU + rank-5 LiDAR + cross-state visual graph conservation:
relative H error `3.24986e-13`; relative g error `4.82723e-14`.
Nonzero prior chart/gradient, whitening orientation, additive constant,
minimized row objective, rank failures and diagnostic parity PASS.
1005 eliminations: max prior rows 15, active IDs 3; historical IDs cannot reenter.
Known extreme-scale fixture retains H=1 through QR while double normal Schur
produces H=0. Frozen old TX155 capsule still gives legacy minimum
`-97.63543657869423`; its SHA remains unchanged. No raw factor graph is fabricated
from that capsule.

## Single real replay

Exactly one Corridor01 V3 P3-200, visual NONE, official calibration, unchanged
input/map/IMU/params/handoff, no GT. Seven input hashes match the previous run;
production manifest gate remains enabled and passes. Exit code 0.

First QR marginalization: tx71, stamp `1517157226248733355`, enforcement 40.
The 39 completed events before it pass the frozen pose/cost/identity comparison.
First observed divergence is tx72 terminal `1517157226349593474`, after the new
backend's prior starts affecting optimization. No unexplained pre-backend drift.

200 raw scans: 51 pre-handoff skips, 149 Window-owned deskews/terminals.
298 completed events: 267 ACCEPTED_UPDATE, 31 CONVERGED_WITHOUT_STEP.
260 QR oldest removals all SUCCESS, all marginalized ranks 15.
91 LiDAR factors committed, 58 not committed; these are health statistics, not
accuracy. NDT/U_obs valid on 149/149; 104 probes; NDT calls 357.

All completed states finite and valid SO3. Candidate basis callbacks 0;
post-handoff IKFoM calls 0; visual events/factors 0; dense sparse-solver fallback
0; non-LiDAR covariance time 0. Completed max window nodes 40, max span
`1.9589149060000002 s`; pre-enforcement nodes can be 41 and are duly reduced.

| Event | Optimizer | Removal attempts | Final nodes | Final span s |
|---|---|---:|---:|---:|
| tx90 | ACCEPTED_UPDATE | 2 SUCCESS | 39 | 1.916218042 |
| tx115 | ACCEPTED_UPDATE | 2 SUCCESS | 39 | 1.916217089 |
| tx155 | CONVERGED_WITHOUT_STEP | 2 SUCCESS | 39 | 1.916217804 |

Tx115 follows a different valid optimization path after prior changes; no claim
that its old small-step numbers should be identical. At tx155 the preserved
natural-small-step branch is exercised: raw norm `1.8246071048892027e-11`, no clip,
candidate rejected and rollback difference 0, convergence without state commit.

## TX155 direct numerical evidence

Stamp remains `1517157234720485283`, enforcement 208. Before removal:
41 nodes / 2.017077923 s. Attempt 1: stack 30×615, Am rank 15,
rank threshold `1.023059255910697e-5`, R diagonal range
`2749.4364922149352 .. 1535816427.8991172`; retained rows 15→15.

The same event's legacy shadow fails the unchanged validator with
`lambda_min=-24.231923242581125`. QR production succeeds, stored H symmetry
defect 0, minimum eigenvalue 0 (zero blocks for other retained nodes),
maximum eigenvalue `348827.40920818585`. It then completes attempt 2:
30×600, rank15, rows15→15, lambda_max `354906.30117067503`;
final window39 / 1.916217804 s. The shadow failure does not stop production.

This is a same-current-event causal contrast, not a reproduction of exactly the
old -97.6 matrix: the prior/trajectory have changed. Current tx155 has no
incident LiDAR row in its stack; the prior+IMU chain still exhibits the legacy
normal-information cancellation failure. It is not attributed to IMU theory.

## Important residual limitation

Pre-measurement sparse covariance is available on 129/149 terminals and
UNAVAILABLE on tx136–155 (20 terminals). The unchanged fail-closed policy
executes no nonlocal probe and admits no selected-NIS LiDAR factor when P is
unavailable. No IKFoM covariance fallback or artificial information was used.
The QR repair therefore does not claim to fix the optimizer/covariance
normal-information conditioning path. This is retained evidence for the
decision AI, not silently repaired in this stage.

## Resources and retained artifacts

Authoritative prior max15 rows /615 extended columns /73920 bytes including b.
H/g compatibility cache and temporary stacks are additional memory, not included
in that prior-row byte count. QR-path assembly/whitening/elimination/compression/
cache time mean4.904978 ms, max6.049293 ms, measured before shadow/spectral work.
Wall168.08 s; user155.78 s; system11.74 s; maxRSS112772 KiB. Diagnostics ON;
these are engineering observations, not a CPU benchmark or paper conclusion.

Small reproducible artifacts are committed in `RUN_P3_200/`; full logs remain at
`/tmp/p6_a3e_r2_corridor_p3_200` and their SHA/size inventory is committed.
No bag, point cloud or large raw input is committed. No complete dataset, GT,
ATE/RPE, parameter search, visual replay or extra P3 replay was run.

`A3E_R2_SQUARE_ROOT_MARGINALIZATION_CODE_CONTRACT=PASS`

`A3E_R2_TX155_NUMERICAL_REPAIR_GATE=PASS`

`A3E_R2_P3_200_REAL_LINK_GATE=PASS`

`REAL_REPLAY_COUNT=1`

`READY_FOR_FORMAL_EXPERIMENT=NO`

STOP. No next-stage implementation or experiment is authorized by this report.
