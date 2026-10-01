# PAPER-P6-ALG-INTEGRATION-A3C-R2

Targeted expression repair is validated. The original tx90 marginalization
failure is removed, including both oldest removals needed to enforce 2 s.
The single P3-100 completed. The conditionally authorized single P3-200 stopped
at **tx115 optimizer failure**, not a Schur symmetry failure. No retry or
parameter change followed.

```
A3C_R2_ALIAS_SAFE_SYMMETRIZATION_CODE_CONTRACT = PASS
A3C_R2_P3_100_REAL_LINK_GATE = PASS
A3C_R2_P3_200_EXTENDED_GATE = FAIL
READY_FOR_FORMAL_EXPERIMENT = NO
```

## Git and changes

START_SHA: `d97c3cd8e2faeb6a047e93a789692b898429fcc9`.
CODE_SHA: `32e7f71a8f810590e1ee1090811b2c383b700305`.
Branch: `research/p6-i6d-full-algorithm`; same worktree
`/home/jian/livox_ws/dog_loc_p6_i6b_ws`. The analysis delivery commit containing
this directory is END_SHA; the final executor response reports its literal SHA
and remote verification. No squash, reset, merge or force push.

28 package self-transpose symmetry assignments were audited: 20 unsafe fixed,
8 existing safe retained. M7 and dense consumed-H assembly now use independent
evaluated temporaries through the same small function. Reliability modules
retain the exact ideal expression; their definitions, directions, normalization,
length scales and thresholds are unchanged. See [inventory](TRANSPOSE_ALIAS_INVENTORY.md).

## Math/build gates

Release build PASS, full CTest **31/31 PASS** (all original 28 retained).
Debug targeted **9/9 PASS**. `git diff --check` PASS.

- A1-R1 clean H/g errors: `1.24607e-10` / `7.85293e-13`.
- A1-R1 noisy H/g errors: `1.24607e-10` / `7.85317e-13`.
- Repeated conservation + 10,000 lifecycle cycles PASS; nonzero rotation prior
  gradient error `0`; existing solve-only jitter regression PASS at `1e-7`,
  with measured H/g indirect deltas `1.71782e-7` / `1.07995e-8`.
- New same-enforcement two-removal H relative error `7.2511587330693086e-17`,
  g error `2.540132453772453e-22`; stored symmetry defect `0` each time.
- A3B-R2 frozen projection and rank-five pose information PASS. No weak-direction
  artificial measurement information was added.
- OFF/ON diagnostic parity PASS: same states, factors, decisions, prior and g.
- Real frozen tx90 capsule: legacy max asymmetry `1.1175870895385742e-8`,
  validator FAIL; actual new production helper max asymmetry `0`, validator
  PASS, independent lambda_min `0`, no clamp/jitter/threshold change.

Schur consumed-subgraph/retained-only contract is unchanged. PSD symmetry
threshold **1e-8** and eigen tolerance **1e-6** unchanged. No new prior regularizer,
eigen clamp, pseudoinverse production solver or damping change. Existing
reliability eigen reconstruction is untouched, not a new fix.

## Real results (engineering only)

| Diagnostic | P3-100 complete | P3-200 stopped |
|---|---:|---:|
| Completed LiDAR terminals | 49 | 63 (through tx114) |
| Window deskews | 49 | 64 (includes failing tx115) |
| Committed LiDAR factors | 46 | 57 (completed events) |
| NIS rejections among completed events | 3 | 6 |
| U_obs valid / sparse P15 available | 49 / 49 | 63 / 63 |
| U_nonlocal probes | 24 | 38 (completed events) |
| Logged NDT calls in completed runtime rows | 97 | 139 |
| Successful oldest removals | 60 | 88 |
| Successful multi-removal enforcement episodes | 30 | 44 |
| Max completed-event nodes / span | 40 / 1.958914906 s | 40 / 1.958914906 s |
| Consumed H / Hmm / stored prior max asymmetry | 0 / 0 / 0 | 0 / 0 / 0 |
| Minimum recorded stored-prior eigenvalue | 0 | 0 |
| Solve-jitter attempts | 0 | 0 |
| Sparse fallback / inner basis callbacks | 0 / 0 | 0 / 0 |
| Visual events/factors / post-handoff IKFoM calls | 0 / 0 / 0 | 0 / 0 / 0 |
| Wall time / max RSS | 39.14 s / 94,308 KiB | 49.21 s / 93,516 KiB |

The frame limit includes 51 pre-handoff raw scans. P3-100 reached tx100 with
all 49 post-handoff LiDAR terminals complete. All processed clouds are
`WINDOW_OWNED_SE3_DESKEW`; counts and real point-time intervals pass. Non-LiDAR
events report `NOT_REQUESTED_NON_LIDAR_EVENT`, without requesting P.
Recorded poses are finite with valid unit quaternions and monotone event times.
Production has no dense marginal reference call (source gate + original tests);
no diagnostic claims runtime instrumentation that was not logged.

Tx90: stamp `1517157228164951397`, optimizer `ACCEPTED_UPDATE`, 8 iterations,
surrogate cost `29.640314594899319 -> 21.667662999656720`. Attempt 1 succeeds
41->40 nodes (still 2.017057491 s); attempt 2 succeeds 40->39, final
**1.916218042 s**. Both stored priors symmetric, finite, PSD, lambda_min=0,
jitter=0. Detailed raw Schur defects and records are in [TX90](TX90_REPAIR_RESULT.md).
Correcting earlier evaluation artifacts changes subsequent states/costs; old
SHA's optimizer/NIS values are not expected to remain identical.

## P3-200 first failure and STOP

Tx115, `1517157230686328484`, `LIDAR_SCAN_END`:
`producer_optimizer:all_optimizer_candidates_rejected`,
`FAILED_ALL_CANDIDATES`, 1 iteration. Initial/final cost
`81.345724589884568`. Failure is **OPTIMIZER_STAGE**, before marginalization.
The frozen acceptance snapshot has zero candidate basis callback calls.

The single logged step was `7.0568167407415232e-9`, predicted reduction
`3.4098706667169588e-14`; candidate cost `81.345724589884583`, actual reduction
`-1.4210854715202004e-14`. These are evidence only, **not** a newly accepted
root-cause attribution or a proposed tolerance/damping repair.

NDT/U_obs/P15/probes at tx115 are recorded. Current selected NIS
`15.237396865140097 > 15.086`, so its LiDAR factor was normally rejected.
Preopt window is 41 nodes / 2.017077208 s; because optimization failed before
duration enforcement, that failure state is not claimed to satisfy the final
2 s bound. Diagnostic transaction state difference is `0` (PASS).

All 88 earlier removals succeeded. No later-attempt marginalization failure
or same-enforcement partial commit was exercised naturally. No failure was
injected to investigate batch rollback. Failure capsule and full preceding
event history are saved in [RUN_P3_200](RUN_P3_200/engineering_summary.json).
No new Schur binary capsule exists because tx115 never entered that stage.

## Limits

No GT was read; no ATE/RPE, accuracy table, full 2777 scan replay, Floor01,
visual experiment, sweep or further estimator repair was performed. Resource
numbers include enabled forensics and are short-link engineering diagnostics,
not a paper benchmark. No localization accuracy PASS is claimed.

P3-100 clears the authorized repair gate; P3-200 exposes a separate unresolved
optimizer issue. It is not hidden or repaired here. Complete detailed handoff:
[DECISION_AI_HANDOFF.md](DECISION_AI_HANDOFF.md). Execution stopped.
