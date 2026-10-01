# Extended gate FAIL; first-failure STOP obeyed

ONE run with frame_limit200 was started after P3-100 satisfied all required
contracts. It stopped at tx115, LIDAR_SCAN_END1517157230686328484; process exit1.
No restart or follow-up localization run occurred.

```
failure=producer_optimizer:all_optimizer_candidates_rejected
optimizer_status=FAILED_ALL_CANDIDATES
optimizer_iterations=1
initial/final cost=81.345724589884568
```

This is an OPTIMIZER_STAGE failure, BEFORE marginalization for tx115.
It is not the old tx90 M7 transpose-alias failure. No conclusion that the
production root cause at tx115 is isolated is made in this repair stage.

Exact first-candidate evidence:

| Field | Value |
|---|---:|
| Damping before / after | 1e-6 / 1e-5 |
| Gradient infinity norm | 2.8457479913868156e-5 |
| Raw / applied step norm | 7.0568167407415232e-9 / same |
| Clipped | NO |
| g dot step | -2.1331542939970057e-14 |
| step H step | 8.5643792127705275e-15 |
| Predicted reduction | 3.4098706667169588e-14 |
| Candidate surrogate cost | 81.345724589884583 |
| Actual reduction | -1.4210854715202004e-14 |
| Accepted | NO |
| Inner basis recomputation calls | 0 |
| Solver | BLOCK_SPARSE_SIMPLICIAL_LDLT |

These tiny candidate changes are reported as recorded facts only. No step,
gradient, acceptance, damping-budget, covariance or NIS tolerance was changed.
Existing automatic failure diagnostics were allowed to flush before throw;
their state difference0 confirms transaction safety. Their sweep candidates
were never committed and were not a new tuning experiment.

Tx115 preopt: Window41nodes/span2.017077208s; nominal NDT converged, fitness
75.624965709133562, objective281.35544678059625,34iterations. U_obs valid,
rank5/weak1; P15 valid, eigenrange4.3335135398224997e-6 to0.10929838668829893.
U_nonlocal triggered with both recorded terminals. Selected NIS
15.237396865140097 >15.086, so current LiDAR factor rejected normally;
factor counts before optimize40IMU/17LiDAR/0visual. This does not explain
away the optimizer failure; it preserves the causal evidence without fixing it.

Prefix:63 complete terminals through tx114;64 deskews including tx115;
57 committed/6 NIS rejected among completed terminals;38 completed probe
events;139 NDT calls in completed runtime rows. Failing tx115 nominal/probe
calls are present in the preopt capsule but not the completed runtime ledger;
139 is explicitly not presented as a full-run total.

88/88 earlier oldest removals succeeded in44 two-removal enforcements. Stored
prior symmetry0, minimum recorded eigenvalue0, no jitter, no dense fallback.
Completed states finite and bounded (max40nodes,1.958914906s). The failed
preopt window's span exceeds2s and was NOT subsequently enforced; final bound
is therefore not claimed for the incomplete run. No natural attempt2 failure
occurred; later-failure batch rollback remains untested/unchanged.

Saved evidence includes all completed prior events (more than20 before failure),
tx115 preopt capsule, exact first-candidate vector/trace, existing derivative
and damping diagnostics, failure summaries, deskew data and runtime/resource
records. No marginalization binary was produced because failure was earlier.

Resource49.21s wall,46.43s user,2.53s system,93,516KiB RSS; diagnostics only.
No GT, accuracy conclusion, full replay, parameter search or root-cause repair.

A3C_R2_P3_200_EXTENDED_GATE=FAIL
READY_FOR_FORMAL_EXPERIMENT=NO
