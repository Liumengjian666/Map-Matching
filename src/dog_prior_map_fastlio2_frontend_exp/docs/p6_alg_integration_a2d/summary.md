# PAPER-P6-ALG-INTEGRATION-A2D

Engineering result: **PASS for the authorized synthetic/infrastructure scope**.
`READY_FOR_FORMAL_EXPERIMENT = NO`.

START_SHA: `b04a1f7f3442266dd48bd2efea5baf09d905b0c8`.
CODE_SHA: `14f0e965639ce7c0e022e77a750670024b5aff37`.
Branch: `research/p6-i6d-full-algorithm`, existing worktree
`/home/jian/livox_ws/dog_loc_p6_i6b_ws`. No new worktree or estimator architecture.
The final evidence commit follows CODE_SHA; see final delivery for its END_SHA.

## Ten required answers

| Question | Answer |
|---|---|
| V3 NDT source wholly Window-owned raw timed deskew? | YES. Raw provider → active optimized scan-start → shared IMU model → SE3 deskew → preprocessSource → existing PCL NDT. Verified on a real-PCL synthetic fixture. |
| Can legacy IKFoM-deskewed cloud enter V3? | NO under the typed provenance contract. Non-raw products reject, and the legacy XYZ provider is never invoked. This assumes truthful exporter metadata, not cryptographic inference of state lineage from XYZ bytes. |
| Can legacy-state-derived visual depth enter formal V3? | NO. Legacy or UNKNOWN provenance does not submit a visual factor. Only RAW_SENSOR_LOCAL_DEPTH / WINDOW_OWNED_DEPTH are eligible. |
| Runtime IMU Jacobian? | ANALYTIC, including nonlinear bias correction. LiDAR state Jacobian is analytic too. FD remains a test oracle; prior-chart rotation FD remains. |
| Complete Hessian eigensolve every optimizer iteration? | NO by default. Explicit debug diagnostics only, with no optimizer-decision contribution. |
| Primary optimizer solve? | 15x15 BLOCK/SPARSE assembly and Eigen SimplicialLDLT; explicit dense fallback/reference retained. |
| Mathematical equivalence? | PASS: independent dense H/g/cost parity plus same-system damping/step and final-state parity. Not a claim of bit-identical real trajectories. |
| Schur information conservation? | PASS. Prior + oldest-touching only; retained active factors are not double counted. Existing and new regression pass. |
| Real data missing raw point time? | Floor01 request_xyz_f32.bin and Corridor01 I6C prepared XYZ bundle. The latter is rotation-only deskewed. Original bags were not re-exported this round. |
| Is only short real-link validation left? | NO. Genuine timed-point export and lineage manifests are still required; independently valid raw/window-owned visual depth products are also missing. Then short causal real-link validation is needed. No formal experiment authorization is implied. |

## Evidence

Release build PASS; CTest **21/21**. Debug targeted build PASS; CTest **5/5**.
`git diff --check` PASS. Existing A1/A2A/A2B/A2C tests retained; the old joint
test explicitly enables optional rank diagnostics without removing its assertion.

Deskew six-motion maximum error 3.30093e-15 m; same-trajectory old geometry
comparison PASS; old runtime end-to-end tests PASS. Analytic-vs-FD maxima:
IMU 3.95194e-9; LiDAR 2.19452e-9. Exact principal-Log branch cut is explicitly
indeterminate for derivatives, not silently resolved by a FD branch.

Block/dense errors: H 7.52677e-19 relative, g 1.01248e-16 relative,
cost 1.73472e-18 absolute; solver step 6.06171e-15 relative; optimizer state
difference 3.86139e-16. Repeated marginalization and explicit dense fallback pass.

V3 synthetic PCL fixture checks four frozen R2 policies. Each has 10 events,
3 Window deskews/LiDAR commits, 1 probe pair and 5 NDT calls. Post-handoff IKFoM
calls=0. The full-rank map fixture does not force visual admission; separate
directional adapter tests exercise real visual-factor admission and start-state
joint correction. Reader/provenance rejection tests pass.

Synthetic 48-node run: dense/block assembly 91.274/1.51257 ms;
dense/sparse solve 17.7264/2.20351 ms; latest marginal 113.051 ms. These are
single-run sanity numbers, **not a formal CPU/RAM/WCET result**. Marginal covariance
and dense prior remain substantive costs. Storage fields are payload accounting,
not process RSS.

The legacy FULL producer body is byte-identical after stripping additive
experimental include/dispatch/usage changes, frozen source hash
`498ec598db3aaec84b74391292d9db7928b2337944be01287621502135abed34`.
No frozen baseline/result, calibration, target preprocessing, PCL objective,
NDT parameter or research acceptance threshold was modified. No GT, complete
Floor01/Corridor01 replay or raw-camera frontend regeneration was executed.

## Delivery and limitations

All required audits, math/test reports, source identities and small scaling CSV
are in this directory. `DECISION_AI_HANDOFF.md` is the detailed manual-review
prompt requested by the user; remote upload status is reported separately after
normal push verification. Nothing relies on an uncommitted /tmp dataset asset.

Independent read-only reviews found no blocking sign/frame/assembly/provenance
error. They identified a near-pi guard risk and stale solver diagnostics, both
addressed. External cross-model review remains the user's final manual action.
No further phase or full trajectory execution is initiated.
