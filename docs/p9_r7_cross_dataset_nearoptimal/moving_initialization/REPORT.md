# R7-R4 causal moving initialization

FINAL_RESULT = `BOOTSTRAP_REGISTRATION_QUALITY_FAIL`.

NEXT = `REASSESS_MOVING_INITIALIZATION_MODEL`.

The user explicitly authorized this additional engineering STOP category after
the first bootstrap registration-quality failure was found. It is not a claim
of mathematical unobservability, a DUAL-U scientific failure, or restored R7
baseline readiness. No frozen gate was relaxed after observing data.

## Git and inputs

Branch: `research/p9-r4-heldout-visual-evidence`.
Start: `c3e4a41c239b2c9bcf28497acd5b84a5d9ec2568`.
End: the commit containing this archive; obtain its full SHA with `git rev-parse HEAD`.
Worktree: `/tmp/dog_loc_paper_r4_ws.Fq21k2`. No push executed.

Protocol: `P9_CORRIDOR01_RAW_SCANEND_V1`. All ten manifest input-file hashes
were verified, including the four consumed raw/IMU files. There was no new raw
extraction. Counts remain 2777 scans, 55957 IMU samples, 79932911 timed points.
The original bag receipt remains
`c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811`.
The 1.28 GB point payload is read per scan, not loaded as one cloud.

The normalized map, historical sensor-only first-five-second map anchor, and
calibration hashes were checked. That anchor is a first-frame pose, not a full
moving state at five seconds. Its timestamp differs from raw TX1 scan end by
about 22 microseconds; no exact historical input equivalence is asserted.
The rounded calibration rotation is projected once to proper SO(3), with the
change recorded in `bootstrap_freeze.json`; native IMU data are not rotated twice.

## Bootstrap and first failure

First sensor timestamp: `1517157219088119030` ns.
Ten-second deadline: `1517157229088119030` ns.
Latest legal scan end not beyond deadline: `1517157229072630478` ns (TX99).
Actual bootstrap span: 9.984511448 s. All used observations are no later than
this endpoint. The first scan's missing leading IMU is retained as bootstrap
boundary data and is never treated as an initialized formal transaction.

Mature code was inspected: FAST-LIO2, LIO-SAM, SuperLoc preintegration usage,
Point-LIO startup code, and the project's sensor-only initializer. LI-Init and
a usable standard GTSAM installation were not found in the bounded search.
Reuse is PCL 1.10 GICP, existing P7 readers/IKFoM/deskew, and a small standard
midpoint preintegration adapter, not an imported full moving LIO optimizer.

`bootstrap_config.json` was frozen before any registration or fitting. Adjacent
raw-scan GICP uses 0.25 m voxel, 2.5 m correspondence distance, 40 iterations,
and transformation epsilon 1e-6. The quality gate includes **untruncated PCL
all-point nearest-neighbor fitness <=0.10 m²**. TX2 instead gave 0.704497419 m²
despite solver convergence and a finite 0.0908 m / 3.2926 degree increment.
The admitted execution stopped there. Its exact failure receipt is preserved.

This fitness gate is a new predeclared engineering gate, not an inherited
oracle threshold. Untruncated fitness includes non-overlap/outlier points;
its failure does not prove the entire relative pose is wrong. No post-result
change to fitness range, threshold, or registration parameters was made.

Separately, explicitly untrusted diagnostic-only continuation covered TX1--99:
98/98 GICP pairs reported solver convergence, but only **32/98** passed the
quality gate. These poses are end-reference approximations from uncompensated
scans, not exact scan-end motion truth. They cannot restore state eligibility.

## State diagnostics, not accepted initialization

Fit data are restricted to 5--8 s, with disjoint 5--6.5 / 6.5--8 s diagnostics.
The 8--boot interval is held out. IMU integration clips at every requested end;
future samples cannot affect interpolation. A bounded terminal hold is recorded
within the previously frozen 0.01 s limit; no leading IMU backfill is allowed.

Tentative main-fit velocity is approximately [1.8640, -0.20495, -0.13371] m/s;
bg [0.01476, -0.02273, -0.000583] rad/s; ba [0.8430, -0.7017, 0.02129] m/s²;
gravity [-0.3208, -1.2438, -9.7245] m/s². **None is an accepted initial state.**
They are provided only to explain why a raw-scan fit cannot be certified.

Gravity-tangent/bias profile: numerical rank 5/5, singular-value condition
25.5581 (normal-equation condition 653.2155), minimum singular value 0.0138904.
The orientation-uncertainty perturbation bound is 0.2812853, so the conservative
minimum-singular-value lower bound is zero. This cannot certify rank; it does
**not** prove that the true physical system has zero rank or is globally
unobservable. Motion-derived uncertainty itself is only an engineering estimate.

Split-fit instability: bg change 0.0574098 rad/s, ba change 18.2241 m/s²,
gravity direction change 132.5669 degrees. Independent-interval diagnostic
prediction RMSE is **0.461450 m / 5.460501 degrees**, exceeding the frozen
0.10 m / 2 degree limits. The pose observations are themselves untrusted, so
these are bootstrap diagnostics, not GT accuracy measurements.

Full coupled state covariance is NOT_RUN following the upstream rejection.
No singular profile is inverted into a deceptively finite accepted covariance.
Position/orientation/velocity/bg/ba/gravity and stamp are all NOT_ACCEPTED.

## IKFoM and verification

Added `initializeMoving(MovingInitializationState)` with explicit 17-DOF input
covariance and map-frame Cartesian gravity tangent basis. It converts gravity
covariance/cross terms to the actual pinned IKFoM S2 coordinates and restores
the fixed-extrinsic constraint. It rejects invalid stamp/state/quaternion,
gravity, basis, covariance, and reinitialization before mutation.
The body of `initializeStatic()` is byte unchanged.

Release build PASS. P9 tests 41/41 PASS; moving/P7 tests 6/6 PASS (eight Python
test cases plus the full-state C++ test and existing P7/frame/deskew tests).
Regressions cover nonzero moving velocity, signed anisotropic gravity
covariance and tilted frames, fixed extrinsics, invalid-state atomicity, the
pinned-S2/runtime-gravity boundary, IMU future-sample tampering, accelerometer
bias signs, unexcited gravity/bias rank, and required-input manifest omission.
The actual IMU sample pair bracketing each integration start is also checked;
artificial endpoint knots cannot hide a missing-IMU gap. Adding this guard
does not alter the stored diagnostic integrals; real-data revalidation is
archived separately. Executed estimator sources are retained in the external cache.

Independent read-only review found and corrected the endpoint causality and
state-injection atomicity issues, plus hash/measurement covariance test gaps.
Per the user's choice, no external cross-model CLI was run. A first P9 test
attempt hit an incompatible `/opt/MVS` libusb; explicit system-library lookup
resolved it without changing code, data, or mathematical parameters.

All 2777 scans are in the ledger: 99 bootstrap and 2678 post-boot inputs NOT_RUN.
Real IKFoM injection, scan-end propagation/deskew, nominal source/T0/U_obs/W2,
formal NDT smoke, Run A and Run B, and their parity checks are NOT_RUN.
Historical NOMINAL_TRACKING_RISK is retained; no new trajectory or GT claim.

## Cost and boundaries

Rejected GICP execution 0.05 s; diagnostic GICP continuation 2.82 s.
Mean/P95 diagnostic per-pair processing 28.528 / 48.672 ms (includes cloud
reading/filtering/registration, not NDT alignment time).
GICP measured peak RSS 19344 KiB. Python fit peak RSS was not measured.
Corrected diagnostics plus input hash recheck took 4.388889 s. The preceding
3.583486 s pre-endpoint-fix diagnostic is preserved externally as superseded.
These are preparation costs, not DUAL-U online increments.

Smoke/Run A/Run B NDT calls = 0/0/0. New oracle263 = 0, B12 = 0,
visual extraction = 0, GT loaded = NO. No core NDT, U_obs, W2, R6 gamma,
static initialization gate, production deskew, or fusion algorithm was changed.
Historical archives and the stable machine-dog workspace were not modified.
Persistent diagnostic cache: Corridor01/results/p9_corridor01_moving_init_v1/.

The engineering STOP is the finding. It neither restores cross-dataset
readiness nor rules out a better causally valid moving-initialization model.
