# P12 minimal DCReg + SOLiD integration — frozen before registration

Task: P12-DCREG-SOLID-MINIMAL-INTEGRATION-R1. Development integration only;
no new mathematical algorithm, localization feedback or accuracy claim.

Start: `578291c999ea375ff72538da7b6a34fb54dc85bb`, branch
`research/p9-r4-heldout-visual-evidence`.

## Author implementations and input

DCReg external checkout: `/home/jian/rosbag/p12_dcreg_solid_r1_20261011/upstream_dcreg`,
author repository https://github.com/JokerJohn/DCReg, pinned
`8ce8451b15491a4bbe17cf85ab02a8bed6696861`. No license statement was found in
the pinned checkout. Neither author source nor compiled binaries are published
in this paper repository. Own adapter includes the external author `dcreg.hpp`
and calls `dcreg::RunRegistration(source,target,test_case,options)` unchanged.
The author's CMake requires 3.25; this host has 3.16.3. Our standalone CMake
compiles the same unmodified headers with installed Eigen/PCL, without editing
upstream build files or mathematics.

SOLiD uses the existing P11 `SolidPlaceRecognitionAdapter::query(source,3)`;
author pin `9a4f2895e5211f69fbbd6d4732577e1a67e9b882`. No changes to SOLiD.
The 389-reference Loop2 DB, generated from independent offline mapping scans,
and the active Loop2 prior map remain unchanged. Queries are independent Loop3.

P11 did not record `/dog_livo/filtered_points` payloads in its output bag.
Only the 24 frozen raw scans are extracted from the original indexed bag, then
reproduce the NDT preprocessing (finite XYZ, .5–80m, .25m voxel, max1400
uniform index sampling). Each resulting source count and FNV content hash must
exactly match the actual P11 NDT diagnostic. No new full ROS replay, full bag
extraction, download, deskew or point-time reconstruction occurs.

Target is the same NDT map preparation: finite XYZ, .15m voxel followed by
.15m target voxel, no cap. No map crop or normal precomputation replaces the
author's correspondence routine. Source is `livox_frame`; target is
`camera_init`. The frozen robot baseline has deskew=false; this limitation is
preserved, not silently upgraded to a deskewed experiment. Header timestamp is
scan start; recorded NDT prediction/result timing is midpoint, and both timing
records remain attached. The probe reproduces the same baseline payload/prior
association; it does not prove a new exact deskew/scan-end contract.

## Sample freeze

24 frames: 16,45,124,708,1285,1413,1891,1994,2011,2237,2696,3058,3409,3493,
3686,3914,4105,4586,4691,4800,4881,5726,5813,5814.
Each of three strata contributes eight, selected by evenly spaced indices in
the corresponding chronological prior-P11 metadata pool, excluding duplicates:

1. Ordinary/low Top3 reference spread (<5m).
2. Straight-motion corridor proxy: prior NDT positions over ±25 scans have
   >2m endpoint displacement and first singular-value energy fraction >.98.
   This is **not** an independently verified long-corridor scene label.
3. Widely separated Top3 references (>50m pairwise spread).

Selection uses existing metadata, never GT or this round's DCReg results.
`samples.csv` and `freeze.json` are immutable before any real registration.
First two samples are the interface smoke; remaining22 complete the same
frozen cohort, with no sample reselection or solver-parameter trials.

## Actual connected flow and coordinate limitation

Local path: verified NDT source + prepared map + **recorded NDT initial guess**
-> author DCReg. NDT raw PCL terminal and step-limited used pose are retained
separately as diagnostics, never treated as GT or used as online init.

Combined path: exact same source -> actual SOLiD query Top3 -> each returned
`map_T_reference` -> `COARSE_POSITION_HYPOTHESIS` -> author DCReg.

FAST-LIVO2's pose logger stores `_state.pos_end/rot_end`, the mapping IMU-origin
state. Exact historical physical extrinsic and raw-scan-to-state time transport
were not independently closed in P11. Therefore **exact pose conversion is not
claimed**. For the expressly permitted coarse experiment:

```
R_map_lidar_initial = current recorded NDT INITIAL rotation
p_map_lidar_initial = candidate map reference IMU-origin position (coarse)
```

No fictitious lever arm, reference yaw conversion, GT or future data is added.
The hypothesis includes IMU/LiDAR origin and time-offset approximation, as well
as potentially wrong place-specific orientation. Successful numerical matching
does not establish the correctness of the retrieved place.

Points stay in source coordinates. Author internal `R*p+t` applies the initial
transform; its output already is map_T_source and is NOT multiplied by the
initial transform again. Output SE(3) checks: finite, orthogonality/determinant
tolerance1e-5, homogeneous bottom row tolerance1e-9.

## Frozen DCReg budget and score meaning

Algorithm DCReg; parameterization SO3 (as the real-scene author example);
parallel Serial; verbose=false; has_ground_truth_pose=false. All author
SolverParameters defaults are retained: radius1m, max30 iterations,
translation convergence1e-3m, rotation1e-5rad, degeneracy condition10,
kappa10, PCG tolerance1e-6/max10, plane neighbors5/thickness.2m,
weight slope.9/min.1, weight derivative enabled. Four calls per scan:
one existing-prior local and three actual SOLiD candidates. No additional NDT.

`success`/`converged` mean author iteration convergence, not correct location.
Author fitness is valid-correspondence fraction; author RMSE is the last
iteration's **pre-update** point-to-plane residual. Not comparable to the NDT
native fitness scale. Degenerate masks are the union of author masks across
iterations, ordered roll,pitch,yaw,x,y,z in its physical-axis characterization.
Large excursion (>3m or >45deg from initial) is a frozen diagnostic, not a new
author solver stopping/acceptance rule.

## Timing and stop rule

Current descriptor/search stage timers are reported. Complete DCReg call wall
time wraps the original API including target copy/KD-tree setup omitted by its
internal iteration timer. Measured combined critical path starts immediately
before actual query, includes three registration calls and result logging, and
excludes the separately executed local-prior control. Total probe scan cost
also includes PCD reading/preprocessing and local registration. Map/DB load
are one-time startup cost, not per-scan query cost. Peak RSS measured on actual
process; no real-time success claim. No scores choose feedback or EKF updates.

Stop after this fixed small experiment and factual archival. Exact calibrated
candidate poses, retrieval accuracy and localization feedback remain unproved.
