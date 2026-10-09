# Corridor01 fixed causal scan-to-map diagnostic

This is a new non-GT, independent scan-to-map diagnostic, not a replay of
Bootstrap V2, not an IKFoM initialization claim, and not a cross-dataset accuracy
PASS. Its rules are fixed before the first real NDT call.

## Startup evidence and exact boundary

The official Corridor01 YAML has `# s 67` and two `world_darpa` matrices.
Pinned SuperOdom `f10e65cd50007767b22e4c401689665e20d827d6` consumes
`init_x/y/z/rpy` or `start_pose.txt`, not those matrix keys. Its pose-file reader
reads duration but the localization initializer does not use it to schedule
the first scan. No inspected official source proves that the comment supplies
this dataset's pose timestamp or that `darpa` is the moving LiDAR/IMU frame.
We do not infer a transform direction or adopt +67 s from a comment.

Historical P8 +67 s experiments are in Git objects `5f78ba4`, `b35554c`, and
`352bdc0`; they are not on the current R4 branch. The raw-map/world identity
and IMU interpretation in P8 were explicit task assumptions. The independent
P8 check still states that relation is conditional and reports about 19.7 deg
wall-normal disagreement. Historical successful NDT checks therefore do not
close the initialization semantics. The earlier P3-R3B map/GT chain is only
Level-2 consistency: even a rotated control has overlapping corridor geometry.
It is not a uniquely validated coordinate transform. No GT is read here,
including for startup; the single-GT-initial-pose mode is NOT RUN.

A lawful sensor-only prior is already available from the archived first 5 s
initializer. The original initializer estimates the **first scan** pose by
using those 5 s, not the pose at the end of 5 s. Its YAML has
`t_init_start=1517157219.18898`, so the conservative availability cutoff is
`1517157224.188980000 s`. The old map normalizer explicitly applies
`inverse(T_map_first_lidar)` to the original official PCD. Identity in that
normalized map is thus the historical first-scan prior, not a fabricated
pose at the new start time.

The new fixed start is TX52, whose scan interval is
`[1517157224.231677055,1517157224.332516266] s`. The **whole scan** is after the
prior's availability cutoff and is covered by the real IMU. At startup we use
identity only as a stale approximate NDT seed. No old registration trajectory,
velocity/gravity constants from GT, or old full-SE3 adapter odometry is used.
Do not backdate the initializer as a causal TX1 measurement.

## Input and propagation

Reuse `P9_CORRIDOR01_RAW_SCANEND_V1`: 2777 scans, 55957 IMU rows, 79932911
timed points. Stream one raw scan at a time. Verify all ten frozen data hashes,
official calibration/init, raw/normalized PCD, and the frozen proper-SO3
extrinsic receipt before execution. No extraction, resampling, or map changes.

Each scan is rotationally compensated to its actual scan end by causal
midpoint gyro integration, with the same integration contract as the archived
V2 input helper. This helper does **not** run V2/GICP. It includes the entire
LiDAR/IMU lever arm via `inverse(T_imu_lidar)*R_relative*T_imu_lidar`.
Only IMU samples at or before the current scan end can enter interpolation;
endpoint hold <=10 ms, real gap <=20 ms, no fabricated leading sample.
Gyro bias is provisional zero, NOT an estimate. Translation deskew is NOT
PERFORMED: this limitation is explicit, and source clouds are exactly shared
between all three arms, independently of their feedback poses.

Prediction uses causal gyro rotation and map-frame constant velocity derived
from the **previous accepted IMU-origin poses** of that arm. The initial zero
velocity is a prior, not estimated truth. LiDAR/IMU lever arm is retained when
predicting the LiDAR origin. This is a minimal scan-to-map motion predictor,
NOT the trusted full P7/IKFoM inertial state. Anchor uses the unchanged R6
2-second lifecycle, but the supplied propagation is gyro/CV, not a second EKF
or a certified inertial anchor. Full translation deskew and full IKFoM causal
localization remain NOT RUN.

## Frozen comparisons and budgets

- A: existing `CurrentFrameNdtRegistration::align()` only.
- B: same align plus existing `weakRefinement()` with `coupled=false`.
- C: same align plus existing `weakRefinement()` with `coupled=true`.

One registration object loads one target/map. Three small arm states are
independent; a selected pose affects that arm's subsequent predictions. The
source is the same for every arm and prepared count/hash must match exactly.
No candidate layer uses GT, oracle IDs, or the other arms' poses. The C-arm
logs its weak candidate before strong correction as a paired same-input
ablation; divergent arm trajectories are not interpreted as a one-frame
causal attribution.

NDT remains .8/.08/1e-5/80; preprocessing .15/.15/.25 m, source cap1400,
range .5–80 m, minimum50 points. Strict existing `SUCCESS` is required: an
80-iteration PCL flag is not accepted. Failed matches propagate the previous
causal prediction and remain explicit failure rows; no resets or frame deletion.

R6 event, Anchor, objective/quality, rho, weak/strong caps and complete
map-product pullback are unchanged. Each arm has one full align/frame, at most
two extra jets and three value-only evaluations, no extra align. Ordinary
frames must have zero extra optimization. Local regularized corrections are
not reported as independently converged PCL matches.

First attempt: TX52 must have strict nominal SUCCESS, otherwise stop and keep
all three startup results. First fixed segment: TX52–351 (300 scans). Extend
to all remaining TX52–2777 only if every arm has >=270 effective nominal NDT
results at that checkpoint and no invalid executed pose. This is a pre-run
engineering extension gate, NOT a selection by GT or a changed accuracy gate.
No alternative start or tuning run follows a failure.

## Reporting boundaries

Report strict NDT success, failure/recovery transitions, longest failure run,
trigger/Anchor/refinement/feedback counts, actual strong changes, numerical
statuses, source identity, per-arm complete mean/P95/max cost, peak process
RSS. Complete per-arm cost includes the shared raw read/gyro deskew cost
(charged fully, not divided among arms) plus that arm's preprocessing/align/
prediction/refinement. Report combined experiment wall time separately.
Jump diagnostic: consecutive executed displacement >.5 m OR rotation >10 deg,
excluding the first initialization correction; this is not a wrong-match label.

Without a reliable independently closed GT/map transform, absolute RMSE/P95/
max localization error, wrong-match and true-recovery counts are NOT_AVAILABLE.
Convergence does not establish correct position. A non-GT objective gain or
continuity advantage alone cannot support a localization-accuracy PASS.

No Floor01 tuning, new covariance weighting, new Anchor rules, production
filter edits, Bootstrap rerun, GT fit, or new data download is authorized.
