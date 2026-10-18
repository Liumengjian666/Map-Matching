# P60-R3 Ground-Challenge LiDAR–Xsens extrinsic recovery

## Result

**EXTRINSIC_SOURCE_NOT_FOUND** — bounded public-source investigation found no
applicable full `T_I_L`. No calibrated matrix was produced or accepted. This is
not a statement that calibration is mathematically impossible, or that an unpublished
author configuration does not exist. Full 6DoF observability remains uncertified.
**Deskew + EKF fusion Smoke: NOT APPROVED by this evidence.**

`p_I = R_I_L p_L + t_I_L`, where I is Xsens `/imu/data` frame `imu`, and L is
the published `/velodyne_points` frame `sensor`. Even whether that published frame
already includes an unrecorded driver transform requires producer configuration.

| Requested transform / validation | Result |
|---|---|
| `R_I_L`, `t_I_L`, full `T_I_L` | UNCONFIRMED; no numerical matrix supplied |
| `T_L_I` | `inverse(T_I_L)` symbolically only |
| SO(3), determinant, inverse consistency | N/A: no admissible numerical candidate |
| Cross-sensor rotation validation on independent windows | NOT EXECUTED: no candidate; IMU-only windows are not this validation |
| Calibration parameter uncertainty | NOT ESTIMATED; no zero/default covariance invented |
| LiDAR–Xsens clock offset | UNCONFIRMED; shared ROS time does not prove zero offset |
| Point-time semantics | Standard Velodyne header-relative seconds strongly supported; exact bag producer version/config NOT VERIFIED |
| IMU coverage under that time interpretation | 973/974; scan 1 not covered |
| Prior-map/init contract | Still separate; first-scan self-map is engineering-only |

## Verified input and bounded work

Bag: `/home/jian/ground-challenge/corridor2/corridor2.bag`.
Size 2,871,922,599 bytes. Full SHA256 recomputed this round:
`6ef85c76a36bdcef4811d411ac0f09219803377f5b7b9bd0cf2a5258cc826cea`.
This matches P60-R2. Fresh offline read inspected all 974 clouds and all 39,315
Xsens measurements; no pseudo-GT topic/file was read.

The R2 contract JSON, reused for TF inventory only, has SHA256
`2550abf04fd631b3c160c63458c52fc969db3ab69393712e7b7c5e9820a50b7d`.
Its TF graph has no `sensor` or `imu` connection; it cannot recover this extrinsic.

Public evidence and commit-specific paths are in `official_evidence.md` and
`source_manifest.json`. Ground-Challenge `calibration.txt` contains camera/body
and wheel/body transforms, not a VLP-16–Xsens transform. Ground-Fusion and
Ground-Fusion++ Ground-Challenge VIO configurations explicitly use `/camera/imu`.
Other LIO configurations belong to M2DGR/M2DGR-plus/M3DGR and were rejected as
non-transferable. The original paper describes software synchronization and
camera–IMU calibration, but does not supply the missing LiDAR–Xsens matrix.

GitHub issue titles #10/#11 ask about extrinsics. Their bodies/replies were not
verified: API attempts returned HTTP 403. We do **not** claim the author has
never replied or has never published a matrix elsewhere. This bounded search stops here.

## Motion feasibility, not a calibration result

Fixed analysis: IMU-only full sequence and 0–30 / 30–60 / 60–end seconds;
100 Hz analysis grid, fixed 21-sample order-3 Savitzky–Golay filter. Original
timestamps/measurements are untouched. Raw and smoothed diagnostics are both retained.

Smoothed gyro RMS `[0.012693, 0.016425, 0.140061] rad/s`.
Dominant axis `[0.044356, 0.007998, 0.998984]` carries **98.0486%** of gyro energy.
Its fraction in the three fixed windows is 99.1255%, 96.4585%, 97.6571%.
Auxiliary Xsens orientation roll/pitch spans are 0.693764° / 0.969363°.
These quaternions are IMU internal orientation estimates, not independent truth.

Rotation alignment sensitivity eigenvalues are
`[0.000391211, 0.0197760, 0.0199287] (rad/s)^2`; ratio 0.0196305.
This is a gyro alignment *design diagnostic*, not joint calibration information.
The lever-arm diagnostic is numerically full rank (smoothed ratio 0.724541),
and raw derivative energy is very large. Thus **strict unobservability is not
established for this real bag**. Treating small nonzero motions/vibration as
certified full-rank LiDAR–IMU information would be equally unjustified.

`observability.md` gives the exact planar nullspace argument, the real-data
counter-evidence, nuisance-state limitations, and LI-Init / GRIL-Calib prerequisites.
No calibrator/dependencies were installed. No height or extrinsic was guessed.

## Point-time findings

Observed FLOAT32 `time` is in `[-0.107265949249, +0.001306368038]`.
Standard Velodyne source calculates
`offset = packet_stamp - scan_header_stamp + firing_offset_seconds`.
Default scan header is the last packet, explaining negative offsets and a small
positive tail. Maximum VLP-16 packet firing offset is 1.306368 ms, matching the
observed maximum. Therefore do **not** clip negatives or add an arbitrary 0.1 s.

Under `t_point_ns = header_ns + round(time*1e9)`, scan 1 starts at
`1658917771506630421`; first IMU is `1658917771516438961`: missing **9.808540 ms**.
Scans 2–974 are bracketed. Whole-sequence maximum IMU gap is 12.513876 ms.
Exact driver identity, fixed/target-frame settings and cross-sensor time offset
remain open; this is conditional coverage, not complete time-contract certification.

## Execution and delivery

New NDT align = 0; ROS nodes = 0; Deskew/fusion runs = 0; Coupled/Weak-only = 0;
GT reads = 0; HIKVISION access = 0. No production `src/` modifications.
Five diagnostic unit tests passed with warnings treated as errors; all three
Python tools pass py_compile. The code-review check kept the full-rank counter-evidence
and source-access limitation rather than overstating an unobservability conclusion.

Reproduce (new output directory required, existing outputs never overwritten):

```bash
cd /home/jian/livox_ws/p60_r3_ground_challenge_extrinsic_ws/docs/p60_r3_ground_challenge_extrinsic_recovery
/usr/bin/python3 -W error -m unittest test_excitation -v
/usr/bin/python3 analyze_motion.py /home/jian/ground-challenge/corridor2/corridor2.bag /home/jian/livox_ws/p60_r3_ground_challenge_extrinsic_verify
```

START_SHA = `7f04bcb3a6a2b3db43738ba5463789d7ed08c2d2`.
CODE_SHA = `5e53ecb960b2159fdf6d60e8b3bc6fb73eefac11`.
Branch = `research/p60-r3-ground-challenge-extrinsic-recovery`.
END_SHA is the archive commit containing this report (query `git rev-parse HEAD`).
Remote R2 HEAD verified as START_SHA; R3 remote ref absent before delivery.
PUSH_EXECUTED = false. No remote delivery is claimed.

## Unique NEXT (not executed)

Ask the dataset author for the **actual Corridor2 FAST-LIO2/LiDAR acquisition
configuration**, including VLP-16→Xsens matrix, frame definitions, applicable rig
version and time convention. This single provenance request is preferable to
running unconstrained calibration or substituting a different platform's matrix.
Then let the research lead decide whether to approve a new bounded validation task.
