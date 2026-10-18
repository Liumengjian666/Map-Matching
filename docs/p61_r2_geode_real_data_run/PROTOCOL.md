# P61-R2 real-data protocol (registered before native geometry results)

START_SHA ed31ade24a6bc7a36720c65321997fba1ef93ab4.
Only GEODE Shield_tunnel1_gamma, no new downloads or Ground-Challenge runs.

## Storage and identity

Read-only external source is the exact user-specified bag, 4,500,527,001 bytes,
on UUID F627-4133. Destination is /home/jian/geode/Shield_tunnel1_gamma/.
Use rsync --partial --append-verify --bwlimit=16384 to a .part, no final-file overwrite.
Continuously monitor device-associated USB4-2, sda, host0, block/filesystem errors.
Transient D-state alone is not a failure. Stop on new real storage faults or prolonged stall.
After copy, independently hash external source at <=16MiB/s and internal partial;
only equal size/SHA permits exclusive publication of the internal bag.
Keep partial on failure. All subsequent analysis reads only the internal copy.

## Real sensor inspection

Check bag magic, indexed topic metadata and traverse ALL serialized messages,
including decompression of all chunks; count reconciliation is mandatory.
Validate actual Livox schema before vectorized parsing. Preserve XYZ, reflectivity,
tag, line, offset_time, header stamp and timebase in the input contract.
Do not relabel or alter timestamps, fabricate point times, assume an IMU frame,
or use gamma_config's unverified extrinsic.

## Native LiDAR smoke and fixed screen

RAW_NATIVE_LIDAR_GEOMETRY_DIAGNOSTIC only, NOT prior-map localization/accuracy.
No Deskew, IMU prior, EKF, GT, Coupled or Recovery in this geometry phase.

1. Twenty consecutive pairs: Source indices 2..21; Target index k-1 only.
2. If engineering smoke passes, select100 unique pairs uniformly in SENSOR time
   over the complete validated LiDAR sequence, using frozen P60 relative-time
   nearest-neighbor selection, earliest-index tie break. Freeze before geometry.
3. No FoV crop, no equal-count subsampling branch; use real native Avia returns.
4. Frozen P60 Source range0.5–80m,0.25m voxel,1400 cap; Target0.15m voxel twice.
5. Native PCL NDT: resolution0.8, step0.08, epsilon0.001, max iterations40,
   outlier0.55; identity initial pose independently for EACH adjacent pair.
6. Corrected R5 ordering and strict actual target_cells_ XYZ leaf=.8f checks.
7. One align and one derivative-only jet per pair;120 total align/jet maximum
   if both phases complete. Overlap across smoke/screen is recorded, not hidden.
8. Hessian chart unchanged: p=p0+0.8 eta_t[m], R=Exp(eta_r[rad])R0.
   Mean negative native score per Source point; exact nonlinear chart pullback.
9. Hessians at each pair's native result, with comparable/spd flags kept separate.
   Support uses native Gaussian neighborhood, not NN fitness.

Engineering smoke PASS: all20 pairs have sufficient Source/Target, real .8m grid,
finite rigid converged align, finite derivative evaluation and >=50% support.
Non-SPD Hessian is retained as diagnostic, NOT itself an adapter/smoke failure.
Do not change optimizer or input parameters after seeing results.

## Frozen inference

Reuse P60 severe descriptive label: SPD, six-DoF min/max<=1e-3 AND
translation-block min/max<=1e-2. A persistent sampled candidate interval needs
>=3 consecutive selected labels with translation weak-axis similarity>=cos20deg.
Uniform samples cannot prove ALL intervening scans were weak; report this limit.
Invalid/non-SPD/unsupported Hessians are not relabeled as geometric degeneracy.
Compare only descriptively against frozen P60-R5 FULL_360 new-common-pose CSV;
never rerun Ground-Challenge or compare mixed-unit unnormalized spectra.
Weak axes are in the previous-scan Target scanner frame, not certified body/world.

## Fusion admission

Only after real point-time, chosen IMU extrinsic, IMU coverage, prior-map coverage
and initialization are all established may a20-frame fused Smoke run.
No stairs.las substitution, GT map fitting or self-map accuracy claims.
No full-sequence localization or Coupled experiment is authorized.
