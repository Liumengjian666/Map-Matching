# PAPER-P3-R7 Corridor02 cross-sequence validation

## Result

`PAPER-P3-R7-PARTIAL`

Reason: `GT_REFERENCE_UNRESOLVED` and official 15.6 GB bag download blocked by
Google Drive quota. The stage stops before counterfactual recoverability; no
cross-sequence scientific claim is made.

## What was established

1. The official release identifies Corridor02 as an independent RC1 SuperLoc
   sequence (Hawkins, RGB/LiDAR/IMU, trajectory index 690, 893 s).
2. Official static assets are present locally and match the recorded SHA256
   values.
3. RC1 extrinsic matrices were recorded without substituting Corridor01 RC2
   calibration. The official source supports the laser-to-IMU convention.
4. The official bag is 15,589,213,681 bytes, but Google Drive currently
   returns a quota-exceeded HTML response; it was not accepted as a bag.
5. The official SuperLoc release does not identify the GT pose origin or an
   absolute map-to-GT transform. The separate ICCV SubT-MRS IMU-pose statement
   cannot be inherited by this SuperLoc-sourced sequence.

## Not performed

No `rosbag info`, point-time audit, full-SE(3) adapter, derived bag, Run A/B,
determinism gate, failure timeline, replay gate, oracle initialization, local
perturbation, or cross-sequence comparison was performed. Required P3-R7 CSVs
and plots are intentionally absent rather than fabricated.

`P4_ALLOWED = NO`.
