# P4-I3 supplemental instruction closure

The expanded instruction adds forward/FB counters, separate projection and
association timing, body XYZ/RPY residuals and saved-pose nearest-time matching.
The original algorithm baseline is `03d38dd5a7df0c81ce1d5cf0ba5dff18c54dd451`.
This supplement starts from the completed first-pass analysis commit
`8def495a82245d52772c3bc78334adfaf797cce9`, without rewriting its history.

## Verification

- Re-read the same raw image shards and saved request clouds offline. No ROS
  playback, NDT or IKFoM execution; no runtime, config, launch or threshold edits.
- Full rerun: 4126 candidate pairs, 3751 attempted, 1802 valid.
- All 47 original non-timing columns match the first-pass CSV exactly on every
  pair, including status, feature/depth/inlier counts, transforms and errors.
- Forward count >= FB count = previous KLT count for every row.
- Projection plus association duration <= enclosing depth duration.
- All 1802 valid pairs have directional scores. XYZ residual vector norms agree
  with scalar translation errors within 1e-5 m.
- Reconstructing the predictor increment from nearest saved poses agrees with
  R10C scalar errors within 1e-5 m / 1e-4 deg. No pose repropagation occurs.
- Python compilation and Ruff checks pass. Independent read-only code review
  found no blocking issue, and independently evaluated all directional pairs.
- Synthetic PnP-direction and MEI-rectification check passes.

## Interpretation

`SPARSE_VISUAL_SIGNAL` remains the result: late coverage is 41.3233%, below 60%.
Late common-body XYZ RMSE is 0.021034/0.012398/0.012755 m for visual versus
0.273556/0.158096/0.032496 m for IKFoM on the same valid subset.
Late yaw residual RMSE is 0.395366 deg versus 0.350653 deg: no yaw superiority
or general stable-yaw claim is supported. These are residual extrinsic xyz
Euler components, not subtracted pose Euler angles.

Saved-pose nearest matching is an authorized approximation, not exact image-time
propagation. Absolute interval-duration difference mean/P95/max is
20.019414/21.473611/31.421180 ms. Native-time metrics and the prior scan-time
sensitivity check are both retained. Visual rotation fails the 20% relative
degradation threshold under the latter sensitivity check. This limits claims;
no synchronization correction, temporal model tuning or fusion was introduced.

New instrumented offline total time mean/P95/max is
23.620814/35.361493/51.758388 ms, excluding bag I/O and evaluation. Previous
timings remain accessible in the first-pass commit; timing differences alone
do not imply an optimization.
