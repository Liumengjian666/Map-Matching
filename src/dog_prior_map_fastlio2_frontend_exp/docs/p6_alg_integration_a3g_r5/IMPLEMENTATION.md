# Validated recovery reinitialization contract

R5 starts at `68005b531c8127904f63c42de5890748bea39196` (clean worktree).
Dataset: SuperLoc Corridor01; GT=false; visual=NONE.

Recovery is explicit relocalization, not an ordinary selected-NIS update.
The ordinary admission function, selected residual/covariance, chi-square
threshold, optimizer, square-root covariance and marginalization are unchanged.

## Validation and reset

Only tracking-recovery alternatives are eligible. A candidate must have finite
pose/objective/fitness, a real non-exhausted positive NDT iteration count, a
non-passthrough terminal, valid existing geometric support (the same production
minimum of 30 correspondences), valid U_obs with positive reliable dimension,
and valid existing measurement covariance. Old-window NIS remains logged but
is not a reset admission criterion.

`FixedLagEventAdapter::resetFromValidatedRecovery` uses the prepared scan-end
state. It converts the measured LiDAR pose using the existing extrinsic; copies
velocity/bg/ba exactly; and builds the block covariance:

```
P_reset = diag(C R_measurement C^T, P15_pre[6:15,6:15])
C[rotation_new,rotation_old] = R_new^T R_old
C[position_map,position_map] = I
```

The pose ordering conversion is from measurement `[position,rotation]` to
state `[rotation,position]`. Pose/non-pose cross terms are zero. Within-block
correlations are retained. LLT solves for information; no explicit inverse,
jitter, eigen clamp, epsilon-I or fallback is used. Invalid SPD construction
rejects the reset without replacing the old Window. New prior gradient is zero.

The controller delegates to an explicit Window reset: construct a replacement
with the same options/noise, run existing `initializeWithPriorAtomic`, then
replace the old graph. All old states/factors/prior/active IDs and optimized
revision are discarded. A monotonic new ID retires the discarded epoch and
the reset event itself; the adapter's global allocator and source transaction
watermark survive. Causal IMU boundary/future samples remain buffered.

No factor is added for the reset observation. Subsequent ordinary factors use
the unchanged selected-NIS path. Optimizing the new zero-gradient reference
marks it prediction-ready. The two-frame configurable cooldown blocks only
another reset trigger, never ordinary matching or NIS. Cross-epoch constant
motion anchors are discarded.

Old CLI and R4 config retain reset-disabled behavior. The new explicit config
is `config/p6_tracking_reinitialization.conf`.

## Reference guidance and verification

Lifecycle guidance (clean-room implementation, no third-party source copied):
[hdl_localization resets PoseEstimator after relocalization](https://github.com/koide3/hdl_localization/blob/master/apps/hdl_localization_nodelet.cpp);
[Autoware initializer stops localization, aligns, publishes reset, restarts](https://raw.githubusercontent.com/autowarefoundation/autoware_universe/3d17d84d080a77e964658e6b7e820f497fc9c067/localization/pose_initializer/src/pose_initializer/pose_initializer_core.cpp).
This task's mandated preservation of v/bg/ba is not replaced by those projects'
default initialization choices.

Tests cover failed-reset atomicity, state/frame conversion, exact v/bg/ba copy,
zero gradient, block covariance, retired IDs, support/passthrough negative
controls, high stale NIS versus explicit reset, and normal post-reset admission.
Healthy-path OFF/ON parity preserves trajectory, deskew, factor/NIS decisions
and NDT/probe counts. Existing tolerances and core mathematical tests remain.

Guarded real runs use the frozen seven-input hash/manifest gate, exclusive
result directories, heavy shadows OFF, health ON, and a 20-ordinary-noncommit
stop guard. Reset observations are not miscounted as ordinary LiDAR commits.
This is an engineering safety guard, not a new estimator admission threshold.
