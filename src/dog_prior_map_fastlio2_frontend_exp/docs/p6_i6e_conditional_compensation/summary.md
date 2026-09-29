# PAPER-P6-I6E — Three-Level Conditional Compensation

## Scope and implementation

Implemented in the existing `research/p6-i6d-full-algorithm` worktree; no new
worktree was created and the prior frozen workspaces/results were not edited.
The implementation keeps B0–B4 on the shared runner and adds conditional
logging/behavior to B4.

- LiDAR local observability now exposes a joint 6D eigenspace and Schur-
  decoupled rotation/translation diagnostics, with explicit geometric map-
  support status. Nonlocal terminal response directions are combined with the
  local weak subspace in the same normalized map-spatial pose chart.
- A LiDAR update is projected onto its reliable subspace. The projection is
  mapped into the IKFoM position/right-SO(3) chart with the IMU–LiDAR lever arm.
  Directional updates bypass the legacy whole-pose step limiter: clipping a
  full pose vector before projection let discarded weak components change the
  scale of the retained measurement. The following limiter reference is the
  filter pose actually produced by that update.
- Visual admission checks tracked/PnP inliers and ratio, 4×3 grid occupancy,
  hull coverage, valid-depth fraction and sensor-time age, image parallax,
  reprojection error, and innovation chi-square. It computes
  `Gv = Hvᵀ Rv⁻¹ Hv`, projects this into the LiDAR weak subspace in the same
  physical pose chart, checks effective rank/conditioning, and applies only the
  complementary position row-space. PnP rotation is not fused.
- Failed/unsupported LiDAR directions and unobservable visual directions
  remain IMU predictions with normal covariance propagation. Sustained coast
  or excessive position/rotation uncertainty requests relocalization.
- Per-scan logs now include map support, Schur spectra, joint reliable/weak
  bases, fusion state/reason, uncertainty sigmas, whether LiDAR was actually
  updated/projected, visual trigger/availability/quality/complement/update and
  rejection counters, coast duration, relocalization latch, and recovery time.
  Per-visual-event logs include quality fields, projected information/eigen
  ranks, innovation, correction, and rejection reason.

The visual anchor/current cross-covariance is not available in the existing
minimal frontend API. The factor therefore retains the conservative
decorrelation covariance bound documented by I6D; this is a prototype
approximation, not an exact correlated historical-state update.

## Deterministic scenarios and tests

Scenarios A–E pass in `p6_i6e_conditional_compensation_test`:

- A: fully supported LiDAR selects `NORMAL_LIDAR` and does not inject vision.
- B: quality-valid visual information complementary to a LiDAR weak direction
  selects `LIDAR_DEGRADED_VISION_VALID`, keeps reliable LiDAR information, and
  permits the projected visual update.
- C: quality-valid but noncomplementary visual is rejected; weak directions
  coast.
- D: quality failure is rejected.
- E: a short outage uses IMU prediction; an over-limit outage requests
  relocalization; restored reliable LiDAR returns to the normal route.

Release runner and tests:

- `p6_i6b_closed_loop`: built successfully against the pinned FAST-LIO2/IKFoM
  and PCL 1.10 dependencies. The build emits existing pedantic/unused warnings
  from the pinned external FAST-LIO2/IKFoM headers and one unused legacy
  `verifyScoreGradientConvention` function; no build errors.
- `P6_I6B_DUAL_RELIABILITY_TEST_PASS`
- `PAPER_P6_I6E_CONDITIONAL_COMPENSATION_TEST_PASS`
- Catkin CTest: 5/5 passed, including `ndt_pose_update_runtime_test` with
  projected-pose discarded-axis invariance.
- Python byte-compilation and `git diff --check`: pass.

## Real-data B4 short replays

Both runs use real sensor observations, pinned map/parameter/input hashes,
GT-free online execution, and post-run resource accounting. They validate
conditional behavior, not comparative accuracy; no GT-based tuning or RMSE
acceptance was performed.

| Dataset window | Scans | NDT calls | M0 nonconverged | U_obs valid / used | U_nonlocal probes | LiDAR update / projected | Visual trigger / available / quality / complementary / applied | Fusion modes | Wall / peak RSS |
|---|---:|---:|---:|---:|---:|---:|---:|---|---|
| Floor01, 0.504–39.938 s | 392 | 546 | 0 | 126 / 126 | 77 | 69 / 69 | 59 / 108 / 19 / 0 / 0 | `RELOCALIZATION_REQUIRED` 392 | 28.22 s / 101,224 KiB |
| Corridor01, 0.043–39.981 s | 397 | 571 | 0 | 126 / 126 | 87 | 54 / 54 | 42 / 88 / 70 / 25 / 25 | `RELOCALIZATION_REQUIRED` 372; `LIDAR_DEGRADED_VISION_VALID` 13; `LIDAR_DEGRADED_VISION_INVALID` 12 | 37.59 s / 56,920 KiB |

Additional observations:

- Floor01 visual event outcomes: 59 triggered observations were rejected for
  insufficient image parallax; 49 events were not triggered. Nineteen factors
  passed the image-level quality gate, but none coincided with a triggered,
  stable LiDAR-weak information subspace, so no visual correction was forced.
  LiDAR projected updates ran on 69 scans where a reliable subspace existed.
- Corridor01 visual outcomes: 25 nonzero projected translation updates;
  8 triggered factors failed valid-depth association, 8 had no complementary
  weak-direction information, and 1 failed feature-distribution quality.
  Forty-six factor events were not triggered. The 13 valid and 12 invalid
  degraded states alternate as observations arrive/fail.
- No `NORMAL_LIDAR` or standalone `IMU_COASTING` label occurred in these two
  windows. The deterministic A/E tests exercise those branches. On both real
  windows the initial rotation sigma was about 1 rad, above the configured
  30-degree limit, so the relocalization latch engaged at startup. Floor01
  never recovered during this window; Corridor01 first recovered near 2.97 s,
  then re-entered `RELOCALIZATION_REQUIRED` near 5.49 s.
- The real route continues supported LiDAR projected updates while the global
  state is `RELOCALIZATION_REQUIRED`; unsupported/weak directions coast. This
  is a request for recovery, not a blanket shutdown of reliable measurements.
- The runs also exposed a significant limitation: U_obs returned
  `NO_VALID_GEOMETRIC_CORRESPONDENCES` on 266/392 Floor01 scans and 271/397
  Corridor01 scans. Maximum position sigma reached 352.4 m and 268.1 m,
  respectively. The router therefore often could not certify a LiDAR reliable
  subspace and correctly requested relocalization rather than claiming a
  successful localization result.

### Input provenance

- Floor01 input manifest SHA-256:
  `75e71ba365b9c2d0b247f4bf3944008994d398f8f5d87fcb2d49e9e7ffd2a09e`
- Floor01 map SHA-256:
  `2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570`
- Floor01 generated visual-factor CSV SHA-256:
  `d2db72284b93bfa6d5fb6a9d813f20d45c3fda3e4f38710dd367fcfb68b0d66c`
- Corridor01 input manifest SHA-256:
  `6d722ec6946570cc09d984c1a8ac7ebaaff799f012f9386ae169e3d47747a043`
- Corridor01 map SHA-256:
  `103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f`
- Corridor01 I6D parameter SHA-256:
  `7e42752ff8b84eae2b2da8d7d9fe179db0bb8f364a923e12236d2e91336e357d`
- Corridor01 generated visual-factor CSV SHA-256:
  `f192b2d77ac427c62357ac6592acebabf678c60fbe762dc4cbe48422de8c5a63`
- Both provenance files say `gt_used_online=NO` and record the compatibility
  preload used to work around the host’s MVS `libusb` symbol collision.

## Result and limitations

The three-level route and selective visual update are implemented in the actual
FULL closed loop, and deterministic branches plus both real 40-second windows
were exercised. Corridor01 demonstrated 25 accepted metric visual updates and
explicit quality/complementarity rejections. Floor01 correctly refused all
visual corrections under the trigger/quality/complementarity conjunction.

This is **not an accuracy pass**: the two windows enter or remain in
`RELOCALIZATION_REQUIRED`, with large growing position uncertainty and
frequent missing geometric correspondences. Do not describe the current runs
as localization recovery or improved RMSE. The fixed reliability thresholds
were not tuned. The `NORMAL_LIDAR` and outage/recovery state branches are
validated deterministically; the real runs did not supply an extended
NORMAL_LIDAR interval.

## Git

- Branch: `research/p6-i6d-full-algorithm`
- Base/current HEAD: `a561c310e97510dc18265ebb3057ac9d586daaee`
- I6E implementation and result artifacts are local and uncommitted; no push
  was attempted because this supplement did not specify a commit/push action.
- Existing I6D and earlier frozen result files were not modified. The
  provisional replay outputs from the intermediate directional-step-limit
  experiments were moved under `/tmp/p6-i6e/` and are not part of this report.
