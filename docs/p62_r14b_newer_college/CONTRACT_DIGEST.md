# R14B authorized contract (received during this turn)

Task: P62-R14B-NEWER-COLLEGE-PRIOR-MAP-BENCHMARK. Only Newer College 2020
`01_short_experiment`; old R14 Wheel-float01 suspended. Baseline
`a611f385025788481a747b3649ea4a90f3316540`. Isolated worktree and branch as requested.
No changes to stable production or R6–R13 history; no push or automatic R15.

Prefer existing official PCD, Leica map, registered_poses and calibration.
New asset root if necessary: `/home/jian/rosbag/public_datasets/newer_college_r14b/`.
New storage ceilings: source 30GiB, map5GiB, reference/calibration1GiB,
temporary/results4GiB, total40GiB; always retain20GiB. No large download needed now.

Before any NDT: official map/reference/LiDAR frame chain and compatible epoch
timestamps must be established. T_A_B maps B to A; never infer C1=L or identity
extrinsic, fit a transform/time offset using test results, or select axes by
registration quality. If coordinate chain is unclosed: COORDINATE_CONTRACT_BLOCKED,
stop. Reference matching cap50ms; no interpolation across excessive reference gaps.

If admitted: first valid consecutive21 scans strictly after5% of full time span;
first scan only reference initialization, following20 evaluation; freeze paths,
timestamps and SHA before NDT; no alternate window. Official Leica downsampled
map preferred; if resampling needed leaf0.20m. Optional single50m static ROI about
predeclared anchor; no per-test-GT recropping or map adjustment after evaluation.

Common-Prediction Pilot: right/body constant-velocity extrapolation using only
completed CONTROL_FULL historical poses and times/safe fallback; fewer than2
history entries use last valid pose. No per-test reference or future output in TP;
all arms share TP, not3 independent closed loops.

Reuse actual R6–R13 native PCL NDT and R13 45-grid training-only coarse recovery.
resolution0.8m,step0.08,epsilon0.001,maxiter40,outlier0.55; FULL1/SPLIT1 each frame;
PROPOSED reuses SPLIT. Only natural permitted events, at most2 extra align/event,
20 total. CoverageV2, R10 events, V3 pool,0.8m/5deg admission, overlap>=0.50,
dedup0.10m/1deg,D>0.02 all fixed. No artificial events, no tuning or expanded grid.

Freeze all outputs before evaluation reference use; no test-trajectory alignment.
Report reference errors (translation/rotation RMSE,median,P95,max), sample coverage,
admission/events/distinct/eligible/supported/accepted counts. Rejections are not
zero errors. Official ICP-registered poses are Benchmark Reference, not independent
mocap. Pipeline costs separately measured, not10Hz/full-trajectory certification.
RViz minimal entry only if pilot succeeds; ROS availability not a C++ blocker.

Stop at missing asset/storage/coordinate/extrinsic/time/map coverage/prediction or
native implementation prerequisites. No alternative dataset/sequence/window;
no thresholds modified; no historical reruns, cleanup or production edits.
H1–H7 remain separate; no event means NO_NATURAL_RECOVERY_EVENT only after real run.

Required outputs are in this archive. Unexecuted stages must be explicit NOT_RUN,
not fabricated zero-error or successful empty evaluation.
