# NDT Self-Try Corner Tuning - 2026-08-26

Goal: continue autonomous attempts to remove the remaining corner mismatches after global source-point increase proved unsafe.

Baseline retained:
- `ndt_source_voxel_size: 0.25`, `ndt_target_voxel_size: 0.15`, `ndt_max_source_points: 1400`, `ndt_resolution: 0.8`.
- Baseline run: `/home/jian/rosbag/loop3/ndt_finer_map_source_loop3_20260826_044853`.
- Baseline full-bag vs FASTLIO2Location: mean 0.417 m, p95 1.535 m, max 2.694 m.

Experiments tried and rejected:
- `ndt_source_voxel020_points1400_loop3_20260826_223355`: finer source voxel 0.20 m with 1400 points. Worse full-bag max 4.855 m.
- `ndt_source_voxel030_points1400_loop3_20260826_224440`: coarser source voxel 0.30 m with 1400 points. Worse full-bag max 4.889 m.
- `ndt_ext_guess_blend02_loop3_20260826_225540`: enable weak external initial guess blend 0.2 from the internal high-rate odom topic. Catastrophic wrong attraction: mean 44.156 m, max 113.893 m. Do not use this coupling in the split-node pipeline.
- `ndt_res085_source1800_loop3_20260826_230612`: 1800 source points plus slightly coarser NDT resolution 0.85 m. Worse: mean 0.717 m, p95 3.028 m, max 14.420 m.

Conclusion:
- Source density/voxel changes are not monotonic. They can improve one corner but strengthen wrong attractors elsewhere.
- Feeding the split EKF high-rate output back as NDT initial guess is unstable in the current architecture and can create positive feedback.
- Keep current default parameters unchanged.

Next candidate worth coding, not just tuning:
- Add local multi-hypothesis candidate verification inside the NDT node. Around the motion-propagated initial guess, evaluate a small set of nearby seeds and choose using a combined cost: NDT fitness, nearest-map residual, and distance from the motion prior. This should be bounded to problem windows or ambiguity triggers to avoid Orin CPU overload.
