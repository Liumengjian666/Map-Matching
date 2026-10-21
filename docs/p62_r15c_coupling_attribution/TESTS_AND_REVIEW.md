# Tests and execution checks

- Configuration: CMake 3.16.3; GCC 9.4.0; PCL 1.10.0.
- Build: `cmake --build /tmp/p62_r15c_coupling_build_01 -j2` — both `run_attribution` and `test_gradient_only` built successfully.
- Unit test: `/tmp/p62_r15c_coupling_build_01/test_gradient_only` — `R15C_GRADIENT_ONLY_TESTS_PASS lambda=0.01 zero_Hsw=pass decomposition=pass trust=pass nonfinite_guard=pass`.
- Upstream frozen model output check: `R15B_FROZEN_MODEL_OUTPUTS_PASS frame_count=25 candidate_outputs=4 gt_pose_values_read=0 align=0`.
- Final model-side replay: `R15C_ATTRIBUTION_COMPLETE frames=25 candidate_evaluations=200 new_NDT_aligns=0 EKF=0 gt_candidate_reads=0 wall_ms=1172.184309 peak_rss_KiB=70696`.
- Post-freeze reference check: `R15C_REFERENCE_EVALUATION_COMPLETE frames=20 rows=160 reference=MAP_ANCHORED_BENCHMARK_REFERENCE`.
- R15C model outputs were re-hashed against their seal after reference evaluation and result summarization. `checksums.sha256` verifies every archived file except itself.

The earlier six-significant-digit serialization probe is retained separately under `run_00_precision_probe/`; it is not used for reported values. The final `run_01/` artifacts use 17-digit CSV serialization. No NDT align, EKF, or GT-driven candidate operation occurred.
