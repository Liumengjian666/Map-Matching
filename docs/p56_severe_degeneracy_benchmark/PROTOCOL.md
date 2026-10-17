# P56 protocol

## Scope freeze

Datasets are kept distinct and processed in priority order:

1. Ground-Challenge Corridor2.
2. Ground-Challenge Corridor1 only after Corridor2 has a valid download and input contract.
3. SubT-MRS Long_Corridor as the next official source if the Ground-Challenge source is inaccessible.

No other sequence is in scope. External large files stay under `/media/jian/HIKVISION/paper rosbag/DegenerateBenchmarks/`; no bag, map, or scan archive is committed.

## Baseline freeze

Use the current P56 worktree's CONTROL implementation and preserve NDT, deskew, EKF/OOSM, Coupled, Recovery, and local-map math. Only topic/message/time/calibration/map/start-state adaptation is allowed. First run a 20-frame smoke, then one complete sequence run only if the source and map contracts are valid. No GT-derived per-frame initialization or GT-assisted map matching is allowed.

## Map and reference rules

An official map must have its source and frame contract established. If none exists, a map built from the same recording is explicitly `SELF_MAP_PIPELINE_DIAGNOSTIC`, not an independent prior-map benchmark. Absolute ATE is prohibited unless the map-to-reference transform and reference-point semantics are independently established. Preserve raw outputs and hashes before any GT is opened.

## Degeneracy rules

Use the repository's `ObservablePclNdt`/existing normalized pose chart after real NDT setup. Preserve the P47 scale convention; do not compare unnormalized mixed meter/radian Hessian eigenvalues. Record convergence, score/fitness, gradients, eigenvalues, weak subspace, innovation, coverage, runtime, and memory. Low curvature is not by itself localization failure; NDT convergence is not proof of correctness; Hessian conditioning alone does not prove multimodality.

## Download limits

At most two bounded download attempts per file, only from the dataset owner's official URLs. Check HTTP Range before resumable downloads, retain partial data on failure, and never append HTML or a non-range response to a partial archive. Verify byte count, archive integrity, and SHA-256 before extraction. No authentication bypass or third-party replacement source.

## Actual execution and stop

Corridor2 access was blocked by a Microsoft sign-in redirect. Corridor1 was not attempted because the Corridor2 priority gate was not satisfied. Long_Corridor was then attempted from its official Drive source; two connections ended early (first exit 18, resumed transfer exit 56), leaving a 3,480,741,223-byte partial of the advertised 3,888,497,930-byte file. The partial was preserved, not extracted, and no third attempt was made. No sequence met the prerequisites for the 20-frame smoke test. ROS was not started; GT read, NDT align, Hessian, and localization counts are all zero. The final classification is `DOWNLOAD_BLOCKED`, not an algorithm result.
