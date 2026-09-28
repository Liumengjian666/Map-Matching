# P6-I5A offline audit protocol

All analyses consume only the frozen, SHA-256-verified P6-I4 CSVs in the
adjacent P6-I4 result directory. No ROS, PCL, map, bag, point cloud, or
registration call is used. P6-I4 acceptance remains frozen: convergence AND
terminal translation separation <= 0.20 m AND rotation separation <=
2 degrees, with only the existing 1e-12 degree angular roundoff allowance.

The NDT-only stage reconstructs acceptance from the terminal pose and fixed
nominal M0, resolves finite winner endpoints by transaction/ray/sign/quantized
alpha, and classifies each outside endpoint by convergence and the frozen
translation/rotation thresholds. Jump magnitudes and tolerance ratios are
reported descriptively; no jump-based basin classifier is introduced.
Unobserved exits at alpha <= 3 remain `SEARCH_CAPPED_NO_DETECTED_EXIT`, not a
proof that the mathematical margin exceeds 3.

Extra-ray logical probes are recovered by merging `EXTRA_MARGIN` and
`EXTRA_RETENTION` on transaction, ray, sign, and alpha rounded to 1e-6 using
positive half-up rounding. Direction IDs are one-based D01..D32 and map to
ray IDs 100..131. Fold AB uses odd directions for margin and even directions
for retention; Fold BA reverses those roles. Correlations are descriptive
retrospective direction-heldout analyses on the same frames, objective,
covariance, and Floor01 sequence, not independent-dataset or correctness
validation. Censored margins are reported separately, never substituted with
their search cap as observed boundaries.

For each recorded finite winning ray, the sampled acceptance sequence through
the recorded outside endpoint is checked for rejected-to-accepted re-entry.
Zero observed re-entry does not rule out transitions between unsampled alpha
values; the finite-grid/capped result remains an estimator, not a certified
continuous first-exit infimum.

Covariance statistics retain rotation and translation traces in separate
physical units. The scale identity is theoretical for full-rank ideal margins;
it is not asserted for finite-grid/capped estimators. Rotation tangent norms
are a local-coordinate diagnostic, not globally unique SO(3) distances.

No runtime call counter is claimed for this audit. The script's offline source
has no ROS/PCL/NDT import, subprocess, or registration runner; P6-I4 call
accounting is reported only as frozen historical context.

The `--posthoc` stage is permitted only after `ndt_only_audit.csv` exists and
its SHA-256 is written to `analysis_manifest.json`. It reads only frozen
`margin_with_gt_posthoc.csv`; it does not read official GT and cannot affect
acceptance, boundaries, parity groups, margin, retention, or thresholds.
