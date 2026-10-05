# H1: frozen-oracle weak-subspace concentration

This is a geometric test in the predeclared nine-frame, 22-major-basin Floor01 oracle cohort. DUAL-U remains within-basin observability plus competing-basin ambiguity; weak-subspace search is only a candidate discovery mechanism. No discovery-efficiency inference is made here.

## Inputs and parity gate

Use exactly the P9-R1 major IDs in `oracle_recovery.csv`. Use original cluster representative matrices, not R1A refined/closed terminals. Preserve 616/P03 and 3796/P06, including their iteration-limit provenance. No redefinition, reclustering, optimization, map, GT, or trajectory input is permitted.

The math-only Eigen helper preserves the original float pose carrier and `mapChartDisplacement()` operations: additive map translation divided by 0.8 m, and left/spatial rotation. The displacement is not a body SE(3) logarithm. Archived U_obs eigenvectors are columns in ascending curvature order; they are not recomputed or sorted again.

Before statistics, require all 22 per-basin projection fractions to agree with `boundary_diagnostic.csv::original_weak_projection` within **1e-6**. Also require all nine archived frame means within 1e-6, overall mean/median to round to 0.8042/0.9238, and 13 fractions >=0.8. A failed parity gate prevents all statistical execution.

## Statistics fixed before execution

For nonzero dimensionless displacement delta and orthonormal W, rho = ||W^T delta||² / ||delta||². First average rho over basins within each frame, then average the nine frames equally. Twenty-two basins are never treated as IID samples.

Primary W2 = [q1,q2]; dimension ablations W1 = [q1], W3 = [q1,q2,q3]; strongest control S2 = [q5,q6]. Enumerate all 15 eigenvector pairs. Rank is 1 + number of statistics greater by more than 1e-12 (ties share the best rank).

Haar null: independent standard-normal 6×k matrices, thin QR with positive R diagonal; independent draws for each frame and replicate. Aggregate the same real basin directions within frames and then across frames. All computations use float64. RNG is NumPy PCG64 with 10,000 replicates and frame order 368,616,2226,2350,2722,2846,3341,3796,3962. Seeds: k=2 **20261006**, k=1 **20261007**, k=3 **20261008**. Monte-Carlo one-sided p = (1 + count(null >= observed)) / 10001. The analytic k/6 baseline (k=2 Beta(1,2)) is sanity only, not a substitute p-value.

Paired exact test: enumerate all 512 frame sign patterns for the mean weak2-minus-strong2 difference. One-sided p = count(permuted >= observed, allowing 1e-14 summation tolerance) / 512; no Monte-Carlo pseudocount.

Bootstrap: PCG64 seed **20261009**, 10,000 draws of nine frame indices with replacement. Percentile 2.5/97.5 intervals using NumPy linear interpolation. Leave-one-frame-out averages the remaining eight frames without changing basin definitions. Excess(k) = T(Wk) - k/6; larger k alone is not an advantage claim.

## Decision contract

H1 PASS requires: historical parity; Haar2 p<0.01; weak2>strong2; exact paired p<0.05; bootstrap difference lower bound>0; >=7/9 frames weak2>strong2; overall weak2 pair rank<=3. If the first six hold but pair specificity fails, return WEAK_DIRECTION_ADVANTAGE_BUT_NOT_EIGENPAIR_SPECIFIC. Otherwise H1 is not supported. A parity failure takes priority over every statistic.

Claims are conditional on this fixed, failure-enriched oracle cohort. Frames are the declared statistical units; exchangeability/generalization outside this cohort is not established. Haar p is a conditional orientation-randomization result. Bootstrap intervals are descriptive frame-resampling uncertainty, not evidence about all NDT scenes. Neither significant concentration nor these null tests establish basin-discovery efficiency, stationary-attractor validity, posterior probability, or filter covariance.
