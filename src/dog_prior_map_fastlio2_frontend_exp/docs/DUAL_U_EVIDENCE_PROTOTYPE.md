# Dual-U candidate-evidence prototype: real-data falsification

Status: `MECHANISM_VALID_BUT_REAL_WORLD_EVIDENCE_MISSING`.

This is an offline candidate-scoring experiment only. It does not modify the
production estimator, `U_obs`, filter state, NDT parameters, vision, recovery,
or the stable machine-dog workspace. The result is not a performance claim.

## Problem and root cause under test

PCL 1.10 NDT's registration objective is an optimizer score, not a calibrated
posterior over spatially separated pose basins. In the installed
`/usr/include/pcl-1.10/pcl/registration/impl/ndt.hpp`, each transformed source
return radius-searches target covariance leaves and `computeDerivatives()`
sums the per-leaf contributions. Consequently, adding a duplicate local leaf
adds score mass (`log(2)` in the equivalent log-sum example), even though the
physical return and candidate pose have not changed. PCL itself documents the
returned transform likelihood as only relatively comparable within a scan;
its normalization constants need modification for global accuracy.

That mechanism is real, but the scientific question is whether it causes raw
PCL ranking to select the wrong spatial basin on real prior-map data, and
whether a normalized evidence rule fixes that error without rejecting correct
matches. Synthetic duplication alone cannot answer this.

## Mature base and proposed rule

Candidate generation is the frozen P5-I1 multi-start NDT schedule: same
scan-end source cloud, fixed prior map, prediction-centered frozen starts,
and one `align()` per start. The same converged terminal set is scored by all
methods. Target resolution is set to `0.8 m` before `setInputTarget()`; the
actual PCL target grid was asserted to be `0.8 m` on all axes. NDT settings
remain resolution `0.8 m`, step `0.08`, epsilon `1e-3`, and 40 iterations.

The recent probabilistic comparator is a fixed-candidate adaptation of Park
and Chung (JFR 2024): per-return Gaussian/uniform mixture likelihood, product
over returns, and their `alpha=0.001` posterior power. The paper's `alpha` is
the posterior exponent, not the outlier-mixture weight. This is explicitly
not an exact reproduction: the paper does not publish a 3-D PCL candidate-set
implementation or all mixture constants. The adaptation uses PCL's default
outlier ratio `0.35` and the same scan-range-shell uniform support for every
candidate. Equal prior is assigned to each retained basin.

The tested rule replaces the best-neighbor inlier density by a normalized
mixture over the same PCL radius-search neighborhood:

```text
p_i(x) = (1-rho) * (1/K_i) * sum_{j in N_i(x)} N(T_x z_i; mu_j, Sigma_j)
         + rho / V
ell(x) = sum_i log p_i(x)
```

If `K_i=0`, only the uniform component is used. Log-sum-exp is used
numerically. Dividing by `K_i` makes duplicating an identical neighbor leave
that return's local mixture unchanged. This is a support-normalized local
association model, not globally normalized map evidence and not a proof that
the candidate list is complete.

PCL raw objective ranking and raw top-two relative margin are also reported.
All scoring methods use the identical fixed source returns, map target,
terminal candidates, and NDT alignment budget. The scorer does not run extra
alignments.

## Independent post-hoc reference and cohort limits

The 32-frame P5-I1 manifest and 8,800 prediction-centered starts were frozen
before reading GT. The result CSV was generated and SHA-256 frozen before the
separate evaluator opened GT. The evaluator reuses P5-I1's fixed first-common-
pose alignment, official GT interpolation, LiDAR-to-IMU conversion, and
correct-like limits (`<=0.50 m`, `<=5 deg`) from P5-I2. No GT value influences
candidate generation or scoring.

Basins use the already frozen P5-I1 deterministic complete-link thresholds
(`0.20 m`, `2 deg`) and support criterion (`>=5` converged starts and `>=2%`
of that frame's converged starts). This prevents treating hundreds of
seed-level fragments as hundreds of independent pose modes. For scorer
comparison, all methods rank the same supported basin representatives by the
best score within each basin.

This is a selected diagnostic cohort, not an unbiased full-sequence sample.
The map artifact is the released/normalized Floor01 H1 map, but its provenance
does not establish mapping-traversal independence from the test traversal.
Therefore this dataset cannot support a broad generalization or paper-level
effectiveness claim. There is also no independently labeled degeneracy
reference, so correct-but-degenerate retention is not evaluable.

## Results

Inputs:

- Frozen map SHA-256: `2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570`
- Runtime bag SHA-256: `860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db`
- Post-hoc result CSV SHA-256: `05129ee4e89a70b78ea63ade825b23226eb44e0eaa1b77461cbd044119f67b9f`
- 32 frames, 8,800 starts; all 8,800 converged and produced finite scores.
- Mean alignment time: `14.37 ms/start`; P95 `39.18 ms`.
- Mean combined scoring time for raw PCL plus both likelihood scorers:
  `3.43 ms/start`; P95 `4.90 ms`.
- Combined mean per start: `17.80 ms`; P95 `43.62 ms`.
- One-frame process peak RSS: `132,452 KiB` (map, PCL target grid, and tool).

| Diagnostic | Raw PCL ranking | Park-style likelihood | Normalized neighborhood mixture |
|---|---:|---:|---:|
| Correct top basin, all 19 frames with supported basins | 5 | 4 | 4 |
| Correct top when a correct supported basin existed (5 frames) | 5/5 | 4/5 | 4/5 |
| Wrong top despite correct supported basin being present | 0/5 | 1/5 | 1/5 |
| False accepts among 14 frames with no correct supported basin, at `p_max >= 0.5` | n/a | 9 | 9 |
| Correct top basins accepted at that threshold | n/a | 0/4 | 0/4 |

There were 13/32 frames with no basin meeting the frozen support criterion.
Across all converged terminals, a correct-like candidate existed in only
8/32 frames; among the 19 frames with at least one supported basin, a
correct-like supported candidate existed in 5/19. Thus most wrong results
were candidate-set misses, not raw score misordering. On the five frames
where the supported set contained a correct-like basin, raw PCL selected it
all five times; the proposed rule did not improve that result. Both
probabilistic candidate posteriors accepted nine wrong top basins when the
correct supported basin was absent and rejected every correct top basin in
this small cohort. The posterior is conditional on the finite candidate set;
with one represented basin it can assign probability one even when that basin
is wrong.

Raw top-two relative margin had descriptive AUC `0.222` for classifying
top-basin correctness over the 14 multi-basin frames where a top-two margin
exists; the Park-style and proposed posterior-confidence AUCs were both
`0.10` over the 19 scored frames. These estimates are extremely small-sample,
selected-cohort diagnostics, not inferential performance statistics.

The synthetic self-test confirms only the mechanism: duplicating one
Gaussian neighbor shifts unnormalized log-sum by `0.693147` while the
normalized mixture changes by zero. The real-data comparison does not show
that this mechanism explains raw PCL errors or that the proposed score fixes
them.

## PROBLEM / solution assessment

**PROBLEM:** A normalized neighborhood score could remove local voxel-count
dependence in PCL's optimizer objective.

**ROOT CAUSE:** PCL sums local leaf contributions for optimization; its score
is not an absolute, globally comparable candidate posterior.

**MATURE BASE:** PCL NDT and the fixed P5-I1 multi-start schedule; Park et al.'s
Gaussian/uniform NDT likelihood and posterior tempering, adapted transparently
to a discrete basin set.

**PROPOSED FIX:** Average the same local target-leaf Gaussian alternatives
before mixing with uniform clutter, then compare equal-prior supported basins.

**WHY IT SHOULD WORK:** A per-return mixture has unit total association mass
independent of the number of locally returned leaves; duplicate support does
not mechanically multiply evidence.

**CODE SCOPE:** Isolated offline prototype only:
`scripts/dual_u_evidence_prototype/dual_u_evidence_scan.cpp` and
`analyze_candidates_posthoc.py`. No production files changed.

**ABLATION:** Same 8,800 terminals ranked by raw PCL score, raw top-two margin,
Park-style probabilistic score, and the proposed normalized-neighborhood
score. GT labels were joined only after the candidate table was frozen.

**FAILURE CONDITION / RESULT:** No real-data reduction in wrong-basin ranking;
raw PCL was already `5/5` when the correct supported basin existed, while the
proposed method was `4/5`. Candidate-miss false acceptance was unchanged
(`9/14`), and correct acceptance was `0/4`. The tested fix therefore fails
its real-data value criterion.

## Decision

`MECHANISM_VALID_BUT_REAL_WORLD_EVIDENCE_MISSING`; not a
`SMALL_ALGORITHMIC_CONTRIBUTION`. The stricter condition for a small
contribution—real raw-score misranking corrected by the proposed evidence
while preserving correct-basin retention—was not met.

The observed root problem is a closed-world candidate posterior: it cannot
represent an ungenerated correct basin. A mathematically appropriate next
model would need an explicit unrepresented-basin hypothesis with a
map-conditioned predictive likelihood (or a complete candidate-generation
bound), not simply a uniform scan-clutter null. The current cohort does not
provide a calibrated predictive distribution for that hypothesis, and this
experiment has no real raw-ranking failure for the proposed rule to repair.
Adding an arbitrary null prior or threshold would therefore be an unsupported
second heuristic. No further correction or production integration is
authorized by this result.

## References

- G. Park and W. Chung, “Uncertainty-aware LiDAR-based localization for
  outdoor mobile robots,” *Journal of Field Robotics*, 2024,
  [DOI 10.1002/rob.22392](https://onlinelibrary.wiley.com/doi/10.1002/rob.22392).
- PCL `NormalDistributionsTransform` documentation, including the registration
  score API and its implementation references:
  [PCL NDT class documentation](https://pointclouds.org/documentation/classpcl_1_1_normal_distributions_transform.html).
- Actual installed implementation audited for this experiment:
  `/usr/include/pcl-1.10/pcl/registration/impl/ndt.hpp` and
  `/usr/include/pcl-1.10/pcl/registration/ndt.h`.
