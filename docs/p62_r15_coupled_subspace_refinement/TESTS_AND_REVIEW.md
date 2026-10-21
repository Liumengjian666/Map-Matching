# Tests and adversarial review

## Tests run

- CMake Release build completed for both `run_coupled` and `test_coupled`.
- `test_coupled` passed all synthetic SE(3), exact-score, derivative, geometry-conversion, association-boundary, weak/strong split, coupled stationarity, and failure-path checks. Exact summary: see `MATHEMATICAL_TESTS.md`.
- Python syntax compilation passed for all R15 helper scripts.
- `freeze_outputs.py` passed: exactly 25 required source frame identities; 50 rows each for WEAK_ONLY, COUPLED, training objective, coupling blocks, and runtime; 1200 held-out rows; candidates never marked converged or formally eligible; 0 align, 0 EKF, 0 in-run GT reads.
- The candidate-output seal was created before opening any of the 20 raw Newer College reference pose rows.
- Before/after asset snapshot matched, both archived checksum inventories passed, and the stable production worktree (1766 non-Git files in the snapshot) had an identical state/content digest.

## Review findings and disposition

The fixed-association nature of PCL's radius-truncated Gaussian objective was treated as a hard local-model boundary. The code compares each stencil's exact Gaussian-leaf identity signature to the nominal signature; all real frames rejected at this boundary and no finite-difference derivatives were reported. No threshold widening, smoothing substitution, or nearest-neighbor surrogate was introduced.

The geometry `G` is explicitly labeled a Gaussian-weighted Gauss–Newton information matrix rather than PCL's optimizer Hessian. It is scaled in dimensionless right/body coordinates and compared with the correctly transformed R6 matrix. Eleven real-frame conversions exceeded the frozen tolerance; these are retained as failures and not normalized away.

`Hss` is only solved from the symmetrized Hessian model after the raw block passes finite, symmetry, positive-definite and condition-number gates; Eigen LDLT is used without explicit inversion. Extra weak directions beyond the first two are recorded as unmodeled/frozen, not misclassified as strongly constrained. Failed `Hss` preserves the independent weak-only ablation in tests.

Two orchestration defects were caught and corrected (pre-freeze run-directory guard; post-freeze reference-inventory nesting), documented in `PROCESS_EXCEPTIONS.md`. The latter was corrected after output freeze but before any raw reference pose values were read. Separately, the prior R14B summary crossover makes reference evaluation non-blind; this is disclosed and prevents confirmatory accuracy claims.

## Remaining scientific gap

Synthetic correctness does not answer whether this exact PCL objective admits a usable fixed-association local model on real scans, nor whether a real coupled response improves geometry or reference-relative error. The present 25-frame result leaves these questions open and does not justify an algorithm-success claim.
