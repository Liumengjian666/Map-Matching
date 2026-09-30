# TX83 basis relinearization comparison

The production linearizer evaluates a directional LiDAR residual `B(x0)^T r(x)` and differentiates that projected residual with the basis frozen at its linearization state. Production candidate acceptance then reevaluates the factor and calls its `basis_relinearizer(candidate)`, yielding `B(candidate)` in the nonlinear objective. The basis also projects the measurement covariance (`Bᵀ R B`), so “basis derivative omitted” here means the local model omits the state dependence induced by relinearizing B in the projected factor; this test does not isolate a residual-only derivative while holding projected covariance fixed.

The diagnostic-only paired evaluator differs in exactly this dimension: one path relinearizes `B(candidate)`; the other captures `B0=B(x0)` and holds it fixed while changing candidate poses. Neither path mutates production factors or Window state.

| Candidate λ / scale | Recomputed-B production actual reduction | Frozen-B actual reduction | candidate objective gap, production minus frozen |
|---:|---:|---:|---:|
| 1e-6 (first LM step) | -1.40669431659e-8 | +1.44852396744e-7 | +1.58919339910e-7 |
| 1e-5 | -1.89476079271e-9 | +1.46366163634e-8 | +1.65313771561e-8 |
| 1e-4 | -7.68149988062e-11 | +1.48751411189e-9 | +1.56432911069e-9 |
| 1e-3 | -1.62181379437e-12 | +1.51593404496e-10 | +1.53215218290e-10 |

Every frozen-B candidate decreases in this sweep region, matching H/g's predicted descent; production recomputes B and rejects all four. The synthetic regression uses an explicitly state-dependent basis, verifies a nonzero production/frozen objective difference, and confirms frozen-B debug evaluation leaves production H/g/cost and factors unchanged.

Interpretation is intentionally narrow: on this failed state, omission of the state-dependent basis terms from the local model is strongly supported as the mismatch mechanism. The sweep's λ=`1e-2` production decrease is strictly positive in double precision but only `7.1e-15`; because production attempted through λ=`1e-3` only, a damping-budget contribution (A) remains possible. The direction-derivative evidence nevertheless establishes a separate local-model/objective inconsistency at useful step scales (D). This is not a repaired optimizer and not a claim about every LiDAR factor or state.
