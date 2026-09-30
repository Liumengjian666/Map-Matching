# Dense waste closure

Removed `H += Zero(D,D)` in the IMU factor loop. It contributed no information,
gradient or cost. The dense-reference regression explicitly verifies zero-add
invariance and independent H/g/cost parity with the block assembler.

Runtime optimizer rank eigensolve is now OFF by default
(`debug_rank_diagnostic=false`). Rank is reported as -1, not invented as zero
rank. Explicit DEBUG_DIAGNOSTIC performs the unchanged rank analysis and records
`rank_diagnostic_ms`; rank never participates in step acceptance or convergence.
The old nine-node test retains its rank assertion by enabling this diagnostic.
No old assertion or threshold was removed.

Active assembly and trial-cost evaluation use 15x15 blocks; the trial objective
has not changed. Sparse SimplicialLDLT is primary. Dense reference and explicit
`SPARSE_SOLVER_FALLBACK_DENSE` remain available. The LM diagonal, clipping,
acceptance, transaction rollback and optimization revision contract are unchanged.
The solver label is reset when a new optimization begins, including legal
convergence without a step.

Remaining costs are recorded rather than hidden: the fixed-chart prior remains
dense, marginal covariance still uses independent dense H X=E, and the prior
chart rotation derivative still uses three FD columns per node. No full H^-1
is formed. No new large dependency or PCL optimizer was introduced.

Summary/runtime timing fields: `linearization_ms` (main optimization iterations),
`solve_ms`, `marginal_covariance_ms` (latest covariance request),
`rank_diagnostic_ms`. These are not a complete WCET profile; initialization,
trial-cost assembly, Schur and allocation are also part of total event time.
