# Authoritative square-root marginalization contract

Scope: A3E-R2; production V3 explicitly selects `SQUARE_ROOT_QR`. Historical
V1/V2 and default test windows retain `LEGACY_INFORMATION_SCHUR`. No automatic
fallback is implemented.

The prior owns `(A, b, reference_states, valid)` and represents
`||A d(x,x_ref) + b||²`. Its information/gradient caches satisfy `H=AᵀA`,
`g=Aᵀb` up to floating-point arithmetic. Initial SPD information is factored
with LLT: `H=L Lᵀ`, `A=Lᵀ`, `L b=g`. No inverse, jitter, clamp or information
addition is used. The initial H/g values are retained verbatim as compatibility
caches to preserve the pre-first-marginalization estimator path; their
square-root reconstruction is tested at machine-level accuracy.

At a nonzero current chart, `d=localDifference(x,x_ref)` and the same existing
right-SO3 chart Jacobian `C` gives `r=A d+b`, `J=A C`.
Thus `JᵀJ=CᵀH C`, `Jᵀr=Cᵀ(Hd+g)`. The chart's rotation columns use the
existing central finite-difference convention/step, not a new coordinate model.
Adding a node extends A by 15 zero columns, without adding rows or information.

Only the existing prior and factors incident to the oldest state are consumed.
Retained-only IMU/LiDAR/visual factors stay active and relinearizable. For each
raw factor covariance `R=L Lᵀ`, triangular solves produce `Jw=L⁻¹J`,
`rw=L⁻¹r`. No consumed normal matrix is formed for production elimination.
LiDAR uses one current-state frozen reliable projection and its selected
covariance, preserving rank-5 and the existing marginalization relinearization
semantics. Visual rows use the existing cross-state selected factor.

Partition the stack as `[Am Ar]`. Column-pivoted Householder QR of Am confirms
rank 15 using `eps*max(rows,cols)*maxPivot`. Applying its orthogonal Q to
`[Ar b]` and discarding the first 15 rows eliminates oldest coordinates.
Rank deficiency fails closed; there is no pseudoinverse, jitter or old-Schur
fallback.

The retained rows are compressed by another column-pivoted Householder QR,
using the same standard relative rank rule. Store `R_top Pᵀ` and
`(Qᵀb)_top`, not a re-rooted normal matrix. Rows are bounded by retained columns;
discarded near-null Jacobian row norm and constant residual energy are logged.
This is numerical row-space compression, not an empirical rank cutoff.

H/g are derived afterwards only for the unchanged optimizer, covariance API,
and diagnostics. The next marginalization consumes A/b, never these caches.
Finite rows/dimensions/rank are authoritative validity; diagnostic tiny negative
eigenvalues of AᵀA are not clamped. Existing legacy PSD thresholds are untouched.

## Constant convention and equivalence boundary

The legacy prior objective stores `dᵀH d+2gᵀd`, omitting state-independent
constants. The row objective differs by `bᵀb`; compression may drop additional
constant-only residual energy. The optimizer maintains this existing
zero-reference constant convention. These offsets do not affect an inner
iteration's candidate comparison, H/g, or the minimizing state. They are not
an accumulated absolute likelihood. Tests compare minimized row objectives
modulo the same constant at multiple retained increments, not just positions.
Nonlinear equivalence is local to the specified reference/chart linearization,
not a claim of globally exact nonlinear marginalization.

## Diagnostic isolation

Legacy Schur is evaluated on a separate copied window only when capture is ON.
Its incident LiDAR callbacks are replaced by pure callbacks returning the
production-frozen projections: copying std::function alone would not isolate
shared captured state. Shadow success/failure cannot affect the QR prior,
rank, state, lifecycle or control flow. Unavailable shadow spectrum is NaN.
Tests include shared callback counters and exact ON/OFF state/prior parity.

Production still uses normal equations in the optimizer, as explicitly scoped.
This change closes the marginalization prior chain, not a square-root optimizer.
Batch rollback across multiple successful removals followed by a later failure
is not newly implemented in this stage. Failures before full row assembly may
have an empty stack capsule; the precise reason and incident counts remain.
