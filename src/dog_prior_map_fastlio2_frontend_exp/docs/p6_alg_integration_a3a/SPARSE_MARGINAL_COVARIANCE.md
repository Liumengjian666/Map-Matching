# Sparse undamped latest-state marginal

`FixedLagWindow::latestMarginalCovariance()` now calls `blockLinearizedSystem()` and `solveLatestMarginalColumnsSparse()`. It does not call the dense reference, form a dense joint Hessian, invert the joint Hessian, or fall back to IKFoM covariance.

For D=15N, the information matrix is the same current Gauss–Newton joint information used by the optimizer, **without LM damping**. Existing fixed-linearization Schur prior and active factors are included exactly once, using the existing block assembler. This is a local linearized covariance, not an exact nonlinear posterior.

Let E select the latest 15 state coordinates. With s_i=1/sqrt(H_ii), S=diag(s), the implementation computes sparse B=S H S, solves B Y=S E, and recovers X=S Y. Only D×15 RHS/solution storage is dense. P15=sym(X.bottomRows(15)).

The sparse LLT uses NaturalOrdering, matching the original dense Cholesky pivot ordering. This matters: the existing balanced-pivot reliability gate min(L_ii²)>1e-12 is ordering-dependent. No epsilon, jitter, damping, or diagonal information floor is introduced.

The normalized backward error is `||H X-E||F / (||H||F ||X||F + ||E||F)` and must be at most 1e-10. A nonpositive diagonal, failed SPD factorization, unreliable pivot, nonfinite result, or failed residual/PSD check returns `WINDOW_MARGINAL_COVARIANCE_UNAVAILABLE`. Previous valid output is reset before attempting a new solve.

The existing coordinate map is retained: the state chart is `(right-body rotation, additive map position, map velocity, gyro bias, accel bias)`. For the IMU pose's map-product chart, J6 has R in its rotation block and I in its position block. P_map6=J6 P15 J6ᵀ, including all rotation/position cross terms. This is not a full left-SE(3) translation retraction and is not a new extrinsic covariance model.

`latestMarginalCovarianceDenseReferenceForTest()` independently uses old dense linearization/balancing/LLT. It is only invoked by tests/debug callers; producer runtime asserts its request counter remains zero. Main optimization still uses A2D block assembly + sparse LDLT. Schur marginalization is not rewritten.
