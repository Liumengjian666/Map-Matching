# Projected covariance freeze tests

The snapshot stores `R_s = B^T R B` after symmetrization and positive-definite
validation. Candidate linearization returns this stored matrix verbatim; it
does not call the basis callback or reconstruct `B^T R B` from a candidate
state.

The synthetic state-dependent-basis test verifies:

1. the projection snapshot calls the callback exactly once;
2. repeated candidate evaluations leave the callback count unchanged;
3. the returned candidate covariance equals the snapshot covariance exactly;
4. the snapshot basis and covariance remain unchanged after candidate
   evaluation;
5. a later outer snapshot may have a different projector and its own frozen
   finite-difference model is consistent.

Release and Debug targeted CTest both pass. The full-rank path is also checked
against an independent residual equation, numerical Jacobian, covariance,
Hessian, gradient, and cost oracle rather than comparing two APIs that share
the same implementation.
