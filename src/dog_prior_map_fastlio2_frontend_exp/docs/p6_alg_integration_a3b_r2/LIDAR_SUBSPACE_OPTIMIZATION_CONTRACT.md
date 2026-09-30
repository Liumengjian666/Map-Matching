# LiDAR subspace optimization contract (A3B-R2)

For each optimizer outer iteration `k`, the current state `x_k` is used once to
construct an immutable `LidarIterationSnapshot`. For every active LiDAR factor
the snapshot owns the keyed identity `(observation_id, stamp_ns)`, basis `B_k`,
reliable rank, and symmetrized projected covariance

`R_s,k = B_k^T R B_k`.

The basis is checked for finite values, valid rank, orthonormal reliable
columns, and positive-definite projected covariance before it enters the
snapshot. The production local system and all candidate costs in that outer
iteration use exactly that snapshot:

`r_s(x; B_k) = B_k^T r_raw(x)`

`J_s(x; B_k) = B_k^T J_raw(x)`

`R_s,k = B_k^T R B_k`.

Thus the iteration's quadratic assembly and candidate acceptance both
evaluate the same frozen-projection surrogate

`E_k(x) = E_prior(x) + E_IMU(x) + Σ E_LiDAR(x; B_k) + E_visual(x)`.

Accepted candidate states are retained. At the next outer iteration the
reliability basis is rebuilt at the then-current states, producing `B_(k+1)`.
Retries at an unchanged state also rebuild the snapshot; deterministic
relinearizers should reproduce the same projector to numerical tolerance.
The projector difference is diagnostic only and does not alter optimizer
control flow.

This is an outer reliability-subspace relinearization with an inner
fixed-subspace Gauss-Newton/LM solve. It does not differentiate `B(x)` or the
state-dependent projection weight, and it does not claim to optimize a
continuous dynamic-basis objective inside an LM step. The historical dynamic-B
objective remains available only to diagnostics/tests; it is not used for
production candidate acceptance.

The damping schedule, step clipping, iteration limit, convergence threshold,
and candidate acceptance rule were left unchanged.
