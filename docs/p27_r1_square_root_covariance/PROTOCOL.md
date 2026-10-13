# P27-R1 square-root covariance integration

START_SHA: c86d38a0d96921e5ff1e835442ac0424a69bf536.

One primary 19 s Corridor01 prefix; no GT or full replay. Physical P26 input,
measurement, CI=0.5, NIS, IMU F/Q, NDT DPD, velocity blend=0.6 and fixed-lag
scheduling remain unchanged. New default-off flag:
`filter/experimental_square_root_covariance_enable`.

Authoritative storage is lower triangular S with positive diagonal. In root mode
legacy P fields are explicitly cleared and never used as covariance: callers use
`covarianceView()` when a dense view is required. History copies/restores S and its
storage tag. Restoring a mismatched tag is an error, not a dense refactorization.

Prediction: QR of `[F S,L_Q]^T`, S_new=R^T with sign-normalized diagonal. L_Q
exactly factors the existing diagonal Q including zeros. Causal partial endpoint
variation noise is factored as the same rank-one positive terms, no jitter.

SKF: selected C=(H S)(H S)^T, gain K=S(H S)^T C^-1. The unchanged CI update
uses QR of `[(I-KH)S,K L_CI]^T`. Reference uncertainty is derived from J S_ref.
NDT: row scaling S_new=D S with the same 0.85, 1e-4 and 1e-5 floors as P26.
No eigenvalue clipping, parameter tuning or unconditional high precision.

Focused tests precede the single replay: real P26 before/after fixture, normal
prediction/Joseph/DPD equivalence, zero Q, partial noise and factor history replay.
Per-step dense diagnostics are forbidden in the performance run. Lightweight
factor finite/triangular/positive-pivot checks cover writes, history and queries.
Five sparse checkpoint factors support offline 80-digit PSD/Gram checks.

Expected prefix: 188 NDT, 187 due NDT APPLIED, 3798 high-rate states, 92 eligible
visual measurements. Final not-yet-due transaction is distinguished from loss.
PSD and covariance scale are separate decisions. Report the pre-existing jump
and any new jumps; no GT accuracy conclusion is authorized.
