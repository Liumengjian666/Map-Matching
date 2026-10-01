# Fixed offline synthetic scale stress

Two deterministic cases; no dataset or parameter tuning. Seed 155 fixes the
relative-coordinate rotation, seed 208 fixes the information rotation.
Six states coordinates comprise two 3D nodes. A strong relative factor has
J=[I,-T], weighted Jacobian A, and exact factor objective H=P+A^T A.
P=diag(1e5,20,5,1e5,20,5), SPD. Two fixed strong scales: 1e6 and 1e18.
The high case has weights [1e18,1e17,1e7].

The factor-theory oracle converts the float64 entries of A/P exactly and
forms P+A^T A at 100 digits. The rounded-entry oracle instead converts the
already formed float64 normal matrix, then performs 100-digit Schur.
These are deliberately different oracles.

| Scale | Double Schur min | Scaled-double min | Rounded-H HP min | Exact-factor HP min |
|---|---:|---:|---:|---:|
| 1e6 | 32.04766316 | 32.04766316 | 32.04766316 | 32.04766316 |
| 1e18 | 50.98639892 | 83.83901615 | 79.65149311 | 32.05139894 |

At extreme scale, substantial errors affect small retained curvature.
Unscaled/scaled matrix relative errors to exact-factor HP are 0.00108133
and 0.00167686, respectively. Scaling is not consistently better.

PSD-LOSS REPRODUCTION: NOT_REPRODUCED in this independent synthetic fixture.
Both double retained minima remain positive. We do not change seeds or priors
to force a desired outcome. The real frozen TX155 capsule is the actual
negative-mode reproduction fixture; the synthetic example illustrates loss
of accurate weak curvature, not an independent proof of PSD failure.

The known integer SPD self-test has exact Schur [[3,1],[1,4]]. It tests
directional/lifted agreement, direct high-precision Schur, eigenvalue extrema
and condition extraction, exact binary float conversion and invertible scaling.
Assertions use 1e-90 for HP Schur/identities and 1e-14 for small double scaling.
Oracle self-test PASS. All calculations are offline numerical sanity checks,
not production fixes or formal performance/accuracy benchmarks.
