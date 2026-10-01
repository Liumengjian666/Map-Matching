# TX155 frozen capsule reconstruction

Status: PASS. Offline only; REAL_REPLAY_COUNT=0.

Input: `../p6_alg_integration_a3d_r1/TX155_FAILURE_CAPSULE.npz`.
SHA256: `b44fe28654437ff1cd944945cb436f62368c7ff315e9e920686836950aed58a8`.

Identity: transaction 155, stamp 1517157234720485283, enforcement 208,
attempt 1; 41 states; duration 2.017077923 s; solve-only jitter 0.
The already-recorded optimizer accepted an update in eight iterations before
this marginalization failed. Those are frozen historical facts, not a new run.

The script verifies the exact capsule SHA, keys, float64 dtype and finite entries.
It reconstructs the 615-by-615 consumed H, 615 gradient, 15-by-15 H_mm,
15-by-600 H_mr, 600-by-600 H_rr, g_m/g_r, stored correction_h/correction_b,
raw production Schur and production gradient. It checks event identity, jitter,
Schur min/max, raw asymmetry, gradient norm/max against the frozen CSV.
Tolerance for these float64 trace checks: rtol=1e-10, atol=1e-10; all PASS.

Reconstructed raw-Schur asymmetry is 0.00390625; evaluated symmetric Schur
has exactly zero asymmetry. Its three negative modes are reproduced.
Gradient norm is 3.361154041118394. The original trace min is
-97.635436582562278; the NumPy reconstruction is approximately
-97.63543657869423 (difference 3.87e-9). This is an eigensolver/BLAS-level
variation within the stated gate, not a different event or capsule.

Only indices 0..29 of consumed H are nonzero. All other 585 coordinates
are exactly disconnected zero rows/columns. High-precision calculations use
that exact structural support: one marginalized 15D state and one retained
15D state. The retained Schur has another 585 exact zero eigenvalues.
This is not numerical rank truncation and does not discard any nonzero factor.

Each original float64 entry is converted with `as_integer_ratio()`, rather
than a rounded decimal display string. mpmath works at 100 decimal digits;
the full active Schur minimum is independently repeated at 120 digits.
No production C++ Schur helper is called.

Float64 eigensolver values of tiny modes in a 4.7e18-scale matrix are not
treated as accurate absolute eigenvalues. For example, the frozen H_c trace
minimum is -209.4965384, whereas the 100-digit eigensolver on the same binary
entries finds -48.81954152. Both indicate machine-relative negativity; the
high-precision directional and Schur calculations determine its retained effect.

See `reconstruction.json` for original trace, exact schema/source identity,
library versions, oracle statistics and solver provenance.

Reproduce offline in a new, nonexistent output directory:

```bash
env OPENBLAS_NUM_THREADS=1 python3 \
  src/dog_prior_map_fastlio2_frontend_exp/scripts/p6_a3e_r1_tx155_schur_oracle.py \
  --output /tmp/p6_a3e_r1_reproduction
```

The output directory must not already exist, preventing accidental replacement
of frozen reports. The thread setting makes this offline calculation reproducible;
this is not a CPU benchmark.
