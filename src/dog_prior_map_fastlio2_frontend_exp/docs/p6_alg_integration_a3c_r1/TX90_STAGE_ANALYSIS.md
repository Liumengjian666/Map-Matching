# tx90 attempt-level stage analysis

Identity: transaction 90, event `LIDAR_SCAN_END`, timestamp
`1517157228164951397`. The raw trace's attempt index is 1 within enforcement
78. Window state count/span before the attempt were 41 / `2.017077923 s`.
Duration-limit trigger was true; node-limit trigger was false.

## M0–M4: incoming prior through `H_mm`

| Stage | Evidence | Assessment |
|---|---|---|
| M0 incoming stored prior | finite 615×615; `lambda_min=0`, `lambda_max=20952.1784389`, relative negative 0; gradient finite, norm 23.2248 | valid; no tolerated negative mode |
| M1 prior in current chart | finite; `lambda_min=0`, `lambda_max=20952.2225531`, relative negative 0; charted gradient finite, norm 41.3963 | congruence mapping did not lose PSD |
| M2 touching factors | 1 IMU + 1 LiDAR + 0 visual; finite; `lambda_min=-2.7247439e-7`, scale `2.0000000125e9`, relative negative `1.3624e-16`; symmetry Fro / scale `4.47e-17` | only scale-relative round-off; not materially indefinite |
| M3 consumed system | finite 615×615; numerical rank 30, nullity 585; `lambda_min=0`, `lambda_max=2.0000074438e9`; relative negative 0 | PSD to the primary numerical rank rule |
| M4 `H_mm` | finite 15×15; primary rank 15; `lambda_min=7.589490168e6`, `lambda_max=1.000015137e9`; condition proxy 131.76 | SPD and not ill-conditioned at tx90 |

The LiDAR component itself is rank 5 as intended: its minimum eigenvalue is
`-1.36e-15`, its spectral scale is `100.07922494`, and its relative negative
ratio is `1.36e-17`. The IMU component's minimum eigenvalue is
`-5.85e-7` at scale `2.0e9` (relative `2.92e-16`). There are no visual
factors. These values support round-off-scale indefiniteness in component
assembly, not artificial completion of the LiDAR weak direction.

The diagnostic initially emitted `M2_TOUCHING_FACTORS` because its first
version applied the production absolute `1e-8` symmetry gate to this
intermediate matrix. That was a trace-classification false positive: the
relative symmetry defect was `4.47e-17`. The classifier was subsequently
made stage-aware; the production replay was not repeated under the
one-replay limit.

## M5–M8: solve, Schur, and gradient

At tx90 the original LDLT gate passed without jitter:

```text
positive D = 15
negative D = 0
near-zero D = 0
min_abs_D = 7589491.108454
max_abs_D = 1000015116.178418
pivot ratio = 0.007589376386
solve_jitter = 0
```

Production solve backward errors were `1.3633e-16` for `H_mm X=H_mr` and
`3.9734e-17` for `H_mm y=g_m`. The raw Schur asymmetry Frobenius norm was
`6.3275346904e-8`.

The matrix capsule was replayed through the exact C++ Eigen expression and
the unchanged production validator. Results:

```text
raw_asymmetry_fro = 6.3275346904335216e-08
production_symmetry_fro = 1.5818836835798959e-08
production_symmetry_max_abs = 1.1175870895385742e-08
out_of_place_symmetry_max_abs = 0
production_validator_pass = false
out_of_place_validator_pass = true
```

The production validator checks first whether
`max(abs(S-S.transpose())) > 1e-8`. The in-place Eigen expression crosses that
gate by `1.1758709e-9`; it is rejected before the eigenvalue check. The
independent symmetric spectrum is nonnegative (`lambda_min=0`), so this is
not a negative-eigenvalue PSD rejection. The new gradient is finite with norm
`40.7850465` and max absolute element `37.39894098`.

Therefore the corrected stage is:

`FIRST_BAD_STAGE = M7_SYMMETRIZED_SCHUR`.
