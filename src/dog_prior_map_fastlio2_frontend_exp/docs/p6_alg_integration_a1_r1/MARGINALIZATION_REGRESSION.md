# Marginalization regression

Executable: `p6_a1_r1_fixed_lag_regression_test`

## Test 1: three-state exact information comparison

The graph contains an explicit initial prior, an `X0-X1` IMU factor, and an
`X1-X2` IMU factor. The test linearizes the complete 45D system, directly
Schur-eliminates the first 15D block, invokes production marginalization, and
compares the result with production `new_prior + retained_X1_X2_factor`.

```text
H Frobenius difference = 1.24609e-10
gradient L2 difference = 7.85276e-13
```

The retained `X1-X2` factor count is one and its ID remains active.

## Test 2: conflicting noisy retained observation

A full-rank LiDAR observation on `X2` is offset by `(0.37,-0.19,0.11) m`, so
the retained graph has a nonzero residual and gradient. The same complete
Schur comparison passes:

```text
H Frobenius difference = 1.24609e-10
gradient L2 difference = 7.85276e-13
```

The nonzero-gradient assertion prevents a zero-residual graph from hiding the
former duplicate-weighting defect.

## Test 3: repeated operation

Twelve add-factor/optimize/marginalize cycles run with a three-node limit.
Before each removal, the test Schurs the complete system; after removal it
compares complete `H` and `g`, checks active factor/ID counts, and confirms the
validated feedback revision survives an algebraically equivalent removal.
The maximum active ID count was five.

## Test 4: nonzero rotation prior chart

An initial state has nonzero 3D rotation and a nonzero rotational prior
gradient. After an accepted optimization step moves it away from the fixed
reference, the production prior `H,g` are compared with an independently
computed central-difference coordinate Jacobian. Hessian error is below
`1e-9`; gradient error is `0` at printed precision. The test validates local
chart consistency only, not nonlinear global equivalence.

## Test 5: jitter sensitivity

A rank-deficient relative visual factor forces the oldest block onto the
solve-only jitter path. The production report records:

```text
jitter = 1e-7
H delta versus unjittered pseudoinverse = 1.71782e-7
g delta versus unjittered pseudoinverse = 1.07995e-8
```
