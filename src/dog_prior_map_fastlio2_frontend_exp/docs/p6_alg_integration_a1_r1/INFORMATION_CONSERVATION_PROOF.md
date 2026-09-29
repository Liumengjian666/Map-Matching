# Fixed-lag information conservation

## Variables and partition

Let the state being removed be `x_m` and all retained states be `x_r`. At one
fixed linearization point, write the normal system as

```text
H_all delta = -g_all
```

and split its sources into:

```text
H_all = H_prior + H_touch + H_keep
g_all = g_prior + g_touch + g_keep
```

`H_touch,g_touch` contain only IMU, LiDAR, and visual factors incident on
`x_m`. `H_keep,g_keep` contain original factors whose endpoints are entirely
inside `x_r`. Consequently their marginalized rows and columns are exactly
zero:

```text
H_keep = [ 0  0       ]       g_keep = [ 0      ]
         [ 0  H_keep,r]                [g_keep,r]
```

Define the consumed marginalization subgraph

```text
H_c = H_prior + H_touch = [H_mm H_mr]
                                [H_rm H_rr]
g_c = [g_m, g_r]
```

The new prior stored on retained variables is

```text
H_new = H_rr - H_rm H_mm^-1 H_mr
g_new = g_r  - H_rm H_mm^-1 g_m.
```

The original retained factors remain active. Their next linearized system is

```text
H_after = H_new + H_keep,r
g_after = g_new + g_keep,r.
```

Schur elimination of the complete graph gives

```text
Schur(H_all)
 = (H_rr + H_keep,r) - H_rm H_mm^-1 H_mr
 = H_new + H_keep,r
```

and identically for the gradient. This proves information conservation at the
chosen linearization point while retaining the ability to relinearize
`H_keep` later. It also shows why the old implementation was wrong: Schuring
`H_all` and then retaining `H_keep` produced `H_new + 2 H_keep,r` and the
corresponding duplicated gradient.

## Existing prior and repeated marginalization

The existing dense prior is consumed in full because it is already the
irreducible result of older eliminations. On the next removal, the current
dense prior and only newly touching raw factors form `H_c`; raw retained-only
factors are still excluded. The repeated regression checks this equality at
every removal, not only after one three-state example.

## Fixed prior local coordinates

The prior is fixed at retained reference states `x_ref`. Its displacement is

```text
d(x) = [Log(R_ref^T R), p-p_ref, v-v_ref, bg-bg_ref, ba-ba_ref].
```

At a later state the implementation computes the local coordinate Jacobian
`J_d = partial d(x boxplus epsilon)/partial epsilon` (central differences for
the SO(3) block) and uses the Gauss-Newton system

```text
H_local = J_d^T H_new J_d
g_local = J_d^T (H_new d + g_new).
```

This is locally consistent with the fixed prior chart. It is not a claim that
the frozen quadratic prior is globally exact for the original nonlinear graph;
second derivatives of the coordinate map and relinearization of consumed
factors are intentionally unavailable after marginalization.

## Solve-only jitter

When `H_mm` is numerically singular, the solve uses `H_mm + lambda I`. Lambda
is not inserted directly into `H_new`, but it changes both inverse products and
therefore can change the stored prior. R1 reports

```text
||H_new(lambda) - H_new(pseudoinverse, lambda=0)||_F
||g_new(lambda) - g_new(pseudoinverse, lambda=0)||_2.
```

The rank-deficient synthetic case measured `lambda=1e-7`, information delta
`1.71782e-7`, and gradient delta `1.07995e-8`.
