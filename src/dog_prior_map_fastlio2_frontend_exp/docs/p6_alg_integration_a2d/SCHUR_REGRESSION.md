# A1-R1 information conservation preserved

The Schur path still calls `linearizeMarginalizationSubgraph`, which assembles
only the existing prior and factors touching the oldest state. Retained-only
IMU/LiDAR/visual factors remain active and are not added to the new prior.

For the same linearization chart, with E the eliminated block and R retained:

```
H_all = H_prior + H_touching + H_retained
g_all = g_prior + g_touching + g_retained
H_new_prior = H_sub_RR - H_sub_RE solve(H_sub_EE,H_sub_ER)
g_new_prior = g_sub_R - H_sub_RE solve(H_sub_EE,g_sub_E)
H_after = H_new_prior + H_retained = Schur(H_all)
g_after = g_new_prior + g_retained = SchurGradient(H_all,g_all)
```

Retained-only factors have zero eliminated rows/columns. Thus they add directly
to the retained Schur result without altering the eliminated solve. They remain
relinearizable active factors. This is local information conservation, not
global nonlinear equivalence of a fixed linearization prior.

Existing A1-R1 regression remains PASS in Release/Debug, including noisy
conflicting measurements, rotations, jitter and 10,000 ID lifecycle operations.
Release report: clean/noisy H error 1.24607e-10; gradient errors about 7.853e-13.
The existing jitter test reports jitter 1e-7, H delta 1.71782e-7 and g delta
1.07995e-8. Jitter's indirect effect is not declared zero.

New block/dense tests verify assembly before and after twelve node insertion /
optimization / marginalization steps; active binary-factor counts remain bounded.
The old nine-node near-noiseless joint test also remains PASS with every
original assertion retained (rank diagnostics are explicitly opted in).
