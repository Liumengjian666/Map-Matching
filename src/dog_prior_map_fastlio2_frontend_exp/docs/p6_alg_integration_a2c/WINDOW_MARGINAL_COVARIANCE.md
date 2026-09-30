# Latest window marginal covariance

`latestMarginalCovariance` assembles the actual current joint linearized H:
historical marginal prior + active IMU + LiDAR + visual factors. It does not use
optimizer damping or add a covariance-specific diagonal floor.

For D=15N and E=[0; I15], solve H X=E and extract X.bottomRows(15). This is the
latest/latest block of H^-1, not the inverse of H's latest diagonal block.
Correlations through other window nodes are retained.

Numerical coordinate equilibration uses D_s=diag(1/sqrt(H_ii)):

```
B = D_s H D_s
Y = solve(B, D_s E)
X = D_s Y
```

This exactly changes coordinates, not information. Tight IMU factors and weak
priors otherwise have disparate scales. LLT failure, nonpositive diagonal,
scaled Cholesky pivot <=1e-12, nonfinite solve or normalized backward error
>1e-10 returns `WINDOW_MARGINAL_COVARIANCE_UNAVAILABLE`. The backward error is
`||HX-E|| / (||H|| ||X|| + ||E||)`.

P15 is symmetrized and checked finite/PSD (negative eigenvalues below -1e-10
are rejected). Numerical tolerances do not inject information or repair an
unobservable H. Existing historical Schur-prior approximation/jitter remains an
upstream property; this interface adds no new jitter.

## Physical pose chart

Window retraction: R_new=R Exp(dtheta_body), p_new=p+dp_map.

Nonlocal chart: R_new=Exp(dphi_map) R, p_new=p+dp_map.

Thus dphi_map=R dtheta_body and:

```
G = [ R  0  0  0  0
      0  I  0  0  0 ]
P_map6 = G P15 G^T
```

P_map6 order is `[map-left rotation,map position]`, unlike LiDAR residual noise
order `[map position,right/body rotation]`. All rotation/position cross blocks
are transformed, not discarded.

Tests: two-node genuine joint H against an independent full inverse (test only),
nonidentity rotation with cross covariance, central SO3 chart FD and a singular
unanchored window. Release/Debug PASS. This is local linearized covariance, not
proof of globally calibrated posterior uncertainty or an NDT-basin probability.
