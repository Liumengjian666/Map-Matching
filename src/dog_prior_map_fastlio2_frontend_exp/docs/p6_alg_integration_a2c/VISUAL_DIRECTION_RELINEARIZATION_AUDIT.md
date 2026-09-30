# Visual admission direction and stale-A closure

Risk history now stores transaction, stamp, frozen map_T_lidar, routed LocalRisk,
NDT convergence, map support and actual factor commit status. It does not store
an authoritative cached Jacobian.

At visual current admission, select the newest risk not later than current
stamp, apply freshness gate and require its timestamp to remain in the active
window. Query its **current optimized WindowState**. For a committed weak
LiDAR, recompute exact A, multiply Uw and take translation rows:

```
W = A_exact(current risk state, frozen measurement, extrinsic, scale) Uw
Wp = W.topRows(3)
Qw = left singular vectors spanning nonzero singular values of Wp
```

Qw is **recomputed at visual admission, then frozen for factor lifetime**.
Admission A and source risk stamp are retained only as diagnostic evidence.
The visual factor depends on reference/current states, not a third LiDAR state.
Its residual remains `p_cur-p_ref-R_ref*z_ref_cur`, projected with frozen Qw;
both endpoint Jacobians and cross information remain in the joint window.

## Optimized-state test

An actual 0.4-radian LiDAR measurement is admitted and the window optimized.
The risk state's rotation changes visibly (>0.1 matrix norm). A_before versus
A_after differs by >0.1. The actually admitted diagnostic A equals A_after to
1e-12 and the admitted Qw projector matches the recomputed reference. This
closes the attachment's required measurable old/new **A_exact** comparison.

Important model fact, not a failed gate: for fixed measured R_m, lever arm t_IL
and length l, the translation rows are `[skew(R_m*t_IL), l I]`, independent of
the estimated risk state. Only A's rotation rows depend on the state. Therefore
Wp/Qw need not change when the state changes. The test explicitly verifies
translation-row invariance instead of manufacturing a basis change.

## Missing/rejected risk and quality

If the risk state was marginalized, admission is NOT_TRIGGERED with
`LIDAR_RISK_STATE_NOT_IN_ACTIVE_WINDOW`; a two-node bounded-window regression
passes. No stale A or hidden pose snapshot substitutes for an active state.

Normal LiDAR rejects unneeded visual fusion. Pure-rotation weak spaces without
lever-arm translation complement yield `NO_TRANSLATIONAL_COMPLEMENT`. Map
support/convergence/global correction unavailable admits only quality-valid
FULL_TRANSLATION relative motion. Rejected LiDAR is explicitly distinguished.

Prepared ref/cur/depth sensor stamps, metric reference-IMU translation and
quality metadata are preserved. Visual R=0.05² I is an empirical engineering
model, not pixel Fisher information. No KLT/PnP rerun or old projected EKF
position update occurs. A2B geometric/quality/degenerate-direction tests remain
enabled and PASS alongside the new stale-A/marginalized-risk tests.
