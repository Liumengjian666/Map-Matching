# P10-R6: budgeted weak-direction coupled refinement

START 392434a5e3945cc711abe183bfb4e4baad9d1891. Floor01 remains development data.
This contract is frozen before any real R6 replay. R1-R5 archives, native PCL
objective, filter/noise, initialization, source, map, extrinsic and deskew remain
unchanged. New opt-in local modes do not execute the old nonlocal candidate path.

## Version 0 algorithm and fixed guards

Same cheap event: nominal/prediction translation >0.12m OR rotation >3deg.
Only effective nominal + triggered + valid current R5 anchor runs the model.
Untriggered or missing anchor: zero extra jet/value/align. R5 advance/settle are
reused: map_T_lidar, corrected-to-predicted causal interval, 2s lifetime, gap
<=.25s, ordinary stable post-update seed, no valid-anchor reset. Real local
feedback consumes/invalidate anchor; next stable frame may seed a new one.
Shadow does not invalidate on a merely recommended correction. No pending here.

Existing nominalPullback/jointPullback/pullNativeJet/strongStep/poseAtEta are
exposed via thin wrappers; old bodies are not rewritten. E=-raw_PCL_score/N.
Both full J^T H_native J and sum(g_native[a] K_a), including mixed terms, remain.
Nominal jet provides the real nominal score without another value call.
Original positive nominal local curvature guard and 1D/2D eigenvalue-ratio split
are retained. W/S are fixed to this frame's nominal model.

    a = chart(anchor_prediction, nominal)
    a_t = (p_anchor-p_nominal)/.8
    a_r = Log(R_anchor*inverse(R_nominal))
    u_anchor = W^T a
    rho = max(mean_positive_weak_eigenvalue,1e-4)

    Hv_eff = Hvv                      # SPD/condition/residual checked
    C = solve(Hv_eff,Hvu)
    b = solve(Hv_eff,gv)
    A = Huu - Huv*C + rho I
    rhs = -gu + Huv*b + rho*u_anchor
    delta_u = solve(A + damping I,rhs)

No explicit inverse. Symmetric eig/LLT, condition <=1e8, relative solve residual
<=1e-6. Schur damping floor=1e-4*max(1,max_abs_diagonal); log actual damping.
Weak step scalar-clipped to <=.15m translation and <=2deg rotation.

Real full weak score must pass BOTH:

    E_weak <= E_nom + .05*max(1,abs(E_nom))
    F_weak < F_nom - 1e-8*max(1,abs(F_nom))
    F = E + rho/2 * ||W^T eta-u_anchor||^2

Also positive finite score, rigid finite pose and physical bounds. The 1e-8
margin is a numerical guard, not a demonstrated localization improvement.
If full weak fails, try exactly one HALF WEAK step/value. If neither passes,
retain nominal. Both weak-only and coupled use the same chosen weak point.

Coupled only: evaluate second true jointJet at this chosen NONZERO weak point
with original nominal base and fixed Q=[W,S]. This exposes displaced Huv/gv,
not a claim that the nominal eigenspace has nonzero cross curvature.
Require its native score/value gap <=1e-9*max(1,abs(weak_score)). Mismatch or
invalid jet keeps the legal weak result, does not create a hidden extra call.

One existing strongStep(include_gradient=true,delta_u=0) at the displaced point;
its original damping/solve guards and .10 coordinate-norm cap are reused.
Keep u fixed: scale only S*delta_v along its convex feasible segment to keep
FINAL total displacement <=.15m/2deg. Evaluate actual final score once. Select
strong correction only if it improves the chosen weak energy by the same
numerical margin AND passes the same F/NDT quality/pose guards. Otherwise retain
the legal weak correction. No final half step based on an obsolete full-point
jet. Strong cap is not .10m. Record nominal/displaced cross norm and gradient.

Hard maximum per frame: two actual native jets, three actual value-only calls,
zero extra PCL align. Normally only one full nominal NDT. No Newton20; no copied
source/target/map, no new filter. Candidate tag LOCAL_REGULARIZED_REFINEMENT,
NOT a native six-DOF PCL NDT SUCCESS/convergence claim.

## Causal experiments and evaluation

Ordered full4127 frames: coupled_weak_shadow, weak_only_feedback,
coupled_weak_feedback. Freeze rules/code/binary/inputs before running, all output
hashes plus engineering receipts before loading GT. Reuse hash-verified R3
CONTROL. No new CONTROL/source replay, raw extraction, Oracle/B12/visual/Corridor.
Actual feedback uses lidarMeasurementToImu + unchanged applyPoseMeasurement;
the next source/nominal prediction uses the actually updated filter state.
Full processing time includes streamed point I/O, prediction/deskew, nominal,
local work, filter update and logs except its own cost row. Also wall, CPU, RSS.

Corrected executed IMU trajectory is primary; raw measurement separate. Same
fixed archived GT/map alignment, no per-frame fit or extrapolation; all4127
ledgers retained. Large jumps remain >.5m OR >10deg. Goal >=1% translation RMSE
improvement, stable P95/rotation, mean<=100ms/P95<=150ms; report actual failures.
Compare coupled vs weak-only, including within-frame weak/coupled differences
to distinguish algorithmic effect from trajectories diverging causally.

At most ONE reasoned targeted modification; preserve first version. No GT grid,
new search budget, changed event gate or automatic R7. Stop after delivery.
