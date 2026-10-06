# R1C3A branch numerical-contract closure

This is a diagnostic-only closure of the R1C3 numerical contracts. It does
not run a new weak path, continuation step, proposal search, NDT alignment,
oracles, or production localization update. The only geometric inputs are the
already archived R1C3 root endpoints, accepted TX616 alpha=0.68125 endpoint,
and TX616 alpha=0.68625 OLD_SUPPORT/NEW_SUPPORT attempts. For TX3341, P02 and
P03 are one shared root and one transition sequence; for TX2350, P01 and P05
are one shared root.

## Predictor contract

The map-product chart and W/S basis are reused without change:

`eta = W u + S v`, with `W=q1,q2`, `S=q3,...,q6` at the frozen nominal `T0`.
For each archived node and frozen support, the FLOAT/PCL and DOUBLE continuous
frozen objective are differentiated in the joint six-dimensional chart using
central finite differences at `h = [0.004, 0.002, 0.001, 0.0005, 0.00025,
0.000125]`. The actual continuation-relevant prediction is

`p(h) = -H_vv(h)^-1 H_vu(h) u_b`.

The root step scale is `delta_alpha=0.05`; the already-archived TX616 boundary
nodes use `delta_alpha=0.005`. A predictor is numerically resolved only if a
contiguous region of at least three h values has FLOAT and DOUBLE `H_vv` SPD,
the DOUBLE predictor changes by at most 20% and its `H_vv` eigenvalues by at
most 10% between adjacent h, and both (i) FLOAT/DOUBLE predicted-step
disagreement and (ii) adjacent FLOAT predicted-step spread do not exceed that
node's archived R1C2 `EPS_DV_NUM`. The archived independent FLOAT/DOUBLE
directional FD results remain separate raw derivative diagnostics.

## Root anchor and support equivalence

The R1C3 root is associated with the nominal branch only if its pre-existing
deterministic corrector provenance is `u=0,v=0,s=A(T0)` and its pose is within
the already-frozen 0.2 m / 2 deg same-basin sanity bound. The older 0.02 m /
0.2 deg closure is reported, but is not treated as a stationarity or IFT
condition.

Only support transitions present in the R1C3 logs are evaluated: eight TX3341
ROOT rounds and both eight-round TX616 alpha=0.68625 support hypotheses. For
each transition, both frozen supports are optimized independently in DOUBLE
strong coordinates from the same archived round-start `v`, at the same frozen
`u`. The solver is the existing deterministic R1C2 `referenceMinimum` on the
fixed-support continuous DOUBLE objective; there is no support update inside
that solver. Each endpoint is then checked against its dynamic support and a
multi-h DOUBLE predictor derivative.

For each support's numerical envelope, `EPS_DV = max(d_resolution,
fine/coarse Newton-displacement difference)` from the R1C2 double-resolution
certificate, and `EPS_E = rho_E = 64 eps_double max(1,|E|)(leaf_terms+32)`.
For a pair, the respective envelopes are summed. Two unequal support
signatures are numerically equivalent only when both fixed-support solutions
are double-resolution stationary, both strong Hessians are SPD, both DOUBLE
predictors have a stable multi-h region, `||v_A*-v_B*|| <= EPS_DV_A+EPS_DV_B`,
and `|E_A*-E_B*| <= EPS_E_A+EPS_E_B`. Exact signature equality and numerical
support equivalence are reported separately; point-change fraction is
descriptive only.

Dynamic support consistency at each optimized endpoint is reported separately.
If a fixed-support stationary endpoint does not reproduce that same support
under the dynamic evaluator, its separation from another fixed-support solution
shows material dependence of the frozen branch objectives, but is not a
certificate of two distinct dynamic local minima.

At each support-pair's shared archived start pose, both frozen-support energies
and the actual dynamic energy are also recorded. This is a fixed-support
mechanism comparison, not a posterior or basin probability calculation.

## Scope limits

`dynamicValueOnly`, `frozenScore`, and the DOUBLE frozen evaluator are used only
to measure existing poses and support hypotheses. `ndt.align()` is never
called. No tolerance is altered to force a pass. Results are offline numerical
contract evidence only; they do not establish branch continuation success or
failure.
