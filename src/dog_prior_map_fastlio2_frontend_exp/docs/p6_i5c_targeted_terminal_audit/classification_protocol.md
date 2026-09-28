# P6-I5C classification protocol (pre-formal-run freeze)

This protocol is committed before formal NDT calls. It does not change the
P6-I4 operational acceptance rule and does not make a global basin-equivalence
claim. The 0.20 m / 2 degree values are used only as the predeclared terminal
separation reference for this selected endpoint pair.

## Replay gate

An endpoint replay passes only when the frozen input gates pass and all three
conditions hold against its saved P6-I4 terminal:

- translation error `<= 0.002 m`;
- SO(3) angle error `<= 0.02 deg`;
- `abs(new fixedPclScore - saved objective) <= max(1e-3, 1e-4*abs(saved objective))`.

Any failing endpoint makes its frame `REPLAY_INVALID`; do not refine or perturb
that frame. Smoke replay failure stops the formal run entirely.

## Separation semantics

For the inside/outside terminal pair, report translation and rotation
separately. The pair is considered merged under this diagnostic reference only
if **both** `Delta_t <= 0.20 m` **and** `Delta_R <= 2 deg`; otherwise it is
reported as separated in at least one component. This pairwise convention is
not a transitive equivalence relation and does not redefine a global optimizer
mode.

## Fixed-objective stationarity compatibility

The fixed objective is the P5-I1/P6-I4 `fixedPclScore`, i.e. PCL 1.10
`computeDerivatives()`'s returned score on the exact preprocessed source cloud
and fixed target. `getFitnessScore()` is recorded independently and is not
substituted for that objective. PCL's parameter order is `[x,y,z,ax,ay,az]`,
with `eulerAngles(0,1,2)` and the same `Tx * Rx * Ry * Rz` reconstruction as
PCL. Its source says the Newton update negates the solve direction because it
maximizes this score; the experiment still verifies analytic/finite-difference
direction agreement rather than assuming this sign alone is sufficient.

At each refined-160 terminal, stationarity compatibility requires:

1. finite, repeatable fixed-pose score (three evaluations with range
   `<= max(1e-8, 1e-10*max(1,abs(score)))`);
2. pose-to-PCL-vector round trip agrees within `1e-5 m` and `1e-4 deg`;
3. analytic gradient agrees in direction (`cosine >= 0.99`) and in the
   perturbation-scaled relative norm (`<= 0.10`) with central-difference
   gradients at both the requested and half requested steps; the two FD
   estimates also agree to the same relative-norm bound;
4. the scaled gradient norm, using scales `[5 mm,5 mm,5 mm,0.1 deg,0.1 deg,
   0.1 deg]`, is `<= 1e-5 * max(1,abs(score))`;
5. the symmetric analytic Hessian is finite and its largest eigenvalue is no
   greater than `1e-8 * max(1, ||H||_2)`, consistent with a local maximum of
   the maximized score (not proof of a strict minimum/maximum in the full
   nonsmooth objective);
6. neither sign of any prescribed fixed-pose `+/-5 mm` translation or
   `+/-0.1 deg` rotation probe has a score increase exceeding the repeated
   score tolerance.

All three center-score evaluations and all 12 prescribed neighbor evaluations
are required and must be finite. If any repeated center score, neighbor score,
or resulting score difference is non-finite, the stationarity/local-neighborhood
check is INDETERMINATE; a non-finite sample must never be skipped as if it were
a non-positive increase. The report records finite/non-finite counts and exact
affected locations (center repeat index or signed probe coordinate, for example
`center_score_repeat_1` or `angle_z-`).

The Hessian scale in gate 5 is the spectral norm of the symmetrized Hessian,
||0.5*(H+H^T)||_2 = max_i |lambda_i|. The predeclared 1e-8 factor is
unchanged.

An analytic/FD disagreement, invalid coordinate round trip, nonrepeatable
score, non-finite derivative, or unresolved objective sign makes stationarity
`INDETERMINATE`, never a pass. These deliberately conservative tolerances are
diagnostic criteria frozen for this selected experiment, not claims about
PCL's general convergence guarantee.

## Local perturbation return

The 12 deterministic signed perturbations are exactly the task-specified
map-frame left rotations and additive translations. Each return uses the
predeclared `0.02 m / 0.2 deg` pairwise reference and additionally requires
`hasConverged()==true`; a geometrically close result reported as
non-converged is not counted as a repeatable return. Strong local
repeatability for this protocol requires all 12 perturbations of each endpoint
to return to their own refined terminal and zero of those 12 to return to the
other terminal. Counts are always reported even when this criterion fails; no
perturbation is dropped.

## Case labels

- `REPLAY_INVALID`: either endpoint fails replay/input parity. No further
  inference is made for that case.
- `REFINED_COLLAPSE`: both valid refined-160 endpoints are within both
  reference tolerances. This does not prove a single mathematical minimum.
- `REFINED_DISTINCT_NOT_STATIONARY`: refined endpoints remain separated in at
  least one component, and the fixed objective/derivative checks are
  determinate, but a required stationarity or local-repeatability criterion
  fails (including non-converged refined terminal).
- `REFINED_DISTINCT_STATIONARITY_COMPATIBLE`: endpoints remain separated;
  both refined terminals pass the frozen objective/derivative/Hessian gates;
  and both 12/12 perturbation sets return to their own side with no cross-side
  returns.
- `INDETERMINATE`: evidence needed to distinguish the above is invalid,
  inconsistent, or numerically unresolved. In particular, any analytic/FD
  direction disagreement or unresolved score sign is indeterminate.

Even `REFINED_DISTINCT_STATIONARITY_COMPATIBLE` means only finite-precision,
selected-case, local-neighborhood stationarity-compatible evidence. It does
not prove distinct mathematical local minima, random-population behavior,
cross-scene validity, or an independent nonlocal reliability semantics for
`m_B^op`.
