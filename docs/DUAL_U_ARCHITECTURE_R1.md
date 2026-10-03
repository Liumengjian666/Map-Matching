# Dual-U Architecture R1

Date: 2026-10-03
Status: diagnostic architecture prototype; no estimator behavior changed.

This document defines two different reliability questions around a single-frame
prior-map NDT result. It does not claim a calibrated posterior, a global
uniqueness certificate, or a novel method. The current evidence supports a
careful architecture prototype only; see the validation limits below.

## Frozen carrier and scope

- Sole baseline: `research/paper-baseline-v2` at
  `70aa6859657c92404751cd354bd4292e9d30c617`.
- R1 branch: `research/dual-u-architecture-r1`, created from that SHA.
- Carrier remains scan-end IMU prediction / deskew, one current-frame NDT
  registration against the frozen prior map, then the existing IKFoM update.
- Dual-U is called after NDT and is diagnostic-only. The diagnostic interface
  has no filter/frontend handle and its outputs are not passed to the update.
- PCL 1.10 constructs `target_cells_` during `setInputTarget()`. The frozen
  baseline sets resolution first; the actual leaf read back on Floor01 is
  `(0.800000012, 0.800000012, 0.800000012) m`.

No fixed-lag/window optimization, visual input, recovery, candidate search,
NIS, adaptive covariance, or state update was added.

## Two different questions

Let `M` be the fixed prior map, `z` one deskewed current scan, and `xi` a pose
in the map frame. A converged registration terminal selects a local attraction
basin `b0` of the NDT objective.

`U_obs` is conditional on that selected basin: assuming `b0` is the basin of
interest, what is the local directional curvature of the same NDT objective
near its terminal? It describes within-basin shape and does not establish that
`b0` is the correct map location.

`U_nonlocal` concerns basin selection: among spatially separated basins
represented by a declared finite search, what terminal support is observed,
and what remains unresolved because the search may have missed another basin?
It does not replace within-basin directional curvature. A single local Hessian
cannot answer this question.

The two axes therefore distinguish, conceptually:

| Within-basin observability | Basin evidence | Interpretation |
|---|---|---|
| adequate | one represented basin, finite search exhausted | locally constrained and single among the evaluated starts; not a global proof |
| weak | one represented basin, finite search exhausted | one evaluated basin, but weak local directions |
| adequate | multiple supported represented basins | each may be locally sharp while the location is ambiguous |
| weak | multiple supported basins or incomplete search | both local and basin-level uncertainty remain |

“Adequate” is only a conceptual label here. R1 emits curvature and candidate
support evidence, but intentionally has no reliability threshold or routing
policy.

## Mixture interpretation and its limit

If the posterior mass is fully represented by basins `b=1..B`, an approximate
mixture can be written

```text
p(xi | z, M) = sum_b pi_b p(xi | b, z, M),
```

with basin means `mu_b` and conditional covariances `Sigma_b`. Then the law of
total covariance gives

```text
mu = sum_b pi_b mu_b
Sigma = sum_b pi_b Sigma_b
      + sum_b pi_b (mu_b - mu)(mu_b - mu)^T.
```

The first term is within-basin uncertainty (the semantic role assigned to
`U_obs`); the second is between-basin uncertainty (one part of `U_nonlocal`).
This identity requires normalized posterior weights and a represented set of
all relevant modes. R1 does not have calibrated basin probabilities, so this is
an interpretation, not a computed covariance.

For finite candidate coverage, include an unresolved event `u` with mass
`pi_u`:

```text
p(xi | z, M) = (1 - pi_u) sum_b pi_b p(xi | b, z, M) + pi_u p(xi | u, z, M).
```

If `p(xi | u, z, M)` or even its center is unknown, its contribution cannot be
represented by an ordinary 6x6 covariance. A finite heuristic seed set cannot
prove `pi_u = 0`. Thus R1 preserves an explicit unresolved/unrepresented risk
flag and does not fabricate a total covariance or a confidence scalar.

## U_obs mathematical contract

### Coordinates and perturbation

The analyzed pose is `T_map_lidar` at the raw NDT terminal. The product chart
is

```text
eta = [delta_t_map / L, delta_theta_map],
t(eta) = t* + L delta_t_map/L,
R(eta) = Exp([delta_theta_map]x) R*.
```

Ordering is translation first, rotation second. Translation perturbation is
additive in map axes; rotation is a left/map-spatial perturbation. The chart is
about the LiDAR origin and is not an SE(3) left-twist chart. `L` is read from
the actual PCL target-grid leaf and must equal configured NDT resolution; on
this frozen baseline `L = 0.8 m`. Rotation components are in radians. The
normalized chart coordinates are dimensionless (radians are dimensionless in
SI), avoiding a raw eigenvalue comparison between metres and radians.

The native PCL 1.10 parameter vector is
`p=[tx,ty,tz,roll,pitch,yaw]`, with `Rx(roll) Ry(pitch) Rz(yaw)`. Around the
base pose, `p=p(eta)`. Euler branch changes and ill-conditioned charts fail
closed. The implementation selects the equivalent XYZ Euler branch nearest
the base angles and rejects a local chart condition number above `1e4`.

### Objective and curvature

PCL 1.10 `computeDerivatives()` implements the analytic gradient and Hessian
of its maximized NDT score sum `S(p)` in the native Euler chart (the
Magnusson-style derivative used by the backend), not a Gauss--Newton matrix.
It is conditional on the target-neighborhood membership used for each source
point; because that membership can change with pose, this is a piecewise
objective and its returned local curvature is not a global smooth Hessian.
R1 defines the minimization surrogate

```text
L(p) = -S(p) / N_source.
```

The reported local curvature is the chart pullback

```text
g_eta = J^T g_p
H_eta = J^T H_p J + sum_i g_p[i] K_i,
J = dp/deta,  K_i = d2 p_i / deta2.
```

The second term is retained because the chart is nonlinear. The pullback
Jacobian and second derivatives of the Euler/product chart are numerically
differentiated. Radius-search membership can change with pose, so a finite
difference that crosses a support boundary is invalid evidence for Hessian
consistency. The reported spectrum is neither a calibrated information matrix
nor a covariance. R1 applies no eigenvalue-ratio threshold.

`WithinBasinObservability` stores the objective gradient, symmetric chart
curvature, ascending eigenvalues/eigenvectors, local convexity and chart
diagnostics. This preserves coupled translation/rotation directions rather
than reducing the result to axis flags.

### Reference point transport

If `r_map` is the vector from IMU origin to LiDAR origin, expressed in map
coordinates, first-order perturbations in these product charts satisfy

```text
eta_imu = G eta_lidar,
G[translation, rotation] has off-diagonal block [r_map]x / L,
H_imu = G^(-T) H_lidar G^(-1).
```

The sign follows `delta_t_lidar = delta_t_imu + delta_theta_map x r_map`.
R1 tests this tangent and curvature transport; runtime U_obs remains at the
LiDAR origin used by NDT.

## U_nonlocal evidence contract

`NonlocalSearchEvidence` names the finite seed domain and records planned,
attempted and converged starts plus every complete-link terminal cluster,
including sub-threshold clusters. The mature clustering producer must mark
exactly one cluster as containing the selected nominal NDT terminal; missing
membership yields `POSSIBLY_UNREPRESENTED`, and multiple memberships yield
`INDETERMINATE`. The classifier reports both support over converged starts
and support over attempted starts. It requires selected and candidate
objectives to match in backend, map identity, source identity/count, target
support/count, configured resolution, actual target-grid leaf, step size,
epsilon and maximum iterations.

Statuses are deliberately candidate-conditioned:

- `MULTI_REPRESENTED`: at least two supported, spatially distinct clusters in
  the supplied, objective-matched candidate set.
- `SINGLE_REPRESENTED`: exactly one supported cluster, all planned finite
  starts attempted, and no omitted converged terminal. The result still sets
  `unrepresented_basin_possible=true`; this is not global uniqueness.
- `POSSIBLY_UNREPRESENTED`: search was not exhausted, no cluster was
  sufficiently supported, the selected nominal terminal was absent, or
  sub-threshold cluster evidence remains.
- `INDETERMINATE`: objective provenance or candidate coverage metadata is
  invalid/incomplete, or the selected nominal terminal was assigned to more
  than one cluster.

The default support cut (`5` converged seeds and `2%` of converged starts) is
only an uncalibrated diagnostic rule for separating supported from
sub-threshold represented clusters. It is not a probability threshold, a
global-completeness guarantee, or a measurement policy.

No score, posterior weight, overlap normalization, or probability is computed
in R1. Multi-start support counts are not posterior probabilities. A future
candidate scorer must address variable overlap, source support, unmatched
returns, target density, and candidate-generator miss risk before it can
compare raw NDT scores. In particular, a normalized posterior over only the
observed candidates cannot by itself represent omitted-basin mass.

## Legacy Dual Reliability audit

The retained P6 implementation is a historical experiment, not the R1 API.

- Legacy `U_obs` (`LocalRisk`) consumed local NDT/geometric curvature products,
  separated translation/rotation blocks, used Schur-decoupled diagnostics in
  the geometric path, and also formed a joint basis in coordinates scaled by a
  configured length. It intended to express within-terminal local weakness.
  The P6-I6B audit records the PCL score/gradient coordinate gate as
  `INDETERMINATE` (8/12 axis checks passed); active U_obs use was therefore
  disabled in those runs. I2 also found the earlier fixed-scale detector had
  no demonstrated advantage over mature comparators. Those numbers do not
  validate the R1 chart contract.
- Legacy `U_nonlocal` (`NonlocalTerminalStability`) used nominal `M0` and
  prior-covariance-conditioned `M+`/`M-` probes, often triggered by innovation
  or periodically. It measured finite perturbation response and terminal
  stability. Returning to a similar terminal after local probes does not show
  that no distant basin exists; the code explicitly ended at
  `RECORDED_NO_BASIN_CLASSIFICATION`.
- Legacy `decideDualReliability()` mapped risk flags to heuristic actions and
  `makePoseMeasurementNoise()` added capped local-direction and probe-response
  covariance increments. The source labels these empirical and
  uncalibrated. This was two engineering signals routed into an update, not a
  posterior decomposition or a complete within-/between-basin uncertainty
  model.

R1 retains those files/results unchanged and does not call them from the new
diagnostic path.

## Dataflow and API

```text
scan-end IMU prediction + deskew
                 |
                 v
       one current-frame PCL NDT
                 |
          raw T_map_lidar
           /           \
          v             v
   U_obs curvature   U_nonlocal candidate evidence
           \           /
            DualUReliabilityResult
                    |
             diagnostics only
                    |
      unchanged baseline IKFoM update
```

The API is in
`src/dog_prior_map_fastlio2_frontend_exp/include/dog_prior_map_fastlio2_frontend_exp/dual_u_architecture.hpp`:

- `WithinBasinObservability` holds U_obs output.
- `NonlocalSearchEvidence` is the bounded candidate evidence input;
  `NonlocalReliability` preserves represented support and unresolved coverage.
- `DualUReliabilityResult` binds the selected raw NDT pose, stamp, both outputs
  and status. It deliberately has no filter pointer or update method.
- `CurrentFrameNdtRegistration::evaluateLocalScoreJetAtPose()` and
  `evaluateLocalObjectiveAtPose()` expose fixed-pose diagnostics only.
- `p7_single_state_runner --dual-u-shadow CSV` writes a sidecar after NDT and
  before the unchanged update. If the option is absent, the original runner
  CLI and baseline output path are unchanged.

## R1 validation

### Unit and chart tests

`dual_u_architecture_test` covers score-to-objective sign/normalization,
nonzero-gradient nonlinear chart pullback (including the `g_i K_i` term),
small-angle alternate Euler branches, fail-closed Euler singularity, LiDAR/IMU
reference-point transport, finite candidate status classification, missing
cluster accounting, provenance mismatch and a GT-free API surface.
`current_frame_ndt_test` reads back the actual 0.8 m target-grid leaf and
evaluates a real PCL terminal. Its finite-difference audit explicitly rejects
perturbations whose target-neighborhood hash changes; on the synthetic PCL
fixture the support changes, so that real-objective finite difference is
reported invalid rather than counted as a pass. The independent analytic toy
objective verifies the chart pullback along its weakest/strongest eigenvectors.
On the frozen 32-frame cohort, all U_obs rows were finite and locally convex.
We selected tx1359 and tx2350 as the minimum and maximum
smallest/largest-curvature-ratio frames within that cohort (ratios
`4.96e-4` and `4.83e-2`) and also audited historical tx120/838. At all four
terminals, both weakest- and strongest-direction finite differences crossed a
target-neighborhood boundary at the tested steps. Their status is
`INVALID_TARGET_NEIGHBORHOOD_CHANGED`; no real-PCL directional agreement is
claimed. Thus real-data local-curvature validation remains incomplete despite
the chart algebra test passing.

### Floor01 shadow

Input bundle, map, initialization, calibration and filter parameters are the
same as the frozen baseline closure. The frozen baseline trajectory is
compared byte-for-byte; registration output is compared excluding only the
wall-clock `alignment_ms` field. The R1 diagnostic has no estimator handle,
and every frame retains the baseline LiDAR update flag. Detailed sidecars are
in the external replay archive at
`/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/dual_u_architecture_r1_20261003/`.

### Historical 32-frame multi-start cohort

The frozen P5-I1 cohort remains 32 frames / 8,800 starts. Its historical
summary is retained unchanged: 19 frames had a stable represented basin;
5/19 included a correct-like stable basin; raw PCL ranking selected the
correct-like basin in all 5 cases where it was present. Candidate absence was
the dominant observed issue. This is not evidence from the corrected baseline
objective: the P5 candidate generator set target before resolution and used
`epsilon=0.001`, `max_iterations=40`; PCL 1.10 therefore built its target
covariance grid at the 1.0 m default, whereas the frozen baseline uses actual
0.8 m, `epsilon=1e-5`, `max_iterations=80`. Hence those candidate terminals
fail R1 objective-provenance matching and are `INDETERMINATE` for current
U_nonlocal. The current Floor01 shadow reports U_obs at those transaction IDs
and `INDETERMINATE / NO_SAME_OBJECTIVE_MULTI_START_SET` for U_nonlocal. The
32-row join is archived as
`Floor01_shadow_verified/dual_u_p5_i1_32frame_cohort.csv`. It reports 32/32
valid U_obs rows and 32/32 indeterminate U_nonlocal rows; the spectrum ratio
range is descriptive only and does not apply a weakness threshold. The
historical 5/5 ranking fact is not transferred to the corrected objective.

The final full shadow is stored at
`Floor01_shadow_final/`. It completed 4,127/4,127 frames with finite state,
4,127 LiDAR updates, and zero prediction-only frames. Its trajectory CSV is
byte-identical to frozen baseline Run1; all registration fields except
wall-clock `alignment_ms` are identical. All 4,127 U_obs score jets were
finite and locally convex. At tx120, tx838, tx1359, and tx2350, both the
weakest- and strongest-curvature finite-difference audits were rejected as
`INVALID_TARGET_NEIGHBORHOOD_CHANGED`; these are invalid checks, not failed
curvature inequalities and not passes. All rows remain
`INDETERMINATE / NO_SAME_OBJECTIVE_MULTI_START_SET` for U_nonlocal.

## Mature foundation and contribution boundary

Mature/reused foundations: PCL NDT score derivatives, local Hessian
curvature, Eigen eigensolvers, deterministic multi-start evidence, terminal
clustering, and the law of total covariance. None is claimed as novel.

The research hypothesis—not yet a contribution claim—is that prior-map NDT
reliability should preserve two separate objects: conditional directional
curvature within one selected basin, and candidate-conditioned support plus
unresolved risk across separated basins. The finite-candidate completeness
problem remains open. R1 does not yet provide a calibrated nonlocal posterior,
objective-matched real multi-start validation, or evidence that using the two
objects improves localization. Do not describe this as “first”, “novel”, or a
validated reliability framework.

## Current decision

Architecture status: `DUAL_U_ARCHITECTURE_PARTIAL`. The final 4,127-frame
shadow is finite and its trajectory is byte-for-byte equal to baseline Run1;
all non-timing NDT registration fields match. This run used 111,588 kB peak
RSS and 1:43.23 wall time. All three P7 standalone CTests pass. However, the
current evidence does not close a real-objective U_obs finite-difference check
(the support changes) or an objective-matched real U_nonlocal cohort (the
frozen starts have different PCL target-grid/optimizer provenance). This is a
diagnostic architecture, not permission to start fusion, recovery, or
accuracy-driven tuning.
