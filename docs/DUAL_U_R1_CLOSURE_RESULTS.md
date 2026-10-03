# Dual-U R1 closure results

Date: 2026-10-03
Baseline parent: `70aa6859657c92404751cd354bd4292e9d30c617`
R1 start: `b1527844a3059c776f308d1ff162c502282c79eb`

## Decision

`DUAL_U_ARCHITECTURE_ESTABLISHED` for the diagnostic architecture only.

The two closure gates passed: (1) the current PCL NDT branch-local curvature
matrix agrees with finite differences of a frozen-active-support objective on
four real frames, and (2) the frozen 32-frame multi-start evidence was
regenerated using the current 0.8 m PCL target grid and current baseline
optimizer/source inputs, with complete provenance and runnable candidate-
conditioned classification. A full 4,127-frame shadow replay remained
byte-identical to the frozen baseline trajectory.

This does **not** establish novelty, posterior calibration, global basin
completeness, localization accuracy improvement, or a fusion/recovery policy.
No estimator output or update rule was changed.

## U_obs: branch-local curvature validation

The tested object is the current NDT score objective, normalized as a cost per
prepared source point and conditioned on the active target-cell support at the
selected terminal. Let `T*` be the terminal and `A_i*` the exact ordered set of
target Gaussian cells returned by PCL's radius search for prepared source point
`i` at `T*`. With PCL's existing Gaussian means/covariances/normalizers,

```text
J_A*(eta) = -(1/N) sum_i sum_{g in A_i*}
              [-gauss_d1 exp(-gauss_d2 r_ig(eta)^T C_g^-1 r_ig(eta) / 2)]
```

where the outer minus converts PCL's maximized score to a minimized cost.
Membership is frozen for `J_A*`; target Gaussians are not rebuilt or changed.
The current U_obs chart is the existing product chart

```text
eta = [delta_t_map / L, delta_theta_map]
t(eta) = t* + L eta[0:3]
R(eta) = Exp(eta[3:6]) R*
L = actual PCL target-grid leaf = 0.8 m
```

Thus translation is additive in map axes and rotation is a left/map-spatial
rotation about the LiDAR origin. The reported matrix is
`H_eta = d² J_A* / d eta²`, obtained by the existing PCL Euler-coordinate
score gradient/Hessian and the existing product-chart pullback, including the
gradient times chart-second-derivative term. Its ascending eigenpairs retain
coupled 6D directions; no scalar degeneracy threshold or covariance claim is
added.

The frozen-support implementation captures PCL target-cell identities at the
center, and subsequent frozen evaluations do not run a new radius search.
Capture also evaluates the center through both frozen scoring and the PCL
runtime score-jet path, rejecting a mismatch above `1e-11` relative scale.
Dynamic diagnostics separately rerun the actual radius search. The test fixture
reported center score difference `0`, with `205/1400` source-point supports
changed and 229 membership identities in symmetric difference for its
nonzero displacement; the four real-frame replay completed all captures.

### Real-frame spectra and eigendirections

Eigenvalues below are ascending in the `eta` chart. Vectors are `[tx/L, ty/L,
tz/L, theta_x_map, theta_y_map, theta_z_map]`; `q0` is weakest and `q5`
strongest. The finite-difference sidecar also stores `q2`, `q3`, every sampled
`h`, all predictions and objective samples.

| tx | source points | center Gaussian memberships | eigenvalues `lambda0 ... lambda5` |
|---:|---:|---:|---|
| 120 | 1400 | 2890 | 2.677861, 3.080990, 3.222171, 12.212700, 563.275390, 601.768780 |
| 838 | 1400 | 4145 | 2.368528, 4.109373, 5.953270, 30.094168, 875.703805, 1033.098581 |
| 1359 | 1400 | 2917 | 2.416787, 3.076451, 3.443530, 1120.141471, 2253.914846, 4867.925383 |
| 2350 | 436 | 1404 | 3.073591, 4.117925, 5.755986, 6.987796, 58.557182, 63.596619 |

| tx | q0 (weak) |
|---:|---|
| 120 | [0.980370, 0.120776, -0.154289, 0.021542, -0.004368, -0.000445] |
| 838 | [-0.075388, 0.175384, -0.937308, 0.290208, 0.025178, -0.012503] |
| 1359 | [-0.483495, -0.044965, -0.873908, -0.005163, 0.021635, 0.000980] |
| 2350 | [0.447281, 0.076986, -0.757775, -0.406465, -0.041758, -0.229853] |

| tx | q5 (strong) |
|---:|---|
| 120 | [-0.001505, 0.016600, -0.002617, 0.004615, 0.135457, 0.990629] |
| 838 | [0.006785, 0.012800, -0.000111, -0.052827, 0.040483, -0.997678] |
| 1359 | [0.000323, -0.007789, -0.000204, 0.159954, -0.023723, 0.986809] |
| 2350 | [0.031610, 0.004483, 0.045621, -0.420786, -0.508593, 0.749114] |

For each of four frames, directions `q0`, `q2`, `q3`, `q5` were evaluated at
`h = 0.02, 0.01, 0.005, 0.0025, 0.001, 0.0005, 0.00025`. The sidecar records
`J(0)`, `g^Tq`, `q^THq`, `J_pred(±h)`, frozen `J_A*(±h)`, central second
difference, dynamic PCL score/cost at `±h`, and support deltas. Define the
reported normalized discrepancy as

```text
abs(central_FD - q^T H q) / max(1, abs(central_FD), abs(q^T H q)).
```

All 16 directions have at least one adjacent pair of step sizes for which both
discrepancies are below `5e-3`; the worst, over directions, of each direction's
best discrepancy is `6.70915e-4`. At the common `h=0.005`, 12/16 direction rows
are below `5e-3` (median `1.92779e-3`, max `4.71663e-2`). Weak-to-strong
curvature ordering agrees with the frozen objective responses. At the smallest
steps, some rows become noisier due to PCL's float pose/point transform
precision; the conclusion is based on adjacent convergent scale regions,
not cherry-picking the smallest `h`.

Representative complete raw rows:

| tx, eig, h | `J0` | `g^Tq` | `q^THq` | frozen `J(+)` / `J(-)` | predicted `J(+)` / `J(-)` | central FD | rel. error |
|---|---:|---:|---:|---|---|---:|---:|
| 120, q0, 0.005 | -1.996387472662 | 0.000933052084 | 2.677860587483 | -1.996349330544 / -1.996358657478 | -1.996349334144 / -1.996358664665 | 2.678292058640 | 1.6110e-4 |
| 2350, q5, 0.005 | -2.805959984522 | 0.028059018778 | 63.596619431903 | -2.805024660980 / -2.805306192306 | -2.805024731685 / -2.805305321873 | 63.564630313202 | 5.0300e-4 |

### Dynamic runtime support switching (separate from the frozen test)

Support means the per-source-point list of **target Gaussian cell identities**
returned by PCL `target_cells_.radiusSearch(transformed_point,
resolution)`. It is not a change to map voxels or Gaussian parameters. Changed
point count increments when that source point's sorted cell-identity list
differs; membership symmetric difference counts inserted plus removed cell
identities across all points.

Across all 224 direction/step/sign perturbations, 29 had zero changed source
points; changed-point fraction median was 1.0%, P90 14.29%, P95 25.36%, max
54.86%. At `h=0.005`, across the 32 direction/sign samples, median changed
fraction was 1.15%, P95 16.21%, max 25.36%; membership symmetric difference
median was 15, P95 275, max 420. Center memberships at these 16 directions
ranged 1404–4145.

Examples at `h=0.005`:

- tx120 q0: `14/1400` changed (+), `17/1400` (-); center memberships 2890,
  perturbed memberships 2888/2893, symmetric differences 14/17. Dynamic costs
  were -1.995264005552 / -1.997819541026, versus frozen costs
  -1.996349330544 / -1.996358657478.
- tx1359 q5: `335/1400` (+), `355/1400` (-), i.e. 23.93%/25.36%; center
  memberships 2917, perturbed counts 2875/2921, symmetric differences 418/420.
- tx2350 q5: `29/436` (+), `22/436` (-), i.e. 6.65%/5.05%; center
  memberships 1404, perturbed counts 1396/1412, symmetric differences 30/22.

This validates the branch-local curvature implementation while showing that
its dynamic runtime applicability radius depends on direction and frame.
Support switching is diagnostic metadata only; it is not converted to a
confidence scalar or fusion weight.

## U_nonlocal: current-objective evidence

The historical P5-I1 terminals were not reused. The prior evidence had source
hash mismatch in 32/32 frames, source point-count mismatch in 9/32, actual PCL
target grid 1.0 m instead of 0.8 m, epsilon `0.001` instead of `1e-5`, maximum
iterations 40 instead of 80, and different prediction/seed centers. Its mode
counts are not pooled with this run.

The new cohort was generated from the 4,127-frame scan-end-deskew baseline
replay. Verified hashes:

- map PCD: `2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570`
- runtime-topic bag: `860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db`
- objective provenance: `1eb0afc2416a555817a23b583c87675afe5c817213b6ed03f9ede3b9212ffeb1`
- search provenance: `f307e69d1d22ab3e4acd92503f71bf47a4f44e6f37f0180f8ca8a7ef51e444b0`

Every frame records raw scan-end source SHA-256/count, prepared source FNV64
hash/count, map/bag SHA-256, target point count, full initial/raw terminal
poses and initial-pose hash, preprocessing-contract hashes, configured
resolution, actual grid leaves, step/epsilon/iteration limit, and per-frame
objective/search provenance hashes. Source preprocessing is finite/range
filter 0.5–80 m, 0.25 m voxel, evenly-spaced cap 1400. Map preprocessing is
finite filtering then 0.15 m and 0.15 m voxel passes. PCL is
`1.10.0+dfsg-5ubuntu1`; actual target leaves were `(0.8000000119,)*3` m.
NDT used resolution 0.8 m, step 0.08, epsilon `1e-5`, max iterations 80.
The old P5-I1 seed generation and cluster implementation/configuration were
reused unchanged.

Seed contract: 245 planar + 18 axial starts on each of 32 frames; the eight
predeclared targeted frames `838, 839, 924, 925, 1497, 1498, 1556, 1557` add
48 wide planar starts each. Perturbation remains right/body
`T_seed = T0 Exp(delta)`. Total planned/attempted was 8,800/8,800; all 8,800
reported PCL convergence. The 0.2 m / 2 degree primary complete-link clusters,
5-seed and 2%-of-converged support rule, and finite-domain caveat were kept.

All 32 nominal zero-seed outputs reproduced their current baseline raw
terminal within max `3.81656e-6 m` and `1.37667e-5 deg` (gate: 0.001 m and
0.01 deg). Primary clustering yielded 2,070 terminal clusters: 214 supported
and 1,856 subthreshold. Candidate-conditioned statuses:

| Status | Frames | Explanation |
|---|---:|---|
| `MULTI_REPRESENTED` | 29 | selected basin supported and at least two supported clusters |
| `POSSIBLY_UNREPRESENTED` | 3 | tx616 and tx2350 selected clusters had support 3; tx1606 had subthreshold clusters remaining |
| `SINGLE_REPRESENTED` | 0 | no frame met the one-supported-cluster/no-subthreshold-cluster condition |
| `INDETERMINATE` | 0 | all provenance, attempt, cluster-coverage and selected-membership checks passed |

Even `SINGLE_REPRESENTED` would mean only one cluster in this exhausted finite
seed domain; `exact_global_completeness_proven` remains false and an
unrepresented basin remains possible. Candidate status rules were not tuned
using GT.

### Post-hoc GT sanity check

Only after candidate, cluster and status files were frozen and hashed, the
existing P5-I1 post-hoc evaluator read GT. GT/extrinsics hashes matched the
evaluator's expected values; GT was never used online or in generation,
clustering, thresholding or classification.

All 32 frames had at least one supported stable cluster. The selected
zero-seed-associated basin was the GT-nearest supported basin in 15/32 frames.
The GT-nearest supported basin was also top-ranked by raw PCL score in 9/32.
Thus this finite candidate evidence contains both represented alternatives
and cases where raw-score rank does not select the post-hoc nearest basin; it
does not make GT a runtime rule. Mean/median/P95/max translation errors were:

| Pose set | mean | median | P95 | max |
|---|---:|---:|---:|---:|
| baseline zero-seed pose (32) | 0.8264 m | 0.6631 m | 1.6328 m | 1.6654 m |
| nearest supported candidate (32) | 0.7816 m | 0.6305 m | 1.6049 m | 1.6356 m |

These are cohort post-hoc diagnostics, not a localization A/B result. There was
no GT-defined correctness threshold and no feedback to U_nonlocal.

## Shadow parity, runtime, and tests

The new full shadow replay processed 4,127/4,127 frames, had 4,127 LiDAR
updates, zero prediction-only frames and finite state throughout. Its
trajectory SHA-256 is
`86d8de73bb5af2015fd37cf33584dcd3a1e54fba87d2e6a38c435eb949520bd9`, exactly
the frozen baseline SHA-256. Wall time was 1:49.31 and peak RSS 111,852 kB.

The 8,800 alignment diagnostic took 11:19.42 wall time and peak RSS 103,116
kB. That is offline evidence-generation cost, not proposed online cost.

Build/tests:

- `current_frame_ndt_test`: PASS; actual grid `(0.8,0.8,0.8)` m, frozen center
  score delta 0, changed support reported quantitatively.
- `p7_replay_io_test`: PASS.
- `dual_u_architecture_test`: PASS.
- P7 standalone CTest: 3/3 PASS.
- `git diff --check`: PASS at closure.

Implementation adds diagnostic-only frozen/dynamic support methods to
`current_frame_ndt.*`, a P7 runner objective-export/curvature-audit option, a
same-baseline 8,800-run diagnostic executable, and GT-blind provenance and
clustering scripts. No filter, estimator update, baseline covariance, visual,
recovery, fixed-lag or window code was changed.

## Archived evidence

The complete raw sidecars are retained outside the repository under:

`/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/dual_u_r1_closure_20261003/same_objective/`

Key files are `curvature_fd.csv`, `source_clouds/cohort.csv`,
`frozen/cohort_frozen.csv`, `frozen/objective_provenance.json`,
`candidates.csv`, `clusters/mode_clusters.csv`,
`clusters/nonlocal_frame_diagnostics.csv`, and `posthoc_gt/`.
