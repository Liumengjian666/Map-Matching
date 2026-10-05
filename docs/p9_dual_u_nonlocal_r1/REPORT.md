# P9-R1 reduced-subspace nonlocal experiment

## Decision

`FINAL_RESULT = WEAK_SUBSPACE_SEARCH_INSUFFICIENT`.

The exact PCL 1.10 optimization-energy contract and the frozen-support directional derivative check are closed for the sampled archive. However, the bounded weak-subspace profile prototype recovered **0 of 22** major competing oracle basins. It therefore does not support this reduced-search implementation as a basin-discovery method. It is an offline negative result; no production EKF or localization behavior was changed.

Laplace basin weights, posterior probabilities, `U_nonlocal`, `U_within`, and the candidate measurement covariance were not computed. The PCL score is a comparable fixed-frame optimizer objective, not a normalized negative log posterior; dynamic support also switches substantially across profile samples, and many terminal points are not stationary.

## Workspace and evidence provenance

- Clean implementation branch: `research/p9-dual-u-nonlocal-r1`, created from baseline `70aa6859657c92404751cd354bd4292e9d30c617`.
- Stable workspace `/home/jian/livox_ws/dog_visual_loc_ws` was not touched.
- No oracle multi-starts were rerun. The historical archive is `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/dual_u_r1_closure_20261003/same_objective/`.
- Oracle provenance: 32 frames, 8,800 planned/attempted alignments, 2,070 primary clusters, 214 supported primary clusters; all finite scheduled seeds exhausted, but global completeness was not proven. Archive records `gt_accessed=false` for the candidate search.
- Bag SHA-256: `860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db`.
- Frozen target map SHA-256: `2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570`; prepared target count 549,606.
- Frozen objective provenance SHA-256: `1eb0afc2416a555817a23b583c87675afe5c817213b6ed03f9ede3b9212ffeb1`.
- PCL `1.10.0+dfsg-5ubuntu1`, NDT resolution `0.8 m`, step `0.08`, epsilon `1e-5`, maximum 80 iterations, outlier ratio `0.55`.
- Source preprocessing: finite XYZ; inclusive range `[0.5, 80] m`; PCL voxel leaf `0.25 m`; deterministic 1,400-point cap using inclusive evenly spaced indices. Per-frame prepared point counts are archive-specific and are verified by hash.
- Target preprocessing: finite XYZ, two successive `0.15 m` PCL voxel filters; `setResolution` before `setInputTarget`; PCL target grid leaf observed as `0.800000011920929 m`.
- Source preprocessing config SHA-256 `30a5d27d98e6e41af83c63d6242cc9cfbb4e7544ebe150be2f2c1cfa85ac784d`; target preprocessing config SHA-256 `2893fd722f02792af6540858d8e12d84af677a216c207fdf1bf98275568ea0d0`.
- Frozen archive has no EKF prediction covariance. The search therefore used the task-authorized physical fallback (`FALLBACK_BOUND_USED=YES`): combined weak displacement limited to at most 2 m translation and 15 degrees rotation. This is a bounded experimental region, not a probabilistic confidence interval.

## U_obs reuse and coordinate contract

`UOBS_REUSABLE = PARTIAL`.

The archived R1 implementation is on `research/dual-u-architecture-r1` at closure `9945c4f5c3d7759104de108a594bcaf2553fd78c`:

- `src/dog_prior_map_fastlio2_frontend_exp/src/dual_u_architecture.cpp`: `analyzeWithinBasinObservability`, `buildMapProductChartPullback`, and `applyMapProductChartIncrement`.
- Matching declarations are in `src/dog_prior_map_fastlio2_frontend_exp/include/dog_prior_map_fastlio2_frontend_exp/dual_u_architecture.hpp`.

The reused chart is `eta=[delta_t_map/L, delta_theta_map]`, `L=0.8 m`; translation is additive in map coordinates and rotation is a left/map-spatial rotation about the LiDAR origin, `R(eta)=Exp(delta_theta)R*`. It is not a six-dimensional left SE(3) twist. Curvature eigenvectors are in this dimensionless chart, sorted from weakest to strongest. The native PCL coordinates use `[tx,ty,tz,roll,pitch,yaw]` and the PCL `Rx Ry Rz` convention; the R1 pullback includes the chart Jacobian and the gradient-weighted chart second derivative. R1 frozen-support FD validation used real frames tx120, tx838, tx1359, and tx2350. R1 source was inspected and reused as a convention reference; no old optimizer/recovery architecture was cherry-picked.

## PCL objective contract and energy gate

For transformed source point `x_i(T)` and every target Gaussian cell `g` returned by PCL's radius search at NDT resolution, define

```text
q_ig = (x_i - mean_g)^T inverse_covariance_g (x_i - mean_g)
score_ig = -d1 * exp(-d2 * q_ig / 2)
S(T) = sum_i sum_{g in A_i(T)} score_ig
E(T) = -S(T)
```

The offline evaluator matches the PCL 1.10 NDT score sum, including its dynamic radius-search support and Gaussian-cell mean/inverse-covariance data. PCL maximizes `S`; the reported evaluator energy is its negative. For same-frame ranking, `E/N` is also recorded. At `rho=0.55`, `r=0.8 m`: `c1=4.5`, `c2=1.07421875`, `d1=-1.646558519810`, `d2=0.528248766985`. The PCL derivative guard `d2*exp(-d2*q/2) <= 1` is inactive here because `d2<1` and `q>=0`.

This closes an exact **same-objective optimizer-energy** comparison; it does not create a normalized likelihood. PCL has no determinant term for each target covariance nor a normalization over the pose-dependent set of returned Gaussian cells. `getFitnessScore()` is nearest-neighbor distance, not the NDT objective. PCL's transformation-probability helper divides score by point count and its source cautions that constants must change for global accuracy. These quantities do not justify posterior odds or Laplace evidence.

Installed-source locations inspected:

- `/usr/include/pcl-1.10/pcl/registration/impl/ndt.hpp:59-64` — score constants.
- `.../ndt.hpp:199-225` — per-point radius search and summation over returned Gaussian cells.
- `.../ndt.hpp:352-367` — Gaussian score contribution and derivative guard.
- `/usr/include/pcl-1.10/pcl/filters/impl/voxel_grid_covariance.hpp:287-345` — covariance-cell computation and regularization (minimum six points; minimum eigenvalue regularized to 0.01 of maximum by default).
- `/usr/include/pcl-1.10/pcl/registration/impl/registration.hpp:117-145` — nearest-neighbor `getFitnessScore()` semantics.

The evaluator was checked on 39 archived terminal poses (20 nominal and 19 selected competitor terminals) over 20 fixed frames (10 low-cluster controls and 10 high-multimodal frames). All 39 score sums matched the archive exactly (maximum absolute difference `0`). All 39 local chart Hessians were positive definite. For the 78 terminal/direction pairs (weakest `q0` and strongest `q5`) over the predeclared step list `0.02, 0.01, 0.005, 0.0025, 0.001, 0.0005`, the best normalized frozen-support gradient and directional-curvature discrepancy were each below `0.005` in 78/78 pairs. The discrepancy is `abs(a-b)/max(1,abs(a),abs(b))`; full raw values are in `sidecars/energy_gate_final_v1/directional_fd.csv`.

Dynamic support is not smooth: over both signs and all FD step samples, changed-source-point fraction had median `2.43%`, P90 `23.46%`, P95 `36.50%`, maximum `55.64%`. Percentiles use linear interpolation over sorted samples. Dynamic-support FD met the same `0.005` discrepancy threshold for only 10/78 gradient pairs and 1/78 curvature pairs. Thus the mathematical branch check passes, while a globally smooth dynamic-support Hessian interpretation does not.

The 39 terminal chart-gradient norms had minimum `1.26581e-5`, median `0.132767`, P90 `1.23605`, P95 `2.03387`, and maximum `4.54034`; only 18/39 were `<=0.1`. Percentiles use linear interpolation over sorted samples. Archived PCL convergence flags are not a stationarity certificate (PCL also marks iteration-limit termination as converged). Accordingly, score comparability is closed, but terminal stationarity is only partial.

## Reduced profile method

Each frame uses the archived nominal NDT terminal and the archived `U_obs` eigensystem. The automatic rule is `k=1` if `lambda1/lambda0 >= 2`, otherwise `k=2`; `k` is never above 2. The weak basis is the first `k` eigenvectors and the remaining eigenvectors form the strong basis. The evaluated profile approximates `Phi(u)=min_v E(Wu+Sv)`: at each weak grid point, start with `v=0`, take at most one damped strong-subspace Newton/GN step with `[1, 0.5]` line search, then evaluate the dynamic-support PCL energy.

The 1D search uses 17 coordinates; the 2D search uses a 9x9 grid, omitting points that violate the combined fallback translation/rotation limits. Only complete-neighborhood interior discrete minima are retained. Each retained minimum receives one full-pose NDT refinement with the frozen NDT settings. Refined terminals are clustered by complete-link using 0.2 m translation and 2 degree geodesic rotation cutoffs. Pose comparisons use the SE(3) logarithm; Euler-angle Euclidean distance is not used.

An oracle basin is counted as major when it is a supported primary cluster, is separated from the nominal terminal by more than 0.2 m or 2 degrees, and is within 5% of the best supported archive score per source point. Recovery requires a refined reduced terminal to be in a non-nominal reduced cluster and within 0.2 m / 2 degrees of that oracle cluster. These labels use only archived same-objective optimizer results, never GT.

## Results

All sample percentiles reported below use linear interpolation over sorted samples.

| Measure | Result |
|---|---:|
| Frames | 32 |
| `k=1` / `k=2` | 1 / 31 |
| Profile grid nodes | 1,672 |
| Dynamic energy evaluations | 5,130; mean 160.31/frame |
| Discrete profile minima | 34 |
| Full NDT refinements | 34; mean 1.0625/frame |
| Frames with >1 reduced terminal cluster | 2/32 |
| Oracle supported clusters represented across cohort | 214 |
| Oracle major competing basins | 22 in 9 frames |
| Recovered / missed oracle major basins | **0 / 22** |
| Lowest-cluster-count Floor01 control frames with >1 reduced terminal cluster | 0/20 |
| Profile energy samples changing support vs nominal | median 92.82%, P90 99.71%, P95 100%, max 100% |
| Summed profile runtime | 801.912 s; mean 25.060 s/frame |

The profile search reduced the count of full NDT align/refine calls from the 8,800-alignment exhaustive oracle by a factor of `258.8` (99.61% fewer full NDT calls). This does **not** imply an online speedup: profile evaluation itself took about 25.1 seconds per frame on this machine. It is an offline search prototype only.

The 20-frame healthy-control stratum is post-hoc: the 20 cohort frames with the fewest supported clusters were selected after the archive was frozen. They are not an independent or GT-defined sample. Zero of these 20 produced more than one reduced terminal cluster, so the prototype did not show broad false multimodality on this control stratum; this small same-archive control does not establish a calibrated false-positive rate.

## Representative misses

All energy gaps below are same-frame **mean optimization-score gaps per prepared source point** relative to the best supported oracle cluster, `delta_Ebar=(S_best-S_b)/N`; they are not posterior NLL gaps. `pi`, weighted `U_nonlocal` eigenvalues, and a weighted principal ambiguity direction are all **INDETERMINATE / NOT COMPUTED**.

| tx | Selected nominal terminal (xyz m) | Major oracle competitor (cluster; xyz m) | Separation | `delta_Ebar` | Weak-space diagnostic | Reduced outcome |
|---:|---|---|---|---:|---|---|
| 616 | T0 `(5.757877, 5.091570, 2.613630)` | P10 `(5.814020, 5.082299, 3.123900)` | 0.51343 m / 9.6832 deg | 0.0219580 | 0.9859 displacement fraction in weak span; inside fallback | Two reduced terminal clusters, but neither non-nominal terminal fell within the oracle recovery 0.2 m / 2 deg gate; P10 missed. |
| 2226 | T0 `(29.249029, 4.538933, 3.744729)` | P09 `(30.000723, 4.588530, 3.257314)` | 0.89726 m / 17.2336 deg | 0.0307513 | 0.9817 displacement fraction in weak span; inside fallback | One reduced terminal cluster; P09 missed. |
| 2350 | T0 `(28.859268, 5.820776, 7.757048)` | P03 `(28.827845, 5.854447, 6.614619)` | 1.14336 m / 6.3103 deg | 0.00141258 | 0.9682 displacement fraction in weak span; the projected rotation component exceeded the 15 deg bound | No retained profile minimum/full refinement; P03 missed. |

Across all 22 major basins, the previously computed mean/median weak-span projection fractions were 0.8042/0.9238; 13/22 were at least 0.8, and 18/22 were inside the fallback region. This suggests the misses are not generally explained by competing displacement being orthogonal to `U_obs` weak space. It does not isolate the cause: the grid is coarse, strong-subspace minimization is only one step, and dynamic support changed at most profile nodes (1641/1672 had one-step status `STRONG_MAX_ITER`; 31/1672 had `STRONG_NO_DESCENT`). These are plausible implementation limitations, not a demonstrated causal diagnosis.

## Interpretation and next step

`WEAK_SUBSPACE_SEARCH_INSUFFICIENT`. The current reduced profile implementation fails the required major-basin recovery condition. The experiment does **not** disprove the general weak-subspace hypothesis: most measured competing displacements have substantial weak-space projection, but the bounded profile approximation did not find/refine them into the correct separate terminals. No probability weights or uncertainty tensor should be reported from this run.

**NEXT:** stop this prototype as a supported algorithm and retain the archived negative result. Do not connect it to the production EKF. A new profile-minimization design would require separate authorization; this result is not a license for an unbounded heuristic/probe sweep.

## Machine-readable artifacts

- `results.json` — summary, contracts, and final scientific status.
- `sidecars/energy_gate_final_v1/terminal_energy.csv` and `directional_fd.csv` — per-terminal score/curvature and raw FD/support diagnostics.
- `sidecars/profile_cohort_final/profile_summary.csv`, `profile_landscape.csv`, `profile_terminals.csv`, `terminal_relations.csv`, and `oracle_recovery.csv` — per-frame profile, terminal, relation, and oracle recovery evidence.
