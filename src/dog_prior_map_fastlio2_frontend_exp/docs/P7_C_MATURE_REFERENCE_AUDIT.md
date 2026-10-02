# P7-C mature-reference source audit

## C0 gate and scope

Source review was completed before changing production code, from P7-B
`29d1f7def41eced4a197f9dec54f58aec7f1fa32`. P7-C is **shadow analysis**:
the existing single-state prediction, one current-frame PCL alignment, terminal
classification, empirical measurement noise and full-pose IKFoM correction stay
unchanged. No projected update, NIS state effect, recovery, visual input or
nonlocal probes are introduced. GT_USED=false; VISUAL=NONE.

This audit separates mature mechanisms from unproven candidate contributions.
Completing it opens the C1 implementation gate, not a claim of novelty or improved
localization accuracy.

## Source provenance and observations

### FAST-LIO2 / IKFoM — KEEP

Local checkout: `/media/jian/HIKVISION/comparison algorithm/FAST_LIO2`, pinned
`7cc4175de6f8ba2edf34bab02a42195b141027e9`.
Reviewed `include/use-ikfom.hpp` (`get_f`, `df_dx`, `df_dw`),
`include/IKFoM_toolkit/esekfom/esekfom.hpp` (`predict`, `update_iterated`), and
`src/IMU_Processing.hpp` (`IMU_init`, `UndistortPcl`).
The process model includes velocity, biases, gravity S2 and SO3; prediction
transports covariance through the manifold chart. Scan processing integrates
causal IMU, predicts the scan-end state and compensates both translation and
rotation, including the LiDAR/IMU lever arm.
[Pinned source](https://github.com/hku-mars/FAST_LIO/blob/7cc4175de6f8ba2edf34bab02a42195b141027e9/include/use-ikfom.hpp).

P7 keeps `FastLio2IkfomFrontend` and existing scan-end deskew helpers. It does not
reimplement ESKF/IEKF, IMU integration, gravity/bias dynamics, SO3 updates or
covariance propagation. The existing projected API's checked Joseph/manifold
logic is retained but not used in C. The baseline full-pose API invokes the
pinned iterated update; we do not inaccurately label that upstream implementation
as a Joseph update. The offline B/C runner reads frozen packed XYZ input, not
raw per-point timestamps; C does not introduce or claim a new runtime deskew.

### Installed PCL NDT — KEEP

Reviewed the installed PCL 1.10 headers/implementations under
`/usr/include/pcl-1.10/pcl/`: `registration/ndt.h`,
`registration/impl/ndt.hpp`, `registration/impl/registration.hpp`,
`filters/voxel_grid_covariance.h` and its implementation.
Functions reviewed: `setInputTarget`, `setInputSource`, `init`, `align`,
`computeTransformation`, terminal/convergence/iteration getters,
`getFitnessScore`, voxel `getMean`/`getCov` and `radiusSearch`.

PCL owns voxel construction, probability model, Newton step and line search.
Zero-step convergence can return with zero iterations; reaching the iteration
limit can also set convergence. P7's existing separate terminal classifier is
therefore preserved. No optimizer, score Hessian or line search is rewritten.
Fitness is nearest-target squared-distance scoring, not calibrated pose noise.

`radiusSearch` already clears leaf output in this installed version. Explicitly
clearing **both** leaf and squared-distance vectors before every P7 query is a
defensive contract, not evidence that the historical runner suffered stale
leaves. Another version-sensitive detail: target initialization precedes
`setResolution` in B; the installed setter rebuilds only when source input is
already present. Thus configured resolution must not be assumed to prove the
target grid's leaf size. C keeps the same actual grid and initialization order;
search radius and physical L use the configured 0.8 m resolution.

### hdl_localization — conceptually ADAPT, no dependency

Local checkout: `/home/jian/livox_ws/hdl_localization_ws/src/hdl_localization`,
`de2dc7769aa2412876d05ae6161408d3295b6a3c`.
Reviewed `apps/hdl_localization_nodelet.cpp` (`points_callback`,
`globalmap_callback`, `relocalize`, `publish_scan_matching_status`),
`src/hdl_localization/pose_estimator.cpp` (`predict`, `correct`), and
`include/hdl_localization/delta_estimater.hpp` (`add_frame`, `reset`).
Prediction precedes preprocessed scan registration and correction; map target
persists across frames. Global relocalization has a separate lifecycle, with
relative motion accumulated during the query. Convergence, inlier and matching
diagnostics are exposed independently.
[Source](https://github.com/koide3/hdl_localization/blob/de2dc7769aa2412876d05ae6161408d3295b6a3c/apps/hdl_localization_nodelet.cpp).

Only lifecycle/diagnostic concepts are adopted. No UKF, global-localization
service or recovery implementation enters P7-C. Existing edge cases in diagnostic
division/neighbor handling are not copied blindly.

### Autoware NDT Scan Matcher — conceptually ADAPT, no dependency

No local checkout was found. Read official raw source without cloning into the
paper workspace: `autowarefoundation/autoware_core`,
`70817eed1be6f891c0bde6b2bfe2e3bfba421bcb`.
Reviewed `localization/autoware_ndt_scan_matcher/src/ndt_scan_matcher_core.cpp`
(`process_sensor_pointcloud`, convergence/score validation, `publish_pose`,
`estimate_covariance`) and `src/ndt_omp/estimate_covariance.cpp` plus its header.
Time-interpolated input prediction seeds alignment. Missing prediction/map and
iteration/score failures have explicit diagnostics; publishing a pose is a
separate decision from producing a registration result. Covariance diagnostics
include Laplace and multi-alignment approaches.
[Core source](https://github.com/autowarefoundation/autoware_core/blob/70817eed1be6f891c0bde6b2bfe2e3bfba421bcb/localization/autoware_ndt_scan_matcher/src/ndt_scan_matcher_core.cpp),
[uncertainty source](https://github.com/autowarefoundation/autoware_core/blob/70817eed1be6f891c0bde6b2bfe2e3bfba421bcb/localization/autoware_ndt_scan_matcher/src/ndt_omp/estimate_covariance.cpp).

P7 adopts the distinction between raw result, diagnostics and validation.
Autoware's thresholds, ROS2 lifecycle, Monte-Carlo initialization, GNSS
regularization and dynamic map management are not copied. Neither Hessian
inversion nor additional covariance alignments are used in C.

### DCReg — REFERENCE ONLY

Local checkout: `/home/jian/livox_ws/DCReg`,
`ce7db8220f549a4a4391729e3bf4de4d4ab74635`.
Reviewed `DCReg/include/dcreg.hpp`: `BuildWeightedLinearSystem`,
`ReduceNormalEquation`, `DetectDegeneracy`, `CharacterizeDegeneracy`,
`AlignEigenBasisToAxes`, `SolveRawNormalEquationQR`, `SolvePreconditionedUpdate`.
It forms weighted normal equations, analyzes rotation/translation Schur spectra
and condition numbers, aligns eigenvectors to physical axes, floors selected
weak values and applies selective preconditioning. Its checked iterative solver
can fall back to QR. These are existing mechanisms, not P7 contributions.

P7 keeps the project's existing PSD pseudo-inverse Schur helper rather than
copying DCReg's ordinary block inverses. Schur is diagnostic-only. P7 does not
copy axis remapping, weak-value repair, preconditioner or registration solver,
and does not link DCReg.

### X-ICP / perfectlyconstrained — novelty boundary, no dependency

No local source was found. Reviewed official
`leggedrobotics/perfectlyconstrained` at
`0fbe4175ea205a271f85287abfd8048e5f7dd32a`, without cloning.
Reviewed `libpointmatcher/pointmatcher/ICP.cpp`
(`calculateOptimizationHessian`, `eigenAnalysis`, `detectLocalizability`,
`detectLocalizabilityWithOptimizedMethod`, solution-remapping helpers) and
`ErrorMinimizers/PointToPlane.cpp` (constraint construction and solver dispatch).
Point-to-plane contributions are evaluated against principal directions to
classify localizability; constraints guide optimization in weak directions.
The repository also implements compared solution-remapping and other mitigation
approaches; these must not all be attributed to the original X-ICP method.
[Detection source](https://github.com/leggedrobotics/perfectlyconstrained/blob/0fbe4175ea205a271f85287abfd8048e5f7dd32a/libpointmatcher/pointmatcher/ICP.cpp),
[constraint source](https://github.com/leggedrobotics/perfectlyconstrained/blob/0fbe4175ea205a271f85287abfd8048e5f7dd32a/libpointmatcher/pointmatcher/ErrorMinimizers/PointToPlane.cpp).

Principal-direction detection, weak-direction classification, constrained ICP
and solution remapping establish prior-art boundaries. No X-ICP code or library
is imported into the P7 production graph.

## Reference decision matrix

| Subsystem | Mature reference | Reference file/function | Mechanism | P7 decision | COPY / ADAPT / KEEP / REJECT | Reason |
| --- | --- | --- | --- | --- | --- | --- |
| IMU propagation | FAST-LIO2 | use-ikfom.hpp: get_f/df_dx/df_dw | Bias/gravity inertial model | Existing frontend | KEEP | No second estimator |
| Manifold filter update | IKFoM | esekfom.hpp: predict/update_iterated | SO3/S2 chart and covariance transport | Existing API | KEEP | Mature pinned implementation |
| Deskew | FAST-LIO2 | IMU_Processing.hpp: UndistortPcl | Scan-end SE3 compensation | Existing scan helpers | KEEP | No time-reference redesign |
| NDT optimizer | PCL | ndt.hpp: computeTransformation | Newton / line search / target grid | Existing object | KEEP | Analysis outside optimizer |
| Prediction-registration lifecycle | hdl_localization | points_callback / PoseEstimator::correct | Prediction, registration, correction | Conceptual lifecycle | ADAPT | Keep IKFoM, not UKF |
| NDT validation | Autoware | process_sensor_pointcloud / publish_pose | Raw result separate from validation | Independent diagnostics | ADAPT | No threshold/ROS2 transplant |
| Schur analysis | DCReg | DetectDegeneracy / CharacterizeDegeneracy | Conditional spectra | Reference only; existing PSD helper | KEEP | Established analysis, diagnostics only |
| Weak-direction constrained registration | X-ICP | detectLocalizability / PointToPlane solver | Direction classification and constraints | NOVELTY BOUNDARY | REJECT | Not imported or claimed as ours |
| Target geometric information | Existing P6 + PCL grid | geometricObservations / analyzeGeometricObservability | Weighted covariance-derived J'WJ | Minimal shadow extraction | ADAPT | Same prepared source and grid |

## C1 numerical and runtime contract

Keep Eigen decompositions and existing target-covariance symmetrization/floors
(absolute 1e-6 m², relative 1e-3). Covariance inversion safeguards are not pose
priors. Do not add epsilon-I to pose information. Save
`H_phys = sum(w Jᵀ Sigma^-1 J)/sum(w)`, with `J=[-skew(Rp), I]`.
Use fixed `D=diag(1,1,1,L,L,L)` and `H_bar=DᵀH_physD`, L=0.8 m from configured
NDT resolution, fixed throughout the run. This is physical nondimensionalization,
not per-frame block equalization or min-max scaling.

The P7 classifier uses the full joint spectrum and preserves coupled modes.
It uses a 100-machine-epsilon spectral floor, rejects significant negative
information and checks orthogonal bases. Ratio 0.05 is a frozen first-version
engineering threshold, not a theoretical optimum or universal constant; later
cross-dataset validation/sensitivity study is required. Schur unavailability
does not veto a valid joint spectrum. True numerical failures fail closed;
finite unusable individual target covariances may be rejected as observations,
with counts and the existing support requirement retained.

P7 does not link the historical scale-balanced `dual_reliability` path. Geometry
is frame-local, only computed after SUCCESS, and cannot affect filter state,
measurement noise, NDT validity or the next seed. Prechange/shadow comparison
must verify all 100 state and registration records, separately from timing.

## INNOVATION_BOUNDARY

The following are not claimed as independent innovations: NDT; FAST-LIO2/IKFoM;
Schur complement; Hessian-eigenvalue degeneracy detection; finding or suppressing
weak directions; retaining observable directions; selective constrained ICP.

### CANDIDATE CONTRIBUTION — not yet established

A. Current-frame information from actual NDT target-voxel covariance support.
B. Explicit fixed-physical nondimensionalization instead of frame-dependent
block equalization.
C. A joint weak/reliable subspace retaining rotation-translation coupling.
D. Later exact mapping into the IKFoM pose-residual chart.
E. Later study of reliable LiDAR constraints plus weak-direction visual
compensation (not implemented in C).
F. Prior-map localization with a single-state, low-memory/low-compute runtime.

These are candidate research directions only. The combination's novelty is not
proven, and neither integration nor completed code establishes it.

## EFFECTIVENESS_GATE

P7-C is a diagnostic foundation, not a performance improvement. P7-D and later
visual work must compare full-pose NDT+IKFoM against reliable-subspace projected
LiDAR and, separately, projected LiDAR plus weak-direction visual compensation.
Required measurements include trajectory error, degenerate-segment error,
maximum error, wrong LiDAR corrections, prediction-only duration, recovery
success, runtime and peak RSS. GT may support later offline evaluation, never an
online decision.

If a candidate cannot reliably beat baseline on genuinely degenerate data, or
has negligible benefit with substantial resource cost, discard it as a main
paper contribution. Do not retain or explain away a mechanism merely because
its code exists.

### NOVELTY_GATE

Before finalizing contributions, separately compare X-ICP, DCReg and recent
degeneracy-aware ESKF/LIO work. Establish that the final mechanism is not renamed
weak-direction suppression, Schur characterization or solution remapping.
Novelty fully proven: NO.
