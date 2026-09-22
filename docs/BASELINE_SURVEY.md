# Baseline Survey for Mode-Structured NDT Reliability Analysis

Survey date: 2026-09-22. Sources are restricted to papers, project pages, and author-maintained repositories. Tasks are kept distinct: prior-map localization, LiDAR registration, odometry, and SLAM are not interchangeable.

## Executive conclusion

The defensible main comparison is not “ours versus every recent LIO.” The main table should contain Vanilla NDT, Autoware classic NDT localization, hdl_localization, FAST_LIO_LOCALIZATION, and the proposed method. ICP/GICP should be added under a same-input registration protocol if the adapter is fair. A second method-level table should compare Hessian/condition-number detection, Park-style NDT pose-distribution covariance, DCReg-style decoupled characterization, and the proposed mode-structured reliability. SuperLoc and one of DCReg/GenZ-ICP are strong recent references; FAST-LIO2 and LIO-SAM anchor supplementary degraded-scene context.

## Paper-by-paper audit

### 1. LIO-SAM: Tightly-coupled LiDAR Inertial Odometry via Smoothing and Mapping

- Year: 2020 preprint / IROS 2020.
- Proposed method: factor-graph LiDAR-inertial odometry and mapping with IMU preintegration, local feature-map matching, GPS and loop-closure factors.
- Datasets: self-collected multi-platform urban/campus/park-scale runs and public-data demonstrations described by the paper/project.
- Compared baselines: LOAM and LIOM are the principal quantitative contemporaries reported by the paper; ablations examine sensor/factor contributions.
- Classic baselines: LOAM.
- Contemporary baselines at publication: LIOM.
- Metrics: absolute/relative trajectory accuracy, map quality, and runtime/real-time behavior.
- Code public: YES, https://github.com/TixiaoShan/LIO-SAM
- Relevance: mature factor-graph LIO anchor; not a direct prior-map-localization baseline.
- Paper: https://arxiv.org/abs/2007.00258

### 2. FAST-LIO2: Fast Direct LiDAR-inertial Odometry

- Year: 2021 preprint / T-RO 2022.
- Proposed method: tightly coupled iterated Kalman filter, direct raw-point-to-map registration, and incremental ikd-Tree mapping.
- Datasets: 19 sequences across multiple public datasets plus aggressive solid-state-LiDAR experiments.
- Compared baselines: LILI-OM, LIO-SAM, and LINS in the principal system comparison; data-structure comparisons are separate.
- Classic baselines: none task-identical in the main LIO table.
- Contemporary baselines: LILI-OM, LIO-SAM, LINS.
- Metrics: translational trajectory RMSE, per-scan/runtime load, and qualitative mapping/robustness.
- Code public: YES, https://github.com/hku-mars/FAST_LIO
- Relevance: mature direct point-to-map LIO and a high-frequency odometry anchor, but it builds/updates its own map rather than localizing against a fixed prior map.
- Paper: https://arxiv.org/abs/2107.06829

### 3. Direct LiDAR-Inertial Odometry (DLIO)

- Year: 2022 preprint / ICRA 2023.
- Proposed method: lightweight LIO with coarse-to-fine continuous-time motion correction, direct scan-to-map registration, and a nonlinear geometric observer.
- Datasets: Newer College and self-collected UCLA campus datasets.
- Compared baselines: DLO, CT-ICP, LIO-SAM, FAST-LIO2.
- Classic baselines: no classic fixed-map localization baseline.
- Contemporary baselines: all four named systems.
- Metrics: trajectory RMSE and average per-scan processing time; map detail is also inspected.
- Code public: YES, https://github.com/vectr-ucla/direct_lidar_inertial_odometry
- Relevance: mature efficiency/deskewing anchor and supplementary robustness comparison.
- Paper: https://arxiv.org/abs/2203.03749

### 4. Point-LIO: Robust High-Bandwidth LiDAR-Inertial Odometry

- Year: 2023.
- Proposed method: point-by-point LiDAR/IMU state updates at true point times with a stochastic-process-augmented kinematic model; very high-rate output.
- Datasets: 12 public benchmark sequences plus aggressive-motion, vibration, UAV, and solid-state-LiDAR experiments.
- Compared baselines: the paper positions FAST-LIO2/other LIO counterparts as accuracy and efficiency references; exact tables must be rechecked before final manuscript transcription.
- Classic baselines: none task-identical.
- Contemporary baselines: FAST-LIO2-class LIO systems.
- Metrics: odometry accuracy, processing cost, output bandwidth, motion-distortion behavior, and robustness under saturation/aggressive motion.
- Code public: YES, https://github.com/hku-mars/Point-LIO
- Relevance: supplementary high-bandwidth LIO; not a prior-map localization baseline.
- Paper: https://doi.org/10.1002/aisy.202200459

### 5. COIN-LIO: Complementary Intensity-Augmented LiDAR Inertial Odometry

- Year: 2023 preprint / ICRA 2024.
- Proposed method: geometry plus LiDAR-intensity photometric residuals; selects intensity patches that complement geometrically weak directions and fuses them in an iterated EKF.
- Datasets: ENWIDE and the public sequences used in its LIO evaluation.
- Compared baselines: KISS-ICP, MD-SLAM, the Du-and-Beltrame intensity method, LIO-SAM, FAST-LIO2, and RI-LIO in the ENWIDE evaluation.
- Classic baselines: KISS-ICP is a simple registration/odometry anchor; LIO-SAM is mature.
- Contemporary baselines: MD-SLAM, FAST-LIO2, RI-LIO and the intensity-based method.
- Metrics: ATE RMSE, relative translation error, completion/robustness, and runtime.
- Code public: YES, https://github.com/ethz-asl/COIN-LIO
- Relevance: strong reference for complementary sensing in geometric degeneracy, but it is not fixed-prior-map NDT localization.
- Paper: https://arxiv.org/abs/2310.01235

### 6. Uncertainty-aware LiDAR-based Localization for Outdoor Mobile Robots

- Year: 2024.
- Proposed method: estimates a probability distribution over map-matching poses from an NDT map, derives uncertainty-dependent weights, and fuses map matching, LIO, and GNSS in a factor graph.
- Datasets: three outdoor university-campus scenarios with changing/dynamic environments.
- Compared baselines: three LiDAR-based localization variants reported by the paper. Exact method names must be transcribed from the full experimental tables before implementation; the accessible official article confirms the count and protocol but no official repository was found.
- Classic baselines: fixed-weight/map-matching localization variants.
- Contemporary baselines: uncertainty-aware localization variants in the paper.
- Metrics: localization trajectory/position performance under inaccurate map matches across the three scenarios.
- Code public: **NOT FOUND** in author/project searches as of 2026-09-22. Do not substitute an unofficial implementation.
- Relevance: closest published method-level comparator to pose-distribution-based NDT uncertainty.
- Paper: https://doi.org/10.1002/rob.22392

### 7. Probabilistic Degeneracy Detection for Point-to-Plane Error Minimization (DRPM)

- Year: 2024.
- Proposed method: propagates point/normal noise into the Hessian, estimates the probability that each direction is non-degenerate, and smoothly attenuates point-to-plane ICP updates.
- Datasets: Rümlang, Seemühle mine, RelyOn, and the Fyllingsdalen bicycle tunnel.
- Compared baselines: Zhang-Kaess-Singh 2016 eigenvalue/solution remapping, degeneracy-aware factors, and Switch-SLAM.
- Classic baselines: Zhang et al. 2016 and degeneracy-aware factors.
- Contemporary baselines: Switch-SLAM.
- Metrics: APE/RPE where GT exists, qualitative drift/map consistency, detection/mitigation behavior, and runtime overhead.
- Code public: YES, https://github.com/ntnu-arl/drpm
- Relevance: strong method-level probabilistic degeneracy baseline for point-to-plane registration; not NDT and not a complete fixed-map system.
- Paper: https://arxiv.org/abs/2410.10784

### 8. SuperLoc: The Key to Robust LiDAR-Inertial Localization Lies in Predicting Alignment Risks

- Year: 2024 preprint / ICRA 2025.
- Proposed method: predicts alignment risk from raw measurements before optimization, estimates observability, and can inject external pose priors in map-based LiDAR-inertial localization.
- Datasets: eight released challenging scenarios spanning caves, corridors, and multi-floor/flat settings; official releases include GT maps, GT trajectories, calibration, and initial poses.
- Compared baselines: hdl_localization and FASTLIO Localization in the main degraded-environment experiments.
- Classic/mature baselines: hdl_localization and FASTLIO Localization.
- Contemporary baselines: proposed SuperLoc; related analysis discusses Hessian/covariance methods.
- Metrics: map inlier/outlier accuracy against millimeter-surveyed maps, trajectory robustness/completion, and alignment-risk behavior.
- Code public: YES, https://github.com/superxslam/SuperOdom (official project link; current default work is ROS 2 oriented).
- Relevance: strongest task-aligned recent system reference and direct evidence that hdl_localization plus FASTLIO Localization are reviewer-recognizable mature baselines.
- Paper: https://arxiv.org/abs/2412.02901
- Project/data: https://superodometry.com/superloc.html

### 9. GenZ-ICP: Generalizable and Degeneracy-Robust LiDAR Odometry Using Adaptive Weighting

- Year: 2024 preprint.
- Proposed method: adapts the balance between point-to-point and point-to-plane residuals using local geometry to avoid corridor ill-conditioning.
- Datasets: Newer College, MulRan, KITTI, HILTI-Oxford Exp07, Ground-Challenge Corridor1/2, and SubT-MRS Long_Corridor.
- Compared baselines: KISS-ICP, CT-ICP, DLO, point-to-point ICP, point-to-plane ICP, Zhang et al. 2016, X-ICP; broader tables include GICP/VGICP, MULLS, F-LOAM, MAD-ICP, SuMa and IMLS-SLAM.
- Classic baselines: point-to-point ICP, point-to-plane ICP, GICP, Zhang et al. 2016.
- Contemporary baselines: KISS-ICP, CT-ICP, DLO, X-ICP, MAD-ICP.
- Metrics: translational APE/RPE, HILTI score, divergence/completion, Hessian condition number, and runtime.
- Code public: YES, https://github.com/cocel-postech/genz-icp
- Relevance: strong robust-registration comparator; not a prior-map NDT localization system.
- Paper: https://arxiv.org/abs/2411.06766

### 10. GEODE: Heterogeneous LiDAR Dataset for Benchmarking Robust Localization in Diverse Degenerate Scenarios

- Year: 2024 preprint / IJRR 2025.
- Proposed contribution: 64 trajectories over more than 64 km, heterogeneous LiDARs and platforms, and seven classes of real geometric degeneracy.
- Datasets/scenes: shield/mine tunnels, flat surfaces, stairs, bridges, urban tunnels, waterways, and off-road runs; GT trajectories for all and selected surveyed GT maps.
- Compared baselines: official benchmark includes FAST-LIO2, DLIO, COIN-LIO, LIO-SAM, FAST-LIVO, LVI-SAM, R3LIVE, and Coco-LIC.
- Classic/mature baselines: LIO-SAM and FAST-LIO2.
- Contemporary baselines: DLIO, COIN-LIO and multimodal systems.
- Metrics: ATE/RTE-style localization statistics, completion/failure, and per-sequence benchmark reports.
- Code/data public: YES, https://thisparticle.github.io/geode/
- Relevance: high-value degenerate-scene benchmark; not itself a competing localization algorithm.
- Paper: https://arxiv.org/abs/2409.04961

### 11. DCReg: Decoupled Characterization for Efficient Degenerate LiDAR Registration

- Year: 2025 preprint.
- Proposed method: Schur-complement decomposition separates translation and rotation Hessian subspaces, maps weak eigendirections to physical motion directions, and uses a targeted preconditioner/PCG mitigation.
- Datasets: FusionPortable, GEODE, SubT-MRS, and self-collected corridor/building/stair/parking/cave data reported by the manuscript.
- Compared baselines: ME, FCN, CN for detection; SR, Tikhonov and TSVD for mitigation; combined ME-SR, FCN-SR, ME-TSVD, ME-TReg, X-ICP and Open3D, with SuperLoc appearing in system results.
- Classic baselines: minimum-eigenvalue/condition-number tests, solution remapping, Tikhonov, TSVD, Open3D registration.
- Contemporary baselines: X-ICP and SuperLoc.
- Metrics: ATE, registration accuracy, Chamfer distance, degeneracy ratio/detection behavior, and runtime.
- Code public: YES, https://github.com/JokerJohn/DCReg
- Relevance: primary direction-characterization method-level comparator; its Schur decomposition addresses local ill-conditioning, not multi-modal NDT ambiguity.
- Paper: https://arxiv.org/abs/2509.06285

### 12. M3DGR / Ground-Fusion++ benchmark

- Year: 2025.
- Proposed contribution: sensor-rich ground-robot dataset with systematic visual, LiDAR, wheel, and GNSS degradation plus a modular resilient fusion system.
- Datasets: 32 released/announced sequences including Corridor01/02, elevator, dynamic, occlusion, illumination, wheel-slip, and outdoor runs.
- Compared baselines: more than 40 VO/VIO/LO/LIO/LVIO systems; relevant LIO entries include LIO-SAM, FAST-LIO2, Point-LIO and DLIO.
- Classic baselines: LOAM/LeGO-LOAM and established factor-graph/filter LIOs.
- Contemporary baselines: Point-LIO, DLIO, FAST-LIVO2 and Ground-Fusion++.
- Metrics: translation/rotation errors, tracking/completion behavior, and scenario-specific robustness.
- Code/data public: YES, https://github.com/sjtuyinjie/M3DGR
- Relevance: independent evidence for the recurring mature anchor set and a useful corridor sequence with ArUco GT.
- Paper: https://arxiv.org/abs/2507.08364

## Frequency observations

The counts below refer only to explicit experimental comparison or official benchmark tables in the 12 audited works; mentions in related work are excluded. A paper can contribute more than one baseline.

| Baseline family | Explicit appearances | Interpretation |
|---|---:|---|
| FAST-LIO / FAST-LIO2 / FASTLIO Localization | 7 | Most recurrent filter/direct-map family; system task differs between odometry and fixed-map localization |
| LIO-SAM | 5 | Most recurrent mature factor-graph LIO anchor |
| ICP family (point-to-point, point-to-plane, GICP/VGICP, CT-ICP, KISS-ICP) | 5 | Essential method-level registration context in degeneracy papers |
| hdl_localization | 1 direct paper comparison | Low frequency but unusually high task match; used by SuperLoc as a principal baseline |
| DCReg/Zhang-style Hessian or condition-number degeneracy handling | 3 | Core method-level comparator family |
| DLIO | 3 | Useful recent LIO/registration robustness anchor |
| Point-LIO | 1 benchmark appearance plus its own paper | Supplementary high-bandwidth LIO, not main fixed-map comparator |
| NDT fixed-prior-map localization | 1 dedicated uncertainty paper plus project baselines | Underrepresented in recent degeneracy literature, which strengthens the need for a task-correct main table |
| FAST_LIO_LOCALIZATION | 1 direct paper comparison | High task match despite limited formal-paper frequency |

These counts are an audit aid, not a bibliometric claim; the survey is deliberately task-focused and not exhaustive.

## Recommended final comparison design

### Main prior-map localization table

1. Vanilla NDT: frozen system with mode analysis/reliability logic disabled.
2. Autoware classic NDT localizer.
3. hdl_localization.
4. FAST_LIO_LOCALIZATION.
5. Ours: mode-structured NDT reliability.

Add ICP and GICP only if the same scan, same prior map, same initial pose, same preprocessing, and equivalent stopping criteria can be enforced.

### Method-level table

1. Hessian minimum-eigenvalue/condition-number detector plus solution remapping (Zhang et al. 2016 style).
2. Park-style NDT pose-distribution covariance/weighting, implemented from the paper only if no official code emerges.
3. DCReg-style decoupled Schur weak-direction characterization.
4. Ours: explicit NDT likelihood mode discovery and intra/inter-mode reliability decomposition.

DRPM is a useful additional point-to-plane probabilistic detector if adapter cost remains manageable.

### Strong recent and supplementary context

- Strong recent direct system: SuperLoc.
- Strong recent registration: GenZ-ICP or DCReg, selected according to the exact experiment.
- Mature LIO anchors: FAST-LIO2 and LIO-SAM.
- Optional supplementary anchors: DLIO and Point-LIO.

## Risks to address before experiments

- hdl_localization and Autoware NDT may fail on non-repetitive Livox scans without equivalent preprocessing; parameter adaptation must be disclosed rather than silently optimized per result.
- FAST_LIO_LOCALIZATION depends on Python 2.7 and Open3D 0.9; reproduce only in isolation.
- SuperLoc currently targets a different ROS/dependency generation than the host baseline; use a container and do not retrofit the host.
- DCReg, DRPM, and GenZ-ICP are method/registration comparators. Calling them full prior-map localization systems would be misleading.
- Public “GT” can be Mocap, total-station positions, surveyed maps, ArUco poses, RTK, or a reference estimator. Report the actual source and its dimensionality.
- Mode ambiguity experiments require a prebuilt map independent of the localization test trajectory, or a clearly documented train/test split.

## Primary sources

- https://arxiv.org/abs/2007.00258
- https://arxiv.org/abs/2107.06829
- https://arxiv.org/abs/2203.03749
- https://doi.org/10.1002/aisy.202200459
- https://arxiv.org/abs/2310.01235
- https://doi.org/10.1002/rob.22392
- https://arxiv.org/abs/2410.10784
- https://arxiv.org/abs/2412.02901
- https://arxiv.org/abs/2411.06766
- https://arxiv.org/abs/2409.04961
- https://arxiv.org/abs/2509.06285
- https://arxiv.org/abs/2507.08364
