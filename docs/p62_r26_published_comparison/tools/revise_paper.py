#!/usr/bin/env python3
"""Produce the full R26 paper from the frozen R25 manuscript, without editing it."""
from pathlib import Path
D=Path(__file__).resolve().parents[1];R=D.parents[1]
s=(R/'docs/p62_r25_results_to_paper_closure/PAPER_DRAFT.md').read_text()
s=s.replace('# Lightweight Prior-Map NDT Localization with Geometry-Aware Admission and Budgeted Recovery','# Gate-Aware Budgeted NDT Recovery for Prior-Map LiDAR–Inertial Localization',1)
a=s.index('*Complete first manuscript');b=s.index('## Abstract')
s=s[:a]+'''*Complete revised manuscript for author and scientific review, R26. Author names and affiliations are to be supplied by the research team. Published reference results and our controlled causal results use separate protocols; this manuscript makes no external state-of-the-art ranking claim.*

'''+s[b:]
s=s.replace('The results support a narrowly defined engineering mechanism, not a new optimizer, universal recovery guarantee, or independent physical accuracy claim.','Published Newer College odometry and prior-map localization results are reported separately because their full-sequence, alignment and initialization protocols differ. The results support a narrowly defined engineering mechanism, not a new optimizer, universal recovery guarantee, or independent physical accuracy claim.')
a=s.index('## 2. Related Work');b=s.index('## 3. Methodology')
s=s[:a]+'''## 2. Related Work

### 2.1. LiDAR–inertial odometry is a distinct evaluation task

FAST-LIO2 directly registers raw points within an iterated filtering architecture and maintains an incremental k-d tree [6]. DLIO combines continuous-time motion correction, a geometric observer, and direct scan-to-submap registration [11]. STEAM-LIO uses a Gaussian-process motion prior in continuous-time sliding-window estimation [12], while Singer-LIO examines treating inertial measurements as observations of the state [13]. These systems address estimation and mapping from an initially unknown environment. A map constructed incrementally from the estimated trajectory is different from an externally surveyed fixed prior.

These papers provide useful quantitative context on original Newer College Short. However, their trajectory-wide, usually aligned translation ATE is not the same experiment as our reference-initialized, deliberately perturbed short-window map localization. FAST-LIO2's 0.3775 m Short result in our published-results table was run by Chen et al. [11], rather than extracted from a Newer College table in the original FAST-LIO2 paper. Several later papers reuse it. Repeated citation of one measurement does not constitute independent replication.

Our integrated estimator reuses an existing custom IMU/EKF implementation, with real inertial propagation and delayed map observations. It is not identified as an unmodified FAST-LIO2 system. The FAST-LIVO2-derived map available for our robot assets likewise does not provide an independent physical reference for their own trajectory.

### 2.2. Fixed prior-map localization and the closest systems

2Fast-2Lamaa supports odometry, mapping and localization against global or topometric maps through continuous distance-field representations and continuous-time undistortion [10]. Its Newer College comparison explicitly separates Umeyama-aligned odometry ATE from direct map-frame localization RMSE. The latter is 0.128 m for the full 01_short sequence, making it a particularly relevant published localization reference. We acknowledge this result without comparing it as an equal-protocol score to our 60-scan perturbed-initialization experiment. Our implementation retains the PCL Gaussian NDT solver and existing EKF/OOSM; it neither introduces a continuous distance field nor claims a new global initializer. The difference studied here is whether a hard pre-alignment geometry veto prevents a fixed, budgeted retry from producing an admissible observation.

Koide et al. tightly couple scan-to-scan, scan-to-prior-map and IMU factors in a sliding-window optimizer, including gravity-aided initialization [14]. That work addresses robustness at map boundaries and under severe geometric ambiguity by changing the estimation formulation. We instead keep the solver and filter model unchanged and study the observation path between them. FAST-LIO-LOCALIZATION is a relevant practical software framework combining LIO with periodic prior-map corrections [15]; its README is not treated as a standalone peer-reviewed Newer College accuracy result. Neither system is equated with our controlled causal ablation, and no unrun baseline result is inferred.

### 2.3. Localizability and observation reliability

X-ICP characterizes direction-dependent registration strength and constrains updates, while GenZ-ICP adapts geometric residual weighting [3,4]. They modify the registration behavior rather than simply controlling whether a frozen optimizer is attempted. We do not present our input covariance prescreening as their localizability analysis or as the true NDT Hessian.

SuperLoc predicts alignment risk before optimization, estimates observability, and actively incorporates supporting pose priors [5]. It is the closest conceptual comparison for pre-registration reliability. G addresses a much narrower issue: an inherited nearest-map distance/overlap predicate can prohibit all optimization when the current prediction is misaligned. Turning that particular predicate into a diagnostic does not invalidate SuperLoc's risk assessment or imply that pre-optimization geometry should always be ignored. Post-registration quality rejection remains active. The papers use different data and evaluations, so we make no accuracy claim against SuperLoc.

### 2.4. Contribution boundary and resource accounting

Spatially restricting targets, reliability gating and multiple initializations are established ideas. Our contribution is a reproducible system-design interaction and its causal ablation: map availability, initial geometry diagnostics, optimized-observation admission and inertial continuity must be assigned distinct responsibilities. G-only and C+G are compared using the same raw input and parameters, while each evolves its own actual filter state. Memory scope is also explicit: NDT-process high-water memory, simultaneous sampled two-node RSS and another paper's final component RAM are different measurements. This positioning supports an engineering study; it does not establish a new registration objective or superior general localization performance.

'''+s[b:]
needle='R19/R20 use frozen 50 m static ROIs'
pos=s.index(needle)
s=s[:pos]+'''The official original Stereo-Cam index describes the Short experiment as 1530 s with Ouster OS1 LiDAR and its internal IMU [16]. Published names `Short`, `01-Short` and `01_short` correspond to this original sequence: their reported length is about1609 m, and the public STEAM-LIO and Singer-LIO `01_short_experiment` outputs each contain15301 timestamps exactly matching the local raw PCD order. This establishes sequence identity, not identical evaluation conditions. Later Multi-Cam/OS0 sequences are not substituted.

'''+s[pos:]
needle='The windows and initialization conditions are fixed.'
pos=s.index(needle)
s=s[:pos]+'''The integrated W1 comparison has60 evaluation scans, raw2601–2660, after reference initialization at raw2600. C3 applies map-axis yaw+8deg to that initial state consistently across the estimator and registration; subsequent states are causal, not initialized from current reference poses. Raw measurements, initialization, calibration, map-selection parameters, Deskew model and NDT configuration are identical. Deskewed point coordinates may subsequently differ as an intended consequence of different filter histories. The actual target set remains unchanged in the decisive ablation.

For reproducibility of reference-relative metrics, each state is compared at its own recorded time: NDT at scan reference and EKF at its actual propagated watermark, which can be later than scan start. The frozen reference interpolation limits are150ms for a bracketing gap and50ms for a near endpoint; no time shift is fitted. Only anchor reference poses enter initialization. Test reference values are read after native outputs and admission decisions have been sealed. Selected historical receipts do not identify a complete CPU model for all internal resource runs; we report this as NR rather than retrospectively assigning today's workstation specification.

'''+s[pos:]
# Link to the unchanged historical figure assets from the new archive.
s=s.replace('](figures/','](../p62_r25_results_to_paper_closure/figures/')
# Describe the revised draft without implying a new R26 experimental run.
s=s.replace('### 5.5. New G-only ablation','### 5.5. Controlled G-only ablation')
s=s.replace('The new R25 run executes','The sealed R25 run executes')
s=s.replace('The local quadruped loop 2 assets are present:','The local quadruped loop2 assets are present:')
s=s.replace('Mid -360','Mid-360')
# Add the public table and separate resource context before Discussion.
pos=s.index('## 6. Discussion')
s=s[:pos]+'''### 5.7. Published Newer College results under their original protocols

**Table A. Published reference results, not a matched ranking against our method.** These translation-only results concern the original complete Short experiment, nominally1530s/1609m. NR means not reported at that scope. The current 2Fast author manuscript has Table7; source versions and pages are pinned in the supplementary extraction ledger.

|Method and original result source|Task|Position result m|Trajectory treatment|Cost at original reported scope|
|---|---|---:|---|---|
|FAST-LIO2, Chen2023 TableI [11]|Odometry ATE RMSE|0.3775|evo; exact original alignment flags NR; later compiled comparisons specify Umeyama|42.86ms, average of five NC sequences|
|DLIO continuous, Chen2023 TableI [11]|Odometry ATE RMSE|0.3606|evo; exact original alignment flags NR|35.74ms, same five-sequence scope|
|STEAM-LIO, Burnett2025 TableII [12]|Odometry ATE RMSE|0.3042|Umeyama SE(3) alignment|74ms NC average|
|Singer-LIO, Burnett2025 TableIII [13]|Odometry ATE RMSE|0.3020|Umeyama alignment|NR for this experiment|
|2Fast-2Lamaa, Le Gentil2026 Table7 [10]|Global-map odometry ATE RMSE|0.262|Umeyama alignment|Separate resource ablation; NC-specific timing NR|
|2Fast-2Lamaa, same Table7 [10]|Fixed prior-map localization RMSE|0.128|Direct map-frame position comparison|Separate resource ablation; NC-specific timing NR|

The first three figures are also printed as0.378,0.360 and0.304 in 2Fast Table7. That paper explicitly obtains FAST-LIO2/DLIO results from Chen et al. and STEAM-LIO results from Burnett et al.; they are not new 2Fast-author baseline executions. The original DLIO0.3606 and later printed0.360 are retained at their respective precision without inventing an explanation for the digit difference. Chen et al. exclude the first100 FAST-LIO2 poses on some sequences; the exact affected list is NR. Thus even published “full-sequence” labels do not guarantee identical timestamp subsets. No corresponding Short rotation RMSE is reported in these tables.

The full-sequence0.128m fixed-map localization result is especially relevant, but it is not a controlled match for our disturbed6s window: initialization, representation, body-frame convention, fusion timing and duration differ. Conversely, aligned odometry ATE removes a global trajectory transform that our map-localization metric does not fit. We therefore neither insert0.209943m into TableA nor claim that the published values rank the present method. SuperLoc and Koide et al. provide valuable prior-map comparisons on different data, and are discussed technically rather than assigned fabricated Newer College scores.

**Public-output reuse.** The official STEAM repository supplies Short trajectories labeled STEAM-LIO and Singer-LIO. Exact identity to the published table-run configuration is not certified. We checked their15301 finite TUM rows and exact timestamp agreement with our scan files, without using reference errors. They start at approximately zero position and identity orientation, and the result files do not themselves certify a Leica-map transform or the sensor-frame binding to our conditional Base interpretation. They also do not embody our+8deg window reset. No new window error is produced by fitting a transform. Our longest matched in-house resource segment has300 scans, so we also cannot manufacture a full1530s C+G score by concatenating independent windows. All public results are reused without compiling external software or rerunning native baselines.

### 5.8. Resource context from published implementations

**Table C. Selected public resource reports; hardware, component and dataset scopes differ.** Internal process/paired RSS and the P45 spatial-memory comparison are given separately in Sections5.1 and5.6.

|Published system|Timing / compute|Memory|Platform and measurement scope|
|---|---|---|---|
|FAST-LIO2 as run by Chen [11]|42.86ms per scan|NR|i7-11800H, described as16-core; five NC sequences average|
|DLIO continuous [11]|35.74ms per scan|NR|Same Chen platform/scope|
|STEAM-LIO [12]|74ms per scan|NR|XeonE5-2698v4,16threads; NC average|
|Singer-LIO [13]|NR at NC scope|NR|NC hardware profile NR|
|2Fast baseline localization [10]|8.86% total CPU; mean component core use1.06/0.71|Final component RAM0.17/0.32GB|i7-1370p,32GB installed; Boreas-RT ablation, not NC|
|2Fast global-map localization [10]|9.16% total CPU; mean component core use1.05/0.78|Final component RAM0.16/2.24GB|Same platform; Table12 resource profile, not the NC Table7 run|
|SuperLoc [5]|45ms per frame,22FPS|NR|Ryzen73700X8-Core; different datasets and pipeline|

For a typical Suburbs sequence, 2Fast reports41ms undistortion/odometry,61ms keyframe registration and54ms map update, or19ms without carving [10]. These are different modules and cadences; summing or comparing them as a common per-LiDAR-frame stopwatch would be misleading. Similarly, its final RAM per component is not the same statistic as our NDT HWM or simultaneously sampled two-node RSS. Different hardware, point budgets, map sizes and fusion paths preclude a defensible inter-paper speedup or memory ratio. The only82.2% reduction claimed here belongs to the historical matched P45 global/local-target comparison, with its disclosed tradeoffs.

'''+s[pos:]
# Rename the retained within-system table for an explicit three-table separation.
s=s.replace('|W1/C3 method|Translation RMSE/P95/Max m|','**Table B. Controlled causal ablation, complete W1/C3 window.**\n\n|W1/C3 method|Translation RMSE/P95/Max m|',1)
# Strengthen the novelty and submission boundary, not SOTA claims.
a=s.index('### 6.2. Scope of originality');b=s.index('### 6.3. Remaining failures',a)
s=s[:a]+'''### 6.2. Scope of originality and external comparison

We do not claim a new NDT objective, degeneracy detector, Schur-complement solver or invention of multistart search. 2Fast-2Lamaa already demonstrates efficient prior-map localization with a new continuous distance-field representation; SuperLoc already assesses alignment risk and adapts supporting fusion; Koide et al. already tightly couple prior-map and inertial constraints [5,10,14]. Our smallest concrete distinction is an attempt/admission responsibility change around a frozen native NDT/filter pipeline, evaluated by an independent causal G-only ablation. This is narrower than a novel general localization framework.

The evidence supports retaining C+G as a useful engineering baseline and a system-design contribution candidate. It does not establish methodological novelty sufficient for a particular journal. No directly comparable external disturbed-initialization result was established, and TableA cannot fill that gap. Any claim to outperform contemporary prior-map localization would require an explicitly matched protocol, not a percentage calculated between a short conditional map-relative RMSE and full-sequence aligned odometry ATE. The present manuscript intentionally makes the more limited claim and leaves venue suitability to scientific and editorial review.

'''+s[b:]
s=s.replace('Establishing sufficient novelty for a journal still requires editorial assessment against the closest systems.','Establishing sufficient novelty for a journal still requires editorial assessment against the closest systems.')
# Bibliographic updates and new peer-reviewed sources.
s=s.replace('ICRA 2025; arXiv:2412.02901.','ICRA2025, pp14080–14086. DOI10.1109/ICRA55743.2025.11127720; arXiv:2412.02901.')
s=s.replace('*FAST-LIO2: Fast Direct LiDAR-inertial Odometry*. arXiv:2107.06829,2021.','*FAST-LIO2: Fast Direct LiDAR-Inertial Odometry*. IEEE T-RO38(4), pp2053–2073,2022. DOI10.1109/TRO.2022.3141876; arXiv:2107.06829.')
s=s.replace('10.1109/IROS 45743.2020.9340849','10.1109/IROS45743.2020.9340849')
pos=s.index('## Reproducibility appendix')
s=s[:pos]+'''10. C. Le Gentil, R. Falque, D. Lisus, and T. D. Barfoot. *2Fast-2Lamaa: Large-scale lidar-inertial localization and mapping with continuous distance fields*. IJRR,2026, OnlineFirst. DOI10.1177/02783649261451003. [Publisher](https://doi.org/10.1177/02783649261451003); [public author manuscript v3](https://arxiv.org/pdf/2410.05433v3). Numeric inspection: Table7 p18; resource Table12/§9.7 p21. Full publisher PDF was not retrieved; author version is identified explicitly.
11. K. Chen, R. Nemiroff, and B. T. Lopez. *Direct LiDAR-Inertial Odometry: Lightweight LIO with Continuous-Time Motion Correction*. ICRA2023, pp3983–3989. DOI10.1109/ICRA48891.2023.10160508. [Paper](https://arxiv.org/abs/2203.03749); [official code](https://github.com/vectr-ucla/direct_lidar_inertial_odometry). Inspected PDF TableI p5; §IV-B1 p6.
12. K. Burnett, A. P. Schoellig, and T. D. Barfoot. *Continuous-Time Radar-Inertial and Lidar-Inertial Odometry Using a Gaussian Process Motion Prior*. IEEE T-RO41, pp1059–1076,2025; online25December2024. DOI10.1109/TRO.2024.3521856. [Author PDF](https://www.dynsyslab.org/wp-content/papercite-data/pdf/burnett-tro24.pdf). Inspected author version TableII p10,§VI-B p11.
13. K. Burnett, A. P. Schoellig, and T. D. Barfoot. *IMU as an input versus a measurement of the state in inertial-aided state estimation*. Robotica43, pp680–700,2025. DOI10.1017/S0263574724002121. [Publisher](https://doi.org/10.1017/S0263574724002121). TableIII printedp698, PDFp19. STEAM/Singer [official code and outputs](https://github.com/utiasASRL/steam_icp).
14. K. Koide, S. Oishi, M. Yokozuka, and A. Banno. *Tightly Coupled Range Inertial Localization on a 3D Prior Map Based on Sliding Window Factor Graph Optimization*. ICRA2024, pp1745–1751. DOI10.1109/ICRA57147.2024.10611195. [Author PDF](https://staff.aist.go.jp/k.koide/assets/pdf/icra2024_02.pdf); [official project](https://staff.aist.go.jp/k.koide/projects/icra2024_gl/). TableI/indoor setup PDFp5.
15. HViktorTsoi. *FAST-LIO-LOCALIZATION*. [Official software repository](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION), accessed2026-10-24. Software, not a separately verified peer-reviewed paper.
16. Oxford Robotics Institute. *Newer College Dataset—Stereo-Cam*. [Official sequence and sensor index](https://ori-drs.github.io/newer-college-dataset/stereo-cam/), accessed2026-10-24.

'''+s[pos:]
s=s.replace('and the present R25 directory.','and `docs/p62_r25_results_to_paper_closure/`.')
s=s.replace("No new experiment is implied by the draft's completion.","R26 adds a primary-source extraction ledger, source hashes/version locators, public-trajectory availability/time checks, comparability grades and this complete revised paper in `docs/p62_r26_published_comparison/`. Seven published papers and one software project are distinguished. It performs zero new NDT calls, external reproductions or dataset downloads. No new experiment is implied by the manuscript's revision.")
(D/'PAPER_FINAL_DRAFT.md').write_text(s)
print('complete manuscript words',len(s.split()),'lines',len(s.splitlines()))
