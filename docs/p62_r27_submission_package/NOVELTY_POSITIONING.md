# Novelty positioning for advisor and editor review

The paper is a system-design and causal-ablation contribution candidate. It does not establish a novel optimizer or state-of-the-art localization accuracy. All component names below refer to verified primary papers or official software documentation.

|Closest method|What already exists|Retained mature components here|Specific experimental contribution and limit|
|---|---|---|---|
|2Fast-2Lamaa, IJRR2026|Continuous-time undistortion, GP distance-field registration, global/topometric prior-map localization, resource evaluation|Prior maps, inertial support, spatial targeting, scan-to-map optimization|We keep Gaussian PCL NDT and an existing delayed EKF; show that one hard pre-geometry veto prevents a bounded retry path from being reached. No new continuous representation or matched external accuracy advantage|
|SuperLoc, ICRA2025|Pre-optimization risk/localizability assessment and active support from motion priors|Reliability assessment before/during registration and inertial continuity|Our result concerns one inherited nearest-map distance/overlap veto, not all predictive-risk systems. G-only's failure plus C+G recovery isolates a responsibility-boundary interaction; it does not refute SuperLoc|
|Koide2024 prior-map localization|Tightly coupled scan-to-scan/map+IMU sliding-window factors, gravity-aided initialization|Map observations and inertial fusion|We change attempt/admission control while keeping a separate NDT measurement and unchanged EKF/OOSM. No stronger-fusion or unknown-initial-pose global-localization claim|
|Autoware NDT Scan Matcher|Quality/iteration/support diagnostics, prediction interpolation, pose checks, Monte Carlo initial localization and optional multi-initialization covariance estimation|NDT quality admission, initial-pose dependence, use of multiple starts|We do not claim the first NDT gate or multiple-initialization implementation. The evidence is an actual closed-loop ablation of one pre-veto and fixed event-only yaw retries, under unchanged post-quality conditions and a declared run quota|

Sources: R26 primary-paper ledger, plus the [official current Autoware documentation](https://autowarefoundation.github.io/autoware_core/main/localization/autoware_ndt_scan_matcher/) inspected2026-10-24. Its45KB README was read, not compiled or run. `OFFICIAL_SOURCE_RECEIPTS.json` records the fetched hash. Current software features are not retroactively treated as published2024 experimental results.

## Maximum defensible claim

A reproducible gate-placement change makes an existing budgeted recovery baseline reachable. A controlled actual IMU/EKF comparison shows that merely allowing nominal NDT to run does not recover the observed trajectory, while an admissible restart changes the filter history and substantially reduces complete-window reference-relative error. Process-memory accounting contributes a separate implementation trade-off study.

This is not a proof that geometry gates are inherently wrong, that C+G is universally robust, or that pairing familiar components constitutes a new optimization theory. The successful window is exposed development data, the reference is map anchored, the pose-frame convention remains conditional, and no external method was evaluated under a matched disturbed-initialization protocol.

## Negative evidence that must remain visible

- All60 G-only alignments converge but all fail post-quality admission.
- The C+G initial maximum rotation remains11.245245°; two outputs exceed the error criterion.
- Relative-only admission has a normal-initialization counterexample and is excluded.
- No later NDT query consumed a replaced corrected-history sample in the tested freshness extension; no benefit is claimed.
- P45's82.2% is per-process high-water memory on a historical matched segment, with slower steady alignment and worse error tails.
- The robot experiment is unperformed because physical LiDAR–IMU calibration provenance is unresolved.

## Editorial judgment

The manuscript is ready to be reviewed by the advisor as a bounded sensing-system integration study. Whether this constitutes enough advance for IEEE Sensors Journal is not established by the executor. Measurement Science and Technology explicitly expects an advance in measurement technique and consideration of uncertainty; it is an alternative, not an easier acceptance guarantee. The conditional frame/reference and narrow C+G validation are genuine evidential weaknesses. No algorithm naming or different-protocol ranking resolves them.
