# Comparability audit and reproduction decision

## Decision

**Published context is sufficient to revise the paper; it is insufficient to claim superior accuracy to an external method.** No new external run is required to repeat known numbers. Zero external GradeA accuracy results were established. Six unique published Newer College method/task rows are GradeB. SuperLoc, the Koide2024 experiments and the FAST-LIO-LOCALIZATION demo are GradeC context. The original/G-only/C+G rows are directly comparable **internal** causal ablations, not evidence of external state of the art.

Grades are relative to our principal W1/C3 experiment, not labels of paper quality. A requires matching sequence/time, task, reference/body frames, alignment, initialization and evaluable output. B shares the dataset but violates at least one of those requirements. C has different data/tasks or no corresponding experiment.

|Comparison target|Shared facts|Blocking protocol differences|Grade / allowed statement|
|---|---|---|---|
|FAST-LIO2 from DLIO TableI|Original NC Short, physical Ouster+IMU|Odometry/online map; nominal1530s versus6s; different initialization; exact evo flags NR; possible first100 exclusions; no+8deg window reset|B: cite published ATE, no accuracy rank against C+G|
|DLIO|Same dataset and sensors|Continuous-time LIO/GICP, full trajectory, not fixed-map observation admission; exact original alignment command NR|B|
|STEAM-LIO / Singer-LIO|Same dataset; public15301time epochs exactly match local PCDs|Full-sequence odometry; Umeyama-aligned ATE; no certified map-from-output transform/body-frame identity; no matching disturbed anchor experiment|B; raw file availability does not imply direct map-relative comparability|
|2Fast odometry|Short1609m/1530s|Online global-map construction, alignment, full duration, different initialization and feature/continuous-field objective|B|
|2Fast localization|Fixed surveyed prior and original Short; direct map-frame position RMSE|Full trajectory; exact NC init matrix and body convention binding NR; different map representation/preprocessing; no matched+8deg reset and true custom EKF/OOSM output times|B; closest quantitative task, still not an equal-protocol trial|
|SuperLoc|Prior-map reliability/admission problem, IMU/other odometry fusion|Different cave/floor/corridor data; map outlier fractions versus our fused-pose RMSE; separate SubT odometry evaluation|C; technical contrast only|
|Koide2024|Fixed prior map and inertial localization|Azure Kinect/own outdoor experiments; batch-generated reference; different factors/initialization; published implementation closed-source|C|
|FAST-LIO-LOCALIZATION README|Prior-map correction over LIO|Software demo, no verified Short result or standalone2022–2026 paper; correction rate not full estimator timing|C|
|Original vs G-only vs C+G|Same raw scans/map/initial state/parameters/IMU-EKF, independent causal histories|Single exposed short window, sequential historical execution, no repeated timing trials; filter histories intentionally diverge|A-like internal causal ablation only; report joint mechanism benefit and its scope|

## Sequence identity is established; protocol identity is not

The official Stereo-Cam table describes sequence1/short experiment,1530s. 2Fast Table7 gives01_short1609m/1530s, DLIO TableI Short1609.40m, and official STEAM output paths name01_short_experiment. For STEAM and Singer, all15301 times match the local PCD order, not just a similar duration. This is much stronger than matching the word “Short,” but does not establish the pose axis or algorithm protocol. The original OS1 release is not merged with the later OS0 Multi-Cam extension.

## Reuse of public trajectories

Two author-published files were retrieved from pinned STEAM repository commit819c6077effc9f9597a8ad6a1318091d6be7b8a1. The method labels are taken from the repository directories; an exact binding of each file to a particular published table-run binary/configuration is not certified. They contain valid finite TUM rows, near-unit quaternions and exact integer-nanosecond time correspondence. First pose is approximately the origin and identity. A TUM file does not specify whether its transform refers to IMU, LiDAR, body or camera, nor a certified transform into Leica map coordinates. A whole-sequence Umeyama fit would reproduce an odometry metric, not our no-fit map-localization criterion. An ad hoc W1 anchor alignment would also change the experiment and still would not simulate a+8deg disturbed-initialization run. No such fitted score is calculated.

`PUBLIC_TRAJECTORY_ASSESSMENT.json` records this successful availability/time check and unresolved frame/initialization contract. Do not report that public trajectories do not exist. For 2Fast/DLIO, a bounded README/project inspection did not establish a corresponding downloadable, certified map-frame localization output. The GitHub API returned rate limits; this is recorded, not converted into a claim of universal absence.

## Can our existing longer outputs match the publication?

No. P45 has a matched300-scan segment, and the principal causal ablation60scans. R19/R20/R21/R22 windows reset state independently and some use pose-history predictors instead of real IMU. Concatenating them is not a1530s continuous run. Re-aligning each method/window would violate the frozen map-frame protocol. There is no sealed complete C+G Short trajectory that can be honestly inserted into the full-sequence published ATE table.

## Decision on an external reproduction

The public numbers needed for RelatedWork/TableA already exist. A new matched prior-map experiment would address a different question: whether C+G is competitive under identical disturbed initialization and fusion/measurement timing. That is useful for a future comparative benchmark but is not needed to repeat the public paper's results or support the narrow internal ablation. It requires an explicit fixed protocol, valid frame binding and initialization treatment. This task therefore executes **zero external reproductions**, with no algorithm source clone/build and no new NDT calls. We do not fill the missing GradeA row with incompatible numbers.

## Claims allowed and prohibited

Allowed: C+G recovers the archived W1/C3 development failure under unchanged post-quality rules; G-only does not. P45 records82.2% lower NDT-process HWM in its own matched segment, with disclosed tradeoffs. Published NC papers provide a strong external context and establish that prior-map localization is already studied.

Not supported: outperforming FAST-LIO2/DLIO/STEAM/Singer/2Fast/SuperLoc; universal82.2% total-system savings; independent physical accuracy; a new registration objective or invented multistart principle;10Hz closed-loop certification; robot deployment; independent C+G holdout validation. A0.128m full-sequence published number versus0.209943m short disturbed-window number supports neither superiority nor inferiority without a shared protocol.
