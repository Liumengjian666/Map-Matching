# Published baseline results: primary-source extraction

R26, 2026-10-24. Seven peer-reviewed works and one software project were examined. This is literature extraction, not a new benchmark. No external estimator was cloned, built, or run. NR means the inspected primary source does not report the requested quantity at the stated scope; it is not an estimate of zero.

## Verified Newer College results

|Source/result ID|Method|Sequence/range|Task and metric|Published value m|Original locator|Who produced this number?|
|---|---|---|---|---:|---|---|
|D1|FAST-LIO2|Original Short, nominal full sequence; some sequences exclude first100 poses, affected list NR|Odometry position ATE RMSE|0.3775|Chen et al. Table I, PDF p5; exclusion in §IV-B1 p6|Chen et al. ran the baseline; not a Newer College table in Xu et al.'s original FAST-LIO2 paper|
|D2|DLIO, continuous correction|Short,1609.40m|Odometry position ATE RMSE|0.3606|Chen Table I, PDF p5|Own proposed-method run|
|S1|STEAM-LIO|01-Short, full trajectory|Odometry position ATE RMSE after Umeyama|0.3042|Burnett T-RO Table II, author PDF p10; attribution/alignment p11|Own STEAM run|
|I1|Singer-LIO|01-Short, full trajectory|Odometry position ATE RMSE after Umeyama|0.3020|Burnett Robotica Table III, PDF p19, printed p698|Own Singer run|
|F1|FAST-LIO2|01_short, nominal1609m/1530s|Odometry ATE, Umeyama protocol stated by reporting paper|0.378|Le Gentil IJRR Table7; author v3 PDF p18|Explicitly reused from Chen2023, not re-run by Le Gentil et al.|
|F2|DLIO|Same|Same|0.360|Same|Explicitly reused from Chen2023. Original is0.3606; preserve each paper's printed precision, do not silently harmonize|
|F3|STEAM-LIO|Same|Same|0.304|Same|Explicitly reused from Burnett T-RO2025; original0.3042|
|F4|2Fast-2Lamaa, global map mode|Same|Odometry ATE after Umeyama; map built incrementally|0.262|Table7, author v3 PDF p18; §9.4.2–3 pp16–17|Own 2Fast odometry run|
|F5|2Fast-2Lamaa, global map mode|Same|Prior-map localization position RMSE, direct map-frame comparison|0.128|Table7 last row and §9.4.4, author v3 PDF p18|Own localization run against provided survey map; separate task from F4|

All values above are translational. Corresponding sequence-specific rotational RMSEs are **NR** in these tables. A full-sequence translation ATE does not imply a rotation result. The five numbers specifically requested by the controller, 0.378/0.360/0.304/0.262/0.128m, have been found in the actual 2Fast author v3 Table7. The publisher-indexed body independently identifies Table7 and the separation of odometry/localization; direct full publisher retrieval returned403/timeouts. Accordingly, numeric/page verification is explicitly tied to the public author manuscript, not falsely described as a downloaded version-of-record PDF.

Primary links: [2Fast IJRR/DOI](https://doi.org/10.1177/02783649261451003), [2Fast author v3](https://arxiv.org/pdf/2410.05433v3), [DLIO paper](https://arxiv.org/pdf/2203.03749), [STEAM author PDF](https://www.dynsyslab.org/wp-content/papercite-data/pdf/burnett-tro24.pdf), [Singer publisher](https://doi.org/10.1017/S0263574724002121).

## Version and attribution corrections

- The downloaded 2Fast v2 has the older NC table numbered6 and is not used for the current comparison. The v3 author manuscript, September2026, carries Table7 and the requested0.262 odometry value. IJRR first-online publication is June2026; this archive does not claim byte-identical publisher and v3 versions. Table/page citations consistently identify the inspected version.
- STEAM-LIO is T-RO volume41,2025, pp1059–1076, first online25December2024, DOI10.1109/TRO.2024.3521856. Calling it only a2024 preprint loses its published status. The inspected author PDF identifies arXiv2402.06174v2,20November2024.
- FAST-LIO2 is T-RO2022, not merely its2021 preprint. The cited NC score belongs to a later Chen et al. benchmark. STEAM, Singer, and 2Fast copying the same baseline is not four independent replications.
- STEAM TableII labels FAST-LIO2 and KISS-ICP as obtained from Chen; the paragraph on p11 distinguishes these copied results. Singer TableIII explicitly marks FAST-LIO2 as obtained from Chen and repeats the original DLIO score. No new Singer-author DLIO rerun is inferred.
- DLIO TableI reports0.3606, whereas 2Fast Table7 prints0.360. This is retained as a source-precision discrepancy, without guessing why the digits differ.

## Sequence identity

The [official Stereo-Cam index](https://ori-drs.github.io/newer-college-dataset/stereo-cam/) names experiment1 “short experiment,”1530s, with Ouster OS1 and its internal IMU. 2Fast Table7 names01_short,1609m/1530s and cites the original Ramezani dataset; DLIO namesShort1609.40m. Official STEAM repository result filenames explicitly name `01_short_experiment_tum.txt`. The two inspected public trajectories each contain15301 epochs, all matching the local `01_short_experiment` PCD filenames exactly in integer nanoseconds, including endpoints1583836591182590976 and1583838121156355584. This substantiates the aliases for this original release; it does not equate later OS0/Multi-Cam sequences with the original OS1 data.

## Conditions that travel with these numbers

DLIO uses Ouster10Hz LiDAR/internal100Hz IMU. TableI timing is averaged over five NC sequences, not a Short-only timing. Its §IV reports default algorithm configurations except specified changes, including disabling FAST-LIO2 online extrinsic estimation, enabling LIO-SAM loop closure, and slowing CT-ICP playback/increasing voxelization. Some FAST-LIO2 initial100 poses are omitted; no exact affected-sequence list or full evo alignment command is supplied in the inspected body. Thus its “full sequence” label is nominal, not a guarantee of identical sample sets across methods.

STEAM and Singer explicitly use Umeyama alignment before translation ATE. They estimate odometry with an online map. This differs from our externally anchored prior-map output without a fitted test-trajectory transform. 2Fast localization directly compares positions in the supplied map frame, making it the closest published task; its complete1530s range, initialization, map representation, and evaluation/body-frame conventions still differ from our6s, deliberately perturbed IMU/EKF experiment. The paper itself cautions against strong localization conclusions from the precision of the NC reference (§9.4.2/4). No numerical localization superiority claim is licensed by these tables.

The [official reference documentation](https://ori-drs.github.io/newer-college-dataset/ground-truth/) describes ICP registration of Ouster scans to the Leica map and distinguishes RS_C1 for Stereo-Cam from Base for Multi-Cam. Our archived conditional Base interpretation is not certified by these external papers. No reference-frame uncertainty has been resolved merely by finding published low errors.

## Complete extraction records

`PUBLICATION_METADATA.json` and `.csv` contain23 named fields per entry, covering all20 requested information categories. They include title, every author, year/venue/DOI, primary PDF, official code, exact task/sensors/map source, initialization/alignment/reference, accuracy/timing/memory/hardware and table/page locators. Fields unavailable in the primary sources are NR. The following concise index identifies the remaining relevant sources.

|Work|Publication and DOI|Official primary/code source|What is usable here?|
|---|---|---|---|
|FAST-LIO2, Xu/Cai/He/Lin/Zhang|T-RO38(4),2053–2073,2022;10.1109/TRO.2022.3141876|[Paper](https://arxiv.org/abs/2107.06829), [code](https://github.com/hku-mars/FAST_LIO)|Direct iterated-filter LIO/ikd-Tree background; NC numbers attributed to Chen, not original FAST-LIO2 dataset table|
|DLIO, Chen/Nemiroff/Lopez|ICRA2023,3983–3989;10.1109/ICRA48891.2023.10160508|[Paper](https://arxiv.org/abs/2203.03749), [code](https://github.com/vectr-ucla/direct_lidar_inertial_odometry)|Original NC TableI and baseline attribution|
|STEAM-LIO, Burnett/Schoellig/Barfoot|T-RO41,1059–1076,2025;10.1109/TRO.2024.3521856|[Author PDF](https://www.dynsyslab.org/wp-content/papercite-data/pdf/burnett-tro24.pdf), [code](https://github.com/utiasASRL/steam_icp)|TableII and public raw trajectories|
|Singer-LIO, same authors|Robotica43,680–700,2025;10.1017/S0263574724002121|[Publisher](https://doi.org/10.1017/S0263574724002121), [code](https://github.com/utiasASRL/steam_icp)|TableIII and public raw trajectories; NC runtime/memory NR|
|SuperLoc, Zhao/Zhu/Gao/Kim/Qiu/Johnson/Scherer|ICRA2025,14080–14086;10.1109/ICRA55743.2025.11127720|[Paper](https://arxiv.org/abs/2412.02901), [project](https://superodometry.com/superloc), project code link to[SuperOdom](https://github.com/superxslam/SuperOdom)|Closest risk/admission context; no NC01_short result found. Localization map-outlier rates and separate SubT odometry ATE must not be pooled|
|Tightly Coupled Range Inertial Localization…, Koide/Oishi/Yokozuka/Banno|ICRA2024,1745–1751;10.1109/ICRA57147.2024.10611195|[Author PDF](https://staff.aist.go.jp/k.koide/assets/pdf/icra2024_02.pdf), [project](https://staff.aist.go.jp/k.koide/projects/icra2024_gl/)|Prior-map fusion competitor; different own indoor/outdoor data. Project explicitly says closed-source; no assumption that current GLIM is the identical implementation|
|FAST-LIO-LOCALIZATION|Software,2021 README news; standalone paper/DOI NR|[Official repository](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION)|Relevant architecture and later published baselines; not an independently verified2022–2026 paper or NC accuracy claim|

For SuperLoc, PDF TableIII p6 prints SubT average0.272 while nearby body text says0.271; neither is imported into the NC table. The target-survey precision is not an independent trajectory truth. For Koide TableI, the original reports ATE with± values; this extraction does not invent their aggregation meaning or treat them as our RMSE.

## Public trajectories and bounded reproduction decision

STEAM repository commit `819c6077effc9f9597a8ad6a1318091d6be7b8a1` provides two inspected Short files: STEAM-LIO and Singer inertial,15301 valid TUM rows each. Their method labels follow the official directory names; exact identity to the paper table-run binary/configuration is not certified. Only these small already-published output files were retrieved; no source clone/build or data packages. `PUBLIC_TRAJECTORY_ASSESSMENT.json` records exact timestamp correspondence, finite values, near-unit quaternions and hashes. Their first translations are near0 and orientations near identity. TUM schema does not establish the output sensor frame or the map-from-odometry transform. No certified transform to our conditional Base/Leica chain or matching+8deg-window initialization is available from these files. Hence **no new direct window RMSE is manufactured** and no fitted SE(3) alignment is performed.

The existing longer in-house segment has300 scans, not the full1530s trajectory; R19–R22 outputs also use short windows, historical predictors and different initialization. Concatenating those independent resets into a full-sequence ATE would be invalid. A new external run would therefore be a new benchmark experiment, not reproduction of a missing public number. It is not needed for the present modest within-system mechanism claim. R26 chooses **0 external algorithm reproductions**; lack of a matched external prior-map baseline remains an explicit limitation rather than an invented comparison.

Public PDFs/trajectories are stored outside Git in `/home/jian/livox_ws/p62_r26_published_baseline_sources/`; fetch manifests and hashes are archived here. GitHub API rate limiting was recorded, not bypassed; small public directory listings and raw result files supplied the needed availability evidence. No exhaustive absence claim is made for trajectories not found in the bounded inspection.
