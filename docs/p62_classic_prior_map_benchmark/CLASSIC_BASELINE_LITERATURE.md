# Published classic prior-map localization baselines

Task: P62-CLASSIC-PRIOR-MAP-LOCALIZATION-BENCHMARK-PLAN. Evidence inspected 2026-10-26; no baseline installation or rerun.

The primary source is K. Koide, S. Oishi, M. Yokozuka and A. Banno, *Tightly Coupled Range Inertial Localization on a 3D Prior Map Based on Sliding Window Factor Graph Optimization*, ICRA 2024, DOI [10.1109/ICRA57147.2024.10611195](https://doi.org/10.1109/ICRA57147.2024.10611195). Sources: [author PDF](https://staff.aist.go.jp/k.koide/assets/pdf/icra2024_02.pdf), [author project](https://staff.aist.go.jp/k.koide/projects/icra2024_gl/), [official dataset](https://zenodo.org/records/10122133). The retained PDF and original numerical source identity are in sources/.

## What the authors actually compared

|Method|Actual task and mechanism|Use in our paper|
|---|---|---|
|hdl_localization|Prior-map NDT registration with UKF/optional IMU prediction|Primary published classic baseline; not our controlled native-PCL/EKF baseline|
|FAST LIO LOCALIZATION|FAST-LIO2 odometry plus map correction|Primary published prior-map baseline; never substitute FAST-LIO2 odometry numbers|
|Koide 2024 proposed|Windowed scan-to-scan/map registration and IMU factor optimization|Advanced published reference, not an algorithm we ran|
|Frozen C+G|Native PCL NDT, EKF/OOSM, diagnostic pre-geometry gate, post-quality admission and bounded yaw retries|Our own complete continuous experiment, pending minimum-chain admission|

The authors ran the localization baselines themselves. Outdoor numbers are not borrowed FAST-LIO2 odometry results. Current official READMEs corroborate the baseline task definitions: [HDL](https://github.com/koide3/hdl_localization), [FAST-LIO Localization](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION). README snapshots/blob identities are retained. These current READMEs do not certify the exact 2024 executable or parameters. The project lists the advanced localization implementation as closed source; no recreation is required here.

## Table II, PDF page 6 / Sec. IV-B

|Published method|Outdoor01 ATE [m], as printed|Corruptions|Outdoor02 ATE [m], as printed|Corruptions|
|---|---:|---:|---:|---:|
|HDL Localization|5.592 ± 5.252|5|7.137 ± 6.833|5|
|FAST-LIO Localization|6.983 ± 6.693|4|4.849 ± 4.692|4|
|Koide proposed|0.959 ± 0.838|0|0.597 ± 0.401|0|

These exact entries are verified against the PDF table. They remain **published ATE centre ± reported term**, not asserted RMSE or mean±standard deviation. The inspected text does not define the centre/± statistic. `PUBLISHED_BASELINE_RESULTS.csv` records the omission as NR rather than deriving a new value from the pair.

The paper uses evo and a batch map-registration/IMU reference. It restarts a corrupted method and counts corruptions. Exact alignment flags, per-restart initialization poses, failure threshold, retained intervals and aggregation across restarted segments are NR. Automatic initialization is described for the proposed outdoor method; manual initialization is explicit in the indoor section, not a complete outdoor-baseline initialization specification. Do not silently extend that specification to every outdoor run.

The public sensor card establishes MID360, two bag parts per sequence, corresponding outdoor map and a map-frame LiDAR reference. Publication labels Outdoor01/02 are operationally associated with `outdoor_hard_01/02` from the named hard outdoor pair; raw sequence integrity and exact matching of evaluation bounds still require payload admission. The dataset reference is map-anchored batch optimization, not independent motion capture.

## Reporting rule

Use the published rows as classic context immediately. Grade-A ranking remains unavailable until sequence bounds, initialization, statistic, alignment and restart handling are compatible. Do not compute percentage improvement from our unaligned, no-reset RMSE to an undefined published ATE centre. Runtime/platform values must also retain their original measurement scope.

## Minimal outstanding protocol questions (not automatically sent)

To upgrade comparability, obtain the exact evo command/statistics key, the definition of the ± term, numerical corruption rule, baseline initialization/restart prescription, and which timestamps/segments enter each reported value. The author's public v1 HTML was additionally checked; it does not supply these missing definitions. Absence is reported as NR in inspected material, not a claim that no private evaluation script exists.

For manuscript use, PAPER_COMPARISON_TABLE.tex supplies a two-column-spanning published-context table and REFERENCES.bib supplies the citation. This is an editable table snippet, not a newly compiled complete manuscript/PDF. Do not insert an unmeasured C+G row. The CSV also records unreported rotation/RPE/hardware/memory; the advanced author's17.21ms/58.1FPS claim has its own scope and is not our measurement.
