# Paper Research Roadmap

## Immutable project identity

- Delivery baseline workspace: `/home/jian/livox_ws/dog_visual_loc_ws`
- Baseline branch: `feature/visual-factor-window`
- Baseline frozen SHA: `41999ea700c66c4cadf0eca9e0c5d73caa2783fd`
- Paper workspace: `/home/jian/livox_ws/dog_loc_paper_ws`
- Paper branch: `paper`
- Dataset root: `/media/jian/HIKVISION/paper rosbag`
- Comparison source root: `/media/jian/HIKVISION/comparison algorithm`
- Third-party execution policy: **DOCKER ISOLATION**
- Host dependency modification: **FORBIDDEN**
- Current stage: **P1 Benchmark Preparation**

The delivery baseline is frozen. No paper experiment may modify its source, configuration, launch files, build products, Git index, or existing dirty work. Paper innovations are developed only on the `paper` branch in the paper workspace.

## Scientific hypothesis

Working title: **Mode-Structured NDT Reliability Analysis**.

NDT scan-to-prior-map failures are provisionally divided into two mechanisms:

1. **Intra-mode uncertainty**: a single likelihood mode is broad because local geometry does not constrain all pose directions.
2. **Inter-mode uncertainty**: multiple separated modes are individually plausible because repeated map structure causes perceptual aliasing.

For modes `M_k = (w_k, mu_k, Sigma_k)`, later stages will investigate reliable mode discovery and representation, online separation of intra/inter-mode risk, and risk-specific update policies. The law of total covariance itself is background mathematics, not a novelty claim.

## Stage plan and gates

| Stage | Purpose | Exit gate |
|---|---|---|
| P0 | Freeze stable delivery baseline | Completed at frozen SHA above |
| P1 | Prepare datasets, benchmark source archives, host fingerprint, and literature audit | Catalogs complete enough to choose reproducible baselines; no host pollution |
| P2 | Screen failures with the frozen baseline | Repeatable failure intervals and metrics, without new estimator logic |
| P3 | Reconstruct offline NDT likelihood landscapes | Score surfaces and candidate modes validated on selected frames |
| P4 | Mode discovery and intra/inter uncertainty modeling | Stable mode representation and offline mechanism separation |
| P5 | Integrate online mode-aware reliability | Controlled update policy with ablations and regression checks |
| P6 | Optional event-triggered visual verification | Enter only if P5 evidence shows map ambiguity cannot be resolved otherwise |
| P7 | Full ablation, benchmark, runtime, CPU, RAM, and robot evaluation | Reproducible comparison package and statistical results |
| P8 | Manuscript | Claims trace to experiments and archived artifacts |

No P2 or later algorithm work is authorized by P1.

## Baseline hierarchy

### Tier A: direct prior-map localization

- Vanilla NDT, represented by the frozen system with mode analysis disabled.
- Autoware classic NDT localizer (`laboshinl/ndt_localizer`).
- `koide3/hdl_localization`.
- `HViktorTsoi/FAST_LIO_LOCALIZATION`.
- Same scan/map/initial-pose ICP and GICP registration ablations where a fair adapter is feasible.

These are the main-table candidates because their task is closest to prior-map localization.

### Tier B: mature LIO/SLAM anchors

- FAST-LIO2.
- LIO-SAM.
- Point-LIO and DLIO as supplementary candidates.

These test trajectory robustness in degraded scenes, but must not be presented as task-identical substitutes for prior-map localization.

### Tier C: recent method-level and strong references

- DCReg for direction-level ill-conditioning characterization and mitigation.
- Park et al. 2024 for uncertainty-aware NDT map-matching covariance.
- SuperLoc as a strong recent map-based LiDAR localization system.
- GenZ-ICP as a modern robust registration reference.
- DRPM as a probabilistic point-to-plane degeneracy detector/mitigator.

### Tier D: reference only

COIN-LIO, PALoc, R3LIVE, LVI-SAM, GEODE benchmark systems, and other multimodal methods remain literature context unless a later gate justifies implementation.

## Fair-comparison rules

1. Separate direct system comparisons, method-level diagnostics, and contextual LIO/SLAM comparisons.
2. Use the same scan, prior map, initial pose, temporal interval, and evaluation transform for ICP/GICP/NDT registration tests.
3. Do not call a reference trajectory ground truth unless the dataset documents the measurement process.
4. Prior maps must come from an official surveyed map or a separately declared mapping traversal/split. Never construct a map from the evaluation traversal without an explicit leakage analysis.
5. Report ATE/RPE and failure/completion rate where GT permits; also report runtime, CPU, RAM, and robustness/failure intervals.
6. Preserve failed runs and parameter changes; do not select only successful trials.
7. Do not mix odometry/SLAM results and prior-map localization results in one ranking without clearly labeling the task difference.

## Dataset acquisition decision

Priority candidates are FusionPortable `20220216_corridor_day`, M3DGR `Corridor01`, and ENWIDE Tunnel. Their official download endpoints were inspected in P1, but large raw bags were not forced through interactive/rate-limited endpoints. GEODE, NTNU Fyllingsdalen Tunnel, and the SuperLoc release were also investigated as high-value degenerate-scene alternatives.

Before every large download, check free space on `/media/jian/HIKVISION` and retain at least 20 GB. Every downloaded artifact must have its official URL, byte size, checksum when published, and integrity result recorded in `DATASET_CATALOG.md`.

## Isolation and host protection

- Never install comparison dependencies with `apt`, `pip`, `conda`, or by editing system paths during benchmark preparation.
- Never change `/opt/ros`, `/usr/local`, `/usr/lib`, system Python, `~/.bashrc`, `~/.profile`, or `/home/jian/livox_ws/devel` for third-party algorithms.
- One algorithm will later use one container/image. Dataset mounts are read-only; outputs go to `/media/jian/HIKVISION/comparison results/<algorithm>/`.
- If Docker is unavailable, record `DOCKER_NOT_AVAILABLE`; installation is a separate user decision.
- The baseline and paper workspaces run only in their own established host ROS environments and never source a comparison workspace.

## P1 asset state on 2026-09-22

- Host fingerprint: `docs/environment/HOST_ENVIRONMENT_BASELINE_20260922.txt`.
- Literature audit: `docs/BASELINE_SURVEY.md`.
- External dataset catalog target: `/media/jian/HIKVISION/paper rosbag/DATASET_CATALOG.md`; not safely writable at closeout because the external mount stopped responding.
- External source catalog target: `/media/jian/HIKVISION/comparison algorithm/ALGORITHM_CATALOG.md`; not safely writable at closeout for the same reason.
- Docker: not available; no installation attempted.
- Third-party source: archived only; no third-party build attempted.
- Large dataset acquisition: partial because official endpoints require interactive access, returned rate limits/TLS errors, or the external drive became temporarily unresponsive during an NTNU transfer.

## Authoritative links

- FusionPortable: https://fusionportable.github.io/dataset/fusionportable/
- M3DGR: https://github.com/sjtuyinjie/M3DGR
- ENWIDE: https://projects.asl.ethz.ch/datasets/enwide/
- GEODE: https://thisparticle.github.io/geode/
- NTNU LiDAR degeneracy files: https://huggingface.co/datasets/ntnu-arl/lidar_degeneracy_datasets
- SuperLoc: https://superodometry.com/superloc.html
