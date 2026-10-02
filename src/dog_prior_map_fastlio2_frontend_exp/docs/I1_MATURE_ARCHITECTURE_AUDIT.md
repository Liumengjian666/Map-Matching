# I1 mature architecture audit

Audit date: 2026-10-02. Base: `739079facf5a298d160b7efc643d17b3839ef975`.
Read together with [current code map](I1_CURRENT_CODE_MAP.md) and
[novelty decision](I1_UOBS_NOVELTY_AUDIT.md). This is an evidence-led architecture
selection, not a claim that the current full-Corridor estimator is stable.

## Search scope and evidence standard

Searched 2023–2026 LiDAR/LIO/LIVO degeneracy, selective/directional fusion,
localizability, scan-to-prior-map information, covariance/Hessian methods,
corridor/tunnel/underground navigation, and low-resource localization. The
representative set below contains **14 recent work families**, plus three
classic foundations. Extra abstracts screened are listed separately. This is
a broad targeted review, not an exhaustive systematic-review/meta-analysis.

Evidence labels: **F** = full text read; **M** = relevant primary method
sections read; **P** = publisher/author summary only; **C** = actual algorithm
source inspected at the pinned commit below. A repository announcement is not
proof that the corresponding released implementation was found or reproduced.
No published runtime/error number is treated as an apples-to-apples P7 result.

| Work / year / primary source | Evidence | Mechanism and state/history distinction | Code availability in this audit |
|---|---|---|---|
| [Selective Kalman Filter](https://arxiv.org/html/2412.17235v1), 2024 preprint; [Fuse only what matters](https://www.sciencedirect.com/science/article/pii/S0924271626002790), 2026 ISPRS | F preprint / P journal | Inverse information incorporates cross-block effects; preprint then uses separate rotation/translation marginal eigenspaces. Selective visual information in a current recursive filter; historical map/visual maintenance is not a multi-state optimizer. | No author implementation located and pinned; do not substitute third-party code. |
| [X-ICP](https://github.com/leggedrobotics/perfectlyconstrained), 2023 online / 2024 TRO | M/C | Localizability contributions and constrained registration. Released tree also contains 6D solution-remapping comparators; these are not all the original X-ICP detector. Current alignment uses a historical reference cloud. | Pinned source inspected. |
| [Informed, Constrained, Aligned](https://arxiv.org/abs/2408.11809), 2024 preprint / 2025 field study | M/C | Comparative degeneracy-aware registration mechanisms; useful evidence against assuming one constraint policy universally wins. | Same perfectlyconstrained tree, explicitly distinguish comparator modes. |
| [DCReg](https://arxiv.org/abs/2509.06285), 2025 preprint / 2026 IJRR | P/C | Schur rotation/translation characterization, basis labeling and selective preconditioning; solves registration, not a replacement inertial filter. | Local author implementation inspected. |
| [Switch-SLAM](https://doi.org/10.1109/LRA.2024.3421792), 2024 | P | Switching complementary LiDAR/visual odometry. Author [thesis summary](https://repository.dl.itc.u-tokyo.ac.jp/record/2013872/files/K-09824-a.pdf) supports the switching concept, not a fully audited estimator/window implementation. | No verified author code pin; exact backend/trigger details UNKNOWN. |
| [MM-LINS](https://arxiv.org/html/2503.19506v1), 2024 T-IV / 2025 preprint | M/C | FAST-LIO-based current-state tracking plus multiple maps and global association. Paper covariance-eigen discussion does not exactly describe the checked code's operational diagonal-covariance gate. | Pinned source inspected; do not copy undocumented constants. |
| [COIN-LIO](https://arxiv.org/abs/2310.01235), 2023 preprint / 2024 ICRA | P/C | Current filter and scan geometry choose complementary intensity patches; checked weak-direction logic is translation 3D. Historical patches/map remain. | Official implementation inspected; Ouster intensity/range-image inputs are not available in P7 packed XYZ. |
| [Probabilistic Degeneracy Detection for Point-to-Plane Error Minimization](https://arxiv.org/abs/2410.10784), 2024 (DRPM) | P/C | Point/normal noise predicts directional information contamination; probabilistic attenuation in full eigenbasis. Not PCL voxel covariance inversion. | Author header implementation inspected. |
| [FAST-LIVO2](https://arxiv.org/abs/2408.14035), 2024 preprint / subsequent TRO implementation | P/C | Sequential LiDAR and direct photometric iterated-filter updates; current state with a unified voxel/patch map, not joint optimization of a pose window. Visual path is maintained, not simply started after failure. | Official implementation inspected. |
| [FAST-LIVO2 on Resource-Constrained Platforms](https://arxiv.org/html/2501.13876v1), 2025 | M | Degeneracy-aware visual frame selection and bounded/local versus retained visual map management. A separate resource-focused work; do not attribute its selector to every FAST-LIVO2 checkout. | Selector implementation not independently pinned. |
| [LODESTAR](https://arxiv.org/abs/2511.09142), 2025 preprint / 2026 RA-L | M | Adaptive Schmidt-Kalman with active/fixed past poses and data exploitation. A genuine multi-state counterexample: retaining cross-correlations/history provides capabilities absent from a current-state filter. | Author code not located at a verifiable pin. |
| [SA-LIVO](https://arxiv.org/html/2606.25699v1), 2026 preprint | M | Joint 6D information soft gating within a current invariant-filter architecture; direct photometric history includes a five-frame visual observation window (each frame can contain multiple observations), not five jointly optimized pose states. | Original source not located; third-party reimplementation excluded from mature-code evidence. |
| [LF-GICP](https://arxiv.org/abs/2608.19522), 2026 preprint | M/C | Voxel-normal translation localizability field, temporal statistics, and information reweighting. Full 6D Hessian enters the weighting; detector is not identical to U_obs. | Author code inspected; many experimental/environment switches preclude treating it as a validated drop-in. |
| [Degeneracy-Resilient Teach and Repeat Using FMCW Lidar](https://arxiv.org/html/2603.10248v1), 2026 preprint | M | Weighted prior-submap ICP, Schur-derived adaptive block scaling, full 6D eigenspaces and remapping; Doppler velocity supplies independent information. Neither fixed L nor our sensor inputs. | Paper equations inspected; matching released code not pinned. |

Additional critical mathematical reference: [Degenerate in Whose Frame?](https://arxiv.org/abs/2608.15532v1),
2026 preprint (**F**, no code located), explicitly compares characteristic-length
metrics and coupled subspace reporting. Its submission status is not peer-review
acceptance. Classical foundations: [FAST-LIO2](https://github.com/hku-mars/FAST_LIO),
[LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) (**C**), and
[LION](https://arxiv.org/abs/2102.03443) (2021, **M**: fixed-lag inertial/scan
estimation and observability-aware supervision; no verified code pin here).

Other 2025–2026 screened leads, not counted as implementation-audited evidence:
[screw-based feature constraint model](https://doi.org/10.1177/02783649261463720),
[anisotropic underground SLAM](https://doi.org/10.1016/j.measurement.2026.122838),
[DALI-SLAM](https://doi.org/10.1016/j.isprsjprs.2025.01.036), and
[LP-ICP](https://arxiv.org/abs/2501.02580). Their abstracts strengthen the need
for careful novelty boundaries; they do not establish equation-level identity.

## SKF version ledger

2024 title is *Selective Kalman Filter: When and How to Fuse Multi-Sensor
Information to Overcome Degeneracy in SLAM*, arXiv 2412.17235v1. Formula-level
comparison in this audit is **preprint evidence**.

2026 publisher record: *Fuse only what matters: Degeneracy-aware multi-sensor
fusion for LiDAR-Inertial-Visual SLAM*, ISPRS 238, 508–518,
DOI 10.1016/j.isprsjprs.2026.05.031. Public author record names Xuanxuan Zhang,
Jie Xu, Chengxi Yang, Guanyu Huang, Lijun Zhao, Ruifeng Li, Shenghai Yuan,
You Li and Lihua Xie. The public contributions describe covariance-based
coupling-aware detection and when/which-direction visual fusion. Experimental
preview identifies detector comparisons with X-ICP, LION and Zhang's method;
discussion acknowledges manually fixed thresholds. Exact added sequences,
changed numerical results and formula revisions versus 2024 are **UNKNOWN**.
No claim of reading the inaccessible journal equations. User explicitly
authorized proceeding with this evidence separation; PDF access is not a stop.

## Actual source ledger

No reference dependency was added to P7. Clones/downloads are outside the
workspace under `/tmp/i1_reference_audit.1Mep3d`; links below identify durable
upstream artifacts. File/function inspection is not equivalent to reproducing
the complete reference system or auditing all of its numerical safeguards.

| Repository / exact commit | File / function inspected | Actual behavior relevant here |
|---|---|---|
| [FAST-LIO](https://github.com/hku-mars/FAST_LIO/tree/7cc4175de6f8ba2edf34bab02a42195b141027e9) | `include/use-ikfom.hpp`: `get_f`, `df_dx`, `df_dw`; `esekfom.hpp`: `predict`, `update_iterated`; `src/IMU_Processing.hpp`: `UndistortPcl` | IMU/manifold/covariance and time-ordered deskew remain mature components. Current offline packed-cloud runner does not itself execute upstream deskew. |
| [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM/tree/0be1fbe6275fb8366d5b800af4fc8c76a885c869) | `src/mapOptmization.cpp::LMOptimization` | AtA eigenanalysis, zero weak rows, project solved increment before pose update; not a degeneracy-specific Kalman covariance update. |
| [perfectlyconstrained](https://github.com/leggedrobotics/perfectlyconstrained/tree/0fbe4175ea205a271f85287abfd8048e5f7dd32a) | `libpointmatcher/pointmatcher/ICP.cpp`: `detectLocalizabilityWithSolutionRemappingMethod`, `solutionRemappingProjectionCalculation`; `ErrorMinimizers/PointToPlane.cpp` remapping dispatch | Full 6D remapping comparator projects `x`; other modes include X-ICP constraints. Do not label every branch the same method. |
| [DCReg](https://github.com/JokerJohn/DCReg/tree/ce7db8220f549a4a4391729e3bf4de4d4ab74635) | `DCReg/include/dcreg.hpp`: `DetectDegeneracy`, `AlignEigenBasisToAxes`, `CharacterizeDegeneracy`, `SolvePreconditionedUpdate`, `SolveRawNormalEquation` | Invertibility checks, Schur blocks, per-block characterization, preconditioned solve/fallback QR. Basis permutation/sign alignment does not mean every vector becomes a Cartesian axis. |
| [COIN-LIO](https://github.com/ethz-asl/COIN-LIO/tree/76729cc4feb3649cbd79d28f82d9f62a2c82889b) | `src/laserMapping.cpp`: `calculateContributions`, post-update weak-direction selection; `src/feature_manager.cpp::detectFeaturesComp` | Translation Jacobian eigenspace guides complementary intensity-gradient patch selection. |
| [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2/tree/0d2c0346107b75b59934975adec9a6eeeb913c64) | `src/LIVMapper.cpp`: `stateEstimationAndMapping`, `handleVIO`; `src/vio.cpp`: `processFrame`, `computeJacobianAndUpdateEKF`, `updateState` | Sequential current-state paths; multilevel direct patch residual/Jacobian, exposure component, error-increase rollback. Not a ready pose-only P7 adapter. |
| [MM-LINS](https://github.com/lian-yue0515/MM-LINS/tree/e449d56d3aae2a49338d262b9c262410693ebdd6) | `FAST-LIO/src/laserMapping.cpp`, post-update covariance/re-map trigger | Logs covariance eigen diagnostics but operational trigger in inspected path tests Pxx/Pyy/Pzz against axis constants. Paper and checked implementation must be distinguished. |
| [LF_GICP](https://github.com/ies0411/LF_GICP/tree/6b88f2877f0eb123e7f591a8bb9824655f97f2de) | `lf_gicp/cpp/lf_gicp_core.cpp`, normal-scatter and `v^T H_i v` weight sections | Planarity-weighted 3D normal field and full-Hessian directional weights. No evidence this supplies a fixed-physical 6D NDT detector. |
| [DRPM](https://github.com/ntnu-arl/drpm/tree/b39a540c946f4059f5e3dfde625216c437eaf158) | `src/degeneracy.h`: `ComputeNoiseEstimate`, `ComputeSignalToNoiseProbabilities`, `SolveWithSnrProbabilities`, `EstimateNormal` | Explicit point/normal noise model; solves with direction probabilities divided by eigenvalues. Needs inputs absent from current P7 proxy. |
| [SuperLoc/SuperOdom](https://github.com/superxslam/SuperOdom/tree/f10e65cd50007767b22e4c401689665e20d827d6) | `super_odometry/src/LidarProcess/LidarSlam.cpp`: `FeatureObservabilityAnalysis`, `computeTranslationObservability`, `analyzeFeatureObservability` | PCA/planarity and physical-axis feature contribution labels; not a mixed 6D eigenbasis. See [author project](https://superodometry.com/superloc.html). |

## Architecture decision matrix

Costs below are structural estimates, not measured I1 timings. Current-state
does not mean no history: map, intensity patches, visual tracks and metadata
can dominate memory even with one filter state. A pose window adds covariance
or optimization state and is justified only when that history is necessary.

| Design Decision | Recent Methods Compared | Actual Implementations Read | Mature Choice | Reason | Compute Cost | Memory Cost | Integration Risk | Use in Our System |
|---|---|---|---|---|---|---|---|---|
| Degeneracy observation: single vs multi-frame | SKF, COIN, SA-LIVO, LF-GICP | COIN, LF-GICP, P7 | Current scan against retained map | Attribute local geometry to present measurement; temporal smoothing changes response/lag | Pair accumulation + 6D solve | Frame-local observations | Low to retain; history may hide onset | Keep U_obs current-frame; do not call it global observability |
| Estimator: single-state vs window | FAST-LIVO2, SA-LIVO, LODESTAR | FAST-LIVO2, pinned IKFoM | Existing single-state recursive filter | No demonstrated need for retained pose states in the selected geometric question | Fixed state-size filter; correspondence cost remains | Fixed covariance + retained map | Low to keep, high to port new filter | Primary carrier |
| Registration | DCReg/X-ICP ICP, LF-GICP GICP, current PCL NDT | All corresponding pinned code above, installed PCL | Existing current-frame PCL NDT | Preserve controlled carrier; changing objective would confound detector study | Existing NDT iterations | Existing target grid | Baseline grid-resolution issue must be settled separately | Keep, no new optimizer |
| Degeneracy representation | SKF covariance, DCReg Schur, DRPM noise model | DCReg, DRPM, current U_obs | Keep current proxy as candidate, not ground truth | Compare against alternatives, not rename an existing detector | 6D eigensolve cheap; support extraction dominates | Small matrices + observations | Information calibration unresolved | Research gate applies |
| Rotation-translation coupling | SA-LIVO, FMCW, SKF, frame-equivariance paper | P7, X-ICP comparator, DCReg | Preserve full joint subspace | Split marginal effects and mixed vectors are different; coupling itself is prior art | Small dense solve | 6x6 | Chart/origin dependence | Keep, no axis masks |
| Weak-direction handling | X-ICP, LIO-SAM, DRPM, SA-LIVO | X-ICP, LIO-SAM, DRPM | Do not activate a new downstream before novelty gate | Existing remapping is comparison, not a new contribution; information projection needs a complete chart/noise contract | Depends on chosen rule | Small if same state | High if optimizer increments are confused with filter measurements | No I1 candidate activated |
| Visual constraint form | SKF, FAST-LIVO2, COIN | FAST-LIVO2, COIN | Direct photometric patches would be the evidence-backed candidate, not generic VO-pose fusion | Strong reusable implementation, but calibrated image/history path is not a minimal packed-XYZ adapter | Patch pyramids/Jacobians | Persistent patches/images | High now; no verified hardware budget | NOT introduced in I1 |
| Visual trigger policy | SKF, resource FAST-LIVO2, FAST-LIVO2 | FAST-LIVO2 continuous path | Distinguish maintain/track from selected state update | On-demand fusion does not imply a cold-started camera frontend | Maintenance cost persists | Track/map persistence | Startup/readiness bias | No visual runtime change |
| IMU role | FAST-LIVO2, MM-LINS, LODESTAR | IKFoM, FAST-LIVO2, MM-LINS | Existing causal propagation and biases/gravity | Mature; cannot create missing absolute corridor position information indefinitely | Linear in IMU samples | Existing state/input storage | Low | Keep |
| Measurement validation | Current P7-E, mature NDT/filter practice | P7 admission and frontend | Preserve existing admission as optional comparison | Geometric rank does not establish correct basin/association | Existing rank-6 evaluation | Constant | Confounds A/B if added simultaneously | Not a new U_obs use |
| Recovery / relocalization | MM-LINS, LION, hdl reference from P7-E | MM-LINS; P7-E audit for hdl | No new recovery | Extra capability and independent evaluation problem | Potentially large search | Multiple maps/database | High | Out of scope |
| State update method | FAST-LIVO2, SKF, SA-LIVO | IKFoM and FAST-LIVO2 | Reuse frontend, no rewrite | Current full-pose API is not equivalent to their residual-level information fusion | Fixed-size | Fixed-size | Coordinate/noise mismatch if ported naively | Keep existing behavior |
| Map representation | SuperLoc, COIN, FAST-LIVO2 | Corresponding code above, PCL | Retain prior-map NDT grid | U_obs requires same actual support as registration | One target build | Map dominates | Map/test independence must be proven | Keep, audit actual leaf size |
| Runtime memory strategy | resource FAST-LIVO2, LODESTAR, current P7 | P7, FAST-LIVO2 | Frame-local geometry, long-lived target only | No benefit from accumulating U_obs vectors | Linear in current support | No per-frame observation growth | Metadata still sequence-sized | Preserve |

## Selected carrier and bounded implementation decision

**ONE PRIMARY:** existing paper current-scan prior-map PCL NDT + pinned
single-state FAST-LIO2/IKFoM. This is an engineering choice supported by the
current-filter examples above, not a claim that single-state is more accurate
than LODESTAR/LION or that P7 already has their robustness.

**FALLBACK:** none activated. No reason to abandon working input/state ownership
or rebuild the stable dog estimator. Retain P7-D/E as disabled/default-bypassed
comparison assets. U_obs stays the primary research object.

**Downstream:** not frozen/implemented before the novelty decision. Repeating
the already-tested mean remapping would be P7-D again, not a new I1 mechanism.
Directly passing U_obs eigenvectors to a pose-residual filter API is invalid;
adding whitening, a new gain, vision and recovery to make it run would violate
the one-mechanism/minimal-adaptation contract. A contribution claim about a new
downstream cannot be made without specifying and auditing that downstream.

## Dataset / evaluation gate

Corridor01 is locally available and geometrically relevant, but availability
and a map SHA do not prove independent mapping/test traversals. No I1 dataset
has been approved solely on that basis. COIN-LIO's tunnel/intensity setting is
relevant but its sensor-specific inputs are not a drop-in replacement for
P7 XYZ assets. No new dataset was downloaded, no GT opened, and no primary
sequence was silently substituted.

If implementation is later authorized, freeze one independently mapped/tested
degenerate sequence before reading evaluation GT, plus one downstream rule.
Run unit -> 1 -> 20 -> 100 -> independently defined degenerate segment -> full.
Use the existing evaluator without altering its alignment/prefix/minimum-sample
contract. Require disabled-path parity and identical inputs. Report failures
and common-prefix errors as well as complete-run errors; do not compare unequal
prefix RMSE as a win. U_obs diagnostics need a reference independent of its own
eigenvalue threshold (geometry/controlled perturbation, not GT-tuned settings).
