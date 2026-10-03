# Dual-U Novelty and Formulation Audit

Audit date: 2026-10-03
Workspace HEAD: `ff243ae541943e067cdae99dcc9d9391dc716023`
Scope: research decision and prior-art falsification only; no production implementation is authorized by this audit.

## Executive decision

The conceptual distinction is useful for diagnosing registration failures:

- `U_obs`: conditional, within-basin local pose information, assuming a basin is the correct one;
- `U_nonlocal`: probability/evidence that the nominal basin is the correct spatial mode among evaluated alternatives, including uncertainty that the candidate set missed the correct mode.

These quantities are not interchangeable. The four combinations are meaningful. But the proposed *joint reliability framework* is substantially represented by probabilistic multi-hypothesis localization: each spatial hypothesis has a within-mode Gaussian pose estimate/covariance, hypotheses have posterior weights, and some methods explicitly retain probability that all tracked candidates are wrong. Therefore, the semantic product and hierarchical use are not, by themselves, a new mathematical object.

The closest direct conceptual precedent is Stannartz et al. (IEEE Access, 2023): a Gaussian-sum localization filter tracks spatially separated weighted Gaussian pose hypotheses, updates each using sensor-to-map pose observations and odometry, and models a null hypothesis for the event that none of the retrieved candidates is correct. Its components jointly represent per-mode uncertainty, cross-mode ambiguity, candidate-set incompleteness, and availability. It uses cross-modal learned matching and a multi-hypothesis recursive estimator rather than one PCL-NDT terminal plus an IKFoM state, but that implementation difference alone does not establish a new reliability formulation. [Paper](https://doi.org/10.1109/ACCESS.2023.3286310), [author-hosted full text](https://eldorado.tu-dortmund.de/bitstreams/0da3e223-e98c-4a05-9109-c40a7787fd44/download).

Other close precedents reinforce the overlap: AGM explicitly models mapped environmental ambiguity and localization error in ambiguous regions; Reliable-loc combines spatially verifiable global hypotheses with sequential pose-uncertainty monitoring and mode switching; Autoware's NDT matcher includes multi-initialization NDT uncertainty modes; and a September 2026 prior-LiDAR-map visual localization preprint propagates association distributions into directional pose-information uncertainty and selectively reinforces factors complementary to weak directions. [AGM paper](https://pmc.ncbi.nlm.nih.gov/articles/PMC6695785/), [Reliable-loc paper](https://arxiv.org/abs/2411.07815), [Reliable-loc code](https://github.com/zouxianghong/Reliable-loc), [Autoware implementation](https://github.com/autowarefoundation/autoware_core/blob/main/localization/autoware_ndt_scan_matcher/src/ndt_scan_matcher_core.cpp), [2026 preprint](https://arxiv.org/abs/2609.27363).

**Gate:** `DUAL_U_NOVELTY_NO_GO` for the current broad claim that the explicit two-axis reliability decomposition and hierarchical combination are the primary algorithmic contribution. The exact PCL-NDT/candidate-evidence instantiation might still be a narrow research question, but no distinct mechanism or demonstrated advantage is established here. Do not create the implementation branch or change production code on this result.

**Three-point structure:** `DUAL-U-NOT-ENOUGH-FOR-PAPER`. A is an informative problem/failure framing but overlaps established multimodal posterior representations; B is the only plausible algorithmic research question and is not yet shown to differ from mature multi-hypothesis likelihood methods; C is presently a system integration of mature mode selection and conditional local uncertainty handling, not a distinct algorithmic contribution.

## 1. Legacy implementation audit

### Legacy `U_obs`

`LocalRisk` in [`dual_reliability.hpp`](../include/dog_prior_map_fastlio2_frontend_exp/dual_reliability.hpp) stores local weak/risk flags, block eigenvalue ratios, directions, joint weak/reliable bases, map support, and a physical length scale. This is a within-terminal/local quantity; it does not establish that the terminal is the correct map location.

The P6-I6B experiment history is more specific than the type name: its active PCL score-Hessian path was not accepted as valid under the gradient/coordinate checks, so the tested UOBS-only path was effectively the baseline. P7 later computed a geometric information proxy from current NDT target-voxel covariance, but the I2 falsification report concluded that fixed-scale U_obs did not outperform the mature DCReg reference and failed reference-point invariance. P7-C therefore remains historical diagnostic evidence, not a validated primary detector. See [`I2_UOBS_FALSIFICATION.md`](I2_UOBS_FALSIFICATION.md) and [`I1_CURRENT_CODE_MAP.md`](I1_CURRENT_CODE_MAP.md).

### Legacy `U_nonlocal`

The active P6-I6B design used nominal `M0` and two NDT reruns seeded by plus/minus one standard deviation along a selected principal direction of the projected prediction covariance. Probes were triggered by innovation or periodically. [`analyzeNonlocalTerminalStability`](../src/reliability_metrics.cpp) records terminal responses and objective differences, but its successful status is literally `RECORDED_NO_BASIN_CLASSIFICATION`.

This measures *response to two tested nearby seeds* (an operational local attraction/stability diagnostic). It does not enumerate spatially separated basins, cluster map-wide alternatives, compare mode evidence, or estimate that an unobserved competing basin exists. Returning to the same terminal under those probes cannot prove cross-map uniqueness. P6-I4's finite directional escape-margin experiments have the same semantic boundary: they probe a nominal basin along a finite set of directions, not the topology of the full map's registration objective.

### Legacy dual decision

[`decideDualReliability`](../src/dual_reliability.cpp) assigns NORMAL, DIRECTIONAL, CAUTIOUS, or PREDICTION_ONLY behavior from the local flags and finite-probe response. [`makePoseMeasurementNoise`](../src/dual_reliability.cpp) adds local directional increments and empirical outer products of the two probe responses to a base covariance; the type explicitly marks this covariance as not statistically calibrated. Invalid requested probes can cause prediction-only. There is no spatial mode set, no posterior over basins, no null/unrepresented-mode mass, and no calibrated four-state classifier.

The P6-I6B results also matter: UOBS_ONLY was the baseline when the active U_obs detector was invalid; DUAL_RELIABILITY collapsed to the nonlocal-only path; v0/v1 did not demonstrate an incremental dual benefit. This is not evidence for the proposed new decomposition.

### Current P7 path

At this HEAD, [`current_frame_ndt.cpp`](../src/current_frame_ndt.cpp) performs current-frame NDT using the configured target grid and computes U_obs after the terminal result. [`p7_single_state_runner.cpp`](../scripts/p7/p7_single_state_runner.cpp) retains the experimental FULL_POSE, MATURE_SOL_REMAP, and MATURE_ADMISSION modes. The default mature carrier is a single-state IKFoM frontend; P7 has no cross-basin candidate generator or U_nonlocal in its normal current-frame path. P7-D remapping and P7-E admission are historical baselines and must not be conflated with a new Dual-U implementation.

## 2. Closest prior art and exact boundary

| Work | What is already present | What is not the same as the proposed P7 instantiation |
|---|---|---|
| Stannartz et al., 2023, ambiguity-aware cross-modality global self-localization | A bounded Gaussian-sum filter tracks spatially distant pose hypotheses. Each has a weight and Gaussian pose uncertainty; per-hypothesis map-matching observations and odometry update both local estimate and mode weight. A null hypothesis estimates that no tracked candidate is correct; clutter/misdetection likelihood and candidate initialization are modeled. This is very close to within-mode uncertainty + cross-mode uniqueness + joint availability. | Uses cross-modal learned correlation/PR, odometry, and multiple recursive hypotheses; not a single PCL-NDT terminal, not a directional target-voxel information detector attached to one IKFoM state. It is nevertheless a strong prior against claiming the general two-axis posterior or hierarchy as new. |
| AGM-AMCL, 2019 | Offline ambiguity grid evaluates environmental perceptual ambiguity/localizability and estimates possible accumulated localization error; uses that map information within probabilistic localization. | 2D occupancy-grid AMCL and environmental ambiguity, rather than 6-DoF NDT basin likelihood plus conditional local geometry. It already weakens a broad claim that prior-map ambiguity/localizability is an unmodeled reliability dimension. |
| Reliable-loc, 2025 | MCL over spatial hypotheses with spatial verification, pose-uncertainty monitoring, and adaptive mode switching; public code repository. | Wearable LiDAR/global localization and sequential cues, not a bounded low-cost NDT terminal candidate posterior with an explicit local directional subspace. It still overlaps joint candidate reliability and local uncertainty. |
| Autoware NDT Scan Matcher | Mature NDT multi-initialization and score-based covariance options evaluate several starts and derive localization covariance/spread. The implementation exposes multiple NDT results and initialization poses. | Primarily covariance diagnostics from a bounded search design; not a map-wide completeness certificate or explicit 6-DoF local-vs-global reliability taxonomy. It makes multi-start NDT a prior art baseline, not a new mechanism. |
| Zhao et al., 2026-09 arXiv preprint | In prior-LiDAR-map visual localization, association distributions are propagated to directional pose-information uncertainty; structural factors are selected to complement weak directions. This directly overlaps the idea that association ambiguity and weak geometry should jointly affect measurement use. | Ambiguity is primarily uncertainty over visual correspondences, not a posterior over spatially separated NDT basins. It is a preprint and should be cited as such. |
| DCReg / X-ICP / SKF-Fusion | Mature local geometric degeneracy, weak directions, covariance/coupling treatment, and/or selective fusion. | Do not by themselves certify map-wide basin uniqueness. They are foundations/comparators, not Dual-U novelty. |

### Direct-overlap test for the closest methods

| Test | Stannartz et al. (2023) GSF | Reliable-loc (2025) | Autoware MULTI_NDT | Zhao et al. (2026-09 preprint) |
|---|---|---|---|---|
| Same input/problem? | Same broad prior-map global/local localization problem and sensor-to-map pose observations; different cross-modal learned frontend, not the same raw NDT point-cloud input. | Same broad LiDAR localization in a prior map and spatially wrong candidate risk; wearable/vehicle map setup differs, not a novelty basis. | Same scan-to-map NDT objective and current scan/map input; candidate starts are a bounded uncertainty probe around an estimate. | Same prior LiDAR map and camera localization, but visual correspondences rather than NDT basins. |
| Same mathematical object? | **High overlap:** weighted spatial Gaussian modes (local mean/covariance) plus between-mode weights and a null/missed-candidate mass. It does not call local covariance a geometry-derived U_obs. | **High partial overlap:** particle/mode distribution, spatial verification and sequential pose uncertainty; not an explicitly factored pair of reliability variables. | **Partial:** covariance from multi-start convergence or score-weighted candidate poses; not a calibrated cross-map uniqueness posterior. | **Partial:** association distributions influence directional pose-information uncertainty; ambiguity is correspondence-level, not spatial mode-level. |
| Same online decision? | **High overlap:** update mode probabilities; make localization available only after competing hypotheses are resolved and null mass is low. | **High partial overlap:** monitor localization status and switch modes using temporal/verifiable cues. | Covariance output/diagnostics; not the proposed basin-validity then conditional-directional update policy. | Select/reinforce structure factors according to complementarity to weak pose directions. |
| Same sensors/map setting? | Prior map yes; radar↔LiDAR or LiDAR↔aerial imagery, not MID-360+IMU single-state NDT. | Prior map and LiDAR yes; not the same sensor, map format, or single-state carrier. | 3D LiDAR scan and prior map yes; different NDT backend and mostly XY covariance. | Prior LiDAR map plus camera; sensor configuration differs. |
| Does difference alone protect novelty? | No. Backend and filter architecture differences are insufficient if the reliability posterior and decision are equivalent. | No. Application/domain differences are insufficient. | No. NDT/backend adaptation alone is insufficient. | No. Sensor difference alone is insufficient. |

No public implementation repository for the Stannartz 2023 paper was located in the targeted search. Its paper equations and method description are therefore the primary evidence for that work. Reliable-loc and Autoware have inspectable public implementations; the 2026 Zhao et al. item is currently preprint evidence, with no code link identified in its arXiv record.

The 2023 paper's operative state is already a mixture:

\[
p(T\mid Z,M)=\sum_{k=1}^{K} \pi_k\,p(T\mid B_k,Z,M),
\]

where each component has a conditional mean/covariance and a mode probability, with a null probability for untracked/candidate-missed cases. In a local tangent chart, the familiar total-covariance identity separates within-component spread from between-component spread. A two-axis status can be read from that richer posterior: local conditional covariance/information describes within-mode behavior, and posterior mass over spatial modes describes ambiguity. That makes the semantic distinction scientifically useful, but it does not alone create a new joint mathematical formulation.

## 3. Two candidate joint formulations

### Formulation 1 — two-axis diagnostic state

Input: one local detector and one candidate/mode detector. Output a pair of calibrated states, e.g. `O ∈ {adequate, weak}` and `N ∈ {unique-enough, ambiguous/unknown}`; do not sum them into a scalar. The Cartesian product gives the four failure classes. Units need not be numerically comparable because the axes are different events, but each detector needs its own calibration and operating threshold. The downstream result is a 2D state and a fixed decision table.

This formulation is transparent and supports controlled ablation, but A's core decomposition is already represented by mixture localization, and the thresholds are not probabilities unless calibrated. A taxonomy alone cannot claim new estimation or decision theory.

### Formulation 2 — hierarchical candidate posterior with conditional local geometry

Let `B_1…B_K` be spatially separated candidate basins proposed by a mature retrieval/generation method, and `B_0` denote a missed/unrepresented basin or clutter explanation. For a common observation model, define basin evidence

\[
E_k=\int_{B_k}p(Z\mid T,M,\mathcal{S})\,p(T\mid B_k,M)\,dT,
\qquad
\pi_k=\frac{P(B_k\mid M)E_k}{\sum_{j=0}^{K}P(B_j\mid M)E_j}.
\]

The local detector supplies conditional information/covariance `H_k` or `Σ_k` *inside* each mode. Cross-basin uniqueness is represented by `π_k` together with `π_0`, not by a raw score gap. A possible decision is hierarchical: first evaluate whether posterior mass is sufficiently concentrated on the nominal basin and candidate-miss risk is acceptable; only then use its conditional directional uncertainty for a pose update. If the application must remain single-state, an uncertain mode should be rejected/held rather than averaged across far-apart modes. A multi-hypothesis state is the statistically cleaner alternative but carries K-scaled work and memory.

This avoids an arbitrary weighted sum because the outputs live in one probabilistic model. It also exposes demanding assumptions: candidate priors, calibrated detection/retrieval recall, consistent all-return likelihoods, explicit clutter/unmatched-point probability, and stable basin integration. It is not selected as a new contribution: the GSF precedent already has weighted Gaussian modes, per-mode Kalman updates, full measurement likelihood with clutter/misdetections, mode pruning/merging, and null-hypothesis mass.

### Candidate support, score comparability, and completeness

Raw PCL NDT `fitness`, transformation probability, or a difference between two scalar scores is not automatically a Bayes factor. Candidates may have different visible source support, map crop, in-map return count, target voxel density, and basin volume. A defensible evidence comparison would need:

1. The same source observations, sensor model, target/map prior, and scoring domain for every candidate.
2. A likelihood for unmatched/outlier returns and a detection/clutter model, rather than silently dropping points that do not find target cells.
3. Candidate prior mass and, if approximating a basin integral from an optimum, a justified local volume term (e.g. a valid Laplace approximation) so that sharpness/support are not confused with peak score.
4. An explicit `B_0` or calibrated retrieval-miss probability. If the candidate generator is not exhaustive and no sound bound is available, the result is **candidate-conditioned ambiguity**, never a global uniqueness proof.

These are principled requirements, not a claimed invention. GSF's SOT likelihood and null hypothesis address closely related clutter/misdetection and missed-candidate concerns. NDT score normalization or a different sensor does not by itself make the contribution new.

## 4. Four controlled states and what joint reporting buys

| State | Controlled construction | Expected observability output | Expected cross-basin output |
|---|---|---|---|
| A: observable + unique | One isolated, geometrically rich map place with one supported basin | Strong, full-rank local information | Posterior concentrated on one candidate; low miss mass |
| B: degenerate + unique | One unique map place with a long straight corridor/plane geometry | Weak direction(s) inside the correct basin | One candidate basin; low miss mass |
| C: observable + ambiguous | Two distant repeated rooms/structures, each with rich local geometry | Strong local information in each basin | Several separated basins with comparable evidence or high miss mass |
| D: degenerate + ambiguous | Repeated parallel corridors with weak axial geometry in each copy | Weak local information per basin | Multiple separated candidates with similar evidence |

The logical non-substitutability counterexamples are straightforward: a local Hessian at one peak can be identical in A and C, so `U_obs` cannot distinguish them; candidate weights can both be concentrated on one mode in A and B, so a nonlocal-only statistic cannot detect B's within-mode weak axis. The pair can classify all four *if* both quantities are valid and calibrated. This establishes utility of reporting both axes, not novelty of the pair or superiority of any decision rule. The experiments must use independent truth labels and ensure repeated geometry creates C/D rather than allowing GT to leak into detection.

## 5. Exactly three proposed contribution points — adversarial assessment

| Point | Closest prior art / exact difference | Mathematical object | Required code and experiment | Independent ablation? | Novelty/risk judgment |
|---|---|---|---|---|---|
| **A. Dual-U reliability formulation and four-state failure taxonomy** | Closest: the 2023 Gaussian-sum localization already represents spatial modes with per-mode Gaussian pose uncertainty, mode weights, and a null hypothesis; AGM separately demonstrates map-conditioned environmental ambiguity. P7's possible difference is to name/measure *directional scan-to-map within-basin geometry* separately from *spatially separated candidate posterior mass* for one NDT observation. That difference is not enough if the outputs are only two flags and their Cartesian product. | Pair `(local conditional information, candidate-mode posterior/null mass)`; four class labels. No meaningful scalar weighted sum is required. | No production change needed to state the taxonomy. A rigorous test would use the four controlled geometries above, report calibration/confusion/abstention, and compare local-only, nonlocal-only, and paired labels. | **YES**, as a diagnostic ablation; it does not independently test a new algorithm. | **Not established as novel.** High prior-art overlap; at most a useful problem framing unless a new falsifiable property beyond mixture decomposition is shown. Risk **HIGH**. |
| **B. Lightweight cross-basin uniqueness estimation** | Closest: GSF's spatial hypotheses + likelihood/null mass; Reliable-loc's MCL/spatial verification and uncertainty monitoring; Autoware MULTI_NDT/MULTI_NDT_SCORE. Possible exact difference: a bounded, same-frame PCL-NDT candidate evidence calculation with an explicit support/clutter model and calibrated candidate-miss mass, without maintaining a full multi-hypothesis trajectory filter. No reviewed source proves this exact engineering instantiation is absent, but replacing a learned matcher/particle filter with NDT is not itself novelty. | Candidate-conditioned basin posterior/evidence `(π_1…π_K, π_0)` under a common generative observation model; conditional on candidate set unless exhaustive retrieval/bounds exist. | Requires a mature spatial candidate generator, candidate clustering, consistent full-source scoring, support/outlier modeling, calibration of `π_0`, and instrumentation. Test repeated-room/corridor maps with independent wrong-basin labels; vary overlap/support, then compare top-1 score, raw margin, local-only, and calibrated candidate evidence for wrong-basin rejection, correct-basin retention, calibration, runtime, and memory. | **YES**; disable the evidence normalization/miss model or compare to a single-candidate localizer while freezing candidate generation. | **Plausible research question, not presently a defensible contribution.** Core mode posterior/null concepts are mature; exact low-compute NDT version and candidate completeness are unproven. Risk **HIGH**. Do not implement until a specific non-equivalent evidence rule is established. |
| **C. Hierarchical Dual-U measurement handling** | Closest: GSF updates each mode with its own Kalman update, changes mode probabilities using map observations/odometry, and withholds availability until hypotheses/null probability meet conditions. The 2026 visual-association preprint also jointly carries association distributions into directional information and selectively uses complementary factors. P7's proposed difference is single-state gating by basin reliability followed by directional local handling only inside an accepted basin. This is currently a composition of mature decisions, not a distinct rule. | A conditional model `p(T|Z,M)=Σπ_k p(T|B_k,Z,M)` with mode acceptance and per-mode directional covariance; to be mathematically stronger than if/else, one would need derive/calibrate a Bayes-risk or integrity decision under explicit costs and candidate-miss mass. | Needs mode-conditioned measurements/covariances or a bounded hypothesis filter; a single IKFoM state cannot represent distant modes. Experiment would compare local-only, mode-only, and hierarchical policy on the four-state data, with false accept/reject, wrong-basin update prevention, degenerate-correct retention, trajectory errors, and resources. | **YES**, but only after the decision rule and state semantics are fixed. | As currently specified: **not an independent algorithmic contribution**; classify as a possible **SYSTEM CONTRIBUTION** if it is only integration. Risk **HIGH**. |

### Can exactly three defensible points be claimed?

**No, not on current evidence.** A is an informative framing but it restates structure already carried by multimodal posteriors. B is the only plausible algorithmic point, but currently describes applying mature candidate-mode likelihood ideas to NDT without a proven new normalized evidence or completeness treatment. C is a mature hierarchical integration unless a new, derived decision criterion is established; counting it as an algorithmic point would mislabel a system contribution.

The four-state experiment is necessary if this line is revisited, but more data, more ablations, an MID-360 deployment, low memory, and code integration cannot turn an already-known mathematical combination into three independent algorithmic contributions.

## 6. Research stop decision

- Overall broad Dual-U claim: `DUAL_U_NOVELTY_NO_GO` as a primary algorithmic contribution.
- Exact PCL-NDT candidate-evidence specialization: unresolved narrow candidate only; no claim that it is novel or that it works.
- Three-contribution structure: **`DUAL-U-NOT-ENOUGH-FOR-PAPER`**.
- Production implementation: **NOT AUTHORIZED / NOT STARTED**.
- New branch: **NOT CREATED**.

If the research group chooses to revisit this, the sole next scientific question should be: *Can a bounded candidate-conditioned NDT evidence model, with common support/outlier accounting and a calibrated missed-candidate mass, reject wrong spatial basins better than mature top-candidate/multi-start baselines while retaining correct but locally degenerate matches at low CPU cost?* This is a falsification experiment for point B, not permission to start downstream integration.

## References and implementation evidence

- Stannartz et al., “Toward Precise Ambiguity-Aware Cross-Modality Global Self-Localization,” IEEE Access 11 (2023), DOI: [10.1109/ACCESS.2023.3286310](https://doi.org/10.1109/ACCESS.2023.3286310). Full text at [TU Dortmund repository](https://eldorado.tu-dortmund.de/bitstreams/0da3e223-e98c-4a05-9109-c40a7787fd44/download). Relevant sections: weighted hypotheses; per-hypothesis Kalman update; complete SOT likelihood for misdetection/clutter; bounded merge/cap/prune; null hypothesis and availability.
- Huang et al., “Reliable and Fast Localization in Ambiguous Environments Using Ambiguity Grid Map,” Sensors 19(15), 3331 (2019), [full text](https://pmc.ncbi.nlm.nih.gov/articles/PMC6695785/).
- Zou et al., “Reliable-loc: Robust sequential LiDAR global localization in large-scale street scenes based on verifiable cues,” ISPRS JPRS 224 (2025), 287–301, [paper/preprint](https://arxiv.org/abs/2411.07815); [public repository](https://github.com/zouxianghong/Reliable-loc). The prior I1 audit pinned code commit `5dccb8b830f49ed989661547cda41412e42a7fd4`; relevant files include `monte_carlo_loc/reliable_loc.py`, `reg_loc.py`, and `sensor_model.py`.
- Autoware `autoware_ndt_scan_matcher`, [core implementation](https://github.com/autowarefoundation/autoware_core/blob/main/localization/autoware_ndt_scan_matcher/src/ndt_scan_matcher_core.cpp), multi-NDT covariance estimation path.
- Zhao et al., “From LiDAR Maps to Visual Localization: Unified Visual Association for Robust Point-Line-Plane Pose Estimation,” arXiv:2609.27363, v1 submitted 2026-09-23, [preprint](https://arxiv.org/abs/2609.27363). Its association-distribution/directional-information claim is treated as preprint evidence, not peer-reviewed publication evidence.
- Local evidence: [`I1_CURRENT_CODE_MAP.md`](I1_CURRENT_CODE_MAP.md), [`I1_MATURE_ARCHITECTURE_AUDIT.md`](I1_MATURE_ARCHITECTURE_AUDIT.md), [`I1_UOBS_NOVELTY_AUDIT.md`](I1_UOBS_NOVELTY_AUDIT.md), [`I2_UOBS_FALSIFICATION.md`](I2_UOBS_FALSIFICATION.md), [`N1_MAP_MATCHING_NOVELTY_DISCOVERY.md`](N1_MAP_MATCHING_NOVELTY_DISCOVERY.md), P6-I4 basin-margin reports, and P6-I6B v0/v1 result reports.
