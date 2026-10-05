# H2 predeclared matched-budget contract

H1 is frozen at a4a89dd3d775a250a58fbdbf2b8d4838594eafa8. DUAL-U remains within-basin observability plus competing-basin ambiguity; this experiment tests only a proposed discovery mechanism. The PCL objective is not a closed posterior NLL. No GT, continuation, fusion, or probability weighting is used.

## Proposals and numerical convention

Nine fixed frames use all 263 actual archived start poses, not reconstructed historical perturbation parameters. T0 is the archived single nominal registration terminal: nominal registration -> U_obs(T0) -> nonlocal search is the intended use, so T0 is not oracle leakage. Ascending-curvature columns q1,q2 form W; q5,q6 form S. The map product chart is eta=[(t-t0)/0.8; Log_spatial(R R0^T)], not a body SE(3) log. Reuse the P9 Matrix4f carrier and poseAtEta, validating all 2367 round trips at 1e-5 m / 1e-4 deg. Archive pose quaternions are normalized in double and converted to Matrix4f exactly as in the original align wrapper.

FULL6D uses the archived start and result. WEAK2, STRONG2, and RANDOM2 project eta by QQ^T, without renormalization, range expansion, deduplication, adaptive ordering, or extra seeds. Every index consumes one call even if projected starts coincide. Duplicate reporting counts all near-duplicate pairs and participating proposals at the same round-trip thresholds, not a new clustered seed pool.

## Frozen oracle and align

The 22 frozen major identities and highest-score real archived canonical terminals are taken from R1A oracle_terminal_requests.csv, exactly matching the H1 archive. R1A closed/refined terminals that changed the original basin are not substituted. Major definitions and clustering are not altered; iteration-limit basins P03 at TX616 and P06 at TX3796 remain. Admission is <=0.2 m AND <=2 deg. Overlapping admission balls, if any, are diagnosed with pair separations; a terminal is assigned to only one qualifying basin by minimum squared normalized translation/rotation distance, then lexicographic cluster ID. This is assignment, not reclustering. Nonconverged calls consume budget and cannot recover a basin; converged iteration-limit returns are retained under the user's frozen recovery rule.

PCL 1.10, resolution .8, step .08, epsilon 1e-5, max80, default outlier ratio .55. Frozen source preprocessing: finite XYZ, inclusive .5-80m, .25 voxel, deterministic inclusive 1400-point cap. Target: finite XYZ, two .15 voxel passes; resolution is set before target. Source point count/FNV and target549606 are checked. Source/map/archive SHA256 are checked before and after. Exact raw terminal score is evaluated with the P9 value-only evaluator at the normalized quaternion terminal carrier, matching the historical wrapper. No optimizer except the specified projected-seed standard NDT calls is used.

## Randomness and compute budget

PCG64(20261009) yields 100 permutations of 0..262, shared across every frame and method. RANDOM2 has five fixed Haar bases per frame, Gaussian thin QR with positive diagonal convention, PCG64(20261010+1000*ascending_frame_index+replicate). Freeze manifests before align. Each projected pool is run once: 2367 WEAK2,2367 STRONG2,11835 RANDOM2; reuse2367 FULL6D. No current FULL6D rerun.

Budgets are 1,2,4,8,16,32,64,128,192,263 incremental calls/frame. Total online calls=1 nominal+B. Within a frame, recall is recovered major IDs / frame major count. Across nine frames macro recall weights frames equally; micro recall uses22. All100 permutation curves are post hoc on the fixed run pools. Main cost is calls and then iterations. Historical FULL6D wall times are descriptive, not paired current-run timing evidence. Projected NDT runtimes cover only align(), excluding source/map preparation, objective measurement, proposal generation, U_obs, and nominal registration; report these exclusions.

## Statistics and decision (frozen before execution)

Normalized trapezoidal AUC in log2(B) over1..263 for each frame/permutation. For each fixed method, median over100 permutations per frame, then mean over9 frames. RANDOM2: median over100 per frame/replicate, then median over5 bases per frame. Exact one-sided sign-flip tests enumerate512 signs on the nine paired frame differences, using >= observed with1e-14 equality tolerance. These frame-level tests do not make100 permutations or22 basins iid samples. Conditional finite-cohort evidence only; temporal dependence of frames in one sequence remains a limitation.

Matched macro-recall targets .25,.50,.70 use the first budget reaching target per permutation. Report reached fraction as well as conditional median/P05/P95; never drop NOT_REACHED silently. Efficiency factors compare median costs only when all permutations for both fixed pools reach target; at final all seeds the pool is identical across permutations. RANDOM2 reports per-replicate costs and pooled descriptive distributions, not independent new frames.

H2 A-H: weak macro AUC>full,strong,random; paired weak-random and weak-strong p<.05; weak final macro>=.70; factor70>=2 when both reach, or explicitly weak reaches/full does not; weak>full in>=7/9frames. PASS means WEAK_GUIDED_DISCOVERY_EFFICIENCY_SUPPORTED. Weak>full but no significant weak-vs-random/strong evidence means GENERIC_DIMENSION_REDUCTION_ONLY. Otherwise WEAK_GEOMETRIC_PRIOR_SUPPORTED_BUT_DISCOVERY_GAIN_NOT_ESTABLISHED. Contract failure means H2_MATCHED_PROPOSAL_CONTRACT_FAIL. No claim of universal NDT efficiency.

Secondary Spearman compares nine H1 weak rho values against weak-minus-random frame AUC, with all9! permutations, two-sided absolute correlation. Rank ties use average ranks. It is secondary, with no GT or outcome-driven cohort selection.
