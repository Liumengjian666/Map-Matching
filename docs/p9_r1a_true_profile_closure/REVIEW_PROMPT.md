Independently adversarial-review PAPER-P9-R1A-TRUE-PROFILE-AND-ORACLE-CONTRACT-CLOSURE.
Find correctness or scientific-contract failures, not validation or a summary.

Read-only scope:
- src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/p9_true_profile_closure.cpp
- src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/p9_strong_profile_solvers.hpp
- src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/prepare_true_profile_oracle.py
- src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/summarize_true_profile_closure.py
- the diff of p9_ndt_energy_contract.cpp and CMakeLists.txt
- docs/p9_r1a_true_profile_closure THEORY.md, REPORT.md, results.json and CSV sidecars
- frozen P9-R1 profile sidecars under docs/p9_dual_u_nonlocal_r1/sidecars/profile_cohort_final
- historical cluster implementation via git show 9945c4f5c3d7759104de108a594bcaf2553fd78c:src/dog_prior_map_fastlio2_frontend_exp/scripts/p5_i1_cluster_modes.py
- if needed, the frozen Floor01 same-objective candidates/clusters/cohort/dual_u CSV files referenced by oracle_preparation.json. Do not read any GT/posthoc_GT inputs.

Contract:
Frozen22 major cluster IDs. Representative must be a real highest-score member, exactly reconstruct complete-link membership and provenance. One canonical NDT refine, disqualify failed refines; report threshold departure separately. Do not silently discard original22. GROUP A is closed-pose weak projection>=.8 AND inside the original region. Fixed eta=[map translation/.8,map left rotation], fixed W/S. At true u_b compare original one-step, iterative Newton20, derivative-free100 value evaluations; zero-v and oracle-v diagnostic starts. Real dynamic score only controls acceptance. No strong-coordinate cap for new solvers. At most one full NDT refinement per endpoint. Recovery uses .2m AND2deg and successful refine. Initial/accepted/budget/gradient provenance must be correct. FD must distinguish branch-local derivatives from changing-support energy. No individual zero-v solver reached70%, so no adaptive grid or transported W. Exact optimizer energy is not posterior NLL; no probabilities, tensor fusion, GT tuning, EKF or production modifications.

Recompute aggregate recovery, GROUP A, all counts, gating, pose-distance fields and claimed mechanisms from sidecars. Check native Euler pullback versus global exponential chart and local endpoint chart. Check that conclusions do not certify min_v or dynamic stationarity. Report actionable findings with file/line evidence, or explicitly state no actionable findings after thorough examination.

Do not write files, use network, invoke optimizers/replay, run Git writes, or spawn agents. Small read-only Python/NumPy calculations from the listed CSV files are permitted.
