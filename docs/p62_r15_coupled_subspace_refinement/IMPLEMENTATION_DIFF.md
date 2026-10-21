# Implementation summary

All edits are confined to `docs/p62_r15_coupled_subspace_refinement/` in the R15 research worktree. No stable production source, NDT parameter, frozen map, prior result, EKF, event detector, candidate generator, deduplication rule, or evidence threshold was changed.

- `tools/coupled.hpp`: a read-only exact PCL 1.10 Gaussian score wrapper; fixed association signatures; right/body SE(3) scaled finite-difference derivative model; Gaussian-weighted normalized GN geometry; explicit R6 geometry coordinate conversion; eigen split; weak-only and regularized conditional strong response; finite/PSD/conditioning/trust gates.
- `tools/run_coupled.cpp`: consumes only frozen R13/R14B Sources, maps, T0 and TP; computes NOMINAL, WEAK_ONLY and COUPLED diagnostics; no `align()` calls. Output candidates are explicitly `POST_NDT_REFINEMENT_CANDIDATE`, `converged=0`, `formal_eligible=0`.
- `tools/test_coupled.cpp`: synthetic mathematical and edge-case tests. Test output is recorded in `MATHEMATICAL_TESTS.md`.
- `tools/seal_inputs.py`: before/after input/archive/stable-worktree identities; archive checksum verification; reference file is SHA-checked only, not parsed for pose values.
- `tools/freeze_outputs.py`: validates and seals the 25-frame local model outputs before reference pose-value access.
- `tools/evaluate_reference.py`: reads only the 20 fixed Newer College reference rows after seal verification, uses the already frozen conditional `T_M_B T_B_L` chain, and refuses alignment fitting or timestamp replacement.

The first synthetic directional-derivative implementation was rejected before any R15 frame processing; its amended central-difference method, stencil sizes, and gates are frozen in `MATHEMATICAL_FORMULATION.md`. The only post-candidate-freeze source edit was a reference-evaluation script inventory-key correction after it failed before parsing any reference pose values. It changed no C++ source, model, candidate, parameter, or sealed output; see `PROCESS_EXCEPTIONS.md`.

Algorithm source SHA convention: SHA-256 of sorted lines `relative-path  file-sha256\n` for `tools/CMakeLists.txt`, `tools/coupled.hpp`, `tools/run_coupled.cpp`, and `tools/test_coupled.cpp`; resulting run CODE_SHA is `322dd58ee84b7f25aae745e13e535e070cfa82ee5f7809c9f0e8c6b1f97dfe30`.
