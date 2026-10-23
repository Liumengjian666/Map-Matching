# R21 necessary implementation checks

CMake Release build succeeded. One C++ test executable passed: strict three-component Pareto, finite nominal comparability, physical difference, deterministic relative ranking, trusted/pending state isolation, single-pose causal shadow seed, confirmed current-pose-only update, shared search/confirmation budget and nominal absolute condition. Synthetic tests call no NDT align.

Read-only code review confirmed separate D/E causal histories, actual confirmation ID3 versus vector index1, absolute-nominal priority (user explicitly confirmed cancellation without extra call), next-frame-only real Full confirmation, pending exclusion from accepted history, per-frame/per-trajectory budget and reference-free native code. Review identified incomplete decision substage timing; before any native experiment this was corrected to include branch work while subtracting separately measured extra align/fitness/validation work. Total frame wall clock always includes all computation. Final rebuild passed.

Policy scope: no production or historical code changes; only new R21 sources. The formal run will not reuse R20 frozen per-frame outputs as new predictions. Input/code and native-output seals are distinct; reference evaluation requires the complete output seal.
