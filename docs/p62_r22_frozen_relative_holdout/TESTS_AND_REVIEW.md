# Necessary R22 checks

Release compilation passed. Initial CTest invocation used unsupported --test-dir on this host and found no tests: preserved in logs/TEST.log, not counted as passing. Correct invocation from build cwd passed dispatch 1/1 (logs/TEST_ACTUAL.log), no NDT calls. Debugging was limited to this invocation error.

Local code review uses the code-review-and-quality checklist: all mathematical/quality functions imported unchanged; four fresh Predictor instances per initialization, no online reference input; separate actual Full calls for all methods; baseline uses R19 basic quality, not the old5degree feedback gate; C and D have the same frozen search and ranking; D alone invokes R21 strict relative comparison if absolute candidate absent. No E execution path. New runner records state lineage, all raw candidates, and full per-frame costs. Headers/parameters/Source and map identities are frozen before native calls. No new dependencies or production edits.

Map preflight uses the unchanged R19 implementation. All244 exact PCD timestamps match; only four anchor pose values parsed. Native loader checks original declared point-byte identity and excludes known zero padding. Four fixed50m/.20m maps have anchor overlap >=.50; no evaluation GT used for ROI. See map receipts for actual values.

Before native execution, local Python was confirmed3.8.10; two dictionary-union expressions in the posthoc evaluator were replaced by compatible dictionary merging. No evaluator or NDT had run. The preliminary algorithm identity is retained as ALGORITHM_FREEZE_PRE_EVALUATOR_COMPATIBILITY.json; final ALGORITHM_FREEZE.json binds the corrected evaluator before any align. The C++ binary and mathematical policies were unchanged.

## Completed native and result checks

All32 independent60-frame trajectories completed once:1920 nominal+124 extra=2044, no E/Split/Coupled/EKF. Native seal commit preceded reference evaluation. Final local review checked all32 history timestamp chains and budgets, every2044 candidate/call identity, frozen source/binary and stable-workspace snapshot. It independently recomputed all32 output RMSEs from sealed error CSV; rejected selected observations are NA. Source/map before-after identity checks passed in the output seal. Only new R22 paths differ from START. No extra native runs were used for review.

The report distinguishes all-four D-vs-A C0 safety PASS from W1/C0 D-vs-C safety counterexample. Only W1 has relative-only admissions:raw2626 C0 andraw2601 C3. C/D outputs first diverge at exactly those frames; other three windows are exactly equal for each initialization. Full32 trajectory tables, complete W1 C/D chain and balanced plots retain the late C0 failure. The .030641219m D-vs-C RMSE worsening is only .000641219m above the predeclared .03m report criterion; extra high-error frames and longest-fallback worsening are also reported, without rerunning to tune the boundary. No novelty or independent physical accuracy claim.
