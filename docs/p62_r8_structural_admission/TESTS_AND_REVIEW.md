# Tests and local adversarial review

## Completed before the first R8 NDT

- Actual C++ compatibility test compared the frozen R7 loadSource/split path against R8's streaming adapter on 200 existing real fixtures (R6's 100 and R7's 100). Point order, full Source FNV/count, training FNV/count and all eight validation blocks matched. Receipt: COMPATIBILITY_TEST.json. No NDT align executed by this test.
- Full native precheck processed all 10905 original frames; 10900 historical Source count/FNV pairs matched, zero mismatches. Native output is preserved separately from enriched admission records.
- Four Python admission tests pass: preserve original motion ordering; require all 100 frames; include the final boundary frame; missing causal initial cannot become legal through identity fallback. These tests exercise selection logic, not cloud preprocessing or optimizer correctness.
- Local fresh-context precheck review found dependency-freeze and completion-state guard gaps. They were fixed before the full precheck: legacy files compared against START_SHA, compatibility receipt bound to actual binary/fixtures, and HEAD/status plus assets checked after execution.
- Local independent selection review recomputed all 402 accepted windows and their order, selected raw 4982–5081 at original rank 1039, and verified all 100 native point/sector counts plus exact causal initial fields. It requested a fresh full asset gate immediately before optimization. run_experiment.py supplies full hashes and exact historical initial comparison both before and after the one run.
- No external CLI or cross-model service requested by the user; none executed.

Post-run accounting, any failures, independent result checks and final asset checks will be appended without replacing these historical facts.

## Completed after the only Shadow run

- Native process returned0, same-process SMOKE20_PASS and COMPLETE receipts present. Exclusive tokens were retained. Full end guard passed for inputs, source, binary, PCL, R6 outputs and production HEAD/status.
- 10 Python unit/record tests pass (4 admission + 6 post-run/independent mathematical checks). The post-run tests account for all416 START/RETURNED journal rows, keep formal/forced results separate, check common singleton pools/NA margins, retain all8 duplicate recovery outputs, verify native-output hashes/copies, and enumerate all70 capped-simplex vertices on a synthetic paired-weight counterexample. These tests call no NDT or EKF.
- analyze_results.py independently verifies all100 Source identities/times/counts; trigger values from recorded8-block losses; exact cached nominal matrices; physical seed limits; duplicate separations; three-selector pool equality; no invalid-native records; and the208 actual native calls. The original native outputs were hash-frozen before this analysis. Original poses are never replaced. Diagnostic angle checks use orthogonal polar factors solely to remove float matrix roundoff for reporting.
- Unmodified R6's earlier18-category synthetic/PCL test receipt is historical evidence, not rerun or counted as new R8 tests. In particular its2 synthetic aligns are not part of this run; R8 actual new aligns remain208.
- Final result and archive review plus final asset-gate receipt will be recorded separately. No external CLI executed.

## Final independent local review

The same fresh-context reviewer completed three bounded selection/run/result review cycles. Final independent recomputation confirmed 10905/3338/402, 208=100+100+8 native calls, zero natural events, four valid diagnostic G spectra, eight duplicate-to-nominal recoveries, 104 common singleton pools, NA margins and NOT_EVALUABLE. Mean/P95/max and peak108884KiB matched the frozen CSVs. All native/copy output hashes matched; START_SHA and current diffs were R8-only. No material unresolved defect was found. The reviewer did not execute native programs, read GT or write files. This is local contract/result review, not an independent localization accuracy test.
