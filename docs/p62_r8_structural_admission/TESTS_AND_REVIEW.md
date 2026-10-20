# Tests and local adversarial review

## Completed before the first R8 NDT

- Actual C++ compatibility test compared the frozen R7 loadSource/split path against R8's streaming adapter on 200 existing real fixtures (R6's 100 and R7's 100). Point order, full Source FNV/count, training FNV/count and all eight validation blocks matched. Receipt: COMPATIBILITY_TEST.json. No NDT align executed by this test.
- Full native precheck processed all 10905 original frames; 10900 historical Source count/FNV pairs matched, zero mismatches. Native output is preserved separately from enriched admission records.
- Four Python admission tests pass: preserve original motion ordering; require all 100 frames; include the final boundary frame; missing causal initial cannot become legal through identity fallback. These tests exercise selection logic, not cloud preprocessing or optimizer correctness.
- Local fresh-context precheck review found dependency-freeze and completion-state guard gaps. They were fixed before the full precheck: legacy files compared against START_SHA, compatibility receipt bound to actual binary/fixtures, and HEAD/status plus assets checked after execution.
- Local independent selection review recomputed all 402 accepted windows and their order, selected raw 4982–5081 at original rank 1039, and verified all 100 native point/sector counts plus exact causal initial fields. It requested a fresh full asset gate immediately before optimization. run_experiment.py supplies full hashes and exact historical initial comparison both before and after the one run.
- No external CLI or cross-model service requested by the user; none executed.

Post-run accounting, any failures, independent result checks and final asset checks will be appended without replacing these historical facts.
