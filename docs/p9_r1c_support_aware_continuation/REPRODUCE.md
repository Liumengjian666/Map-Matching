# Reproduce R1C

From the paper workspace, use a fresh output directory for a new run. Existing
formal CSVs are never silently overwritten by the execution entry point.

```bash
cmake -S src/dog_prior_map_fastlio2_frontend_exp/scripts/p9 -B /tmp/p9_r1c_build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/p9_r1c_build -j2
(cd /tmp/p9_r1c_build && ctest --output-on-failure)
python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/run_support_aware_continuation.py --runner /tmp/p9_r1c_build/p9_support_aware_continuation --output /tmp/p9_r1c_reproduction
git diff --check
```

Runner pins all six R1B input SHA256 values before and after execution. It sets
LD_LIBRARY_PATH=/lib/x86_64-linux-gnu (the existing P9 runtime requirement).
Source/target filtering, score constants and map are unchanged. No GT files
are opened. The verification/summarization command regenerates derived JSON
and empty, conditionally untriggered CSVs from fixed evidence, without calling
the solver:

```bash
python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/run_support_aware_continuation.py --verify
```

7 CTests cover old P9 math, R1A/R1B tests, R1C numerical/helper/support-flow and
independent certificate-audit regression. Real FD results are not unit-test
assertions forced to pass:41/42 is archived as FAIL. Formal runtime varies.

The preliminary --probe run was executed once before formal --run; its14
correctors used identical h, tolerances and budgets. No extra full NDT ran.
Conditional continuation/recapture/barrier/spawn/refine work was not executed
after failed roots. No inference relies on endpoint complete-link labels.
