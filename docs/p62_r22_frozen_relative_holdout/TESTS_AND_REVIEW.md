# Necessary R22 checks

Release compilation passed. Initial CTest invocation used unsupported --test-dir on this host and found no tests: preserved in logs/TEST.log, not counted as passing. Correct invocation from build cwd passed dispatch 1/1 (logs/TEST_ACTUAL.log), no NDT calls. Debugging was limited to this invocation error.

Local code review uses the code-review-and-quality checklist: all mathematical/quality functions imported unchanged; four fresh Predictor instances per initialization, no online reference input; separate actual Full calls for all methods; baseline uses R19 basic quality, not the old5degree feedback gate; C and D have the same frozen search and ranking; D alone invokes R21 strict relative comparison if absolute candidate absent. No E execution path. New runner records state lineage, all raw candidates, and full per-frame costs. Headers/parameters/Source and map identities are frozen before native calls. No new dependencies or production edits.

Map preflight uses the unchanged R19 implementation. All244 exact PCD timestamps match; only four anchor pose values parsed. Native loader checks original declared point-byte identity and excludes known zero padding. Four fixed50m/.20m maps have anchor overlap >=.50; no evaluation GT used for ROI. See map receipts for actual values.
