# R4 source-cloud provenance recovery

FINAL_RESULT = R4_SOURCE_CLOUD_PROVENANCE_CLOSED
NEXT = R4_RESUME_HELDOUT_ORACLE_FROM_RECOVERED_SOURCES

This is provenance closure only. R4 scientific result remains NOT_RUN.
No oracle263, B12 candidates, visual extraction or GT were run/loaded.

## Git and implementation

Branch: `research/p9-r4-heldout-visual-evidence`
Start: `e66009c8c5e3b9addd7329acfd9e68db6a81f4fb`
Frozen replay source: `9945c4f5c3d7759104de108a594bcaf2553fd78c`
Workspace: `/tmp/dog_loc_paper_r4_ws.Fq21k2`
Detached replay worktree: `/tmp/p9_r4_source_replay_ws.KUx4Q7`
External cache: `/tmp/p9_r4_same_objective_source_recovery.l6hlb3lc`
Binary SHA256: `44837ab84857a9775429fba3c5f2bd6eb5ee67ecda05451a38a10aa9ca61702b`

Only export predicate/192-ID table and expected count changed. No production runner or R4 algorithm was replaced.
End SHA is reported by git rev-parse HEAD after commit (not embedded in its own content-addressed archive).

## Ordered parity gates

| Gate | PASS | Full denominator | Evaluated |
| --- | ---: | ---: | ---: |
| historical_raw_parity | 32 | 32 | 32 |
| full_source_trajectory_parity | 4127 | 4127 | 4127 |
| heldout_prepared_source_parity | 160 | 160 | 160 |

Manifest non-source parity: {'passed': 2400, 'rows': 160, 'status': 'PASS', 'total': 2400, 'unchanged_fields': 15}

TX2932 expected/recovered points: 413 / 413
TX2932 expected/recovered hash: 3530993910003886054 / 3530993910003886054
TX2932: PASS

## Provenance-only cost

| Metric | Seconds / calls |
| --- | ---: |
| Baseline data NDT calls | 4127 |
| Frozen P7 synthetic test alignments (separate) | 2 |
| Wrapper wall | 124.151204761 |
| Prediction + deskew | 15.932621196 |
| NDT alignment | 78.669583528 |
| NDT total including preprocessing | 90.087494445 |
| Cloud input I/O | 4.429121815 |
| Export I/O upper bound only | 2.604194689 |

Export I/O is NOT_SEPARATELY_MEASURED; the bound includes diagnostics, setup and polling.
Delivery ledger uses explicit seconds keys/units; original frozen-audit cost receipt is retained unchanged.
The wrapper wall includes progress-polling tail latency. These are not online-method runtimes.

## Verification and preservation

Release P7 build; P7 tests3/3; P9 tests38/38; selection192/192;
integer-hash/manifest and canonical-guard self-tests PASS; CSV/JSON/hash checks PASS.
Initial P9 invocation hit two libusb loader errors; fixed only the invocation's library path, not code or parameters.
Original blocker files and R4 code remain unchanged, including the old topic-source manifest.
Recovery manifest retains cloud_data_sha256 as legacy topic payload metadata, not recovered raw SHA.
Raw192 exports and full replay outputs are outside Git in /tmp. Preserve this cache for the next task.
The bundle contains code/receipts/ancestry, not raw clouds. Original T0, initial state, U_obs and score remain authoritative.
