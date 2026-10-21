# R13 tests and bounded review

Actual final build receipt records64 C++ assertions and5 Python mock tests passed. Tests run zero native aligns. Synthetic assertions are not physical candidate or accuracy evidence.

C++ tests cover all45 float bounds/order, SE3 translation Jacobian/right frame convention, deterministic256 endpoints/one point, odd-sector rejection, NN query count/scaling, .5m equality and saturated losses, strict local .25m/4deg versus inclusive .30m/4deg diversity, sorting/ties,0/1/2 seed paths, invalid scores and NN results. The score-saturation counterexample remains a limitation, not a new tuned gate.

Coarse artifact adversarial review2 cycles: moved BLOCK_LOSSES reading after complete coarse selection; added boundary tests; retained uninformed all-saturated deterministic selection as explicit limitation. No remaining blocking coarse issue found.

Runner artifact review2 cycles: replaced timeout wrapper with explicit Popen kill/wait and true launched count; separated exit/AFTER/hash/copy failures; RUN FINALIZATION remains attempted on failures.5 mock tests cover success, nonzero, timeout reaping, failed launch and isolated archival failure. Final directed read-only review found no blocking issue. No real process failure was injected.

First build failed at linker with unresolved libusb_set_option in libpcl_io. Failed source96c202823... and original frozen_01 logs/link command retained. Actual R11 CMake already explicitly linksusb-1.0; R13 restored that dependency, used new frozen_02 and passed. No native R13 process had started. No system installation or production edit.

One local result reviewer independently read CSVs/identities without running scripts or algorithms. It confirmed225 scores,10 calls,5 groups across4 events,0 legal/accepted and12 directed LP values; flagged costs excludingSource, zero-grid exact-T0 duplicate, near1deg physical differences, and the limited scope of reference discovery. Reports retain all these counterexamples. Review did not certify absolute correctness or different optimization basins.

Offline independent matrix expm checks225 seed matrices/actual float casts and redoes deterministic sorting from recorded scores. It does NOT independently recompute225 NN scores. LP verification applies to12 directed records, not12 independent observations. Source/FNV/point records and training-only function guards support provenance of the real scorer.

One post-review arithmetic/report refinement preserves INITIAL_NUMERICAL_RESULTS.json and changes no raw result/C++/parameters. The sole actual native experiment remains10 calls. External second-opinion option was offered; no external CLI was authorized/executed in this task.
