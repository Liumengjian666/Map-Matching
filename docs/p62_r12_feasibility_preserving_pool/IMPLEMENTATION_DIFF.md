# Actual implementation delta

Only docs/p62_r12_feasibility_preserving_pool is changed.

- tools/pool_v3.hpp: original raw copy plus derived canonical view; direct frozen admissible before grouping; unchanged duplicate comparator; fixed-anchor greedy membership; minimum eligible ID representative; explicit raw nominal evidence union; real opponents and strict paired>.02; final admission recheck. No optimizer or new seeds.
- tools/replay.cpp: load precise frozen R11 matrices/metadata, all8 block records and prediction. CONTROL calls actual old helpers and verifies their historical outputs. V3 uses the new header. Outputs mapping, representatives, evidence, selectors, final comparisons and timings. No original CSV overwritten; output directory must not exist.
- tools/test_pool.cpp:27 real C++ checks. Tests use synthetic fixtures only, never labelled as new true localization evidence.
- execute.py: hashes against R11 frozen Git identity and checksums; readonly production snapshot; exclusive build and replay directories; actual build/test provenance. A reviewed wrapper binding permits only wrapper changes while proving C++ sources/binary unchanged. Process failures/timeouts still retain exit, AFTER and failure receipt; no retry. No Bundle operation, no ROS or NDT executable invocation.
- summarize.py: offline arithmetic/LP and lineage verification of completed outputs; does not call the C++ replay or a selector. Exclusive result writes prevent silent overwrite.

R6 contract.hpp, R9 coverage.hpp, R10 events.hpp and R11 shadow.cpp remain unchanged. Inherited headers include PCL definitions, but no NativeNdt instance, align(), geometry(), validation NN, map load, Source preprocessing or candidate generation is executed in R12. `validate()` semantics are consumed through its frozen records and actual `admissible()` recheck, not recomputed from point clouds.

Group-level PHYSICALLY_DISTINCT_FROM_NOMINAL is intentionally different from decision-level PHYSICALLY_DISTINCT_RECOVERY. A group representative need not be supported or accepted. Singleton robust victory is prohibited by the explicit raw nominal competitor and positive opponent count.
