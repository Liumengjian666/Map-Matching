# Verification and bounded review

Release standalone build: /tmp/p10_r2_build.CdJ4jf, CMAKE_BUILD_TYPE=Release.
Pinned FAST-LIO2 7cc4175de6f8ba2edf34bab02a42195b141027e9. Threads fixed to one.

Commands actually used:

    cmake --build /tmp/p10_r2_build.CdJ4jf -j2
    (cd /tmp/p10_r2_build.CdJ4jf; LD_LIBRARY_PATH=/lib/x86_64-linux-gnu ctest --output-on-failure)
    cmake --build /tmp/p9_r4_release.Eirto1 -j2
    (cd /tmp/p9_r4_release.Eirto1; LD_LIBRARY_PATH=/lib/x86_64-linux-gnu ctest --output-on-failure)
    (cd /tmp/p10_coupled_build.jxTN7w; LD_LIBRARY_PATH=/lib/x86_64-linux-gnu ctest --output-on-failure)

P7/P10 tests 6/6 PASS for both runtime versions. P9 41/41 PASS; R1 2/2 PASS.
New tests: actual post-update seed; valid anchor not reset to NDT; true causal
interval; duplicate propagation rejection; frozen W; three contributions and
two future confirmations; meaningful equality rejection; missing anchor zero
backend work; exact2s boundary; expired pending and no same-frame reseed; spatial
SO3 and scaled map translation; nonrigid rejection; R4 diagnostics independent
and cumulative invalidity sticky; feedback consumption; final version's
two-existing-terminal anchor-aware rank without extra alignment.

One fresh-context read-only review, bounded to three cycles. Findings reconciled:
R4 diagnostic veto leakage fixed by separate R3 confirmation/diagnostic path;
post-update seeding and before-clear frozen-W snapshots checked; ordinary expiry
same-frame reseed fixed with invalidation stamp; cumulative diagnostic validity
made sticky. The old R3 alternative propagation is explicitly preserved, not
misrepresented as IMU-only. Cross-model second opinion was offered; user chose
only the current independent review and continuation. No external CLI executed.

The incremental, Git, doubt, debugging and code-review skills guided changes and
checks; their auxiliary shared references were absent. No extra research audit
or theoretical certification was run. The failed independent Python carrier
audit and corrected audit are both archived, with unchanged tolerance and no
repeated real replay; see AUDIT_CORRECTION.md.

Runtime guards independently verify all4127 IDs, budgets, zero work on ordinary
and anchor-unavailable frames, true anchor propagation, constant event W,
individual and cumulative chart costs, R4 diagnostic arithmetic, exact shadow
source/measurement/filter state parity, single pending consumption, and next
prediction divergence after actual feedback. These checks freeze before GT.

Coupled math, temporal R3/R4 implementation, IKFoM, time/deskew/readers and geometry
remain byte-exact to START. CMake only links the new anchor module into the
existing opt-in research library; no production node enables anchor feedback.
Production ROS launch/deployment is NOT_RUN. Standalone real replay is the
integration under test, not a production localization claim.

Scientific versions contain one full shadow and one full feedback each; no
extra CONTROL/source replay, raw extraction, Oracle/B12/Corridor/visual work.
Full content hashes plus committed-source/binary/rule lineage and finite
JSON/CSV-width/denominator audit are delivered in artifact_hashes.json.
