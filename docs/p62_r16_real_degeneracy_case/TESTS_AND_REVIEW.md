# Bounded execution and local review

The review used the code-review-and-quality skill locally, restricted to input identity, prediction/reference separation, native-call accounting and output interpretation. No external AI/CLI review or additional experiment was run.

## Verified

- Real C++ targets compiled against PCL 1.10.0. `run_ndt.cpp` and its executable still match every pre-run frozen source/binary SHA.
- One input-only fixed-stride prescreen: 726 sampled and 756 unique PCD reads, cap 800. One fixed 31-frame window.
- Native PCL read and original-byte/XYZ-bit agreement for all 31 selected PCDs; each has 3,846 zero padding bytes beyond the declared records. The selected Sources equal the pre-registered FNV/point counts.
- Exact timestamp joins for all 31 frames. Only the anchor pose values were parsed before native NDT. `run_ndt.cpp` reads no reference file; the evaluation program verifies the frozen native output hashes before parsing evaluation poses.
- 30 Full +30 Split returned native calls; extra calls 0. No recovery, Coupled, EKF or GT-based initialization after the anchor.
- Common TP uses only past Full admission results. TP19 equals Full17 exactly after Full18 rejection.
- Archive-facing CSV row widths, 60 NDT rows, 30 geometry/event rows, 60 call-ledger rows and 90 error rows pass the final consistency check.
- Selected 31 PCD and official map/reference before/after identities agree; immutable native output hashes agree. Existing R6–R15C tracked files have no changes relative to START.
- All Python sources parse. No new synthetic mathematical suite was added.

## Failures retained and corrections

1. The first prescreen process failed before `main` due to the existing PCL I/O/libusb loader conflict. Its failure log is retained. The same binary then used R14B's existing `LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 LD_BIND_NOW=1` environment. The prescreen itself ran once.
2. The prescreen raw CSV first-column header is mislabeled one-based while values are zero-based. The raw file remains unchanged. The preregistration helper was corrected using canonical path/time identities, without another selection. See `PRESCREEN_FIELD_AUDIT.md`.
3. Build preparation first referenced the not-yet-created runner, then used one too many `../` segments. The map reader also needed `&text[0]` for C++14's writable string storage. These were fixed before any native run.
4. `map_preflight_01` stopped with `MAP_PREFLIGHT_STOP map::at`: the first SCANS schema lacked the later expected identity columns. It opened/stat'ed the first PCD but did not read declared point records, call PCL source loading, load the map or align. Its header-only receipt remains. `SCANS_V2.csv` was derived from the unchanged preregistration; `map_preflight_02` then completed all format tests and the sole ROI build.
5. Version recording initially tried unavailable `pcl_config`, then unavailable `pkg-config pcl_common`. The freeze script now records the actual included `/usr/include/pcl-1.10/pcl/pcl_config.h` version. These failures occurred before the freeze was written and before NDT.
6. The immutable native runtime CSV contains one extra zero in the map-startup row only. `tools/finalize_records.py` checks that exact defect and writes the normalized top-level `RUNTIME_AND_MEMORY.csv`; the original raw CSV and its frozen hash remain. Per-frame timings, NDT poses and geometry were not edited.
7. The fresh isolated clone had no Git commit identity. Local-only identity was set to the already configured paper-workspace identity `Codex <codex@local>`. The source archive commit was then created; no global config or stable-workspace config changed.
8. In the raw PCD preflight CSV, the column named `declared_records_bytes` reports the end byte of the declared region including the 250-byte header (1,900,794), rather than payload bytes alone. The actual payload is 65,536 ×29 =1,900,544 bytes; `inspectTail()` uses this correct value to establish the 3,846-byte padding. This label defect does not alter parsing or Source values and the raw receipt is retained.

## Runtime interpretation and remaining quality limits

Actual compiler flags were `-std=gnu++14`; CMake build type and general CXX flags were empty. No optimization-level change or timing rerun followed the experiment. Timings describe this unoptimized prototype only. The R9 G matrix is the inherited Gaussian geometric approximation with `[0.8I,-skew(Rp)]`, not a native Hessian or calibrated covariance. The source archive commit was made after the run; pre-run byte hashes supply the actual execution identity.

The native runner and raw reporting source were preserved as executed, including the disclosed startup-row defect. The requested result CSV is a traceable normalization. The stable workspace was not written; a separate production-state snapshot was not collected, so a measured before/after production PASS is not asserted.

Final local review finding: no unresolved issue changes the reported 60 poses, 30 spectra or reference-relative error metrics. Scientific limitations remain explicit in REPORT.md. No merge, push or next-stage experiment was performed.
