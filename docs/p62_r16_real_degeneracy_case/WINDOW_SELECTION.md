# R16 window-selection contract

This rule is frozen before any pre-screen PCD is read. It uses only the canonical R14B scan inventory and raw scan geometry; it does not read reference pose values, map correspondences, NDT outcomes, or localization errors.

1. Use the 15,301 rows in `docs/p62_r14b_newer_college/LOCAL_SCAN_INVENTORY.csv` in their existing timestamp order. A stray `(1).pcd` not present in that canonical inventory is not a sequence frame.
2. Find the first scan at or after 5% of the integer-nanosecond sequence time span.
3. Starting there, inspect fixed-stride-20 scans, stopping before the sequence tail so a 31-frame window can be centered. The complete prescreen plus any uninspected frames in the selected 31-frame window is capped at 800 unique PCD reads.
4. For each inspected PCD, apply the frozen R14B PCL binary-PCD reader and the same preprocessing: finite XYZ, range 0.5–80 m, 0.25 m voxel, endpoint-uniform cap at 1,400 points. In the resulting scanner-frame source, calculate the 3-D sample covariance eigenvalues in ascending order. The selection score is `lambda_min/lambda_max`; the smallest score wins, exact ties go to the earliest ordinal. This is only a raw-input anisotropy proxy, not the R9 map-conditioned NDT geometry matrix and not a degeneracy decision.
5. The unique window is the 31 consecutive canonical scans centered on the winning sampled scan, shifted only as needed to remain within the post-5%-eligible sequence bounds. The 31 identities and hashes are frozen before any NDT. The anchor is frame 1; the following 30 are evaluations. If the frozen window contains any invalid input, stop; do not pick a replacement.

Prescreen implementation is `tools/prescreen.cpp`. It emits all sampled and selected-window geometry rows, records the number of unique reads, and fails if the 800-read cap is exceeded. All time values used here come from PCD filename timestamps in the frozen inventory; no registered pose values are read.

## Selection result

The one completed prescreen inspected 726 fixed-stride samples and 756 unique PCDs including the selected window, below the 800-read cap. The winning sample is canonical index 7566, one-based ordinal 7567, with covariance ratio 0.0062502250993394246. The unique 31-frame window is one-based ordinals **7552–7582**: anchor 7552, evaluations 7553–7582. Its time support is 1583837346270734080–1583837349270243328 ns. No alternative window was evaluated.

`TEST_FRAME_PREREGISTRATION.json` SHA256 is `c61498a2e0020d83788edb0b8ccc4b72999eba8c85fc6a0265ec300fe67ad9a5`. It was frozen before anchor reference pose values, map correspondences, or NDT results were read. All 31 scans later matched official reference timestamps exactly; evaluation pose values were read only after `NATIVE_OUTPUT_FREEZE.json`.

The raw prescreen CSV first-column label says one-based but the executed writer emitted the zero-based index. `PRESCREEN_FIELD_AUDIT.md` preserves this counterexample and the unchanged raw CSV. The downstream mapping was corrected by checking the path and integer timestamp against the canonical inventory; no rescan or window change occurred. The sampled anisotropy ratio remains a search heuristic and is not the formal NDT geometry spectrum.
