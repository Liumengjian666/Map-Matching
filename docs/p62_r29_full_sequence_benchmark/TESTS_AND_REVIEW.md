# Final local verification and review

This review packages already completed runs; it starts no NDT, downloads, or new dataset experiments. Formal online source remains CODE `f66a55457ba3e63d55ad8d588fb2cf5b1ead169b`.

- Release build succeeded with the existing ROS Noetic/Livox overlay. Initial dependency/link and interface failures remain in logs; they are not successful benchmark trajectories. All 28 preliminary native calls are accounted separately from the 3625 formal calls.
- The finalizer independently checked each of four online seals, per-run CODE/binary hashes, completed-input receipts, all 2023 consecutive output rows, 2022 valid references, and raw1336's explicitly unknown reference. It rechecked B/C pose/time/admission/fusion equality and the per-frame/per-60-input-frame restart limits against the original admission/schedule CSVs.
- All original high-rate output files remain at the paths in RAW_ARTIFACT_MANIFEST.json. Portable gzip copies of decision, candidate, schedule, observation-lineage, map and publisher records decompress to the original byte hashes. No large bag, map or executable is committed.
- Frozen EKF, IMU propagation, Deskew, observation and local-map model source hashes match BUILD_FREEZE.json. Online source has not changed after the formal CODE commit. Stable worktree HEAD, branch, index, status and protected source/config hashes match STABLE_BEFORE exactly; STABLE_AFTER and FINAL_VALIDATION retain the actual comparison.
- Reviewed the three R29 C++/launch changes: research-only default-off native baseline and frame-count budgets; snapshot sensor scheduling and explicit OOSM terminal acknowledgment; stale causal input rejection without a map measurement. No new registration objective, EKF/OOSM mathematics, relative admission or search direction is introduced. The original finite/future/provenance and freshness constraints remain active.
- Two reporting corrections are explicit: a symbolic extrinsic inverse typo in prose (numeric runtime chain was already correct), and unknown reference must not end a failure episode. Neither correction changes online states, thresholds, RMSE/P95/Max, or test selection. Initial derived output is retained externally.
- Both figures were rendered and visually checked. Position errors use a labelled logarithmic axis; the B/C overlap is labelled, all failed segments remain, and the reference sample count/limitations are visible.
- This is evidence consistency verification, not a scientific PASS. Azure depth snapshot input, unintegrated large IMU gaps, 185 stale predictions per run, invalid consistency statistics and full tracking failures remain material limitations. No external AI review was used.

Reproduce packaging/verification only (no localization):

```bash
PYTHONDONTWRITEBYTECODE=1 python3 docs/p62_r29_full_sequence_benchmark/tools/finalize_delivery.py
```

`checksums.sha256` covers the committed R29 archive, excluding itself. Commit identity is obtained from the final local Git HEAD; it is deliberately not embedded as a self-referential checksum. No push or subsequent experiment is authorized by these commands.
