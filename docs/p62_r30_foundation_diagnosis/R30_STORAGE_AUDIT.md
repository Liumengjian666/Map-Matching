# Storage inventory and approval-only cleanup proposals

No files were deleted. No dataset was downloaded. Raw bags/maps/GT and R29 logs remain intact. Disk values are actual allocated bytes from du and free bytes from the filesystem; they must not be confused with reclaimable logical sizes.

Root free before worktree creation:35244015616bytes. At final identity check:32919322624bytes (30.66GiB), above20GiB reserve. Observed reduction2324692992bytes includes unrelated filesystem activity and is not claimed as exact task-only allocation. External data disk free449049264128bytes. Exact inventory is STORAGE_AUDIT.json.

|Path|Allocated bytes|Current role|
|---|---:|---|
|/home/jian/livox_ws/p62_r30_foundation_ws|2312732672|Full research checkout, historical archives, new build and evidence|
|/home/jian/livox_ws/p62_r30_foundation_ws/build|432586752|Rebuildable CMake/object output|
|/home/jian/livox_ws/p62_r30_foundation_ws/devel|182067200|Actual repaired node/test binaries; retain current identities|
|/home/jian/livox_ws/p62_r30_foundation_ws/.r30_diagnostic_build|819200|Actual probe and regression executables; locally ignored, retained|
|/home/jian/livox_ws/p62_r29_full_sequence_ws/build|914251776|Old rebuildable objects/cache|
|/home/jian/livox_ws/p62_r29_full_sequence_ws/devel|357072896|Original formal runtime binaries, protected|
|/home/jian/rosbag/public_datasets/koide_p62_r29|24798375936|Official archives, extracted bags, converted input/maps, protected by default|
|/home/jian/livox_ws/p62_r29_full_sequence_data|135495680|Original failure traces and actual receipts, protected|

## Candidates for a later explicit approval (NOT deleted, NOT automatic SAFE_TO_DELETE)

- R29 `build`914251776bytes, and R30 `build`432586752bytes: source/build recipes are preserved, actual formal binaries live in devel. These are rebuildable outputs, not asserted byte-identical directories. Deletion would remove caches and require rebuilding future tests. Retain all binaries/evidence first; user approval required.
- `/home/jian/rosbag/public_datasets/koide_p62_r29/indoor_easy_01/indoor_easy_01/decompressed_01_0.db3` logical4394782720bytes and `/home/jian/rosbag/public_datasets/koide_p62_r29/indoor_hard_01/indoor_hard_01/decompressed_03_0.db3` logical5090283520bytes: losslessly expanded representations of the retained official ZIPs, not byte-identical ZIP copies. R29 sealed extracted SHA and ZIP CRC/MD5 receipts support reconstruction with the original ZIP extraction procedure. Actual forensic/read dependencies and archive integrity must be rechecked immediately before any approved deletion. No second large rehash was performed this round. Do not delete the only retained official ZIP or any required current input.

The two candidate db3 files are proposals about redundant storage representation, not permission to delete original scientific data. No workspace or branch was pruned; no cleanup of the R10 development preservation was attempted. The erroneous-working-directory CTest artifacts created this round were moved intact into the new R30 log archive, with no historical artifact overwritten.
