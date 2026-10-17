# P56 Severe-Degeneracy Dataset Baseline

## Final status

**Overall: `DOWNLOAD_BLOCKED`.** No localization, NDT, Hessian, or degeneracy experiment ran. The official Ground-Challenge Corridor2 share redirects to Microsoft authentication. The official SubT-MRS Long_Corridor archive was attempted twice (including one byte-range resume), but the second connection terminated before the advertised file size; the preserved `.part` is incomplete and was not extracted. Corridor1 was not attempted because the higher-priority Corridor2 did not pass access validation. These are source/input barriers, not algorithm failures.

## Environment and provenance

- P56 isolated worktree: `/home/jian/livox_ws/p56_severe_degeneracy_benchmark_ws`
- Branch: `research/p56-severe-degeneracy-benchmark`
- Base/start revision: `a93d41d46ac60adc04552d76a09861e08f7ed87d`
- External disk: UUID `F627-4133`, mounted read-write at `/media/jian/HIKVISION`; 427.8 GiB free after the bounded attempts.
- External data root: `/media/jian/HIKVISION/paper rosbag/DegenerateBenchmarks/`
- No files in the protected P10, P53, P54, P55, `dog_loc_paper_ws`, or `dog_visual_loc_ws` worktrees were changed.

## Per-sequence disposition

| Sequence | Download / input evidence | Map and reference contract | Runtime |
|---|---|---|---|
| Ground-Challenge Corridor2 | Official OneDrive share ended at Microsoft sign-in; only an HTML auth page was returned, not saved as a dataset. | Official repo has no listed prior map; its calibration file does not provide LiDAR–IMU extrinsics. `psudo_gt` is not certified as independent truth. | Not eligible; 0 frames. `DOWNLOAD_BLOCKED`. |
| Ground-Challenge Corridor1 | Not attempted under the requested priority gate. | No independent map or complete transform contract established. | 0 frames. Not evaluated. |
| SubT-MRS Long_Corridor | Official ZIP advertises 3,888,497,930 bytes. After two bounded attempts, the preserved partial is 3,480,741,223 bytes, SHA-256 `6fb37b96d9ac80ebe3fd32cc113504d1c8c6c791919a30b073a0b4a8786f16fa`; `file` recognizes a ZIP prefix, but the archive is incomplete and was not CRC-tested or extracted. | Official extrinsics file was retrieved (584 bytes, SHA-256 `03df4bc1d32ba7148fd90a35f370d319bb83531c774d1848b3da0acf25f390bc`), but `laser_to_imu` direction/frame semantics are not independently established. No official prior map is listed. The public GT listing says `Limited access`; no GT was read. | Not eligible; 0 frames. `DOWNLOAD_BLOCKED`. |

## Scientific limits

There is no observed NDT failure and no evidence that any of the three scenes has or lacks severe geometric degeneracy. No GT was opened, no map was built, and no absolute or relative localization metric was computed. Dataset descriptions (straight/zigzag corridor, duration, distance) are not substitutes for a measured degeneracy result. The Ground-Challenge and SuperLoc naming schemes were not mixed.

Download attempt details, exact source links, and available calibration content are in `download_attempts.md` and `DATASET_SOURCES.md`. Machine-readable disposition is in `results.json`.

## Git handoff

The archive is documentation-only; no algorithm source or binary was changed, so `CODE_SHA` equals `START_SHA`. The P56 branch did not exist on origin at the final read-only check. `PUSH_EXECUTED=false`. The final local `END_SHA` is given in the execution receipt; the report does not embed its own commit ID because that would be self-referential. No push was performed.
