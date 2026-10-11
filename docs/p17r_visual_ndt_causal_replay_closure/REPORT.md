# P17R storage recovery attempt

TASK: P17R-VISUAL-NDT-CAUSAL-REPLAY-CLOSURE

FINAL_RESULT: **FROZEN_MAP_SAFE_COPY_FAILED_STORAGE_BLOCKED**

The same-map recovery gate failed. No new ROS replay was launched. This is
an input storage blocker, **not a failure of the visual algorithm**. The
user's explicit stop-on-copy-failure rule was followed; no second read or
alternate map was attempted.

## Git and unchanged runtime

- Repository: Liumengjian666/Map-Matching.
- Branch: `research/p9-r4-heldout-visual-evidence`.
- START_SHA / unchanged CODE_SHA:
  `7430e4e30447d76bc2123b30fa48c7841ca36b21`.
- Live remote branch was checked and matched that SHA before this attempt.
- PUSH_EXECUTED: NO. This receipt is a new local archive, not uploaded work.
- No NDT, visual, EKF, OOSM, Anchor, initialization, filter or dataset source
  changes; no rebuild. P17 and all historical failed runs are preserved.
- The main dirty workspace and stable robot workspace were not edited.

## Actual storage evidence (2026-10-12, Asia/Shanghai)

The local research SSD is `/dev/nvme0n1p2`, ext4, about 58 GB available.
Searches for Corridor/SuperLoc/normalized map PCDs in the local research
workspaces, home data locations and temporary research copies found no
usable full frozen Corridor01 map. Local scan/crop PCDs and the Floor01 map
are not replacements. Search results do not prove that an arbitrarily named
backup elsewhere cannot exist.

`/proc/mounts` reports HIKVISION `/dev/sda2`, fuseblk, `rw`. Permission or
capacity is not the demonstrated blocker. `lsblk -f` labels the underlying
filesystem exfat. Kernel logs contain USB/UAS resets, 180 s command timeouts,
READ I/O errors and a lost async page WRITE at 03:21:11. No recent new kernel
entry was found; absence of new entries does not prove recovery.

One bounded `stat` on the original map succeeded immediately:

```
/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/map/derived/corridor01_map_normalized.pcd
size=4058751 bytes
mtime=2026-09-23 05:19:24 +0800
```

One and only one copy of this approximately 4 MB file was attempted:

```bash
timeout --signal=TERM --kill-after=2s 15s cp --no-clobber '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/map/derived/corridor01_map_normalized.pcd' .p17r_visual_ndt_causal_replay_closure/assets/corridor01_map_normalized.pcd
```

The timeout process was killed; the destination contained **0 bytes**. Its
SHA256 was the empty-file hash
`e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`,
not the required
`103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f`.
The incomplete file was rejected and moved to the explicit quarantine name
`.p17r_visual_ndt_causal_replay_closure/assets/corridor01_map_normalized.pcd.incomplete`.
The original map was not overwritten or changed. No task copy process
remained after termination. No fsck, remount, formatting, privilege bypass,
map regeneration, alternate map or repeated bulk reads were performed.

Metadata access is therefore insufficient evidence of readable PCD contents.
The specific gate that failed was recovering the identical frozen map to
healthy storage, before ROS map loading, NDT or GT evaluation.

## Reusable local inputs rechecked

- Frozen V2 bag SHA PASS:
  `7f72f808a9f2282c65705e7e1e731061bb2cc2950329e46da768608e4a7d552e`.
- Raw saved visual CSV SHA PASS:
  `10869519890a5adcb6f3310bba05f379a959018cf123ea8a90c1cf4ea1314c79`.
- CSV has 396 records and 169 valid pairs. It was neither regenerated nor
  filtered; historical coverage remains 169/395 = 42.7848%.
- Existing window coverage remains 35/59 at 6-12 s, 0/40 at 26-30 s,
  0/48 at 30-35 s. These are input facts, not new positioning results.
- Current loopback TCP bind/listen passed; socket permission is not the
  blocker. No roscore retry was needed.

## Requested replay and accuracy results

All **new** short Control NDT/OOSM, short Visual NDT/applications and full
Visual NDT/OOSM/applications: **NOT_RUN**, actual new calls/updates **0**.
No new GT was read. Short Control parity is untested, not PASS. New visual
initial/raw/EKF RMSE, per-window accuracy, jumps, nonfinite-state counts,
NDT/interface costs and localization peak RSS: **NOT_MEASURED**. Zero calls
do not certify absence of numerical failures during a replay.

For historical comparison only, frozen Control V2 3-35 s translation
RMSE/P95/max is **1.887877 / 5.011039 / 5.667157 m**; rotation is
**3.026246 / 3.705965 / 16.393782 deg**. No new Visual-Aided score exists.

Prior P17 offline visual-stage mean/P95 was **74.499 / 84.215 ms per valid
pair**, excluding camera IO and MEI rectification; it was not remeasured.
The old 194.5 MiB visual RSS value was a sample, not peak RSS. This receipt
claims neither real-time deployment nor complete-system memory compliance.

## Only next step

NEXT: **RESTORE_FROZEN_MAP_ON_HEALTHY_STORAGE**.

Human intervention is required: restore stable USB cable/port/enclosure
access, or supply an existing healthy-SSD copy of the exact frozen map.
After a successful complete copy, its full SHA must equal the required hash.
Only then resume the existing disabled-prefix parity gate, enabled short
smoke and one full visual replay. No algorithm tuning is justified by this
storage failure.
