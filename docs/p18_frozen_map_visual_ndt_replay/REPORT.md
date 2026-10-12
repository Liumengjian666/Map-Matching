# P18: device discovery gate stopped

TASK: P18-FROZEN-MAP-RECOVERY-AND-VISUAL-NDT-REPLAY

FINAL_RESULT: **STORAGE_DEVICE_NOT_DETECTED**

At 2026-10-12 09:14 +0800 the current Ubuntu host did not expose HIKVISION:
`lsblk -f` showed only the internal NVMe and loop devices; `lsusb` showed no
USB storage device; `findmnt --mountpoint /media/jian/HIKVISION` found no
mount; `df` reported that path absent. No device node was assumed or forced.

Kernel records at **04:32-04:33** show disconnect-related I/O errors followed
by USB descriptor error -110, address error -62, and `unable to enumerate USB
device`. These precede this run and are explicitly historical, not newly
observed 09:14 errors. No USB/storage event appeared in the queried preceding
two hours. The user's reconnection report alone does not establish current
device visibility or health. The specific present blocker is absent device
and mount, not a demonstrated new map-copy timeout.

The research workspace contains only the rejected P17R zero-byte incomplete
map, not a valid recovered map. Local frozen V2 bag and visual CSV SHA checks
passed again. No external map content was read; **copy attempts = 0**. No
device repair, mount, fsck, speed test, large scan or extra retry was performed.

No runtime code/configuration was changed. No `--dataset-config` adaptation
or P18 configuration was created before its map prerequisite. No ROS replay,
NDT call, OOSM update, visual application or new GT evaluation occurred.
Control parity, visual smoke, full replay, H1/H2/H3, accuracy and performance
are **NOT_RUN / NOT_EVALUATED**, not failed visual-algorithm results. Missing
CSV experiment outputs were not fabricated.

Repository: Liumengjian666/Map-Matching. Branch:
`research/p9-r4-heldout-visual-evidence`. START_SHA and unchanged CODE_SHA:
`3b3d53da8fe755c5f3c24dd8c99008579625b7a0`; live remote matched at start,
confirming P17R was uploaded. This new receipt was not pushed by the agent.
The existing untracked caches were preserved; no stable or main workspace
was modified. Only this small stop receipt is added.

NEXT: **RESTORE_HIKVISION_DEVICE_VISIBILITY**.
Ask the user to check cable, port, enclosure and power until Ubuntu can see
the actual device and mount, or provide a healthy-SSD identical map backup.
Then resume the same P18 contract with one controlled copy and the original
map SHA; do not use the old zero-byte file, an alternate map or new parameters.
