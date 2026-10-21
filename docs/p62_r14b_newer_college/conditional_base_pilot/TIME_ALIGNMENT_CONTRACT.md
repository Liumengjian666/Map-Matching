# File-level time pairing only

Reused frozen inventory has15301 Ouster PCD filenames and15301 reference timestamps. Selected21 pair exactly at integer ns, epoch seconds*10^9+nanoseconds; zero timestamp delta. No interpolation, time-offset fitting, or altered point-time definition. Anchor ordinal767 timestamp1583836667780782336ns. Evaluation ordinals768–787 are consecutive after full sequence5% time.

This certifies deterministic file-level association, not physical hardware time accuracy or per-point absolute sampling times. No new deskew or IMU alignment. All21 filename/SHA/roles are pre-registered before the formal align. Evaluation pose values are parsed only after NATIVE_OUTPUT_FREEZE and EVALUATION_ADMISSION gates; hashing source bytes is not use of numeric test poses in prediction.
