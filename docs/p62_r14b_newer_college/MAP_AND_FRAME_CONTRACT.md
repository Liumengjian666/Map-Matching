# Coordinate admission: COORDINATE_CONTRACT_BLOCKED

Convention: T_A_B maps B coordinates to A. M is the official Leica map XYZ,
L the intended Ouster LiDAR frame, C the left-camera optical frame, B the
co-located robotics Base, I Ouster IMU. Co-location does not imply equal rotation.

## Facts with official provenance

- `sources/ground_truth.html` says stereo reference poses are in RS_C1. It also
  says Ouster scans were registered against the Leica prior by ICP.
- `sources/platform.html` describes RS_C1 as left-camera optical, then explicitly
  distinguishes the co-located Base axes (x-forward,y-left,z-up).
- Official `sources/frontier_README.md`, at branch commit
  `0b039393334a7767e24a711bd05816bfc7f7485a`, describes GT in that robotics Base.
- Original paper, SecIII/V, also calls the reference Base at the left camera:
  https://arxiv.org/html/2003.05691v2 . This is support for the Base interpretation,
  not an identified writer for this actual CSV or an explicit precedence rule
  resolving the currently conflicting optical wording.
- Official local `cam-ouster-imu/camchain-...yaml` provides T_cam_imu (I to C),
  and `ouster_imu_lidar_transforms.yaml` provides LiDAR-to-IMU and inverse.
  Their actual identities are in ASSET_MANIFEST.json. No values were estimated.

## Two distinct chains, not two experiments

If CSV is T_M_C: T_M_L = T_M_C T_C_I T_I_L.
If CSV is T_M_B: T_M_L = T_M_B T_B_C T_C_I T_I_L.

The fixed robotics/optical rotation would be
T_B_C rotation = [[0,0,1],[-1,0,0],[0,-1,0]], translation zero.
This is an axis-definition conversion, not a fitted transform. It cannot be
inserted or omitted merely because one projection scores better.

Both chains are mathematically meaningful, but the supplied authoritative
material has not unambiguously bound the CSV quaternion axes to one of them.
No experiment was run to select an interpretation. No map/trajectory alignment,
ICP, Umeyama, or NDT-assisted coordinate fitting was performed.

## SOCC-ICP cannot arbitrate this ambiguity

At pinned commit `557a98e39a163f0b369a1612db7fc69f8b5aae60`, the actual consumer
`sources/socc_newer_college.py` loads xyzw and right-multiplies a hardcoded
approximate rotation/translation, then resets all poses by inverse(first_pose).
Its data organization is reusable, but its approximate calibration and relative
world normalization are not proof of a precise official map-localization chain.
We did not execute its consumer or its registration algorithm.

## Scope and exact missing evidence

The official Leica/registered-reference geometric relationship is documented.
The unresolved part is the released CSV's exact orientation-frame semantics,
not evidence that the data or calibration are erroneous. A release/file-specific
official exporter, metadata, or authoritative clarification can resolve it.
No need to prove physical ground truth independence to perform this conditional
benchmark; nevertheless independence cannot be claimed.

Additional provenance limit: the archived current Ouster driver can build a LUT
with a LiDAR-to-sensor transform; its actual historical PCD writer/caller is not
identified. Platform docs define OS_L as the point-cloud frame, but driver-only
arguments cannot independently certify the saved PCD frame or time-reference
caller. These are not findings of actual coordinate/time violations.

Under the user stop gate, no pilot was admitted. Do not call this result an
unconditional failure of Newer College or a proof that Base interpretation is wrong.
