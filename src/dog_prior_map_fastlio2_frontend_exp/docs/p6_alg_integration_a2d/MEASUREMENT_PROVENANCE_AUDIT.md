# A2D source audit before implementation

START_SHA: b04a1f7f3442266dd48bd2efea5baf09d905b0c8

HEAD and branch matched the authorized starting identity; worktree was clean.

Confirmed directly from the frozen scripts:

- `scripts/p6_i1_prepare_inputs.py:222`: Floor01 export assigns
  `cloud = message.cloud_end_frame`, validates the request hash, extracts only
  XYZ, and packs float32 triples into `request_xyz_f32.bin`. The exported cloud
  is an old runtime SE3-deskew sensor product, not a raw timed scan. The binary
  has no point timestamps. A matching hash proves identity, not independence
  from the old estimator state.
- `scripts/p4_i3_visual_increment.py:178`: depth association receives
  `old_req.cloud_end_frame`. Frozen Floor01 visual increments therefore have
  LEGACY_STATE_DERIVED_DEPTH provenance. No new raw-camera evidence is claimed.
- `scripts/p6_i6c_prepare_corridor01.py:192`: manifest explicitly declares
  `rotationally_deskewed_only; no second deskew; no translational deskew`.
  Export again writes XYZ only. This is SENSOR_LOCAL_ROTATION_ONLY, not a
  WINDOW_OWNED_SE3_DESKEW product, and lacks raw point time for a new deskew.

Both current XYZ-only bundles are RAW_POINT_TIME_UNAVAILABLE and
NOT_ELIGIBLE_FOR_WINDOW_OWNED_DESKEW. Do not guess offsets from point order or
uniformly synthesize time. Raw rosbag re-export is a later authorized task.

A2C V2 remains COMPATIBILITY_ONLY_NOT_FORMAL_INPUT for historical regression.
V3 must consume explicit timed raw scans and reject legacy-state-derived
cloud/depth by default. This audit changes no historical script or frozen data.
