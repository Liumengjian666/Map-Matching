# Measurement provenance and V3 input format

LiDAR provenance enums: `RAW_TIMED_SENSOR`, `SENSOR_LOCAL_ROTATION_ONLY`,
`WINDOW_OWNED_SE3_DESKEW`, `LEGACY_STATE_DERIVED_SE3_DESKEW`.
Raw V3 admission accepts only the first; already deskewed products cannot be
deskewed as raw. Window output is explicitly `WINDOW_OWNED_SE3_DESKEW`.

Visual enums: `RAW_SENSOR_LOCAL_DEPTH`, `WINDOW_OWNED_DEPTH`,
`LEGACY_STATE_DERIVED_DEPTH`, `UNKNOWN`. `FrozenVisualEvent` carries provenance.
V3 disables compatibility at adapter construction. Legacy and UNKNOWN are
rejected before visual graph mutation with
`VISUAL_PROVENANCE_REJECTED_NOT_FORMAL_INPUT`. Hash validity alone never changes
this semantic classification.

Historical V2 adapters keep compatibility intentionally. V2 diagnostics carry
the actual cloud provenance, `COMPATIBILITY_ONLY_NOT_FORMAL_INPUT`, and
`VISUAL_PROVENANCE_COMPATIBILITY_ONLY` for legacy depth. No compatibility opt-in
is exposed by the V3 CLI. A typed provider is trusted to describe its lineage
truthfully; metadata does not cryptographically prove that a caller has not lied.

## Raw timed file contract

CSV header, exact spelling:

```
transaction_id,scan_start_ns,scan_end_ns,byte_offset,point_count,provenance
```

Binary record: little-endian `float64 x,y,z,intensity` followed by `uint64
absolute_sensor_point_stamp_ns` (40 bytes). Offsets and counts are byte-bounded;
finite payloads and timestamps within the scan interval are required. The reader
does not derive any time from a point's index. Missing times reject the record.
The successful runner reports SHA256 of the raw file and catalog.

Visual sidecar header: `ref_ns,cur_ns,provenance`. Missing metadata becomes
UNKNOWN and is rejected by V3. It does not relabel legacy Floor01 measurements.
`NONE` supplies no metadata, not automatic RAW provenance.

Future real export must additionally document original bag identity, topic,
point-time field and conversion convention, calibration and preprocessing in
its input manifest. This stage supplies the format/reader, not a newly eligible
Floor01/Corridor01 bundle or independent visual-depth frontend.

## Evidence

`measurement_provenance_test` checks adapter gates and unchanged graph revision
on rejection. `A2D_FIXTURE` checks actual PCL production wiring under all four
R2 policies, no legacy provider invocation, rejected legacy LiDAR with zero NDT
calls, rejected legacy visual factors, and binary reader preservation of a
nonuniform sensor timestamp. It also checks zero time, truncation, out-of-range
timestamps and transactional reader output.

Each V3 policy fixture: 10 timestamp events, three raw scans/deskews, three
LiDAR commits, one nonlocal probe pair, five NDT calls, zero post-handoff IKFoM
calls. This fully reliable synthetic map does **not** admit visual factors;
actual visual admission and scan-start influence are covered separately by the
degraded-direction adapter test. No claim of real visual improvement is made.
