# P3-R3B: official GT lineage and origin closure

Status: `PAPER-P3-R3B-PASS` for GT lineage/origin and relative evaluation; the map/world transform is only Level-2 supported, not officially confirmed.
Scope: offline evidence audit only. No runtime algorithm or frozen baseline changes.

## Result

The SuperLoc Corridor01 GT trajectory directly matches the official ICCV 2023 SubT-MRS Hawkins `Long_Corridor` RC2 challenge GT: both have 1,385 pose rows and the same trajectory samples. Translation differences are exactly zero; normalized quaternion comparison using `abs(dot)` gives mean/P95/max rotation differences of approximately `6.10e-7 / 2.41e-6 / 3.42e-6` degrees. There are 1,284 exact text-represented timestamps; the other 101 differ by no more than 239 ns, with no constant shift. Classification: `IDENTICAL` pose sequence, with timestamp serialization precision differences.

The official challenge submission convention explicitly defines trajectory samples as IMU sensor poses in a fixed/world frame. Under the stage's exact-lineage rule, set evaluation reference origin to `IMU` with evidence level `OFFICIAL_LINEAGE` (not direct SuperLoc metadata). Primary source: [ICCV 2023 challenge rules](https://superodometry.com/iccv23_challenge_Mul). The SuperLoc page independently identifies Corridor01 as SubT-MRS / Hawkins / RC2 but only specifies TUM field order, not pose origin: [SuperLoc dataset release](https://superodometry.com/superloc).

The exact relative-evaluator results and source files are in:

```text
/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p3_reference_lineage_closure/
```

See `gt_origin_decision.md`, `official_gt_lineage_sources.md`, and `relative_failure_timeline_final.csv` there. The configured evaluation-time origin is `1517157224.188978910 s`; the relative-pose zero is the first in-coverage pair at `1517157224.231677055 s` (`+0.042698145 s`). CSV times and reported crossings use the configured origin as their time axis. Convert a crossing to relative-pose-zero time by subtracting `0.042698145 s`:

| Threshold | Configured-origin time | Relative-pose-zero time |
|---|---:|---:|
| 0.25 m | 3.068314 s | 3.025616 s |
| 0.5 m | 5.589692 s | 5.546994 s |
| 1 m | 34.131304 s | 34.088606 s |
| 2 m | 34.635604 s | 34.592906 s |
| 5 m | 35.341563 s | 35.298865 s |

No GT extrapolation was used.

## Boundaries

- `corridor01.yaml`'s `extrinsic*world_darpa` direction/semantics remain unresolved; the official public SuperOdom source audit did not find a parser for these keys.
- The current frozen localizer does not load this initial-pose YAML and begins from identity on its first scan. A numerical comparison to the YAML is therefore not justified.
- The exact fixed `T_map_GT` is not officially documented. Direct scan/map NN supports the YAML-matrix direction, but a rotated control also overlaps; map status remains `OFFICIAL_LINEAGE_PLUS_DATA_CONSISTENCY_SUPPORTED`, not `OFFICIAL_CONFIRMED`.
- Global absolute ATE remains alignment-dependent. Relative-from-start temporal analysis can continue because it removes a common fixed global transform.
- `FAILURE_MECHANISM = UNRESOLVED`; `P4_ALLOWED = NO`.

Full audit: [P3_R3B_MAP_FRAME_CLOSURE.md](P3_R3B_MAP_FRAME_CLOSURE.md).
