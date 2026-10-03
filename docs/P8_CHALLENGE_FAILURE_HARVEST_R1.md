# P8 Challenge Failure Harvest R1

Date: 2026-10-04
Decision: `DATA_OR_INITIALIZATION_CONTRACT_NOT_READY`

## Scope and frozen baseline

This run was prepared on `research/p8-challenge-failure-harvest`, created at
the frozen paper baseline `70aa6859657c92404751cd354bd4292e9d30c617`.
Only a Corridor01 data-format adapter and its unit test were added. No
localization, filter, NDT, map, initialization, Dual-U, recovery, visual, or
stable dog-workspace code was changed. No localization replay was started.

The stable single-state baseline remains IMU propagation + scan-end deskew +
one current-frame NDT registration + the existing lightweight IKFoM update.
The frozen runner requires a provenance-backed initial `map_T_lidar` and a
causal static-IMU initialization window. Its only NDT initial guess is the
scan-end propagated pose. The unchanged baseline contract is documented in
`PAPER_BASELINE_CLOSURE_R1.md` and implemented by
`scripts/p7/p7_single_state_runner.cpp`.

## Corridor01 raw timestamp contract

Input bag:

```text
/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/raw/Long_Corridor_Rosbag/raw_data_core_2023-07-25-03-01-44.bag
SHA-256 c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811
```

The bag has 2,777 `/velodyne_packets` messages and 55,957 `/imu/data`
messages. The point export is a 40-byte little-endian record:
`float64 x,y,z,intensity` plus `uint64 absolute_sensor_nanoseconds`. Its
timestamp is formed from the packet ROS stamp and the pinned VLP-16 decoder's
per-return firing time:

```text
point_stamp_ns = packet.stamp.toNSec()
                 + llround(point.time_seconds * 1e9)
```

The source exporter uses the pinned `ros-drivers/velodyne` revision
`29abd0e1361cb7f5eda451d2b51c35eeca45e0d5` and the official VLP-16
calibration. The raw scan header was audited as the first packet stamp. The
per-scan timing contract is:

```text
scan_start = minimum decoded valid-return timestamp
point_offset_ns = point_stamp_ns - scan_start
scan_end = last_packet_stamp_ns + 1,306,368 ns
filter transaction stamp = scan_end
```

The `1,306,368 ns` end allowance is the last scheduled VLP-16 firing offset.
This is not a guessed offset from the bag record time. `rosbag info` records
the bag start at `1690254112.82 s`, whereas the first sensor header/point epoch
is `1517157219.088119030 s`; the recorder clock must not be substituted for
sensor message time. The SubT-MRS paper documents PPS synchronization of IMU,
LiDAR, and thermal data through the CPU clock and a sensor-pair gap no greater
than 3 ms ([CVPR 2024 paper](https://openaccess.thecvf.com/content/CVPR2024/html/Zhao_SubT-MRS_Dataset_Pushing_SLAM_Towards_All-weather_Environments_CVPR_2024_paper.html)).
No additional fixed clock offset was estimated or applied. The original raw
bag does not independently encode a per-sensor offset audit; measured IMU
bracketing gaps below are sample-grid gaps, not estimates of synchronization
error.

Whole-sequence timing audit:

| Quantity | Result |
|---|---:|
| Scans / points | 2,777 / 79,932,911 |
| Scan duration min / median / max | 90.881309 / 100.839449 / 100.839450 ms |
| Scan-start period min / median / max | 93.671016 / 100.859881 / 110.818021 ms |
| Positive interval overlaps | 4; maximum 40.366 µs |
| Maximum gap after scan end | 9.978810 ms |
| Point-order timestamp backward events | 276 events in 272 scans; maximum 92.341 µs |
| Finite raw XYZ/intensity records | 79,932,911 / 79,932,911 |
| IMU period median / maximum | 4.992 / 14.992 ms |
| Scan intervals bracketed by IMU samples | 2,776 / 2,777 |

The four adjacent scan overlaps are at transaction pairs 366→367 (40.366 µs),
951→952 (40.366 µs), 1655→1656 (39.412 µs), and 2240→2241 (38.458 µs).
Point times are not monotonic in serialized point order, so the adapter
preserves record order and does not sort. The P7 deskewer consumes an
independent timestamp per point and does not require chronological point
serialization. The one unbracketed scan is the first: IMU begins 71.097 ms
after its `scan_start`. No deskew was run in this task.

The adapter was checked on 20 deterministically spaced scans from transaction
2 through 2777. For every sampled scan, point timestamps remained inside the
catalog interval, converted offsets matched `absolute_stamp - scan_start`
exactly, output XYZ was finite, and IMU samples bracketed the full interval.
All sampled `offset_min_ns` values were zero.

| Tx | Points | Min point ns | Max point ns | Duration ms | Max offset ms | IMU bracket | Finite XYZ |
|---:|---:|---:|---:|---:|---:|:---:|:---:|
| 2 | 28,863 | 1517157219188978910 | 1517157219289818360 | 100.839450 | 100.839450 | yes | yes |
| 148 | 29,083 | 1517157233913687944 | 1517157234014465405 | 100.777461 | 100.777461 | yes | yes |
| 294 | 29,054 | 1517157248638333082 | 1517157248739172293 | 100.839211 | 100.839211 | yes | yes |
| 440 | 29,086 | 1517157263362977982 | 1517157263463817431 | 100.839449 | 100.839449 | yes | yes |
| 586 | 29,047 | 1517157278087622881 | 1517157278188462330 | 100.839449 | 100.839449 | yes | yes |
| 732 | 21,255 | 1517157292812328100 | 1517157292913167311 | 100.839211 | 100.839211 | yes | yes |
| 878 | 29,052 | 1517157307536973000 | 1517157307637812449 | 100.839449 | 100.839449 | yes | yes |
| 1024 | 29,051 | 1517157322261619091 | 1517157322362458302 | 100.839211 | 100.839211 | yes | yes |
| 1170 | 29,020 | 1517157336986325979 | 1517157337087104393 | 100.778414 | 100.778414 | yes | yes |
| 1316 | 28,868 | 1517157351710974932 | 1517157351811814381 | 100.839449 | 100.839449 | yes | yes |
| 1463 | 28,933 | 1517157366536483049 | 1517157366637322260 | 100.839211 | 100.839211 | yes | yes |
| 1609 | 28,999 | 1517157381261131048 | 1517157381361970259 | 100.839211 | 100.839211 | yes | yes |
| 1755 | 29,077 | 1517157395985821009 | 1517157396086618258 | 100.797249 | 100.797249 | yes | yes |
| 1901 | 28,911 | 1517157410710490942 | 1517157410811329437 | 100.838495 | 100.838495 | yes | yes |
| 2047 | 29,084 | 1517157425435139894 | 1517157425535979344 | 100.839450 | 100.839450 | yes | yes |
| 2193 | 28,973 | 1517157440159790993 | 1517157440260630442 | 100.839449 | 100.839449 | yes | yes |
| 2339 | 28,933 | 1517157454884500980 | 1517157454985340429 | 100.839449 | 100.839449 | yes | yes |
| 2485 | 29,060 | 1517157469609150887 | 1517157469709990336 | 100.839449 | 100.839449 | yes | yes |
| 2631 | 28,909 | 1517157484333800077 | 1517157484434639288 | 100.839211 | 100.839211 | yes | yes |
| 2777 | 26,809 | 1517157499058448076 | 1517157499159287287 | 100.839211 | 100.839211 | yes | yes |

## Adapter and software checks

Added:

- `src/dog_prior_map_fastlio2_frontend_exp/scripts/p8/convert_corridor01_p7_timed.py`
- `src/dog_prior_map_fastlio2_frontend_exp/tests/p8_corridor01_adapter_test.py`

The adapter validates source-manifest hashes and row/count contracts, verifies
that catalog rows cover the complete point binary, and hashes the exact point
bytes consumed during conversion against the source manifest. It preserves
point order, converts only XYZ representation (`float64`→`float32`) and time
origin (`absolute uint64 ns`→`relative uint32 ns`), and emits the P7 16-byte
record plus scan index/filter table. It does not decode packets, filter points,
deskew, choose initialization, read GT, or invoke localization. Intensity is
not copied because the frozen P7 input record is XYZ plus per-point time.

The full output has 2,777 scans / 79,932,911 points; output binary SHA-256 is
`6195878e0d0a68891b392c490078e7ed3be185cd6425d1738ccc13847df31916`.
The source manifest and hashes validated. The output is archived outside the
repository under
`/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p8_failure_harvest_adapter_r1/`.

Checks performed:

- `p8_corridor01_adapter_test.py`: 8/8 PASS (absolute-to-relative conversion,
  out-of-range timestamps, uint32 overflow rejection, nonfinite rejection,
  full catalog coverage, end-to-end fixture conversion, mutation between
  manifest validation and consumption, and nonempty-output refusal).
- `current_frame_ndt_test`: PASS; actual PCL target leaf reports
  `0.8,0.8,0.8 m`.
- `p7_replay_io_test`: PASS.
- Full source-manifest hash/row validation: PASS.
- 20-scan real adapter checks: 20/20 PASS.
- `git diff --check`: PASS.

These are adapter/software contract checks, not a Corridor localization smoke
run or evidence that localization works on this sequence.

## Map, initialization, and GT frame gate

The raw official map file is
`map/corridor01.pcd` (SHA-256
`4b5231d58ebd4aeb4fc293559051dc30a18b6f07376237938d064962bfe8f40a`). A
derived normalized map also exists (SHA-256
`103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f`), but
its normalization is not promoted here to an independently documented
map-origin convention.

The official SuperLoc release lists Corridor01's bag, extrinsics, map,
trajectory, and a generic “Initialization poses” download
([official dataset page](https://superodometry.com/datasets)). The local
`initial_pose/corridor01.yaml` SHA-256 is
`0670732e26f0d9ee19e6115110e29d0aeca5d26f62ab4848d1629c5f9a6eb62b`. It has
comment `# s 67` and supplies:

```text
R = [ 0.135990  -0.990409  -0.024406
      0.990705   0.136027   0.000140
      0.003181  -0.024198   0.999702 ]
t = [1.968147, -6.879292, -0.896125] m
keys: extrinsicRotation_world_darpa, extrinsicTranslation_world_darpa
```

The official page does not define the source/target direction, whether the
matrix is `T_map_lidar`, `T_world_darpa`, or a world-frame re-expression, nor
the exact timestamp represented by `# s 67`. The audited public SuperOdom
source did not parse these keys. The local prior map↔GT audit classifies the
fixed transform as `OFFICIAL_LINEAGE_PLUS_DATA_CONSISTENCY_SUPPORTED`, not
officially confirmed. A direct-map consistency check favors one YAML
direction, but an arbitrary 90-degree control also has nontrivial overlap;
this is not adequate provenance to select a runtime initial pose. GT cannot
be used to choose between hypotheses.

Official calibration `laser_to_imu` gives `T_imu_lidar` with translation
`[0.080, 0.029, 0.030] m` and the rotation in
`calibration/corridor01_extrinsics.yaml` (SHA-256
`59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d`). The
required initialization conversion would be:

```text
T_map_imu0 = T_map_lidar0 * inverse(T_imu_lidar)
```

But `T_map_lidar0` and its epoch are not yet established, so this conversion
cannot be populated without assuming frame semantics.

The frozen P7 frontend requires 200 causal static IMU samples; its baseline
gate is maximum per-axis sample standard deviation ≤`0.5 m/s²` for
acceleration and ≤`0.05 rad/s` for gyro. The first 200 samples fail with
`2.212268 m/s²` and `0.404158 rad/s`. There are later valid windows: the
earliest qualifying 200-sample window starts at
`1517157221.789088000 s` and ends at `1517157222.784032000 s`, approximately
`2.701–3.696 s` after first LiDAR scan start. However, no eligible
map-referenced initial pose at this time is documented. The 200 samples
centered near the official file's nominal `s 67` fail the same static gate
(`1.630536 m/s²`, `0.136238 rad/s`).

GT lineage is independently closed as IMU-origin, but the fixed transform
between that GT world and the released PCD is not official-confirmed. Thus the
two required initial conditions are not jointly available: an admitted
`T_map_lidar0` at a known epoch and a compatible causal static initialization
window. Identity initialization, a first-segment map alignment, or a GT-derived
pose would each violate the current contract.

## Replay gate and scientific decision

No one-frame localization smoke, Run A/B/C, full Corridor01 replay, GT
evaluation, failure-window extraction, U_obs diagnostics, or same-objective
multi-start analysis was run. This is intentional: running before the
map/frame/initial-pose gate closes could turn a coordinate/startup mistake
into a false “natural NDT failure.” Consequently:

```text
trajectory metrics = NOT RUN
repeatability = NOT TESTED
first natural failure onset = NOT ESTABLISHED
U_obs / U_nonlocal failure-window relevance = NOT TESTED
```

Do not proceed to Cave01 under the current rule: Corridor01 was not shown to
be stable or failure-free; it is not yet admissibly initialized. The next
prerequisite is an authoritative transform convention and epoch for the
provided Corridor01 initialization file (or another approved, non-GT
`T_map_lidar0` with timestamp), plus confirmation that the static IMU window
used is compatible with that pose epoch. Until then the correct result is
`DATA_OR_INITIALIZATION_CONTRACT_NOT_READY`, not a localization failure or a
negative failure-harvest result.

## Git and protection

```text
workspace = /home/jian/livox_ws/dog_loc_paper_ws
branch = research/p8-challenge-failure-harvest
base/start SHA = 70aa6859657c92404751cd354bd4292e9d30c617
stable dog workspace touched = NO
baseline algorithm changed = NO
protected localization/interfaces source changed = NO
```
