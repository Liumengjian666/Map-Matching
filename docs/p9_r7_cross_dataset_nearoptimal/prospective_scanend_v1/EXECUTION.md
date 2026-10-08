# Execution receipt

Working directory: `/tmp/dog_loc_paper_r4_ws.Fq21k2`.
ROS Noetic environment and existing adapter workspace overlay were sourced for
message type lookup only. No adapter executable was invoked.

Commands used (same meanings as actual invocations; paths shown explicitly):

```bash
cmake -S src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/corridor_raw_scanend -B /tmp/p9_corridor_raw_scanend_build -DCMAKE_BUILD_TYPE=Release -DVELODYNE_ROOT=/home/jian/livox_ws/superloc_adapter_ws/src/velodyne
cmake --build /tmp/p9_corridor_raw_scanend_build -j2
```

Initial build errors were resolved before extraction: ambiguous `TopicQuery`
initializer became an explicit vector; yaml-cpp >=0.5 requires upstream's
`HAVE_NEW_YAMLCPP` compile definition. No pinned Velodyne source was edited.
Both raw timing and real parser synthetic packet tests passed before extraction.

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/corridor_raw_scanend/run_protocol.py extract --binary /tmp/p9_corridor_raw_scanend_build/p9_corridor_extract --archive docs/p9_r7_cross_dataset_nearoptimal/prospective_scanend_v1
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/corridor_raw_scanend/run_protocol.py audit --archive docs/p9_r7_cross_dataset_nearoptimal/prospective_scanend_v1
```

`extract` was invoked exactly once. Do not run again: its exclusive preflight and
output directory are deliberately retained. `audit` also produced exclusive
receipts; use `finalize_audit.py verify` for later hash checks, not re-extraction.

The CMake was then extended for the no-NDT real-input probe:

```bash
cmake -S src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/corridor_raw_scanend -B /tmp/p9_corridor_raw_scanend_build -DFASTLIO2_ROOT='/media/jian/HIKVISION/comparison algorithm/FAST_LIO2'
cmake --build /tmp/p9_corridor_raw_scanend_build -j2
```

From that build directory: `OMP_NUM_THREADS=1 ctest --output-on-failure` passed
6/6. Real input probe command:

```bash
/tmp/p9_corridor_raw_scanend_build/p9_corridor_input_probe '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p9_corridor01_raw_scanend_v1'
```

Tool-output transcription (not an independently captured process log):

```text
P7_READER_PARITY=PASS scans=2777 points=79932911 imu=55957
P7_STATIC_VARIANCE_ADMISSION=FAIL reason=static_imu_variance_exceeds_gate samples=200
NDT_CALLS=0 PROPAGATED_REAL_SCANS=0 GT_LOADED=NO
```

Synthetic bag test (normal/missing topics/dual return/no overwrite), five cases:

```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/corridor_raw_scanend/test_protocol.py /tmp/p9_corridor_raw_scanend_build/p9_corridor_extract /home/jian/livox_ws/superloc_adapter_ws/src/velodyne/velodyne_pointcloud/params/VLP16db.yaml
```

P9 Release build and 41 tests were run from `/tmp/p9_r4_release.Eirto1`:

```bash
cmake --build /tmp/p9_r4_release.Eirto1 -j2
env PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 LD_LIBRARY_PATH=/lib/x86_64-linux-gnu ctest --output-on-failure
```

Archive writer initially encountered relative/absolute path resolution bugs.
These were fixed using the repository root; its already written preflight CMake
snapshot was checked byte-exact, not overwritten. Completion of the source diff
did not rerun sensors/NDT. Final independent verification command:

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/corridor_raw_scanend/finalize_audit.py verify --repo /tmp/dog_loc_paper_r4_ws.Fq21k2
```

The raw-input SHA in preflight is a new full-file calculation. All persistent
generated inputs are separately hashed, and the external input manifest must
equal the Git receipt. The pre-extraction converter binary SHA is unchanged after
adding the optional P7 probe target.
