# R13 commands and one-run limit

Historical executed commands (NOT instructions to repeat the approved experiment):

```bash
python3 docs/p62_r13_training_guided_multihypothesis/reference_inventory.py
python3 docs/p62_r13_training_guided_multihypothesis/execute.py prepare
python3 docs/p62_r13_training_guided_multihypothesis/execute.py build
python3 docs/p62_r13_training_guided_multihypothesis/execute.py run
python3 docs/p62_r13_training_guided_multihypothesis/analyze.py
python3 docs/p62_r13_training_guided_multihypothesis/analyze.py --reviewed
```

There were2 build attempts: frozen_01 link failure, frozen_02 success; only1 real R13 process. Build/source/binary details in BUILD_PROVENANCE.json and FAILED_BUILD_01.json. Native actual argv in raw/STARTED.json; fixed RUN already exists and cannot be reused. Never delete it, change output paths or use remaining budget to re-execute. --reviewed is an offline reporting refinement, retaining initial results; its existing backup prevents another overwrite.

Safe read-only post-archive checks:

```bash
cd /home/jian/livox_ws/p62_r13_multihypothesis_ws/docs/p62_r13_training_guided_multihypothesis
sha256sum -c checksums.sha256
python3 tests/test_runner.py
```

The mock test launches no real algorithm. Actual binary, map/bag and build directories stay on internal disk, not Git. No ROS, rosbag play, EKF, GT alignment or new dataset command belongs to this experiment.
