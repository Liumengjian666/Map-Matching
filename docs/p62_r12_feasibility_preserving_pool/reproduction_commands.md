# R12 reproduction and no-rerun boundary

Historical executed commands, NOT permission to repeat the formal replay:

```bash
python3 docs/p62_r12_feasibility_preserving_pool/execute.py prepare
python3 docs/p62_r12_feasibility_preserving_pool/execute.py build
python3 docs/p62_r12_feasibility_preserving_pool/execute.py bind
python3 docs/p62_r12_feasibility_preserving_pool/execute.py run
python3 docs/p62_r12_feasibility_preserving_pool/summarize.py
```

The actual C++ argv, compiler, full build commands, binary hash, tests and source versions are in BUILD_PROVENANCE.json, RUNTIME_BINDING.json, raw/STARTED.json and REPLAY_RECEIPT.json. The exclusive fixed RUN directory already exists; never delete it or switch output paths to replay again. summarize.py writes exclusive derived results; do not overwrite the archive.

Safe post-archive read-only checks:

```bash
cd /home/jian/livox_ws/p62_r12_feasibility_pool_ws/docs/p62_r12_feasibility_preserving_pool
sha256sum -c checksums.sha256
python3 tests/test_execute.py
```

The second command uses mocks only. All full real results are preserved for controller review. No rosbag/roslaunch/NDT/EKF command is part of this task. build logs and binary remain on internal disk; small logs and identities are archived.
