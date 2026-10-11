#!/usr/bin/env bash
# Offline probe only; does not start ROS or change any localization state.
set -euo pipefail
task_root=/home/jian/rosbag/p12_dcreg_solid_r1_20261011
task_workspace=/home/jian/livox_ws/p10_corridor01_oosm_r1_ws
task_map=/home/jian/rosbag/loop2/loop2mapping/pcd/loop2_simtime_rebuild_2026_08_12_001_all_downsampled_points.pcd
task_database=/home/jian/rosbag/p11_solid_r1_20261011/solid_database.txt
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:"$task_workspace/devel/lib"
git -C "$task_root/upstream_dcreg" diff HEAD --exit-code
test "$(git -C "$task_root/upstream_dcreg" rev-parse HEAD)" = 8ce8451b15491a4bbe17cf85ab02a8bed6696861
python3 - "$task_root/experiment" <<'PY'
import hashlib,json,sys
from pathlib import Path
root=Path(sys.argv[1]); freeze=json.loads((root/'freeze.json').read_text())
assert hashlib.sha256((root/'samples.csv').read_bytes()).hexdigest()==freeze['samples_sha256']
for key in ('target','database'):
    hash_key='map_sha256' if key=='target' else 'database_sha256'
    assert hashlib.sha256(Path(freeze[key]).read_bytes()).hexdigest()==freeze[hash_key]
PY
sha256sum "$task_root/build/dcreg_solid_integration_probe" "$task_root/build/interface_test" > "$task_root/binary_hashes.sha256"
(cd "$task_root/build" && ctest --output-on-failure) > "$task_root/tests.log" 2>&1
/usr/bin/time -v -o "$task_root/smoke_resources.txt" \
  "$task_root/build/dcreg_solid_integration_probe" "$task_map" "$task_database" \
  "$task_root/experiment/samples.csv" "$task_root/experiment/smoke" 0 2 0 \
  > "$task_root/smoke.log" 2>&1
python3 - "$task_root/experiment/smoke/registration.csv" <<'PY'
import csv,sys
rows=list(csv.DictReader(open(sys.argv[1])))
assert len(rows)==8 and all(row['valid_se3']=='1' for row in rows)
# Convergence is an experimental outcome, not an interface/completeness gate.
PY
/usr/bin/time -v -o "$task_root/cohort_resources.txt" \
  "$task_root/build/dcreg_solid_integration_probe" "$task_map" "$task_database" \
  "$task_root/experiment/samples.csv" "$task_root/experiment/cohort" 2 22 0 \
  > "$task_root/cohort.log" 2>&1
