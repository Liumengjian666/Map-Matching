# Corridor01 causal replay: complete terminal procedure

Run this in a regular Ubuntu terminal, not in the restricted Codex sandbox.
This uses the already-built research worktree and the frozen 39.96 s slice; it
does not rebuild, re-extract, or load GT.

Run modes serially and in this order:
1. COUPLED_SHADOW, output directory shadow/.
2. Only after Shadow parity is PASS, WEAK_ONLY_FEEDBACK, output directory weak_only_feedback/.
3. Then COUPLED_FEEDBACK, output directory coupled_feedback/.
4. Freeze and hash every runtime output before any GT evaluation.

Each run is a separate full replay. Stop all terminals between runs. Never
reuse an output directory. If a run is interrupted or fails, preserve its
files and stop; do not overwrite or silently retry it.

## 1. Preflight once

Every hash must print OK. This verifies the actual NDT and EKF binaries, frozen
input, map, extrinsics, configs, and Control references.

~~~bash
sha256sum --check <<'SHA'
9da66b47b89053e523f3ce2d18086395e565ef6d9353ebd34c4f4f315afd3db4  /tmp/dog_loc_paper_r4_ws.Fq21k2/devel/lib/dog_prior_map_localization/dog_prior_map_ndt_node_cpp
4d1655cc3f7d3fdc6cac2372f380ca8fe7b0b250227235767947da3d4030f402  /tmp/dog_loc_paper_r4_ws.Fq21k2/devel/lib/dog_prior_map_localization/dog_prior_map_ekf_node_cpp
f6e77c96d97402e551835e81a1ce8d780d075da2f18a9cb5e4fc0646628b86d6  /home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_prospective_causal_replay_r1/input/corridor01_p10_causal_startup5_eval35_v1.bag
103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f  /media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/map/derived/corridor01_map_normalized.pcd
59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d  /media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/calibration/corridor01_extrinsics.yaml
fc5c262f4eacbb8b23d95a9adb5877e956fa215f84c675a378c62e2e63d94d6c  /tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_localization/config/p10_corridor01_prospective_causal.yaml
482d11e26b998aad0fcb07840d9d8eede788291348c1a93a748a74edf1fe6b83  /tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_localization/config/dog_prior_map_localization_ndt.yaml
2851892e14e852098808f78baf42d2e878782a268fae13f869091c9f4b556aa1  /home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_prospective_causal_replay_r1/run_control/control_topics.bag
aa3be25e06e3f8e287bf5e3d03fec31367b533ab8d63d995812101741c9ecb41  /home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_prospective_causal_replay_r1/run_control/ndt_diagnostics.csv
e02466b7941c6593a552e7929fe06b19b1c1e2e79e2d907aec5f3f37469c7e05  /tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_localization/launch/dog_prior_map_localization_split.launch
668e4ed552e8dfd313b873bb5c8a33b476db14fa0325e5e6c7dbddc0916914dc  /tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_localization/scripts/p10_corridor01_prospective/compare_shadow_control.py
1d8bc7796adef84a2912339eb1829554bf2f632f1dc559ace68f19a5dd790b61  /tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_localization/scripts/p10_corridor01_prospective/evaluate_with_frozen_control_alignment.py
f6998574fb60e6b36eb3519e9228d17cc5209857054870557738d1affb11fe5d  /tmp/dog_loc_paper_r4_ws.Fq21k2/docs/p10_corridor01_frozen_control/posthoc_metrics.json
fbe0f41638e55d992052342079de6efc92da3eca3ab4b3adc9cabd4003aab16f  /tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp
c99e5bd62d7fe35b532dca10c0d8534c1558ece65bdb7f50492c08860edae77b  /tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_localization/include/dog_prior_map_localization/core/causal_pose_prediction.hpp
aa311032b230181e7a6bc25d089280c1e38a7837d861b40cf866b74347b46981  /tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_fastlio2_frontend_exp/include/dog_prior_map_fastlio2_frontend_exp/observable_pcl_ndt.hpp
385aa81558272162251812556e2d0c7b927e9dc6166b8e4b535978e81f1f5627  /tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_fastlio2_frontend_exp/src/current_frame_ndt.cpp
3fcec19cb67ecdedc71b3314606e881e3af5b3339bc1ccda155b88fb1fea9a54  /tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_localization/scripts/p10_corridor01_prospective/evaluate_frozen_control.py
SHA
~~~

Confirm the ROS package resolves to the research worktree and inspect the
frozen slice metadata:

~~~bash
source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/devel/setup.bash
source /tmp/dog_loc_paper_r4_ws.Fq21k2/devel/setup.bash
test "$(rospack find dog_prior_map_localization)" = "/tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_localization" && echo "research package OK"
rosbag info /home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_prospective_causal_replay_r1/input/corridor01_p10_causal_startup5_eval35_v1.bag
~~~

Expected input: 5 s startup + 35 s evaluation, 396 scans, 39.96 s. Every run
uses the same frozen raw point/IMU bag, map, configs, scan-start reference,
nominal logic, step limiter, and EKF settings; only the explicit mode differs.
Because feedback changes the EKF state that drives causal scan deskew, derived
deskewed point clouds may then differ between feedback runs. Preserve and report
each run's actual cloud_hash values; do not claim processed-cloud parity across
independent feedback trajectories. Shadow must match Control exactly. Do not
query the mode parameter before launch; it does not exist until the NDT node
starts.

## 2. Run Shadow using four terminals

For each run, set the two variables shown below in every terminal that uses
them. All other commands stay identical.

| Run | RUN_ID | MODE |
|---|---|---|
| Shadow parity | shadow | COUPLED_SHADOW |
| Weak-only feedback | weak_only_feedback | WEAK_ONLY_FEEDBACK |
| Coupled feedback | coupled_feedback | COUPLED_FEEDBACK |

Never run these at the same time: they all use ROS master port 11311.

### Terminal 1: create the output directory and start roscore

For the first run, use RUN_ID=shadow. For later runs, substitute the RUN_ID
from the table. Do not remove an existing output directory.

~~~bash
RUN_ID=shadow
OUT="/home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_coupled_causal_comparison_r1/$RUN_ID"
test ! -e "$OUT" || { echo "Refusing to overwrite: $OUT"; exit 1; }
set -euo pipefail
mkdir -p "$OUT/ros_home" "$OUT/logs"
date --iso-8601=seconds | tee "$OUT/run_times.txt"
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
export ROS_HOME="$OUT/ros_home" ROS_LOG_DIR="$OUT/logs"
set +e
roscore 2>&1 | tee "$OUT/logs/roscore.log"
ROSCORE_STATUS=$?
set -e
date --iso-8601=seconds | tee -a "$OUT/run_times.txt"
printf 'roscore_exit=%s\n' "$ROSCORE_STATUS" | tee -a "$OUT/run_times.txt"
test "$ROSCORE_STATUS" -eq 130 || { echo "roscore exited unexpectedly; preserve this run and stop"; exit 1; }
~~~

Leave it running.

### Terminal 2: launch the frozen NDT + EKF chain

Wait until both nodes report started before continuing. For later modes, set
RUN_ID and MODE from the table.

~~~bash
RUN_ID=shadow
MODE=COUPLED_SHADOW
OUT="/home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_coupled_causal_comparison_r1/$RUN_ID"
source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/devel/setup.bash
source /tmp/dog_loc_paper_r4_ws.Fq21k2/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
export ROS_HOME="$OUT/ros_home" ROS_LOG_DIR="$OUT/logs"
set -euo pipefail
PKG="$(rospack find dog_prior_map_localization)"
printf 'package=%s\n' "$PKG"
test "$PKG" = "/tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_localization" || exit 1
rosparam set /use_sim_time true || { echo "Failed to enable simulated time"; exit 1; }
roslaunch dog_prior_map_localization dog_prior_map_localization_split.launch \
  config:=/tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_localization/config/dog_prior_map_localization_ndt.yaml \
  dataset_config:=/tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_localization/config/p10_corridor01_prospective_causal.yaml \
  rviz:=false publish_path:=false coupled_mode:="$MODE" \
  runtime_csv_path:="$OUT/ekf_runtime.csv" \
  ndt_diagnostics_csv_path:="$OUT/ndt_diagnostics.csv" \
  oosm_csv_path:="$OUT/oosm.csv" \
  ekf_prediction_diagnostics_csv_path:="$OUT/ekf_lineage.csv" \
  imu_deskew_csv_path:="$OUT/deskew.csv" \
  diagnostic_determinism_enable:=true scan_reference_time:=start \
  oosm_enable:=true oosm_max_alignment_sec:=0.02 \
  2>&1 | tee "$OUT/logs/launch.log"
~~~

Check launch.log for the expected mode, normalized map path, and target point
count. If package, map, binary, or mode is wrong, stop before playback.

### Terminal 3: verify the mode and start recording

Run only after both nodes started. The mode check must print the requested
value and pass.

~~~bash
RUN_ID=shadow
MODE=COUPLED_SHADOW
OUT="/home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_coupled_causal_comparison_r1/$RUN_ID"
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
export ROS_HOME="$OUT/ros_home" ROS_LOG_DIR="$OUT/logs"
set -euo pipefail
ACTUAL_MODE="$(rosparam get /dog_prior_map_ndt/coupled_mode)"
printf 'mode=%s\n' "$ACTUAL_MODE"
test "$ACTUAL_MODE" = "$MODE" || exit 1
test "$(rosparam get /dog_prior_map_ndt/map/pcd_fallback_path)" = "/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/map/derived/corridor01_map_normalized.pcd"
test "$(rosparam get /dog_prior_map_ndt/lidar_update/scan_reference_time)" = "start"
python3 - /tmp/dog_loc_paper_r4_ws.Fq21k2/devel/lib/dog_prior_map_localization/dog_prior_map_ndt_node_cpp /tmp/dog_loc_paper_r4_ws.Fq21k2/devel/lib/dog_prior_map_localization/dog_prior_map_ekf_node_cpp <<'PY'
import hashlib, os, pathlib, sys
expected = {os.path.realpath(arg): arg for arg in sys.argv[1:]}
found = {path: [] for path in expected}
for entry in pathlib.Path("/proc").iterdir():
    if not entry.name.isdigit():
        continue
    try:
        exe_link = entry / "exe"
        exe = os.path.realpath(exe_link)
        if exe in found:
            digest = hashlib.sha256(exe_link.read_bytes()).hexdigest()
            found[exe].append((entry.name, digest))
    except (OSError, PermissionError):
        continue
for exe, matches in found.items():
    assert len(matches) == 1, ("expected exactly one live node binary", exe, matches)
    print(f"live_pid={matches[0][0]} executable={exe} sha256={matches[0][1]}")
PY
rosbag record -O "$OUT/topics.bag" /clock /dog_livo/ndt_odom /dog_livo/odom_high_rate /dog_livo/odom_corrected 2>&1 | tee "$OUT/logs/record.log"
~~~

Leave recording active.

### Terminal 4: play the frozen slice once

The player must reach end-of-bag naturally. Do not Ctrl-C it early.

~~~bash
RUN_ID=shadow
OUT="/home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_coupled_causal_comparison_r1/$RUN_ID"
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
set -euo pipefail
rosbag play --wait-for-subscribers --clock --rate 0.25 /home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_prospective_causal_replay_r1/input/corridor01_p10_causal_startup5_eval35_v1.bag 2>&1 | tee "$OUT/logs/player.log"
~~~

Before starting Terminal 4, wait for the recorder log to show it subscribed to
all four requested topics. The player also uses the installed ROS Noetic
wait-for-subscribers barrier so it will not publish before its input topics
have subscribers. After the recorder reports all four subscriptions, wait 2
seconds for ROS connections to settle before starting the player.

### Drain, stop, and freeze that run

After Terminal 4 exits naturally, leave the nodes and recorder running for 10
wall seconds so queued callbacks can drain. Then stop in this order:
1. Ctrl-C in Terminal 2 to stop roslaunch and close/flush its CSV files.
2. Wait 2 seconds, then Ctrl-C in Terminal 3 to close the output bag.
3. Ctrl-C in Terminal 1 to stop roscore and write the end timestamp.

Verify completeness before hashing or loading GT. Every mode must contain 396
NDT rows, 396 deskew rows, 396 OOSM rows, and the same pose-topic message counts
as the frozen Control bag: 396 NDT, 396 corrected, 7988 high-rate. Convergence
and OOSM status counts are reported, not silently filtered.

~~~bash
RUN_ID=shadow
MODE=COUPLED_SHADOW
OUT="/home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_coupled_causal_comparison_r1/$RUN_ID"
source /opt/ros/noetic/setup.bash
set -euo pipefail
python3 - "$OUT" "$MODE" <<'PY' || exit 1
import collections, csv, json, pathlib, sys
import rosbag

out = pathlib.Path(sys.argv[1])
mode = sys.argv[2]
def rows(name):
    with (out / name).open(newline="") as stream:
        return list(csv.DictReader(stream))

ndt = rows("ndt_diagnostics.csv")
deskew = rows("deskew.csv")
oosm = rows("oosm.csv")
assert len(ndt) == 396, ("NDT rows", len(ndt))
assert len(deskew) == 396, ("deskew rows", len(deskew))
assert len(oosm) == 396, ("OOSM rows", len(oosm))
assert all(row["coupled_mode"] == mode for row in ndt), "mode mismatch in diagnostics"
assert all(row["status"] == "PUBLISHED" for row in deskew), "deskew rejection found"

counts = collections.Counter()
with rosbag.Bag(str(out / "topics.bag"), "r") as bag:
    for topic, _, _ in bag.read_messages():
        counts[topic] += 1
expected = {"/dog_livo/ndt_odom": 396, "/dog_livo/odom_corrected": 396,
            "/dog_livo/odom_high_rate": 7988}
actual = {topic: counts[topic] for topic in expected}
assert actual == expected, ("pose topic counts", actual, expected)
report = {
    "mode": mode,
    "ndt_rows": len(ndt),
    "ndt_converged_rows": sum(row["ndt_has_converged"] == "1" for row in ndt),
    "deskew_rows": len(deskew),
    "oosm_rows": len(oosm),
    "oosm_status_counts": dict(collections.Counter(row["oosm_result"] for row in oosm)),
    "pose_topic_counts": actual,
    "completeness": "PASS",
}
(out / "completeness.json").write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps(report, indent=2))
PY
test -s "$OUT/ekf_runtime.csv" && test -s "$OUT/ekf_lineage.csv" || exit 1
(cd "$OUT" && find . -type f ! -name runtime_hashes.sha256 -print0 | sort -z | xargs -0 sha256sum) | tee "$OUT/runtime_hashes.sha256"
python3 - "$OUT" <<'PY'
import hashlib, pathlib, sys
out = pathlib.Path(sys.argv[1])
manifest = out / "runtime_hashes.sha256"
entries = {}
for line in manifest.read_text().splitlines():
    digest, name = line.split("  ", 1)
    name = name[2:] if name.startswith("./") else name
    assert name not in entries, ("duplicate hash entry", name)
    entries[name] = digest
actual = sorted(p.relative_to(out).as_posix() for p in out.rglob("*")
                if p.is_file() and p != manifest)
assert sorted(entries) == actual, ("runtime hash coverage mismatch",
                                   sorted(entries), actual)
for name, expected in entries.items():
    actual_hash = hashlib.sha256((out / name).read_bytes()).hexdigest()
    assert actual_hash == expected, ("runtime hash mismatch", name)
print(f"Runtime hash coverage PASS: {len(actual)} files")
PY
~~~

## 3. Shadow parity gate

Run after Shadow processes stop and files close:

~~~bash
source /opt/ros/noetic/setup.bash
set -euo pipefail
python3 /tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_localization/scripts/p10_corridor01_prospective/compare_shadow_control.py --control-dir /home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_prospective_causal_replay_r1/run_control --shadow-dir /home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_coupled_causal_comparison_r1/shadow | tee /home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_coupled_causal_comparison_r1/shadow/control_shadow_parity.json
python3 - /home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_coupled_causal_comparison_r1/shadow/control_shadow_parity.json <<'PY'
import json, sys
p = json.loads(open(sys.argv[1]).read())
d, b = p["ndt_diagnostics"], p["output_topics"]
assert p["result"] == "PASS", p
assert d["rows_control"] == d["rows_shadow"] == 396, d
assert d["failure_count"] == 0 and b["failure_count"] == 0, p
c = d["shadow_execution_counts"]
assert c["feedback_applied"] == 0 and c["feedback_changed_observation"] == 0, c
print("Shadow hard gate PASS: 396 rows; source, score, pose and feedback parity")
PY
~~~

Proceed only if the command exits 0 and the JSON reports result PASS, 396
diagnostic rows on both sides, and zero diagnostic/topic parity failures. The
checker also requires zero Shadow feedback, zero extra jet/value work on
untriggered frames, and checks source identity, timestamps, nominal/raw/used
poses, NDT fitness, and all three recorded pose topics. If anything fails,
preserve the output and stop. Do not change tolerances or run feedback.

## 4. Run both feedback modes

Only after Shadow parity passes, repeat all of Section 2 (four terminals,
same commands and playback rate) once per mode, changing RUN_ID and MODE in
each relevant terminal to these exact values:

~~~text
RUN_ID=weak_only_feedback
MODE=WEAK_ONLY_FEEDBACK

RUN_ID=coupled_feedback
MODE=COUPLED_FEEDBACK
~~~

Each is an independent full causal run: its feedback changes the next frame's
EKF prediction. Do not run modes in one ROS process, stitch trajectories, or
drop frames. If a run fails, preserve its output and report it.

After the three runs are complete and runtime files are closed, verify the
runtime hashes before any GT evaluation:

~~~bash
for RUN_ID in shadow weak_only_feedback coupled_feedback; do
  OUT="/home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_coupled_causal_comparison_r1/$RUN_ID"
  (cd "$OUT" && sha256sum --check runtime_hashes.sha256) || exit 1
done
~~~

## 5. Post-hoc evaluation, only after runtime freeze

GT is first read here, after all three runtime outputs are stopped and
hash-verified. Both feedback methods use the exact frozen Control transform in
docs/p10_corridor01_frozen_control/posthoc_metrics.json. Do not fit another
transform.

~~~bash
source /opt/ros/noetic/setup.bash
set -euo pipefail
EVALUATOR=/tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_localization/scripts/p10_corridor01_prospective/evaluate_with_frozen_control_alignment.py
GT="/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/gt/corridor01_gt.txt"
EXTRINSICS="/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/calibration/corridor01_extrinsics.yaml"
CONTROL_POSTHOC=/tmp/dog_loc_paper_r4_ws.Fq21k2/docs/p10_corridor01_frozen_control/posthoc_metrics.json
ROOT=/home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_coupled_causal_comparison_r1
EXPECTED_GT_SHA256=3cabcc78ecea4d991aa6e3eddb811cefc4fdacf5f3387b98950fa09ad338dd03
ACTUAL_GT_SHA256="$(sha256sum "$GT" | awk '{print $1}')"
test "$ACTUAL_GT_SHA256" = "$EXPECTED_GT_SHA256" || { echo "GT hash differs from frozen Control receipt"; exit 1; }
for RUN_ID in weak_only_feedback coupled_feedback; do
  OUT="$ROOT/$RUN_ID"
  python3 "$EVALUATOR" --bag "$OUT/topics.bag" --gt "$GT" --extrinsics "$EXTRINSICS" --control-posthoc "$CONTROL_POSTHOC" --output-dir "$OUT/posthoc_eval" | tee "$OUT/posthoc_eval.log" || exit 1
done
~~~

Finally hash post-hoc results separately:

~~~bash
set -euo pipefail
for RUN_ID in weak_only_feedback coupled_feedback; do
  OUT="/home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_coupled_causal_comparison_r1/$RUN_ID"
  sha256sum "$OUT/posthoc_eval.log" "$OUT/posthoc_eval/posthoc_metrics.json" "$OUT/posthoc_eval/posthoc_per_sample_errors.csv" | tee "$OUT/posthoc_hashes.sha256"
done
~~~

This is PREFIX-ALIGNED RELATIVE DRIFT, not proof that absolute map and GT
frames coincide. Frozen Control values remain references, not new measurements.
Do not present Shadow candidate error as the executed feedback trajectory error.
