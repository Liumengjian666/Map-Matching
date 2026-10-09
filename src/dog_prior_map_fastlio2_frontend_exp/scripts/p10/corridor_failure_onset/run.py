#!/usr/bin/env python3
"""One-shot, non-GT common-predictor paired replay for the fixed onset window."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

REPO = Path(__file__).resolve().parents[5]
ARCHIVE = REPO / "docs/p10_corridor01_failure_onset"
INPUT = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p9_corridor01_raw_scanend_v1")
MAP = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/map/derived/corridor01_map_normalized.pcd")
OLD_CACHE = Path("/home/jian/livox_ws/dog_loc_paper_ws/.p9_experiment_cache/p10_corridor01_real_degeneracy_benchmark")


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: run.py /absolute/path/to/p10_corridor_failure_onset")
    binary = Path(sys.argv[1]).resolve()
    selection_path = ARCHIVE / "selection_freeze.json"
    predictor = ARCHIVE / "common_predictor.csv"
    freeze = json.loads(selection_path.read_text())
    if freeze["GT_LOADED"] or freeze["frame_count"] != 346:
        raise RuntimeError("pre-GT selection freeze is invalid")
    if sha(predictor) != freeze["predictor_csv_sha256"]:
        raise RuntimeError("common predictor artifact changed after freeze")
    for item in freeze["input_hashes"]:
        if sha(item["path"]) != item["sha256"]:
            raise RuntimeError("frozen input changed before replay: " + item["path"])
    if sha(MAP) != freeze["P9_map_sha256"]:
        raise RuntimeError("map changed after selection freeze")
    old_frames = OLD_CACHE / "frames.csv"
    old_ledger = OLD_CACHE / "source_ledger.csv"
    if sha(old_frames) != freeze["source_frames_sha256"] or sha(old_ledger) != freeze["source_ledger_sha256"]:
        raise RuntimeError("historical P10 source record changed before replay")
    if not binary.is_file():
        raise RuntimeError("paired replay binary missing")
    if any((ARCHIVE / name).exists() for name in
           ("PAIRED_RUN_STARTED", "paired_candidates.csv", "paired_execution.json", "paired_failure.json",
            "process_exit.json", "experiment_freeze.json", "T_imu_lidar.txt", "run.log")):
        raise RuntimeError("one-shot replay already started; no implicit rerun")

    extrinsic_path = ARCHIVE / "T_imu_lidar.txt"
    with extrinsic_path.open("x") as stream:
        stream.write("\n".join(" ".join(format(float(x), ".17g") for x in row)
                               for row in freeze["T_imu_lidar"]) + "\n")
    command = [str(binary), str(INPUT), str(MAP), str(predictor), str(ARCHIVE), str(extrinsic_path)]
    code_files = [
        REPO / "src/dog_prior_map_fastlio2_frontend_exp/scripts/p10/corridor_failure_onset/paired_replay.cpp",
        REPO / "src/dog_prior_map_fastlio2_frontend_exp/scripts/p10/corridor_benchmark/causal_rotation.hpp",
        REPO / "src/dog_prior_map_fastlio2_frontend_exp/src/current_frame_ndt.cpp",
        REPO / "src/dog_prior_map_fastlio2_frontend_exp/src/coupled_ndt_weak_refinement.cpp",
        REPO / "src/dog_prior_map_fastlio2_frontend_exp/src/coupled_ndt_anchor.cpp",
        REPO / "src/dog_prior_map_fastlio2_frontend_exp/include/dog_prior_map_fastlio2_frontend_exp/current_frame_ndt.hpp",
        REPO / "src/dog_prior_map_fastlio2_frontend_exp/include/dog_prior_map_fastlio2_frontend_exp/coupled_ndt_weak_refinement.hpp",
        REPO / "src/dog_prior_map_fastlio2_frontend_exp/include/dog_prior_map_fastlio2_frontend_exp/coupled_ndt_anchor.hpp",
        REPO / "src/dog_prior_map_fastlio2_frontend_exp/src/p7_replay_io.cpp",
        REPO / "src/dog_prior_map_fastlio2_frontend_exp/include/dog_prior_map_fastlio2_frontend_exp/p7_replay_io.hpp",
        REPO / "src/dog_prior_map_fastlio2_frontend_exp/scripts/p10/corridor_benchmark/CMakeLists.txt",
        REPO / "src/dog_prior_map_fastlio2_frontend_exp/scripts/p10/corridor_failure_onset/freeze_predictors.py",
        REPO / "src/dog_prior_map_fastlio2_frontend_exp/scripts/p10/corridor_failure_onset/run.py",
    ]
    config = {
        "TASK": "P10-CORRIDOR01-FAILURE-ONSET-COUPLED-BENCHMARK",
        "phase": "PRE_NDT_REPLAY_FREEZE",
        "protocol": freeze["protocol"],
        "GT_LOADED": False,
        "causal_feedback": "NOT_RUN; Experiment A is paired conditional only",
        "initial_pose_source": freeze["common_predictor_source"],
        "time_reference": "scan_end_ns for predictor, cloud, align, and later GT interpolation",
        "selection": {k: freeze[k] for k in ("evaluation_origin_ns", "window_duration_ns", "first_transaction",
                                               "last_transaction", "frame_count", "first_scan_end_elapsed_s",
                                               "last_scan_end_elapsed_s", "selection_rule", "excluded_boundary")},
        "P9_input_manifest_sha256": freeze["P9_input_manifest_sha256"],
        "P9_map_sha256": freeze["P9_map_sha256"],
        "common_predictor_sha256": sha(predictor),
        "historical_P10_frames_sha256": sha(old_frames),
        "historical_P10_source_ledger_sha256": sha(old_ledger),
        "binary_sha256": sha(binary),
        "code_sha256": {str(path.relative_to(REPO)): sha(path) for path in code_files},
        "command": command,
        "ndt": {"resolution_m": 0.8, "step_size": 0.08, "epsilon": 1e-5, "max_iterations": 80},
        "candidate_methods": ["NOMINAL", "R6_WEAK_ONLY", "R6_COUPLED"],
        "full_align_calls": "one shared nominal align per scan; refinements reuse its source/map/objective",
        "map_instances": 1,
        "R6_anchor": "reconstructed causally from the prior P10 Nominal predictor/executed rows; not an archived R6-arm anchor",
        "R6_trigger": "recomputed from nominal-vs-prediction; translation >0.12m OR rotation >3deg",
        "input_hashes": freeze["input_hashes"],
        "source_base_commit": subprocess.check_output(["git", "-C", str(REPO), "rev-parse", "HEAD"], text=True).strip(),
        "remote_push": False,
    }
    (ARCHIVE / "experiment_freeze.json").write_text(json.dumps(config, indent=2, allow_nan=False) + "\n")
    (ARCHIVE / "PAIRED_RUN_STARTED").write_text("NO_IMPLICIT_RESTART; fixed 346-scan paired replay\n")
    env = dict(os.environ, OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1",
               LD_LIBRARY_PATH="/lib/x86_64-linux-gnu")
    start = time.monotonic()
    with (ARCHIVE / "run.log").open("x") as log:
        result = subprocess.run(command, env=env, stdout=log, stderr=subprocess.STDOUT)
    receipt = {"exit_code": result.returncode, "wall_s": time.monotonic() - start,
               "GT_LOADED": False, "command": command}
    (ARCHIVE / "process_exit.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps(receipt, indent=2))
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
