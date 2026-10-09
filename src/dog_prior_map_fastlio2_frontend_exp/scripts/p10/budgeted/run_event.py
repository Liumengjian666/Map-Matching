"""P10-R3 full-sequence builder: no GT, oracle, labels or state switching."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
from run_budgeted import ROOT, sha, read, csv_write, json_write

ARCHIVE = ROOT / "docs/p10_r3_event_triggered_coupled_ndt"
CACHE = Path("/home/jian/livox_ws/dog_loc_paper_ws/.p9_experiment_cache/p10_r3_event_triggered_coupled_ndt")
START_SHA = "489ac45653796617477e156d5b9eadf415d8ccf0"


def run(build, attempt):
    if attempt not in (0, 1):
        raise RuntimeError("initial version plus at most one engineering improvement")
    if attempt and not (ARCHIVE / "TARGETED_IMPROVEMENT_1.md").exists():
        raise RuntimeError("improvement must be reasoned and frozen first")
    build = Path(build).resolve()
    lineage = ROOT / "docs/p9_r4_heldout_visual_evidence/source_recovery/replay_input_hashes.json"
    historical = json.loads(lineage.read_text())
    keys = ("imu_csv", "filter_scans_csv", "raw_timed_scan_index", "raw_timed_point_bin", "params_txt", "map_pcd")
    files = {key: historical["inputs"][key]["path"] for key in keys}
    hashes = {str(lineage): sha(lineage)}
    for key, path in files.items():
        actual = sha(path)
        if actual != historical["inputs"][key]["sha256"]:
            raise RuntimeError("frozen input mismatch: " + key)
        hashes[path] = actual
    scans = read(files["raw_timed_scan_index"])
    ids = [int(row["transaction_id"]) for row in scans]
    if not ids or ids != sorted(set(ids)):
        raise RuntimeError("nonmonotonic/duplicate frozen scan index")
    archive, cache = ARCHIVE / f"attempt_{attempt}", CACHE / f"attempt_{attempt}"
    archive.mkdir(parents=True, exist_ok=False)
    cache.mkdir(parents=True, exist_ok=False)
    if shutil.disk_usage(cache).free < 2 * (1 << 30):
        raise RuntimeError("insufficient persistent output reserve")
    csv_write(archive / "frame_selection.csv", [dict(transaction_id=tx, selected_full=1,
        segment_1_200=int(1 <= tx <= 200), segment_616=int(abs(tx-616) <= 10),
        segment_2350=int(abs(tx-2350) <= 10), segment_3341=int(abs(tx-3341) <= 10)) for tx in ids])
    code_sha = subprocess.check_output(["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True).strip()
    changed = subprocess.check_output(["git", "-C", str(ROOT), "diff", "--name-only", "HEAD"], text=True)
    if changed.strip():
        raise RuntimeError("tracked code must be committed before real execution")
    package = ROOT / "src/dog_prior_map_fastlio2_frontend_exp"
    paths = [package / p for p in ("src/coupled_ndt_shadow.cpp", "src/coupled_ndt_temporal.cpp",
        "src/current_frame_ndt.cpp", "include/dog_prior_map_fastlio2_frontend_exp/coupled_ndt_shadow.hpp",
        "include/dog_prior_map_fastlio2_frontend_exp/current_frame_ndt.hpp", "src/scan_processor.cpp",
        "src/p7_replay_io.cpp", "src/fastlio2_frontend_ikfom.cpp", "src/lidar_deskew_geometry.cpp",
        "scripts/p7/CMakeLists.txt", "scripts/p10/budgeted/p10_r2_replay.cpp",
        "scripts/p10/budgeted/shadow_logging.hpp", "scripts/p10/budgeted/run_event.py")]
    binary = cache / "p10_r3_replay"
    shutil.copy2(build / "p10_r2_replay", binary)
    freeze = dict(task="PAPER-P10-R3-EVENT-TRIGGERED-COUPLED-NDT", start_sha=START_SHA,
        code_sha=code_sha, input_sha256=hashes,
        source_sha256={str(p.relative_to(ROOT)): sha(p) for p in paths}, binary_sha256=sha(binary),
        theory_sha256=sha(ARCHIVE / "THEORY.md"), selection_sha256=sha(archive / "frame_selection.csv"),
        output_directory=str(cache), total_frames=len(ids), first_transaction=ids[0], last_transaction=ids[-1],
        GT_LOADED=False, ORACLE_LOADED=False, NOMINAL_STATE_SWITCHED=False,
        trigger_translation_m=.12, trigger_rotation_deg=3, confirmations_required=2,
        max_prediction_gap_s=.25, initial_previews=8, max_previews=16, max_extra_aligns=2,
        max_total_aligns=3, near_quality_fraction=.05, confirmation_dt_m=.2, confirmation_dr_deg=2,
        nonlocal_recommendation=False, R2_math_changed=False,
        timing="frame processing plus CSV serialization except its own cost row; process wall separately",
        peak_RSS="getrusage process high-water mark; incremental difference is only a proxy")
    json_write(archive / "execution_freeze.json", freeze)
    env = dict(os.environ, LD_LIBRARY_PATH="/lib/x86_64-linux-gnu", OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1")
    receipts = []
    for mode in ("control", "event"):
        destination, record = cache / mode, archive / mode
        destination.mkdir(); record.mkdir()
        command = [str(binary), files["imu_csv"], files["filter_scans_csv"], files["raw_timed_scan_index"],
            files["raw_timed_point_bin"], files["map_pcd"], files["params_txt"],
            str(destination / "trajectory.csv"), str(destination / "registration.csv"), str(destination / "runtime.csv"),
            str(len(ids)), "0", mode, "C"]
        json_write(record / "command.json", command)
        start = time.monotonic()
        with (record / "engine.log").open("x") as stream:
            process = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, env=env, check=False)
        receipt = dict(mode=mode, returncode=process.returncode, wall_s=time.monotonic()-start)
        receipts.append(receipt); json_write(record / "receipt.json", receipt)
        for path in destination.glob("*.csv"):
            shutil.copyfile(path, record / path.name)
        print(json.dumps(receipt), flush=True)
        if process.returncode:
            raise RuntimeError("real replay failed; original log retained: " + mode)
    csv_write(archive / "job_cost.csv", receipts)
    json_write(archive / "blind_outputs_freeze.json", dict(
        output_sha256={str(p.relative_to(archive)): sha(p) for p in archive.rglob("*.csv")},
        GT_LOADED=False, ORACLE_LOADED=False, NOMINAL_STATE_SWITCHED=False))
    print("BLIND_OUTPUTS_FROZEN", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("build"); parser.add_argument("--attempt", type=int, default=0)
    args = parser.parse_args(); run(args.build, args.attempt)
