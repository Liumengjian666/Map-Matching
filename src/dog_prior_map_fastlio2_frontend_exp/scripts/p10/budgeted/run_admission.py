"""R4 full causal replay; this builder never reads GT/canonical labels."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
from run_budgeted import ROOT, sha, read, csv_write, json_write

ARCHIVE = ROOT / "docs/p10_r4_nonoracle_branch_admission"
CACHE = Path("/home/jian/livox_ws/dog_loc_paper_ws/.p9_experiment_cache/p10_r4_nonoracle_branch_admission")
START_SHA = "b87504877cab938fdfc606e4d93ed5ad5be931fc"
CONTROL = ROOT / "docs/p10_r3_event_triggered_coupled_ndt/attempt_0/control"


def run(build, attempt):
    if attempt not in (0, 1):
        raise RuntimeError("initial plus at most one admission-only improvement")
    if attempt and not (ARCHIVE / "TARGETED_IMPROVEMENT_1.md").exists():
        raise RuntimeError("improvement reason/contract must be frozen first")
    if subprocess.check_output(["git", "-C", str(ROOT), "diff", "--name-only", "HEAD"], text=True).strip():
        raise RuntimeError("tracked code must be committed before replay")
    subprocess.run(["git", "-C", str(ROOT), "merge-base", "--is-ancestor", START_SHA, "HEAD"], check=True)
    lineage = ROOT / "docs/p9_r4_heldout_visual_evidence/source_recovery/replay_input_hashes.json"
    inputs = json.loads(lineage.read_text())["inputs"]
    keys = ("imu_csv", "filter_scans_csv", "raw_timed_scan_index", "raw_timed_point_bin", "params_txt", "map_pcd")
    files, hashes = {}, {str(lineage): sha(lineage)}
    for key in keys:
        path = inputs[key]["path"]
        if sha(path) != inputs[key]["sha256"]:
            raise RuntimeError("frozen input mismatch: " + key)
        files[key] = path; hashes[path] = inputs[key]["sha256"]
    scans = read(files["raw_timed_scan_index"])
    ids = [int(r["transaction_id"]) for r in scans]
    if ids != list(range(1, 4128)):
        raise RuntimeError("full immutable input denominator/order failed")
    previous = ROOT / "docs/p10_r3_event_triggered_coupled_ndt/attempt_0/blind_outputs_freeze.json"
    old_hashes = json.loads(previous.read_text())["output_sha256"]
    control_hashes = {}
    for path in CONTROL.glob("*.csv"):
        if sha(path) != old_hashes["control/" + path.name]:
            raise RuntimeError("R3 CONTROL receipt changed: " + path.name)
        control_hashes[str(path.relative_to(ROOT))] = sha(path)
    package = ROOT / "src/dog_prior_map_fastlio2_frontend_exp"
    # Algorithm core, filter, time/deskew and original input reader are immutable.
    unchanged = ["src/coupled_ndt_shadow.cpp", "src/fastlio2_frontend_ikfom.cpp",
        "src/p7_replay_io.cpp", "src/scan_processor.cpp", "src/lidar_deskew_geometry.cpp"]
    for name in unchanged:
        path = package / name
        original = subprocess.check_output(["git", "-C", str(ROOT), "show", START_SHA+":"+str(path.relative_to(ROOT))])
        if path.read_bytes() != original:
            raise RuntimeError("frozen core changed: " + name)
    paths = [package / name for name in unchanged + ["src/coupled_ndt_temporal.cpp", "src/current_frame_ndt.cpp",
        "src/registration_geometry.cpp", "include/dog_prior_map_fastlio2_frontend_exp/coupled_ndt_shadow.hpp",
        "include/dog_prior_map_fastlio2_frontend_exp/current_frame_ndt.hpp", "scripts/p7/CMakeLists.txt",
        "scripts/p10/budgeted/p10_r2_replay.cpp", "scripts/p10/budgeted/shadow_logging.hpp",
        "scripts/p10/budgeted/run_admission.py", "scripts/p10/budgeted/test_admission.cpp"]]
    archive, cache = ARCHIVE / f"attempt_{attempt}", CACHE / f"attempt_{attempt}"
    archive.mkdir(parents=True, exist_ok=False); cache.mkdir(parents=True, exist_ok=False)
    if shutil.disk_usage(cache).free < 2*(1<<30):
        raise RuntimeError("insufficient persistent capacity")
    csv_write(archive / "frame_selection.csv", [dict(transaction_id=tx,selected=1) for tx in ids])
    binary = cache / "p10_r4_replay"
    shutil.copy2(Path(build) / "p10_r2_replay", binary)
    freeze = dict(task="PAPER-P10-R4-NONORACLE-BRANCH-ADMISSION-AND-CAUSAL-REPLAY",attempt=attempt,
        start_sha=START_SHA,code_sha=subprocess.check_output(["git","-C",str(ROOT),"rev-parse","HEAD"],text=True).strip(),
        input_sha256=hashes,control_sha256=control_hashes,source_sha256={str(p.relative_to(ROOT)):sha(p) for p in paths},
        binary_sha256=sha(binary),theory_sha256=sha(ARCHIVE/"THEORY.md"),output_directory=str(cache),
        total_frames=len(ids),GT_LOADED=False,ORACLE_LOADED=False,PRODUCTION_CHANGED=False,
        trigger_translation_m=.12,trigger_rotation_deg=3,window_frames=3,confirmations=2,
        motion_translation_scale_m=2,motion_rotation_scale_deg=15,motion_weight=.05,tie_tolerance=1e-6,
        max_previews=16,max_complete_aligns=3,max_pending_extra_aligns=1,
        IMU_INTERVAL="inverse(actual previous corrected map_T_lidar)*current predicted map_T_lidar",
        R3_ALTERNATIVE_PROPAGATION="unchanged previous alternative*inverse(previous prediction)*current prediction",
        initial_pending_rank="unchanged R3 absolute prediction merit; not GT",
        feedback="only admitted current alternative via existing lidarMeasurementToImu/applyPoseMeasurement",
        timing="full scan-end processing plus logging except cost row; process wall also provided",
        large_jump="consecutive corrected translation>0.5m OR rotation>10deg; descriptive")
    freeze["rotation_guard_enabled"] = bool(attempt)
    if attempt:
        freeze["improvement_contract_sha256"] = sha(ARCHIVE/"TARGETED_IMPROVEMENT_1.md")
        freeze["prior_posthoc_GT_inspected"] = True
        freeze["improvement_basis"] = "persistent rotation offset not distinguishable by incremental residual; no per-frame GT rule"
    json_write(archive / "execution_freeze.json", freeze)
    env = dict(os.environ,LD_LIBRARY_PATH="/lib/x86_64-linux-gnu",OMP_NUM_THREADS="1",OPENBLAS_NUM_THREADS="1")
    receipts=[]
    for mode in ("event_admission","guarded_feedback"):
        destination,record=cache/mode,archive/mode
        destination.mkdir();record.mkdir()
        command=[str(binary),*[files[k] for k in keys[:4]],files["map_pcd"],files["params_txt"],
            str(destination/"trajectory.csv"),str(destination/"registration.csv"),str(destination/"runtime.csv"),
            str(len(ids)),"0",mode+"_rotation_guard" if attempt else mode,"C"]
        json_write(record/"command.json",command)
        began=time.monotonic()
        with (record/"engine.log").open("x") as log:
            process=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,env=env,check=False)
        receipt=dict(mode=mode,returncode=process.returncode,wall_s=time.monotonic()-began)
        receipts.append(receipt);json_write(record/"receipt.json",receipt)
        for path in destination.glob("*.csv"):
            shutil.copyfile(path,record/path.name)
        print(json.dumps(receipt),flush=True)
        if process.returncode:
            json_write(archive/"failed_execution_freeze.json",dict(GT_LOADED=False,failed_mode=mode,
                output_sha256={str(p.relative_to(archive)):sha(p) for p in archive.rglob("*.csv")}))
            raise RuntimeError("causal replay failed; stop, preserve log; no nominal fallback")
    csv_write(archive/"job_cost.csv",receipts)
    json_write(archive/"blind_outputs_freeze.json",dict(GT_LOADED=False,ORACLE_LOADED=False,
        FEEDBACK_EXPERIMENT_ONLY=True,output_sha256={str(p.relative_to(archive)):sha(p) for p in archive.rglob("*.csv")}))
    print("BLIND_OUTPUTS_FROZEN",flush=True)


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("build");parser.add_argument("--attempt",type=int,default=0)
    args=parser.parse_args();run(args.build,args.attempt)
