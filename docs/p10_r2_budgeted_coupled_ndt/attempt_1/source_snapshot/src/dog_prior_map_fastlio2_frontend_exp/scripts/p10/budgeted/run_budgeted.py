"""Bounded P10-R2 experiments. Builder inputs never contain GT/canonical poses."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

ROOT = Path(__file__).resolve().parents[5]
HERE = Path(__file__).resolve().parent
ARCHIVE = ROOT / "docs/p10_r2_budgeted_coupled_ndt"
CACHE = Path("/home/jian/livox_ws/dog_loc_paper_ws/.p9_experiment_cache/p10_r2_budgeted_coupled_ndt")
SAME = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/dual_u_r1_closure_20261003/same_objective")
MAP = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/map/frozen/floor01_h1_map_p5_frozen.pcd")
START_SHA = "69d84d9ccef0776e2fdff9e88bb2783b34af3f1d"


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def read(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def csv_write(path, rows):
    if not rows:
        raise RuntimeError("refusing empty scientific CSV")
    with Path(path).open("x", newline="") as stream:
        writer = csv.DictWriter(stream, list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def json_write(path, data):
    with Path(path).open("x") as stream:
        json.dump(data, stream, indent=2, allow_nan=False)
        stream.write("\n")


def inputs():
    historical = json.loads((ROOT / "docs/p9_r4_heldout_visual_evidence/source_recovery/replay_input_hashes.json").read_text())
    names = ("imu_csv", "filter_scans_csv", "raw_timed_scan_index", "raw_timed_point_bin", "params_txt", "map_pcd")
    files = {k: historical["inputs"][k]["path"] for k in names}
    hashes = {}
    for key, path in files.items():
        actual = sha(path)
        if actual != historical["inputs"][key]["sha256"]:
            raise RuntimeError("historical input mismatch: " + key)
        hashes[path] = actual
    if sha(SAME / "registration.csv") != historical["original_registration_sha256"]:
        raise RuntimeError("nominal archive hash mismatch")
    hashes[str(SAME / "registration.csv")] = sha(SAME / "registration.csv")
    cohort = SAME / "frozen/cohort_frozen.csv"
    if sha(cohort) != "24513cc9ffe73b817e5da7f40d4e025551a006220fc7f0ea44996d7f89d44a42":
        raise RuntimeError("32-frame cohort changed")
    hashes[str(cohort)] = sha(cohort)
    frames = []
    for row in read(cohort):
        if sha(row["raw_cloud_file"]) != row["raw_source_sha256"]:
            raise RuntimeError("raw cloud hash mismatch")
        hashes[row["raw_cloud_file"]] = row["raw_source_sha256"]
        frames.append(dict(transaction_id=row["transaction_id"], stamp_ns=row["stamp_ns"],
                           raw_cloud_file=row["raw_cloud_file"], raw_point_count=row["raw_point_count"],
                           source_count=row["prepared_source_point_count"], source_hash=row["prepared_source_hash"],
                           target_count=row["target_point_count"], prediction_pose=row["initial_pose_xyz_q_xyzw"]))
    if len(frames) != 32:
        raise RuntimeError("wrong integration cohort size")
    return files, hashes, frames


def run(build, attempt, reason):
    if attempt not in (0, 1, 2) or not reason.strip():
        raise RuntimeError("initial plus at most two reasoned improvements")
    build = Path(build).resolve()
    files, hashes, frames = inputs()
    archive, cache = ARCHIVE / f"attempt_{attempt}", CACHE / f"attempt_{attempt}"
    archive.mkdir(parents=True, exist_ok=False)
    cache.mkdir(parents=True, exist_ok=False)
    if shutil.disk_usage(cache).free < 2 * (1 << 30):
        raise RuntimeError("less than 2GiB persistent reserve")
    csv_write(cache / "integration_manifest.csv", frames)
    csv_write(cache / "two_frame_manifest.csv", [r for r in frames if r["transaction_id"] in ("616", "2226")])
    for name in ("integration_manifest.csv", "two_frame_manifest.csv"):
        shutil.copyfile(cache / name, archive / name)
    source_files = list(HERE.glob("*.cpp")) + list(HERE.glob("*.hpp")) + list(HERE.glob("*.py"))
    package = ROOT / "src/dog_prior_map_fastlio2_frontend_exp"
    source_files += [package / p for p in ("src/coupled_ndt_shadow.cpp", "src/current_frame_ndt.cpp",
        "include/dog_prior_map_fastlio2_frontend_exp/coupled_ndt_shadow.hpp",
        "include/dog_prior_map_fastlio2_frontend_exp/current_frame_ndt.hpp", "scripts/p7/CMakeLists.txt",
        "scripts/p7/p7_single_state_runner.cpp", "src/scan_processor.cpp", "src/p7_replay_io.cpp",
        "src/fastlio2_frontend_ikfom.cpp")]
    freeze = dict(task="PAPER-P10-R2-BUDGETED-COUPLED-NDT", attempt=attempt, reason=reason,
        start_sha=START_SHA, input_sha256=hashes,
        source_sha256={str(p.relative_to(ROOT)): sha(p) for p in source_files},
        theory_sha256=sha(ARCHIVE/"THEORY.md"),
        binary_sha256={name: sha(build / name) for name in ("p10_r2_single", "p10_r2_replay")},
        builder_columns=list(frames[0]), ORACLE_INITIALIZATION=False, ORACLE_RANKING=False, GT_LOADED_BY_BUILDER=False,
        continuous_selection="original scan indices TX1-200, selected before experiments; no GT criterion",
        initial_candidates=8, maximum_candidates=16, maximum_extra_ndt=2, nominal_ndt=1,
        controlled_ablation="A/B/C same fixed16 covering proposals and max2 extra alignments; derivative work separately counted",
        integration="32 historical SAME_OBJECTIVE singles followed by TX1-200 causal replay control and A/B/C shadow",
        rank_rule="meanE/max(1,abs(Enom)) + .05*((dtPrediction/2m)^2+(drPrediction/15deg)^2)",
        recommendation="successful objective-better by frozen score tolerance and lower merit, else nominal; SHADOW ONLY",
        output_directory=str(cache))
    json_write(archive / "execution_freeze.json", freeze)
    snapshot=archive/"source_snapshot"
    snapshot.mkdir()
    for path in source_files:
        destination=snapshot/path.relative_to(ROOT)
        destination.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(path,destination)
    binary_snapshot=cache/"binary_snapshot"
    binary_snapshot.mkdir()
    for name in ("p10_r2_single","p10_r2_replay"):
        shutil.copyfile(build/name,binary_snapshot/name)
    if attempt:
        json_write(archive/"improvement_receipt.json",dict(reason=reason,previous_attempt_preserved=True,
            previous_posthoc_GT_inspected=True,modification_basis="cost and objective-preview diagnostics only; not GT tuning",
            contract_sha256=sha(ARCHIVE/"TARGETED_IMPROVEMENT_1.md")))
    environment = dict(os.environ, LD_LIBRARY_PATH="/lib/x86_64-linux-gnu", OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1")
    jobs = [(f"two_{method}", [str(build / "p10_r2_single"), str(MAP), str(cache / "two_frame_manifest.csv"),
                              "OUTPUT", method, "1"]) for method in "ABC"]
    jobs += [("integration_C", [str(build / "p10_r2_single"), str(MAP), str(cache / "integration_manifest.csv"), "OUTPUT", "C", "0"])]
    for method in ("control", "A", "B", "C"):
        jobs.append((f"continuous_{method}", [str(build / "p10_r2_replay"), files["imu_csv"], files["filter_scans_csv"],
            files["raw_timed_scan_index"], files["raw_timed_point_bin"], files["map_pcd"], files["params_txt"],
            "TRAJECTORY", "REGISTRATION", "RUNTIME", "200", "0", "control" if method=="control" else "shadow",
            "C" if method=="control" else method]))
    receipts = []
    for name, command in jobs:
        destination, record = cache / name, archive / name
        destination.mkdir(); record.mkdir()
        substitutions = dict(OUTPUT=str(destination), TRAJECTORY=str(destination / "trajectory.csv"),
                             REGISTRATION=str(destination / "registration.csv"), RUNTIME=str(destination / "runtime.csv"))
        command = [substitutions.get(part, part) for part in command]
        json_write(record / "command.json", command)
        began = time.monotonic()
        with (record / "engine.log").open("x") as log:
            process = subprocess.run(command, env=environment, stdout=log, stderr=subprocess.STDOUT, check=False)
        receipt = dict(job=name, returncode=process.returncode, wall_s=time.monotonic()-began)
        receipts.append(receipt); json_write(record / "receipt.json", receipt)
        for path in destination.glob("*.csv"):
            shutil.copyfile(path, record / path.name)
        if process.returncode:
            raise RuntimeError("execution failed, preserve original log: " + name)
        print(json.dumps(receipt), flush=True)
    csv_write(archive / "job_cost.csv", receipts)
    outputs = {str(p.relative_to(archive)): sha(p) for p in archive.rglob("*.csv")}
    json_write(archive / "blind_outputs_freeze.json", dict(output_sha256=outputs, GT_LOADED=False,
        CANONICAL_LOADED=False, NOMINAL_STATE_SWITCHED=False))
    print("BLIND_OUTPUTS_FROZEN", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("build"); parser.add_argument("--attempt", type=int, default=0)
    parser.add_argument("--reason", default="initial bounded residual-corrected shadow prototype")
    args = parser.parse_args()
    run(args.build, args.attempt, args.reason)
