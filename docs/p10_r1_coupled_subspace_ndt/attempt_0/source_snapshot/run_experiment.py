"""Run a bounded non-oracle P10 prototype, then freeze outputs for evaluation."""
import argparse
import csv
import hashlib
import json
import os
import pathlib
import shutil
import subprocess
import time

ROOT = pathlib.Path(__file__).resolve().parents[4]
HERE = pathlib.Path(__file__).resolve().parent
ARCHIVE = ROOT / "docs/p10_r1_coupled_subspace_ndt"
INPUT = pathlib.Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/dual_u_r1_closure_20261003/same_objective")
MAP = pathlib.Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/map/frozen/floor01_h1_map_p5_frozen.pcd")
CACHE = pathlib.Path("/home/jian/livox_ws/dog_loc_paper_ws/.p9_experiment_cache/p10_r1_coupled_subspace_ndt")
START_SHA = "1327c767f92ef8a85eb05a3ef049ff396657cd54"
METHODS = ("A_WEAK_ONLY", "B_WARM_NEWTON", "C_COUPLED_PREDICTOR", "D_COUPLED_CORRECTOR")


def sha(path):
    h = hashlib.sha256()
    with pathlib.Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def json_write(path, value):
    with path.open("x") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")


def csv_read(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def csv_write(path, rows):
    with path.open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def verified_inputs():
    historical = json.loads((ROOT / "docs/p9_r1b_strong_attractor_closure/input_provenance.json").read_text())
    checked = {}
    for old_path, expected in historical.items():
        path = pathlib.Path(old_path)
        if "canonical_oracle.csv" in path.name:
            # Digest only; the builder never receives canonical content/path.
            path = ROOT / "docs/p9_r1a_true_profile_closure/canonical_oracle.csv"
        digest = sha(path)
        if digest != expected:
            raise RuntimeError("historical input SHA mismatch: " + str(path))
        checked[str(path)] = digest
    if len(checked) != 6:
        raise RuntimeError("unexpected historical input manifest")
    return checked


def score_winners(probes):
    winners = []
    for tx in ("616", "2226"):
        for method in METHODS:
            pool = [r for r in probes if r["transaction_id"] == tx and r["method"] == method
                    and r["full_ndt_status"] == "SUCCESS"]
            if not pool:
                winners.append({"transaction_id": tx, "method": method, "visit_order": "",
                                "grid_index": "", "refined_score_sum": "", "status": "NO_SUCCESSFUL_TERMINAL"})
                continue
            best = min(pool, key=lambda r: (-float(r["refined_score_sum"]), int(r["visit_order"])))
            winners.append({"transaction_id": tx, "method": method, "visit_order": best["visit_order"],
                            "grid_index": best["grid_index"], "refined_score_sum": best["refined_score_sum"],
                            "status": "DIAGNOSTIC_ONLY_NO_POSE_SWITCH"})
    return winners


def run(binary, revision, reason):
    if revision not in (0, 1, 2):
        raise RuntimeError("at most initial plus two targeted revisions")
    if revision and not reason.strip():
        raise RuntimeError("targeted revision requires recorded reason")
    binary = pathlib.Path(binary).resolve()
    inputs = verified_inputs()
    output = CACHE / ("attempt_" + str(revision))
    output.mkdir(parents=True, exist_ok=False)
    archive = ARCHIVE / ("attempt_" + str(revision))
    archive.mkdir(exist_ok=False)
    if os.statvfs(output).f_bavail * os.statvfs(output).f_frsize < 2 * (1 << 30):
        raise RuntimeError("persistent output has less than 2GiB free reserve")
    files = list(HERE.glob("*.cpp")) + list(HERE.glob("*.hpp")) + list(HERE.glob("*.py")) + [HERE / "CMakeLists.txt"]
    files += [ROOT / "src/dog_prior_map_fastlio2_frontend_exp/scripts/p9" / name
              for name in ("p9_ndt_energy_contract.cpp", "p9_true_profile_closure.cpp", "p9_strong_profile_solvers.hpp")]
    snapshot = archive / "source_snapshot"
    snapshot.mkdir()
    for path in files:
        shutil.copyfile(path, snapshot / path.name)
    # Revision is the preserved development-attempt number, not an engine
    # variant. A targeted change must be represented by its frozen source and
    # reason; the currently defined engine variant remains zero.
    algorithm_variant = 0
    command = [str(binary), str(MAP), str(INPUT / "frozen/cohort_frozen.csv"),
               str(INPUT / "dual_u.csv"), str(output), str(algorithm_variant)]
    environment = dict(os.environ, LD_LIBRARY_PATH="/lib/x86_64-linux-gnu", OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1")
    freeze = {"task": "PAPER-P10-R1-COUPLED-SUBSPACE-NDT-PROTOTYPE", "revision": revision,
              "algorithm_variant": algorithm_variant, "reason": reason,
              "start_sha": START_SHA,
              "implementation_commit": subprocess.check_output(["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True).strip(),
              "command": command, "binary_sha256": sha(binary),
              "source_sha256": {str(p.relative_to(ROOT)): sha(p) for p in files}, "input_sha256": inputs,
              "output_directory": str(output), "GT_LOADED": False, "ORACLE_INITIALIZATION": False,
              "proposal_contract": "original bounded weak grid; nominal zero start; independent deterministic method histories",
              "oracle_file_passed_to_builder": False,
              "NDT": [0.8, 0.08, 1e-5, 80, 0.55], "conditional_max_iterations_B_D": 20,
              "budget_note": "D predictor jet is additional measured work; same node/refine and corrector iteration caps, not identical CPU cost"}
    json_write(archive / "execution_freeze.json", freeze)
    json_write(output / "execution_freeze.json", freeze)
    began = time.monotonic()
    with (archive / "engine.log").open("x") as log:
        process = subprocess.run(["/usr/bin/time", "-v", "-o", str(archive / "resources.txt")] + command,
                                 env=environment, stdout=log, stderr=subprocess.STDOUT, check=False)
    json_write(archive / "execution_receipt.json", {"returncode": process.returncode, "wall_s": time.monotonic()-began,
                                                  "GT_LOADED": False, "scientific_retry_performed": False})
    if process.returncode:
        raise RuntimeError("prototype execution failed; preserve log before deciding a coding fix")
    csv_write(output / "score_selected.csv", score_winners(csv_read(output / "probes.csv")))
    generated = [p for p in output.iterdir() if p.is_file() and p.name != "execution_freeze.json"]
    for path in generated:
        shutil.copyfile(path, archive / path.name)
    json_write(archive / "blind_outputs_freeze.json", {
        "output_sha256": {p.name: sha(p) for p in generated}, "candidate_builder_read_oracle": False,
        "GT_LOADED": False, "score_selection": "max successful refined raw score, tie lowest visit_order; diagnostic only",
        "evaluation_targets_loaded": False})
    print(json.dumps({"revision": revision, "builder": "PASS", "output": str(output),
                      "candidate_outputs": "FROZEN_BEFORE_TARGET_EVALUATION"}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("binary")
    parser.add_argument("--revision", type=int, default=0)
    parser.add_argument("--reason", default="initial original-grid prototype")
    args = parser.parse_args()
    run(args.binary, args.revision, args.reason)
