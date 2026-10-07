#!/usr/bin/env python3
"""Freeze predictor-conditioned proposals, then execute each selected call once."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time

import numpy as np
from run_r2_terminal_stability import (ARCHIVE, MAP, ROOT, FRAME_ORDER, digest,
                                       read_csv, write_csv, require, vector, text, haar_basis)

HERE = Path(__file__).resolve().parent
OUT = ROOT / "docs/p9_r2a_predictor_conditioned_search"
R2 = ROOT / "docs/p9_r2_terminal_stability"
H2 = ROOT / "docs/p9_foundation_weak_discovery/h2"
START_SHA = "cfd76389688c72f181b294c316c5cd1911359ecb"
BRANCH = "research/p9-r2a-predictor-conditioned-search"
METHODS = (("COND_WEAK2", -1), ("COND_STRONG2", -1)) + tuple(("COND_RANDOM2", i) for i in range(3))
BUDGETS = (4, 8, 12, 16)
ENV = dict(os.environ, LD_LIBRARY_PATH="/lib/x86_64-linux-gnu", OMP_NUM_THREADS="1",
           PYTHONDONTWRITEBYTECODE="1")
DEFAULT_BINARY = Path("/tmp/p9_r2_build/p9_r2a_predictor_search")


def git_state():
    branch = subprocess.check_output(["git", "branch", "--show-current"], cwd=ROOT, text=True).strip()
    head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    require(branch == BRANCH, "incorrect R2A branch")
    subprocess.run(["git", "merge-base", "--is-ancestor", START_SHA, "HEAD"], cwd=ROOT, check=True)
    return dict(branch=branch, start_sha=START_SHA, execution_head=head)


def farthest(rows, budget=16):
    by_seed = {int(row["seed_index"]): row for row in rows}
    require(len(rows) == 263 and sorted(by_seed) == list(range(263)), "base seed coverage invalid")
    points = np.array([[float(by_seed[j]["coord0"]), float(by_seed[j]["coord1"])] for j in range(263)])
    require(np.isfinite(points).all() and np.linalg.norm(points[122]) <= 1e-12,
            "predictor seed122 must have zero selection coordinate")
    order = [122]
    distances = np.sum((points - points[122]) ** 2, axis=1)
    selected = [(1, 122, 0.0, by_seed[122])]
    while len(order) < budget:
        distances[order] = -np.inf
        largest = float(np.max(distances))
        tolerance = 1e-12 * max(1.0, abs(largest))
        seed = int(np.flatnonzero(distances >= largest - tolerance)[0])
        selected.append((len(order) + 1, seed, float(np.sqrt(max(0., distances[seed]))), by_seed[seed]))
        order.append(seed)
        distances = np.minimum(distances, np.sum((points - points[seed]) ** 2, axis=1))
    return selected


def check_inputs():
    git = git_state()
    previous = json.loads((R2 / "execution_manifest.json").read_text())
    frozen = subprocess.check_output(["git", "show", START_SHA + ":docs/p9_r2_terminal_stability/execution_manifest.json"], cwd=ROOT)
    import hashlib
    require(digest(R2 / "execution_manifest.json") == hashlib.sha256(frozen).hexdigest(), "committed R2 manifest changed")
    hashes = {}
    for name, expected in previous["input_sha256"].items():
        path = Path(name)
        if str(path).startswith("/media/") or str(path).startswith("/usr/include/pcl-"):
            require(path.is_file() and digest(path) == expected, "frozen archive/header differs: " + name)
            hashes[name] = expected
    cohort = read_csv(ARCHIVE / "frozen/cohort_frozen.csv")
    require(len(cohort) == 32 and [int(row["transaction_id"]) for row in cohort] == list(FRAME_ORDER),
            "32-frame frozen cohort/order mismatch")
    baseline_path = ARCHIVE / "clusters/manifest_current_baseline.csv"
    baseline = {int(row["transaction_id"]): row for row in read_csv(baseline_path)}
    for row in cohort:
        tx = int(row["transaction_id"])
        require(baseline[tx]["predicted_pose_xyz_q_xyzw"] == row["initial_pose_xyz_q_xyzw"] ==
                baseline[tx]["initial_pose_xyz_q_xyzw"], "frozen initial pose is not the saved predictor")
        require(baseline[tx]["stamp_ns"] == row["stamp_ns"], "predictor timestamp mismatch")
    hashes[str(baseline_path)] = digest(baseline_path)
    generator_sha = subprocess.check_output(["git", "rev-parse", "research/dual-u-architecture-r1"], cwd=ROOT, text=True).strip()
    generator_path = "src/dog_prior_map_fastlio2_frontend_exp/scripts/p7/dual_u_r1_multistart_runner.cpp"
    generator = subprocess.check_output(["git", "show", generator_sha + ":" + generator_path], cwd=ROOT)
    require(b"rightPerturb(frame.initial_pose, seed)" in generator and b"nominal_prediction_seed_missing" in generator,
            "historical zero-seed generator provenance not recovered")
    provenance = dict(generator_git_sha=generator_sha, generator_path=generator_path,
                      generator_sha256=hashlib.sha256(generator).hexdigest(),
                      explicit_predictor_field="cohort.initial_pose_xyz_q_xyzw = saved predicted_pose_xyz_q_xyzw")
    return git, hashes, provenance


def prepare(binary):
    git, hashes, provenance = check_inputs()
    OUT.mkdir(parents=True, exist_ok=True)
    require(not (OUT / "execution_manifest.json").exists(), "prepared manifest exists; refusing overwrite")
    require(not any((OUT / "runs").glob("*")), "NDT output already exists")
    require(binary.is_file(), "Release binary missing")
    (OUT / "runs").mkdir(exist_ok=True)
    random = read_csv(R2 / "random_subspaces.csv")
    require(len(random) == 96, "three frozen R2 random bases per frame missing")
    for index, frame in enumerate(FRAME_ORDER):
        for rep in range(3):
            row = next(r for r in random if int(r["frame"]) == frame and int(r["random_rep"]) == rep)
            seed = 20261007 + 1000 * index + rep
            require(int(row["seed"]) == seed and row["rng"] == "PCG64" and
                    np.array_equal(vector(row["basis_rowmajor"], 12).reshape(6, 2), haar_basis(seed)),
                    "R2 pre-frozen random basis differs from deterministic schedule")
    write_csv(OUT / "random_subspaces.csv", random)
    started = time.perf_counter()
    command = [str(binary), "--prepare", str(ARCHIVE / "frozen/cohort_frozen.csv"), str(ARCHIVE / "dual_u.csv"),
               str(ARCHIVE / "candidates.csv"), str(OUT / "random_subspaces.csv"), str(OUT)]
    result = subprocess.run(command, cwd=ROOT, env=ENV, text=True, capture_output=True)
    (OUT / "proposal_stdout.log").write_text(result.stdout + result.stderr)
    require(result.returncode == 0 and "R2A_PREDICTOR_PARITY=PASS PROPOSAL_PARITY=PASS" in result.stdout,
            "predictor/proposal parity failed; inspect proposal_stdout.log")
    pool = read_csv(OUT / "conditioned_proposals.csv")
    require(len(pool) == 32 * 263 * 5, "conditional proposal pool incomplete")
    groups = {}
    for row in pool:
        groups.setdefault((int(row["frame"]), row["method"], int(row["random_rep"])), []).append(row)
    probes = []
    for tx in FRAME_ORDER:
        for method, rep in METHODS:
            for rank, seed, distance, row in farthest(groups[tx, method, rep]):
                probes.append(dict(row, probe_rank=rank, selection_distance=format(distance, ".17g")))
    write_csv(OUT / "probe_manifest.csv", probes)
    require(len(probes) == 2560, "selected probe count mismatch")
    prep_seconds = time.perf_counter() - started
    code = [HERE / name for name in ("p9_r2a_predictor_search.cpp", "p9_ndt_energy_contract.cpp",
                                    "run_r2a_predictor_search.py", "run_r2_terminal_stability.py", "CMakeLists.txt")]
    hashes.update({str(path): digest(path) for path in code})
    theory = OUT / "THEORY.md"
    require(theory.is_file(), "pre-run mathematical and decision contract missing")
    hashes[str(theory)] = digest(theory)
    proposal_hashes = {str(OUT / name): digest(OUT / name) for name in (
        "random_subspaces.csv", "predictor_parity.csv", "proposal_roundtrip.csv", "conditioned_proposals.csv", "probe_manifest.csv")}
    hashes.update(proposal_hashes)
    manifest = dict(task="PAPER-P9-R2A-PREDICTOR-CONDITIONED-WEAK-SUBSPACE-SEARCH", git=git,
        current_run_state="PREPARED", input_sha256=hashes, generator_provenance=provenance,
        binary=str(binary), binary_sha256=digest(binary), build_type="Release", pcl="1.10",
        ndt=dict(resolution=.8, outlier_ratio=.55, step=.08, epsilon=1e-5, max_iterations=80),
        proposal_formula="eta_pred + Q Q^T (eta_j - eta_pred)", seed_pool="archived 0..262; predictor=122",
        selection="fixed first122; deterministic farthest point; lowest seed index tie",
        budgets=list(BUDGETS), full_prefix_budgets=list(range(1,17)), random_bases="pre-frozen R2 bases reused",
        random_rng="PCG64", random_seed_formula="20261007 + 1000*cohort_frame_index + replicate(0..2)",
        preparation_seconds=prep_seconds, planned_new_ndt_calls=2560, full6d_new_calls=0,
        oracle_used_for_proposals=False, gt_used_for_proposals=False, source_preprocessing="unchanged frozen R1",
        target_preprocessing="finite filter + two 0.15m voxel grids")
    (OUT / "execution_manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(json.dumps(dict(stage="PREPARED", predictor_parity="PASS", proposals=len(pool), probes=len(probes),
                         preparation_seconds=prep_seconds), indent=2))


def verify_manifest():
    git_state()
    manifest = json.loads((OUT / "execution_manifest.json").read_text())
    for name, expected in manifest["input_sha256"].items():
        require(Path(name).is_file() and digest(name) == expected, "frozen input/code changed: " + name)
    require(digest(manifest["binary"]) == manifest["binary_sha256"], "Release binary hash changed")
    return manifest


def run():
    manifest = verify_manifest()
    require(manifest["current_run_state"] == "PREPARED" and not any((OUT / "runs").glob("*")),
            "existing/started alignments preserved; refusing repeat")
    manifest["current_run_state"] = "RUNNING"
    (OUT / "execution_manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    command = [manifest["binary"], "--align-all", str(MAP), str(ARCHIVE / "frozen/cohort_frozen.csv"),
               str(ARCHIVE / "dual_u.csv"), str(OUT / "probe_manifest.csv"), str(OUT)]
    started = time.perf_counter()
    with (OUT / "engine_stdout.log").open("w") as log:
        process = subprocess.Popen(command, cwd=ROOT, env=ENV, text=True, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, bufsize=1)
        for line in process.stdout:
            log.write(line); log.flush(); print(line, end="", flush=True)
        result = process.wait()
    require(result == 0, "NDT execution failed; completed/partial artifacts preserved")
    runs = []
    for tx in FRAME_ORDER:
        for method, rep in METHODS:
            rows = read_csv(OUT / "runs" / f"tx_{tx}_{method}_r{rep}.csv")
            require(len(rows) == 16, "incomplete result shard")
            runs.extend(rows)
    probes = read_csv(OUT / "probe_manifest.csv")
    keys = lambda r: (int(r["frame"]), r["method"], int(r["random_rep"]), int(r["probe_rank"]))
    by_probe = {keys(row): row for row in probes}
    cohort = {int(r["transaction_id"]): r for r in read_csv(ARCHIVE / "frozen/cohort_frozen.csv")}
    require(len(runs) == len({keys(r) for r in runs}) == 2560, "NDT rows incomplete/duplicated")
    for row in runs:
        expected = by_probe[keys(row)]; source = cohort[int(row["frame"])]
        require(row["start_pose_matrix16"] == expected["start_pose_matrix16"] and row["seed_index"] == expected["seed_index"],
                "selected proposal/result parity failed")
        require(row["source_hash"] == source["prepared_source_hash"] and row["source_points"] == source["prepared_source_point_count"]
                and row["target_points"] == "549606", "NDT input parity failed")
        vector(row["terminal_pose_matrix16"],16); vector(row["terminal_pose_xyz_q_xyzw"],7)
        require(row["converged"] in ("0","1") and 0<=int(row["iterations"])<=80 and
                np.isfinite([float(row[k]) for k in ("runtime_ms","raw_ndt_score_sum","nominal_ndt_score_sum")]).all(),
                "invalid terminal/status")
    write_csv(OUT / "ndt_runs.csv", runs)
    manifest.update(current_run_state="RUN_COMPLETE", completed_new_ndt_calls=len(runs),
                    ndt_runs_sha256=digest(OUT / "ndt_runs.csv"), run_wall_seconds=time.perf_counter()-started)
    (OUT / "execution_manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print("R2A_RUN_COMPLETE CALLS=2560", flush=True)


def self_test():
    rows = [dict(seed_index=j,coord0=(0.0 if j==122 else .001*(j+1)),coord1=0.) for j in range(263)]
    rows[0].update(coord0=1.,coord1=0.);rows[1].update(coord0=-1.,coord1=0.)
    selected = farthest(rows)
    require(selected[0][1]==122 and selected[1][1]==0 and selected[2][1]==1,
            "fixed predictor/tie selection regression")
    require(len({row[1] for row in selected})==16 and selected==farthest(list(reversed(rows))),
            "selection depends on source row order")
    print("R2A_ORCHESTRATION_SELF_TEST=PASS")


if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("stage",choices=("prepare","run","self-test"))
    parser.add_argument("--binary",type=Path,default=DEFAULT_BINARY)
    args=parser.parse_args()
    if args.stage=="prepare": prepare(args.binary.resolve())
    elif args.stage=="run": run()
    else: self_test()
