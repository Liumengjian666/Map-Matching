#!/usr/bin/env python3
"""Frozen matched proposals and one run per seed; permutations never run NDT."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess

import numpy as np
from scipy.spatial.transform import Rotation

from run_h1_concentration import (ARCHIVE, FRAMES, ROOT, digest, haar_subspaces,
                                  read_csv, require, vector)

H1_SHA = "a4a89dd3d775a250a58fbdbf2b8d4838594eafa8"
OUT = ROOT / "docs/p9_foundation_weak_discovery/h2"
H1 = ROOT / "docs/p9_foundation_weak_discovery/h1"
CANONICAL = ROOT / "docs/p9_r1a_true_profile_closure/oracle_terminal_requests.csv"
MAP = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/map/frozen/floor01_h1_map_p5_frozen.pcd")
BUDGETS = np.array([1, 2, 4, 8, 16, 32, 64, 128, 192, 263])
METHODS = [("WEAK2", -1), ("STRONG2", -1)] + [("RANDOM2", r) for r in range(5)]
ENV = dict(os.environ, LD_LIBRARY_PATH="/lib/x86_64-linux-gnu",
           PYTHONDONTWRITEBYTECODE="1", OMP_NUM_THREADS="1")


class ExistingRunError(RuntimeError):
    """Refusing a repeat must not erase an earlier completed experiment."""


def assert_empty_run_output(directory):
    if (directory / "ndt_runs.csv").exists() or any((directory / "runs").glob("*.csv")):
        raise ExistingRunError("existing NDT output preserved; do not overwrite or repeat alignments")


def write_csv(path, rows):
    require(bool(rows), "empty sidecar: " + str(path))
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, list(rows[0]), lineterminator="\n")
        writer.writeheader(); writer.writerows(rows)


def text(values):
    return ";".join(format(float(v), ".17g") for v in np.asarray(values).reshape(-1))


def git_check():
    branch = subprocess.check_output(["git", "branch", "--show-current"], cwd=ROOT, text=True).strip()
    head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    require(branch == "research/p9-foundation-weak-discovery", "wrong foundation branch")
    subprocess.run(["git", "merge-base", "--is-ancestor", H1_SHA, "HEAD"], cwd=ROOT, check=True)
    message = subprocess.check_output(["git", "log", "-1", "--format=%s", H1_SHA], cwd=ROOT, text=True).strip()
    require(message == "research: validate P9 weak-subspace concentration at frame level", "H1 commit missing")
    return dict(branch=branch, start_sha=H1_SHA, execution_head=head)


def inputs():
    git_check()
    h1 = json.loads((H1 / "results.json").read_text())
    require(h1["final_result"] == "WEAK_SUBSPACE_CONCENTRATION_SUPPORTED" and
            h1["current_run_state"] == "COMPLETE", "H1 not complete/PASS")
    paths = [ARCHIVE / "candidates.csv", ARCHIVE / "frozen/cohort_frozen.csv",
             ARCHIVE / "dual_u.csv", ARCHIVE / "clusters/mode_clusters.csv",
             ARCHIVE / "frozen/objective_provenance.json", CANONICAL,
             H1 / "results.json", H1 / "major_projection_statistics.csv",
             H1 / "frame_projection_statistics.csv", MAP]
    expected = dict(h1["input_sha256"])
    expected[str(MAP)] = "2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570"
    expected[str(ARCHIVE / "frozen/objective_provenance.json")] = "7f2652bb568bb5958bfafcac3b4bfa09d8f6fab3ae381e26f1821a76eb64999e"
    for path in paths:
        actual = digest(path)
        if ROOT in path.parents:
            frozen = subprocess.check_output(["git", "show", H1_SHA + ":" + str(path.relative_to(ROOT))], cwd=ROOT)
            require(actual == hashlib.sha256(frozen).hexdigest(), "committed input changed: " + str(path))
        else:
            require(actual == expected[str(path)], "archive input hash changed: " + str(path))
    cohort = {int(r["transaction_id"]): r for r in read_csv(paths[1])}
    require(len(cohort) == 32, "oracle cohort not 32")
    candidates = read_csv(paths[0]); require(len(candidates) == 8800, "archive not 8800")
    selected = [r for r in candidates if int(r["transaction_id"]) in FRAMES]
    require(len(selected) == 2367 and len({(r["transaction_id"], r["seed_index"]) for r in selected}) == 2367,
            "candidate key/count mismatch")
    for tx in FRAMES:
        frame = cohort[tx]; require(frame["wide_targeted"] == "0", "targeted frame in H2")
        rows = [r for r in selected if int(r["transaction_id"]) == tx]
        require(sorted(int(r["seed_index"]) for r in rows) == list(range(263)), "seed indices incomplete")
        for row in rows:
            vector(row["start_pose_xyz_q_xyzw"], 7); vector(row["final_pose_matrix16"], 16)
            require(row["source_hash_actual"] == row["source_hash_expected"] == frame["prepared_source_hash"] and
                    row["source_points"] == frame["prepared_source_point_count"] and row["target_points"] == "549606" and
                    row["input_map_sha256"] == expected[str(MAP)] and row["converged"] in ("0", "1") and
                    0 <= int(row["iterations"]) <= 80 and np.isfinite(float(row["raw_ndt_score_sum"])) and
                    np.isfinite(float(row["runtime_ms"])), "invalid archived result")
        require(float(frame["configured_resolution_m"]) == .8 and float(frame["step_size"]) == .08 and
                float(frame["transformation_epsilon"]) == 1e-5 and int(frame["maximum_iterations"]) == 80,
                "frozen NDT settings differ")
        raw = Path(frame["raw_cloud_file"])
        require(digest(raw) == frame["raw_source_sha256"] and raw.stat().st_size == int(frame["raw_point_count"]) * 12,
                "raw source hash/size mismatch")
        paths.append(raw)
    canonical = read_csv(CANONICAL)
    ids = {(int(r["tx"]), r["cluster_id"]) for r in read_csv(H1 / "major_projection_statistics.csv")}
    require(len(canonical) == 22 and {(int(r["transaction_id"]), r["cluster_id"]) for r in canonical} == ids,
            "frozen major identity changed")
    by_seed = {(int(r["transaction_id"]), int(r["seed_index"])): r for r in selected}
    for r in canonical:
        candidate = by_seed[int(r["transaction_id"]), int(r["canonical_seed_index"])]
        require(candidate["final_pose_matrix16"] == r["canonical_pose_matrix16"], "canonical not archived real terminal")
    return {str(p): digest(p) for p in paths}, selected


def near_duplicate_count(rows):
    matrices = np.array([vector(r["start_pose_matrix16"], 16).reshape(4, 4) for r in rows])
    rot = Rotation.from_matrix(matrices[:, :3, :3])
    count = 0; participating = set()
    for i in range(len(rows) - 1):
        distance = np.linalg.norm(matrices[i + 1:, :3, 3] - matrices[i, :3, 3], axis=1)
        angle = (rot[i].inv() * rot[i + 1:]).magnitude() * 180 / np.pi
        hits = np.flatnonzero((distance <= 1e-5) & (angle <= 1e-4)) + i + 1
        count += len(hits)
        for j in hits: participating.update((i, int(j)))
    return dict(pair_count=count, participating_proposals=len(participating))


def prepare(binary):
    assert_empty_run_output(OUT)
    hashes, _ = inputs(); OUT.mkdir(parents=True, exist_ok=True)
    random_rows = []
    for fi, frame in enumerate(FRAMES):
        for rep in range(5):
            seed = 20261010 + 1000 * fi + rep
            q = haar_subspaces(np.random.Generator(np.random.PCG64(seed)), 1, 2)[0]
            random_rows.append(dict(frame=frame, random_rep=rep, rng="PCG64", seed=seed, basis_rowmajor=text(q)))
    write_csv(OUT / "random_subspaces.csv", random_rows)
    rng = np.random.Generator(np.random.PCG64(20261009))
    perms = [dict(permutation=r, rng="PCG64", seed=20261009,
                  seed_indices=";".join(map(str, rng.permutation(263)))) for r in range(100)]
    write_csv(OUT / "permutation_manifest.csv", perms)
    command = [str(binary), "--proposals", str(ARCHIVE / "dual_u.csv"), str(ARCHIVE / "candidates.csv"),
               str(OUT / "random_subspaces.csv"), str(OUT)]
    subprocess.run(command, check=True, env=ENV)
    parity = read_csv(OUT / "projected_seed_parity.csv")
    require(len(parity) == 2367 and all(r["pass"] == "1" for r in parity), "proposal round-trip parity failed")
    proposals = read_csv(OUT / "proposal_contract.csv")
    require(len(proposals) == 18936, "proposal pool incomplete")
    duplicates = []
    for tx in FRAMES:
        for method, rep in [("FULL6D", -1)] + METHODS:
            rows = [r for r in proposals if int(r["frame"]) == tx and r["method"] == method and int(r["random_rep"]) == rep]
            require(len(rows) == 263, "projection pool mismatch")
            duplicates.append(dict(frame=tx, method=method, random_rep=rep, **near_duplicate_count(rows)))
    write_csv(OUT / "projected_duplicates.csv", duplicates)
    code = [Path(__file__).resolve(), Path(__file__).with_name("p9_h2_matched_discovery.cpp"),
            Path(__file__).with_name("p9_ndt_energy_contract.cpp"), Path(__file__).with_name("CMakeLists.txt")]
    code += [Path(__file__).with_name("run_h1_concentration.py"),
             Path(__file__).with_name("summarize_h2_discovery.py"), OUT / "THEORY.md"]
    headers = [Path("/usr/include/pcl-1.10") / name for name in (
        "pcl/registration/ndt.h", "pcl/registration/impl/ndt.hpp",
        "pcl/registration/impl/registration.hpp", "pcl/filters/impl/voxel_grid.hpp",
        "pcl/filters/impl/voxel_grid_covariance.hpp")]
    code += headers
    hashes.update({str(p.resolve()): digest(p) for p in code})
    hashes.update({str(OUT / name): digest(OUT / name) for name in (
        "random_subspaces.csv", "permutation_manifest.csv", "proposal_contract.csv", "projected_seed_parity.csv", "projected_duplicates.csv")})
    cache = binary.parent / "CMakeCache.txt"
    require("CMAKE_BUILD_TYPE:STRING=Release" in cache.read_text(), "runner is not a Release build")
    manifest = dict(git=git_check(), input_sha256=hashes, binary=str(binary), binary_sha256=digest(binary),
                    build_type="Release", cmake_cache_sha256=digest(cache),
                    compiler=subprocess.check_output(["g++", "--version"], text=True).splitlines()[0],
                    ndt_linkage=subprocess.check_output(["ldd", str(binary)], text=True, env=ENV),
                    ndt=dict(pcl="1.10", resolution=.8, step=.08, epsilon=1e-5, max_iter=80, outlier_ratio=.55),
                    permutations=100, permutation_seed=20261009, random_replicates=5,
                    roundtrip=dict(pass_=True, max_translation_m=max(float(r["translation_error_m"]) for r in parity),
                                   max_rotation_deg=max(float(r["rotation_error_deg"]) for r in parity)),
                    gate="PASS", gt_used=False, new_ndt_calls_planned=16569, full6d_calls_reused=2367)
    (OUT / "execution_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({"PROPOSAL_CONTRACT": manifest["roundtrip"], "new_ndt_calls_planned": 16569}, indent=2))


def verify_manifest():
    manifest = json.loads((OUT / "execution_manifest.json").read_text())
    require(manifest["gate"] == "PASS", "preflight gate missing")
    for name, expected in manifest["input_sha256"].items():
        require(digest(Path(name)) == expected, "input/code changed since preflight: " + name)
    require(digest(Path(manifest["binary"])) == manifest["binary_sha256"], "runner binary changed")
    return manifest


def run():
    assert_empty_run_output(OUT)
    manifest = verify_manifest(); inputs(); (OUT / "runs").mkdir(exist_ok=True)
    (OUT / "results.json").write_text(json.dumps(dict(current_run_state="RUNNING", h2="NOT_EVALUATED")) + "\n")
    for method, rep in METHODS:
        target = OUT / "runs" / f"{method}_{rep}.csv"
        command = [manifest["binary"], "--align", str(MAP), str(ARCHIVE / "frozen/cohort_frozen.csv"),
                   str(OUT / "proposal_contract.csv"), method, str(rep), str(target)]
        subprocess.run(command, check=True, env=ENV)
        require(len(read_csv(target)) == 2367, "incomplete new result pool")
    verify_manifest()
    print("H2_NEW_RUNS_COMPLETE=16569", flush=True)


def failure(error):
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "results.json").write_text(json.dumps(dict(current_run_state="FAILED", h2="NOT_EVALUATED",
        final_result="H2_MATCHED_PROPOSAL_CONTRACT_FAIL", error=str(error)), indent=2) + "\n")
    (OUT / "REPORT.md").write_text("# H2 contract failed\n\n" + str(error) + "\n\nNo algorithm claim is supported by this run.\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("stage", choices=("prepare", "run", "analyze", "audit", "self-test"))
    parser.add_argument("--binary", type=Path, default=Path("/tmp/p9_h2_build/p9_h2_matched_discovery"))
    args = parser.parse_args()
    try:
        if args.stage == "prepare": prepare(args.binary.resolve())
        elif args.stage == "run": run()
        else:
            from summarize_h2_discovery import analyze, audit, self_test
            {"analyze": analyze, "audit": audit, "self-test": self_test}[args.stage]()
    except ExistingRunError as error:
        print("PREEXISTING_RUN_PRESERVED: " + str(error), flush=True)
        raise
    except Exception as error:
        if args.stage in ("prepare", "run", "analyze"): failure(error)
        raise


if __name__ == "__main__": main()
