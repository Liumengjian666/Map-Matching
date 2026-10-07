#!/usr/bin/env python3
"""P9-R2 low-budget terminal-stability experiment; probes are frozen before NDT."""
import argparse
import csv
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
SCRIPT_DIR = Path(__file__).resolve().parent
ARCHIVE = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/"
               "dual_u_r1_closure_20261003/same_objective")
MAP = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/map/frozen/"
           "floor01_h1_map_p5_frozen.pcd")
OUT = ROOT / "docs/p9_r2_terminal_stability"
H1 = ROOT / "docs/p9_foundation_weak_discovery/h1"
R1C3B = ROOT / "docs/p9_r1c3b_discrete_support_evidence"
START_SHA = "ac85959deaff085960148ecdb96f5780d7872566"
FRAME_ORDER = (120, 244, 368, 616, 740, 838, 839, 864, 924, 925, 1111, 1235,
               1359, 1497, 1498, 1556, 1557, 1606, 1730, 1854, 2102, 2226,
               2350, 2598, 2722, 2846, 3094, 3217, 3341, 3631, 3796, 3962)
BUDGETS = (4, 8, 12, 16)
METHODS = (("WEAK2", -1), ("STRONG2", -1), ("RANDOM2", 0),
           ("RANDOM2", 1), ("RANDOM2", 2))
RANDOM_BASE_SEED = 20261007
PERMUTATION_SEED = 20261008
PERMUTATION_REPLICATES = 10000
ENV = dict(os.environ, LD_LIBRARY_PATH="/lib/x86_64-linux-gnu", OMP_NUM_THREADS="1",
           PYTHONDONTWRITEBYTECODE="1")
BIN_DEFAULT = Path("/tmp/p9_r2_build/p9_r2_terminal_stability")


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def read_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows):
    require(bool(rows), "refusing to write empty CSV: " + str(path))
    path = Path(path)
    fields = list(rows[0])
    for row in rows[1:]:
        fields.extend(key for key in row if key not in fields)
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def text(values):
    return ";".join(format(float(value), ".17g") for value in np.asarray(values).reshape(-1))


def vector(value, count):
    result = np.fromstring(value, sep=";")
    require(result.size == count and np.isfinite(result).all(), "invalid numeric vector")
    return result


def git_state():
    branch = subprocess.check_output(["git", "branch", "--show-current"], cwd=ROOT, text=True).strip()
    head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    require(branch == "research/p9-r2-terminal-stability", "R2 branch mismatch")
    subprocess.run(["git", "merge-base", "--is-ancestor", START_SHA, "HEAD"], cwd=ROOT, check=True)
    require(subprocess.check_output(["git", "log", "-1", "--format=%s", START_SHA],
                                    cwd=ROOT, text=True).strip() ==
            "research: add P9 discrete support evidence", "R1C3B base commit mismatch")
    return {"branch": branch, "start_sha": START_SHA, "execution_head": head}


def archive_paths():
    cohort = ARCHIVE / "frozen/cohort_frozen.csv"
    return {
        "candidates": ARCHIVE / "candidates.csv",
        "cohort": cohort,
        "uobs": ARCHIVE / "dual_u.csv",
        "objective_provenance": ARCHIVE / "frozen/objective_provenance.json",
        "map": MAP,
    }


def preflight_inputs():
    git_state()
    expected_map = "2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570"
    expected_hashes = json.loads((R1C3B / "execution_manifest.json").read_text())["input_sha256"]
    paths = archive_paths()
    for path in paths.values():
        require(path.is_file(), "missing frozen input: " + str(path))
    actual_hashes = {str(path): digest(path) for path in paths.values()}
    require(actual_hashes[str(MAP)] == expected_map, "official frozen map hash mismatch")
    for path in paths.values():
        if str(path) in expected_hashes:
            require(actual_hashes[str(path)] == expected_hashes[str(path)],
                    "frozen archive hash differs from R1C3B: " + str(path))

    cohort_rows = read_csv(paths["cohort"])
    candidate_rows = read_csv(paths["candidates"])
    uobs_rows = read_csv(paths["uobs"])
    require(len(cohort_rows) == 32 and len(candidate_rows) == 8800,
            "frozen 32-frame/8800-candidate archive count mismatch")
    require(len({int(row["transaction_id"]) for row in cohort_rows}) == 32,
            "duplicate cohort frame")
    cohort = {int(row["transaction_id"]): row for row in cohort_rows}
    require(set(cohort) == set(FRAME_ORDER), "frozen cohort IDs/order mismatch")
    uobs = {int(row["transaction_id"]): row for row in uobs_rows}
    require(all(tx in uobs and uobs[tx]["uobs_valid"] == "1" and
                uobs[tx]["uobs_status"] == "PASS_LOCAL_NDT_CURVATURE" for tx in FRAME_ORDER),
            "a cohort frame has no valid frozen U_obs")
    candidate_by_frame = {tx: [] for tx in FRAME_ORDER}
    for row in candidate_rows:
        tx = int(row["transaction_id"])
        if tx in candidate_by_frame:
            candidate_by_frame[tx].append(row)
    raw_hashes = {}
    for tx in FRAME_ORDER:
        frame = cohort[tx]
        rows = candidate_by_frame[tx]
        base = [row for row in rows if 0 <= int(row["seed_index"]) < 263]
        require(len(base) == 263 and sorted(int(row["seed_index"]) for row in base) == list(range(263)),
                f"frame {tx} does not have exactly archived base seeds 0..262")
        for row in base:
            require(row["source_hash_actual"] == row["source_hash_expected"] ==
                    frame["prepared_source_hash"], f"candidate/source hash mismatch for {tx}")
            require(row["source_points"] == frame["prepared_source_point_count"] and
                    row["target_points"] == "549606", f"candidate point-count mismatch for {tx}")
            require(row["input_map_sha256"] == expected_map, f"candidate map hash mismatch for {tx}")
        require(float(frame["configured_resolution_m"]) == 0.8 and
                float(frame["step_size"]) == 0.08 and
                float(frame["transformation_epsilon"]) == 1e-5 and
                int(frame["maximum_iterations"]) == 80,
                f"frozen NDT contract mismatch for frame {tx}")
        raw = Path(frame["raw_cloud_file"])
        require(raw.is_file() and raw.stat().st_size == int(frame["raw_point_count"]) * 12,
                f"raw cloud missing or byte count mismatch for {tx}")
        raw_hash = digest(raw)
        require(raw_hash == frame["raw_source_sha256"], f"raw cloud SHA mismatch for {tx}")
        if str(raw) in expected_hashes:
            require(raw_hash == expected_hashes[str(raw)], f"raw cloud differs from R1C3B hash for {tx}")
        raw_hashes[str(raw)] = raw_hash
    hashes = {**actual_hashes, **raw_hashes}
    return cohort, hashes


def haar_basis(seed):
    rng = np.random.Generator(np.random.PCG64(seed))
    matrix = rng.standard_normal((6, 2))
    q, r = np.linalg.qr(matrix, mode="reduced")
    signs = np.where(np.diag(r) < 0, -1.0, 1.0)
    q = q * signs[None, :]
    require(np.linalg.norm(q.T @ q - np.eye(2)) <= 1e-12, "Haar basis is not orthonormal")
    return q


def choose_farthest(rows, budget=16):
    """Deterministic farthest-point order in the saved 2-D projected coordinates."""
    by_seed = {int(row["seed_index"]): row for row in rows}
    require(sorted(by_seed) == list(range(263)), "farthest selection requires 263 unique seed IDs")
    points = {seed: np.array([float(row["coord0"]), float(row["coord1"])])
              for seed, row in by_seed.items()}
    require(all(np.isfinite(point).all() for point in points.values()), "non-finite projected coordinate")
    first = min(points, key=lambda seed: (float(points[seed] @ points[seed]), seed))
    order = [first]
    selection_distance = {first: math.sqrt(float(points[first] @ points[first]))}
    while len(order) < budget:
        distances = {}
        for seed, point in points.items():
            if seed in selection_distance:
                continue
            distances[seed] = min(float(np.sum((point - points[chosen]) ** 2)) for chosen in order)
        largest = max(distances.values())
        tolerance = 1e-12 * max(1.0, abs(largest))
        chosen = min(seed for seed, value in distances.items() if value >= largest - tolerance)
        order.append(chosen)
        selection_distance[chosen] = math.sqrt(max(0.0, distances[chosen]))
    return [(rank, seed, selection_distance[seed], by_seed[seed])
            for rank, seed in enumerate(order, start=1)]


def make_random_manifest():
    rows = []
    for frame_index, frame in enumerate(FRAME_ORDER):
        for replicate in range(3):
            seed = RANDOM_BASE_SEED + 1000 * frame_index + replicate
            rows.append({"frame": frame, "random_rep": replicate, "rng": "PCG64",
                         "seed": seed, "basis_rowmajor": text(haar_basis(seed).reshape(-1))})
    return rows


def prepare(binary):
    git = git_state()
    cohort, hashes = preflight_inputs()
    prior_hashes = json.loads((R1C3B / "execution_manifest.json").read_text())["input_sha256"]
    for path in (H1 / "frame_projection_statistics.csv", H1 / "results.json"):
        require(str(path) in prior_hashes and digest(path) == prior_hashes[str(path)],
                "H1 frozen evaluation input differs from R1C3B provenance: " + str(path))
    OUT.mkdir(parents=True, exist_ok=True)
    for name in ("proposal_pool.csv", "proposal_roundtrip.csv",
                 "probe_manifest.csv", "execution_manifest.json", "ndt_probe_results.csv"):
        require(not (OUT / name).exists(), "R2 artifact already exists; refusing overwrite: " + name)
    require(binary.is_file(), "Release binary missing: " + str(binary))
    (OUT / "runs").mkdir(exist_ok=True)
    random_rows = make_random_manifest()
    random_path = OUT / "random_subspaces.csv"
    if random_path.exists():
        require(read_csv(random_path) == [{k: str(v) for k, v in row.items()} for row in random_rows],
                "existing random subspace manifest differs from frozen deterministic schedule")
    else:
        write_csv(random_path, random_rows)
    projection_started = time.perf_counter()
    command = [str(binary), "--prepare", str(ARCHIVE / "frozen/cohort_frozen.csv"),
               str(ARCHIVE / "dual_u.csv"), str(ARCHIVE / "candidates.csv"),
               str(OUT / "random_subspaces.csv"), str(OUT)]
    env = dict(ENV, PYTHONDONTWRITEBYTECODE="1")
    try:
        prepared = subprocess.run(command, cwd=ROOT, env=env, text=True, capture_output=True, check=True)
    except subprocess.CalledProcessError as error:
        (OUT / "proposal_engine_stdout.log").write_text((error.stdout or "") + (error.stderr or ""))
        raise RuntimeError("proposal preparation failed; see proposal_engine_stdout.log") from error
    (OUT / "proposal_engine_stdout.log").write_text(prepared.stdout + prepared.stderr)
    require("ROUNDTRIP_PASS=YES" in prepared.stdout, "proposal/chart parity gate failed")
    require(digest(OUT / "random_subspaces.csv") and
            len(read_csv(OUT / "proposal_roundtrip.csv")) == 32 * 263,
            "random/proposal preparation artifacts incomplete")
    pool = read_csv(OUT / "proposal_pool.csv")
    require(len(pool) == 32 * 263 * 5, "projected proposal pool is incomplete")
    pool_groups = {}
    for row in pool:
        key = (int(row["frame"]), row["method"], int(row["random_rep"]))
        pool_groups.setdefault(key, []).append(row)
    probe_rows = []
    for frame in FRAME_ORDER:
        for method, replicate in METHODS:
            selected = choose_farthest(pool_groups[(frame, method, replicate)])
            for rank, seed, selection_distance, row in selected:
                probe_rows.append({
                    "frame": frame, "frame_id": row["frame_id"], "method": method,
                    "random_rep": replicate, "probe_rank": rank, "seed_index": seed,
                    "coord0": row["coord0"], "coord1": row["coord1"],
                    "selection_distance": format(selection_distance, ".17g"),
                    "eta_projected": row["eta_projected"],
                    "nominal_pose_matrix16": row["nominal_pose_matrix16"],
                    "start_pose_matrix16": row["start_pose_matrix16"]})
    require(len(probe_rows) == 32 * 5 * 16, "selected probe count mismatch")
    write_csv(OUT / "probe_manifest.csv", probe_rows)
    projection_seconds = time.perf_counter() - projection_started

    source_files = [SCRIPT_DIR / name for name in (
        "p9_r2_terminal_stability.cpp", "p9_ndt_energy_contract.cpp",
        "run_r2_terminal_stability.py", "CMakeLists.txt")]
    headers = [Path("/usr/include/pcl-1.10") / name for name in (
        "pcl/registration/ndt.h", "pcl/registration/impl/ndt.hpp",
        "pcl/registration/impl/registration.hpp", "pcl/filters/impl/voxel_grid.hpp",
        "pcl/filters/impl/voxel_grid_covariance.hpp")]
    for path in source_files + headers:
        require(path.is_file(), "missing code/PCL provenance input: " + str(path))
        hashes[str(path.resolve())] = digest(path)
    for path in (R1C3B / "execution_manifest.json", H1 / "frame_projection_statistics.csv",
                 H1 / "results.json", OUT / "random_subspaces.csv", OUT / "proposal_pool.csv",
                 OUT / "proposal_roundtrip.csv", OUT / "probe_manifest.csv"):
        hashes[str(path)] = digest(path)
    parity = read_csv(OUT / "proposal_roundtrip.csv")
    max_t = max(float(row["translation_error_m"]) for row in parity)
    max_r = max(float(row["rotation_error_deg"]) for row in parity)
    require(all(row["pass"] == "1" for row in parity), "a base seed failed chart round-trip parity")
    cache = binary.parent / "CMakeCache.txt"
    require(cache.is_file() and "CMAKE_BUILD_TYPE:STRING=Release" in cache.read_text(),
            "runner must be built in Release mode")
    hashes[str(cache)] = digest(cache)
    build_details = {
        "binary": str(binary), "binary_sha256": digest(binary),
        "cmake_cache_sha256": digest(cache), "build_type": "Release",
        "compiler": subprocess.check_output(["g++", "--version"], text=True).splitlines()[0],
        "ndt_linkage": subprocess.check_output(["ldd", str(binary)], text=True, env=ENV),
        "numpy_version": np.__version__,
        "pcl_headers": "PCL 1.10 (/usr/include/pcl-1.10)",
    }
    settings = {"pcl": "1.10", "resolution_m": 0.8, "step_size": 0.08,
                "transformation_epsilon": 1e-5, "maximum_iterations": 80,
                "outlier_ratio": 0.55, "source_preprocessing": "frozen objective provenance",
                "target_preprocessing": "finite filter + two 0.15m voxel grids"}
    manifest = {
        "task": "PAPER-P9-R2-LOW-BUDGET-TERMINAL-STABILITY-EVIDENCE",
        "current_run_state": "PREPARED_NOT_RUN", "git": git,
        "frames": list(FRAME_ORDER), "frame_count": 32,
        "probe_contract": {"base_seed_indices": "0..262 inclusive per frame",
            "farthest_point": "start nearest 2D projected coordinate to origin; then max minimum squared Euclidean distance; numerical tie within 1e-12*max(1,|max d2|) resolved to lowest seed_index",
            "budgets": list(BUDGETS), "per_2d_pool_max_probes": 16,
            "methods": ["WEAK2", "STRONG2", "RANDOM2 rep 0,1,2"],
            "projected_proposals_not_renormalized": True,
            "random_rng": "NumPy Generator(PCG64); basis via QR of standard-normal 6x2 with R diagonal sign canonicalization",
            "random_base_seed": RANDOM_BASE_SEED,
            "random_seed_formula": "20261007 + 1000*frame_index_in_frozen_cohort_order + replicate(0..2)",
            "projected_pool_rows": len(pool), "selected_probe_rows": len(probe_rows),
            "planned_new_ndt_calls": len(probe_rows),
            "selection_elapsed_s": projection_seconds},
        "roundtrip_parity": {"tested": len(parity), "pass": True,
            "translation_tolerance_m": 1e-5, "rotation_tolerance_deg": 1e-4,
            "max_translation_error_m": max_t, "max_rotation_error_deg": max_r},
        "ndt": settings, "build": build_details,
        "input_sha256": hashes, "gt_used": False, "labels_used_for_probe_selection": False,
        "support_features_used": False, "full_6d_alignment_calls": 0,
        "novelty_warning": "terminal second moment is sensitivity evidence, not posterior covariance"}
    (OUT / "execution_manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"PROBE_MANIFEST": str(OUT / "probe_manifest.csv"),
                      "PROBES": len(probe_rows), "ROUNDTRIP": manifest["roundtrip_parity"],
                      "PLANNED_NEW_NDT_CALLS": len(probe_rows),
                      "SELECTION_SECONDS": projection_seconds}, indent=2))


def verify_manifest(binary=None):
    manifest = json.loads((OUT / "execution_manifest.json").read_text())
    require(manifest["current_run_state"] in ("PREPARED_NOT_RUN", "RUNNING", "RUN_COMPLETE"),
            "invalid R2 execution state")
    require(manifest["git"]["branch"] == "research/p9-r2-terminal-stability" and
            manifest["git"]["start_sha"] == START_SHA, "R2 execution manifest Git identity mismatch")
    for name, expected in manifest["input_sha256"].items():
        path = Path(name)
        require(path.is_file() and digest(path) == expected, "frozen input/code changed: " + name)
    binary_path = Path(manifest["build"]["binary"] if binary is None else binary)
    require(binary_path.is_file() and digest(binary_path) == manifest["build"]["binary_sha256"],
            "R2 Release binary changed")
    return manifest


def recover_prepared_manifest(binary):
    """Rebind only the orchestration-source digest after a pre-align path error."""
    manifest_path = OUT / "execution_manifest.json"
    manifest = json.loads(manifest_path.read_text())
    require(manifest["current_run_state"] == "RUNNING", "recovery requires a failed pre-run state")
    require(not (OUT / "ndt_probe_results.csv").exists() and
            all(not path.exists() for path in shard_paths()),
            "recovery refused: NDT result shard exists; do not rebind a started experiment")
    log_path = OUT / "engine_stdout.log"
    require(log_path.is_file() and "cannot create R2 result shard" in log_path.read_text(),
            "recovery requires the known output-directory failure signature")
    require("runs/runs/tx_120_RANDOM2_r0.csv.partial" in log_path.read_text(),
            "failure log does not prove the pre-align path error")
    require(digest(binary) == manifest["build"]["binary_sha256"],
            "recovery refused: executable changed")
    cohort, current_archive_hashes = preflight_inputs()
    del cohort
    for name, expected in manifest["input_sha256"].items():
        path = Path(name)
        if path.resolve() == (SCRIPT_DIR / "run_r2_terminal_stability.py").resolve():
            continue
        require(path.is_file() and digest(path) == expected,
                "recovery refused: a frozen non-orchestrator input changed: " + name)
    for name, expected in current_archive_hashes.items():
        require(manifest["input_sha256"].get(name) == expected,
                "recovery refused: archive source hash differs: " + name)

    expected_random = [{k: str(v) for k, v in row.items()} for row in make_random_manifest()]
    require(read_csv(OUT / "random_subspaces.csv") == expected_random,
            "recovery refused: frozen random bases differ from deterministic schedule")
    pool = read_csv(OUT / "proposal_pool.csv")
    require(len(pool) == 32 * 263 * 5, "recovery refused: proposal pool incomplete")
    groups = {}
    for row in pool:
        groups.setdefault((int(row["frame"]), row["method"], int(row["random_rep"])), []).append(row)
    frozen_probes = read_csv(OUT / "probe_manifest.csv")
    probe_groups = {}
    for row in frozen_probes:
        probe_groups.setdefault((int(row["frame"]), row["method"], int(row["random_rep"])), []).append(row)
    require(len(frozen_probes) == 32 * 5 * 16 and len(probe_groups) == 32 * 5,
            "recovery refused: frozen selected probe manifest incomplete")
    for key, candidates in groups.items():
        selected = choose_farthest(candidates)
        saved = sorted(probe_groups[key], key=lambda row: int(row["probe_rank"]))
        require(len(saved) == 16, "recovery refused: selected pool row count mismatch")
        for expected, actual in zip(selected, saved):
            rank, seed, distance, source = expected
            require(int(actual["probe_rank"]) == rank and int(actual["seed_index"]) == seed and
                    abs(float(actual["selection_distance"]) - distance) <= 1e-12 and
                    actual["start_pose_matrix16"] == source["start_pose_matrix16"] and
                    actual["eta_projected"] == source["eta_projected"],
                    "recovery refused: probe selection differs from frozen farthest-point contract")

    script_path = str((SCRIPT_DIR / "run_r2_terminal_stability.py").resolve())
    require(script_path in manifest["input_sha256"], "orchestration source digest is absent")
    manifest["input_sha256"][script_path] = digest(script_path)
    manifest["current_run_state"] = "PREPARED_NOT_RUN"
    manifest["pre_run_failure_recovery"] = {
        "cause": "runner passed OUT/runs although C++ appends /runs; failed before opening a result shard",
        "ndt_align_calls": 0,
        "frozen_random_pool_and_probe_manifest_revalidated": True,
        "only_rebound_input_digest": script_path,
    }
    manifest["git"]["execution_head"] = git_state()["execution_head"]
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print("R2_PREPARED_MANIFEST_RECOVERY=PASS NO_NDT_CALLS=0 PROBE_ORDER_REVALIDATED=YES")


def rebind_analysis_source():
    """Record the final report-only code revision without changing frozen NDT outputs."""
    manifest_path = OUT / "execution_manifest.json"
    manifest = json.loads(manifest_path.read_text())
    require(manifest["current_run_state"] == "RUN_COMPLETE", "analysis rebind requires a complete NDT run")
    require((OUT / "ndt_probe_results.csv").is_file() and
            digest(OUT / "ndt_probe_results.csv") == manifest["ndt_probe_result_sha256"],
            "analysis rebind refused: NDT output missing or changed")
    require(len(read_csv(OUT / "ndt_probe_results.csv")) == 2560 and
            all(path.is_file() for path in shard_paths()),
            "analysis rebind refused: NDT result pool/shards incomplete")
    script_path = str((SCRIPT_DIR / "run_r2_terminal_stability.py").resolve())
    require(script_path in manifest["input_sha256"], "analysis source digest is absent")
    for name, expected in manifest["input_sha256"].items():
        if Path(name).resolve() == Path(script_path):
            continue
        path = Path(name)
        require(path.is_file() and digest(path) == expected,
                "analysis rebind refused: frozen non-analysis input changed: " + name)
    manifest["input_sha256"][script_path] = digest(script_path)
    manifest["analysis_source_rebind"] = {
        "reason": "add key-frame trace/escape summaries and report audit after frozen NDT completion",
        "ndt_outputs_changed": False,
        "ndt_align_calls": 0,
        "rebound_source": script_path,
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print("R2_ANALYSIS_SOURCE_REBIND=PASS NDT_OUTPUTS_UNCHANGED=YES NDT_CALLS=0")


def shard_paths():
    result = []
    for frame in FRAME_ORDER:
        for method, rep in METHODS:
            result.append(OUT / "runs" / f"tx_{frame}_{method}_r{rep}.csv")
    return result


def consolidate_shards():
    target = OUT / "ndt_probe_results.csv"
    require(not target.exists(), "consolidated NDT results already exist; refusing overwrite")
    shards = shard_paths()
    require(all(path.is_file() for path in shards), "one or more atomic NDT shards are missing")
    rows = []
    for path in shards:
        current = read_csv(path)
        require(len(current) == 16, "NDT shard row count mismatch: " + str(path))
        rows.extend(current)
    require(len(rows) == 2560, "R2 NDT total call/result count mismatch")
    write_csv(target, rows)
    return target


def run(binary):
    manifest = verify_manifest(binary)
    manifest["current_run_state"] = "RUNNING"
    (OUT / "execution_manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    OUT.joinpath("runs").mkdir(exist_ok=True)
    log = OUT / "engine_stdout.log"
    start = time.perf_counter()
    command = [str(binary), "--align-all", str(MAP), str(ARCHIVE / "frozen/cohort_frozen.csv"),
               str(ARCHIVE / "dual_u.csv"), str(OUT / "probe_manifest.csv"), str(OUT)]
    # Completed per-frame/method shards are skipped by the C++ runner; only missing frozen jobs execute.
    with log.open("a") as output:
        subprocess.run(command, cwd=ROOT, env=ENV, stdout=output, stderr=subprocess.STDOUT, check=True)
    elapsed = time.perf_counter() - start
    results = consolidate_shards() if not (OUT / "ndt_probe_results.csv").exists() else OUT / "ndt_probe_results.csv"
    actual = read_csv(results)
    require(len(actual) == 2560, "completed R2 result count mismatch")
    keys = {(int(row["frame"]), row["method"], int(row["random_rep"]), int(row["probe_rank"]))
            for row in actual}
    require(len(keys) == 2560, "duplicate NDT probe result identity")
    manifest["current_run_state"] = "RUN_COMPLETE"
    manifest["ndt_run_elapsed_wall_s"] = elapsed
    manifest["ndt_probe_result_sha256"] = digest(results)
    manifest["completed_shards"] = len(shard_paths())
    manifest["completed_ndt_calls"] = len(actual)
    (OUT / "execution_manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(f"R2_NDT_CALLS_COMPLETE={len(actual)} WALL_SECONDS={elapsed:.3f}")


def roc_auc(labels, scores):
    labels = np.asarray(labels, dtype=bool)
    scores = np.asarray(scores, dtype=float)
    require(labels.size == scores.size and labels.any() and (~labels).any(), "AUC label/score mismatch")
    order = np.argsort(scores, kind="mergesort")
    ranks = np.empty(len(scores), dtype=float)
    i = 0
    while i < len(order):
        j = i + 1
        while j < len(order) and scores[order[j]] == scores[order[i]]:
            j += 1
        ranks[order[i:j]] = (i + 1 + j) / 2.0
        i = j
    positive_count = int(labels.sum())
    negative_count = len(labels) - positive_count
    return float((ranks[labels].sum() - positive_count * (positive_count + 1) / 2.0) /
                 (positive_count * negative_count))


def best_training_threshold(labels, scores):
    labels = np.asarray(labels, dtype=bool)
    scores = np.asarray(scores, dtype=float)
    unique = np.unique(scores)
    if len(unique) > 1:
        mids = (unique[:-1] + unique[1:]) / 2.0
    else:
        mids = np.array([], dtype=float)
    candidates = np.r_[-np.inf, mids, np.inf]
    best = None
    for threshold in candidates:
        pred = scores >= threshold
        tpr = float(pred[labels].mean())
        tnr = float((~pred[~labels]).mean())
        balanced = 0.5 * (tpr + tnr)
        # Deterministic tie break: choose the highest threshold (most conservative positive calls).
        key = (balanced, threshold)
        if best is None or key > best[0]:
            best = (key, float(threshold), tpr, tnr, balanced)
    return best[1:]


def make_permutation_indices(frame_count=32, positives=9):
    rng = np.random.Generator(np.random.PCG64(PERMUTATION_SEED))
    return np.array([rng.choice(frame_count, size=positives, replace=False)
                     for _ in range(PERMUTATION_REPLICATES)], dtype=np.int16)


def permutation_auc_p(labels, scores, permutations):
    labels = np.asarray(labels, dtype=bool)
    scores = np.asarray(scores, dtype=float)
    require(len(labels) == 32 and int(labels.sum()) == 9, "R2 frame labels must be 9/23")
    auc = roc_auc(labels, scores)
    # Rank-sum permits all frozen label permutations to share a single score ranking.
    order = np.argsort(scores, kind="mergesort")
    ranks = np.empty(len(scores), dtype=float)
    i = 0
    while i < len(order):
        j = i + 1
        while j < len(order) and scores[order[j]] == scores[order[i]]:
            j += 1
        ranks[order[i:j]] = (i + 1 + j) / 2.0
        i = j
    npos, nneg = 9, 23
    null_aucs = (ranks[permutations].sum(axis=1) - npos * (npos + 1) / 2.0) / (npos * nneg)
    p = (1 + int(np.sum(null_aucs >= auc - 1e-12))) / (len(null_aucs) + 1)
    return auc, p


def lofo(labels, scores, frame_ids):
    labels = np.asarray(labels, dtype=bool)
    scores = np.asarray(scores, dtype=float)
    rows = []
    correct = []
    training_aucs = []
    for index in range(len(labels)):
        keep = np.arange(len(labels)) != index
        threshold, tpr, tnr, balanced = best_training_threshold(labels[keep], scores[keep])
        prediction = bool(scores[index] >= threshold)
        is_correct = prediction == bool(labels[index])
        correct.append(is_correct)
        training_aucs.append(roc_auc(labels[keep], scores[keep]))
        rows.append({"frame": frame_ids[index], "label_major": int(labels[index]),
                     "score": format(float(scores[index]), ".17g"),
                     "train_only_threshold": format(threshold, ".17g"),
                     "training_balanced_accuracy": format(balanced, ".17g"),
                     "training_tpr": format(tpr, ".17g"), "training_tnr": format(tnr, ".17g"),
                     "training_auc": format(training_aucs[-1], ".17g"),
                     "predicted_major": int(prediction), "correct": int(is_correct)})
    return rows, float(np.mean(correct)), (float(min(training_aucs)), float(max(training_aucs)))


def frozen_labels_after_run():
    rows = read_csv(H1 / "frame_projection_statistics.csv")
    major = {int(row["frame"]) for row in rows if int(row["major_count"]) > 0}
    expected = {368, 616, 2226, 2350, 2722, 2846, 3341, 3796, 3962}
    require(major == expected and len(rows) == 9, "frozen H1 major-frame identity changed")
    # The frozen cohort, not probe results, supplies the denominator; all other 23 are no-major frames.
    return major


def key_frame_summaries(tensor_rows, major_frames):
    healthy = [frame for frame in FRAME_ORDER if frame not in major_frames]
    frames = [368, 2226, 2350, 2722, 3796] + healthy[:5]
    index = {(int(row["frame"]), row["method"], int(row["random_rep"]), int(row["budget"])): row
             for row in tensor_rows}
    rows = []
    for frame in frames:
        for budget in BUDGETS:
            for method, reps in (("WEAK2", (-1,)), ("STRONG2", (-1,)), ("RANDOM2", (0, 1, 2))):
                values = [index[(frame, method, rep, budget)] for rep in reps]
                metrics = {}
                for field, output in (("S_terminal", "S_terminal"),
                                      ("trace_A_terminal", "trace_A_terminal"),
                                      ("escape_fraction", "escape_fraction")):
                    numbers = [float(row[field]) for row in values]
                    metrics[output] = format(float(np.median(numbers)), ".17g")
                    metrics[output + "_min"] = format(float(min(numbers)), ".17g")
                    metrics[output + "_max"] = format(float(max(numbers)), ".17g")
                rows.append({"frame": frame,
                    "label": "MAJOR_COMPETITOR" if frame in major_frames else "NO_MAJOR_BASIN",
                    "budget": budget, "method": method, "replicate_count": len(reps), **metrics})
    return rows


def analyze():
    manifest = verify_manifest()
    results_path = OUT / "ndt_probe_results.csv"
    require(results_path.is_file() and manifest["current_run_state"] == "RUN_COMPLETE",
            "R2 alignments are not complete")
    result_rows = read_csv(results_path)
    require(len(result_rows) == 2560, "NDT output must contain exactly 2560 rows")
    major_frames = frozen_labels_after_run()
    frame_ids = list(FRAME_ORDER)
    labels = np.array([frame in major_frames for frame in frame_ids], dtype=bool)
    require(int(labels.sum()) == 9, "major/no-major frame counts are not 9/23")
    probes = read_csv(OUT / "probe_manifest.csv")
    probe_index = {(int(r["frame"]), r["method"], int(r["random_rep"]), int(r["probe_rank"])): r
                   for r in probes}
    result_index = {}
    for row in result_rows:
        key = (int(row["frame"]), row["method"], int(row["random_rep"]), int(row["probe_rank"]))
        require(key in probe_index and key not in result_index, "NDT row does not match frozen probe manifest")
        require(int(row["seed_index"]) == int(probe_index[key]["seed_index"]), "seed/probe order mismatch")
        require(int(row["converged"]) in (0, 1) and 0 <= int(row["iterations"]) <= 80 and
                np.isfinite(float(row["runtime_ms"])), "invalid returned NDT status/cost")
        result_index[key] = row
    require(len(result_index) == len(probe_index) == 2560, "probe/result join incomplete")

    terminal_rows = []
    displacements = {}
    for key, row in sorted(result_index.items()):
        frame, method, rep, rank = key
        xi = np.array([float(row[f"terminal_disp_{axis}"]) for axis in range(6)])
        require(np.isfinite(xi).all(), "non-finite terminal displacement")
        displacements[key] = xi
        terminal_rows.append({"frame": frame, "frame_id": row["frame_id"], "label":
            "MAJOR_COMPETITOR" if frame in major_frames else "NO_MAJOR_BASIN",
            "method": method, "random_rep": rep, "probe_rank": rank,
            "seed_index": int(row["seed_index"]), **{f"xi_{i}": format(float(xi[i]), ".17g") for i in range(6)},
            "xi_norm": format(float(np.linalg.norm(xi)), ".17g"),
            "translation_from_nominal_m": row["translation_from_nominal_m"],
            "rotation_from_nominal_deg": row["rotation_from_nominal_deg"],
            "nominal_return": int(float(row["translation_from_nominal_m"]) <= 0.2 and
                                   float(row["rotation_from_nominal_deg"]) <= 2.0),
            "converged": row["converged"], "iterations": row["iterations"],
            "runtime_ms": row["runtime_ms"], "delta_raw_ndt_score_sum": row["delta_score_sum"]})
    write_csv(OUT / "terminal_displacements.csv", terminal_rows)

    tensor_rows = []
    score_by_budget = {}
    for frame in frame_ids:
        for method, replicate in METHODS:
            xi = np.vstack([displacements[(frame, method, replicate, rank)] for rank in range(1, 17)])
            result_subset = [result_index[(frame, method, replicate, rank)] for rank in range(1, 17)]
            for budget in BUDGETS:
                selected = xi[:budget]
                tensor = selected.T @ selected / budget
                eigenvalues, eigenvectors = np.linalg.eigh(tensor)
                eigenvalues = np.maximum(eigenvalues, 0.0)
                principal = eigenvectors[:, -1].copy()
                nonzero = np.flatnonzero(np.abs(principal) > 1e-12)
                if nonzero.size and principal[nonzero[0]] < 0:
                    principal *= -1.0
                subset_rows = result_subset[:budget]
                returns = sum(float(row["translation_from_nominal_m"]) <= 0.2 and
                              float(row["rotation_from_nominal_deg"]) <= 2.0 for row in subset_rows)
                mean_delta_score = float(np.mean([float(row["delta_score_sum"]) for row in subset_rows]))
                score = math.sqrt(float(eigenvalues[-1]))
                score_by_budget[(frame, method, replicate, budget)] = score
                row = {"frame": frame, "method": method, "random_rep": replicate, "budget": budget,
                       "S_terminal": format(score, ".17g"), "trace_A_terminal": format(float(np.trace(tensor)), ".17g"),
                       "median_xi_norm": format(float(np.median(np.linalg.norm(selected, axis=1))), ".17g"),
                       "max_xi_norm": format(float(np.linalg.norm(selected, axis=1).max()), ".17g"),
                       "nominal_return_fraction": format(returns / budget, ".17g"),
                       "escape_fraction": format(1.0 - returns / budget, ".17g"),
                       "mean_delta_raw_ndt_score_sum": format(mean_delta_score, ".17g"),
                       "principal_eigenvalue": format(float(eigenvalues[-1]), ".17g"),
                       "principal_direction": text(principal),
                       "tensor_rowmajor": text(tensor.reshape(-1)),
                       **{f"eigenvalue_{i}": format(float(eigenvalues[i]), ".17g") for i in range(6)}}
                tensor_rows.append(row)
    write_csv(OUT / "frame_terminal_tensor.csv", tensor_rows)
    key_rows = key_frame_summaries(tensor_rows, major_frames)
    write_csv(OUT / "key_frame_summary.csv", key_rows)

    permutations = make_permutation_indices()
    statistics_rows = []
    lofo_rows = []
    by_frame_stats = {}
    method_sets = [("WEAK2", -1), ("STRONG2", -1), ("RANDOM2", 0),
                   ("RANDOM2", 1), ("RANDOM2", 2)]
    for budget in BUDGETS:
        frame_scores = {}
        for method, rep in method_sets:
            scores = np.array([score_by_budget[(frame, method, rep, budget)] for frame in frame_ids])
            auc, pvalue = permutation_auc_p(labels, scores, permutations)
            fold_rows, accuracy, train_auc_range = lofo(labels, scores, frame_ids)
            stats = {"budget": budget, "method": method, "random_rep": rep,
                     "major_mean": float(scores[labels].mean()), "major_median": float(np.median(scores[labels])),
                     "healthy_mean": float(scores[~labels].mean()), "healthy_median": float(np.median(scores[~labels])),
                     "roc_auc": auc, "frame_permutation_p": pvalue, "lofo_accuracy": accuracy,
                     "lofo_training_auc_min": train_auc_range[0], "lofo_training_auc_max": train_auc_range[1],
                     "random_rep_auc_median": "", "random_rep_auc_min": "", "random_rep_auc_max": ""}
            frame_scores[(method, rep)] = scores
            statistics_rows.append({k: (format(v, ".17g") if isinstance(v, float) else v)
                                    for k, v in stats.items()})
            for fold in fold_rows:
                lofo_rows.append({"budget": budget, "method": method, "random_rep": rep, **fold})
            by_frame_stats[(budget, method, rep)] = stats

        random_stack = np.vstack([frame_scores[("RANDOM2", rep)] for rep in range(3)])
        random_median_scores = np.median(random_stack, axis=0)
        auc, pvalue = permutation_auc_p(labels, random_median_scores, permutations)
        fold_rows, accuracy, train_auc_range = lofo(labels, random_median_scores, frame_ids)
        stats = {"budget": budget, "method": "RANDOM2_MEDIAN", "random_rep": -2,
                 "major_mean": float(random_median_scores[labels].mean()),
                 "major_median": float(np.median(random_median_scores[labels])),
                 "healthy_mean": float(random_median_scores[~labels].mean()),
                 "healthy_median": float(np.median(random_median_scores[~labels])),
                 "roc_auc": auc, "frame_permutation_p": pvalue, "lofo_accuracy": accuracy,
                 "lofo_training_auc_min": train_auc_range[0], "lofo_training_auc_max": train_auc_range[1],
                 "random_rep_auc_median": float(np.median([by_frame_stats[(budget, "RANDOM2", rep)]["roc_auc"]
                                                           for rep in range(3)])),
                 "random_rep_auc_min": float(min(by_frame_stats[(budget, "RANDOM2", rep)]["roc_auc"]
                                                 for rep in range(3))),
                 "random_rep_auc_max": float(max(by_frame_stats[(budget, "RANDOM2", rep)]["roc_auc"]
                                                 for rep in range(3)))}
        statistics_rows.append({k: (format(v, ".17g") if isinstance(v, float) else v)
                                for k, v in stats.items()})
        for fold in fold_rows:
            lofo_rows.append({"budget": budget, "method": "RANDOM2_MEDIAN", "random_rep": -2, **fold})
        by_frame_stats[(budget, "RANDOM2_MEDIAN", -2)] = stats
        for i, frame in enumerate(frame_ids):
            by_frame_stats[(budget, "RANDOM2_MEDIAN", -2, frame)] = float(random_median_scores[i])
    write_csv(OUT / "budget_statistics.csv", statistics_rows)
    write_csv(OUT / "lofo.csv", lofo_rows)

    # The per-frame comparisons are the three methods' actual frame-level scores.
    per_frame_rows = []
    for budget in BUDGETS:
        for index, frame in enumerate(frame_ids):
            weak = score_by_budget[(frame, "WEAK2", -1, budget)]
            strong = score_by_budget[(frame, "STRONG2", -1, budget)]
            random_values = [score_by_budget[(frame, "RANDOM2", rep, budget)] for rep in range(3)]
            per_frame_rows.append({"frame": frame, "label": "MAJOR_COMPETITOR" if frame in major_frames else "NO_MAJOR_BASIN",
                "major_count": int(frame in major_frames), "budget": budget,
                "S_weak2": format(weak, ".17g"), "S_strong2": format(strong, ".17g"),
                "S_random2_median": format(float(np.median(random_values)), ".17g"),
                "S_random2_min": format(float(min(random_values)), ".17g"),
                "S_random2_max": format(float(max(random_values)), ".17g"),
                "weak_minus_strong": format(weak - strong, ".17g"),
                "weak_minus_random_median": format(weak - float(np.median(random_values)), ".17g"),
                "rho_W2_H1_secondary": ""})
    h1_rho = {int(row["frame"]): float(row["R_weak2"])
              for row in read_csv(H1 / "frame_projection_statistics.csv")}
    selected_key_budget = None
    budget_decisions = []
    for budget in BUDGETS:
        weak = by_frame_stats[(budget, "WEAK2", -1)]
        strong = by_frame_stats[(budget, "STRONG2", -1)]
        random_median = by_frame_stats[(budget, "RANDOM2_MEDIAN", -2)]
        eligible = (weak["roc_auc"] >= 0.80 and weak["frame_permutation_p"] < 0.05 and
                    weak["lofo_accuracy"] >= 0.80 and weak["roc_auc"] > strong["roc_auc"] and
                    weak["roc_auc"] > random_median["roc_auc"])
        budget_decisions.append({"budget": budget, "weak_auc": weak["roc_auc"],
            "weak_p": weak["frame_permutation_p"], "weak_lofo_accuracy": weak["lofo_accuracy"],
            "strong_auc": strong["roc_auc"], "random2_median_auc": random_median["roc_auc"],
            "eligible": bool(eligible)})
        if eligible and selected_key_budget is None:
            selected_key_budget = budget

    directionality = []
    if selected_key_budget is not None:
        tensor_lookup = {(int(row["frame"]), row["method"], int(row["random_rep"]), int(row["budget"])): row
                         for row in tensor_rows}
        uobs_rows = read_csv(ARCHIVE / "dual_u.csv")
        uobs_by_frame = {int(row["transaction_id"]): row for row in uobs_rows}
        for frame in frame_ids:
            weak_row = tensor_lookup[(frame, "WEAK2", -1, selected_key_budget)]
            principal = vector(weak_row["principal_direction"], 6)
            # The W2 subspace is the frozen first two columns of the R1 U_obs eigenbasis.
            obs = uobs_by_frame[frame]
            q = vector(obs["curvature_eigenvectors_rowmajor"], 36).reshape(6, 6)
            angle = math.degrees(math.acos(float(np.clip(np.linalg.norm(q[:, :2].T @ principal), 0.0, 1.0))))
            directionality.append({"frame": frame, "label": "MAJOR_COMPETITOR" if frame in major_frames else "NO_MAJOR_BASIN",
                                   "budget": selected_key_budget,
                                   "principal_axis_angle_to_weak2_deg": format(angle, ".17g")})
        write_csv(OUT / "directionality_secondary.csv", directionality)

    # Populate H1 secondary rho only after all probe orders and NDT outputs are fixed.
    for row in per_frame_rows:
        frame = int(row["frame"])
        row["rho_W2_H1_secondary"] = (format(h1_rho[frame], ".17g") if frame in h1_rho else "")
    write_csv(OUT / "per_frame_budget_scores.csv", per_frame_rows)

    best = next((item for item in budget_decisions if item["eligible"]), None)
    weak_any = [by_frame_stats[(b, "WEAK2", -1)] for b in BUDGETS]
    weak_partial = any(s["roc_auc"] >= 0.80 and s["frame_permutation_p"] < 0.05
                       for s in weak_any)
    weak_discrimination = [s for s in weak_any if s["roc_auc"] >= 0.80 and
                           s["frame_permutation_p"] < 0.05 and s["lofo_accuracy"] >= 0.80]
    if best is not None:
        final_result = ("LOW_BUDGET_TERMINAL_STABILITY_SUPPORTED" if best["budget"] <= 8
                        else "TERMINAL_STABILITY_EVIDENCE_SUPPORTED")
    elif any(s["roc_auc"] <= by_frame_stats[(s["budget"], "STRONG2", -1)]["roc_auc"] or
             s["roc_auc"] <= by_frame_stats[(s["budget"], "RANDOM2_MEDIAN", -2)]["roc_auc"]
             for s in weak_discrimination):
        final_result = "GENERIC_TERMINAL_SENSITIVITY_ONLY"
    elif weak_partial and not weak_discrimination:
        final_result = "TERMINAL_STABILITY_COHORT_DEPENDENT"
    else:
        final_result = "TERMINAL_STABILITY_NOT_DISCRIMINATIVE"

    # All random replicates are retained; report their performance range per budget.
    random_ranges = {}
    for budget in BUDGETS:
        aucs = [by_frame_stats[(budget, "RANDOM2", rep)]["roc_auc"] for rep in range(3)]
        random_ranges[str(budget)] = {"median_auc": float(np.median(aucs)), "min_auc": float(min(aucs)),
                                      "max_auc": float(max(aucs))}
    total_runtime = sum(float(row["runtime_ms"]) for row in result_rows)
    total_iterations = sum(int(row["iterations"]) for row in result_rows)
    total_objective_eval = sum(float(row["objective_eval_ms"]) for row in result_rows)
    budget_cost_rows = []
    budget_cost_summary = []
    for budget in BUDGETS:
        cost_groups = [("WEAK2", [-1]), ("STRONG2", [-1]),
                       ("RANDOM2_REP0", [0]), ("RANDOM2_REP1", [1]),
                       ("RANDOM2_REP2", [2]), ("RANDOM2_ALL", [0, 1, 2])]
        for method_label, reps in cost_groups:
            selected = [row for row in result_rows
                        if int(row["probe_rank"]) <= budget and
                        ((method_label == "WEAK2" and row["method"] == "WEAK2") or
                         (method_label == "STRONG2" and row["method"] == "STRONG2") or
                         (method_label.startswith("RANDOM2") and row["method"] == "RANDOM2" and
                          int(row["random_rep"]) in reps))]
            expected_calls_per_frame = budget * len(reps)
            require(len(selected) == len(frame_ids) * expected_calls_per_frame,
                    "budget cost group has incomplete probe rows")
            mean_runtime_frame = sum(float(row["runtime_ms"]) for row in selected) / len(frame_ids)
            mean_iterations_frame = sum(int(row["iterations"]) for row in selected) / len(frame_ids)
            mean_runtime_probe = sum(float(row["runtime_ms"]) for row in selected) / len(selected)
            mean_iterations_probe = sum(int(row["iterations"]) for row in selected) / len(selected)
            cost = {"budget": budget, "method": method_label,
                    "ndt_calls_per_frame": expected_calls_per_frame,
                    "mean_runtime_ms_per_probe": mean_runtime_probe,
                    "mean_runtime_ms_per_frame": mean_runtime_frame,
                    "mean_iterations_per_probe": mean_iterations_probe,
                    "mean_iterations_per_frame": mean_iterations_frame}
            budget_cost_summary.append(cost)
            budget_cost_rows.append({key: (format(value, ".17g") if isinstance(value, float) else value)
                                     for key, value in cost.items()})
    write_csv(OUT / "budget_costs.csv", budget_cost_rows)
    per_frame_cost = {}
    for frame in frame_ids:
        frame_rows = [row for row in result_rows if int(row["frame"]) == frame]
        per_frame_cost[str(frame)] = {"ndt_calls": len(frame_rows),
            "align_runtime_ms": sum(float(row["runtime_ms"]) for row in frame_rows),
            "iterations": sum(int(row["iterations"]) for row in frame_rows)}
    status_counts = {status: sum(row["status"] == status for row in result_rows)
                     for status in sorted({row["status"] for row in result_rows})}
    summary = {
        "task": "PAPER-P9-R2-LOW-BUDGET-TERMINAL-STABILITY-EVIDENCE",
        "current_run_state": "COMPLETE", "final_result": final_result,
        "git": git_state(), "start_sha": START_SHA,
        "labels": {"major_competitor_frames": sorted(major_frames), "major_count": 9,
                   "no_major_count": 23, "loaded_after_ndt_run": True},
        "probe_contract": manifest["probe_contract"],
        "ndt": manifest["ndt"], "random_basis_seed_formula": manifest["probe_contract"]["random_seed_formula"],
        "alignment": {"new_ndt_calls": len(result_rows), "nonconverged_status_counts": status_counts,
            "mean_runtime_ms_per_probe": total_runtime / len(result_rows),
            "mean_objective_eval_ms_per_probe": total_objective_eval / len(result_rows),
            "mean_iterations_per_probe": total_iterations / len(result_rows),
            "mean_runtime_ms_per_frame_all_methods": total_runtime / len(frame_ids),
            "mean_iterations_per_frame_all_methods": total_iterations / len(frame_ids),
            "proposal_generation_and_selection_seconds": manifest["probe_contract"]["selection_elapsed_s"],
            "budget_costs": budget_cost_summary,
            "per_frame_cost": per_frame_cost,
            "note": "timing is descriptive; same current executable/platform across WEAK2/STRONG2/RANDOM2"},
        "permutation_test": {"type": "Monte Carlo frame-label permutation, one-sided high AUC",
            "replicates": PERMUTATION_REPLICATES, "pcg64_seed": PERMUTATION_SEED,
            "plus_one_correction": True},
        "budget_decisions": budget_decisions, "random_auc_ranges": random_ranges,
        "best_budget": None if best is None else best["budget"],
        "directionality_secondary": directionality,
        "interpretation": "terminal second moment is perturb-and-reoptimize sensitivity evidence, not posterior covariance, basin recovery, or Bayesian evidence",
        "gt_used": False, "support_features_used": False, "canonical_basin_pose_used": False,
        "artifact_sha256": {},
    }
    report = render_report(summary, statistics_rows, per_frame_rows, budget_decisions,
                           status_counts, key_rows)
    (OUT / "REPORT.md").write_text(report)
    (OUT / "results.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    artifacts = [OUT / name for name in (
        "random_subspaces.csv", "proposal_pool.csv", "proposal_roundtrip.csv", "probe_manifest.csv",
        "ndt_probe_results.csv", "terminal_displacements.csv", "frame_terminal_tensor.csv",
        "budget_statistics.csv", "budget_costs.csv", "lofo.csv", "per_frame_budget_scores.csv", "key_frame_summary.csv",
        "REPORT.md")]
    artifacts += shard_paths()
    artifacts += [OUT / "directionality_secondary.csv"] if (OUT / "directionality_secondary.csv").exists() else []
    summary["artifact_sha256"] = {path.relative_to(OUT).as_posix(): digest(path) for path in artifacts}
    (OUT / "results.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"FINAL_RESULT": final_result, "BEST_BUDGET": summary["best_budget"],
                      "BUDGET_DECISIONS": budget_decisions,
                      "NDT_CALLS": len(result_rows), "STATUS_COUNTS": status_counts}, indent=2))


def render_report(summary, statistics_rows, per_frame_rows, decisions, status_counts, key_rows):
    lines = ["# P9-R2 Low-Budget Terminal Stability", "",
        f"FINAL_RESULT = `{summary['final_result']}`", "",
        "This is perturb-and-reoptimize terminal-instability evidence. It is not basin recovery, a posterior, or a covariance.", "",
        f"Frozen cohort: 32 frames; 9 major-competitor and 23 no-major frames. New NDT calls: {summary['alignment']['new_ndt_calls']}.",
        f"NDT statuses: `{json.dumps(status_counts, sort_keys=True)}`.", "",
        "## Probe contract", "",
        "The 263 archived base seeds (indices 0–262) were projected into WEAK2, STRONG2, and three frozen RANDOM2 subspaces. Each projected 2-D pool was ordered by deterministic farthest-point selection; no proposals were renormalized or deduplicated. Selection manifests were frozen before any NDT call.", "",
        "RANDOM2 uses PCG64 and seeds `20261007 + 1000*frame_index + replicate` (replicates 0–2). Budgets are 4, 8, 12, and 16. All 2,560 selected probes were evaluated once.", "",
        "## Budget results", "",
        "| B | Method | Major mean / median S | Healthy mean / median S | ROC AUC | frame permutation p | LOFO accuracy |", "|---:|---|---:|---:|---:|---:|---:|"]
    for row in statistics_rows:
        if row["method"] not in ("WEAK2", "STRONG2", "RANDOM2_MEDIAN"):
            continue
        lines.append(f"| {row['budget']} | {row['method']} | {float(row['major_mean']):.4g} / {float(row['major_median']):.4g} | {float(row['healthy_mean']):.4g} / {float(row['healthy_median']):.4g} | {float(row['roc_auc']):.3f} | {float(row['frame_permutation_p']):.4f} | {float(row['lofo_accuracy']):.3f} |")
    lines += ["", "Random2 median rows use the per-frame median of the three pre-frozen random-subspace scores; individual random replicate ROC AUC values and ranges are retained in `budget_statistics.csv`.", "",
        "## Frozen major frames and controls at B=16", "",
        "Each cell shows `S_terminal / trace(A_terminal) / escape fraction`; RANDOM2 reports the median across its three frozen subspaces and the bracketed range for S.", "",
        "| Frame | Label | WEAK2 | STRONG2 | RANDOM2 median [S range] |", "|---:|---|---:|---:|---:|"]
    chosen_frames = [368, 2226, 2350, 2722, 3796]
    healthy = [int(row["frame"]) for row in per_frame_rows if row["label"] == "NO_MAJOR_BASIN"]
    chosen_frames += sorted(set(healthy))[:5]
    final_budget = 16
    key_index = {(int(row["frame"]), row["method"]): row for row in key_rows
                 if int(row["budget"]) == final_budget}
    for frame in chosen_frames:
        weak = key_index[(frame, "WEAK2")]
        strong = key_index[(frame, "STRONG2")]
        random = key_index[(frame, "RANDOM2")]
        fmt = lambda row: f"{float(row['S_terminal']):.3g} / {float(row['trace_A_terminal']):.3g} / {float(row['escape_fraction']):.2f}"
        lines.append(f"| {frame} | {weak['label']} | {fmt(weak)} | {fmt(strong)} | {fmt(random)} [{float(random['S_terminal_min']):.3g}, {float(random['S_terminal_max']):.3g}] |")
    lines += ["", "## Cost", "",
        f"Across all 2,560 calls, mean NDT alignment time was {summary['alignment']['mean_runtime_ms_per_probe']:.3f} ms/probe and mean iterations were {summary['alignment']['mean_iterations_per_probe']:.2f}/probe. Proposal generation, projection, and deterministic selection took {summary['alignment']['proposal_generation_and_selection_seconds']:.2f} s total ({summary['alignment']['proposal_generation_and_selection_seconds'] / 32:.3f} s/frame).",
        "At budget B, WEAK2 and STRONG2 each consume B extra calls/frame; RANDOM2 consumes 3B calls/frame across its three fixed subspaces.", "",
        "| B | Method | NDT calls/frame | Mean NDT ms/frame | Mean iterations/frame |", "|---:|---|---:|---:|---:|"]
    for row in summary["alignment"]["budget_costs"]:
        lines.append(f"| {row['budget']} | {row['method']} | {row['ndt_calls_per_frame']} | {row['mean_runtime_ms_per_frame']:.1f} | {row['mean_iterations_per_frame']:.1f} |")
    lines += ["", "Full all-method per-frame totals are retained in `results.json`; the same cost rows are archived in `budget_costs.csv`.", "",
        "## Decision", "",
        "| B | WEAK AUC | WEAK p | WEAK LOFO accuracy | STRONG AUC | RANDOM2 median AUC | Gate |", "|---:|---:|---:|---:|---:|---:|---|"]
    for d in decisions:
        lines.append(f"| {d['budget']} | {d['weak_auc']:.3f} | {d['weak_p']:.4f} | {d['weak_lofo_accuracy']:.3f} | {d['strong_auc']:.3f} | {d['random2_median_auc']:.3f} | {'PASS' if d['eligible'] else 'FAIL'} |")
    lines += ["", f"Best qualifying budget: `{summary['best_budget']}`.", "",
        "No GT, support event, canonical basin pose, posterior weighting, or EKF state was used.", ""]
    return "\n".join(lines)


def audit():
    manifest = verify_manifest()
    require(manifest["current_run_state"] == "RUN_COMPLETE", "R2 run is not complete")
    results = read_csv(OUT / "ndt_probe_results.csv")
    require(len(results) == 2560, "NDT result row count audit failed")
    require(digest(OUT / "ndt_probe_results.csv") == manifest["ndt_probe_result_sha256"],
            "consolidated NDT hash differs from execution manifest")
    require(len(read_csv(OUT / "probe_manifest.csv")) == 2560, "probe manifest row count audit failed")
    require(len(read_csv(OUT / "proposal_roundtrip.csv")) == 32 * 263, "round-trip rows missing")
    require(len(read_csv(OUT / "random_subspaces.csv")) == 96, "random subspace count mismatch")
    require(len(read_csv(OUT / "proposal_pool.csv")) == 32 * 263 * 5, "proposal pool size mismatch")
    require(len(read_csv(OUT / "terminal_displacements.csv")) == 2560, "terminal displacement rows missing")
    require(len(read_csv(OUT / "frame_terminal_tensor.csv")) == 32 * 5 * 4, "frame tensor row count mismatch")
    require(len(read_csv(OUT / "key_frame_summary.csv")) == 10 * 3 * 4, "key-frame summary row count mismatch")
    require(len(read_csv(OUT / "budget_statistics.csv")) == 4 * 6, "budget statistic row count mismatch")
    require(len(read_csv(OUT / "budget_costs.csv")) == 4 * 6, "budget cost row count mismatch")
    require(len(read_csv(OUT / "lofo.csv")) == 4 * 6 * 32, "LOFO row count mismatch")
    summary = json.loads((OUT / "results.json").read_text())
    for name, expected in summary["artifact_sha256"].items():
        path = OUT / name
        require(path.is_file() and digest(path) == expected, "archived artifact hash mismatch: " + name)
    require((OUT / "REPORT.md").is_file(), "final report missing")
    return {"audit": "PASS", "ndt_rows": len(results), "artifact_count": len(summary["artifact_sha256"])}


def self_test():
    # Farthest-point center and deterministic tie-break, including duplicate coordinates.
    rows = [{"seed_index": str(i), "coord0": str(x), "coord1": str(y)} for i, (x, y) in enumerate(
        [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1), (2, 0)] +
        [(0.0001 * (i - 6), 0.0001) for i in range(6, 263)])]
    first = choose_farthest(rows, 6)
    second = choose_farthest(rows, 6)
    require([(x[0], x[1]) for x in first] == [(x[0], x[1]) for x in second],
            "farthest selection is not deterministic")
    require(first[0][1] == 0 and first[1][1] == 5 and first[2][1] == 1,
            "farthest selection center/tie regression")
    q = haar_basis(20261007)
    require(np.linalg.norm(q.T @ q - np.eye(2)) < 1e-12 and
            np.array_equal(q, haar_basis(20261007)), "PCG64 Haar basis determinism regression")
    labels = np.array([1, 1, 0, 0], dtype=bool)
    require(abs(roc_auc(labels, np.array([4., 3., 2., 1.])) - 1.0) < 1e-12,
            "ROC AUC rank statistic regression")
    require(abs(roc_auc(labels, np.array([1., 2., 3., 4.])) - 0.0) < 1e-12,
            "ROC AUC reverse ordering regression")
    tensor = np.eye(6) * 4.0
    require(abs(math.sqrt(np.linalg.eigvalsh(tensor).max()) - 2.0) < 1e-12 and
            abs(np.trace(tensor) - 24.0) < 1e-12, "terminal tensor statistic regression")
    train_y = np.array([1, 1, 0, 0], dtype=bool)
    threshold, _, _, balanced = best_training_threshold(train_y, np.array([4., 3., 2., 1.]))
    require(2.0 < threshold <= 2.5 and balanced == 1.0, "train-only threshold regression")
    heldout = ["a", "b", "c", "d"]
    fold_before, _, _ = lofo(train_y, np.array([4., 3., 2., 1.]), heldout)
    changed_scores = np.array([400., 3., 2., 1.])
    fold_after, _, _ = lofo(train_y, changed_scores, heldout)
    require(fold_before[0]["train_only_threshold"] == fold_after[0]["train_only_threshold"],
            "LOFO threshold leaked its held-out score")
    require(len(make_permutation_indices()) == PERMUTATION_REPLICATES,
            "frame-label permutation schedule regression")
    print("P9_R2_STATISTICS_SELF_TEST=PASS")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("stage", choices=("prepare", "recover-prepared", "run", "rebind-analysis",
                                           "analyze", "audit", "self-test"))
    parser.add_argument("--binary", type=Path, default=BIN_DEFAULT)
    args = parser.parse_args()
    if args.stage == "self-test":
        self_test()
    elif args.stage == "prepare":
        prepare(args.binary.resolve())
    elif args.stage == "recover-prepared":
        recover_prepared_manifest(args.binary.resolve())
    elif args.stage == "rebind-analysis":
        rebind_analysis_source()
    elif args.stage == "run":
        run(args.binary.resolve())
    elif args.stage == "analyze":
        analyze()
    elif args.stage == "audit":
        print(json.dumps(audit(), indent=2))


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("P9_R2_CONTRACT_FAIL=" + repr(error), file=sys.stderr)
        raise
