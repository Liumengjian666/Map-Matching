#!/usr/bin/env python3
"""Frozen-oracle H1 only: no NDT, search, map, GT, or trajectory input."""
import argparse
import csv
import ctypes
import hashlib
import itertools
import json
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
START_SHA = "7d32e7915bc02d7339f940426a1f2090d71e3d39"
ARCHIVE = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/"
               "dual_u_r1_closure_20261003/same_objective")
R1 = ROOT / "docs/p9_dual_u_nonlocal_r1"
R1A = ROOT / "docs/p9_r1a_true_profile_closure"
FRAMES = (368, 616, 2226, 2350, 2722, 2846, 3341, 3796, 3962)
REPLICATES = 10000
SEEDS = {"haar2": 20261006, "haar1": 20261007,
         "haar3": 20261008, "frame_bootstrap": 20261009}
PARITY_TOL = 1e-6  # Fixed before executing parity; not fit to the observations.
OUTPUT = ROOT / "docs/p9_foundation_weak_discovery/h1"


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def read_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def vector(text, size):
    result = np.fromstring(text, sep=";")
    require(result.size == size and np.isfinite(result).all(), "invalid vector")
    return result


def chart_function(library):
    function = ctypes.CDLL(str(library)).p9_h1_displacement
    pointer = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
    function.argtypes = [pointer, pointer, pointer]
    function.restype = None

    def evaluate(pose, matrix):
        result = np.empty(6)
        function(np.ascontiguousarray(pose), np.ascontiguousarray(matrix), result)
        require(np.isfinite(result).all(), "invalid chart displacement")
        return result
    return evaluate


def build_chart(directory):
    source = Path(__file__).resolve().with_name("p9_h1_chart.cpp")
    source_hash = digest(source)
    library = Path(directory) / "chart.so"
    flags = ["-std=c++14", "-O3", "-DNDEBUG", "-shared", "-fPIC", "-I/usr/include/eigen3"]
    command = ["g++", *flags, str(source), "-o", str(library)]
    subprocess.run(command, check=True)
    require(digest(source) == source_hash, "chart source changed during build")
    return library, dict(source=str(source.relative_to(ROOT)), source_sha256=source_hash,
                         compiler=subprocess.check_output(["g++", "--version"], text=True).splitlines()[0],
                         command=command, binary_sha256=digest(library))


def failed_run(output, error):
    # Old CSVs are preserved but cannot constitute a current successful result.
    output.mkdir(parents=True, exist_ok=True)
    result = dict(task="PAPER-P9-FOUNDATION-H1-WEAK-SUBSPACE-CONCENTRATION-CLOSURE",
                  h1="NOT_EVALUATED", final_result="H1_HISTORICAL_PROJECTION_PARITY_FAIL"
                  if "H1_HISTORICAL_PROJECTION_PARITY_FAIL" in error else "H1_INPUT_OR_VERIFICATION_FAILED",
                  current_run_state="FAILED", error=error, statistical_sidecars_current=False,
                  new_ndt_calls=0, gt_used=False, h2_run=False)
    (output / "results.json").write_text(json.dumps(result, indent=2) + "\n")
    (output / "REPORT.md").write_text("# H1 run failed\n\n" + result["final_result"] +
                                     "\n\n" + error + "\n\nPrevious statistical CSVs are not current results.\n")


def load_inputs(chart):
    paths = [ARCHIVE / "candidates.csv", ARCHIVE / "clusters/mode_clusters.csv",
             ARCHIVE / "frozen/cohort_frozen.csv", ARCHIVE / "dual_u.csv",
             R1 / "results.json", R1 / "sidecars/profile_cohort_final/oracle_recovery.csv",
             R1A / "boundary_diagnostic.csv", R1A / "oracle_terminal_requests.csv",
             R1A / "oracle_preparation.json",
             ROOT / "docs/p9_r1b_strong_attractor_closure/input_provenance.json"]
    inherited = json.loads((R1A / "oracle_preparation.json").read_text())["inputs_sha256"]
    inherited.update(json.loads((ROOT / "docs/p9_r1b_strong_attractor_closure/input_provenance.json").read_text()))
    hashes = {str(path): digest(path) for path in paths}
    for path in paths:
        if ROOT in path.parents:
            relative = str(path.relative_to(ROOT))
            expected = hashlib.sha256(subprocess.check_output(
                ["git", "show", f"{START_SHA}:{relative}"], cwd=ROOT)).hexdigest()
        else:
            require(str(path) in inherited, "missing frozen input digest: " + str(path))
            expected = inherited[str(path)]
        require(hashes[str(path)] == expected, "input hash changed: " + str(path))
    recovery = read_csv(paths[5])
    ids = [(int(r["transaction_id"]), cluster) for r in recovery
           for cluster in r["missed_major_cluster_ids"].split(";")
           if cluster not in ("", "NONE")]
    require(len(ids) == len(set(ids)) == 22 and
            sorted({tx for tx, _ in ids}) == list(FRAMES), "frozen major cohort mismatch")
    candidates = {(int(r["transaction_id"]), int(r["seed_index"])): r
                  for r in read_csv(paths[0])}
    clusters = {(int(r["transaction_id"]), r["cluster_id"]): r for r in read_csv(paths[1])
                if r["threshold_set"] == "primary"}
    cohort = {int(r["transaction_id"]): r for r in read_csv(paths[2])}
    uobs = {int(r["transaction_id"]): r for r in read_csv(paths[3])}
    historical = {(int(r["transaction_id"]), r["cluster_id"]): r for r in read_csv(paths[6])}
    requests = {(int(r["transaction_id"]), r["cluster_id"]): r for r in read_csv(paths[7])}
    major, normalized, bases = [], {}, {}
    for tx, cluster in sorted(ids):
        u, c, request = uobs[tx], clusters[(tx, cluster)], requests[(tx, cluster)]
        require(u["uobs_valid"] == "1" and u["uobs_status"] == "PASS_LOCAL_NDT_CURVATURE", "invalid U_obs")
        require(u["source_cloud_hash"] == cohort[tx]["prepared_source_hash"], "source mismatch")
        eigenvalues = vector(u["curvature_eigenvalues"], 6)
        q = vector(u["curvature_eigenvectors_rowmajor"], 36).reshape(6, 6)
        require(np.all(np.diff(eigenvalues) >= 0) and eigenvalues[0] > 0,
                "eigenvalues not positive ascending")
        require(np.linalg.norm(q.T @ q - np.eye(6)) < 1e-12, "basis not orthonormal")
        require(c["representative_pose_matrix16"] == request["canonical_pose_matrix16"], "representative changed")
        seed = int(request["canonical_seed_index"])
        candidate = candidates[(tx, seed)]
        require(candidate["final_pose_matrix16"] == c["representative_pose_matrix16"], "canonical seed mismatch")
        pose = np.array([float(u[k]) for k in ("raw_x", "raw_y", "raw_z", "raw_qx", "raw_qy", "raw_qz", "raw_qw")])
        delta = chart(pose, vector(c["representative_pose_matrix16"], 16))
        norm = float(np.linalg.norm(delta))
        require(norm > 1e-12, "zero competing displacement")
        squares = (q.T @ (delta / norm)) ** 2
        old = float(historical[(tx, cluster)]["original_weak_projection"])
        row = dict(tx=tx, frame=c["frame_id"], cluster_id=cluster)
        row.update({f"delta_{i}": float(delta[i]) for i in range(6)})
        row.update(delta_norm=norm, rho_weak1=float(squares[0]),
                   rho_weak2=float(squares[:2].sum()), rho_weak3=float(squares[:3].sum()),
                   rho_strong2=float(squares[-2:].sum()), sqrt_rho_weak2=float(np.sqrt(squares[:2].sum())),
                   historical_projection_fraction=old,
                   parity_error=abs(float(np.sqrt(squares[:2].sum())) - old),
                   canonical_seed_index=seed, iteration_limit_flag=int(int(candidate["iterations"]) >= 80))
        major.append(row)
        normalized.setdefault(tx, []).append(delta / norm)
        bases[tx] = q
    fractions = np.array([r["sqrt_rho_weak2"] for r in major])
    parity = dict(major_basins=len(major), frames=len(normalized), historical_mean=0.8042,
                  reproduced_mean=float(fractions.mean()), historical_median=0.9238,
                  reproduced_median=float(np.median(fractions)), historical_ge_0p8=13,
                  reproduced_ge_0p8=int((fractions >= .8).sum()),
                  max_per_basin_error=max(r["parity_error"] for r in major), tolerance=PARITY_TOL)
    errors = [parity["max_per_basin_error"] <= PARITY_TOL,
              abs(parity["reproduced_mean"] - .8042) <= .00005,
              abs(parity["reproduced_median"] - .9238) <= .00005,
              parity["reproduced_ge_0p8"] == 13]
    for r in recovery:
        tx = int(r["transaction_id"])
        if tx in FRAMES:
            actual = np.mean([m["sqrt_rho_weak2"] for m in major if m["tx"] == tx])
            errors.append(abs(actual - float(r["weak_span_projection_fraction_mean"])) <= PARITY_TOL)
    parity["pass"] = bool(all(errors))
    return major, normalized, bases, parity, hashes


def haar_subspaces(rng, count, k):
    q, r = np.linalg.qr(rng.standard_normal((count, 6, k)))
    signs = np.where(np.diagonal(r, axis1=1, axis2=2) < 0, -1., 1.)
    return q * signs[:, None, :]


def exact_sign_flip(difference):
    signs = np.array(list(itertools.product((-1., 1.), repeat=len(difference))))
    observed = float(np.mean(difference))
    null = signs @ difference / len(difference)
    return float(np.mean(null >= observed - 1e-14))


def rank_of(values, index):
    return 1 + int(np.sum(values > values[index] + 1e-12))


def self_test(chart):
    origin = np.array([0., 0., 0., 0., 0., 0., 1.])
    identity = np.eye(4).reshape(-1)
    np.testing.assert_allclose(chart(origin, identity), 0, atol=1e-14)
    shifted = np.eye(4); shifted[:3, 3] = [.8, -1.6, 2.4]
    np.testing.assert_allclose(chart(origin, shifted.reshape(-1))[:3], [1, -2, 3], atol=2e-7)
    # Independent spatial rotation check with a nonidentity base orientation.
    from scipy.spatial.transform import Rotation
    base_rotation = Rotation.from_rotvec([.4, -.2, .3])
    base = np.r_[np.array([2., -1., .5]), base_rotation.as_quat()]
    target = np.eye(4)
    target[:3, :3] = (Rotation.from_rotvec([0., .1, 0.]) * base_rotation).as_matrix()
    target[:3, 3] = base[:3]
    np.testing.assert_allclose(chart(base, target.reshape(-1))[3:], [0, .1, 0], atol=2e-7)
    require(exact_sign_flip(np.ones(9)) == 1/512, "exact-test denominator regression")
    require(exact_sign_flip(np.zeros(9)) == 1, "exact-test tie regression")
    q = haar_subspaces(np.random.Generator(np.random.PCG64(9)), 10000, 2)
    np.testing.assert_allclose(np.einsum('nki,nkj->nij', q, q),
                               np.broadcast_to(np.eye(2), (10000, 2, 2)), atol=1e-12)
    require(abs(np.mean(np.sum(q[:, 0, :]**2, axis=1)) - 1/3) < .01, "Haar sanity")
    # Exercise the actual production aggregation, not a test-only approximation.
    normalized = {tx: [np.eye(6)[0 if i == 0 else 5]] for i, tx in enumerate(FRAMES)}
    bases = {tx: np.eye(6) for tx in FRAMES}
    major = [dict(tx=tx, cluster_id="SYNTHETIC", rho_weak1=float(i == 0),
                  rho_weak2=float(i == 0), rho_weak3=float(i == 0), rho_strong2=float(i != 0))
             for i, tx in enumerate(FRAMES)]
    first, _ = statistics(major, normalized, bases)
    normalized[FRAMES[0]] *= 6
    second, _ = statistics(major + [major[0].copy() for _ in range(5)], normalized, bases)
    np.testing.assert_allclose([first["main"]["T_weak2"], second["main"]["T_weak2"]], [1/9, 1/9], atol=1e-14)
    np.testing.assert_allclose([first["main"]["T_strong2"], second["main"]["T_strong2"]], [8/9, 8/9], atol=1e-14)
    with tempfile.TemporaryDirectory(prefix="p9-h1-failure-selftest-", dir="/tmp") as directory:
        path = Path(directory)
        (path / "results.json").write_text('{"h1":"PASS"}')
        failed_run(path, "H1_HISTORICAL_PROJECTION_PARITY_FAIL")
        invalidated = json.loads((path / "results.json").read_text())
        require(invalidated["current_run_state"] == "FAILED" and invalidated["h1"] != "PASS",
                "stale success invalidation regression")
    require(rank_of(np.array([.3, .3, .1]), 1) == 1, "pair tie regression")
    return "PASS"


def statistics(major, normalized, bases):
    frame_rows, pair_values = [], []
    pairs = list(itertools.combinations(range(6), 2))
    for tx in FRAMES:
        entries = [r for r in major if r["tx"] == tx]
        squared = (np.array(normalized[tx]) @ bases[tx]) ** 2
        values = np.array([squared[:, list(pair)].sum(axis=1).mean() for pair in pairs])
        pair_values.append(values)
        row = dict(frame=tx, major_count=len(entries),
                   major_ids=";".join(r["cluster_id"] for r in entries))
        row.update({"R_"+k: float(np.mean([r["rho_"+k] for r in entries]))
                    for k in ("weak1", "weak2", "weak3", "strong2")})
        row.update(weak2_minus_strong2=row["R_weak2"]-row["R_strong2"],
                   weak2_eigenpair_rank=rank_of(values, 0),
                   weak2_gt_strong2=int(row["R_weak2"] > row["R_strong2"]))
        frame_rows.append(row)
    matrix = np.array([[r["R_weak1"], r["R_weak2"], r["R_weak3"], r["R_strong2"]] for r in frame_rows])
    difference = matrix[:, 1] - matrix[:, 3]
    pair_values = np.array(pair_values)
    pair_macro = pair_values.mean(axis=0)
    pair_rows = []
    for i, pair in enumerate(pairs):
        row = dict(pair=f"q{pair[0]+1}_q{pair[1]+1}", T_pair=float(pair_macro[i]),
                   overall_rank=rank_of(pair_macro, i))
        row.update({f"R_{tx}": float(pair_values[j, i]) for j, tx in enumerate(FRAMES)})
        pair_rows.append(row)
    nulls, null_summary = {}, []
    for k in (1, 2, 3):
        rng = np.random.Generator(np.random.PCG64(SEEDS[f"haar{k}"]))
        frame_nulls = []
        for tx in FRAMES:
            q = haar_subspaces(rng, REPLICATES, k)
            projections = np.einsum('bi,rik->rbk', np.array(normalized[tx]), q)
            frame_nulls.append(np.sum(projections**2, axis=2).mean(axis=1))
        null = np.mean(frame_nulls, axis=0)
        nulls[k] = null
        observed = float(matrix[:, k-1].mean())
        null_summary.append(dict(k=k, replicates=REPLICATES, seed=SEEDS[f"haar{k}"], rng="PCG64",
                                 mean=float(null.mean()), median=float(np.median(null)),
                                 p95=float(np.percentile(null, 95)), p99=float(np.percentile(null, 99)),
                                 max=float(null.max()), observed=observed, random_baseline=k/6,
                                 excess_over_random=observed-k/6,
                                 p_one_sided=float((1+np.sum(null >= observed))/(REPLICATES+1))))
    indices = np.random.Generator(np.random.PCG64(SEEDS["frame_bootstrap"])).integers(
        0, len(FRAMES), size=(REPLICATES, len(FRAMES)))
    boot = np.column_stack((matrix[indices, 1].mean(axis=1), matrix[indices, 3].mean(axis=1),
                            difference[indices].mean(axis=1)))
    intervals = np.percentile(boot, [2.5, 97.5], axis=0).T
    bootstrap_rows = [dict(replicate=i, frame_indices=";".join(map(str, idx)),
                           T_weak2=float(v[0]), T_strong2=float(v[1]), difference=float(v[2]))
                      for i, (idx, v) in enumerate(zip(indices, boot))]
    lofo = [dict(excluded_frame=tx, T_weak2=float(np.delete(matrix[:, 1], i).mean()),
                 T_strong2=float(np.delete(matrix[:, 3], i).mean()),
                 difference=float(np.delete(difference, i).mean())) for i, tx in enumerate(FRAMES)]
    main = dict(T_weak2=float(matrix[:, 1].mean()), weak2_ci95=intervals[0].tolist(),
                T_strong2=float(matrix[:, 3].mean()), strong2_ci95=intervals[1].tolist(),
                difference=float(difference.mean()), difference_ci95=intervals[2].tolist(),
                frames_weak_gt_strong=int(np.sum(difference > 0)), exact_sign_flip_p=exact_sign_flip(difference))
    conditions = dict(A_historical_parity=True, B_haar_p=null_summary[1]["p_one_sided"] < .01,
                      C_weak_gt_strong=main["difference"] > 0,
                      D_exact_p=main["exact_sign_flip_p"] < .05,
                      E_ci_lower=main["difference_ci95"][0] > 0,
                      F_frames=main["frames_weak_gt_strong"] >= 7,
                      G_pair_rank=rank_of(pair_macro, 0) <= 3)
    passed = all(conditions.values())
    if passed:
        final, nxt = "WEAK_SUBSPACE_CONCENTRATION_SUPPORTED", "H2_MATCHED_BUDGET_BASIN_DISCOVERY_EFFICIENCY"
    elif all(v for k, v in conditions.items() if k != "G_pair_rank"):
        final, nxt = "WEAK_DIRECTION_ADVANTAGE_BUT_NOT_EIGENPAIR_SPECIFIC", "REVIEW_UOBS_EIGENSPACE_SELECTION_RULE"
    else:
        final, nxt = "WEAK_SUBSPACE_CONCENTRATION_NOT_SUPPORTED", "STOP_WEAK_GUIDED_SEARCH_AS_PRIMARY_UNONLOCAL_IMPLEMENTATION_CANDIDATE"
    result = dict(main=main, frame_stats=frame_rows, haar_null=null_summary,
                  eigenpair=dict(weak2_overall_rank=rank_of(pair_macro, 0),
                                 frame_top1_count=sum(r["weak2_eigenpair_rank"] == 1 for r in frame_rows),
                                 frame_top3_count=sum(r["weak2_eigenpair_rank"] <= 3 for r in frame_rows),
                                 top5=sorted(pair_rows, key=lambda r: -r["T_pair"])[:5]),
                  lofo=dict(weak_gt_strong_count=sum(r["difference"] > 0 for r in lofo),
                            min_difference=min(r["difference"] for r in lofo),
                            max_difference=max(r["difference"] for r in lofo)),
                  conditions=conditions, h1="PASS" if passed else "FAIL", final_result=final, next=nxt)
    files = {"major_projection_statistics.csv": major, "frame_projection_statistics.csv": frame_rows,
             "eigenpair_statistics.csv": pair_rows, "haar_null_summary.csv": null_summary,
             "haar_null.csv": [dict(replicate=i, **{f"T_random{k}": float(nulls[k][i]) for k in (1, 2, 3)})
                               for i in range(REPLICATES)],
             "bootstrap.csv": bootstrap_rows, "lofo.csv": lofo}
    return result, files


def write_csv(path, rows):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader(); writer.writerows(rows)


def audit(output, chart):
    result = json.loads((output / "results.json").read_text())
    for key in ("input_sha256", "source_sha256"):
        for path, expected in result[key].items():
            actual = Path(path) if key == "input_sha256" else ROOT / path
            require(digest(actual) == expected, "hash audit: " + path)
    for name, expected in result["sidecar_sha256"].items():
        require(digest(output / name) == expected, "sidecar hash audit: " + name)
    require(digest(output / "THEORY.md") == result["theory_sha256"], "theory hash audit")
    require(digest(output / "REPORT.md") == result["report_sha256"], "report hash audit")
    require(result["chart_build"]["source_sha256"] == result["source_sha256"][result["chart_build"]["source"]],
            "binary-source build provenance audit")
    major, normalized, bases, parity, _ = load_inputs(chart)
    saved_major = read_csv(output / "major_projection_statistics.csv")
    require(len(saved_major) == 22, "major count audit")
    for expected, saved in zip(major, saved_major):
        for key, value in expected.items():
            if isinstance(value, str):
                require(value == saved[key], "major label audit")
            else:
                np.testing.assert_allclose(float(saved[key]), value, atol=1e-12, rtol=1e-12)
    require(result["retained_iteration_limit_basins"] == ["616/P03", "3796/P06"], "iteration-limit retention audit")
    frames = read_csv(output / "frame_projection_statistics.csv")
    require([int(r["frame"]) for r in frames] == list(FRAMES), "frame order audit")
    for row in frames:
        entries = [m for m in major if m["tx"] == int(row["frame"])]
        require(int(row["major_count"]) == len(entries), "frame count audit")
        for key in ("weak1", "weak2", "weak3", "strong2"):
            np.testing.assert_allclose(float(row["R_"+key]), np.mean([m["rho_"+key] for m in entries]), atol=1e-12)
    weak = np.array([float(r["R_weak2"]) for r in frames])
    strong = np.array([float(r["R_strong2"]) for r in frames])
    np.testing.assert_allclose([weak.mean(), strong.mean(), (weak-strong).mean()],
                               [result["main"]["T_weak2"], result["main"]["T_strong2"], result["main"]["difference"]], atol=1e-12)
    require(exact_sign_flip(weak-strong) == result["main"]["exact_sign_flip_p"], "exact p audit")
    pairs = read_csv(output / "eigenpair_statistics.csv")
    require(len(pairs) == 15, "pair count audit")
    for row, pair in zip(pairs, itertools.combinations(range(6), 2)):
        values = []
        for tx in FRAMES:
            directions = np.array(normalized[tx])
            value = np.square(directions @ bases[tx][:, pair]).sum(axis=1).mean()
            np.testing.assert_allclose(float(row[f"R_{tx}"]), value, atol=1e-12)
            values.append(value)
        np.testing.assert_allclose(float(row["T_pair"]), np.mean(values), atol=1e-12)
    require(rank_of(np.array([float(r["T_pair"]) for r in pairs]), 0) == result["eigenpair"]["weak2_overall_rank"], "rank audit")
    null = read_csv(output / "haar_null.csv")
    require(len(null) == REPLICATES, "null replicate count audit")
    for summary in result["haar_null"]:
        values = np.array([float(r[f"T_random{summary['k']}"]) for r in null])
        expected = [values.mean(), np.median(values), *np.percentile(values, [95, 99]), values.max()]
        np.testing.assert_allclose([summary[k] for k in ("mean", "median", "p95", "p99", "max")], expected, atol=1e-12)
        p = (1 + np.sum(values >= summary["observed"])) / (REPLICATES + 1)
        require(p == summary["p_one_sided"], "Monte-Carlo p audit")
    bootstrap = read_csv(output / "bootstrap.csv")
    require(len(bootstrap) == REPLICATES, "bootstrap replicate count audit")
    computed = []
    for row in bootstrap:
        indices = np.array([int(i) for i in row["frame_indices"].split(";")])
        require(len(indices) == 9 and np.all((indices >= 0) & (indices < 9)), "bootstrap FRAME indices audit")
        triple = [weak[indices].mean(), strong[indices].mean(), (weak-strong)[indices].mean()]
        np.testing.assert_allclose([float(row[k]) for k in ("T_weak2", "T_strong2", "difference")], triple, atol=1e-12)
        computed.append(triple)
    np.testing.assert_allclose(np.percentile(computed, [2.5, 97.5], axis=0).T,
                               [result["main"][k] for k in ("weak2_ci95", "strong2_ci95", "difference_ci95")], atol=1e-12)
    lofo = read_csv(output / "lofo.csv")
    require(len(lofo) == 9, "LOFO count audit")
    for i, row in enumerate(lofo):
        require(int(row["excluded_frame"]) == FRAMES[i], "LOFO identity audit")
        np.testing.assert_allclose(float(row["difference"]), np.delete(weak-strong, i).mean(), atol=1e-12)
    require(result["new_ndt_calls"] == 0 and result["gt_used"] is False and result["h2_run"] is False, "scope audit")
    return "PASS"


def report_text(result):
    p, m, e = result["parity"], result["main"], result["eigenpair"]
    lines = ["# H1: weak-subspace concentration closure", "", result["final_result"], "",
             "This result supports a geometric association only in the frozen nine-frame oracle cohort; it does not establish search efficiency or a global NDT law.", "",
             "## Historical parity", "",
             f"22 basins / 9 frames. Fraction mean {p['reproduced_mean']:.12f}, median {p['reproduced_median']:.12f}, >=0.8: 13. Max per-basin error {p['max_per_basin_error']:.12g}; gate PASS.", "",
             "Original representative matrices are used. Both iteration-limit basins 616/P03 and 3796/P06 are retained. No reclustering or new optimization was performed.", "",
             "## Frame-macro results", "",
             "| tx | major IDs | n | W1 | W2 | W3 | S2 | difference | W2 pair rank |",
             "|---|---|---:|---:|---:|---:|---:|---:|---:|"]
    for row in result["frame_stats"]:
        lines.append(f"| {row['frame']} | {row['major_ids']} | {row['major_count']} | " +
                     " | ".join(f"{row[k]:.9f}" for k in ("R_weak1", "R_weak2", "R_weak3", "R_strong2", "weak2_minus_strong2")) +
                     f" | {row['weak2_eigenpair_rank']} |")
    lines += ["", f"T_weak2={m['T_weak2']:.12f}, CI95={m['weak2_ci95']}.",
              f"T_strong2={m['T_strong2']:.12f}, CI95={m['strong2_ci95']}.",
              f"Difference={m['difference']:.12f}, CI95={m['difference_ci95']}.",
              f"Weak>strong: {m['frames_weak_gt_strong']}/9; exact sign-flip p={m['exact_sign_flip_p']:.12g} (512 patterns).", "",
              "## Random-subspace null and dimension ablation", "",
              "| k | T_weak | k/6 | excess | null mean | null median | null95 | null99 | null max | p |",
              "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for row in result["haar_null"]:
        lines.append(f"| {row['k']} | " + " | ".join(f"{row[k]:.9f}" for k in
                     ("observed", "random_baseline", "excess_over_random", "mean", "median", "p95", "p99", "max", "p_one_sided")) + " |")
    lines += ["", "Each null has 10,000 replicates and independent subspaces per frame; PCG64 seeds and software versions are recorded in results.json. Frame bootstrap also has 10,000 replicates; all frame draws are stored in bootstrap.csv.", "",
              f"WEAK2 overall eigenpair rank: {e['weak2_overall_rank']}/15; frame top1 {e['frame_top1_count']}/9, top3 {e['frame_top3_count']}/9.", "",
              "Top5 eigenpairs: " + "; ".join(f"{r['pair']}={r['T_pair']:.9f}" for r in e["top5"]) + ".", "",
              f"LOFO weak>strong: {result['lofo']['weak_gt_strong_count']}/9; difference range [{result['lofo']['min_difference']:.12f}, {result['lofo']['max_difference']:.12f}].", "",
              "## Acceptance and limitations", "", f"A-G conditions: {result['conditions']}; H1={result['h1']}.", "",
              "WEAK2 is not the highest-projection pair in every frame: TX368 rank6, TX2722 rank8, TX2846 rank5. The whole cohort is historically selected and failure-enriched; temporal independence and generalization to unrelated NDT scenes are not established. Read THEORY.md for the conditional-null and resampling boundaries.", "",
              "NEW_NDT_CALLS=0; GT_USED=NO; H2_RUN=NO. No production core or old archive files were modified.", "",
              "## Reproduction", "", "```bash",
              "PYTHONDONTWRITEBYTECODE=1 python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/run_h1_concentration.py --self-test",
              "PYTHONDONTWRITEBYTECODE=1 python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/run_h1_concentration.py",
              "PYTHONDONTWRITEBYTECODE=1 python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/run_h1_concentration.py --audit",
              "```", "", "Source/input/CSV/theory/report hashes are recorded in results.json. The C++ helper has only Eigen dependencies and exposes no registration interface.", "",
              "The runner builds its helper from the recorded source in a fresh /tmp directory; arbitrary prebuilt libraries are not accepted. Failed runs replace the current JSON/report with failure state; old CSVs are preserved but explicitly invalidated. The weighting self-test invokes the actual statistics() path on a synthetic duplicated-basin fixture.", "",
              "NEXT=" + result["next"], ""]
    return "\n".join(lines)


def run(args, chart, build):
    major, normalized, bases, parity, hashes = load_inputs(chart)
    print(json.dumps(parity, indent=2))
    require(parity["pass"], "H1_HISTORICAL_PROJECTION_PARITY_FAIL")
    if args.parity_only:
        return
    require(args.output is not None, "output required for statistics")
    require(args.output.resolve() == OUTPUT,
            "refusing to write outside the dedicated H1 archive")
    result, files = statistics(major, normalized, bases)
    args.output.mkdir(parents=True, exist_ok=True)
    for name, rows in files.items():
        write_csv(args.output / name, rows)
    result.update(task="PAPER-P9-FOUNDATION-H1-WEAK-SUBSPACE-CONCENTRATION-CLOSURE",
                  branch=subprocess.check_output(["git", "branch", "--show-current"], cwd=ROOT, text=True).strip(),
                  start_sha=START_SHA, parity=parity, rng_type="PCG64", seeds=SEEDS,
                  replicates=REPLICATES, statistical_unit="FRAME", frame_order=list(FRAMES),
                  new_ndt_calls=0, gt_used=False, h2_run=False, self_test="PASS",
                  retained_iteration_limit_basins=[f"{r['tx']}/{r['cluster_id']}" for r in major if r["iteration_limit_flag"]],
                  input_sha256=hashes,
                  source_sha256={str(p.relative_to(ROOT)): digest(p) for p in
                                 [Path(__file__).resolve(), Path(__file__).resolve().with_name("p9_h1_chart.cpp"),
                                  Path(__file__).resolve().with_name("p9_ndt_energy_contract.cpp")]},
                  sidecar_sha256={name: digest(args.output / name) for name in files},
                  theory_sha256=digest(args.output / "THEORY.md"),
                  chart_build=build, current_run_state="COMPLETE", statistical_sidecars_current=True,
                  versions=dict(python=sys.version, numpy=np.__version__))
    (args.output / "REPORT.md").write_text(report_text(result))
    result["report_sha256"] = digest(args.output / "REPORT.md")
    (args.output / "results.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps({k: result[k] for k in ("main", "eigenpair", "lofo", "h1", "final_result")}, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--parity-only", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--audit", action="store_true")
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    require(args.output.resolve() == OUTPUT, "dedicated H1 output required")
    try:
        with tempfile.TemporaryDirectory(prefix="p9-h1-build-", dir="/tmp") as directory:
            library, build = build_chart(directory)
            chart = chart_function(library)
            print("SELF_TEST=" + self_test(chart))
            if args.audit:
                print("CSV_JSON_HASH_AUDIT=" + audit(args.output, chart))
            elif not args.self_test:
                run(args, chart, build)
    except Exception as error:
        if not args.audit and not args.self_test:
            failed_run(args.output, str(error))
        raise


if __name__ == "__main__":
    main()
