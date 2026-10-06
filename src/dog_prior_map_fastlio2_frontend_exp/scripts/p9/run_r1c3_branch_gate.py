#!/usr/bin/env python3
"""R1C3: frozen H2 seed audit; oracle weak path only enters the engine."""
import argparse
import csv
import hashlib
import json
import subprocess
from pathlib import Path

import numpy as np

from run_h1_concentration import ARCHIVE, ROOT, digest, read_csv, require, vector
from run_h2_discovery import CANONICAL, ENV, MAP, text, write_csv

START = "251cc8d6654735ea034b820fd4058082ad310493"
BRANCH = "research/p9-r1c3-secondgen-branch-gate"
OUT = ROOT / "docs/p9_r1c3_secondgen_branch_gate"
H2 = ROOT / "docs/p9_foundation_weak_discovery/h2"
CASES = [(2226, "P05"), (2350, "P01"), (2350, "P05"),
         (3341, "P02"), (3341, "P03"), (2722, "P05"),
         (616, "P10"), (3796, "P06")]
SOURCE = Path(__file__).resolve().parent


def historical_build_lineage():
    path = SOURCE / "CMakeLists.txt"
    expected = json.loads((H2 / "results.json").read_text())["execution_manifest"]["input_sha256"][str(path)]
    content = subprocess.check_output(["git", "show", START + ":" + str(path.relative_to(ROOT))], cwd=ROOT)
    require(hashlib.sha256(content).hexdigest() == expected, "historical H2 build lineage mismatch")
    return dict(commit=START, path=str(path.relative_to(ROOT)), sha256=expected)


def inputs():
    branch = subprocess.check_output(["git", "branch", "--show-current"], cwd=ROOT, text=True).strip()
    require(branch == BRANCH, "wrong R1C3 branch")
    subprocess.run(["git", "merge-base", "--is-ancestor", START, "HEAD"], cwd=ROOT, check=True)
    h2 = json.loads((H2 / "results.json").read_text())
    require(h2["current_run_state"] == "COMPLETE" and h2["h2"] == "FAIL", "H2 provenance")
    hashes = dict(h2["execution_manifest"]["input_sha256"])
    # The authorized new target changes the current build recipe, not the H2 archive.
    historical_build_lineage()
    del hashes[str(SOURCE / "CMakeLists.txt")]
    for name, sha in h2["sidecar_sha256"].items():
        hashes[str(H2 / name)] = sha
    hashes[str(H2 / "results.json")] = digest(H2 / "results.json")
    for path, sha in hashes.items():
        require(digest(Path(path)) == sha, "frozen input changed: " + path)
    for tx in {tx for tx, _ in CASES}:
        row = next(r for r in read_csv(ARCHIVE / "frozen/cohort_frozen.csv") if int(r["transaction_id"]) == tx)
        raw = Path(row["raw_cloud_file"])
        require(digest(raw) == row["raw_source_sha256"], "raw source changed")
        hashes[str(raw)] = digest(raw)
    return hashes


def prepare():
    require(not (OUT / "branch_nodes.csv").exists(), "existing branch experiment preserved")
    hashes = inputs()
    obs = {int(r["transaction_id"]): r for r in read_csv(ARCHIVE / "dual_u.csv")}
    major = {(int(r["tx"]), r["cluster_id"]): r for r in
             read_csv(ROOT / "docs/p9_foundation_weak_discovery/h1/major_projection_statistics.csv")}
    canonical = {(int(r["transaction_id"]), r["cluster_id"]): r for r in read_csv(CANONICAL)}
    proposals = {(int(r["frame"]), int(r["seed_index"])): r for r in read_csv(H2 / "proposal_contract.csv")
                 if r["method"] == "FULL6D"}
    admissions = {(int(r["frame"]), r["method"], int(r["seed_index"])): r
                  for r in read_csv(H2 / "terminal_admission.csv") if r["method"] in ("FULL6D", "WEAK2")}
    requests, metadata, rows, summaries = [], [], [], []
    for index, key in enumerate(CASES):
        tx, cluster = key
        q = vector(obs[tx]["curvature_eigenvectors_rowmajor"], 36).reshape(6, 6)
        require(np.linalg.norm(q.T @ q - np.eye(6)) < 1e-12, "basis contract")
        d = np.array([float(major[key][f"delta_{i}"]) for i in range(6)])
        u, v = q[:, :2].T @ d, q[:, 2:].T @ d
        require(np.linalg.norm(q[:, :2] @ u + q[:, 2:] @ v - d) < 1e-12, "chart reconstruction")
        # Only u_b, not canonical pose/v/support, is passed to continuation.
        requests.append(dict(case_index=index, tx=tx, cluster=cluster, u_b=text(u)))
        metadata.append(dict(tx=tx, cluster=cluster, rho_W2=float(major[key]["rho_weak2"]),
                             u_b=text(u), v_b_offline_only=text(v), delta_b=text(d),
                             canonical_pose=canonical[key]["canonical_pose_matrix16"]))
        successful = []
        for seed in range(263):
            full, weak = admissions[tx, "FULL6D", seed], admissions[tx, "WEAK2", seed]
            if full["converged"] != "1" or full["recovered_cluster"] != cluster:
                continue
            eta = vector(proposals[tx, seed]["eta_original"], 6)
            uj, vj = q[:, :2].T @ eta, q[:, 2:].T @ eta
            row = dict(tx=tx, cluster=cluster, seed_index=seed, eta=text(eta), u_j=text(uj), v_j=text(vj),
                       u_norm=float(np.linalg.norm(uj)), v_norm=float(np.linalg.norm(vj)),
                       strong_fraction=float(np.linalg.norm(q[:, 2:] @ vj) / np.linalg.norm(eta)) if np.linalg.norm(eta) else 0.0,
                       full_eligible_clusters=full["eligible_clusters"],
                       same_index_weak_recovery=int(weak["recovered_cluster"] == cluster),
                       same_index_weak_eligible=int(cluster in weak["eligible_clusters"].split(";")))
            rows.append(row); successful.append(row)
        require(successful, "no FULL recovery of frozen case")
        weak_count = sum(admissions[tx, "WEAK2", seed]["recovered_cluster"] == cluster for seed in range(263))
        require((weak_count == 0) == (index < 6), "H2 missed/control identity changed")
        summaries.append(dict(tx=tx, cluster=cluster, full_successful_seeds=len(successful),
                              median_u=float(np.median([r["u_norm"] for r in successful])),
                              median_v=float(np.median([r["v_norm"] for r in successful])),
                              median_strong_fraction=float(np.median([r["strong_fraction"] for r in successful])),
                              same_index_weak_recovery=sum(r["same_index_weak_recovery"] for r in successful)))
    OUT.mkdir(parents=True, exist_ok=True)
    write_csv(OUT / "success_seed_strong_component.csv", rows)
    write_csv(OUT / "success_seed_summary.csv", summaries)
    write_csv(OUT / "weak_path_requests.csv", requests)
    write_csv(OUT / "case_metadata_offline_only.csv", metadata)
    (OUT / "input_manifest.json").write_text(json.dumps(dict(start_sha=START, input_sha256=hashes,
        historical_build_lineage=historical_build_lineage(),
        cases=CASES, engine_oracle_inputs="u_b ONLY", new_ndt_calls=0,
        prepared_sha256={name: digest(OUT / name) for name in ("success_seed_strong_component.csv",
                         "success_seed_summary.csv", "weak_path_requests.csv", "case_metadata_offline_only.csv")}), indent=2) + "\n")
    print(json.dumps(summaries, indent=2))
    print("R1C3_SUCCESS_SEED_AUDIT=PASS NEW_NDT_CALLS=0")


def verify_execution():
    manifest = json.loads((OUT / "execution_manifest.json").read_text())
    require(manifest["historical_build_lineage"] == historical_build_lineage(), "historical lineage changed")
    frozen = dict(manifest["input_sha256"])
    frozen.update(manifest["source_sha256"])
    frozen.update({str(OUT / name): sha for name, sha in manifest["prepared_sha256"].items()})
    frozen[manifest["binary"]] = manifest["binary_sha256"]
    for path, sha in frozen.items():
        require(digest(Path(path)) == sha, "execution hash changed: " + path)
    return manifest


def run(binary):
    # Do not overwrite previous attempts or repeat any final refine.
    require(not (OUT / "branch_nodes.csv").exists() and not (OUT / "final_refine.csv").exists(),
            "existing branch outputs preserved; repeat prohibited")
    prepared = json.loads((OUT / "input_manifest.json").read_text())
    require(inputs() == prepared["input_sha256"], "prepared input lineage changed")
    for name, sha in prepared["prepared_sha256"].items():
        require(digest(OUT / name) == sha, "prepared request/data changed")
    cache = binary.parent / "CMakeCache.txt"
    require("CMAKE_BUILD_TYPE:STRING=Release" in cache.read_text(), "Release build required")
    names = ["p9_secondgen_branch_gate.cpp", "p9_numeric_branch_certificate.hpp", "p9_stationarity_numerics.hpp",
             "p9_true_profile_closure.cpp", "p9_ndt_energy_contract.cpp", "p9_strong_profile_solvers.hpp",
             "run_r1c3_branch_gate.py", "summarize_r1c3_branch_gate.py", "run_numerical_stationarity.py", "CMakeLists.txt"]
    paths = [SOURCE / n for n in names] + [OUT / "THEORY.md",
        Path("/usr/include/pcl-1.10/pcl/registration/impl/ndt.hpp"),
        Path("/usr/include/pcl-1.10/pcl/filters/impl/voxel_grid_covariance.hpp")]
    manifest = dict(prepared, branch=BRANCH, execution_head=subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        source_sha256={str(p): digest(p) for p in paths}, binary=str(binary), binary_sha256=digest(binary),
        build_cache_sha256=digest(cache), ldd=subprocess.check_output(["ldd", str(binary)], env=ENV, text=True),
        environment={k: ENV[k] for k in ("LD_LIBRARY_PATH", "OMP_NUM_THREADS", "PYTHONDONTWRITEBYTECODE")},
        engine_inputs=[str(MAP), str(ARCHIVE / "frozen/cohort_frozen.csv"), str(ARCHIVE / "dual_u.csv"),
                       str(OUT / "weak_path_requests.csv")])
    (OUT / "execution_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    verify_execution()
    try:
        subprocess.run([str(binary), "--run", *manifest["engine_inputs"], str(OUT)], env=ENV, check=True)
        verify_execution()
        nodes = read_csv(OUT / "branch_nodes.csv")
        final = [r for r in nodes if r["certified"] == r["accepted"] == "1" and abs(float(r["alpha"]) - 1) < 1e-12]
        if final:
            write_csv(OUT / "final_refine_requests.csv", final)
            subprocess.run([str(binary), "--refine", str(MAP), str(ARCHIVE / "frozen/cohort_frozen.csv"),
                            str(OUT / "final_refine_requests.csv"), str(OUT / "final_refine.csv")], env=ENV, check=True)
        else:
            with (OUT / "final_refine.csv").open("w", newline="") as stream:
                csv.writer(stream, lineterminator="\n").writerow(["tx", "cluster", "branch_id", "pre_pose_matrix16",
                    "post_pose_matrix16", "iterations", "converged", "status", "runtime_ms",
                    "pre_support_hash", "post_support_hash", "support_change"])
        verify_execution()
        (OUT / "run_completed.json").write_text(json.dumps(dict(engine_completed=True,
            full_ndt_calls_inside_continuation=0, final_diagnostic_calls=len(final))) + "\n")
        print("R1C3_ENGINE_COMPLETE FINAL_DIAGNOSTIC_NDT_CALLS=" + str(len(final)))
    except Exception as exc:
        (OUT / "results.json").write_text(json.dumps(dict(current_run_state="FAILED", error=str(exc),
            scientific_result="NOT_EVALUATED", partial_csvs_not_current=True), indent=2) + "\n")
        raise


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("stage", choices=["prepare", "run", "analyze", "audit", "self-test"])
    parser.add_argument("--binary", type=Path, default=Path("/tmp/p9_r1c3_build/p9_secondgen_branch_gate"))
    args = parser.parse_args()
    if args.stage == "prepare":
        prepare()
    elif args.stage == "run":
        run(args.binary.resolve())
    else:
        from summarize_r1c3_branch_gate import analyze, audit, self_test
        {"analyze": analyze, "audit": audit, "self-test": self_test}[args.stage]()


if __name__ == "__main__":
    main()
