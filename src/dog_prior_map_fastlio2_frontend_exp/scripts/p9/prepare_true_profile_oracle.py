#!/usr/bin/env python3
"""Reproduce frozen complete-link membership and select real oracle terminals.

This reads only optimizer evidence. The 22 target IDs are frozen by P9-R1;
reclustering verifies their membership instead of assigning by proximity.
"""

import argparse
import csv
import hashlib
import json
import subprocess
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


CLOSURE_SHA = "9945c4f5c3d7759104de108a594bcaf2553fd78c"
CLUSTER_SOURCE = "src/dog_prior_map_fastlio2_frontend_exp/scripts/p5_i1_cluster_modes.py"
EXPECTED_CLUSTER_SHA256 = "13dd0a7fe9fe87506f1f3728a81a6c81faa5afbbd0789cc4e8fc4cc89fa38805"


def read_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def digest(path):
    sha = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            sha.update(block)
    return sha.hexdigest()


def write_csv(path, rows):
    if not rows:
        raise RuntimeError(f"empty output: {path}")
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def rigid_distance(a, b):
    # Same atan2/quaternion geodesic convention as the P9 C++ report path.
    ra, rb = Rotation.from_matrix(a[:3, :3]), Rotation.from_matrix(b[:3, :3])
    return float(np.linalg.norm(a[:3, 3] - b[:3, 3])), float((ra.inv() * rb).magnitude() * 180 / np.pi)


def run(args):
    archive = Path(args.archive)
    candidate_path = archive / "candidates.csv"
    cluster_path = archive / "clusters/mode_clusters.csv"
    cohort_path = archive / "frozen/cohort_frozen.csv"
    provenance = json.loads((archive / "frozen/objective_provenance.json").read_text())
    if provenance.get("gt_accessed") is not False:
        raise RuntimeError("oracle search does not declare GT-blind generation")
    source = subprocess.check_output(["git", "show", f"{CLOSURE_SHA}:{CLUSTER_SOURCE}"])
    source_digest = hashlib.sha256(source).hexdigest()
    if source_digest != EXPECTED_CLUSTER_SHA256:
        raise RuntimeError("historical clustering implementation digest changed")
    namespace = {"__name__": "p9_frozen_clusterer"}
    exec(compile(source, f"git:{CLOSURE_SHA}:{CLUSTER_SOURCE}", "exec"), namespace)

    candidates = read_csv(candidate_path)
    frozen = {int(row["transaction_id"]): row for row in read_csv(cohort_path)}
    clusters = read_csv(cluster_path)
    prior = read_csv(args.r1_recovery)
    target_ids = {(int(row["transaction_id"]), cluster_id)
                  for row in prior for cluster_id in row["missed_major_cluster_ids"].split(";")
                  if cluster_id not in ("", "NONE")}
    if len(target_ids) != 22 or any(int(row["recovered_major_clusters"]) != 0 for row in prior):
        raise RuntimeError("P9-R1 target oracle set is not the frozen 22 missed basins")
    by_tx = defaultdict(list)
    for row in candidates:
        tx = int(row["transaction_id"])
        if tx in {key[0] for key in target_ids}:
            by_tx[tx].append(row)
    primary = {(int(row["transaction_id"]), row["cluster_id"]): row for row in clusters
               if row["threshold_set"] == "primary"}
    requests, membership = [], []
    verified_primary = 0
    for tx in sorted(by_tx):
        frame = frozen[tx]
        runs = [row for row in by_tx[tx] if int(row["converged"]) == 1]
        if len({int(row["seed_index"]) for row in runs}) != len(runs):
            raise RuntimeError(f"duplicate seed at tx{tx}")
        for row in runs:
            if row["source_hash_actual"] != frame["prepared_source_hash"] or \
                    row["source_hash_expected"] != frame["prepared_source_hash"]:
                raise RuntimeError(f"source provenance mismatch at tx{tx}")
        groups = namespace["connected_clusters"](runs, 0.20, 2.0)
        # The historical cluster order is minimum seed index, followed by stable
        # descending best score. Reproduce this exact ordering and tie behavior.
        groups.sort(key=lambda entries: -max(float(row["raw_ndt_score_sum"]) for row, _ in entries))
        for rank, group in enumerate(groups, 1):
            cluster_id = f"P{rank:02d}"
            saved = primary[(tx, cluster_id)]
            best = max((row for row, _ in group), key=lambda row: float(row["raw_ndt_score_sum"]))
            if saved["representative_pose_matrix16"] != best["final_pose_matrix16"] or \
                    abs(float(saved["best_score"]) - float(best["raw_ndt_score_sum"])) > 1e-10 or \
                    int(saved["seed_count"]) != len(group):
                raise RuntimeError(f"archive cluster reproduction failed at tx{tx}/{cluster_id}")
            verified_primary += 1
            if (tx, cluster_id) not in target_ids:
                continue
            if saved["stable_mode_candidate"] != "1":
                raise RuntimeError("major target is not supported")
            rep = namespace["pose"](saved["representative_pose_matrix16"])
            canonical = namespace["pose"](best["final_pose_matrix16"])
            dt, dr = rigid_distance(rep, canonical)
            seed_ids = [str(int(row["seed_index"])) for row, _ in group]
            requests.append({
                "transaction_id": tx, "frame_id": frame["frame_id"], "cluster_id": cluster_id,
                "seed_count": len(group), "canonical_seed_index": int(best["seed_index"]),
                "member_seed_indices": ";".join(seed_ids),
                "representative_pose_matrix16": saved["representative_pose_matrix16"],
                "canonical_pose_matrix16": best["final_pose_matrix16"],
                "representative_score": float(saved["representative_score"]),
                "canonical_score": float(best["raw_ndt_score_sum"]),
                "representative_canonical_translation_m": dt,
                "representative_canonical_rotation_deg": dr,
                "representative_canonical_mean_energy_difference":
                    (float(saved["representative_score"]) - float(best["raw_ndt_score_sum"])) /
                    int(frame["prepared_source_point_count"]),
                "selection": "FROZEN_P9_R1_MAJOR_ORACLE_ID",
            })
            for row, _ in group:
                membership.append({"transaction_id": tx, "cluster_id": cluster_id,
                                   "seed_index": int(row["seed_index"]),
                                   "raw_ndt_score_sum": float(row["raw_ndt_score_sum"]),
                                   "is_canonical": int(row is best)})
    if len(requests) != 22:
        raise RuntimeError("not all 22 target clusters canonicalized")
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    write_csv(out / "oracle_terminal_requests.csv", requests)
    write_csv(out / "oracle_membership.csv", membership)
    summary = {
        "major_basins": 22, "canonicalized": len(requests),
        "verified_primary_clusters_in_target_frames": verified_primary,
        "canonical_rule": "maximum raw_ndt_score_sum within exactly reconstructed complete-link membership",
        "representative_is_real_best_terminal": True,
        "representative_contract_pass": True,
        "representative_over_0p05m_0p5deg": sum(
            row["representative_canonical_translation_m"] > .05 or row["representative_canonical_rotation_deg"] > .5
            for row in requests),
        "representative_over_0p10m_1deg": sum(
            row["representative_canonical_translation_m"] > .1 or row["representative_canonical_rotation_deg"] > 1
            for row in requests),
        "representative_over_0p20m_2deg": sum(
            row["representative_canonical_translation_m"] > .2 or row["representative_canonical_rotation_deg"] > 2
            for row in requests),
        "clustering_git_sha": CLOSURE_SHA, "clustering_source": CLUSTER_SOURCE,
        "clustering_source_sha256": source_digest,
        "gt_accessed": False,
        "inputs_sha256": {str(path): digest(path) for path in
                          [candidate_path, cluster_path, cohort_path, Path(args.r1_recovery)]},
    }
    (out / "oracle_preparation.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", required=True)
    parser.add_argument("--r1-recovery", required=True)
    parser.add_argument("--output", required=True)
    run(parser.parse_args())
