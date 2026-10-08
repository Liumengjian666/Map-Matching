#!/usr/bin/env python3
"""Isolated BASE263 label extractor; never imported by R4 evidence builders."""
import argparse
import ctypes
from collections import defaultdict
import hashlib
import json
import subprocess

import numpy as np

import p9_r4_contract as c

CLUSTER_PATH = "src/dog_prior_map_fastlio2_frontend_exp/scripts/p5_i1_cluster_modes.py"
CLUSTER_HASH = "13dd0a7fe9fe87506f1f3728a81a6c81faa5afbbd0789cc4e8fc4cc89fa38805"


class FrozenGeometry:
    def __init__(self, library):
        self.path = library
        self.lib = ctypes.CDLL(str(library))
        self.fn = self.lib.p9_r4_oracle_distances
        self.fn.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.POINTER(ctypes.c_double)]
        self.fn.restype = ctypes.c_int

    def distances(self, nominal_xyzq, candidate16):
        output = (ctypes.c_double * 2)()
        c.require(self.fn(nominal_xyzq.encode(), candidate16.encode(), output) == 0,
                  "invalid frozen oracle geometry")
        return output[0], output[1]


def historical_clusterer():
    source = subprocess.check_output(["git", "show", c.HISTORY_SHA + ":" + CLUSTER_PATH], cwd=c.ROOT)
    c.require(hashlib.sha256(source).hexdigest() == CLUSTER_HASH, "historical complete-link source changed")
    namespace = {"__name__": "r4_frozen_oracle_clusterer"}
    exec(compile(source, "git:" + c.HISTORY_SHA + ":" + CLUSTER_PATH, "exec"), namespace)
    return namespace


def major_clusters(primary, nominal_text, source_points, geometry):
    """Literal P9-R1 definition, including its per-source max(1,...) floor."""
    supported = [row for row in primary if int(row["stable_mode_candidate"]) == 1]
    c.require(supported, "no supported oracle primary cluster")
    c.require(source_points > 0, "invalid source point denominator")
    best = max(float(row["best_score"]) for row in supported)
    energy_gap_limit = .05 * max(1., abs(best / source_points))
    result = []
    for row in supported:
        dt, dr = geometry.distances(nominal_text, row["representative_pose_matrix16"])
        gap = (best - float(row["best_score"])) / source_points
        if (dt > .2 or dr > 2.) and gap <= energy_gap_limit:
            result.append(dict(row, nominal_translation_m=dt, nominal_rotation_deg=dr,
                               energy_gap_per_source=gap, energy_gap_limit=energy_gap_limit))
    return result


def extract(frame, candidates, nominal_text, geometry):
    c.require(len(candidates) == 263 and sorted(int(r["seed_index"]) for r in candidates) == list(range(263)),
              "oracle BASE263 seed identity/count mismatch")
    runs = [row for row in candidates if row["converged"] == "1"]
    clusterer = historical_clusterer()
    groups = clusterer["connected_clusters"](runs, .2, 2.)
    primary = [clusterer["summarize_cluster"](group, frame, "primary", len(runs)) for group in groups]
    primary.sort(key=lambda row: (-row["best_score"], row["cluster_id"]))
    for rank, row in enumerate(primary, 1):
        row.update(cluster_id=f"P{rank:02d}", objective_rank=rank)
    major = major_clusters(primary, nominal_text, int(frame["prepared_source_point_count"]), geometry)
    tx = int(frame["transaction_id"])
    label = dict(transaction_id=tx, label="MAJOR" if major else "NO_MAJOR", major_count=len(major),
                 major_cluster_ids=";".join(row["cluster_id"] for row in major),
                 oracle_seed_count=263, converged_seed_count=len(runs), primary_cluster_count=len(primary))
    major_ids = {row["cluster_id"] for row in major}
    for row in primary:
        row["is_major"] = int(row["cluster_id"] in major_ids)
    return primary, label


def parity(geometry):
    receipt = json.loads((c.OUT / "selection_freeze.json").read_text())
    c.require(c.digest(c.OUT / "heldout_ordered_pool.csv") == receipt["artifacts"]["heldout_ordered_pool.csv"],
              "ordered pool not frozen before label parity")
    paths = [c.ARCHIVE / "candidates.csv", c.ARCHIVE / "clusters/mode_clusters.csv",
             c.ARCHIVE / "frozen/cohort_frozen.csv", c.ARCHIVE / "dual_u.csv"]
    provenance_path = c.ROOT / "docs/p9_r1b_strong_attractor_closure/input_provenance.json"
    preparation_path = c.ROOT / "docs/p9_r1a_true_profile_closure/oracle_preparation.json"
    c.pinned(provenance_path)
    c.pinned(preparation_path)
    provenance = json.loads(provenance_path.read_text())
    for name, sha in json.loads(preparation_path.read_text())["inputs_sha256"].items():
        c.require(name not in provenance or provenance[name] == sha, "conflicting historical hash lineage")
        provenance[name] = sha
    for path in paths:
        c.require(str(path) in provenance and c.digest(path) == provenance[str(path)],
                  "historical oracle input hash missing/mismatched: " + str(path))
    frames = [row for row in c.read_csv(paths[2]) if row["wide_targeted"] == "0"]
    c.require(len(frames) == 24, "historical BASE263 frame count not24")
    candidates = defaultdict(list)
    for row in c.read_csv(paths[0]):
        candidates[int(row["transaction_id"])].append(row)
    saved = {(int(r["transaction_id"]), r["cluster_id"]): r for r in c.read_csv(paths[1])
             if r["threshold_set"] == "primary"}
    uobs = {int(row["transaction_id"]): row for row in c.read_csv(paths[3])}
    recovery_path = c.ROOT / "docs/p9_dual_u_nonlocal_r1/sidecars/profile_cohort_final/oracle_recovery.csv"
    c.pinned(recovery_path)
    recovery = {int(row["transaction_id"]): row for row in c.read_csv(recovery_path)}
    rows = []
    reproduced_ids = set()
    for frame in frames:
        tx = int(frame["transaction_id"])
        center = ";".join(uobs[tx]["raw_" + key] for key in ("x","y","z","qx","qy","qz","qw"))
        primary, label = extract(dict(frame, saved_raw_pose_xyz_q_xyzw=frame["raw_terminal_pose_xyz_q_xyzw"]),
                                 candidates[tx], center, geometry)
        expected_ids = {v for v in recovery[tx]["missed_major_cluster_ids"].split(";") if v not in ("", "NONE")}
        actual_ids = {v for v in label["major_cluster_ids"].split(";") if v}
        c.require(actual_ids == expected_ids, f"ORACLE_LABEL_CONTRACT_PARITY_FAIL tx{tx}: {actual_ids} != {expected_ids}")
        c.require(len(primary) == sum(k[0] == tx for k in saved), "primary cluster count changed")
        for row in primary:
            old = saved[tx, row["cluster_id"]]
            c.require(row["representative_pose_matrix16"] == old["representative_pose_matrix16"] and
                      row["seed_count"] == int(old["seed_count"]) and
                      row["stable_mode_candidate"] == int(old["stable_mode_candidate"]) and
                      abs(row["best_score"] - float(old["best_score"])) <= 1e-10,
                      "historical primary membership/representative/score parity fail")
        reproduced_ids.update((tx, cluster) for cluster in actual_ids)
        rows.append(dict(label, expected_major_ids=";".join(sorted(expected_ids)), parity="PASS"))
        print("R4_ORACLE_PARITY", tx, label["label"], label["major_cluster_ids"], flush=True)
    c.require(len(reproduced_ids) == 22 and sum(r["label"] == "MAJOR" for r in rows) == 9,
              "ORACLE_LABEL_CONTRACT_PARITY_FAIL aggregate24/9/15/22")
    c.write_csv(c.OUT / "oracle_parity.csv", rows)
    c.save_json(c.OUT / "oracle_parity_freeze.json", dict(status="PASS", frames=24, major=9, no_major=15,
        major_ids=22, cluster_source_sha256=CLUSTER_HASH, cluster_git_sha=c.HISTORY_SHA,
        major_definition_source="p9_ndt_energy_contract.cpp: energy_gap_limit/separated/gap",
        major_definition_source_sha256=c.pinned(c.HERE / "p9_ndt_energy_contract.cpp"),
        frozen_geometry_library=str(geometry.path), frozen_geometry_library_sha256=c.digest(geometry.path),
        extractor_sha256=c.digest(__file__),
        input_sha256={str(path): c.digest(path) for path in paths},
        artifacts={"oracle_parity.csv": c.digest(c.OUT / "oracle_parity.csv")},
        ordered_pool_sha256=c.digest(c.OUT / "heldout_ordered_pool.csv"), new_ndt_calls=0, gt_loaded=False))
    print("R4_HISTORICAL_BASE263_ORACLE_LABEL_PARITY=PASS frames24 major9 no_major15 ids22")


def self_test(geometry):
    from p9_r2b_nonoracle_evidence import text
    identity = np.eye(4)
    competitor = identity.copy()
    competitor[0, 3] = .5
    primary = [dict(cluster_id="P01", stable_mode_candidate=1, best_score=100.,
                    representative_pose_matrix16=text(identity)),
               dict(cluster_id="P02", stable_mode_candidate=1, best_score=95.,
                    representative_pose_matrix16=text(competitor)),
               dict(cluster_id="P03", stable_mode_candidate=0, best_score=101.,
                    representative_pose_matrix16=text(competitor))]
    # The per-source floor is essential: 5/1400 < .05 even when 5% of100 is5.
    nominal = "0;0;0;0;0;0;1"
    c.require([r["cluster_id"] for r in major_clusters(primary, nominal, 1400, geometry)] == ["P02"],
              "supported/nominal/per-source-floor label definition regression")
    primary[1]["best_score"] = 20.
    c.require(not major_clusters(primary, nominal, 1400, geometry), "noncompetitive cluster counted as major")
    competitor[:3, 3] = [.12, .16, 0]
    primary[1].update(best_score=100., representative_pose_matrix16=text(competitor))
    boundary = major_clusters(primary, nominal, 1400, geometry)
    c.require(len(boundary) == 1 and boundary[0]["nominal_translation_m"] == float(np.float32(.2))
              and boundary[0]["nominal_translation_m"] > .2, "float threshold boundary drift")
    historical_clusterer()
    print("P9_R4_ORACLE_LABEL_SELF_TEST=PASS")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("parity", "self-test"))
    parser.add_argument("--library", type=c.Path, required=True)
    args = parser.parse_args()
    geometry = FrozenGeometry(args.library)
    self_test(geometry)
    if args.stage == "parity":
        parity(geometry)
