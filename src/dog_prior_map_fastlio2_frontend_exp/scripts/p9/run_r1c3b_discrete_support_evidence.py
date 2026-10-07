#!/usr/bin/env python3
"""Run and audit the offline P9-R1C3B discrete support-event study."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import time

import numpy as np


ROOT = Path(__file__).resolve().parents[4]
SOURCE = Path(__file__).resolve().parent
OUT = ROOT / "docs/p9_r1c3b_discrete_support_evidence"
R1C3A = ROOT / "docs/p9_r1c3a_contract_closure"
R1C3 = ROOT / "docs/p9_r1c3_secondgen_branch_gate"
H1 = ROOT / "docs/p9_foundation_weak_discovery/h1"
ARCHIVE = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/dual_u_r1_closure_20261003/same_objective")
MAP = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/map/frozen/floor01_h1_map_p5_frozen.pcd")
COHORT = ARCHIVE / "frozen/cohort_frozen.csv"
UOBS = ARCHIVE / "dual_u.csv"
BRANCH = "research/p9-r1c3b-discrete-support-evidence"
START_SHA = "81d90c60d06ba0eba155072727f98d7600c2c558"
MAJOR_TX = {368, 616, 2226, 2350, 2722, 2846, 3341, 3796, 3962}
N_MAJOR = len(MAJOR_TX)
PERMUTATIONS = 100000
PERMUTATION_SEED = 20261016
ASSOCIATION_SEED = 20261017
ENV = dict(os.environ, LD_LIBRARY_PATH="/lib/x86_64-linux-gnu", OMP_NUM_THREADS="1",
           PYTHONDONTWRITEBYTECODE="1")
ENGINE_OUTPUTS = ["support_events_raw.csv", "event_stationary_pairs.csv",
                  "event_repeatability.csv", "frame_support_features_unlabeled.csv"]
FINAL_OUTPUTS = ENGINE_OUTPUTS + ["frame_support_features.csv", "major_healthy_statistics.csv",
    "lofo.csv", "h1_support_association.csv", "execution_manifest.json", "REPORT.md",
    "results.json", "artifact_hashes.json", "engine_stdout.log"]


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text())


def read_csv(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows, fields):
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fields, lineterminator="\n", extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def write_json(path, value):
    with Path(path).open("w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def finite_float(value, default=None):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def mean(values):
    return float(sum(values) / len(values)) if values else None


def median(values):
    return float(np.median(np.asarray(values, dtype=float))) if values else None


def rank_average(values):
    order = sorted(range(len(values)), key=lambda i: values[i])
    ranks = [0.0] * len(values)
    i = 0
    while i < len(order):
        j = i + 1
        while j < len(order) and values[order[j]] == values[order[i]]:
            j += 1
        rank = (i + 1 + j) * 0.5
        for k in range(i, j):
            ranks[order[k]] = rank
        i = j
    return ranks


def roc_auc(labels, scores):
    positives = [scores[i] for i, label in enumerate(labels) if label]
    negatives = [scores[i] for i, label in enumerate(labels) if not label]
    if not positives or not negatives:
        return None
    wins = sum((p > n) + 0.5 * (p == n) for p in positives for n in negatives)
    return float(wins / (len(positives) * len(negatives)))


def spearman(x, y):
    if len(x) != len(y) or len(x) < 2:
        return None
    rx = np.asarray(rank_average(x), dtype=float)
    ry = np.asarray(rank_average(y), dtype=float)
    if np.std(rx) == 0 or np.std(ry) == 0:
        return None
    return float(np.corrcoef(rx, ry)[0, 1])


def threshold_for_training(labels, scores):
    unique = sorted(set(scores))
    candidates = [-math.inf]
    candidates.extend((a + b) * 0.5 for a, b in zip(unique, unique[1:]))
    candidates.append(math.inf)
    best_threshold = candidates[0]
    best_score = -1.0
    for threshold in candidates:
        pred = [score >= threshold for score in scores]
        tp = sum(p and y for p, y in zip(pred, labels))
        tn = sum((not p) and (not y) for p, y in zip(pred, labels))
        npos = sum(labels)
        nneg = len(labels) - npos
        balanced = 0.5 * (tp / npos + tn / nneg)
        if balanced > best_score + 1e-15 or (abs(balanced - best_score) <= 1e-15 and threshold < best_threshold):
            best_score = balanced
            best_threshold = threshold
    return best_threshold, best_score


def permutation_mean_gap(labels, scores, rng, repeats=PERMUTATIONS):
    labels = np.asarray(labels, dtype=bool)
    scores = np.asarray(scores, dtype=float)
    observed = float(scores[labels].mean() - scores[~labels].mean())
    npos = int(labels.sum())
    exceed = 0
    for _ in range(repeats):
        chosen = rng.choice(scores.size, size=npos, replace=False)
        mask = np.zeros(scores.size, dtype=bool)
        mask[chosen] = True
        stat = float(scores[mask].mean() - scores[~mask].mean())
        if stat >= observed - 1e-15:
            exceed += 1
    return observed, (exceed + 1) / (repeats + 1)


def permutation_spearman(x, y, rng, repeats=PERMUTATIONS):
    observed = spearman(x, y)
    if observed is None:
        return None
    x_rank = np.asarray(rank_average(x), dtype=float)
    y_rank = np.asarray(rank_average(y), dtype=float)
    x_rank = (x_rank - x_rank.mean()) / np.linalg.norm(x_rank - x_rank.mean())
    y_centered = y_rank - y_rank.mean()
    denom = np.linalg.norm(y_centered)
    exceed = 0
    for _ in range(repeats):
        permuted = rng.permutation(y_centered)
        value = float(np.dot(x_rank, permuted) / denom)
        if abs(value) >= abs(observed) - 1e-15:
            exceed += 1
    return observed, (exceed + 1) / (repeats + 1)


def frame_level_statistics(rows, feature, rng):
    ordered = sorted(rows, key=lambda row: int(row["tx"]))
    labels = [row["label"] == "MAJOR_COMPETITOR" for row in ordered]
    scores = [float(row[feature]) for row in ordered]
    auc = roc_auc(labels, scores)
    gap, p_value = permutation_mean_gap(labels, scores, rng)
    loo = []
    for held in range(len(ordered)):
        train_labels = labels[:held] + labels[held + 1:]
        train_scores = scores[:held] + scores[held + 1:]
        threshold, train_balanced = threshold_for_training(train_labels, train_scores)
        predicted = scores[held] >= threshold
        loo.append({"feature": feature, "tx": ordered[held]["tx"], "label": ordered[held]["label"],
            "heldout_score": scores[held], "threshold": threshold if math.isfinite(threshold) else str(threshold),
            "predicted_label": "MAJOR_COMPETITOR" if predicted else "NO_MAJOR_BASIN",
            "correct": int(predicted == labels[held]), "training_balanced_accuracy": train_balanced,
            "heldout_training_auc": roc_auc(train_labels, train_scores),
            "omitted_major_minus_no_major_mean": mean_gap_without(labels, scores, held)})
    lofo_accuracy = mean([row["correct"] for row in loo])
    lofo_min_auc = min(row["heldout_training_auc"] for row in loo)
    lofo_max_auc = max(row["heldout_training_auc"] for row in loo)
    positive_omissions = sum(row["omitted_major_minus_no_major_mean"] > 0 for row in loo)
    summary = {"feature": feature, "n_major_frames": sum(labels), "n_no_major_frames": len(labels) - sum(labels),
        "major_mean": mean([score for score, label in zip(scores, labels) if label]),
        "no_major_mean": mean([score for score, label in zip(scores, labels) if not label]),
        "major_minus_no_major_mean": gap, "roc_auc": auc, "permutation_p_one_sided": p_value,
        "lofo_accuracy": lofo_accuracy, "lofo_min_auc": lofo_min_auc, "lofo_max_auc": lofo_max_auc,
        "lofo_positive_mean_gap_omissions": positive_omissions, "lofo_n": len(loo)}
    return summary, loo


def mean_gap_without(labels, scores, held):
    kept = [(score, label) for i, (score, label) in enumerate(zip(scores, labels)) if i != held]
    positive = [score for score, label in kept if label]
    negative = [score for score, label in kept if not label]
    return float(np.mean(positive) - np.mean(negative))


def sha_manifest_inputs():
    prior_manifest = read_json(R1C3A / "execution_manifest.json")
    prior_results = read_json(R1C3A / "results.json")
    prior_artifacts = read_json(R1C3A / "artifact_hashes.json")
    hashes = {}
    for raw_path, expected in prior_manifest.get("input_sha256", {}).items():
        path = Path(raw_path)
        require(path.is_file(), "frozen R1C3A input missing: " + str(path))
        actual = digest(path)
        require(actual == expected, "frozen R1C3A input hash mismatch: " + str(path))
        hashes[str(path)] = actual
    for raw_path, expected in prior_manifest.get("source_sha256", {}).items():
        path = Path(raw_path)
        if path.name == "CMakeLists.txt":
            continue  # this branch adds only the new offline executable/test
        require(path.is_file(), "frozen lineage source missing: " + str(path))
        actual = digest(path)
        require(actual == expected, "frozen lineage source hash mismatch: " + str(path))
        hashes[str(path)] = actual
    for name, expected in prior_artifacts.items():
        path = R1C3A / name
        require(path.is_file(), "R1C3A artifact missing: " + str(path))
        actual = digest(path)
        require(actual == expected, "R1C3A artifact hash mismatch: " + str(path))
        hashes[str(path)] = actual
    # R1C3A results.json contains an obsolete self-hash value. Its independent
    # artifact_hashes.json records and verifies the current byte-level hash.
    for name, expected in prior_results.get("artifact_sha256", {}).items():
        if name == "results.json":
            continue
        path = R1C3A / name
        require(path.is_file() and digest(path) == expected,
                "R1C3A results.json artifact parity mismatch: " + str(path))
    hashes[str(R1C3A / "artifact_hashes.json")] = digest(R1C3A / "artifact_hashes.json")
    old_sidecars = read_json(R1C3 / "results.json").get("sidecar_sha256", {})
    for name in ["support_events.csv", "numeric_resolution_checks.csv", "branch_nodes.csv"]:
        require(name in old_sidecars, "R1C3 result sidecar hash missing: " + name)
        path = R1C3 / name
        actual = digest(path)
        require(actual == old_sidecars[name], "R1C3 event archive hash mismatch: " + str(path))
        hashes[str(path)] = actual
    for path in [MAP, COHORT, UOBS, H1 / "frame_projection_statistics.csv",
                 H1 / "results.json", R1C3A / "execution_manifest.json",
                 R1C3A / "results.json", R1C3A / "support_pair_stationary.csv",
                 R1C3 / "support_events.csv", R1C3 / "numeric_resolution_checks.csv",
                 R1C3 / "branch_nodes.csv"]:
        require(path.is_file(), "required frozen input missing: " + str(path))
        hashes[str(path)] = digest(path)
    expected_map = prior_results.get("input_sha256", {}).get(str(MAP))
    if expected_map:
        require(hashes[str(MAP)] == expected_map, "official map hash differs from R1C3A manifest")
    cohort_rows = read_csv(COHORT)
    require(len(cohort_rows) == 32, "expected frozen 32-frame cohort")
    txs = [int(row["transaction_id"]) for row in cohort_rows]
    require(len(set(txs)) == 32, "duplicate transaction in frozen cohort")
    for row in cohort_rows:
        source = Path(row["raw_cloud_file"])
        require(source.is_file(), "raw cohort source missing: " + str(source))
        actual = digest(source)
        require(actual == row["raw_source_sha256"], "raw source hash mismatch tx=" + row["transaction_id"])
        hashes[str(source)] = actual
    require(digest(MAP) == hashes[str(MAP)], "map SHA256 changed during input audit")
    return hashes, cohort_rows


def git_state():
    branch = subprocess.check_output(["git", "branch", "--show-current"], cwd=ROOT, text=True).strip()
    head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    require(branch == BRANCH, "R1C3B branch mismatch")
    require(head == START_SHA, "R1C3B must start from the frozen R1C3A commit")
    return {"branch": branch, "start_sha": head}


def build_manifest_sources(binary):
    names = ["CMakeLists.txt", "p9_discrete_support_evidence.cpp",
        "p9_true_profile_closure.cpp", "p9_ndt_energy_contract.cpp",
        "p9_stationarity_numerics.hpp", "p9_r1c3a_contract_closure.cpp",
        "p9_secondgen_branch_gate.cpp", "p9_numeric_branch_certificate.hpp"]
    result = {str(SOURCE / name): digest(SOURCE / name) for name in names}
    result[str(Path(__file__))] = digest(__file__)
    result[str(OUT / "THEORY.md")] = digest(OUT / "THEORY.md")
    result[str(binary)] = digest(binary)
    return result


def prepare_output():
    OUT.mkdir(parents=True, exist_ok=True)
    require((OUT / "THEORY.md").is_file(), "write/freeze THEORY.md before execution")
    for name in FINAL_OUTPUTS:
        require(not (OUT / name).exists(), "refusing to overwrite existing artifact: " + name)


def run_engine(binary, temp_out):
    args = [str(binary), "--run", str(MAP), str(COHORT), str(UOBS),
        str(R1C3A / "support_pair_stationary.csv"), str(R1C3 / "support_events.csv"),
        str(R1C3 / "numeric_resolution_checks.csv"), str(R1C3 / "branch_nodes.csv"), str(temp_out)]
    log_path = temp_out / "engine_stdout.log"
    with log_path.open("w", encoding="utf-8") as log:
        process = subprocess.Popen(args, cwd=ROOT, env=ENV, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True, bufsize=1)
        for line in process.stdout:
            print(line.rstrip(), flush=True)
            log.write(line)
            log.flush()
        code = process.wait()
    require(code == 0, "R1C3B event engine failed with exit code " + str(code))
    engine_text = log_path.read_text()
    # The engine completion record uses lowercase snake_case for this field;
    # the separate historical-registry validator uses the uppercase token.
    require(re.search(r"\bndt_align_calls=0\b", engine_text) is not None,
            "engine did not report zero NDT align calls")
    require("frames=32" in engine_text and "edges=768" in engine_text,
            "event engine did not process the complete frozen cohort")
    for name in ENGINE_OUTPUTS:
        require((temp_out / name).is_file(), "event engine output missing: " + name)


def summarize(temp_out, input_hashes, source_hashes, binary, git, engine_wall_seconds):
    raw_events = read_csv(temp_out / "support_events_raw.csv")
    pairs = read_csv(temp_out / "event_stationary_pairs.csv")
    repeatability = read_csv(temp_out / "event_repeatability.csv")
    unlabeled = read_csv(temp_out / "frame_support_features_unlabeled.csv")
    h1_rows = read_csv(H1 / "frame_projection_statistics.csv")
    h1_txs = {int(row["frame"]) for row in h1_rows}
    require(h1_txs == MAJOR_TX, "frozen H1 major-frame set mismatch")
    require(sum(int(row["major_count"]) for row in h1_rows) == 22, "frozen H1 major basin count mismatch")
    h1_by_tx = {int(row["frame"]): row for row in h1_rows}
    features_by_tx = {int(row["tx"]): row for row in unlabeled}
    require(len(raw_events) == 768 + 24, "raw support event row count is not 32x24 plus 24 history rows")
    require(len(unlabeled) == 32, "expected one unlabeled feature row per cohort frame")
    require(len(repeatability) == 32 * 6, "expected six repeatability stencils per frame")
    require(len(pairs) >= 24, "historical and local support pair outputs are incomplete")
    cohort_rows = read_csv(COHORT)
    require(set(features_by_tx) == {int(row["transaction_id"]) for row in cohort_rows},
            "event feature transaction set differs from frozen cohort")

    frame_rows = []
    for tx in sorted(features_by_tx):
        old = features_by_tx[tx]
        label = "MAJOR_COMPETITOR" if tx in MAJOR_TX else "NO_MAJOR_BASIN"
        h1 = h1_by_tx.get(tx)
        rho = finite_float(h1["R_weak2"]) if h1 else None
        frame_rows.append({"tx": tx, "frame_id": old["frame_id"], "label": label,
            "major_count": int(h1["major_count"]) if h1 else 0,
            "major_ids": h1["major_ids"] if h1 else "",
            "rho_W2": rho if rho is not None else "",
            "support_event_count": int(old["support_event_count"]),
            "resolved_event_count": int(old["resolved_event_count"]),
            "material_event_count": int(old["material_event_count"]),
            "strong_material_event_count": int(old["strong_material_event_count"]),
            "repeatable_event_count": int(old["repeatable_event_count"]),
            "exact_support_pair_recurrence_count": int(old["exact_support_pair_recurrence_count"]),
            "FRAME_MAX_Iv": finite_float(old["FRAME_MAX_Iv"], 0.0),
            "FRAME_MEDIAN_Iv": finite_float(old["FRAME_MEDIAN_Iv"]),
            "FRAME_MAX_IE": finite_float(old["FRAME_MAX_IE"], 0.0),
            "FRAME_MEDIAN_IE": finite_float(old["FRAME_MEDIAN_IE"]),
            "max_translation_separation_m": finite_float(old["max_translation_separation_m"], 0.0),
            "max_rotation_separation_deg": finite_float(old["max_rotation_separation_deg"], 0.0)})
    fields = ["tx", "frame_id", "label", "major_count", "major_ids", "rho_W2",
        "support_event_count", "resolved_event_count", "material_event_count", "strong_material_event_count",
        "repeatable_event_count", "exact_support_pair_recurrence_count", "FRAME_MAX_Iv", "FRAME_MEDIAN_Iv",
        "FRAME_MAX_IE", "FRAME_MEDIAN_IE", "max_translation_separation_m", "max_rotation_separation_deg"]
    write_csv(temp_out / "frame_support_features.csv", frame_rows, fields)

    # Classifier/permutation only sees the 32 event features after the event
    # engine is complete. H1 labels and rho values are joined at this point.
    rng = np.random.Generator(np.random.PCG64(PERMUTATION_SEED))
    feature_names = ["repeatable_event_count", "FRAME_MAX_Iv", "FRAME_MAX_IE"]
    stats_rows, loo_rows = [], []
    for feature in feature_names:
        summary, loo = frame_level_statistics(frame_rows, feature, rng)
        stats_rows.append(summary)
        loo_rows.extend(loo)
    write_csv(temp_out / "major_healthy_statistics.csv", stats_rows,
        ["feature", "n_major_frames", "n_no_major_frames", "major_mean", "no_major_mean",
         "major_minus_no_major_mean", "roc_auc", "permutation_p_one_sided", "lofo_accuracy",
         "lofo_min_auc", "lofo_max_auc", "lofo_positive_mean_gap_omissions", "lofo_n"])
    write_csv(temp_out / "lofo.csv", loo_rows,
        ["feature", "tx", "label", "heldout_score", "threshold", "predicted_label", "correct",
         "training_balanced_accuracy", "heldout_training_auc", "omitted_major_minus_no_major_mean"])

    major = [row for row in frame_rows if row["label"] == "MAJOR_COMPETITOR"]
    association_rng = np.random.Generator(np.random.PCG64(ASSOCIATION_SEED))
    association_rows = []
    for feature in feature_names:
        rho_x = [float(row["rho_W2"]) for row in major]
        values_y = [float(row[feature]) for row in major]
        rho, p_value = permutation_spearman(rho_x, values_y, association_rng)
        association_rows.append({"feature": feature, "n_major_frames": len(major),
            "spearman_rho_W2_vs_feature": rho, "two_sided_permutation_p": p_value,
            "permutations": PERMUTATIONS, "rng": "NumPy PCG64", "seed_stream_base": ASSOCIATION_SEED})
    write_csv(temp_out / "h1_support_association.csv", association_rows,
        ["feature", "n_major_frames", "spearman_rho_W2_vs_feature", "two_sided_permutation_p",
         "permutations", "rng", "seed_stream_base"])

    primary = stats_rows[0]
    major_repeatable_stencils = sum(int(row["repeatable_event_count"]) for row in major)
    major_material_stencils = sum(1 for row in repeatability if int(row["tx"]) in MAJOR_TX and
        int(row["material_count"]) + int(row["strong_material_count"]) > 0)
    unstable_fraction = (major_repeatable_stencils / major_material_stencils
                         if major_material_stencils else 1.0)
    all_positive_omissions = primary["lofo_positive_mean_gap_omissions"] == primary["lofo_n"]
    supported = (primary["permutation_p_one_sided"] < 0.05 and primary["roc_auc"] >= 0.80 and
        primary["lofo_accuracy"] >= 0.80 and primary["lofo_min_auc"] >= 0.70 and all_positive_omissions)
    if supported:
        final_result = "DISCRETE_SUPPORT_EVIDENCE_SUPPORTED"
    elif major_material_stencils >= 5 and unstable_fraction < 0.5:
        final_result = "SUPPORT_EVENTS_NUMERICALLY_UNSTABLE"
    elif ((primary["roc_auc"] >= 0.80 or primary["permutation_p_one_sided"] < 0.05) and
          (primary["lofo_accuracy"] < 0.80 or primary["lofo_min_auc"] < 0.70 or not all_positive_omissions)):
        final_result = "SUPPORT_EVIDENCE_COHORT_DEPENDENT"
    else:
        final_result = "DISCRETE_SUPPORT_EVIDENCE_NOT_DISCRIMINATIVE"

    raw_stencil_events = [row for row in raw_events if row["origin"] == "DETERMINISTIC_STENCIL"]
    transition_count = sum(row["transition"] == "1" for row in raw_stencil_events)
    local_pair_rows = [row for row in pairs if row["origin"] == "DETERMINISTIC_STENCIL"]
    material_pairs = sum(row["classification"] == "EVENT_MATERIAL" for row in local_pair_rows)
    strong_pairs = sum(row["classification"] == "EVENT_STRONGLY_MATERIAL" for row in local_pair_rows)
    unresolved_pairs = sum(row["classification"] == "UNRESOLVED" for row in local_pair_rows)
    history_pairs = [row for row in pairs if row["origin"] == "R1C3A_ARCHIVED"]
    require(len(history_pairs) == 24, "all 24 R1C3A archived pairs must be preserved separately")
    major_total = sum(int(row["major_count"]) for row in major)
    require(major_total == 22 and len(major) == 9, "frozen major cohort changed")

    association = {row["feature"]: row for row in association_rows}
    result = {
      "task": "PAPER-P9-R1C3B-DISCRETE-SUPPORT-TRANSITION-EVIDENCE",
      "git": git,
      "result": final_result,
      "science_boundary": "MATERIAL_FIXED_SUPPORT_RESPONSE_TO_SUPPORT_TRANSITIONS; dynamic multi-attractor NOT ESTABLISHED",
      "archive_provenance_note": "R1C3A results.json has an obsolete embedded self-hash; its independent artifact_hashes.json matches the actual file and was used as the authoritative check. Historical files were not modified.",
      "cohort": {"frames": 32, "major_frames": 9, "no_major_frames": 23,
                 "frozen_major_basins": 22, "major_frame_ids": sorted(MAJOR_TX),
                 "h1_projection_rows": len(h1_rows)},
      "event_contract": {"chart": "eta=[delta_t_map/0.8m,delta_theta_map]=W2u+S4v",
        "stencil_offsets": [-0.02,-0.01,0.0,0.01,0.02], "axes": 6,
        "edges_per_frame": 24, "full_chart_events_evaluated": 768,
        "historical_pairs_supplemental_only": 24,
        "historical_pairs_excluded_from_classifier": True,
        "solver": "R1C2 DOUBLE referenceMinimum on fixed support, common u and v initialization",
        "dynamic_support": "PCL float point transform/radius search; used for event detection and self-consistency only",
        "fixed_support_energy": "continuous double transform, exact frozen PCL leaf score divided by prepared source N",
        "classification": {"negligible": "Iv<=1 AND IE<=1", "material": "Iv>1 OR IE>1",
          "strongly_material": "Iv>3 OR IE>3", "unresolved": "stationarity or SPD check failed",
          "resolved_labels_are_exclusive": True,
          "material_label_semantics": "EVENT_MATERIAL is the moderate-only band; EVENT_STRONGLY_MATERIAL takes precedence"},
        "repeatability": "per axis stencil: >=3/4 adjacent event edges same material class and consistent Iv direction/scale or signed IE/scale; no hash recurrence requirement"},
      "event_counts": {"stencil_support_transitions": transition_count,
        "stationary_pair_solves": len(local_pair_rows), "material_pair_count": material_pairs,
        "strong_material_pair_count": strong_pairs, "unresolved_pair_count": unresolved_pairs,
        "historical_supplemental_pair_count": len(history_pairs), "NDT_align_calls": 0,
        "both_dynamic_support_consistent_pair_count": sum(
          row["support_equal_A"] == "1" and row["support_equal_B"] == "1" for row in local_pair_rows),
        "GT_used": "NO", "posterior_or_EKF_used": "NO"},
      "primary_discriminator": "repeatable_event_count",
      "major_material_stencils": major_material_stencils,
      "major_repeatable_stencils": major_repeatable_stencils,
      "repeatable_fraction_of_major_material_stencils": unstable_fraction,
      "statistics": stats_rows,
      "frame_features": frame_rows,
      "H1_frame_level_associations": association_rows,
      "classification_gates": {"primary_one_sided_permutation_p_lt_0p05": primary["permutation_p_one_sided"] < 0.05,
        "primary_ROC_AUC_ge_0p80": primary["roc_auc"] >= 0.80,
        "LOFO_threshold_accuracy_ge_0p80": primary["lofo_accuracy"] >= 0.80,
        "LOFO_min_AUC_ge_0p70": primary["lofo_min_auc"] >= 0.70,
        "positive_mean_gap_under_every_omission": all_positive_omissions,
        "repeatability instability diagnostic": "major material stencil count >=5 and repeatable/material stencil ratio <0.5"},
      "cost": {"dynamic_support_evaluations": None, "double_energy_evaluations": None,
        "alignment_calls": 0, "wall_seconds": engine_wall_seconds},
      "input_sha256": input_hashes,
      "source_sha256": source_hashes,
      "binary": str(binary), "binary_sha256": digest(binary),
      "rng": {"main_frame_label_permutation": {"type": "NumPy PCG64", "base_seed": PERMUTATION_SEED,
            "replicates_per_feature": PERMUTATIONS, "plus_one_correction": True},
        "major_only_spearman_permutation": {"type": "NumPy PCG64", "seed": ASSOCIATION_SEED,
            "replicates_per_feature": PERMUTATIONS, "two_sided": True}}
    }
    # Pull evaluator counts from the deterministic engine completion line.
    log = (temp_out / "engine_stdout.log").read_text()
    for line in log.splitlines():
        if line.startswith("R1C3B_EVENT_ENGINE=COMPLETE"):
            for token in line.split():
                if token.startswith("double_evaluations="):
                    result["cost"]["double_energy_evaluations"] = int(token.split("=", 1)[1])
                elif token.startswith("dynamic_support_evaluations="):
                    result["cost"]["dynamic_support_evaluations"] = int(token.split("=", 1)[1])
    report = make_report(result, frame_rows, stats_rows, association_rows, raw_events,
                         local_pair_rows, history_pairs, repeatability)
    (temp_out / "REPORT.md").write_text(report, encoding="utf-8")
    return result


def fmt(value, digits=6):
    return "N/A" if value is None else format(value, "." + str(digits) + "g")


def make_report(result, frame_rows, stats_rows, association_rows, raw_events,
                local_pair_rows, history_pairs, repeatability):
    lines = ["# P9-R1C3B discrete support-transition evidence", "",
      "## Result", "",
      "`FINAL_RESULT = " + result["result"] + "`", "",
      "Interpretation is restricted to **material fixed-support response to support transitions**. "
      "Dynamic NDT multiple self-consistent local minima were not established.", "",
      "## Frozen contract", "",
      "- 32 frozen Floor01 frames: 9 major-competitor frames / 22 frozen basins and 23 predeclared no-major frames.",
      "- Local event construction used the fixed P9 map-product chart and six 1-D deterministic stencils at offsets -0.02, -0.01, 0, +0.01, +0.02; 768 adjacent edges total.",
      "- Each changed edge compared its endpoint supports at the same midpoint `(u,v)` using fixed-support DOUBLE strong minimization. H1 basin labels were joined only after event construction.",
      "- The 24 R1C3A archived support-pair rows are supplemental historical examples, separate from the cohort classifier.",
      "- `NDT_ALIGN_CALLS=0`; no GT, posterior, trajectory error, or EKF was used.", "",
      "## Cohort event counts", "",
      "- Support transitions: " + str(result["event_counts"]["stencil_support_transitions"]),
      "- Moderate-only material responses (`EVENT_MATERIAL`): " + str(result["event_counts"]["material_pair_count"]),
      "- Strongly material responses: " + str(result["event_counts"]["strong_material_pair_count"]),
      "- Unresolved fixed-support pairs: " + str(result["event_counts"]["unresolved_pair_count"]),
      "- Historical supplemental pairs: " + str(len(history_pairs)),
      "- Dynamic support evaluations: " + str(result["cost"]["dynamic_support_evaluations"]),
      "- DOUBLE energy evaluations: " + str(result["cost"]["double_energy_evaluations"]),
      "- Cohort engine wall time (seconds): " + fmt(result["cost"]["wall_seconds"], 4),
      "- Pairs whose endpoints both match their frozen support hypothesis: " +
        str(result["event_counts"]["both_dynamic_support_consistent_pair_count"]), "",
      "Resolved event classes are mutually exclusive; `EVENT_MATERIAL` is the moderate-only band, while `EVENT_STRONGLY_MATERIAL` takes precedence.", "",
      "## Frame-level major vs no-major tests", "",
      "| Feature | Major mean | No-major mean | Difference | ROC-AUC | one-sided permutation p | LOFO accuracy | LOFO AUC range | positive omitted gaps |",
      "|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for row in stats_rows:
        lines.append("| {feature} | {major} | {healthy} | {gap} | {auc} | {p} | {acc} | {lo}:{hi} | {positive}/{n} |".format(
            feature=row["feature"], major=fmt(row["major_mean"]), healthy=fmt(row["no_major_mean"]),
            gap=fmt(row["major_minus_no_major_mean"]), auc=fmt(row["roc_auc"]),
            p=fmt(row["permutation_p_one_sided"]), acc=fmt(row["lofo_accuracy"]),
            lo=fmt(row["lofo_min_auc"]), hi=fmt(row["lofo_max_auc"]),
            positive=row["lofo_positive_mean_gap_omissions"], n=row["lofo_n"]))
    lines += ["", "The primary discriminator is `repeatable_event_count`. The 100,000-replicate frame-label test uses PCG64 seed 20261016 and plus-one correction. The 32 frames—not the individual events—are the statistical units.", "",
      "## Per-frame evidence", "",
      "| tx | class | major basins | rho_W2 | events | moderate-only | strong | repeatable stencils | max Iv | max IE |",
      "|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for row in frame_rows:
        lines.append("| {tx} | {label} | {basins} | {rho} | {events} | {material} | {strong} | {rep} | {iv} | {ie} |".format(
          tx=row["tx"], label=row["label"], basins=row["major_count"], rho=fmt(finite_float(row["rho_W2"])),
          events=row["support_event_count"], material=row["material_event_count"],
          strong=row["strong_material_event_count"], rep=row["repeatable_event_count"],
          iv=fmt(row["FRAME_MAX_Iv"]), ie=fmt(row["FRAME_MAX_IE"])))
    lines += ["", "## H1 association (secondary, nine major frames)", "",
      "| Feature | Spearman rho with H1 rho_W2 | two-sided permutation p |", "|---|---:|---:|"]
    for row in association_rows:
        lines.append("| {feature} | {rho} | {p} |".format(feature=row["feature"],
            rho=fmt(row["spearman_rho_W2_vs_feature"]), p=fmt(row["two_sided_permutation_p"])))
    lines += ["", "## Interpretation", "",
      "The output keeps support hash changes, fixed-support impact, dynamic-support self-consistency, and frame discrimination distinct. A support hash change alone is not evidence. Even a material fixed-support response is not a certificate of multiple dynamic local minima.", "",
      "The result gate uses the preregistered primary feature: one-sided permutation p < 0.05, ROC-AUC >= 0.80, LOFO threshold accuracy >= 0.80, LOFO minimum AUC >= 0.70, and a positive major-minus-no-major gap after every single-frame omission. See `results.json` and the CSVs for complete detail.", "",
      "## Provenance", "",
      "Branch: `{}`<br>".format(result["git"]["branch"]),
      "Start SHA: `{}`<br>".format(result["git"]["start_sha"]),
      "PCL: 1.10; Release binary SHA256: `{}`".format(result["binary_sha256"]),
      result["archive_provenance_note"],
      "All input and source SHA256 values are recorded in `results.json` and `execution_manifest.json`.", ""]
    return "\n".join(lines)


def audit_stage(temp_out, result):
    require(result["event_counts"]["NDT_align_calls"] == 0, "NDT align call count is nonzero")
    for name in ENGINE_OUTPUTS + ["frame_support_features.csv", "major_healthy_statistics.csv",
                                  "lofo.csv", "h1_support_association.csv", "REPORT.md"]:
        require((temp_out / name).is_file(), "required R1C3B artifact missing: " + name)
    raw = read_csv(temp_out / "support_events_raw.csv")
    pairs = read_csv(temp_out / "event_stationary_pairs.csv")
    features = read_csv(temp_out / "frame_support_features.csv")
    stats = read_csv(temp_out / "major_healthy_statistics.csv")
    lofo = read_csv(temp_out / "lofo.csv")
    require(len(raw) == 792, "raw support event CSV row count mismatch")
    require(len(features) == 32, "frame feature CSV row count mismatch")
    require(len(stats) == 3 and len(lofo) == 96, "statistical CSV row count mismatch")
    require(sum(row["origin"] == "R1C3A_ARCHIVED" for row in pairs) == 24,
            "historical support pair separation mismatch")
    require(sum(row["label"] == "MAJOR_COMPETITOR" for row in features) == 9,
            "major frame join count mismatch")
    require(sum(int(row["major_count"]) for row in features) == 22,
            "major basin count mismatch")
    json_stats = {row["feature"]: row for row in result["statistics"]}
    for row in stats:
        expected = json_stats.get(row["feature"])
        require(expected is not None, "statistics JSON feature missing from CSV")
        for field in ["major_mean", "no_major_mean", "major_minus_no_major_mean", "roc_auc",
                      "permutation_p_one_sided", "lofo_accuracy", "lofo_min_auc", "lofo_max_auc"]:
            require(abs(float(row[field]) - float(expected[field])) <= 1e-12,
                    "CSV/JSON statistics mismatch: " + row["feature"] + "/" + field)
    json_frames = {int(row["tx"]): row for row in result.get("frame_features", [])}
    require(len(json_frames) == 32, "frame-level feature JSON missing")
    for row in features:
        expected = json_frames[int(row["tx"])]
        for field in ["label", "support_event_count", "material_event_count", "strong_material_event_count",
                      "repeatable_event_count", "exact_support_pair_recurrence_count"]:
            require(str(row[field]) == str(expected[field]), "CSV/JSON frame mismatch: " + field)
    result["artifact_sha256"] = {name: digest(temp_out / name) for name in
      ENGINE_OUTPUTS + ["frame_support_features.csv", "major_healthy_statistics.csv", "lofo.csv",
        "h1_support_association.csv", "REPORT.md", "THEORY.md"]}
    write_json(temp_out / "results.json", result)
    write_json(temp_out / "execution_manifest.json", {
      "git": result["git"], "input_sha256": result["input_sha256"],
      "source_sha256": result["source_sha256"], "binary_sha256": result["binary_sha256"],
      "rng": result["rng"], "event_contract": result["event_contract"],
      "new_ndt_align_calls": 0, "gt_used": "NO", "posterior_used": "NO", "ekf_used": "NO"})
    hashes = {name: digest(temp_out / name) for name in
      ENGINE_OUTPUTS + ["frame_support_features.csv", "major_healthy_statistics.csv", "lofo.csv",
        "h1_support_association.csv", "REPORT.md", "results.json", "execution_manifest.json",
        "engine_stdout.log", "THEORY.md"]}
    # The artifact hash file intentionally does not self-hash.
    write_json(temp_out / "artifact_hashes.json", hashes)
    require(result["artifact_sha256"]["support_events_raw.csv"] == digest(temp_out / "support_events_raw.csv"),
            "CSV hash consistency failed")


def run(binary):
    git = git_state()
    prepare_output()
    input_hashes, cohort_rows = sha_manifest_inputs()
    cache = binary.parent / "CMakeCache.txt"
    require(cache.is_file() and "CMAKE_BUILD_TYPE:STRING=Release" in cache.read_text(),
            "Release CMake build required")
    require(binary.is_file(), "R1C3B diagnostic binary missing")
    engine_source = (SOURCE / "p9_discrete_support_evidence.cpp").read_text()
    require(re.search(r"\.align\s*\(", engine_source) is None,
            "NDT align invocation is forbidden in this offline event engine")
    source_hashes = build_manifest_sources(binary)
    start = time.time()
    with tempfile.TemporaryDirectory(prefix="p9_r1c3b_") as stage_name:
        stage = Path(stage_name)
        history_args = [str(binary), "--validate-history", str(MAP), str(COHORT), str(UOBS),
            str(R1C3A / "support_pair_stationary.csv"), str(R1C3 / "support_events.csv"),
            str(R1C3 / "numeric_resolution_checks.csv"), str(R1C3 / "branch_nodes.csv")]
        validation = subprocess.run(history_args, cwd=ROOT, env=ENV, text=True,
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        print(validation.stdout, end="", flush=True)
        require(validation.returncode == 0, "historical support registry validation failed")
        require("pairs=24" in validation.stdout and "NDT_ALIGN_CALLS=0" in validation.stdout,
                "historical support validation did not close exact pair contract")
        engine_start = time.time()
        run_engine(binary, stage)
        engine_wall_seconds = time.time() - engine_start
        result = summarize(stage, input_hashes, source_hashes, binary, git, engine_wall_seconds)
        result["elapsed_wall_seconds"] = time.time() - start
        result["cohort_source_rows"] = len(cohort_rows)
        # Hashes bind executable and inputs; capture complete stdout after run.
        shutil.copy2(OUT / "THEORY.md", stage / "THEORY.md")
        audit_stage(stage, result)
        for name in FINAL_OUTPUTS:
            shutil.copy2(stage / name, OUT / name)
    print("R1C3B_FINAL_RESULT=" + result["result"])
    print("R1C3B_ARTIFACT_DIR=" + str(OUT))
    return result


def self_test():
    require(abs(roc_auc([False, True], [0.0, 1.0]) - 1.0) < 1e-12, "AUC self-test")
    require(abs(roc_auc([False, True], [1.0, 1.0]) - 0.5) < 1e-12, "tie AUC self-test")
    require(abs(spearman([1, 2, 3], [2, 4, 6]) - 1.0) < 1e-12, "Spearman self-test")
    threshold, balanced = threshold_for_training([False, False, True, True], [0, 1, 3, 4])
    require(1 < threshold < 3 and balanced == 1.0, "LOFO threshold self-test")
    labels = [True] * 9 + [False] * 23
    scores = [2.0] * 9 + [0.0] * 23
    gap, p = permutation_mean_gap(labels, scores, np.random.Generator(np.random.PCG64(7)), repeats=2000)
    require(gap > 0 and 0 < p < 1, "frame permutation self-test")
    require(len(MAJOR_TX) == 9 and N_MAJOR == 9, "frozen cohort self-test")
    print("P9_R1C3B_STATISTICS_SELF_TEST=PASS")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--binary", type=Path, default=Path("/tmp/p9_r1c3b_build/p9_discrete_support_evidence"))
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if args.run:
        run(args.binary.resolve())
        return
    parser.error("choose --self-test or --run")


if __name__ == "__main__":
    main()
