#!/usr/bin/env python3
"""Evaluate R14B reference-relative errors only after candidate output freeze."""
import csv
import hashlib
import json
import math
import pathlib
import sys
from datetime import datetime, timezone

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = pathlib.Path(__file__).resolve().parents[3]
ARCHIVE = ROOT / "docs/p62_r15_coupled_subspace_refinement"
RUN = ARCHIVE / "run_01"
SEAL = ARCHIVE / "CANDIDATE_OUTPUT_FREEZE.json"
INPUT_FREEZE = ARCHIVE / "INPUT_FREEZE.json"
ADMISSION = ARCHIVE / "REFERENCE_ACCESS_ADMISSION.json"
COMPLETE = ARCHIVE / "REFERENCE_EVALUATION_COMPLETE.json"


def sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def read_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def pipe_pose(text):
    values = np.asarray([float(x) for x in text.split("|")], dtype=float)
    if values.size != 16 or not np.isfinite(values).all():
        raise ValueError("non-finite or malformed frozen pose")
    t = values.reshape((4, 4))
    if np.linalg.norm(t[3] - np.array([0., 0., 0., 1.])) > 1e-9:
        raise ValueError("invalid homogeneous row")
    if np.linalg.norm(t[:3, :3].T @ t[:3, :3] - np.eye(3)) > 1e-6 or np.linalg.det(t[:3, :3]) < .999:
        raise ValueError("frozen pose is not SE(3)")
    return t


def reference_base_pose(values, t_bl):
    v = np.asarray(values, dtype=float)
    if v.shape != (7,) or not np.isfinite(v).all() or abs(np.linalg.norm(v[3:]) - 1.) > 1e-5:
        raise ValueError("invalid registered reference pose")
    t_mb = np.eye(4)
    t_mb[:3, :3] = Rotation.from_quat(v[3:]).as_matrix()
    t_mb[:3, 3] = v[:3]
    return t_mb @ t_bl


def error(estimate, reference):
    et = float(np.linalg.norm(estimate[:3, 3] - reference[:3, 3]))
    er = float(np.linalg.norm(Rotation.from_matrix(reference[:3, :3].T @ estimate[:3, :3]).as_rotvec()) * 180. / math.pi)
    return et, er


def stats(values):
    if not values:
        return {"n": 0, "rmse": None, "median": None, "p95_linear": None, "max": None}
    a = np.asarray(values, dtype=float)
    if not np.isfinite(a).all():
        raise ValueError("non-finite error statistic input")
    return {"n": int(a.size), "rmse": float(np.sqrt(np.mean(a * a))),
            "median": float(np.median(a)), "p95_linear": float(np.quantile(a, .95, method="linear")),
            "max": float(np.max(a))}


def main():
    if not SEAL.is_file() or not INPUT_FREEZE.is_file():
        raise RuntimeError("candidate output and input freezes are both required")
    if ADMISSION.exists() or COMPLETE.exists() or (ARCHIVE / "REFERENCE_ERROR_COMPARISON.csv").exists():
        raise RuntimeError("reference evaluation artifacts already exist; refusing overwrite")
    seal = json.loads(SEAL.read_text())
    if seal.get("reference_pose_values_read") is not False or seal.get("status") != "CANDIDATE_AND_DECISION_OUTPUTS_FROZEN_BEFORE_REFERENCE_POSE_READ":
        raise RuntimeError("candidate freeze does not attest pre-reference state")
    for rel, identity in seal["outputs"].items():
        path = ROOT / rel
        if not path.is_file() or sha(path) != identity["sha256"] or path.stat().st_size != identity["bytes"]:
            raise RuntimeError(f"candidate output changed after freeze: {rel}")
    inputs = json.loads(INPUT_FREEZE.read_text())
    prereg = ROOT / "docs/p62_r14b_newer_college/conditional_base_pilot/TEST_FRAME_PREREGISTRATION.json"
    reg_obj = json.loads(prereg.read_text())
    reference_meta = reg_obj["source_inputs"][0]
    gt_path = pathlib.Path(reference_meta["path"])
    if not gt_path.is_file() or sha(gt_path) != reference_meta["sha256"]:
        raise RuntimeError("registered_poses.csv identity mismatch after candidate freeze")
    # seal_inputs.py stores the inventory under the top-level "inputs" key.
    recorded = {entry["path"]: entry["sha256"] for entry in inputs["inputs"]["file_identities"]}
    if recorded.get(str(gt_path)) != reference_meta["sha256"]:
        raise RuntimeError("registered reference identity was not in the pre-run freeze")

    with open(ADMISSION, "x") as f:
        json.dump({"task": "P62-R15-COUPLED-SUBSPACE-REFINEMENT-R1",
                   "created_utc": datetime.now(timezone.utc).isoformat(),
                   "candidate_output_freeze_sha256": sha(SEAL),
                   "candidate_output_hashes_rechecked": True,
                   "registered_poses_sha256_rechecked": True,
                   "reference_pose_values_opened_at_this_point": False}, f, indent=2, sort_keys=True)
        f.write("\n")

    # No reference pose is accessed until the immutable candidate-output seal
    # above has been verified and the admission receipt has been created.
    selected = {}
    for frame in reg_obj["frames"]:
        raw = int(frame["raw_ordinal_one_based"])
        if 768 <= raw <= 787:
            if int(frame["reference_match_ns"]) != int(frame["stamp_ns"]) or int(frame["reference_delta_ns"]) != 0:
                raise ValueError("frozen Newer College reference timestamp is not exact")
            selected[int(frame["stamp_ns"])] = (raw, frame)
    if len(selected) != 20:
        raise ValueError("fixed Newer College reference frame set incomplete")

    t_bl = np.asarray(reg_obj["T_B_L"], dtype=float)
    if t_bl.shape != (4, 4) or not np.isfinite(t_bl).all() or np.linalg.norm(t_bl[3] - [0., 0., 0., 1.]) > 1e-10:
        raise ValueError("frozen T_B_L is invalid")
    if np.linalg.norm(t_bl[:3, :3].T @ t_bl[:3, :3] - np.eye(3)) > 1e-6 or np.linalg.det(t_bl[:3, :3]) < .999:
        raise ValueError("frozen T_B_L rotation is invalid")

    references = {}
    with open(gt_path, newline="") as f:
        reader = csv.reader(f)
        next(reader)
        for row in reader:
            ns = int(row[0]) * 1_000_000_000 + int(row[1])
            if ns not in selected:
                continue
            if ns in references:
                raise ValueError("duplicate selected registered pose timestamp")
            references[ns] = reference_base_pose(row[2:], t_bl)
    if len(references) != 20:
        raise ValueError("fixed 20 references not found; no interpolation or replacement")

    sub = {int(r["raw_index"]): r for r in read_rows(RUN / "SUBSPACE_DECOMPOSITION.csv")
           if r["dataset"] == "NEWER_COLLEGE_R14B"}
    weak = {(int(r["raw_index"]), float(r["alpha"])): r for r in read_rows(RUN / "WEAK_ONLY_RESULTS.csv")
            if r["dataset"] == "NEWER_COLLEGE_R14B"}
    coupled = {(int(r["raw_index"]), float(r["alpha"])): r for r in read_rows(RUN / "COUPLED_RESULTS.csv")
               if r["dataset"] == "NEWER_COLLEGE_R14B"}

    results = []
    aggregate = {}
    methods = ["TP", "T0", "WEAK_ONLY_ALPHA_0.5", "COUPLED_ALPHA_0.5",
               "WEAK_ONLY_ALPHA_1.0", "COUPLED_ALPHA_1.0"]
    for raw in range(768, 788):
        stamp = int(sub[raw]["stamp_ns"])
        if stamp not in selected or selected[stamp][0] != raw:
            raise ValueError("frozen R15 frame/time identity differs from R14 preregistration")
        ref = references[stamp]
        base = sub[raw]
        pose_records = {
            "TP": (pipe_pose(base["TP_pose_pipe"]), "NA", 0, 0, "CAUSAL_PREDICTION"),
            "T0": (pipe_pose(base["T0_pose_pipe"]), "NA", 0, int(base["frozen_T0_converged"]), "FROZEN_NOMINAL_NDT"),
        }
        for alpha in (.5, 1.):
            for method, table in (("WEAK_ONLY", weak), ("COUPLED", coupled)):
                record = table.get((raw, alpha))
                if record is None:
                    raise ValueError(f"missing frozen {method} row for raw frame {raw} alpha {alpha}")
                pose_text = record["pose_pipe"]
                pose_records[f"{method}_ALPHA_{alpha:.1f}"] = (
                    pipe_pose(pose_text) if pose_text != "NA" else None,
                    alpha,
                    int(record["formal_eligible"]),
                    int(record["converged"]),
                    "POST_NDT_DIAGNOSTIC_ONLY",
                )
        for method in methods:
            estimate, alpha, formal_eligible, converged, role = pose_records[method]
            valid = estimate is not None
            et, er = error(estimate, ref) if valid else (None, None)
            results.append({"dataset": "NEWER_COLLEGE_R14B", "frame_key": str(raw), "raw_index": str(raw),
                            "stamp_ns": stamp, "alpha": alpha, "method": method, "comparison_role": role,
                            "pose_available": int(valid), "formal_eligible": formal_eligible,
                            "native_converged": converged, "reference_evaluable": 1,
                            "translation_error_m": et if valid else "NA", "rotation_error_deg": er if valid else "NA",
                            "reference_kind": "MAP_ANCHORED_BENCHMARK_REFERENCE",
                            "coordinate_assumption": "PAPER_SUPPORTED_BASE_FRAME_ASSUMPTION",
                            "reference_file_sha256": reference_meta["sha256"], "reference_pose_read": 1})
            if valid:
                aggregate.setdefault(method, {"translation": [], "rotation": []})
                aggregate[method]["translation"].append(et)
                aggregate[method]["rotation"].append(er)

    # R13 has no independent reference in its frozen archive; record the gap,
    # never emit synthetic zero errors.
    r13_sub = [r for r in read_rows(RUN / "SUBSPACE_DECOMPOSITION.csv") if r["dataset"] == "GEODE_R13"]
    for base in r13_sub:
        for method in methods:
            results.append({"dataset": "GEODE_R13", "frame_key": base["frame_key"], "raw_index": base["raw_index"],
                            "stamp_ns": base["stamp_ns"], "alpha": "NA", "method": method,
                            "comparison_role": "NO_REFERENCE", "pose_available": 0,
                            "formal_eligible": 0, "native_converged": 0, "reference_evaluable": 0,
                            "translation_error_m": "NA", "rotation_error_deg": "NA",
                            "reference_kind": "INDEPENDENT_REFERENCE_NOT_AVAILABLE",
                            "coordinate_assumption": "NOT_APPLICABLE",
                            "reference_file_sha256": "NA", "reference_pose_read": 0})

    out_path = ARCHIVE / "REFERENCE_ERROR_COMPARISON.csv"
    with open(out_path, "x", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(results[0]), lineterminator="\n")
        writer.writeheader(); writer.writerows(results)
    summary = {method: {"translation_m": stats(value["translation"]),
                        "rotation_deg": stats(value["rotation"])}
               for method, value in aggregate.items()}
    summary_payload = {
        "reference_kind": "MAP_ANCHORED_BENCHMARK_REFERENCE",
        "not_independent_motion_capture_truth": True,
        "coordinate_assumption": "PAPER_SUPPORTED_BASE_FRAME_ASSUMPTION",
        "file_level_reference_axis_certified": False,
        "time_matching": "exact integer nanoseconds; zero delta; no interpolation",
        "T_M_L_reference": "T_M_B times frozen T_B_L",
        "alignment_fitted": False,
        "reference_pose_rows_used": 20,
        "metrics": summary,
        "quantile": "NumPy linear p95",
        "candidate_output_freeze_sha256": sha(SEAL),
    }
    with open(ARCHIVE / "REFERENCE_SUMMARY.json", "x") as f:
        json.dump(summary_payload, f, indent=2, sort_keys=True); f.write("\n")
    with open(COMPLETE, "x") as f:
        json.dump({"task": "P62-R15-COUPLED-SUBSPACE-REFINEMENT-R1",
                   "completed_utc": datetime.now(timezone.utc).isoformat(),
                   "candidate_freeze_sha256": sha(SEAL), "reference_sha256": reference_meta["sha256"],
                   "reference_pose_rows_used": 20, "reference_file_rows_parsed": 20,
                   "reference_kind": "MAP_ANCHORED_BENCHMARK_REFERENCE"}, f, indent=2, sort_keys=True)
        f.write("\n")
    print(json.dumps({"reference_kind": summary_payload["reference_kind"], "metrics": summary}, indent=2))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"REFERENCE_EVALUATION_FAILED: {exc}", file=sys.stderr)
        raise
