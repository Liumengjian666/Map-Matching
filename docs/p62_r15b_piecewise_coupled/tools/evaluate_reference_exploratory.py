#!/usr/bin/env python3
"""Evaluate sealed R15B poses against the conditional R14B map reference.

This is intentionally exploratory: the reference is map-anchored and the CSV
axis interpretation remains a paper/model-supported assumption, not exporter-
bound independent motion-capture truth.
"""

import csv
import hashlib
import json
import math
import pathlib
import sys
from datetime import datetime, timezone

import numpy as np
from scipy.spatial.transform import Rotation


ROOT = pathlib.Path.cwd()
ARCHIVE = ROOT / "docs/p62_r15b_piecewise_coupled"
RUN = ARCHIVE / "run_01"
SEAL = RUN / "MODEL_OUTPUT_FREEZE.json"
ADMISSION = RUN / "REFERENCE_EVALUATION_ADMISSION.json"
COMPLETE = RUN / "REFERENCE_EVALUATION_COMPLETE.json"
ERRORS = ARCHIVE / "REFERENCE_ERROR_EXPLORATORY.csv"
SUMMARY = ARCHIVE / "REFERENCE_SUMMARY_EXPLORATORY.json"
R14 = ROOT / "docs/p62_r14b_newer_college/conditional_base_pilot"


def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_rows(path):
    with path.open("r", newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def matrix_pipe(text):
    vals = np.asarray([float(v) for v in text.split("|")], dtype=float)
    if vals.size != 16 or not np.isfinite(vals).all():
        raise ValueError("malformed/non-finite frozen pose")
    pose = vals.reshape(4, 4)
    if (np.linalg.norm(pose[3] - [0., 0., 0., 1.]) > 1e-9 or
            np.linalg.norm(pose[:3, :3].T @ pose[:3, :3] - np.eye(3)) > 1e-5 or
            np.linalg.det(pose[:3, :3]) < .999):
        raise ValueError("frozen pose is not a valid SE(3) transform")
    return pose


def checked_transform(value, label):
    pose = np.asarray(value, dtype=float)
    if pose.shape != (4, 4) or not np.isfinite(pose).all():
        raise ValueError(label + " malformed/non-finite")
    if (np.linalg.norm(pose[3] - [0., 0., 0., 1.]) > 1e-10 or
            np.linalg.norm(pose[:3, :3].T @ pose[:3, :3] - np.eye(3)) > 1e-6 or
            np.linalg.det(pose[:3, :3]) < .999):
        raise ValueError(label + " is not SE(3)")
    return pose


def reference_lidar_pose(row_values, t_bl):
    values = np.asarray(row_values, dtype=float)
    if values.shape != (7,) or not np.isfinite(values).all() or abs(np.linalg.norm(values[3:]) - 1.) > 1e-5:
        raise ValueError("invalid registered Base reference pose")
    t_mb = np.eye(4)
    t_mb[:3, :3] = Rotation.from_quat(values[3:]).as_matrix()
    t_mb[:3, 3] = values[:3]
    return t_mb @ t_bl


def pose_error(estimate, reference):
    trans = float(np.linalg.norm(estimate[:3, 3] - reference[:3, 3]))
    rot = Rotation.from_matrix(reference[:3, :3].T @ estimate[:3, :3])
    return trans, float(np.linalg.norm(rot.as_rotvec()) * 180. / math.pi)


def stats(values):
    if not values:
        return {"n": 0, "rmse": None, "median": None, "p95_linear": None, "max": None}
    vals = np.asarray(values, dtype=float)
    if not np.isfinite(vals).all():
        raise ValueError("non-finite error values")
    return {"n": int(vals.size), "rmse": float(np.sqrt(np.mean(vals * vals))),
            "median": float(np.median(vals)), "p95_linear": float(np.quantile(vals, .95, method="linear")),
            "max": float(np.max(vals))}


def main():
    for path in (ERRORS, SUMMARY, COMPLETE):
        if path.exists():
            raise RuntimeError("refusing to overwrite existing reference evaluation artifact: " + str(path))
    if not SEAL.is_file():
        raise RuntimeError("model output seal missing")
    seal = json.loads(SEAL.read_text(encoding="utf-8"))
    if seal.get("status") != "MODEL_OUTPUTS_FROZEN_BEFORE_REFERENCE_EVALUATION" or seal.get("reference_error_file_absent_at_freeze") is not True:
        raise RuntimeError("model output freeze does not establish pre-reference state")
    if seal.get("frame_count") != 25 or seal.get("new_ndt_aligns") != 0 or seal.get("ekf_runs") != 0 or seal.get("gt_pose_reads_before_freeze") != 0:
        raise RuntimeError("model freeze receipt violates the R15B contract")
    for item in seal["outputs"]:
        path = RUN / item["path"]
        if not path.is_file() or path.stat().st_size != item["bytes"] or sha(path) != item["sha256"]:
            raise RuntimeError("model output changed after freeze: " + item["path"])

    prereg_path = R14 / "TEST_FRAME_PREREGISTRATION.json"
    prereg = json.loads(prereg_path.read_text(encoding="utf-8"))
    if prereg.get("assumption") != "PAPER_SUPPORTED_BASE_FRAME_ASSUMPTION":
        raise RuntimeError("unexpected R14B base-frame assumption")
    ref_meta = prereg["source_inputs"][0]
    gt_path = pathlib.Path(ref_meta["path"])
    if not gt_path.is_file() or gt_path.stat().st_size != ref_meta["bytes"] or sha(gt_path) != ref_meta["sha256"]:
        raise RuntimeError("registered_poses.csv frozen identity mismatch")

    parent = json.loads((ARCHIVE / "PARENT_INPUT_FREEZE.json").read_text(encoding="utf-8"))
    matching_identity = [entry for entry in parent["inputs"]["file_identities"] if entry["path"] == str(gt_path)]
    if len(matching_identity) != 1 or matching_identity[0]["sha256"] != ref_meta["sha256"] or matching_identity[0]["bytes"] != ref_meta["bytes"]:
        raise RuntimeError("reference is not part of the pre-run frozen input manifest")

    selected = {}
    for frame in prereg["frames"]:
        raw = int(frame["raw_ordinal_one_based"])
        if 768 <= raw <= 787:
            stamp = int(frame["stamp_ns"])
            if int(frame["reference_match_ns"]) != stamp or int(frame["reference_delta_ns"]) != 0:
                raise RuntimeError("frozen exact reference timestamp mismatch")
            selected[stamp] = (raw, frame)
    if len(selected) != 20:
        raise RuntimeError("fixed Newer College reference frame set incomplete")
    t_bl = checked_transform(prereg["T_B_L"], "frozen T_B_L")
    r15b_prereg = json.loads((ARCHIVE / "PRE_REGISTRATION.json").read_text(encoding="utf-8"))
    r13_frame_to_raw = dict(zip(r15b_prereg["fixed_frames"]["GEODE_R13"]["archive_frame_keys"],
                                r15b_prereg["fixed_frames"]["GEODE_R13"]["raw_scan_indices"]))

    source_identity = sha(prereg_path)
    admission_payload = {"task": "P62-R15B-PIECEWISE-COUPLED-REFINEMENT",
                         "created_utc": datetime.now(timezone.utc).isoformat(),
                         "model_output_freeze_sha256": sha(SEAL),
                         "model_outputs_rechecked": True,
                         "registered_poses_sha256": ref_meta["sha256"],
                         "registered_poses_bytes": ref_meta["bytes"],
                         "registered_poses_identity_rechecked_before_value_read": True,
                         "r14b_preregistration_sha256": source_identity,
                         "reference_pose_values_opened_at_this_point": False,
                         "reference_kind": "MAP_ANCHORED_BENCHMARK_REFERENCE",
                         "axis_assumption": "PAPER_SUPPORTED_BASE_FRAME_ASSUMPTION",
                         "warning": "Exploratory Reference-Relative Pose Error only; not independent mocap truth and no exporter-bound axis certification."}
    if ADMISSION.exists():
        old_admission = json.loads(ADMISSION.read_text(encoding="utf-8"))
        if (old_admission.get("model_output_freeze_sha256") != sha(SEAL) or
                old_admission.get("registered_poses_sha256") != ref_meta["sha256"] or
                old_admission.get("reference_pose_values_opened_at_this_point") is not False):
            raise RuntimeError("existing reference admission receipt does not match frozen model/input identity")
    else:
        with ADMISSION.open("x", encoding="utf-8") as stream:
            json.dump(admission_payload, stream, indent=2, sort_keys=True)
            stream.write("\n")

    # No selected reference pose values are read until after the model seal and
    # the explicit access-admission receipt above have been written.
    references = {}
    with gt_path.open("r", newline="", encoding="utf-8") as stream:
        reader = csv.reader(stream)
        next(reader)
        for row in reader:
            stamp = int(row[0]) * 1_000_000_000 + int(row[1])
            if stamp not in selected:
                continue
            if stamp in references:
                raise ValueError("duplicate frozen reference timestamp")
            references[stamp] = reference_lidar_pose(row[2:], t_bl)
    if len(references) != 20:
        raise ValueError("not all 20 exact-time reference poses were found")

    boundary = read_rows(RUN / "ASSOCIATION_BOUNDARY_ANALYSIS.csv")
    candidates = read_rows(RUN / "CANDIDATE_OBJECTIVES.csv")
    pair = read_rows(RUN / "WEAK_ONLY_VS_COUPLED.csv")
    pred_archive = read_rows(R14 / "FINAL_OBSERVATIONS.csv")
    sub_by_raw = {int(r["frame_key"]): r for r in boundary if r["dataset"] == "NEWER_COLLEGE_R14B"}
    if len(sub_by_raw) != 20:
        raise ValueError("missing frozen R15B Newer College frame identities")
    candidate_by = {(int(r["raw_index"]), float(r["alpha"]), r["method"]): r for r in candidates if r["dataset"] == "NEWER_COLLEGE_R14B"}
    pair_by = {(int(r["frame_key"]), float(r["alpha"])): r for r in pair if r["dataset"] == "NEWER_COLLEGE_R14B"}
    pred_by = {int(r["frame"]): r for r in pred_archive}
    if len(candidate_by) != 20 * 2 * 3 or len(pair_by) != 40 or len(pred_by) != 20:
        raise ValueError("frozen prediction/candidate table identities incomplete")

    method_names = ["TP", "T0", "WEAK_ONLY_ALPHA_0.5", "COUPLED_ALPHA_0.5", "WEAK_ONLY_ALPHA_1.0", "COUPLED_ALPHA_1.0"]
    results = []
    aggregate = {name: {"translation": [], "rotation": []} for name in method_names}
    paired = {name: {"translation_delta_vs_T0": [], "rotation_delta_vs_T0": [], "translation_better": 0, "rotation_better": 0} for name in method_names if name != "T0"}
    for raw in range(768, 788):
        b = sub_by_raw[raw]
        stamp = int(b["stamp_ns"])
        if stamp not in selected or selected[stamp][0] != raw:
            raise ValueError("model output timestamp differs from frozen R14B preregistration")
        ref = references[stamp]
        pred_row = pred_by[raw - 767]
        # Build matrices explicitly in row-major order; prediction is frozen
        # R14B common-causal input, not the evaluated reference pose.
        tp_vals = [float(pred_row[f"prediction_{i}{j}"]) for i in range(4) for j in range(4)]
        tp = matrix_pipe("|".join(str(v) for v in tp_vals))
        estimates = {
            "TP": (tp, "CAUSAL_COMMON_PREDICTION", 0, 0),
            "T0": (matrix_pipe(candidate_by[(raw, .5, "NOMINAL_NDT")]["pose_pipe"]), "FROZEN_NATIVE_NDT", 0,
                                int(candidate_by[(raw, .5, "NOMINAL_NDT")]["converged"])),
        }
        for alpha in (.5, 1.):
            pair_row = pair_by[(raw, alpha)]
            for method, col in (("WEAK_ONLY", "weak_pose"), ("COUPLED", "coupled_pose")):
                pose_text = pair_row[col]
                candidate_row = candidate_by[(raw, alpha, method + "_PROBE")]
                estimate = matrix_pipe(pose_text) if pose_text != "NA" else None
                estimates[f"{method}_ALPHA_{alpha:.1f}"] = (estimate, "POST_NDT_REFINEMENT_CANDIDATE",
                    int(candidate_row["formal_eligible"]), int(candidate_row["converged"]))
        t0_error = pose_error(estimates["T0"][0], ref)
        for method in method_names:
            estimate, role, formal_eligible, native_converged = estimates[method]
            available = estimate is not None
            trans, rotation = pose_error(estimate, ref) if available else (None, None)
            results.append({"dataset": "NEWER_COLLEGE_R14B", "frame_key": str(raw), "raw_index": str(raw),
                            "stamp_ns": stamp, "method": method, "comparison_role": role,
                            "pose_available": int(available), "formal_eligible": formal_eligible,
                            "native_converged": native_converged, "reference_evaluable": 1,
                            "translation_error_m": trans if available else "NA",
                            "rotation_error_deg": rotation if available else "NA",
                            "reference_kind": "MAP_ANCHORED_BENCHMARK_REFERENCE",
                            "coordinate_assumption": "PAPER_SUPPORTED_BASE_FRAME_ASSUMPTION",
                            "reference_file_sha256": ref_meta["sha256"], "reference_pose_read": 1})
            if available:
                aggregate[method]["translation"].append(trans)
                aggregate[method]["rotation"].append(rotation)
                if method != "T0":
                    paired[method]["translation_delta_vs_T0"].append(trans - t0_error[0])
                    paired[method]["rotation_delta_vs_T0"].append(rotation - t0_error[1])
                    paired[method]["translation_better"] += int(trans < t0_error[0])
                    paired[method]["rotation_better"] += int(rotation < t0_error[1])

    # GEODE has no admissible independent reference in the frozen archive.
    for row in boundary:
        if row["dataset"] != "GEODE_R13":
            continue
        for method in method_names:
            results.append({"dataset": "GEODE_R13", "frame_key": row["frame_key"],
                            "raw_index": str(r13_frame_to_raw[row["frame_key"]]),
                            "stamp_ns": row["stamp_ns"], "method": method, "comparison_role": "NO_REFERENCE",
                            "pose_available": 0, "formal_eligible": 0, "native_converged": 0,
                            "reference_evaluable": 0, "translation_error_m": "NA", "rotation_error_deg": "NA",
                            "reference_kind": "INDEPENDENT_REFERENCE_NOT_AVAILABLE", "coordinate_assumption": "NOT_APPLICABLE",
                            "reference_file_sha256": "NA", "reference_pose_read": 0})

    fieldnames = list(results[0].keys())
    with ERRORS.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(results)

    summary = {method: {"translation_error_m": stats(vals["translation"]),
                        "rotation_error_deg": stats(vals["rotation"])} for method, vals in aggregate.items()}
    payload = {
        "status": "EXPLORATORY_REFERENCE_RELATIVE_ERROR_COMPUTED",
        "reference_kind": "MAP_ANCHORED_BENCHMARK_REFERENCE",
        "not_independent_motion_capture_truth": True,
        "coordinate_assumption": "PAPER_SUPPORTED_BASE_FRAME_ASSUMPTION",
        "file_level_reference_axis_certified": False,
        "time_matching": "frozen exact integer nanoseconds, zero delta; no interpolation or fitting",
        "T_M_L_reference": "T_M_B times frozen T_B_L",
        "alignment_fitted": False,
        "reference_pose_rows_used": 20,
        "reference_file_sha256": ref_meta["sha256"],
        "reference_pose_value_reads_before_model_freeze": 0,
        "model_output_freeze_sha256": sha(SEAL),
        "metrics": summary,
        "paired_differences_vs_T0": paired,
        "quantile": "NumPy linear p95",
        "limitations": ["Newer College reference is generated by registering Ouster scans to the Leica map, not independent mocap.",
                        "Base interpretation has paper/URDF support but no direct registered_poses.csv exporter binding.",
                        "R15B was context-informed by earlier benchmark summaries; reference-relative result is exploratory, not blind.",
                        "All refinement poses are POST_NDT_REFINEMENT_CANDIDATE, native_converged=false, formally ineligible; no EKF feedback."],
    }
    with SUMMARY.open("x", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
    with COMPLETE.open("x", encoding="utf-8") as stream:
        json.dump({"task": "P62-R15B-PIECEWISE-COUPLED-REFINEMENT",
                   "completed_utc": datetime.now(timezone.utc).isoformat(),
                   "model_output_freeze_sha256": sha(SEAL),
                   "reference_file_sha256": ref_meta["sha256"],
                   "reference_pose_rows_used": 20,
                   "offline_evaluation_attempt": (3 if (ARCHIVE / "REFERENCE_EVALUATION_ATTEMPT_02_FAILURE.json").exists()
                                                  else 2 if (ARCHIVE / "REFERENCE_EVALUATION_ATTEMPT_01_FAILURE.json").exists()
                                                  else 1),
                   "prior_evaluator_failures_preserved": sum((ARCHIVE / f"REFERENCE_EVALUATION_ATTEMPT_0{i}_FAILURE.json").exists()
                                                              for i in (1, 2)),
                   "reference_kind": "MAP_ANCHORED_BENCHMARK_REFERENCE",
                   "coordinate_assumption": "PAPER_SUPPORTED_BASE_FRAME_ASSUMPTION",
                   "transform_fit": False}, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps({"status": payload["status"], "reference_rows": 20, "summary": summary}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print("REFERENCE_EVALUATION_FAILED: " + str(exc), file=sys.stderr)
        raise
