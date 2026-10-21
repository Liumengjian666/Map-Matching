#!/usr/bin/env python3
"""Post-freeze exploratory Reference-Relative Pose Error for Newer College."""
import csv
import hashlib
import json
import math
import pathlib
import sys

import numpy as np
from scipy.spatial.transform import Rotation


def sha(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def checked_pose(pipe, label):
    values = np.asarray([float(x) for x in pipe.split("|")], dtype=float)
    if values.size != 16 or not np.isfinite(values).all():
        raise ValueError(label + " malformed pose")
    p = values.reshape(4, 4)
    if (np.linalg.norm(p[3] - [0., 0., 0., 1.]) > 1e-9 or
            np.linalg.norm(p[:3, :3].T @ p[:3, :3] - np.eye(3)) > 1e-5 or
            np.linalg.det(p[:3, :3]) < .999):
        raise ValueError(label + " is not SE(3)")
    return p


def reference_pose(values, t_bl):
    v = np.asarray(values, dtype=float)
    if v.shape != (7,) or not np.isfinite(v).all() or abs(np.linalg.norm(v[3:]) - 1.) > 1e-5:
        raise ValueError("invalid frozen registered Base reference row")
    t_mb = np.eye(4)
    t_mb[:3, :3] = Rotation.from_quat(v[3:]).as_matrix()
    t_mb[:3, 3] = v[:3]
    return t_mb @ t_bl


def error(estimate, reference):
    trans = float(np.linalg.norm(estimate[:3, 3] - reference[:3, 3]))
    rot = Rotation.from_matrix(reference[:3, :3].T @ estimate[:3, :3])
    return trans, float(np.linalg.norm(rot.as_rotvec()) * 180. / math.pi)


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: evaluate_reference.py <R15C archive> <R14B conditional archive>")
    archive = pathlib.Path(sys.argv[1])
    r14 = pathlib.Path(sys.argv[2])
    run = archive / "run_01"
    seal_path = run / "MODEL_OUTPUT_FREEZE.json"
    admission_path = run / "REFERENCE_EVALUATION_ADMISSION.json"
    complete_path = run / "REFERENCE_EVALUATION_COMPLETE.json"
    output_path = archive / "REFERENCE_ERROR_EXPLORATORY.csv"
    for path in (admission_path, complete_path, output_path):
        if path.exists():
            raise RuntimeError("refusing to overwrite reference-evaluation artifact: " + str(path))
    seal = json.loads(seal_path.read_text(encoding="utf-8"))
    if (seal.get("status") != "R15C_MODEL_OUTPUTS_FROZEN_BEFORE_REFERENCE_EVALUATION" or
            seal.get("frame_count") != 25 or seal.get("new_NDT_aligns") != 0 or
            seal.get("gt_pose_values_read_before_freeze") != 0):
        raise RuntimeError("R15C output freeze does not authorize post-freeze evaluation")
    for entry in seal["outputs"]:
        path = run / entry["path"]
        if not path.is_file() or path.stat().st_size != entry["bytes"] or sha(path) != entry["sha256"]:
            raise RuntimeError("R15C model output changed after freeze: " + entry["path"])

    prereg_path = r14 / "TEST_FRAME_PREREGISTRATION.json"
    prereg = json.loads(prereg_path.read_text(encoding="utf-8"))
    if prereg.get("assumption") != "PAPER_SUPPORTED_BASE_FRAME_ASSUMPTION":
        raise RuntimeError("R14B Base frame assumption changed")
    source_rows = prereg.get("source_inputs", [])
    matches = [r for r in source_rows if r.get("path", "").endswith("registered_poses.csv")]
    if len(matches) != 1:
        raise RuntimeError("R14B registered_poses identity not uniquely declared")
    ref_meta = matches[0]
    gt_path = pathlib.Path(ref_meta["path"])
    if not gt_path.is_file() or gt_path.stat().st_size != ref_meta["bytes"] or sha(gt_path) != ref_meta["sha256"]:
        raise RuntimeError("frozen registered_poses identity mismatch")
    if "T_B_L" not in prereg:
        raise RuntimeError("frozen T_B_L missing from R14B pre-registration")
    t_bl = np.asarray(prereg["T_B_L"], dtype=float)
    if t_bl.shape != (4, 4) or not np.isfinite(t_bl).all():
        raise RuntimeError("invalid frozen T_B_L")

    selected = {}
    for frame in prereg["frames"]:
        raw = int(frame["raw_ordinal_one_based"])
        if 768 <= raw <= 787:
            stamp = int(frame["stamp_ns"])
            if int(frame["reference_match_ns"]) != stamp or int(frame["reference_delta_ns"]) != 0:
                raise RuntimeError("R14B fixed exact timestamp matching changed")
            selected[stamp] = raw
    if len(selected) != 20:
        raise RuntimeError("fixed R14B Newer College frame set incomplete")

    metrics = list(csv.DictReader((run / "CANDIDATE_METRICS.csv").open(newline="", encoding="utf-8")))
    nc_rows = [r for r in metrics if r["dataset"] == "NEWER_COLLEGE_R14B"]
    if len(nc_rows) != 20 * 2 * 4:
        raise RuntimeError("frozen candidate metric rows do not match fixed 20-frame/two-alpha/four-method design")
    model_access = {"task": "P62-R15C-COUPLING-ATTRIBUTION-AND-NOVELTY",
                    "model_output_freeze_sha256": sha(seal_path),
                    "model_outputs_rechecked": True,
                    "registered_poses_sha256": ref_meta["sha256"],
                    "registered_poses_bytes": ref_meta["bytes"],
                    "R14B_preregistration_sha256": sha(prereg_path),
                    "reference_kind": "MAP_ANCHORED_BENCHMARK_REFERENCE",
                    "axis_assumption": "PAPER_SUPPORTED_BASE_FRAME_ASSUMPTION",
                    "reference_pose_values_opened_at_this_point": False,
                    "warning": "Exploratory Reference-Relative Pose Error only; map-anchored benchmark reference, not independent mocap truth; R14B base-axis interpretation is paper/model-supported but not exporter-bound."}
    with admission_path.open("x", encoding="utf-8") as f:
        json.dump(model_access, f, indent=2, sort_keys=True); f.write("\n")

    # No selected GT values are read until after the model seal and admission are in place.
    references = {}
    with gt_path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.reader(f); next(reader)
        for row in reader:
            stamp = int(row[0]) * 1_000_000_000 + int(row[1])
            if stamp in selected:
                if stamp in references:
                    raise ValueError("duplicate R14B reference timestamp")
                references[stamp] = reference_pose(row[2:], t_bl)
    if len(references) != 20:
        raise RuntimeError("fixed 20 reference poses not recovered")

    grouped = {}
    records = []
    for row in nc_rows:
        raw = int(row["frame_key"])
        stamp = next((s for s, r in selected.items() if r == raw), None)
        if stamp is None or stamp not in references:
            raise RuntimeError("candidate frame lacks frozen reference match")
        estimate = checked_pose(row["pose_pipe"], "candidate")
        trans, rot = error(estimate, references[stamp])
        record = {"dataset": row["dataset"], "raw_index": raw, "stamp_ns": stamp,
                  "alpha": row["alpha"], "method": row["method"],
                  "translation_error_m": trans, "rotation_error_deg": rot,
                  "reference_kind": "MAP_ANCHORED_BENCHMARK_REFERENCE",
                  "candidate_kind": "FROZEN_NATIVE_NDT" if row["method"] == "NOMINAL_NDT" else "POST_NDT_REFINEMENT_CANDIDATE"}
        records.append(record)
        grouped.setdefault((row["alpha"], row["method"]), []).append(record)
    with output_path.open("x", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(records[0].keys())); writer.writeheader(); writer.writerows(records)
    summaries = {}
    for (alpha, method), rows in sorted(grouped.items()):
        t = np.asarray([r["translation_error_m"] for r in rows])
        q = np.asarray([r["rotation_error_deg"] for r in rows])
        summaries[f"alpha_{alpha}_{method}"] = {
            "n": int(t.size),
            "translation_rmse_m": float(np.sqrt(np.mean(t*t))),
            "translation_median_m": float(np.median(t)),
            "translation_p95_m": float(np.quantile(t, .95, method="linear")),
            "translation_max_m": float(t.max()),
            "rotation_rmse_deg": float(np.sqrt(np.mean(q*q))),
            "rotation_median_deg": float(np.median(q)),
            "rotation_p95_deg": float(np.quantile(q, .95, method="linear")),
            "rotation_max_deg": float(q.max())}
    complete = {"status": "R15C_EXPLORATORY_REFERENCE_EVALUATION_COMPLETE",
                "model_output_freeze_sha256": sha(seal_path),
                "registered_poses_sha256": ref_meta["sha256"],
                "frame_count": 20, "rows": len(records),
                "reference_kind": "MAP_ANCHORED_BENCHMARK_REFERENCE",
                "axis_assumption": "PAPER_SUPPORTED_BASE_FRAME_ASSUMPTION",
                "stats": summaries}
    with complete_path.open("x", encoding="utf-8") as f:
        json.dump(complete, f, indent=2, sort_keys=True); f.write("\n")
    print("R15C_REFERENCE_EVALUATION_COMPLETE frames=20 rows=%d reference=MAP_ANCHORED_BENCHMARK_REFERENCE" % len(records))


if __name__ == "__main__":
    main()
