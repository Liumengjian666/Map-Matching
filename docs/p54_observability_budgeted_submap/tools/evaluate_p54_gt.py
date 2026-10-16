#!/usr/bin/env python3
"""Post-freeze P54 conditional-GT evaluation; hashes are verified first."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def read_rows(path: Path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def pose_from(row, prefix=""):
    p = np.array([float(row[prefix + k]) for k in ("tx", "ty", "tz")], dtype=float)
    q = np.array([float(row[prefix + k]) for k in ("qx", "qy", "qz", "qw")], dtype=float)
    if not np.all(np.isfinite(p)) or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-12:
        raise ValueError("invalid frozen pose")
    return p, Rotation.from_quat(q / np.linalg.norm(q))


def errors(p, r, p_ref, r_ref):
    return float(np.linalg.norm(p - p_ref)), float((r_ref.inv() * r).magnitude() * 180.0 / math.pi)


def summarize(values):
    a = np.asarray(values, dtype=float)
    if len(a) == 0:
        return {"n": 0, "rmse": None, "mean": None, "median": None, "p95": None, "max": None}
    return {"n": int(len(a)), "rmse": float(np.sqrt(np.mean(a * a))),
            "mean": float(np.mean(a)), "median": float(np.median(a)),
            "p95": float(np.percentile(a, 95)), "max": float(np.max(a))}


def main():
    p = argparse.ArgumentParser()
    for name in ("freeze", "p43", "sample", "align", "gt", "accuracy_csv", "metrics_json"):
        p.add_argument("--" + name.replace("_", "-"), type=Path, required=True)
    a = p.parse_args()
    if a.accuracy_csv.exists() or a.metrics_json.exists():
        raise FileExistsError("refusing to overwrite P54 post-freeze GT outputs")

    freeze = json.loads(a.freeze.read_text())
    if freeze.get("protocol") != "P54_PRE_GT_OUTPUT_FREEZE_V1" or freeze.get("gt_opened") is not False:
        raise ValueError("P54 freeze does not certify pre-GT freeze")
    frozen_files = freeze["inputs_and_outputs"] + freeze["frozen_sources"] + freeze["raw_scans"] + freeze["active_tiles"]
    for item in frozen_files:
        path = Path(item["path"])
        if not path.is_file() or sha256(path) != item["sha256"]:
            raise ValueError("frozen pre-GT file changed: " + str(path))

    p43 = json.loads(a.p43.read_text())
    gt_sha = sha256(a.gt)
    if p43.get("hashes", {}).get(str(a.gt)) != gt_sha:
        raise ValueError("GT hash differs from frozen P43 input manifest")
    T_B_L = (np.asarray(p43["T_B_C"], dtype=float) @ np.asarray(p43["T_C_I"], dtype=float) @
             np.asarray(p43["T_I_L"], dtype=float))
    R_B_L = Rotation.from_matrix(T_B_L[:3, :3])

    with a.gt.open(newline="") as stream:
        reader = csv.reader(stream)
        header = [value.strip().lstrip("#").strip() for value in next(reader)]
        gt_rows = [dict(zip(header, row)) for row in reader if row]
    gt_by_ns = {}
    for row in gt_rows:
        stamp = int(row["sec"]) * 1_000_000_000 + int(row["nsec"])
        if stamp in gt_by_ns:
            raise ValueError("duplicate conditional GT timestamp")
        q = np.asarray([float(row[k]) for k in ("qx", "qy", "qz", "qw")], dtype=float)
        pos = np.asarray([float(row[k]) for k in ("x", "y", "z")], dtype=float)
        if not np.all(np.isfinite(q)) or not np.all(np.isfinite(pos)) or np.linalg.norm(q) < 1e-12:
            raise ValueError("invalid conditional GT row")
        R_M_B = Rotation.from_quat(q / np.linalg.norm(q))
        p_M_B = pos
        gt_by_ns[stamp] = (p_M_B + R_M_B.apply(T_B_L[:3, 3]), R_M_B * R_B_L)

    samples = read_rows(a.sample)
    align_rows = read_rows(a.align)
    if len(samples) != 154 or len(align_rows) != 616:
        raise ValueError("unexpected P54 sample/alignment row count")
    alignment = {(r["frame_index"], r["budget_points"], r["method"]): r for r in align_rows}
    output = []
    errors_by_group = {}
    missing_gt = []

    for sample in samples:
        frame, stamp = sample["frame_index"], int(sample["scan_ns"])
        if stamp not in gt_by_ns:
            missing_gt.append(stamp)
            continue
        p_ref, R_ref = gt_by_ns[stamp]
        p, R = pose_from(sample, "nominal_")
        et, er = errors(p, R, p_ref, R_ref)
        output.append({"frame_index": frame, "scan_ns": stamp, "group": "P46_LOCAL_CONTROL_NDT",
                       "budget_points": "", "method": "BASELINE", "status": "FROZEN_P46_NOMINAL",
                       "pose_available": 1, "converged": 1, "translation_error_m": et,
                       "rotation_error_deg": er, "delta_translation_error_m": 0.0,
                       "delta_rotation_error_deg": 0.0, "pose_delta_from_p46_m": 0.0,
                       "pose_delta_from_p46_deg": 0.0})
        errors_by_group.setdefault("P46_LOCAL_CONTROL_NDT", []).append((frame, et, er))
        for budget in ("1200000", "1500000"):
            for method in ("DISTANCE_BASED", "INFORMATION_AWARE"):
                row = alignment[(frame, budget, method)]
                available = row["candidate_valid"] == "1"
                converged = row["converged"] == "1"
                if available:
                    pc, Rc = pose_from(row, "raw_")
                    cet, cer = errors(pc, Rc, p_ref, R_ref)
                    dpose_t = float(np.linalg.norm(pc - p))
                    dpose_r = float((R.inv() * Rc).magnitude() * 180.0 / math.pi)
                    de_t, de_r = cet - et, cer - er
                    errors_by_group.setdefault(f"{method}_{budget}", []).append((frame, cet, cer))
                else:
                    cet = cer = de_t = de_r = dpose_t = dpose_r = None
                group = f"{method}_{budget}"
                output.append({"frame_index": frame, "scan_ns": stamp, "group": group,
                               "budget_points": budget, "method": method, "status": row["status"],
                               "pose_available": int(available), "converged": int(converged),
                               "translation_error_m": cet, "rotation_error_deg": cer,
                               "delta_translation_error_m": de_t, "delta_rotation_error_deg": de_r,
                               "pose_delta_from_p46_m": dpose_t, "pose_delta_from_p46_deg": dpose_r})

    if missing_gt:
        raise ValueError(f"P43 requires exact GT timestamps; {len(missing_gt)} sample timestamps have no exact row")
    if len(output) != 154 * 5:
        raise ValueError("not all frozen samples produced the five expected evaluation rows")

    fieldnames = list(output[0])
    a.accuracy_csv.parent.mkdir(parents=True, exist_ok=True)
    with a.accuracy_csv.open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(output)

    metrics = {"protocol": "P54_POST_FREEZE_CONDITIONAL_GT_V1",
               "conditional_reference": "P43_BASE_FRAME_GEOMETRICALLY_SUPPORTED_NOT_EXPORTER_SOURCE_CERTIFIED",
               "gt_sha256": gt_sha, "p43_manifest_sha256": sha256(a.p43),
               "input_freeze_sha256": sha256(a.freeze), "gt_interpolation": "none_exact_timestamp_only",
               "T_B_L": T_B_L.tolist(), "sample_count": len(samples),
               "groups": {}, "paired_vs_p46": {},
               "limitations": ["GT was opened only after pre-GT inputs and all NDT outputs verified against input_freeze.json",
                               "P43 Base-frame semantics are geometrically supported but not exporter-source certified",
                               "This is a 154-frame diagnostic, not continuous localization or runtime memory evidence"]}
    for group, values in errors_by_group.items():
        metrics["groups"][group] = {
            "translation_error_m": summarize([v[1] for v in values]),
            "rotation_error_deg": summarize([v[2] for v in values]),
            "pose_available": len(values),
        }

    baseline = {r["frame_index"]: (float(r["translation_error_m"]), float(r["rotation_error_deg"]))
                for r in output if r["group"] == "P46_LOCAL_CONTROL_NDT"}
    for group in sorted(k for k in errors_by_group if k != "P46_LOCAL_CONTROL_NDT"):
        group_rows = [r for r in output if r["group"] == group]
        deltas_t = [float(r["delta_translation_error_m"]) for r in group_rows if r["pose_available"] == 1]
        deltas_r = [float(r["delta_rotation_error_deg"]) for r in group_rows if r["pose_available"] == 1]
        b_t = [baseline[r["frame_index"]][0] for r in group_rows if r["pose_available"] == 1]
        c_t = [float(r["translation_error_m"]) for r in group_rows if r["pose_available"] == 1]
        b_r = [baseline[r["frame_index"]][1] for r in group_rows if r["pose_available"] == 1]
        c_r = [float(r["rotation_error_deg"]) for r in group_rows if r["pose_available"] == 1]
        metrics["paired_vs_p46"][group] = {
            "n": len(deltas_t), "baseline_translation_rmse_m": summarize(b_t)["rmse"],
            "candidate_translation_rmse_m": summarize(c_t)["rmse"],
            "translation_rmse_relative_change_pct": (summarize(c_t)["rmse"] / summarize(b_t)["rmse"] - 1.0) * 100.0,
            "baseline_rotation_rmse_deg": summarize(b_r)["rmse"],
            "candidate_rotation_rmse_deg": summarize(c_r)["rmse"],
            "rotation_rmse_change_deg": summarize(c_r)["rmse"] - summarize(b_r)["rmse"],
            "translation_error_improved_count": sum(x < 0 for x in deltas_t),
            "translation_error_worsened_count": sum(x > 0 for x in deltas_t),
            "rotation_error_improved_count": sum(x < 0 for x in deltas_r),
            "rotation_error_worsened_count": sum(x > 0 for x in deltas_r),
            "delta_translation_error_m": summarize(deltas_t),
            "delta_rotation_error_deg": summarize(deltas_r),
            "max_candidate_translation_error_m": max(c_t) if c_t else None,
            "max_candidate_rotation_error_deg": max(c_r) if c_r else None,
        }
    a.metrics_json.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"status": "P54_POST_FREEZE_GT_EVALUATION_COMPLETE", "gt_sha256": gt_sha,
                      "exact_gt_rows": len(gt_by_ns), "samples": len(samples),
                      "accuracy_csv": str(a.accuracy_csv), "metrics_json": str(a.metrics_json)}, indent=2))


if __name__ == "__main__":
    main()
