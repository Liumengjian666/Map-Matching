#!/usr/bin/env python3
"""Post-freeze, single-pair conditional-GT evaluation for P47 shadows."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def read_rows(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def csv_pose(row, prefix=""):
    p = np.asarray([float(row[prefix + k]) for k in ("tx", "ty", "tz")], dtype=np.float64)
    q = np.asarray([float(row[prefix + k]) for k in ("qx", "qy", "qz", "qw")], dtype=np.float64)
    if not np.all(np.isfinite(p)) or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-12:
        raise ValueError("non-finite or invalid pose in frozen P47 output")
    return p, Rotation.from_quat(q / np.linalg.norm(q))


def vector_field(text):
    values = np.asarray([float(value) for value in text.split()], dtype=np.float64)
    if values.shape != (6,) or not np.all(np.isfinite(values)):
        raise ValueError("invalid six-vector in frozen paired output")
    return values


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--artifact-root", type=Path, required=True)
    parser.add_argument("--freeze", type=Path, required=True)
    parser.add_argument("--gt", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError("refusing to overwrite evaluation output: " + str(args.output))

    freeze = json.loads(args.freeze.read_text())
    if freeze.get("gt_read_before_freeze") is not False:
        raise ValueError("freeze record does not certify that output hashes preceded GT")
    for name, expected in {**freeze["frozen_inputs"], **freeze["frozen_shadow_outputs"]}.items():
        path = args.artifact_root / name
        if not path.is_file() or sha256(path) != expected:
            raise ValueError("frozen P47 artifact changed or missing: " + str(path))

    gt_sha = sha256(args.gt)
    p43_path = args.repo_root / "docs/p43_newer_college_gt_base_frame/input_manifest.json"
    p43 = json.loads(p43_path.read_text())
    expected_gt_sha = p43.get("hashes", {}).get(str(args.gt))
    if not expected_gt_sha:
        raise ValueError("GT path is not listed in the frozen P43 input contract")
    if expected_gt_sha != gt_sha:
        raise ValueError("GT SHA differs from the frozen P43 input contract")
    T_B_L = (np.asarray(p43["T_B_C"], dtype=np.float64) @
             np.asarray(p43["T_C_I"], dtype=np.float64) @
             np.asarray(p43["T_I_L"], dtype=np.float64))

    manifest = read_rows(args.artifact_root / "sample_manifest.csv")
    pairs = read_rows(args.artifact_root / "paired_shadow_comparison_retry3.csv")
    candidates = read_rows(args.artifact_root / "candidate_diagnostics_retry3.csv")
    if len(pairs) != 1 or len(candidates) != 2:
        raise ValueError("P47 frozen contract expected exactly one pair and two candidate rows")
    pair = pairs[0]
    frame = pair["frame_index"]
    stamp_ns = int(pair["scan_ns"])
    sample = next(row for row in manifest if row["frame_index"] == frame)
    if int(sample["scan_ns"]) != stamp_ns:
        raise ValueError("paired frame and frozen sample timestamp differ")

    with args.gt.open(newline="") as stream:
        reader = csv.reader(stream)
        header = [field.strip().lstrip("#") for field in next(reader)]
        gt_rows = [dict(zip(header, row)) for row in reader if row]
    gt_row = next((row for row in gt_rows
                   if int(row["sec"]) * 1_000_000_000 + int(row["nsec"]) == stamp_ns), None)
    if gt_row is None:
        raise ValueError("conditional GT has no exact row at frozen pair timestamp")
    q_gt = np.asarray([float(gt_row[k]) for k in ("qx", "qy", "qz", "qw")])
    R_M_B = Rotation.from_quat(q_gt / np.linalg.norm(q_gt))
    p_M_B = np.asarray([float(gt_row[k]) for k in ("x", "y", "z")])
    R_B_L = Rotation.from_matrix(T_B_L[:3, :3])
    p_M_L = p_M_B + R_M_B.apply(T_B_L[:3, 3])
    R_M_L = R_M_B * R_B_L

    p_nominal, R_nominal = csv_pose(sample, "nominal_")
    W_fields = ["W00", "W10", "W20", "W30", "W40", "W50",
                "W01", "W11", "W21", "W31", "W41", "W51"]
    degeneracy = next(row for row in read_rows(args.artifact_root / "degeneracy_metrics_retry3.csv")
                      if row["frame_index"] == frame)
    W = np.asarray([float(degeneracy[key]) for key in W_fields], dtype=np.float64).reshape(2, 6).T
    eta_gt = np.concatenate(((p_M_L - p_nominal) / 0.8,
                             (R_M_L * R_nominal.inv()).as_rotvec()))

    output = []
    output.append({"frame_index": frame, "scan_ns": stamp_ns, "arm": "CONTROL_NOMINAL",
                   "translation_error_m": np.linalg.norm(p_nominal - p_M_L),
                   "rotation_error_deg": (R_M_L.inv() * R_nominal).magnitude() * 180.0 / np.pi,
                   "local_weak_coordinate_error_norm": np.linalg.norm(W.T @ eta_gt),
                   "conditional_reference": "P43_BASE_FRAME_GEOMETRICALLY_SUPPORTED",
                   "gt_sha256": gt_sha})
    for arm in ("weak_only", "coupled"):
        row = next(value for value in candidates
                   if value["frame_index"] == frame and value["arm"] == arm)
        p, R = csv_pose(row, "candidate_")
        # Candidate poses in the P47 CSV are serialized to 1e-6 precision.
        eta = vector_field(pair["weak_eta"])
        if arm == "coupled":
            eta += vector_field(pair["strong_eta"])
        output.append({"frame_index": frame, "scan_ns": stamp_ns, "arm": arm.upper(),
                       "translation_error_m": np.linalg.norm(p - p_M_L),
                       "rotation_error_deg": (R_M_L.inv() * R).magnitude() * 180.0 / np.pi,
                       "local_weak_coordinate_error_norm": np.linalg.norm(W.T @ (eta_gt - eta)),
                       "conditional_reference": "P43_BASE_FRAME_GEOMETRICALLY_SUPPORTED",
                       "gt_sha256": gt_sha})

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(output[0]))
        writer.writeheader()
        writer.writerows(output)
    print(json.dumps({"gt_sha256": gt_sha, "frame_index": frame, "scan_ns": stamp_ns,
                      "rows_written": len(output), "output": str(args.output)}, indent=2))


if __name__ == "__main__":
    main()
