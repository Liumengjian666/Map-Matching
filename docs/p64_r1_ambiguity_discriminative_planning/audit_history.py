"""Recompute P63-R10 history from the archived raw and joined telemetry."""
import csv
import gzip
import hashlib
import json
import sys
from pathlib import Path


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main(archive):
    run = archive / "docs/p63_r10_hdl_imu_full_hard"
    trajectory = run / "FULL_TRAJECTORY.csv"
    native_path = run / "raw/full_01/NATIVE_NDT.csv.gz"
    reference = archive / "docs/p63_r11_harmful_ndt_observation/REFERENCE_DIAGNOSIS.csv"
    with trajectory.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    with gzip.open(native_path, "rt", newline="") as stream:
        native = list(csv.DictReader(stream))
    with reference.open(newline="") as stream:
        independent_table = next(row for row in csv.DictReader(stream) if row["raw_ordinal"] == "309")
    assert [int(row["raw_ordinal"]) for row in rows] == list(range(5, 2380))
    assert len(native) == len(rows)
    for row, trace in zip(rows, native):
        for field in ("stamp_ns", "converged", "source_xyz_sha256", "target_xyz_sha256"):
            assert row[field] == trace[field]
    row = rows[309 - 5]
    assert row["stamp_ns"] == independent_table["stamp_ns"]
    for field, ref_field in (("pred_translation_error_m", "pred_t_m"), ("ndt_translation_error_m", "ndt_t_m"),
                             ("pred_rotation_error_deg", "pred_R_deg"), ("ndt_rotation_error_deg", "ndt_R_deg")):
        assert abs(float(row[field]) - float(independent_table[ref_field])) < 1e-8
    frame_fields = ("stamp_ns", "relative_s", "iterations", "inlier_fraction", "pred_translation_error_m",
                    "ndt_translation_error_m", "pred_rotation_error_deg", "ndt_rotation_error_deg",
                    "ndt_correction_m", "ndt_correction_deg")
    valid = [row for row in rows if row["reference_valid"] == "True"]
    final_fields = ("raw_ordinal", "translation_error_m", "rotation_error_deg", "ndt_translation_error_m",
                    "ndt_rotation_error_deg", "pred_translation_error_m", "pred_rotation_error_deg",
                    "inlier_fraction", "iterations")
    return {
        "rows": len(rows),
        "reference_valid": len(valid),
        "reported_converged": sum(row["converged"] == "1" for row in native),
        "last_reference_valid_frame": {field: float(valid[-1][field]) for field in final_fields},
        "raw309": {field: int(row[field]) if field in {"stamp_ns", "iterations"} else float(row[field])
                   for field in frame_fields},
        "sha256": {str(path.relative_to(archive)): sha256(path) for path in (trajectory, native_path, reference)},
    }


if __name__ == "__main__":
    print(json.dumps(main(Path(sys.argv[1])), indent=2))
