#!/usr/bin/env python3
"""Audit fixed-lag Control/Shadow parity without loading GT."""

import argparse
import collections
import csv
import json
import pathlib
import sys

SCRIPT_DIR = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR.parent / "p10_corridor01_prospective"))
import compare_shadow_control  # noqa: E402


def rows(path):
    with pathlib.Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def compare_rows(left, right, excluded_fields=()):
    if len(left) != len(right):
        return [{"reason": "row_count", "left": len(left), "right": len(right)}]
    failures = []
    for index, (a, b) in enumerate(zip(left, right)):
        if set(a) != set(b):
            failures.append({"row": index, "reason": "field_set"})
            continue
        for field in a:
            if field in excluded_fields:
                continue
            if a[field] != b[field]:
                failures.append({"row": index, "field": field,
                                 "control": a[field], "shadow": b[field]})
                if len(failures) >= 20:
                    return failures
    return failures


def fixed_lag_audit(control_rows, shadow_rows):
    expected_fields = (
        "observation_id", "ndt_stamp", "deadline_stamp", "release_state_stamp",
        "replay_imu_count", "correction_applied", "status",
    )
    failures = compare_rows(control_rows, shadow_rows, excluded_fields=(
        "arrival_wall_sec", "processing_wall_sec", "arrival_state_stamp"))
    summaries = []
    for name, records in (("control", control_rows), ("shadow", shadow_rows)):
        counts = collections.Counter(row["status"] for row in records)
        queued = [row for row in records if row["status"] == "QUEUED"]
        applied = [row for row in records if row["status"] == "APPLIED"]
        late = [row for row in records if "LATE" in row["status"]]
        if counts != {"QUEUED": 396, "APPLIED": 396}:
            failures.append({"run": name, "reason": "fixed_lag_status_counts",
                             "counts": dict(counts)})
        if len(queued) != 396 or len(applied) != 396 or late:
            failures.append({"run": name, "reason": "fixed_lag_completeness",
                             "queued": len(queued), "applied": len(applied),
                             "late": len(late)})
        if any(float(row["arrival_state_stamp"]) >= float(row["deadline_stamp"])
               for row in queued):
            failures.append({"run": name, "reason": "queued_after_deadline"})
        if any(float(row["release_state_stamp"]) < float(row["deadline_stamp"]) or
               row["correction_applied"] != "1" for row in applied):
            failures.append({"run": name, "reason": "invalid_release_or_no_correction"})
        if any(not all(field in row for field in expected_fields) for row in records):
            failures.append({"run": name, "reason": "fixed_lag_field_missing"})
        summaries.append({
            "run": name,
            "queued": len(queued),
            "applied": len(applied),
            "late": len(late),
            "unique_release_nodes": len({row["release_state_stamp"] for row in applied}),
            "all_arrivals_pre_deadline": all(
                float(row["arrival_state_stamp"]) < float(row["deadline_stamp"])
                for row in queued),
            "all_applied": all(row["correction_applied"] == "1" for row in applied),
        })
    return failures, summaries


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--control-dir", type=pathlib.Path, required=True)
    parser.add_argument("--shadow-dir", type=pathlib.Path, required=True)
    parser.add_argument("--output", type=pathlib.Path)
    args = parser.parse_args()

    ndt = compare_shadow_control.compare_csv(
        args.control_dir / "ndt_diagnostics.csv",
        args.shadow_dir / "ndt_diagnostics.csv")
    topics = compare_shadow_control.compare_bags(
        args.control_dir / "topics.bag", args.shadow_dir / "topics.bag")
    deskew_left = rows(args.control_dir / "deskew.csv")
    deskew_right = rows(args.shadow_dir / "deskew.csv")
    deskew_failures = compare_rows(deskew_left, deskew_right,
                                   excluded_fields=("deskew_processing_ms",))
    oosm_left = rows(args.control_dir / "oosm.csv")
    oosm_right = rows(args.shadow_dir / "oosm.csv")
    oosm_failures = compare_rows(oosm_left, oosm_right)
    fixed_left = rows(args.control_dir / "oosm_fixed_lag.csv")
    fixed_right = rows(args.shadow_dir / "oosm_fixed_lag.csv")
    fixed_failures, fixed_summary = fixed_lag_audit(fixed_left, fixed_right)

    # The historical first mismatch is explicitly reported against its
    # sensor timestamp so the regression is easy to locate in either run.
    historical_stamp = 1517157219.693280
    historical_rows = []
    for run_name, records in (("control", rows(args.control_dir / "ndt_diagnostics.csv")),
                              ("shadow", rows(args.shadow_dir / "ndt_diagnostics.csv"))):
        nearest = min(records, key=lambda row: abs(float(row["lidar_header_stamp"]) -
                                                   historical_stamp))
        historical_rows.append({"run": run_name,
                                "frame_index": nearest["frame_index"],
                                "ndt_stamp": nearest["lidar_header_stamp"],
                                "stamp_error_sec": abs(float(nearest["lidar_header_stamp"]) -
                                                        historical_stamp)})

    report = {
        "protocol": "P10_CORRIDOR01_DETERMINISTIC_OOSM_REPLAY_V2",
        "GT_LOADED": False,
        "ndt_diagnostics": ndt,
        "deskew": {"control_rows": len(deskew_left), "shadow_rows": len(deskew_right),
                   "processing_time_excluded": True,
                   "failure_count": len(deskew_failures),
                   "first_failures": deskew_failures[:20]},
        "oosm": {"control_rows": len(oosm_left), "shadow_rows": len(oosm_right),
                 "failure_count": len(oosm_failures),
                 "first_failures": oosm_failures[:20]},
        "fixed_lag": {"control_rows": len(fixed_left), "shadow_rows": len(fixed_right),
                      "runs": fixed_summary,
                      "failure_count": len(fixed_failures),
                      "first_failures": fixed_failures[:20]},
        "output_topics": topics,
        "historical_first_discrepancy_reference": {
            "old_ndt_stamp": historical_stamp,
            "nearest_v2_rows": historical_rows,
            "note": "Sensor-time fixed release is audited in fixed_lag rows; state parity is checked over all recorded output poses."
        },
    }
    report["result"] = "PASS" if all((
        ndt["failure_count"] == 0,
        len(deskew_left) == 396 and len(deskew_right) == 396 and not deskew_failures,
        len(oosm_left) == 396 and len(oosm_right) == 396 and not oosm_failures,
        len(fixed_left) == 792 and len(fixed_right) == 792 and not fixed_failures,
        topics["failure_count"] == 0,
    )) else "FAIL"
    rendered = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered)
    print(rendered, end="")
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
