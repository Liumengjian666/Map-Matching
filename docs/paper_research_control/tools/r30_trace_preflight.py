#!/usr/bin/env python3
"""Read-only cross-policy chronology for the archived R29 causal trajectories.

This is an evidence preflight, NOT a replay, NDT fit, evaluation oracle, or
root-cause classifier. It reads every trajectory row, retains the failure
intervals, and identifies which recorded quantities diverged first.

Run:
 python3 docs/paper_research_control/tools/r30_trace_preflight.py --json /tmp/r30.json

Default inputs are committed R29 derived trajectory CSVs. This program
never reads GT to initialize a state or changes NDT/EKF parameters.
"""
import argparse
import csv
import json
from pathlib import Path

METHODS = "ABCD"
STATE_FIELDS = (
    "scan_stamp_ns", "state_stamp_ns", "px", "py", "pz", "qx", "qy",
    "qz", "qw", "map_published", "ekf_update_applied",
)
REQUIRED_FIELDS = set(STATE_FIELDS) | {
    "raw_ordinal", "reference_valid", "reference_high_error", "reason",
    "translation_error_m", "rotation_error_deg", "extra_calls",
}


def read_rows(path):
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames:
            raise ValueError("empty header: " + str(path))
        missing = REQUIRED_FIELDS - set(reader.fieldnames)
        if missing:
            raise ValueError("missing required fields " + ",".join(sorted(missing)) + " in " + str(path))
        rows = list(reader)
    if not rows:
        raise ValueError("no data rows: " + str(path))
    previous = None
    for row in rows:
        ordinal = int(row["raw_ordinal"])
        if previous is not None and ordinal != previous + 1:
            raise ValueError("nonconsecutive raw ordinal in " + str(path))
        previous = ordinal
    return rows


def true_value(value):
    return str(value).strip().lower() in ("1", "true", "yes")


def first_row(rows, predicate):
    return next((int(row["raw_ordinal"]) for row in rows if predicate(row)), None)


def profile(rows):
    valid = [r for r in rows if true_value(r["reference_valid"])]
    high = [r for r in valid if true_value(r["reference_high_error"])]
    observed = [r for r in rows if true_value(r["map_published"])]
    fused = [r for r in rows if true_value(r["ekf_update_applied"])]
    extras = sum(int(r["extra_calls"]) for r in rows)
    return {
        "input_rows": len(rows),
        "raw_first": int(rows[0]["raw_ordinal"]),
        "raw_last": int(rows[-1]["raw_ordinal"]),
        "valid_reference_rows": len(valid),
        "unavailable_reference_rows": [int(r["raw_ordinal"]) for r in rows if not true_value(r["reference_valid"])],
        "first_reference_high_error_raw": int(high[0]["raw_ordinal"]) if high else None,
        "first_map_observation_raw": int(observed[0]["raw_ordinal"]) if observed else None,
        "last_map_observation_raw": int(observed[-1]["raw_ordinal"]) if observed else None,
        "first_ekf_fusion_raw": int(fused[0]["raw_ordinal"]) if fused else None,
        "last_ekf_fusion_raw": int(fused[-1]["raw_ordinal"]) if fused else None,
        "map_observations": len(observed),
        "actual_ekf_updates": len(fused),
        "extra_ndt_calls": extras,
        "first_no_native_raw": first_row(rows, lambda r: r["reason"] == "NO_NATIVE_RESULT"),
        "first_rejected_quality_raw": first_row(rows, lambda r: r["reason"] == "NO_ACCEPTABLE_MAP_MEASUREMENT"),
        "first_stale_prediction_raw": first_row(rows, lambda r: "STALE" in r["reason"]),
    }


def divergence(left_rows, right_rows):
    if len(left_rows) != len(right_rows):
        raise ValueError("different number of rows; cannot join policies by row index")
    for a, b in zip(left_rows, right_rows):
        if a["raw_ordinal"] != b["raw_ordinal"] or a["scan_stamp_ns"] != b["scan_stamp_ns"]:
            raise ValueError("policy inputs are not ordinal/time aligned")
        changed = [name for name in STATE_FIELDS if a[name] != b[name]]
        if changed:
            return {"raw_ordinal": int(a["raw_ordinal"]), "different_state_fields": changed}
    return None


def audit(archive):
    paths = {m: archive / "FULL_SEQUENCE_TRAJECTORIES" / ("EASY_C0_" + m + ".csv") for m in METHODS}
    traces = {m: read_rows(p) for m, p in paths.items()}
    summaries = {m: profile(traces[m]) for m in METHODS}
    for name in ("A", "B", "D"):
        if summaries[name]["raw_first"] != summaries["C"]["raw_first"] or summaries[name]["raw_last"] != summaries["C"]["raw_last"]:
            raise ValueError("different pre-registered sequence bounds")
    return {
        "tool_scope": "READ_ONLY_R29_DERIVED_LOGS_NOT_A_NEW_EXPERIMENT",
        "source_archive": str(archive),
        "summary": summaries,
        "first_state_divergence": {
            "A_vs_B": divergence(traces["A"], traces["B"]),
            "B_vs_C": divergence(traces["B"], traces["C"]),
            "B_vs_D": divergence(traces["B"], traces["D"]),
        },
        "interpretation_limits": [
            "First output divergence is not necessarily the first causal root cause.",
            "Reference error is a map-anchored batch-reference proxy, not independent mocap.",
            "IMU interval gaps, local-map selection and native candidate quality need raw run logs.",
            "This tool does not repair, replay or certify any localization method.",
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    default = Path(__file__).resolve().parents[2] / "p62_r29_full_sequence_benchmark"
    parser.add_argument("--archive", type=Path, default=default)
    parser.add_argument("--json", type=Path, help="write a NEW JSON file; do not overwrite")
    args = parser.parse_args()
    result = audit(args.archive.resolve())
    payload = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.json:
        with args.json.open("x", encoding="utf-8") as target:
            target.write(payload)
    print(payload, end="")


if __name__ == "__main__":
    main()
