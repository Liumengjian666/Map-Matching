"""Read-only real-input test; no registration, extraction or point reordering."""
import csv
import json
import pathlib
import sys
import numpy as np
from prepare_v2 import INPUT, ARCHIVE, OUTPUT, sha, write_csv

def check():
    config = json.loads((ARCHIVE / "bootstrap_v2_config.json").read_text())
    scans = list(csv.DictReader((INPUT / "raw_timed_scan_index.csv").open()))
    filters = list(csv.DictReader((INPUT / "filter_scans.csv").open()))
    imu = list(csv.DictReader((INPUT / "imu.csv").open()))
    times = np.array([int(r["stamp_ns"]) for r in imu], dtype=np.int64)
    if len(scans) != 2777 or len(imu) != 55957 or np.any(np.diff(times) <= 0):
        raise RuntimeError("raw frame/IMU contract changed")
    if len(filters) != len(scans) or any(f["transaction_id"] != s["transaction_id"] or f["stamp_ns"] != s["scan_end_ns"] for f, s in zip(filters, scans)):
        raise RuntimeError("filter/raw scan alignment mismatch")
    layout = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("offset_ns", "<u4")])
    if layout.itemsize != 16:
        raise RuntimeError("packed raw layout mismatch")
    receipts = []
    with (INPUT / "raw_timed_points.bin").open("rb") as stream:
        for scan in scans:
            if int(scan["scan_end_ns"]) > config["bootstrap_end_ns"]:
                break
            start, end = int(scan["scan_start_ns"]), int(scan["scan_end_ns"])
            stream.seek(int(scan["cloud_byte_offset"]))
            raw = np.fromfile(stream, dtype=layout, count=int(scan["cloud_point_count"]))
            if len(raw) != int(scan["cloud_point_count"]) or not all(np.all(np.isfinite(raw[k])) for k in ("x", "y", "z")):
                raise RuntimeError("invalid or incomplete raw point block")
            if len(raw) == 0 or int(raw["offset_ns"].max()) > end-start:
                raise RuntimeError("point timestamp outside scan")
            first_used = max(start, int(times[0]))
            right = int(np.searchsorted(times, end, side="right"))
            left = max(0, int(np.searchsorted(times[:right], first_used, side="right"))-1)
            observed = times[left:right]
            if len(observed) < 2 or end-int(observed[-1]) > 10_000_000 or np.max(np.diff(observed)) > 20_000_000:
                raise RuntimeError("causal scan IMU timing guard")
            prefix = int(np.count_nonzero(raw["offset_ns"].astype(np.int64)+start < times[0]))
            if prefix and int(scan["transaction_id"]) != 1:
                raise RuntimeError("missing IMU outside declared TX1 boundary")
            receipts.append({"transaction_id": scan["transaction_id"], "scan_start_ns": start, "scan_end_ns": end,
                "raw_points": len(raw), "packed_point_bytes": layout.itemsize,
                "unsupported_TX1_prefix_points": prefix, "imu_samples_not_later_than_end": len(observed),
                "maximum_observed_imu_gap_ns": int(np.diff(observed).max()),
                "endpoint_hold_ns": end-int(observed[-1]), "point_order": "UNCHANGED", "status": "PASS"})
    if len(receipts) != 99 or len(filters) != len(scans):
        raise RuntimeError("incorrect raw bootstrap/filter count")
    write_csv(ARCHIVE / "real_input_preflight.csv", receipts)
    write_csv(OUTPUT / "real_input_preflight.csv", receipts)
    print(json.dumps({"real_input_tests": "PASS", "bootstrap_scans": len(receipts),
        "TX1_unsupported_prefix_points": receipts[0]["unsupported_TX1_prefix_points"], "GICP_calls": 0, "NDT_calls": 0}))

if __name__ == "__main__":
    check()
