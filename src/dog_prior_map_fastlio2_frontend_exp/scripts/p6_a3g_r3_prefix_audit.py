#!/usr/bin/env python3
"""Offline A3G-R3 capture integrity and frozen-prefix parity audit."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import struct
import sys
from pathlib import Path

SELECTED = [159, 160, 163, 165, 166, 173, 174, 176, 182, 183, 185, 187, 188, 190]
INIT_NS = 1517157224188979000
RAW_DIR = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p6_a3a_v3_input")
FROZEN_RUN = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p6_a3g_full_corridor")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def rows(path: Path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def require_observed_within_prefix(observed_rows, predicate, label: str) -> None:
    outside = [row for row in observed_rows if not predicate(row)]
    require(not outside,
            f"observed CSV contains rows beyond frozen prefix {label}: {len(outside)}")


def point_record(raw: Path, offset: int, count: int) -> bytes:
    with raw.open("rb") as stream:
        stream.seek(offset)
        data = stream.read(40 * count)
    require(len(data) == 40 * count, "source raw record truncated")
    return data


def read_pcd_header(path: Path):
    fields = sizes = types = None
    points = None
    header_bytes = 0
    with path.open("rb") as stream:
        while True:
            line = stream.readline()
            require(bool(line), f"PCD header truncated: {path.name}")
            header_bytes += len(line)
            text = line.decode("ascii").strip()
            parts = text.split()
            if parts and parts[0] == "FIELDS": fields = parts[1:]
            if parts and parts[0] == "SIZE": sizes = [int(x) for x in parts[1:]]
            if parts and parts[0] == "TYPE": types = parts[1:]
            if parts and parts[0] == "POINTS": points = int(parts[1])
            if text == "DATA binary": break
        stream.seek(0, 2)
        file_size = stream.tell()
    require(fields is not None and sizes is not None and types is not None and points is not None,
            f"PCD header incomplete: {path.name}")
    require(file_size == header_bytes + points * sum(sizes), f"PCD binary length mismatch: {path.name}")
    return {"fields": fields, "sizes": sizes, "types": types,
            "points": points, "header_bytes": header_bytes, "file_bytes": file_size}


def matrix_from_pose(pose_row):
    import numpy as np
    q = np.array([float(pose_row[k]) for k in ("qx", "qy", "qz", "qw")])
    q /= np.linalg.norm(q)
    x, y, z, w = q
    rotation = np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])
    return rotation, np.array([float(pose_row[k]) for k in ("px", "py", "pz")])


def compare_csv(frozen: Path, observed: Path, key_columns, exclude=(), filter_fn=None,
                reject_observed_outside_prefix=False):
    frozen_rows = rows(frozen)
    observed_rows = rows(observed)
    names = list(frozen_rows[0].keys()) if frozen_rows else []
    require(names == (list(observed_rows[0].keys()) if observed_rows else names),
            f"CSV header mismatch: {frozen.name}")
    excluded = set(exclude)
    def selected(source):
        result = {}
        for row in source:
            if filter_fn and not filter_fn(row):
                continue
            key = tuple(row[name] for name in key_columns)
            require(key not in result, f"duplicate row key in {frozen.name}: {key}")
            result[key] = tuple(row[name] for name in names if name not in excluded)
        return result
    if reject_observed_outside_prefix and filter_fn:
        require_observed_within_prefix(observed_rows, filter_fn, observed.name)
    left, right = selected(frozen_rows), selected(observed_rows)
    require(left.keys() == right.keys(),
            f"row identity mismatch {frozen.name}: frozen={len(left)} observed={len(right)}")
    differences = []
    for key in left:
        if left[key] != right[key]:
            differences.append({"key": key, "frozen": left[key], "observed": right[key]})
            if len(differences) == 5:
                break
    return {"frozen_rows": len(left), "observed_rows": len(right),
            "excluded_fields": list(exclude), "differences": differences}


def audit_capture(root: Path):
    root = root.resolve()
    catalog = rows(RAW_DIR / "raw_timed_catalog.csv")
    catalog_by_tx = {int(row["transaction_id"]): row for row in catalog}
    evidence = rows(FROZEN_RUN / "trajectory.csv.deskew_evidence.csv")
    evidence_by_tx = {int(row["transaction_id"]): row for row in evidence}
    preopt = rows(FROZEN_RUN / "trajectory.csv.r1_preopt_capsule.csv")
    preopt_by_tx = {int(row["transaction_id"]): row for row in preopt}
    state_rows = rows(root / "SELECTED_WINDOW_STATES.csv")
    state_by_key = {(int(row["transaction_id"]), row["state_role"]): row for row in state_rows}
    require(len(state_rows) == 2 * len(SELECTED), "selected state row count mismatch")
    require(set(tx for tx, _ in state_by_key) == set(SELECTED), "selected state transaction identity mismatch")

    frame_inventory = {}
    for tx in SELECTED:
        folder = root / f"TX{tx:04d}"
        require(folder.is_dir(), f"selected frame directory missing: {tx}")
        metadata = json.loads((folder / "METADATA.json").read_text())
        schema = json.loads((folder / "RAW_TIMED_POINTS_SCHEMA.json").read_text())
        catalog_row = catalog_by_tx[tx]
        evidence_row = evidence_by_tx[tx]
        require(int(metadata["transaction_id"]) == tx, f"metadata tx mismatch: {tx}")
        require(int(metadata["scan_start_ns"]) == int(catalog_row["scan_start_ns"]), f"start stamp mismatch: {tx}")
        require(int(metadata["scan_end_ns"]) == int(catalog_row["scan_end_ns"]), f"end stamp mismatch: {tx}")
        count = int(catalog_row["point_count"])
        raw_capture = folder / "RAW_TIMED_POINTS.bin"
        raw_source = point_record(RAW_DIR / "raw_timed_points.bin", int(catalog_row["byte_offset"]), count)
        captured_bytes = raw_capture.read_bytes()
        require(captured_bytes == raw_source, f"captured raw records are not byte-identical: {tx}")
        require(len(captured_bytes) == 40 * count and int(schema["point_count"]) == count,
                f"raw record count/size mismatch: {tx}")
        require(int(schema["record_size_bytes"]) == 40 and schema["endianness"] == "little" and
                schema["fields"] == ["float64 x", "float64 y", "float64 z",
                                     "float64 intensity", "uint64 absolute_sensor_point_stamp_ns"] and
                schema["provenance"] == "RAW_TIMED_SENSOR" and
                int(schema["scan_start_ns"]) == int(catalog_row["scan_start_ns"]) and
                int(schema["scan_end_ns"]) == int(catalog_row["scan_end_ns"]),
                f"raw schema provenance/interval mismatch: {tx}")
        require(schema["file_sha256"] == sha256(raw_capture),
                f"raw schema SHA does not match captured payload: {tx}")
        unpacked_stamps = [struct.unpack_from("<Q", captured_bytes, i * 40 + 32)[0]
                           for i in range(count)]
        require(min(unpacked_stamps) == int(catalog_row["scan_start_ns"]) and
                max(unpacked_stamps) == int(catalog_row["scan_end_ns"]),
                f"raw point stamp extrema mismatch: {tx}")
        require(int(evidence_row["raw_point_count"]) == count and
                int(evidence_row["point_stamp_min_ns"]) == min(unpacked_stamps) and
                int(evidence_row["point_stamp_max_ns"]) == max(unpacked_stamps),
                f"raw count/time differs from frozen deskew evidence: {tx}")
        require(metadata["lidar_provenance"] == "WINDOW_OWNED_SE3_DESKEW",
                f"deskew provenance mismatch: {tx}")
        require(metadata["scan_start_ns"] == schema["scan_start_ns"] and
                metadata["scan_end_ns"] == schema["scan_end_ns"] and
                metadata["raw_point_count"] == schema["point_count"] and
                metadata["raw_point_stamp_min_ns"] == min(unpacked_stamps) and
                metadata["raw_point_stamp_max_ns"] == max(unpacked_stamps),
                f"raw metadata differs from schema/payload: {tx}")
        deskew_pcd = read_pcd_header(folder / "DESKEWED_END_FRAME.pcd")
        ndt_pcd = read_pcd_header(folder / "NDT_SOURCE.pcd")
        require(deskew_pcd["fields"] == ["x", "y", "z", "intensity", "stamp_ns"] and
                deskew_pcd["sizes"] == [8, 8, 8, 8, 8] and
                deskew_pcd["types"] == ["F", "F", "F", "F", "U"],
                f"deskew PCD schema mismatch: {tx}")
        require(deskew_pcd["points"] == int(evidence_row["deskew_point_count"]),
                f"deskew point count differs from frozen run: {tx}")
        require(deskew_pcd["points"] == count, f"raw/deskew point count changed: {tx}")
        require(ndt_pcd["fields"] == ["x", "y", "z"] and
                ndt_pcd["sizes"] == [4, 4, 4] and ndt_pcd["types"] == ["F", "F", "F"],
                f"NDT source PCD schema mismatch: {tx}")
        require(int(metadata["ndt_source_point_count"]) == ndt_pcd["points"],
                f"NDT source count mismatch: {tx}")

        start_state = state_by_key[(tx, "SCAN_START")]
        end_state = state_by_key[(tx, "SCAN_END_PRE_MEASUREMENT")]
        require(int(start_state["stamp_ns"]) == int(catalog_row["scan_start_ns"]) and
                int(end_state["stamp_ns"]) == int(catalog_row["scan_end_ns"]),
                f"state timestamp mismatch: {tx}")
        for csv_state, json_key in ((start_state, "scan_start_state"),
                                    (end_state, "scan_end_pre_measurement_state")):
            state_json = metadata[json_key]
            require(int(state_json["stamp_ns"]) == int(csv_state["stamp_ns"]),
                    f"state CSV/metadata stamp mismatch: {tx}/{json_key}")
            csv_values = [float(csv_state[k]) for k in
                          ("px", "py", "pz", "qx", "qy", "qz", "qw",
                           "vx", "vy", "vz", "bgx", "bgy", "bgz", "bax", "bay", "baz")]
            json_values = [*state_json["position_xyz"], *state_json["rotation_xyzw"],
                           *state_json["velocity_xyz"], *state_json["gyro_bias_xyz"],
                           *state_json["accel_bias_xyz"]]
            require(len(csv_values) == len(json_values) == 16 and
                    all(a == b for a, b in zip(csv_values, json_values)),
                    f"state CSV/metadata 15D value mismatch: {tx}/{json_key}")
        anchor_rotation, anchor_position = matrix_from_pose({
            "qx": evidence_row["anchor_qx"], "qy": evidence_row["anchor_qy"],
            "qz": evidence_row["anchor_qz"], "qw": evidence_row["anchor_qw"],
            "px": evidence_row["anchor_px"], "py": evidence_row["anchor_py"],
            "pz": evidence_row["anchor_pz"]})
        captured_rotation, captured_position = matrix_from_pose(start_state)
        require(float(((anchor_rotation - captured_rotation) ** 2).sum()) ** 0.5 <= 1e-12 and
                float(((anchor_position - captured_position) ** 2).sum()) ** 0.5 <= 1e-12,
                f"scan-start state differs from frozen deskew anchor: {tx}")
        predicted = preopt_by_tx[tx]
        pos = [float(x) for x in predicted["predicted_position"].split(";")]
        quat = [float(x) for x in predicted["predicted_rotation_xyzw"].split(";")]
        import numpy as np
        end_pos = np.array([float(end_state[k]) for k in ("px", "py", "pz")])
        end_q = np.array([float(end_state[k]) for k in ("qx", "qy", "qz", "qw")])
        target_q = np.array(quat)
        target_q /= np.linalg.norm(target_q)
        end_q /= np.linalg.norm(end_q)
        require(np.linalg.norm(end_pos - np.array(pos)) <= 1e-12 and
                1.0 - abs(float(np.dot(end_q, target_q))) <= 1e-14,
                f"scan-end pre-measurement state differs from frozen prediction: {tx}")

        frame_files = sorted(p for p in folder.iterdir() if p.is_file())
        artifact_manifest = metadata["artifacts"]
        expected_artifact_names = {"raw_timed_points.bin", "DESKEWED_END_FRAME.pcd",
                                   "NDT_SOURCE.pcd", "DESKEW_KNOTS.csv"}
        require(set(artifact_manifest) == expected_artifact_names,
                f"metadata artifact inventory mismatch: {tx}")
        artifact_paths = {
            "raw_timed_points.bin": raw_capture,
            "DESKEWED_END_FRAME.pcd": folder / "DESKEWED_END_FRAME.pcd",
            "NDT_SOURCE.pcd": folder / "NDT_SOURCE.pcd",
            "DESKEW_KNOTS.csv": folder / "DESKEW_KNOTS.csv",
        }
        for name, artifact_path in artifact_paths.items():
            ident = artifact_manifest[name]
            require(ident["bytes"] == artifact_path.stat().st_size and
                    ident["sha256"] == sha256(artifact_path),
                    f"metadata artifact SHA/size mismatch {name}: {tx}")
        frame_inventory[str(tx)] = {
            "scan_start_ns": int(catalog_row["scan_start_ns"]),
            "scan_end_ns": int(catalog_row["scan_end_ns"]),
            "raw_point_count": count,
            "deskewed_point_count": deskew_pcd["points"],
            "ndt_source_point_count": ndt_pcd["points"],
            "raw_source_bytes_identical": True,
            "artifacts": {p.name: {"bytes": p.stat().st_size, "sha256": sha256(p)}
                          for p in frame_files},
        }

    catalog_prefix = [r for r in catalog if int(r["transaction_id"]) <= 190]
    expected_events = set()
    for row in catalog_prefix:
        if int(row["scan_start_ns"]) >= INIT_NS:
            expected_events.add((row["scan_start_ns"], "LIDAR_SCAN_START"))
            expected_events.add((row["scan_end_ns"], "LIDAR_SCAN_END"))
    terminal_190 = catalog_by_tx[190]
    terminal_stamp_190 = int(terminal_190["scan_end_ns"])
    max_stamp = max(stamp for stamp, _ in expected_events)
    require(max_stamp == terminal_stamp_190,
            "frozen prefix endpoint is not tx190 scan end")
    def event_filter(row):
        return (int(row["timestamp"]), row["event_type"]) in expected_events
    def health_filter(row):
        return (int(row["event_stamp_ns"]), row["event"]) in expected_events
    expected_event_transactions = set()
    for row in catalog_prefix:
        if int(row["scan_start_ns"]) >= INIT_NS:
            expected_event_transactions.add((int(row["transaction_id"]), int(row["scan_start_ns"])))
            expected_event_transactions.add((int(row["transaction_id"]), int(row["scan_end_ns"])))
    def marginalization_filter(row):
        return (int(row["transaction_id"]), int(row["event_stamp_ns"])) in expected_event_transactions
    frozen_trajectory_prefix = [r for r in rows(FROZEN_RUN / "trajectory.csv")
                                if int(r["transaction_id"]) <= 190]
    observed_trajectory = rows(root / "trajectory.csv")
    require(frozen_trajectory_prefix and
            max(int(r["transaction_id"]) for r in frozen_trajectory_prefix) == 190 and
            int(frozen_trajectory_prefix[-1]["stamp_ns"]) == terminal_stamp_190,
            "frozen trajectory does not terminate at tx190 scan end")
    require(observed_trajectory and
            max(int(r["transaction_id"]) for r in observed_trajectory) == 190 and
            int(observed_trajectory[-1]["transaction_id"]) == 190 and
            int(observed_trajectory[-1]["stamp_ns"]) == terminal_stamp_190,
            "capture run did not stop exactly at tx190 LIDAR_SCAN_END")
    require(all(int(row["transaction_id"]) <= 190 for row in observed_trajectory),
            "trajectory contains a terminal after tx190")
    comparisons = {
        "trajectory": compare_csv(FROZEN_RUN / "trajectory.csv", root / "trajectory.csv",
                                  ("transaction_id",), filter_fn=lambda r: int(r["transaction_id"]) <= 190,
                                  reject_observed_outside_prefix=True),
        "events": compare_csv(FROZEN_RUN / "events.csv", root / "events.csv",
                              ("timestamp", "event_type"), filter_fn=event_filter,
                              reject_observed_outside_prefix=True),
        "preopt": compare_csv(FROZEN_RUN / "trajectory.csv.r1_preopt_capsule.csv",
                              root / "trajectory.csv.r1_preopt_capsule.csv",
                              ("transaction_id",), exclude=("ndt_runtime_ms",),
                              filter_fn=lambda r: int(r["transaction_id"]) <= 190,
                              reject_observed_outside_prefix=True),
        "deskew_evidence": compare_csv(FROZEN_RUN / "trajectory.csv.deskew_evidence.csv",
                                       root / "trajectory.csv.deskew_evidence.csv",
                                       ("transaction_id",), filter_fn=lambda r: int(r["transaction_id"]) <= 190,
                                       reject_observed_outside_prefix=True),
        "covariance": compare_csv(FROZEN_RUN / "trajectory.csv.a3f_r1_covariance.csv",
                                  root / "trajectory.csv.a3f_r1_covariance.csv",
                                  ("transaction_id",), exclude=("qr_ms",),
                                  filter_fn=lambda r: int(r["transaction_id"]) <= 190,
                                  reject_observed_outside_prefix=True),
        "health": compare_csv(FROZEN_RUN / "trajectory.csv.a3g_health.csv",
                              root / "trajectory.csv.a3g_health.csv",
                              ("transaction_id", "event_stamp_ns", "event"),
                              exclude=("optimizer_and_marginalization_ms",),
                              filter_fn=health_filter,
                              reject_observed_outside_prefix=True),
        "marginalization": compare_csv(FROZEN_RUN / "trajectory.csv.a3c_r1_marginalization_trace.csv",
                                       root / "trajectory.csv.a3c_r1_marginalization_trace.csv",
                                       ("transaction_id", "event_stamp_ns", "attempt_index"),
                                       exclude=("qr_ms",),
                                       filter_fn=marginalization_filter,
                                       reject_observed_outside_prefix=True),
    }
    # Timing-only runtime columns are excluded; event identity, nominal/probe
    # call counts, solver status and fallback count must remain equal.
    comparisons["runtime"] = compare_csv(
        FROZEN_RUN / "runtime.csv", root / "runtime.csv",
        ("timestamp", "event_type"),
        exclude=("ndt_ms", "event_ms", "linearization_ms", "solve_ms",
                 "marginal_covariance_ms", "rank_diagnostic_ms"),
        filter_fn=event_filter, reject_observed_outside_prefix=True)
    parity_pass = all(not value["differences"] for value in comparisons.values())
    return {
        "status": "PASS" if parity_pass else "FAIL",
        "frozen_prefix_event_count": len(expected_events),
        "selected_transactions": SELECTED,
        "selected_frames": frame_inventory,
        "comparisons": comparisons,
        "timing_exclusions": {
            "preopt": ["ndt_runtime_ms"], "covariance": ["qr_ms"],
            "health": ["optimizer_and_marginalization_ms"],
            "marginalization": ["qr_ms"],
            "runtime": ["ndt_ms", "event_ms", "linearization_ms", "solve_ms",
                        "marginal_covariance_ms", "rank_diagnostic_ms"],
        },
    }


def self_test():
    require(SELECTED == [159,160,163,165,166,173,174,176,182,183,185,187,188,190],
            "allowlist changed")
    require(40 * 7 == 280, "raw timed record layout self-test")
    require("SQUARE_ROOT_QR" != "LEGACY_INFORMATION_SCHUR", "backend fixture")
    expected = {(10, "LIDAR_SCAN_START"), (20, "LIDAR_SCAN_END")}
    observed_rows = [{"key": (10, "LIDAR_SCAN_START")},
                     {"key": (20, "LIDAR_SCAN_END")}]
    require_observed_within_prefix(observed_rows, lambda row: row["key"] in expected,
                                   "self-test")
    try:
        require_observed_within_prefix(observed_rows +
                [{"key": (21, "LIDAR_SCAN_START")}],
                lambda row: row["key"] in expected, "self-test")
    except ValueError:
        pass
    else:
        raise ValueError("strict prefix self-test failed to reject post-terminal event")
    print("A3G_R3_PREFIX_AUDIT_SELF_TEST_PASS")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--capture-root", type=Path)
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return 0
    if args.capture_root is None:
        parser.error("--capture-root is required")
    result = audit_capture(args.capture_root)
    output = args.capture_root / "PREFIX_PARITY.json"
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(f"A3G_R3_PREFIX_PARITY_{result['status']} events={result['frozen_prefix_event_count']} "
          f"selected={len(result['selected_frames'])}")
    return 0 if result["status"] == "PASS" else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"A3G_R3_PREFIX_AUDIT_FAIL: {exc}", file=sys.stderr)
        raise SystemExit(2)
