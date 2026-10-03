#!/usr/bin/env python3
"""Convert the audited Corridor01 absolute-time point records to P7's packed format.

This is a data-format adapter only. It does not decode sensors, deskew points,
select an initialization, or invoke a localization algorithm.
"""

import argparse
import csv
import hashlib
from pathlib import Path

import numpy as np


SOURCE_DTYPE = np.dtype(
    [("x", "<f8"), ("y", "<f8"), ("z", "<f8"), ("intensity", "<f8"), ("stamp_ns", "<u8")]
)
P7_DTYPE = np.dtype(
    [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("offset_ns", "<u4")]
)

SOURCE_MANIFEST = "RAW_TIMED_INPUT_MANIFEST.txt"
SOURCE_POINTS = "raw_timed_points.bin"
SOURCE_CATALOG = "raw_timed_catalog.csv"
SOURCE_FILTER = "filter_scans.csv"


class AdapterError(RuntimeError):
    pass


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_manifest(path):
    values = {}
    for line in Path(path).read_text().splitlines():
        if not line or "=" not in line:
            raise AdapterError("invalid_source_manifest_line")
        key, value = line.split("=", 1)
        if key in values:
            raise AdapterError("duplicate_source_manifest_key:" + key)
        values[key] = value
    return values


def read_catalog(path):
    with Path(path).open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    required = {
        "transaction_id",
        "scan_start_ns",
        "scan_end_ns",
        "byte_offset",
        "point_count",
        "provenance",
    }
    if not rows or not required.issubset(rows[0]):
        raise AdapterError("invalid_source_catalog_header")
    return rows


def pack_points(points, scan_start_ns, scan_end_ns):
    """Preserve point order while changing only numeric representation/time origin."""
    if points.dtype != SOURCE_DTYPE:
        raise AdapterError("unexpected_source_point_record_dtype")
    xyz = np.column_stack((points["x"], points["y"], points["z"]))
    if not np.isfinite(xyz).all() or not np.isfinite(points["intensity"]).all():
        raise AdapterError("nonfinite_source_point")
    timestamps = points["stamp_ns"].astype(np.uint64, copy=False)
    if timestamps.size == 0 or np.any(timestamps < scan_start_ns) or np.any(timestamps > scan_end_ns):
        raise AdapterError("point_timestamp_outside_catalog_scan_interval")
    offsets = timestamps - np.uint64(scan_start_ns)
    if int(offsets.max()) > np.iinfo(np.uint32).max:
        raise AdapterError("relative_point_offset_exceeds_uint32_ns")

    packed = np.empty(points.size, dtype=P7_DTYPE)
    packed["x"] = points["x"].astype(np.float32)
    packed["y"] = points["y"].astype(np.float32)
    packed["z"] = points["z"].astype(np.float32)
    packed["offset_ns"] = offsets.astype(np.uint32)
    if not np.isfinite(np.column_stack((packed["x"], packed["y"], packed["z"]))).all():
        raise AdapterError("point_overflow_after_float32_conversion")
    return packed


def validate_source(source_dir):
    source_dir = Path(source_dir)
    manifest = read_manifest(source_dir / SOURCE_MANIFEST)
    expected = {
        "dataset": "SuperLoc Corridor01",
        "provenance": "RAW_TIMED_SENSOR",
        "raw_lidar_topic": "/velodyne_packets",
        "raw_lidar_message_type": "velodyne_msgs/VelodyneScan",
        "sensor_frame_id": "cmu_rc2_velodyne",
        "point_time_unit": "absolute_sensor_nanoseconds",
        "point_time_reference": "packet.stamp; packet-local scan_start_time",
        "point_time_conversion_formula": "packet.stamp.toNSec()+llround(unpack(packet,scan_start_time=packet.stamp).time*1e9)",
        "scan_start_semantic": "minimum timestamp of decoded valid range returns",
        "GT_USED": "false",
        "LEGACY_STATE_USED": "false",
        "WINDOW_STATE_USED": "false",
        "DESKEW_PERFORMED": "false",
    }
    for key, value in expected.items():
        if manifest.get(key) != value:
            raise AdapterError("source_manifest_contract_mismatch:" + key)

    source_files = {
        "raw_timed_points": SOURCE_POINTS,
        "raw_timed_catalog": SOURCE_CATALOG,
        "filter_scans": SOURCE_FILTER,
    }
    for identity, filename in source_files.items():
        if manifest.get(identity + "_file") != filename:
            raise AdapterError("source_manifest_filename_mismatch:" + identity)
        if sha256(source_dir / filename) != manifest.get(identity + "_sha256"):
            raise AdapterError("source_manifest_sha256_mismatch:" + identity)

    rows = read_catalog(source_dir / SOURCE_CATALOG)
    if len(rows) != int(manifest["scan_count"]):
        raise AdapterError("source_scan_count_mismatch")
    point_file_size = (source_dir / SOURCE_POINTS).stat().st_size
    if point_file_size % SOURCE_DTYPE.itemsize != 0:
        raise AdapterError("source_point_binary_record_alignment_error")
    if point_file_size // SOURCE_DTYPE.itemsize != int(manifest["point_count"]):
        raise AdapterError("source_point_count_mismatch")
    with (source_dir / SOURCE_FILTER).open(newline="") as stream:
        schedule = list(csv.DictReader(stream))
    if len(schedule) != len(rows):
        raise AdapterError("source_filter_count_mismatch")

    previous_end = 0
    expected_byte_offset = 0
    expected_filter = []
    for index, row in enumerate(rows, start=1):
        transaction = int(row["transaction_id"])
        start_ns = int(row["scan_start_ns"])
        end_ns = int(row["scan_end_ns"])
        byte_offset = int(row["byte_offset"])
        point_count = int(row["point_count"])
        if (
            transaction != index
            or row["provenance"] != "RAW_TIMED_SENSOR"
            or start_ns <= 0
            or end_ns <= start_ns
            or end_ns <= previous_end
            or byte_offset != expected_byte_offset
            or point_count <= 0
        ):
            raise AdapterError("invalid_source_catalog_row:" + str(index))
        if schedule[index - 1] != {
            "transaction_id": str(transaction),
            "stamp_ns": str(end_ns),
        }:
            raise AdapterError("source_filter_timestamp_mismatch:" + str(index))
        expected_filter.append((transaction, end_ns))
        expected_byte_offset += point_count * SOURCE_DTYPE.itemsize
        previous_end = end_ns
    if expected_byte_offset != point_file_size:
        raise AdapterError("source_catalog_does_not_cover_point_binary")
    return manifest, rows, expected_filter


def convert(source_dir, output_dir):
    source_dir = Path(source_dir)
    output_dir = Path(output_dir)
    manifest, rows, schedule = validate_source(source_dir)
    if output_dir.exists() and any(output_dir.iterdir()):
        raise AdapterError("refusing_nonempty_output_directory")
    output_dir.mkdir(parents=True, exist_ok=True)

    packed_path = output_dir / "p7_timed_points.bin"
    index_path = output_dir / "raw_timed_scan_index.csv"
    filter_path = output_dir / "filter_scans.csv"
    consumed_points_digest = hashlib.sha256()
    with (source_dir / SOURCE_POINTS).open("rb") as source, packed_path.open("wb") as packed_file:
        with index_path.open("w", newline="") as index_stream:
            writer = csv.writer(index_stream, lineterminator="\n")
            writer.writerow(
                ["transaction_id", "scan_start_ns", "scan_end_ns", "cloud_byte_offset", "cloud_point_count"]
            )
            output_offset = 0
            for row in rows:
                transaction = int(row["transaction_id"])
                start_ns = int(row["scan_start_ns"])
                end_ns = int(row["scan_end_ns"])
                source_offset = int(row["byte_offset"])
                point_count = int(row["point_count"])
                source.seek(source_offset)
                source_bytes = source.read(point_count * SOURCE_DTYPE.itemsize)
                if len(source_bytes) != point_count * SOURCE_DTYPE.itemsize:
                    raise AdapterError("truncated_source_scan:" + str(transaction))
                consumed_points_digest.update(source_bytes)
                points = np.frombuffer(source_bytes, dtype=SOURCE_DTYPE)
                if int(points["stamp_ns"].min()) != start_ns:
                    raise AdapterError("catalog_scan_start_not_min_point_time:" + str(transaction))
                packed = pack_points(points, start_ns, end_ns)
                packed.tofile(packed_file)
                writer.writerow([transaction, start_ns, end_ns, output_offset, point_count])
                output_offset += point_count * P7_DTYPE.itemsize

    if consumed_points_digest.hexdigest() != manifest["raw_timed_points_sha256"]:
        raise AdapterError("source_points_sha256_mismatch_during_conversion")

    with filter_path.open("w", newline="") as filter_stream:
        writer = csv.writer(filter_stream, lineterminator="\n")
        writer.writerow(["transaction_id", "stamp_ns"])
        writer.writerows(schedule)

    output_manifest = {
        "adapter": "P8 Corridor01 absolute-ns to P7 relative-ns format only",
        "source_manifest_sha256": sha256(source_dir / SOURCE_MANIFEST),
        "source_points_sha256": manifest["raw_timed_points_sha256"],
        "source_catalog_sha256": manifest["raw_timed_catalog_sha256"],
        "original_bag_sha256": manifest["original_bag_sha256"],
        "scan_count": len(rows),
        "point_count": manifest["point_count"],
        "source_record": "little-endian float64 x,y,z,intensity; uint64 absolute_sensor_nanoseconds",
        "output_record": "little-endian float32 x,y,z; uint32 offset_ns",
        "offset_contract": "point_stamp_ns - catalog_scan_start_ns; scan_start is min decoded return timestamp",
        "point_order": "preserved; no sorting, point filtering, deskew, or overlap clipping",
        "scan_end_contract": manifest["scan_end_semantic"],
        "output_points_sha256": sha256(packed_path),
        "output_index_sha256": sha256(index_path),
        "output_filter_sha256": sha256(filter_path),
        "GT_USED": "false",
    }
    with (output_dir / "P8_ADAPTER_MANIFEST.txt").open("w") as stream:
        for key, value in output_manifest.items():
            stream.write(f"{key}={value}\n")
    return output_manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    print(convert(args.source_dir, args.output_dir))


if __name__ == "__main__":
    main()
