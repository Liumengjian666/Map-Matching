#!/usr/bin/env python3
"""Fail-closed, rate-limited, read-only source mirror for P59 inputs.

The only source filesystem is the approved HIKVISION mount. All outputs are
written beneath the fixed internal-disk destination. Source bytes are hashed
in the same streaming pass used for the copy; destination bytes are re-hashed
from the internal disk. The script never opens a source path for writing.
"""

from __future__ import annotations

import csv
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path


WORKSPACE = Path("/home/jian/livox_ws/p59_newer_college_full_sequence_ws")
DATA_ROOT = Path("/home/jian/livox_ws/p59_newer_college_full_sequence_data")
MOUNT = Path("/media/jian/HIKVISION")
UUID = "F627-4133"
DATASET_ROOT = MOUNT / "paper rosbag/NewerCollege/01_short_experiment"
SCAN_ROOT = DATASET_ROOT / "raw_format/ouster_scan"
TILE_ROOT = MOUNT / "paper rosbag/NewerCollege/.p45_local_map/tiles"
P46_ROOT = MOUNT / "paper rosbag/NewerCollege/.p46_real_tile_switch/formal1800"
P40_MANIFEST = WORKSPACE / "docs/p40_newer_college_download/download_manifest.json"
P45_TILE_MANIFEST = WORKSPACE / "docs/p45_newer_college_local_map/tile_manifest.json"
P45_RESULTS = WORKSPACE / "docs/p45_newer_college_local_map/results.json"
P46_INPUT_HASHES = WORKSPACE / "docs/p46_newer_college_real_tile_switch/input_scan_sha256.csv"
P46_RUN_MANIFEST = WORKSPACE / "docs/p46_newer_college_real_tile_switch/run_frozen_manifest.json"

EXPECTED_SCANS = 15301
MAX_RATE_BYTES_S = 2 * 1024 * 1024
CHECKPOINT_BYTES = 64 * 1024 * 1024
CHUNK_BYTES = 1024 * 1024
CANONICAL_SCAN = re.compile(r"cloud_(\d+)_(\d{9})\.pcd\Z")
SENSOR_RE = re.compile(
    r"usb reset|usb disconnect|uas.*(error|abort|timeout)|"
    r"I/O error|Buffer I/O|device offline|exfat.*(error|fail)|"
    r"lost async page write|blocked for more than",
    re.IGNORECASE,
)


@dataclass(frozen=True)
class InputFile:
    source: Path
    relative_path: Path
    expected_sha256: str | None = None
    scan_index: int | None = None
    stamp_ns: int | None = None


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def enumerate_scans(scan_root: Path, expected_count: int = EXPECTED_SCANS) -> tuple[list[tuple[int, Path, int]], list[str]]:
    scans: list[tuple[int, Path, int]] = []
    ignored: list[str] = []
    seen: set[int] = set()
    with os.scandir(scan_root) as entries:
        for entry in entries:
            if not entry.name.startswith("cloud_") or not entry.name.endswith(".pcd"):
                continue
            match = CANONICAL_SCAN.fullmatch(entry.name)
            if not match:
                ignored.append(entry.name)
                continue
            if entry.is_symlink() or not entry.is_file(follow_symlinks=False):
                raise RuntimeError(f"non-regular canonical scan: {entry.name}")
            stamp = int(match.group(1)) * 1_000_000_000 + int(match.group(2))
            if stamp in seen:
                raise RuntimeError(f"duplicate canonical scan timestamp: {stamp}")
            seen.add(stamp)
            scans.append((stamp, Path(entry.path), entry.stat(follow_symlinks=False).st_size))
    scans.sort(key=lambda row: row[0])
    if len(scans) != expected_count:
        raise RuntimeError(f"expected {expected_count} canonical scans, found {len(scans)}")
    if not scans or any(scans[i][0] >= scans[i + 1][0] for i in range(len(scans) - 1)):
        raise RuntimeError("scan timestamps are not strictly increasing")
    gaps = [scans[i + 1][0] - scans[i][0] for i in range(len(scans) - 1)]
    if gaps and max(gaps) > 150_000_000:
        raise RuntimeError(f"scan sequence has a >150 ms timestamp gap: {max(gaps)} ns")
    return scans, ignored


def _manifest_file_map(manifest: dict) -> dict[str, dict]:
    return {str(item["path"]): item for item in manifest["files"]}


def prepare_inputs() -> tuple[list[InputFile], dict]:
    with P40_MANIFEST.open(encoding="utf-8") as stream:
        p40 = json.load(stream)
    p40_files = _manifest_file_map(p40)
    with P45_TILE_MANIFEST.open(encoding="utf-8") as stream:
        p45 = json.load(stream)
    with P45_RESULTS.open(encoding="utf-8") as stream:
        p45_results = json.load(stream)
    with P46_INPUT_HASHES.open(newline="", encoding="utf-8") as stream:
        p46_rows = list(csv.DictReader(stream))
    with P46_RUN_MANIFEST.open(encoding="utf-8") as stream:
        p46_run = json.load(stream)

    expected_p46 = {int(row["scan_index"]): row for row in p46_rows}
    if len(expected_p46) != 1800 or sorted(expected_p46) != list(range(1, 1801)):
        raise RuntimeError("P46 input hash reference must contain exactly frames 1..1800")

    scans, ignored = enumerate_scans(SCAN_ROOT)
    unexpected_sizes = sorted({size for _, _, size in scans if size != 1_904_640})
    if unexpected_sizes:
        raise RuntimeError(f"canonical PCD byte sizes differ from the P40/P46 frozen schema: {unexpected_sizes}")
    inputs: list[InputFile] = []
    scan_bytes = 0
    for index, (stamp, source, size) in enumerate(scans, 1):
        expected = None
        if index <= 1800:
            ref = expected_p46[index]
            if int(ref["stamp_ns"]) != stamp or Path(ref["pcd_path"]).name != source.name:
                raise RuntimeError(f"P46 frozen scan identity mismatch at frame {index}")
            if int(ref["size_bytes"]) != size:
                raise RuntimeError(f"P46 frozen scan size mismatch at frame {index}")
            expected = ref["sha256"]
        inputs.append(InputFile(source, Path("dataset/01_short_experiment/raw_format/ouster_scan") / source.name,
                                expected, index, stamp))
        scan_bytes += size

    for relative, destination in (
        ("01_short_experiment/raw_format/ouster_imu/data.csv", Path("dataset/01_short_experiment/raw_format/ouster_imu/data.csv")),
        ("01_short_experiment/ground_truth/registered_poses.csv", Path("dataset/01_short_experiment/ground_truth/registered_poses.csv")),
        ("01_short_experiment/time_offsets/time_offsets.csv", Path("dataset/01_short_experiment/time_offsets/time_offsets.csv")),
    ):
        item = p40_files.get(relative)
        if item is None:
            raise RuntimeError(f"P40 verified download manifest missing {relative}")
        source = MOUNT / "paper rosbag/NewerCollege" / relative
        if source.stat().st_size != item["bytes"]:
            raise RuntimeError(f"P40 verified asset size mismatch: {source}")
        inputs.append(InputFile(source, destination, item["sha256"]))

    tile_hashes = {Path(tile["tile_path"]).name: tile["sha256"] for tile in p45["tiles"]}
    with os.scandir(TILE_ROOT) as entries:
        tile_files = sorted((Path(entry.path) for entry in entries if entry.is_file(follow_symlinks=False)),
                            key=lambda path: path.name)
    expected_tile_names = set(tile_hashes) | {"tile_manifest.json"}
    if {path.name for path in tile_files} != expected_tile_names:
        raise RuntimeError("P45 tile directory contents differ from the frozen tile manifest")
    for source in tile_files:
        expected = tile_hashes.get(source.name)
        if source.name == "tile_manifest.json":
            expected = p45_results["map"]["tile_manifest_sha256"]
        inputs.append(InputFile(source, Path("map_tiles/original") / source.name, expected))

    p46_runtime = {Path(item["path"]).name: item for item in p46_run["runtime_outputs"]}
    with os.scandir(P46_ROOT) as entries:
        p46_files = sorted((Path(entry.path) for entry in entries if entry.is_file(follow_symlinks=False)),
                           key=lambda path: path.name)
    if {path.name for path in p46_files} != set(p46_runtime):
        raise RuntimeError("P46 runtime output files differ from its frozen output manifest")
    for source in p46_files:
        item = p46_runtime[source.name]
        if source.stat().st_size != item["size_bytes"]:
            raise RuntimeError(f"P46 runtime output size differs from its frozen manifest: {source}")
        inputs.append(InputFile(source, Path("p46_reference") / source.name, item["sha256"]))

    total_bytes = sum(path.source.stat().st_size for path in inputs)
    plan = {
        "dataset": "Newer College 2020 / 01_short_experiment",
        "external_uuid": UUID,
        "canonical_scan_count": len(scans),
        "first_scan_stamp_ns": str(scans[0][0]),
        "last_scan_stamp_ns": str(scans[-1][0]),
        "scan_bytes": scan_bytes,
        "all_mirror_bytes": total_bytes,
        "ignored_noncanonical_pcd_names": ignored,
        "p45_tile_count": len(p45["tiles"]),
        "p46_reference_file_count": len(p46_files),
        "files": [
            {"source": str(item.source), "destination": item.relative_path.as_posix(),
             "size_bytes": item.source.stat().st_size, "expected_sha256": item.expected_sha256,
             "scan_index": item.scan_index, "stamp_ns": str(item.stamp_ns) if item.stamp_ns is not None else None}
            for item in inputs
        ],
    }
    return inputs, plan


def check_preflight(source_root: Path, destination_root: Path, started_at: str,
                    required_bytes: int) -> None:
    if not os.path.ismount(MOUNT):
        raise RuntimeError("HIKVISION mount is not mounted")
    source_device = os.stat(MOUNT).st_dev
    if os.stat(source_root).st_dev != source_device:
        raise RuntimeError("source path is not on the mounted HIKVISION filesystem")
    internal_device = os.stat(WORKSPACE).st_dev
    if source_device == internal_device:
        raise RuntimeError("source and internal destination unexpectedly share a filesystem")
    free_bytes = shutil.disk_usage(WORKSPACE).free
    if free_bytes < required_bytes + 10 * 1024**3:
        raise RuntimeError(f"insufficient internal disk space: free={free_bytes}, required+reserve={required_bytes + 10 * 1024**3}")

    lsblk = subprocess.run(["lsblk", "-nrpo", "UUID,MOUNTPOINT"], check=True, text=True,
                           capture_output=True, timeout=5)
    expected_line = f"{UUID} {MOUNT}"
    if expected_line not in {line.strip() for line in lsblk.stdout.splitlines()}:
        raise RuntimeError("HIKVISION UUID/mountpoint no longer match the frozen preflight")
    ps = subprocess.run(["ps", "-eo", "stat,comm,args"], check=True, text=True,
                        capture_output=True, timeout=5)
    if any(line.split(None, 1)[0].startswith("D") for line in ps.stdout.splitlines()[1:] if line.split()):
        raise RuntimeError("a D-state process exists; refusing source reads")
    kernel = subprocess.run(["journalctl", "-k", "-b", "--since", started_at,
                             "--no-pager", "-o", "short-precise"], check=True, text=True,
                            capture_output=True, timeout=8)
    bad = [line for line in kernel.stdout.splitlines() if SENSOR_RE.search(line)]
    if bad:
        raise RuntimeError("new USB/block/exFAT error in kernel log: " + bad[-1])
    if not destination_root.parent.exists() or os.stat(destination_root.parent).st_dev != internal_device:
        raise RuntimeError("destination parent is missing or not on the internal filesystem")


def copy_one(item: InputFile, destination_root: Path, rate_bytes_s: int,
             rate_origin: float, total_bytes_before: int,
             approved_source_device: int | None = None,
             retry_partial: bool = False) -> tuple[str, str, int, float]:
    destination = destination_root / item.relative_path
    destination.parent.mkdir(parents=True, exist_ok=True)
    internal_device = os.stat(WORKSPACE).st_dev
    if os.stat(destination.parent).st_dev != internal_device:
        raise RuntimeError(f"destination escaped the internal filesystem: {destination.parent}")
    if destination.exists() or destination.is_symlink():
        raise RuntimeError(f"refusing to overwrite existing destination: {destination}")
    partial = destination.with_name(destination.name + (".p59part.retry1" if retry_partial else ".p59part"))
    if partial.exists() or partial.is_symlink():
        raise RuntimeError(f"preserving unexpected partial; refusing to overwrite: {partial}")
    source_stat = item.source.lstat()
    if not item.source.is_file() or item.source.is_symlink():
        raise RuntimeError(f"source is not a regular non-symlink file: {item.source}")
    if approved_source_device is not None:
        if not os.path.ismount(MOUNT) or source_stat.st_dev != approved_source_device or os.stat(MOUNT).st_dev != approved_source_device:
            raise RuntimeError(f"source no longer resides on the approved HIKVISION mount: {item.source}")
    if item.scan_index is not None and source_stat.st_size != 1_904_640:
        raise RuntimeError(f"canonical PCD has unexpected byte size at frame {item.scan_index}: {source_stat.st_size}")

    source_hash = hashlib.sha256()
    copied = 0
    with item.source.open("rb", buffering=0) as src, partial.open("xb", buffering=0) as dst:
        while True:
            block = src.read(CHUNK_BYTES)
            if not block:
                break
            dst.write(block)
            source_hash.update(block)
            copied += len(block)
            if rate_bytes_s > 0:
                expected_elapsed = (total_bytes_before + copied) / rate_bytes_s
                delay = rate_origin + expected_elapsed - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
        dst.flush()
        os.fsync(dst.fileno())
    if copied != source_stat.st_size:
        raise RuntimeError(f"short read while copying {item.source}: {copied}/{source_stat.st_size}")
    src_sha = source_hash.hexdigest()
    if item.expected_sha256 and src_sha != item.expected_sha256:
        raise RuntimeError(f"source SHA mismatch for {item.source}: {src_sha} != {item.expected_sha256}")
    os.replace(partial, destination)
    destination_hash = file_sha256(destination)
    if destination_hash != src_sha:
        raise RuntimeError(f"destination SHA mismatch for {destination}: {destination_hash} != {src_sha}")
    return src_sha, destination_hash, copied, time.monotonic()


def prepare_destination(inputs: list[InputFile], plan: dict) -> tuple[set[int], int]:
    """Create a new destination or resume only the exact observed 10-second stop."""
    manifest = DATA_ROOT / "transfer_manifest.jsonl"
    checkpoints = DATA_ROOT / "transfer_checkpoints.jsonl"
    if not DATA_ROOT.exists():
        DATA_ROOT.mkdir(parents=False)
        (DATA_ROOT / "transfer_plan.json").write_text(json.dumps(plan, indent=2) + "\n", encoding="utf-8")
        manifest.open("x", encoding="utf-8").close()
        checkpoints.open("x", encoding="utf-8").close()
        return set(), 0

    try:
        existing_plan = json.loads((DATA_ROOT / "transfer_plan.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError("existing destination has no readable frozen transfer plan") from error
    if existing_plan != plan:
        raise RuntimeError("existing destination plan differs; refusing resume")
    if not manifest.is_file() or not checkpoints.is_file() or manifest.stat().st_size or checkpoints.stat().st_size:
        raise RuntimeError("resume is allowed only for the exact initial interruption with empty logs")

    expected = {item.relative_path.as_posix(): (index, item) for index, item in enumerate(inputs, 1)}
    allowed = {"transfer_plan.json", "transfer_manifest.jsonl", "transfer_checkpoints.jsonl"}
    completed: set[int] = set()
    completed_bytes = 0
    partials: list[Path] = []
    for path in DATA_ROOT.rglob("*"):
        if path.is_dir():
            continue
        relative = path.relative_to(DATA_ROOT).as_posix()
        if relative in allowed:
            continue
        if relative.endswith(".p59part"):
            partials.append(path)
            continue
        item_pair = expected.get(relative)
        if item_pair is None:
            raise RuntimeError(f"unexpected file in interrupted destination: {relative}")
        index, item = item_pair
        if path.is_symlink() or not path.is_file() or item.expected_sha256 is None:
            raise RuntimeError(f"cannot safely validate pre-existing destination: {relative}")
        if path.stat().st_size != item.source.stat().st_size or file_sha256(path) != item.expected_sha256:
            raise RuntimeError(f"pre-existing destination failed frozen SHA/size check: {relative}")
        completed.add(index)
        completed_bytes += path.stat().st_size

    if completed != {1, 2, 3}:
        raise RuntimeError(f"interrupted destination is not the expected three-file prefix: {sorted(completed)}")
    if len(partials) != 1:
        raise RuntimeError("interrupted destination must contain exactly one preserved partial")
    fourth = inputs[3]
    expected_partial = DATA_ROOT / (fourth.relative_path.as_posix() + ".p59part")
    if partials[0] != expected_partial or expected_partial.is_symlink() or expected_partial.stat().st_size != CHUNK_BYTES:
        raise RuntimeError("preserved partial differs from the observed 1 MiB fourth-scan prefix")

    manifest_lines: list[str] = []
    digest = hashlib.sha256()
    for index in sorted(completed):
        item = inputs[index - 1]
        path = DATA_ROOT / item.relative_path
        size = path.stat().st_size
        record = {
            "index": index, "source": str(item.source), "destination": item.relative_path.as_posix(),
            "size_bytes": size, "source_sha256": item.expected_sha256,
            "destination_sha256": item.expected_sha256, "expected_sha256_match": True,
            "scan_index": item.scan_index, "stamp_ns": str(item.stamp_ns) if item.stamp_ns is not None else None,
            "recovered_from_published_file": True,
        }
        manifest_lines.append(json.dumps(record, sort_keys=True))
        digest.update((item.relative_path.as_posix() + "\0" + str(size) + "\0" + item.expected_sha256 + "\n").encode())
    with manifest.open("a", encoding="utf-8") as stream:
        stream.write("\n".join(manifest_lines) + "\n")
        stream.flush()
        os.fsync(stream.fileno())
    resume_event = {
        "reason": "initial copy process stopped by 10-second command timeout before first checkpoint",
        "preserved_partial": str(expected_partial), "preserved_partial_bytes": CHUNK_BYTES,
        "resumed_verified_files": sorted(completed), "utc_local": datetime.now().astimezone().isoformat(),
    }
    with (DATA_ROOT / "transfer_resume_events.jsonl").open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(resume_event, sort_keys=True) + "\n")
        stream.flush()
        os.fsync(stream.fileno())
    checkpoint = {
        "batch": 1, "files": len(completed), "bytes": completed_bytes,
        "cumulative_bytes": completed_bytes, "sha256_of_batch_records": digest.hexdigest(),
        "elapsed_sec": 0.0, "utc_local": datetime.now().astimezone().isoformat(),
        "rate_limit_bytes_s": MAX_RATE_BYTES_S, "resumed_verified_prefix": True,
    }
    with checkpoints.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(checkpoint, sort_keys=True) + "\n")
        stream.flush()
        os.fsync(stream.fileno())
    return completed, completed_bytes


def make_relocated_tile_manifest(destination_root: Path) -> tuple[str, str]:
    source = json.loads(P45_TILE_MANIFEST.read_text(encoding="utf-8"))
    expected_hash = json.loads(P45_RESULTS.read_text(encoding="utf-8"))["map"]["tile_manifest_sha256"]
    actual_source_hash = file_sha256(P45_TILE_MANIFEST)
    if actual_source_hash != expected_hash:
        raise RuntimeError("tracked P45 tile manifest SHA does not match the frozen P45 result")
    relocated = json.loads(json.dumps(source))
    tiles_dir = destination_root / "map_tiles/original"
    for tile in relocated["tiles"]:
        tile_path = tiles_dir / Path(tile["tile_path"]).name
        if not tile_path.is_file() or file_sha256(tile_path) != tile["sha256"]:
            raise RuntimeError(f"copied P45 tile failed frozen hash verification: {tile_path}")
        tile["tile_path"] = str(tile_path)
    target = destination_root / "map_tiles/tile_manifest_internal.json"
    target.write_text(json.dumps(relocated, indent=2) + "\n", encoding="utf-8")
    return actual_source_hash, file_sha256(target)


def main() -> int:
    if len(sys.argv) != 1:
        print("This bounded task script accepts no path overrides.", file=sys.stderr)
        return 2
    started_wall = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    inputs, plan = prepare_inputs()
    free_bytes = shutil.disk_usage(WORKSPACE).free
    required = int(plan["all_mirror_bytes"])
    if free_bytes < required + 10 * 1024**3:
        raise RuntimeError(f"internal disk reserve check failed: free={free_bytes}, need={required + 10 * 1024**3}")
    check_preflight(SCAN_ROOT, DATA_ROOT, started_wall, required)
    approved_source_device = os.stat(MOUNT).st_dev

    completed_indexes, completed_bytes = prepare_destination(inputs, plan)
    records_path = DATA_ROOT / "transfer_manifest.jsonl"
    batches_path = DATA_ROOT / "transfer_checkpoints.jsonl"
    start = time.monotonic()
    total_bytes = completed_bytes
    batch_bytes = 0
    batch_files = 0
    batch_index = 0
    batch_digest = hashlib.sha256()
    rate_origin = start - completed_bytes / MAX_RATE_BYTES_S
    record_stream = records_path.open("a", encoding="utf-8")
    checkpoint_stream = batches_path.open("a", encoding="utf-8")
    try:
        for index, item in enumerate(inputs, 1):
            if index in completed_indexes:
                continue
            source_sha, target_sha, size, _ = copy_one(
                item, DATA_ROOT, MAX_RATE_BYTES_S, rate_origin, total_bytes, approved_source_device,
                retry_partial=(index == 4 and (DATA_ROOT / (item.relative_path.as_posix() + ".p59part")).exists()))
            record = {
                "index": index, "source": str(item.source), "destination": item.relative_path.as_posix(),
                "size_bytes": size, "source_sha256": source_sha, "destination_sha256": target_sha,
                "expected_sha256_match": item.expected_sha256 is None or source_sha == item.expected_sha256,
                "scan_index": item.scan_index, "stamp_ns": str(item.stamp_ns) if item.stamp_ns is not None else None,
            }
            encoded = json.dumps(record, sort_keys=True)
            record_stream.write(encoded + "\n")
            batch_digest.update((item.relative_path.as_posix() + "\0" + str(size) + "\0" + source_sha + "\n").encode())
            total_bytes += size
            batch_bytes += size
            batch_files += 1
            if batch_bytes >= CHECKPOINT_BYTES or index == len(inputs):
                record_stream.flush()
                os.fsync(record_stream.fileno())
                check_preflight(SCAN_ROOT, DATA_ROOT, started_wall, required)
                batch_index += 1
                checkpoint = {
                    "batch": batch_index, "files": batch_files, "bytes": batch_bytes,
                    "cumulative_bytes": total_bytes, "sha256_of_batch_records": batch_digest.hexdigest(),
                    "elapsed_sec": round(time.monotonic() - start, 3), "utc_local": datetime.now().astimezone().isoformat(),
                    "rate_limit_bytes_s": MAX_RATE_BYTES_S,
                }
                checkpoint_stream.write(json.dumps(checkpoint, sort_keys=True) + "\n")
                checkpoint_stream.flush()
                os.fsync(checkpoint_stream.fileno())
                print(json.dumps(checkpoint, sort_keys=True), flush=True)
                batch_bytes = 0
                batch_files = 0
                batch_digest = hashlib.sha256()
        source_manifest_sha, relocated_manifest_sha = make_relocated_tile_manifest(DATA_ROOT)
        summary = {
            "plan": plan,
            "copied_files": len(inputs),
            "copied_bytes": total_bytes,
            "transfer_manifest_sha256": file_sha256(records_path),
            "checkpoint_manifest_sha256": file_sha256(batches_path),
            "source_p45_tile_manifest_sha256": source_manifest_sha,
            "relocated_tile_manifest_sha256": relocated_manifest_sha,
            "elapsed_sec": round(time.monotonic() - start, 3),
            "rate_limit_bytes_s": MAX_RATE_BYTES_S,
            "external_write_count": 0,
        }
        (DATA_ROOT / "input_mirror_summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({"complete": True, **summary}, sort_keys=True), flush=True)
    finally:
        record_stream.close()
        checkpoint_stream.close()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"P59 mirror stopped: {error}", file=sys.stderr, flush=True)
        raise
