#!/usr/bin/env python3
"""Audit only internal P59 PCD copies against the frozen P46 scan hashes."""

from __future__ import annotations

import argparse
import csv
import hashlib
import re
from pathlib import Path


CANONICAL = re.compile(r"cloud_(\d+)_(\d{9})\.pcd\Z")
FIELDS = ["frame_index", "stamp_ns", "relative_path", "size_bytes", "sha256",
          "p46_expected_sha256", "match"]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def audit(mirror_scan_root: Path, p46_manifest: Path) -> list[dict[str, str]]:
    references: dict[int, dict[str, str]] = {}
    with p46_manifest.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            stamp = int(row["stamp_ns"])
            if stamp in references:
                raise ValueError(f"duplicate timestamp in frozen P46 manifest: {stamp}")
            references[stamp] = row

    files = []
    for path in mirror_scan_root.iterdir():
        if ".p59part" in path.name:
            continue
        match = CANONICAL.fullmatch(path.name)
        if not match:
            raise ValueError(f"unexpected file in internal scan mirror: {path.name}")
        stamp = int(match.group(1)) * 1_000_000_000 + int(match.group(2))
        files.append((stamp, path))
    files.sort(key=lambda item: item[0])
    rows: list[dict[str, str]] = []
    for ordinal, (stamp, path) in enumerate(files, 1):
        ref = references.get(stamp)
        if ref is None or int(ref["scan_index"]) != ordinal:
            raise ValueError(f"internal prefix is not contiguous at ordinal {ordinal}: {path.name}")
        actual_hash = sha256(path)
        expected_hash = ref["sha256"]
        if path.stat().st_size != int(ref["size_bytes"]) or actual_hash != expected_hash:
            raise ValueError(f"frozen P46 input mismatch at frame {ordinal}: {path.name}")
        rows.append({
            "frame_index": str(ordinal), "stamp_ns": str(stamp),
            "relative_path": path.name, "size_bytes": str(path.stat().st_size),
            "sha256": actual_hash, "p46_expected_sha256": expected_hash, "match": "true",
        })
    return rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mirror-scan-root", type=Path, default=Path(
        "/home/jian/livox_ws/p59_newer_college_full_sequence_data/dataset/01_short_experiment/raw_format/ouster_scan"))
    parser.add_argument("--p46-manifest", type=Path, default=Path(
        "/home/jian/livox_ws/p59_newer_college_full_sequence_ws/docs/p46_newer_college_real_tile_switch/input_scan_sha256.csv"))
    parser.add_argument("--output", type=Path, default=Path(
        "/home/jian/livox_ws/p59_newer_college_full_sequence_ws/docs/p59_newer_college_full_sequence/partial_scan_hash_verification.csv"))
    args = parser.parse_args()
    rows = audit(args.mirror_scan_root, args.p46_manifest)
    with args.output.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    print(f"verified_partial_prefix={len(rows)} output={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
