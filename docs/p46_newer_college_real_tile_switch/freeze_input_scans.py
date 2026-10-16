#!/usr/bin/env python3
"""Hash the exact 1800 PCD inputs consumed by the paired P46 replays."""

import argparse
import csv
import hashlib
import json
from pathlib import Path


def digest(path):
    result = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def rows(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--local-receipts", required=True, type=Path)
    parser.add_argument("--global-receipts", required=True, type=Path)
    parser.add_argument("--p44-input-hashes", required=True, type=Path)
    parser.add_argument("--archive-dir", required=True, type=Path)
    args = parser.parse_args()
    local = rows(args.local_receipts)
    global_ = rows(args.global_receipts)
    if len(local) != 1800 or len(global_) != 1800:
        raise RuntimeError("P46 input hash contract expects exactly 1800 scans in each run")
    for fields in (("scan_stamp_ns", "pcd_path", "point_count", "max_point_offset_ns"),):
        if any(tuple(a[key] for key in fields) != tuple(b[key] for key in fields)
               for a, b in zip(local, global_)):
            raise RuntimeError("paired Local/Global publisher receipts differ")

    p44 = rows(args.p44_input_hashes)
    p44_by_path = {str(Path(row["pcd_path"]).resolve()): row for row in p44}
    output = []
    for index, receipt in enumerate(global_, 1):
        path = Path(receipt["pcd_path"])
        if not path.is_file():
            raise FileNotFoundError(str(path))
        item = {"scan_index": index, "stamp_ns": receipt["scan_stamp_ns"],
                "pcd_path": str(path.resolve()), "size_bytes": path.stat().st_size,
                "point_count": int(receipt["point_count"]),
                "max_point_offset_ns": int(receipt["max_point_offset_ns"]),
                "sha256": digest(path)}
        if index <= len(p44):
            previous = p44_by_path.get(str(path.resolve()))
            if not previous or previous["sha256_after_run"] != item["sha256"]:
                raise RuntimeError("P46 scan input differs from P44 hash at frame " + str(index))
            item["matches_P44_frozen_sha256"] = True
        output.append(item)

    args.archive_dir.mkdir(parents=True, exist_ok=True)
    target = args.archive_dir / "input_scan_sha256.csv"
    with target.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(output[0].keys()), lineterminator="\n")
        writer.writeheader()
        writer.writerows(output)
    manifest = {
        "dataset": "Newer College 2020 / 01_short_experiment",
        "paired_receipts_identical": True,
        "scan_count": len(output),
        "first_stamp_ns": output[0]["stamp_ns"],
        "last_stamp_ns": output[-1]["stamp_ns"],
        "total_pcd_bytes": sum(item["size_bytes"] for item in output),
        "total_pcd_gib": sum(item["size_bytes"] for item in output) / (1024 ** 3),
        "p44_first_300_hash_match": all(item.get("matches_P44_frozen_sha256")
                                         for item in output[:len(p44)]),
        "all_input_hashes": "input_scan_sha256.csv",
        "sha256_of_input_hash_csv": digest(target),
        "gt_used": False,
    }
    (args.archive_dir / "input_hash_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
