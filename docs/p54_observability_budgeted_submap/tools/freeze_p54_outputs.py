#!/usr/bin/env python3
"""Freeze P54 pre-GT inputs/outputs; this program deliberately never opens GT."""

import argparse
import csv
import hashlib
import json
from pathlib import Path


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def rows(path: Path):
    with path.open(newline="") as stream:
        yield from csv.DictReader(stream)


def record(path: Path, kind: str) -> dict:
    return {"kind": kind, "path": str(path.resolve()), "bytes": path.stat().st_size, "sha256": sha256(path)}


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--sample", type=Path, required=True)
    p.add_argument("--catalog", type=Path, required=True)
    p.add_argument("--tile-manifest", type=Path, required=True)
    p.add_argument("--p47-degeneracy", type=Path, required=True)
    p.add_argument("--source-dir", type=Path, required=True)
    p.add_argument("--geometry", type=Path, required=True)
    p.add_argument("--selection-dir", type=Path, required=True)
    p.add_argument("--alignment-dir", type=Path, required=True)
    p.add_argument("--binary", type=Path, required=True)
    p.add_argument("--source-code", type=Path, required=True)
    p.add_argument("--cmake", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()

    sample_rows = list(rows(a.sample))
    if len(sample_rows) != 154:
        raise SystemExit(f"expected 154 frozen samples, got {len(sample_rows)}")
    catalog = {r["tile_id"]: r for r in rows(a.catalog)}
    active_ids = set()
    for row in sample_rows:
        active_ids.update(x for x in row["active_tile_ids"].split(";") if x)
    if not active_ids or not active_ids.issubset(catalog):
        raise SystemExit("active Tile IDs are empty or absent from frozen catalog")

    files = []
    files.extend([
        record(a.sample, "frozen_P47_sample_manifest"),
        record(a.catalog, "P47_P45_tile_catalog"),
        record(a.tile_manifest, "P45_tile_manifest"),
        record(a.p47_degeneracy, "P47_native_NDT_curvature_diagnostics"),
        record(a.geometry, "P46_geometry_diagnostics"),
        record(a.binary, "P54_executable"),
        record(a.source_code, "P54_source_code"),
        record(a.cmake, "research_build_gate"),
    ])
    for path, kind in [
        (a.selection_dir / "selection_manifest.csv", "selection_output"),
        (a.selection_dir / "information_metrics.csv", "information_output"),
        (a.alignment_dir / "align_results.csv", "NDT_output"),
        (a.alignment_dir / "coverage_comparison.csv", "coverage_output"),
        (a.alignment_dir / "runtime_comparison.csv", "runtime_output"),
    ]:
        files.append(record(path, kind))

    sources = []
    raw_scans = []
    selection_by_frame = {}
    for selection_row in rows(a.selection_dir / "selection_manifest.csv"):
        selection_by_frame.setdefault(selection_row["frame_index"], []).append(selection_row)
    for row in sample_rows:
        frame = row["frame_index"]
        source = a.source_dir / f"source_frame_{frame}.pcd"
        raw = Path(row["pcd_path"])
        source_hash = sha256(source)
        raw_hash = sha256(raw)
        selection_rows = selection_by_frame.get(frame, [])
        if len(selection_rows) != 4 or any(r["source_file_sha256"] != source_hash for r in selection_rows):
            raise SystemExit(f"Source/output parity not complete at frame {frame}")
        if raw_hash != row["pcd_sha256"]:
            raise SystemExit(f"raw PCD SHA mismatch at frame {frame}")
        sources.append({"frame_index": frame, "scan_ns": row["scan_ns"],
                        "path": str(source.resolve()), "bytes": source.stat().st_size,
                        "sha256": source_hash, "stable_hash": row["expected_source_hash"]})
        raw_scans.append({"frame_index": frame, "scan_ns": row["scan_ns"],
                          "path": str(raw.resolve()), "bytes": raw.stat().st_size,
                          "sha256": raw_hash})

    tiles = []
    for tile_id in sorted(active_ids):
        tile = catalog[tile_id]
        path = Path(tile["tile_path"])
        digest = sha256(path)
        if digest != tile["sha256"]:
            raise SystemExit(f"P45 Tile SHA mismatch: {tile_id}")
        tiles.append({"tile_id": tile_id, "path": str(path.resolve()),
                      "point_count": int(tile["point_count"]), "bytes": path.stat().st_size,
                      "sha256": digest})

    doc = {
        "protocol": "P54_PRE_GT_OUTPUT_FREEZE_V1",
        "gt_opened": False,
        "frozen_sample_count": len(sample_rows),
        "active_tile_count": len(tiles),
        "selection_rows": 616,
        "alignment_rows": 616,
        "inputs_and_outputs": files,
        "frozen_sources": sources,
        "raw_scans": raw_scans,
        "active_tiles": tiles,
    }
    a.output.parent.mkdir(parents=True, exist_ok=True)
    if a.output.exists():
        raise SystemExit(f"refusing to overwrite freeze manifest: {a.output}")
    a.output.write_text(json.dumps(doc, indent=2, sort_keys=True) + "\n")
    print(f"P54_PRE_GT_FREEZE_PASS samples={len(sample_rows)} active_tiles={len(tiles)} source_files={len(sources)} raw_scans={len(raw_scans)}")


if __name__ == "__main__":
    main()
