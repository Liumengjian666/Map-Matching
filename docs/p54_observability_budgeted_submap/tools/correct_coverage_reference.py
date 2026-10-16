#!/usr/bin/env python3
"""Correct only the P46 baseline column in the post-run coverage comparison."""

import argparse
import csv
import hashlib
import json
import shutil
from pathlib import Path


def sha256(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def read_rows(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--coverage", type=Path, required=True)
    p.add_argument("--samples", type=Path, required=True)
    p.add_argument("--freeze", type=Path, required=True)
    p.add_argument("--local-geometry", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--preserve-original", type=Path, required=True)
    p.add_argument("--provenance", type=Path, required=True)
    a = p.parse_args()
    for target in (a.output, a.preserve_original, a.provenance):
        if target.exists():
            raise FileExistsError("refusing to overwrite: " + str(target))

    freeze = json.loads(a.freeze.read_text())
    expected_original = next((x["sha256"] for x in freeze["inputs_and_outputs"]
                              if x["kind"] == "coverage_output"), None)
    if not expected_original or sha256(a.coverage) != expected_original:
        raise ValueError("original coverage CSV does not match the frozen run output")
    shutil.copyfile(a.coverage, a.preserve_original)

    samples = {r["scan_ns"]: r for r in read_rows(a.samples)}
    local = {r["scan_ns"]: r for r in read_rows(a.local_geometry)}
    with a.coverage.open(newline="") as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames
        output_rows = list(reader)
    if len(samples) != 154 or len(output_rows) != 616:
        raise ValueError("unexpected P54 sample/coverage row count")
    for row in output_rows:
        stamp = row["scan_ns"]
        if stamp not in samples or stamp not in local:
            raise ValueError("P46 Local geometry timestamp missing: " + stamp)
        if int(local[stamp]["target_points"]) != int(samples[stamp]["target_points"]):
            raise ValueError("P46 Local Target point count differs from frozen P47 sample")
        row["baseline_p46_initial_overlap_2m"] = local[stamp]["initial_overlap_2m"]

    with a.output.open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(output_rows)
    provenance = {
        "protocol": "P54_POST_FREEZE_BASELINE_COVERAGE_REFERENCE_CORRECTION_V1",
        "gt_used": False,
        "reason": "The first exported comparison column referenced P46 Global diagnostics; this derivative uses the correct P46 Local formal1800 diagnostics.",
        "selection_or_ndt_outputs_changed": False,
        "original_coverage_frozen_sha256": expected_original,
        "preserved_original_path": str(a.preserve_original.resolve()),
        "preserved_original_sha256": sha256(a.preserve_original),
        "correct_P46_local_geometry_path": str(a.local_geometry.resolve()),
        "correct_P46_local_geometry_sha256": sha256(a.local_geometry),
        "P47_sample_manifest_sha256": sha256(a.samples),
        "corrected_coverage_path": str(a.output.resolve()),
        "corrected_coverage_sha256": sha256(a.output),
        "rows": len(output_rows),
        "reference_target_points_matched_all_samples": True,
    }
    a.provenance.write_text(json.dumps(provenance, indent=2, sort_keys=True) + "\n")
    print("P54_COVERAGE_REFERENCE_CORRECTION_PASS rows=616 gt_used=0 ndt_rerun=0")


if __name__ == "__main__":
    main()
