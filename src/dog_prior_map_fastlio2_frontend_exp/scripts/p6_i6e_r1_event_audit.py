#!/usr/bin/env python3
"""Independently audit trigger, visual-quality, complementarity and update counts."""

import argparse
import csv
import json
from pathlib import Path


REQUIRED_COLUMNS = {
    "triggered",
    "quality_passed",
    "visual_subspace_status",
    "status",
    "position_correction_norm_m",
    "velocity_correction_norm_m",
}


def truth(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes"}:
        return True
    if normalized in {"0", "false", "no", ""}:
        return False
    raise ValueError(f"unexpected boolean field value: {value!r}")


def audit(path: Path):
    with path.open("r", newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        fields = set(reader.fieldnames or ())
        missing = REQUIRED_COLUMNS - fields
        if missing:
            raise ValueError(f"{path}: missing required columns: {sorted(missing)}")
        rows = list(reader)

    triggered = [truth(row["triggered"]) for row in rows]
    quality = [truth(row["quality_passed"]) for row in rows]
    complementary = [
        row["visual_subspace_status"].strip()
        == "VISUAL_COMPLEMENTS_LIDAR_WEAK_SUBSPACE"
        for row in rows
    ]
    applied = [row["status"].strip().startswith("APPLIED_") for row in rows]
    nonzero_applied = [
        applied[index]
        and (
            abs(float(row["position_correction_norm_m"])) > 1e-12
            or abs(float(row["velocity_correction_norm_m"])) > 1e-12
        )
        for index, row in enumerate(rows)
    ]

    count = lambda mask: sum(1 for value in mask if value)
    triggered_and_quality = [t and q for t, q in zip(triggered, quality)]
    triggered_quality_complementary = [
        t and q and c for t, q, c in zip(triggered, quality, complementary)
    ]
    return {
        "input": str(path),
        "total_events": len(rows),
        "triggered": count(triggered),
        "quality_passed": count(quality),
        "triggered_and_quality_passed": count(triggered_and_quality),
        "complementary": count(complementary),
        "triggered_quality_complementary": count(triggered_quality_complementary),
        "applied": count(applied),
        "nonzero_applied": count(nonzero_applied),
        "quality_passed_but_not_triggered": count(
            [q and not t for t, q in zip(triggered, quality)]
        ),
        "triggered_but_quality_rejected": count(
            [t and not q for t, q in zip(triggered, quality)]
        ),
        "triggered_quality_passed_but_noncomplementary": count(
            [t and q and not c for t, q, c in zip(triggered, quality, complementary)]
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="+", type=Path)
    args = parser.parse_args()
    print(json.dumps([audit(path) for path in args.csv], indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
