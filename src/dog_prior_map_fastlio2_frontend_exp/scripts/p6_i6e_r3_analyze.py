#!/usr/bin/env python3
"""Aggregate the frozen P6-I6E-R3 short closed-loop experiments."""

from __future__ import annotations

import csv
import math
import statistics
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
RESULT_ROOT = PACKAGE / "docs/p6_i6e_r3_residual_consistency"
RUNS = (
    ("floor01", "Floor01", "P0_LEGACY", "LEGACY_BASE_NO_GATE", "LEGACY"),
    ("floor01", "Floor01", "P0_EXACT_RESIDUAL", "LEGACY_BASE_NO_GATE", "EXACT_RESIDUAL"),
    ("floor01", "Floor01", "P3_LEGACY", "ADAPTIVE_SELECTED_NIS", "LEGACY"),
    ("floor01", "Floor01", "P3_EXACT_RESIDUAL", "ADAPTIVE_SELECTED_NIS", "EXACT_RESIDUAL"),
    ("corridor01", "Corridor01", "P0_LEGACY", "LEGACY_BASE_NO_GATE", "LEGACY"),
    ("corridor01", "Corridor01", "P0_EXACT_RESIDUAL", "LEGACY_BASE_NO_GATE", "EXACT_RESIDUAL"),
    ("corridor01", "Corridor01", "P3_LEGACY", "ADAPTIVE_SELECTED_NIS", "LEGACY"),
    ("corridor01", "Corridor01", "P3_EXACT_RESIDUAL", "ADAPTIVE_SELECTED_NIS", "EXACT_RESIDUAL"),
)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    coordinate = (len(ordered) - 1) * fraction
    lower = int(coordinate)
    upper = min(lower + 1, len(ordered) - 1)
    return (ordered[lower] * (upper - coordinate) +
            ordered[upper] * (coordinate - lower))


def first_event(rows: list[dict[str, str]], predicate) -> tuple[str, str]:
    for row in rows:
        if predicate(row):
            return row["transaction_id"], row["time_s"]
    return "", ""


def maximum_rss_mib(path: Path) -> float:
    for line in path.read_text(encoding="utf-8").splitlines():
        if "Maximum resident set size" in line:
            return int(line.rsplit(":", 1)[1].strip()) / 1024.0
    raise RuntimeError(f"missing maximum RSS in {path}")


def write_rows(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        raise RuntimeError(f"refusing to write empty aggregate: {path}")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]),
                                lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def run_prefix(dataset_dir: str, dataset_name: str,
               variant: str) -> Path:
    return RESULT_ROOT / dataset_dir / variant / f"B4_{dataset_name}"


def main() -> int:
    metrics: list[dict[str, object]] = []
    shadow_rows: list[dict[str, object]] = []
    actual_rows: list[dict[str, object]] = []
    for dataset_dir, dataset, variant, r2_policy, r3_mode in RUNS:
        prefix = run_prefix(dataset_dir, dataset, variant)
        reliability = read_csv(Path(str(prefix) + "_reliability.csv"))
        actual = read_csv(Path(str(prefix) +
                               "_reliability.csv.r3_actual_state.csv"))
        visual = read_csv(Path(str(prefix) +
                               "_reliability.csv.visual_updates.csv"))
        runtime = read_csv(Path(str(prefix) + "_runtime.csv"))
        trajectory = read_csv(Path(str(prefix) + "_trajectory.csv"))
        shadow = read_csv(Path(str(prefix) +
                               "_reliability.csv.r3_shadow_comparison.csv"))
        if not (len(reliability) == len(actual) == len(runtime) ==
                len(trajectory)):
            raise RuntimeError(f"row-count mismatch in {dataset}/{variant}")
        total_ms = [float(row["total_ms"]) for row in runtime]
        first_nis = first_event(
            actual, lambda row: row["lidar_nis_rejected"] == "1")
        first_support_loss = first_event(
            reliability,
            lambda row: row["uobs_map_support_sufficient"] != "1")
        first_zero_support = first_event(
            reliability,
            lambda row: int(row["uobs_valid_correspondences"]) == 0)
        maximum_velocity = max(math.sqrt(sum(
            float(row[f"velocity_post_{axis}"]) ** 2 for axis in "xyz"))
            for row in trajectory)
        metrics.append({
            "dataset": dataset,
            "variant": variant,
            "r2_policy": r2_policy,
            "r3_mode": r3_mode,
            "frames": len(reliability),
            "ndt_calls": sum(int(row["ndt_call_count"])
                             for row in reliability),
            "M0_converged": sum(row["M0_converged"] == "1"
                                for row in reliability),
            "uobs_valid": sum(row["uobs_valid"] == "1"
                              for row in reliability),
            "map_support_insufficient": sum(
                row["uobs_map_support_sufficient"] != "1"
                for row in reliability),
            "zero_correspondences": sum(
                int(row["uobs_valid_correspondences"]) == 0
                for row in reliability),
            "lidar_attempted": sum(row["lidar_update_attempted"] == "1"
                                   for row in actual),
            "lidar_committed": sum(row["lidar_update_committed"] == "1"
                                   for row in actual),
            "lidar_nis_rejected": sum(row["lidar_nis_rejected"] == "1"
                                      for row in actual),
            "visual_triggered": sum(row["triggered"] == "1"
                                    for row in visual),
            "visual_quality_pass": sum(row["quality_passed"] == "1"
                                       for row in visual),
            "visual_actual_update": sum(
                row["status"] ==
                "APPLIED_PROJECTED_COMPLEMENTARY_TRANSLATION"
                for row in visual),
            "imu_only_intervals": sum(
                row["route_actual"] == "IMU_ONLY_THIS_INTERVAL"
                for row in actual),
            "visual_only_intervals": sum(
                row["route_actual"] == "VISUAL_ONLY_THIS_INTERVAL"
                for row in actual),
            "first_nis_rejection_tx": first_nis[0],
            "first_nis_rejection_time_s": first_nis[1],
            "first_map_support_loss_tx": first_support_loss[0],
            "first_map_support_loss_time_s": first_support_loss[1],
            "first_zero_support_tx": first_zero_support[0],
            "first_zero_support_time_s": first_zero_support[1],
            "maximum_position_sigma": max(
                float(row["position_sigma_corrected"]) for row in actual),
            "maximum_rotation_sigma": max(
                float(row["rotation_sigma_corrected"]) for row in actual),
            "maximum_velocity_norm": maximum_velocity,
            "mean_runtime_ms": statistics.fmean(total_ms),
            "p95_runtime_ms": percentile(total_ms, 0.95),
            "maximum_runtime_ms": max(total_ms),
            "peak_rss_mib": maximum_rss_mib(
                Path(str(prefix) + "_resource.txt")),
            "maximum_actual_external_gap_s": max(
                float(row["actual_external_gap_s"]) for row in actual),
            "relocalization_pending_rows": sum(
                row["relocalization_pending_validation"] == "1"
                for row in actual),
            "verified_recovery_nonzero_rows": sum(
                row["verified_recovery_ns"] != "0" for row in actual),
            "shadow_official_state_unchanged": sum(
                row["official_state_unchanged"] == "1" for row in shadow),
            "shadow_rows": len(shadow),
        })
        prefix_fields = {
            "dataset": dataset,
            "variant": variant,
            "r2_policy_run": r2_policy,
            "r3_mode_run": r3_mode,
        }
        shadow_rows.extend({**prefix_fields, **row} for row in shadow)
        actual_rows.extend({**prefix_fields, **row} for row in actual)

    divergences: list[dict[str, object]] = []
    state_fields = (
        "corrected_imu_tx", "corrected_imu_ty", "corrected_imu_tz",
        "corrected_imu_qx", "corrected_imu_qy", "corrected_imu_qz",
        "corrected_imu_qw", "velocity_post_x", "velocity_post_y",
        "velocity_post_z")
    for dataset_dir, dataset in (("floor01", "Floor01"),
                                 ("corridor01", "Corridor01")):
        for policy in ("P0", "P3"):
            legacy = read_csv(Path(str(run_prefix(
                dataset_dir, dataset, policy + "_LEGACY")) +
                "_trajectory.csv"))
            exact = read_csv(Path(str(run_prefix(
                dataset_dir, dataset, policy + "_EXACT_RESIDUAL")) +
                "_trajectory.csv"))
            result: dict[str, object] = {
                "dataset": dataset, "r2_policy": policy,
                "first_divergent_transaction": "",
                "first_divergent_stamp_ns": "",
                "maximum_component_difference_at_first_divergence": "",
            }
            for old, new in zip(legacy, exact):
                difference = max(abs(float(old[field]) - float(new[field]))
                                 for field in state_fields)
                if difference > 1e-12:
                    result.update({
                        "first_divergent_transaction": old["transaction_id"],
                        "first_divergent_stamp_ns": old["stamp_ns"],
                        "maximum_component_difference_at_first_divergence":
                            difference,
                    })
                    break
            divergences.append(result)

    write_rows(RESULT_ROOT / "run_metrics.csv", metrics)
    write_rows(RESULT_ROOT / "shadow_comparison.csv", shadow_rows)
    write_rows(RESULT_ROOT / "actual_state_audit.csv", actual_rows)
    write_rows(RESULT_ROOT / "first_state_divergence.csv", divergences)
    print(f"R3_ANALYSIS_PASS runs={len(metrics)} shadow_rows={len(shadow_rows)} "
          f"actual_rows={len(actual_rows)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
