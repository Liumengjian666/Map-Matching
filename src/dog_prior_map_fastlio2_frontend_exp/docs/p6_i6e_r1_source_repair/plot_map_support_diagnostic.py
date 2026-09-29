#!/usr/bin/env python3
"""Plot map-support diagnostics without changing the localization results."""

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def read_rows(path: Path):
    with path.open("r", newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"no diagnostic rows in {path}")
    return rows


def values(rows, field):
    return [float(row[field]) for row in rows]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("diagnostic_csv", type=Path)
    parser.add_argument("output_png", type=Path)
    parser.add_argument("--title", default="Map-support diagnostic")
    args = parser.parse_args()

    rows = read_rows(args.diagnostic_csv)
    time_s = values(rows, "time_s")
    fig, axes = plt.subplots(2, 2, figsize=(14, 9), constrained_layout=True)
    fig.suptitle(args.title)

    ax = axes[0, 0]
    sampled = [max(1.0, float(row["sampled_points"])) for row in rows]
    for label, field in (
        ("nominal r", "nominal_queries_with_neighbors_r"),
        ("nominal 2r", "nominal_queries_with_neighbors_2r"),
        ("predicted r", "predicted_queries_with_neighbors_r"),
        ("predicted 2r", "predicted_queries_with_neighbors_2r"),
    ):
        ratios = [100.0 * float(row[field]) / n for row, n in zip(rows, sampled)]
        ax.plot(time_s, ratios, label=label)
    ax.set_ylabel("sampled source points with target neighbors (%)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.plot(time_s, values(rows, "M0_fitness"), label="M0 fitness")
    ax.set_ylabel("NDT fitness")
    ax2 = ax.twinx()
    ax2.plot(time_s, values(rows, "predicted_to_nominal_translation_m"),
             color="tab:orange", label="predicted-to-M0 translation")
    ax2.plot(time_s, values(rows, "predicted_to_nominal_rotation_rad"),
             color="tab:green", label="predicted-to-M0 rotation")
    ax2.set_ylabel("pose difference (m / rad)")
    lines = ax.get_lines() + ax2.get_lines()
    ax.legend(lines, [line.get_label() for line in lines], fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    ax.plot(time_s, values(rows, "maximum_position_sigma_m"),
            label="maximum position sigma (m)")
    ax2 = ax.twinx()
    ax2.plot(time_s, values(rows, "maximum_rotation_sigma_rad"),
             color="tab:red", label="maximum rotation sigma (rad)")
    ax.set_ylabel("position sigma (m)")
    ax2.set_ylabel("rotation sigma (rad)")
    lines = ax.get_lines() + ax2.get_lines()
    ax.legend(lines, [line.get_label() for line in lines], fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    ax.plot(time_s, values(rows, "uobs_valid_correspondences"),
            label="U_obs valid correspondences")
    ax2 = ax.twinx()
    ax2.step(time_s, values(rows, "lidar_reliable_dimension"), where="post",
             label="LiDAR reliable dimension", color="tab:purple")
    ax2.set_ylim(-0.2, 6.2)
    ax.set_ylabel("valid geometric correspondences")
    ax2.set_ylabel("reliable dimension (0–6)")
    lines = ax.get_lines() + ax2.get_lines()
    ax.legend(lines, [line.get_label() for line in lines], fontsize=8,
              loc="upper right")
    ax.grid(True, alpha=0.3)

    for ax in axes.flat:
        ax.set_xlabel("time from frozen input start (s)")
    args.output_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output_png, dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()
