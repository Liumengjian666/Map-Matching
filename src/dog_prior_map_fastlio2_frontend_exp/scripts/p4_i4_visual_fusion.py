#!/usr/bin/env python3
"""Fixed-measurement offline replay, not a ROS runtime or visual frontend."""

import argparse
import csv
import hashlib
import subprocess
import tempfile
from pathlib import Path

import numpy as np
import p4_i2_state_contamination as base
import yaml

START = "a328d3e3adb9109035a7d6f965d2db2c271cee17"
PACKAGE = base.PACKAGE
OUT = PACKAGE / "docs/p4_i4_visual_translation_fusion"
VISUAL = PACKAGE / "docs/p4_i3_visual_increment/visual_increment.csv"
MODES = [
    "BASELINE",
    "VISUAL_XYZ_005",
    "VISUAL_XY_005",
    "VISUAL_XYZ_003",
    "VISUAL_XYZ_010",
]


def read_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def prepare_visual(path):
    frozen = subprocess.check_output(
        [
            "git",
            "-C",
            str(base.WORKSPACE),
            "show",
            START + ":" + str(VISUAL.relative_to(base.WORKSPACE)),
        ]
    )
    assert base.sha256(VISUAL) == hashlib.sha256(frozen).hexdigest(), (
        "visual input changed"
    )
    calibration = yaml.safe_load(base.EXTRINSICS.read_text())
    t_ic = np.array(calibration["rgb_camera_to_imu"]["data"]).reshape(4, 4)
    rows = []
    for r in read_csv(VISUAL):
        if r["status"] != "VALID":
            continue
        measured = np.eye(4)
        measured[:3] = np.array(
            [float(r[f"T_Ccur_Cref_{a}{b}"]) for a in range(3) for b in range(4)]
        ).reshape(3, 4)
        # PnP R is used ONLY for the already-verified origin/direction transform.
        # No visual orientation is passed into the filter helper.
        translation = (t_ic @ np.linalg.inv(measured) @ np.linalg.inv(t_ic))[:3, 3]
        rows.append(
            {
                "ref_ns": int(r["timestamp_ref_ns"]),
                "cur_ns": int(r["timestamp_cur_ns"]),
                "zx": translation[0],
                "zy": translation[1],
                "zz": translation[2],
                "inliers": int(r["pnp_inliers"]),
                "ratio": float(r["inlier_ratio"]),
                "reprojection": float(r["reprojection_rmse_px"]),
            }
        )
    assert len(rows) == 1802
    write_csv(path, rows)
    return rows


def compile_helper(executable):
    command = [
        "g++",
        "-std=c++14",
        "-O2",
        "-fopenmp",
        f"-I{PACKAGE / 'include'}",
        f"-I{base.FRONTEND_ROOT / 'include'}",
        "-I/usr/include/eigen3",
        str(PACKAGE / "scripts/p4_i4_visual_fusion_replay.cpp"),
        "-o",
        str(executable),
    ]
    subprocess.run(command, check=True)
    return command


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--report-existing",
        type=Path,
        help="resume evaluation from completed replay CSVs",
    )
    args = parser.parse_args()
    assert (
        subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=base.WORKSPACE, text=True
        ).strip()
        == START
    )
    assert (
        subprocess.check_output(
            ["git", "branch", "--show-current"], cwd=base.WORKSPACE, text=True
        ).strip()
        == "paper"
    )
    assert not subprocess.check_output(
        [
            "git",
            "status",
            "--porcelain",
            "--",
            *map(str, base.COMPILED_FRONTEND_SOURCES),
        ],
        cwd=base.WORKSPACE,
        text=True,
    ).strip()
    base.check_artifact_hashes()
    if args.report_existing:
        temporary = args.report_existing
        runs = {
            mode: read_csv(temporary / f"{mode}.csv")
            for mode in [*MODES, "VISUAL_SKIP"]
        }
        gate = base.run_full_update_gate(runs["BASELINE"], temporary / "scans.csv")
        assert runs["VISUAL_SKIP"] == runs["BASELINE"]
        for mode in MODES[1:]:
            assert len(runs[mode]) == 4127
            assert len(read_csv(temporary / f"{mode}_updates.csv")) == 1802
        from p4_i4_visual_fusion_report import report

        report(runs, temporary, OUT, gate, "See p4_i4_visual_fusion.py compile_helper")
        return
    temporary = Path(tempfile.mkdtemp(prefix="p4_i4_visual_fusion_"))
    print("REPLAY_DATA_DIR", temporary, flush=True)
    visual = prepare_visual(temporary / "visual.csv")
    counts = base.extract_runtime_inputs(temporary / "imu.csv", temporary / "scans.csv")
    base.write_parameters(temporary / "params.txt")
    print("FROZEN_INPUTS", counts, "visual", len(visual), flush=True)
    executable = temporary / "replay"
    command = compile_helper(executable)
    runs = {}
    for mode in ["BASELINE", "VISUAL_SKIP", *MODES[1:]]:
        path = temporary / f"{mode}.csv"
        subprocess.run(
            [
                str(executable),
                mode,
                str(temporary / "imu.csv"),
                str(temporary / "scans.csv"),
                str(temporary / "params.txt"),
                str(temporary / "visual.csv"),
                str(path),
                str(temporary / f"{mode}_updates.csv"),
            ],
            check=True,
        )
        runs[mode] = read_csv(path)
        if mode == "BASELINE":
            gate = base.run_full_update_gate(runs[mode], temporary / "scans.csv")
        elif mode == "VISUAL_SKIP":
            assert runs[mode] == runs["BASELINE"], "visual skip changed baseline"
            print("VISUAL_SKIP_EXACT_BASELINE_PASS", flush=True)
        else:
            assert len(read_csv(temporary / f"{mode}_updates.csv")) == 1802
    # Only now load GT and evaluate. No GT value is an input to the C++ process.
    from p4_i4_visual_fusion_report import report

    report(runs, temporary, OUT, gate, command)


if __name__ == "__main__":
    main()
