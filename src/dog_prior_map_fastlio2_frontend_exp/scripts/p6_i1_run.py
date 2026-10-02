#!/usr/bin/env python3
"""Run the frozen-input P6-I1 offline branched-recovery replay.

The five replay trajectories are completed and checked before this driver
invokes the separate post-hoc evaluator that reads Floor01 ground truth.
"""

import csv
import hashlib
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml

WORKSPACE = Path("/home/jian/livox_ws/dog_loc_paper_ws")
PACKAGE = WORKSPACE / "src/dog_prior_map_fastlio2_frontend_exp"
OUT = PACKAGE / "docs/p6_i1_branched_recovery"
ASSETS = Path("/home/jian/livox_ws/p6_i1_recovery_assets_20260927")
INPUT = ASSETS / "input"
BUILD = ASSETS / "build"
EXECUTABLE = BUILD / "p6_i1_branched_recovery"
MAP = ASSETS / "floor01_h1_map_p5_frozen.pcd"
EXTERNAL_MAP = Path(
    "/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/map/frozen/"
    "floor01_h1_map_p5_frozen.pcd"
)
MAP_PROVENANCE = EXTERNAL_MAP.with_suffix(".provenance.txt")
BAG = Path(
    "/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/"
    "p3_r10b_fix1_floor01_full_rerun_20260926/floor01_fix1_runtime_topics.bag"
)
CONFIG = BAG.with_name("floor01_superloc_smoke.yaml")
EXTRINSICS = Path(
    "/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/calibration/"
    "floor01_extrinsics.yaml"
)
VISUAL_SOURCE = PACKAGE / "docs/p4_i3_visual_increment/visual_increment.csv"
EXPECTED_START_SHA = "cf8d7be2cac53120e68fb74b31033f5174f232d8"
EXPECTED_MAP_SHA = "2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570"
EXPECTED_BAG_SHA = "860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db"
EXPECTED_CONFIG_SHA = "4e9584a4c1d5c2ada963700892880cdf2a7f4e75e43f0ff258b5fd4272af7d77"
EXPECTED_EXTRINSICS_SHA = "fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414"
EXPECTED_DCREG_SHA = "ce7db8220f549a4a4391729e3bf4de4d4ab74635"
EXPECTED_FASTLIO_SHA = "7cc4175de6f8ba2edf34bab02a42195b141027e9"
EXPECTED_VISUAL_COUNT = 1802
EXPECTED_SCANS = 4127
MODES = (
    "DCREG_ONLY",
    "MULTISTART_OBJECTIVE",
    "MULTISTART_VISUAL",
    "FULL_ROUTER",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git_output(cwd: Path, *args: str) -> str:
    return subprocess.check_output(["git", "-C", str(cwd), *args], text=True).strip()


def check_frozen_inputs():
    branch = git_output(WORKSPACE, "branch", "--show-current")
    head = git_output(WORKSPACE, "rev-parse", "HEAD")
    remote = subprocess.check_output(
        ["git", "-C", str(WORKSPACE), "ls-remote", "origin", "refs/heads/paper"],
        text=True,
    ).strip()
    if branch != "paper" or head != EXPECTED_START_SHA:
        raise RuntimeError(f"paper baseline mismatch: branch={branch} HEAD={head}")
    if remote.split()[0] != EXPECTED_START_SHA:
        raise RuntimeError(f"origin/paper mismatch: {remote}")
    if sha256(MAP) != EXPECTED_MAP_SHA or sha256(EXTERNAL_MAP) != EXPECTED_MAP_SHA:
        raise RuntimeError("frozen map exact-SHA verification failed")
    if not MAP_PROVENANCE.is_file():
        raise RuntimeError(f"external map provenance missing: {MAP_PROVENANCE}")
    manifest = dict(
        line.split("=", 1)
        for line in (INPUT / "input_manifest.txt").read_text().splitlines()
        if "=" in line
    )
    if manifest.get("bag_sha256") != EXPECTED_BAG_SHA:
        raise RuntimeError("prepared bag SHA does not match the frozen manifest")
    if manifest.get("map_sha256") != EXPECTED_MAP_SHA:
        raise RuntimeError("prepared map SHA does not match the frozen manifest")
    if int(manifest.get("imu_count", "0")) != 83342 or int(
        manifest.get("scan_count", "0")
    ) != EXPECTED_SCANS:
        raise RuntimeError("prepared input counts do not match frozen Floor01 data")
    if sha256(BAG) != EXPECTED_BAG_SHA:
        raise RuntimeError("source runtime bag SHA mismatch")
    if sha256(CONFIG) != EXPECTED_CONFIG_SHA:
        raise RuntimeError("source Floor01 runtime config SHA mismatch")
    if sha256(EXTRINSICS) != EXPECTED_EXTRINSICS_SHA:
        raise RuntimeError("official calibration SHA mismatch")
    dcreg_root = Path("/home/jian/livox_ws/DCReg")
    fastlio_root = Path("/media/jian/HIKVISION/comparison algorithm/FAST_LIO2")
    if git_output(dcreg_root, "rev-parse", "HEAD") != EXPECTED_DCREG_SHA:
        raise RuntimeError("DCReg pinned commit mismatch")
    if git_output(fastlio_root, "rev-parse", "HEAD") != EXPECTED_FASTLIO_SHA:
        raise RuntimeError("FAST-LIO2 pinned commit mismatch")
    for name, root in (("DCReg", dcreg_root), ("FAST-LIO2", fastlio_root)):
        if git_output(root, "status", "--short"):
            raise RuntimeError(f"pinned {name} source worktree is dirty")
    frozen_visual = subprocess.check_output(
        [
            "git",
            "-C",
            str(WORKSPACE),
            "show",
            f"{EXPECTED_START_SHA}:{VISUAL_SOURCE.relative_to(WORKSPACE)}",
        ]
    )
    if hashlib.sha256(frozen_visual).hexdigest() != sha256(VISUAL_SOURCE):
        raise RuntimeError("P4-I3 frozen visual CSV differs from paper baseline")
    print(f"PAPER_HEAD={head}", flush=True)
    print(f"ORIGIN_PAPER={remote}", flush=True)
    print(f"MAP_SHA256={EXPECTED_MAP_SHA} MAP_RECOVERY=EXACT", flush=True)
    print(f"BAG_SHA256={EXPECTED_BAG_SHA}", flush=True)
    print(f"VISUAL_SHA256={sha256(VISUAL_SOURCE)}", flush=True)


def prepare_visual(path: Path):
    calibration = yaml.safe_load(EXTRINSICS.read_text())
    t_ic = np.asarray(
        calibration["rgb_camera_to_imu"]["data"], dtype=np.float64
    ).reshape(4, 4)
    rows = []
    previous_tx = 0
    with VISUAL_SOURCE.open(newline="") as stream:
        for row in csv.DictReader(stream):
            if row["status"] != "VALID":
                continue
            tx_ref = int(row["transaction_ref"])
            tx_cur = int(row["transaction_cur"])
            stamp_ref = int(row["timestamp_ref_ns"])
            stamp_cur = int(row["timestamp_cur_ns"])
            if tx_cur != tx_ref + 1 or tx_cur <= previous_tx or stamp_cur <= stamp_ref:
                raise RuntimeError("invalid P4-I3 frozen visual transaction lineage")
            pnp = np.eye(4)
            pnp[:3, :] = np.asarray(
                [
                    float(row[f"T_Ccur_Cref_{axis}{column}"])
                    for axis in range(3)
                    for column in range(4)
                ],
                dtype=np.float64,
            ).reshape(3, 4)
            # Exact P4-I3/P4-I4 frame-direction convention: rotation is used
            # only in the origin/lever-arm conversion, never as a filter cue.
            relative_imu = t_ic @ np.linalg.inv(pnp) @ np.linalg.inv(t_ic)
            translation = relative_imu[:3, 3]
            if not np.all(np.isfinite(translation)):
                raise RuntimeError("nonfinite frozen P4-I3 relative translation")
            rows.append(
                [
                    tx_ref,
                    tx_cur,
                    stamp_ref,
                    stamp_cur,
                    *translation.tolist(),
                    int(row["pnp_inliers"]),
                    float(row["inlier_ratio"]),
                    float(row["reprojection_rmse_px"]),
                ]
            )
            previous_tx = tx_cur
    if len(rows) != EXPECTED_VISUAL_COUNT:
        raise RuntimeError(f"expected 1802 VALID visual pairs, got {len(rows)}")
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(
            [
                "transaction_ref",
                "transaction_cur",
                "timestamp_ref_ns",
                "timestamp_cur_ns",
                "zx",
                "zy",
                "zz",
                "inliers",
                "ratio",
                "reprojection",
            ]
        )
        writer.writerows(rows)
    print(f"FROZEN_VISUAL_PREPARED rows={len(rows)} sha256={sha256(path)}", flush=True)


def read_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def check_baseline_gate():
    path = OUT / "baseline_replay.csv"
    rows = read_csv(path)
    saved = read_csv(INPUT / "filter_scans.csv")
    if len(rows) != EXPECTED_SCANS or len(saved) != EXPECTED_SCANS:
        raise RuntimeError("baseline replay frame count mismatch")
    maxima = {
        name: max(float(row[name]) for row in rows)
        for name in (
            "predictor_t_difference_m",
            "predictor_r_difference_deg",
            "raw_t_difference_m",
            "raw_r_difference_deg",
            "used_t_difference_m",
            "used_r_difference_deg",
            "corrected_t_difference_m",
            "corrected_r_difference_deg",
        )
    }
    for replay, reference in zip(rows, saved):
        if replay["stamp_ns"] != reference["stamp_ns"]:
            raise RuntimeError("baseline timestamp mismatch")
        if replay["source_hash_expected"] != replay["source_hash_actual"]:
            raise RuntimeError("baseline source hash mismatch")
        if replay["replayed_converged"] != "1":
            raise RuntimeError("baseline NDT non-convergence")
    if not all(
        maxima[key] < 0.005
        for key in (
            "predictor_t_difference_m",
            "raw_t_difference_m",
            "used_t_difference_m",
            "corrected_t_difference_m",
        )
    ) or not all(
        maxima[key] < 0.05
        for key in (
            "predictor_r_difference_deg",
            "raw_r_difference_deg",
            "used_r_difference_deg",
            "corrected_r_difference_deg",
        )
    ):
        raise RuntimeError(f"BASELINE_REPLAY_GATE_FAIL: {maxima}")
    trajectory = read_csv(OUT / "trajectory_BASELINE.csv")
    if len(trajectory) != EXPECTED_SCANS:
        raise RuntimeError("baseline corrected trajectory missing/incomplete")
    print(f"BASELINE_REPLAY_GATE_PASS {maxima}", flush=True)
    return maxima


def run_logged(command, log_path):
    environment = os.environ.copy()
    environment.pop("LD_LIBRARY_PATH", None)
    print("RUN", " ".join(str(item) for item in command), flush=True)
    with log_path.open("w") as log:
        process = subprocess.Popen(
            command,
            cwd=WORKSPACE,
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        assert process.stdout is not None
        for line in process.stdout:
            print(line, end="", flush=True)
            log.write(line)
            log.flush()
        status = process.wait()
    if status:
        raise RuntimeError(f"command failed with exit={status}; see {log_path}")


def aggregate_mode_csv(prefix: str, filename: str):
    part_paths = [OUT / f"{prefix}_{mode}.csv" for mode in MODES]
    rows = []
    header = None
    for part_path in part_paths:
        part_rows = read_csv(part_path)
        with part_path.open(newline="") as stream:
            part_header = next(csv.reader(stream))
        if header is None:
            header = part_header
        elif header != part_header:
            raise RuntimeError(f"inconsistent {prefix} headers")
        rows.extend(part_rows)
    if header is None:
        raise RuntimeError(f"no {prefix} mode outputs")
    with (OUT / filename).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=header, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def aggregate_mode_outputs():
    for prefix, filename in (
        ("branch_events", "branch_events.csv"),
        ("dcreg_events", "dcreg_events.csv"),
        ("multistart_events", "multistart_events.csv"),
        ("candidate_modes", "candidate_modes.csv"),
        ("visual_arbitration", "visual_arbitration.csv"),
        ("runtime_breakdown", "runtime_breakdown.csv"),
    ):
        aggregate_mode_csv(prefix, filename)


def build_runner():
    cmake_source = PACKAGE / "scripts/p6_i1"
    run_logged(
        ["cmake", "-S", str(cmake_source), "-B", str(BUILD),
         "-DCMAKE_BUILD_TYPE=Release"],
        OUT / "runner_cmake_configure.log",
    )
    run_logged(
        ["cmake", "--build", str(BUILD), "--parallel", "2"],
        OUT / "runner_build.log",
    )
    if not EXECUTABLE.is_file():
        raise RuntimeError(f"P6 executable missing after build: {EXECUTABLE}")
    source_paths = (
        PACKAGE / "scripts/p6_i1_branched_recovery.cpp",
        PACKAGE / "scripts/p4_i2_state_contamination_replay.cpp",
        PACKAGE / "scripts/p5_i1_ndt_mode_landscape.cpp",
        PACKAGE / "src/fastlio2_frontend_ikfom.cpp",
        PACKAGE / "src/registration_geometry.cpp",
        PACKAGE / "include/dog_prior_map_fastlio2_frontend_exp/registration_geometry.hpp",
        PACKAGE / "scripts/p6_i1_run.py",
        PACKAGE / "scripts/p6_i1_report.py",
        cmake_source / "CMakeLists.txt",
    )
    lines = [
        "P6-I1 reproducible runner build provenance",
        f"workspace_head={git_output(WORKSPACE, 'rev-parse', 'HEAD')}",
        f"build_type=Release",
        f"executable={EXECUTABLE}",
        f"executable_sha256={sha256(EXECUTABLE)}",
        f"map_sha256={EXPECTED_MAP_SHA}",
        f"DCReg_commit={EXPECTED_DCREG_SHA}",
        f"FAST_LIO2_commit={EXPECTED_FASTLIO_SHA}",
    ]
    for source_path in source_paths:
        lines.append(f"source_sha256[{source_path.relative_to(WORKSPACE)}]={sha256(source_path)}")
    cache = BUILD / "CMakeCache.txt"
    if cache.is_file():
        for entry in cache.read_text().splitlines():
            if entry.startswith(("CMAKE_BUILD_TYPE:STRING=", "CMAKE_CXX_COMPILER:FILEPATH=")):
                lines.append(f"cmake_cache={entry}")
    (OUT / "build_provenance.txt").write_text("\n".join(lines) + "\n")
    print(f"P6_RUNNER_BUILT sha256={sha256(EXECUTABLE)}", flush=True)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    check_frozen_inputs()
    build_runner()
    visual_path = INPUT / "p4_i3_visual_valid_imu_translation.csv"
    prepare_visual(visual_path)
    run_logged(
        [
            str(EXECUTABLE),
            "baseline",
            str(INPUT / "imu.csv"),
            str(INPUT / "filter_scans.csv"),
            str(INPUT / "scans.csv"),
            str(INPUT / "request_xyz_f32.bin"),
            str(MAP),
            str(INPUT / "params.txt"),
            str(OUT / "baseline_replay.csv"),
            str(OUT / "trajectory_BASELINE.csv"),
        ],
        OUT / "baseline_replay.log",
    )
    gate = check_baseline_gate()
    for mode in MODES:
        run_logged(
            [
                str(EXECUTABLE),
                mode,
                str(INPUT / "imu.csv"),
                str(INPUT / "filter_scans.csv"),
                str(INPUT / "scans.csv"),
                str(INPUT / "request_xyz_f32.bin"),
                str(MAP),
                str(INPUT / "params.txt"),
                str(visual_path),
                str(OUT / f"trajectory_{mode}.csv"),
                str(OUT / f"branch_events_{mode}.csv"),
                str(OUT / f"dcreg_events_{mode}.csv"),
                str(OUT / f"multistart_events_{mode}.csv"),
                str(OUT / f"candidate_modes_{mode}.csv"),
                str(OUT / f"visual_arbitration_{mode}.csv"),
                str(OUT / f"runtime_breakdown_{mode}.csv"),
            ],
            OUT / f"{mode}.log",
        )
        trajectory = read_csv(OUT / f"trajectory_{mode}.csv")
        if len(trajectory) != EXPECTED_SCANS:
            raise RuntimeError(f"{mode} trajectory incomplete: {len(trajectory)}")
    aggregate_mode_outputs()
    # No GT was opened above. The evaluator is started only after all five
    # closed-loop trajectories exist and have the exact expected frame count.
    gate_path = OUT / "baseline_gate.txt"
    gate_path.write_text(
        "P6-I1 baseline replay gate PASS\n"
        + "\n".join(f"{key}={value:.17g}" for key, value in gate.items())
        + "\n"
    )
    evaluator = PACKAGE / "scripts/p6_i1_report.py"
    run_logged(
        [sys.executable, str(evaluator)],
        OUT / "posthoc_evaluation.log",
    )
    print("PAPER_P6_I1_ALL_REPLAYS_AND_POSTHOC_EVALUATION_COMPLETE", flush=True)


if __name__ == "__main__":
    main()
