#!/usr/bin/env python3
"""Run the fixed 67 s Corridor01 replay without any runtime GT/reference path."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
from pathlib import Path

import yaml


WORKSPACE = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = WORKSPACE / "src/dog_prior_map_localization/config/datasets/superloc_corridor01_public_raw_canonical.yaml"
DEFAULT_EXECUTABLE = WORKSPACE / "build/p7_b/p7_single_state_runner"
DEFAULT_OUTPUT = WORKSPACE / "docs/p8_corridor01_dataset_initial_velocity_r1/replay"


def reject_reference_paths(paths):
    for key, value in paths.items():
        if not isinstance(value, str):
            continue
        token = value.lower()
        if any(term in token for term in ("/gt/", "groundtruth", "reference_trajectory", "gt.txt")):
            raise ValueError(f"runtime dataset config contains a reference/GT path: {key}")


def csv_vector(values):
    if len(values) != 3:
        raise ValueError("initial-state vector must contain exactly three values")
    return ",".join(format(float(value), ".17g") for value in values)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--executable", type=Path, default=DEFAULT_EXECUTABLE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--mode", choices=("all", "baseline", "dataset_velocity"), default="all")
    parser.add_argument("--frames", type=int)
    args = parser.parse_args()

    config = yaml.safe_load(args.config.read_text())
    paths = config["paths"]
    initial = config["initial_state"]
    reject_reference_paths(paths)
    if config["start_time_s"] != 67.0:
        raise ValueError("this replay contract requires bag-relative start_time=67.0 s")
    if not config["localization_mode"] or not config["imu_preroll"]:
        raise ValueError("localization mode and causal IMU pre-roll must be enabled")
    if not initial["use_initial_velocity"]:
        raise ValueError("dataset initial velocity is disabled in the runtime YAML")
    if initial["velocity_frame"] != "raw_map_world":
        raise ValueError("velocity must be expressed in the P7 raw-map/world frame")
    if int(initial["anchor_timestamp_ns"]) != int(paths["one_frame_anchor_imu_header_ns"]):
        raise ValueError("startup epoch and frozen 67 s anchor differ")
    if not args.executable.is_file():
        raise FileNotFoundError(f"P7 replay executable does not exist: {args.executable}")

    args.output.mkdir(parents=True, exist_ok=True)
    used_config = args.output / "dataset_config_used.yaml"
    shutil.copyfile(args.config, used_config)
    frame_count = args.frames if args.frames is not None else int(initial["replay_frame_count"])
    if frame_count < 1 or frame_count > int(initial["replay_frame_count"]):
        raise ValueError("requested replay frame count is outside the frozen YAML bound")
    common = [
        paths["imu_csv"], paths["filter_scans_csv"], paths["timed_scan_index_csv"],
        paths["timed_points_bin"], paths["raw_map"], paths["runtime_parameters"],
        "", "", "", str(frame_count),
        str(int(initial["anchor_timestamp_ns"])),
    ]
    modes = ("baseline", "dataset_velocity") if args.mode == "all" else (args.mode,)
    for mode in modes:
        output_dir = args.output / mode
        output_dir.mkdir(parents=True, exist_ok=True)
        common[6] = str(output_dir / "trajectory.csv")
        common[7] = str(output_dir / "registration.csv")
        common[8] = str(output_dir / "runtime.csv")
        pose = ",".join(format(float(value), ".17g")
                        for value in config["frames"]["T_world_imu_row_major"])
        command = [
            str(args.executable), *common,
            "--dataset-state-replay",
            "BASELINE" if mode == "baseline" else "DATASET_VELOCITY",
            str(int(config["imu_calibration"]["window_start_ns"])),
            str(int(config["imu_calibration"]["window_end_ns"])),
            str(int(initial["first_transaction_id"])),
            pose,
            csv_vector(initial["velocity_m_s"]),
            csv_vector(initial["gyro_bias_rad_s"]),
            csv_vector(initial["accel_bias_m_s2"]),
            format(float(initial["velocity_std_m_s"]), ".17g"),
            format(float(initial["gyro_bias_std_rad_s"]), ".17g"),
            format(float(initial["accel_bias_std_m_s2"]), ".17g"),
        ]
        (output_dir / "command.txt").write_text(" ".join(command) + "\n")
        environment = os.environ.copy()
        system_library_dir = "/usr/lib/x86_64-linux-gnu"
        existing_library_path = environment.get("LD_LIBRARY_PATH", "")
        environment["LD_LIBRARY_PATH"] = system_library_dir + (
            os.pathsep + existing_library_path if existing_library_path else "")
        (output_dir / "runtime_environment.txt").write_text(
            f"LD_LIBRARY_PATH={environment['LD_LIBRARY_PATH']}\n")
        with (output_dir / "run.log").open("w") as log:
            completed = subprocess.run(command, cwd=WORKSPACE, stdout=log,
                                       stderr=subprocess.STDOUT, text=True,
                                       env=environment)
        if completed.returncode != 0:
            raise RuntimeError(f"{mode} replay failed; inspect {output_dir / 'run.log'}")
        print(f"MODE={mode} OUTPUT={output_dir} EXIT=0")
    print("REFERENCE_USED_AT_RUNTIME=NO")
    print("GT_UPDATES_AFTER_INITIALIZATION=0")
    print(f"DATASET_CONFIG={args.config}")


if __name__ == "__main__":
    main()
