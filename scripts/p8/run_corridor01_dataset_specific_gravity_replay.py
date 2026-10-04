#!/usr/bin/env python3
"""Run the frozen 100-frame Corridor01 single-gravity-variable A/B."""

from __future__ import annotations

import argparse
import os
import re
import shlex
import shutil
import subprocess
from pathlib import Path

import numpy as np
import yaml


WORKSPACE = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = WORKSPACE / "src/dog_prior_map_localization/config/datasets/superloc_corridor01_public_raw_canonical.yaml"
DEFAULT_EXECUTABLE = WORKSPACE / "build/p7_b/p7_single_state_runner"
DEFAULT_OUTPUT = WORKSPACE / "docs/p8_corridor01_dataset_specific_gravity_r1/replay"


def vector_csv(values):
    if len(values) != 3:
        raise ValueError("expected three-vector")
    return ",".join(format(float(v), ".17g") for v in values)


def logged_vector(line, key):
    match = re.search(rf"{re.escape(key)}=([^\n]+)", line)
    if not match:
        raise ValueError(f"missing {key} in runtime log")
    vector = np.fromstring(match.group(1), sep=" ")
    if vector.size != 3:
        raise ValueError(f"malformed {key} in runtime log")
    return vector


def run_profile(name, gravity_override, config, config_path, executable, output_root):
    paths, initial = config["paths"], config["initial_state"]
    output_dir = output_root / name
    output_dir.mkdir(parents=True, exist_ok=True)
    common = [
        paths["imu_csv"], paths["filter_scans_csv"], paths["timed_scan_index_csv"],
        paths["timed_points_bin"], paths["raw_map"], paths["runtime_parameters"],
        str(output_dir / "trajectory.csv"), str(output_dir / "registration.csv"),
        str(output_dir / "runtime.csv"), str(int(initial["replay_frame_count"])),
        str(int(initial["anchor_timestamp_ns"])),
    ]
    pose = ",".join(format(float(v), ".17g")
                     for v in config["frames"]["T_world_imu_row_major"])
    command = [
        str(executable), *common, "--dataset-state-replay", "DATASET_VELOCITY",
        str(int(config["imu_calibration"]["window_start_ns"])),
        str(int(config["imu_calibration"]["window_end_ns"])),
        str(int(initial["first_transaction_id"])), pose,
        vector_csv(initial["velocity_m_s"]), vector_csv(initial["gyro_bias_rad_s"]),
        vector_csv(initial["accel_bias_m_s2"]),
        format(float(initial["velocity_std_m_s"]), ".17g"),
        format(float(initial["gyro_bias_std_rad_s"]), ".17g"),
        format(float(initial["accel_bias_std_m_s2"]), ".17g"),
    ]
    if gravity_override:
        command += ["--initial-gravity-map", vector_csv(initial["gravity_m_s2"])]
    (output_dir / "command.txt").write_text(shlex.join(command) + "\n")
    shutil.copyfile(config_path, output_dir / "dataset_config_used.yaml")
    environment = os.environ.copy()
    system_library_dir = "/usr/lib/x86_64-linux-gnu"
    existing = environment.get("LD_LIBRARY_PATH", "")
    environment["LD_LIBRARY_PATH"] = system_library_dir + (os.pathsep + existing if existing else "")
    (output_dir / "runtime_environment.txt").write_text(
        f"LD_LIBRARY_PATH={environment['LD_LIBRARY_PATH']}\n")
    with (output_dir / "run.log").open("w") as log:
        completed = subprocess.run(command, cwd=WORKSPACE, stdout=log,
                                   stderr=subprocess.STDOUT, text=True, env=environment)
    if completed.returncode:
        raise RuntimeError(f"{name} replay failed; see {output_dir / 'run.log'}")
    lines = (output_dir / "run.log").read_text().splitlines()
    gravity_lines = [line for line in lines if line.startswith("GRAVITY_INITIALIZATION=")]
    count_lines = [line for line in lines if line.startswith("INITIAL_GRAVITY_INJECTION_COUNT=")]
    if len(gravity_lines) != 1 or len(count_lines) != 1:
        raise RuntimeError(f"{name}: missing unique gravity initialization diagnostics")
    actual = logged_vector(gravity_lines[0], "gravity_map")
    if gravity_override:
        if count_lines[0] != "INITIAL_GRAVITY_INJECTION_COUNT=1":
            raise RuntimeError(f"{name}: gravity override was not injected exactly once")
        expected = np.asarray(initial["gravity_m_s2"], dtype=float)
    else:
        if count_lines[0] != "INITIAL_GRAVITY_INJECTION_COUNT=0":
            raise RuntimeError(f"{name}: legacy control unexpectedly used the new override")
        expected = np.asarray(config["gravity_ab_control"]["old_static_transport_map_m_s2"], dtype=float)
    tolerance = float(config["gravity_ab_control"]["old_profile_value_tolerance_m_s"])
    if np.linalg.norm(actual - expected) > tolerance:
        raise RuntimeError(f"{name}: effective initial gravity differs from frozen profile: {actual}")
    if sum(line.startswith("INITIAL_VELOCITY_INJECTION ") for line in lines) != 1:
        raise RuntimeError(f"{name}: initial velocity injection count is not one")
    print(f"PROFILE={name} frames={initial['replay_frame_count']} gravity={actual.tolist()} output={output_dir}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--executable", type=Path, default=DEFAULT_EXECUTABLE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--mode", choices=("all", "old", "new"), default="all")
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    paths, initial = config["paths"], config["initial_state"]
    if any(any(tag in str(value).lower() for tag in ("/gt/", "groundtruth", "gt.txt"))
           for value in paths.values()):
        raise ValueError("runtime config contains a GT/reference path")
    if not config["localization_mode"] or not config["imu_preroll"] or config["start_time_s"] != 67.0:
        raise ValueError("Corridor01 67 s causal replay contract mismatch")
    if not initial["use_initial_velocity"] or not initial["use_initial_biases"] or not initial["use_covariance_overrides"]:
        raise ValueError("dataset velocity/bias/covariance profile must be enabled")
    if not initial["use_initial_gravity"] or initial["gravity_frame"] != "raw_map_world":
        raise ValueError("dataset gravity constant is missing or has the wrong frame")
    if int(initial["replay_frame_count"]) != 100:
        raise ValueError("this task requires the frozen 100-frame TX666-TX765 replay")
    if not args.executable.is_file():
        raise FileNotFoundError(args.executable)
    modes = (("old_gravity", False), ("dataset_specific_gravity", True))
    selected = modes if args.mode == "all" else (modes[0 if args.mode == "old" else 1],)
    args.output.mkdir(parents=True, exist_ok=True)
    for name, use_override in selected:
        run_profile(name, use_override, config, args.config, args.executable, args.output)
    print("REFERENCE_USED_AT_RUNTIME=NO")
    print("GT_UPDATES_AFTER_INITIALIZATION=0")


if __name__ == "__main__":
    main()
