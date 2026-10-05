#!/usr/bin/env python3
"""Run independent 100-frame Corridor01 replays with velocity-prior scale varied only."""

from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
import os
import re
import shlex
import subprocess
from pathlib import Path
from typing import Optional

import numpy as np
import yaml


WORKSPACE = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = WORKSPACE / (
    "src/dog_prior_map_localization/config/datasets/"
    "superloc_corridor01_public_raw_canonical.yaml"
)
DEFAULT_EXECUTABLE = WORKSPACE / "build/p7_b/p7_single_state_runner"
DEFAULT_OUTPUT = WORKSPACE / "docs/p8_corridor01_weak_velocity_prior_r1/replay"
PROFILES = (("V0", 0.5), ("V1", 2.0), ("V2", 10.0), ("V3", 100.0))
EXPECTED_BIAS_STD = {"gyro_bias_std_rad_s": 0.05, "accel_bias_std_m_s2": 0.5}
EXPECTED_GRAVITY = np.array([0.7620000204116083, 0.062189083539576764, -9.779159958134503])


def vector_csv(values) -> str:
    if len(values) != 3:
        raise ValueError("expected a 3-vector")
    return ",".join(format(float(value), ".17g") for value in values)


def read_csv(path: Path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def fingerprint(path: Path) -> dict:
    path = path.resolve()
    if not path.is_file():
        raise FileNotFoundError(path)
    return {"path": str(path), "size_bytes": path.stat().st_size, "sha256": sha256(path)}


def runtime_input_paths(config: dict, config_path: Path, executable: Path) -> dict[str, Path]:
    paths = config["paths"]
    consumed_keys = ("imu_csv", "filter_scans_csv", "timed_scan_index_csv",
                     "timed_points_bin", "raw_map", "runtime_parameters")
    result = {key: Path(paths[key]) for key in consumed_keys}
    result["dataset_config"] = config_path
    result["executable"] = executable
    return result


def fingerprint_inputs(config: dict, config_path: Path, executable: Path) -> dict:
    return {key: fingerprint(path) for key, path in runtime_input_paths(
        config, config_path, executable).items()}


def shared_config_identity(config: dict) -> str:
    shared = copy.deepcopy(config)
    shared["initial_state"].pop("velocity_std_m_s", None)
    payload = json.dumps(shared, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def logged_vector(line: str, key: str, next_key: Optional[str] = None) -> np.ndarray:
    suffix = rf"(?=\s+{re.escape(next_key)}=|$)" if next_key else r"$"
    match = re.search(rf"(?:^|\s){re.escape(key)}=([^\n]*?){suffix}", line)
    if not match:
        raise ValueError(f"missing {key} vector in diagnostic: {line}")
    result = np.fromstring(match.group(1), sep=" ")
    if result.size != 3 or not np.isfinite(result).all():
        raise ValueError(f"malformed {key} vector in diagnostic: {line}")
    return result


def make_command(config: dict, output_dir: Path, executable: Path, velocity_std: float):
    paths = config["paths"]
    initial = config["initial_state"]
    common = [
        paths["imu_csv"], paths["filter_scans_csv"], paths["timed_scan_index_csv"],
        paths["timed_points_bin"], paths["raw_map"], paths["runtime_parameters"],
        str(output_dir / "trajectory.csv"), str(output_dir / "registration.csv"),
        str(output_dir / "runtime.csv"), str(int(initial["replay_frame_count"])),
        str(int(initial["anchor_timestamp_ns"])),
    ]
    pose = ",".join(format(float(value), ".17g")
                     for value in config["frames"]["T_world_imu_row_major"])
    command = [
        str(executable), *common, "--dataset-state-replay", "DATASET_VELOCITY",
        str(int(config["imu_calibration"]["window_start_ns"])),
        str(int(config["imu_calibration"]["window_end_ns"])),
        str(int(initial["first_transaction_id"])), pose,
        vector_csv(initial["velocity_m_s"]), vector_csv(initial["gyro_bias_rad_s"]),
        vector_csv(initial["accel_bias_m_s2"]), format(velocity_std, ".17g"),
        format(float(initial["gyro_bias_std_rad_s"]), ".17g"),
        format(float(initial["accel_bias_std_m_s2"]), ".17g"),
        "--initial-gravity-map", vector_csv(initial["gravity_m_s2"]),
    ]
    return command


def validate_contract(config: dict, config_path: Path, executable: Path, output: Path):
    paths, initial = config["paths"], config["initial_state"]
    if any(any(tag in str(value).lower() for tag in ("/gt/", "groundtruth", "gt.txt"))
           for value in paths.values()):
        raise ValueError("runtime config contains a GT/reference path")
    if config["start_time_s"] != 67.0 or not config["localization_mode"] or not config["imu_preroll"]:
        raise ValueError("Corridor01 causal 67 s startup contract mismatch")
    if int(initial["anchor_timestamp_ns"]) != 1517157286165072000 or \
            int(initial["first_transaction_id"]) != 666 or \
            int(initial["replay_frame_count"]) != 100:
        raise ValueError("replay must be the frozen TX666-TX765 100-frame cohort")
    if not initial["use_initial_velocity"] or not initial["use_initial_biases"] or \
            not initial["use_covariance_overrides"] or not initial["use_initial_gravity"]:
        raise ValueError("dataset-specific velocity, zero biases, covariance, and gravity are required")
    if not np.array_equal(np.asarray(initial["gyro_bias_rad_s"], dtype=float), np.zeros(3)) or \
            not np.array_equal(np.asarray(initial["accel_bias_m_s2"], dtype=float), np.zeros(3)):
        raise ValueError("bias means must stay exactly zero for every profile")
    for key, expected in EXPECTED_BIAS_STD.items():
        if not np.isclose(float(initial[key]), expected, rtol=0.0, atol=1e-12):
            raise ValueError(f"{key} must remain frozen at {expected}")
    if not np.allclose(np.asarray(initial["gravity_m_s2"], dtype=float),
                       EXPECTED_GRAVITY, rtol=0.0, atol=1e-12):
        raise ValueError("dataset gravity must remain at the frozen Corridor01 constant")
    if not executable.is_file():
        raise FileNotFoundError(executable)
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"refusing to overwrite non-empty output directory: {output}")
    output.mkdir(parents=True, exist_ok=True)
    return sha256(config_path)


def run_profile(name: str, velocity_std: float, base_config: dict,
                config_sha: str, executable: Path, output_root: Path):
    output_dir = output_root / name
    output_dir.mkdir()
    profile_config = copy.deepcopy(base_config)
    profile_config["initial_state"]["velocity_std_m_s"] = velocity_std
    profile_config_path = output_dir / "dataset_config_used.yaml"
    profile_config_path.write_text(yaml.safe_dump(profile_config, sort_keys=False))
    profile_config_sha = sha256(profile_config_path)
    command = make_command(profile_config, output_dir, executable, velocity_std)
    (output_dir / "command.txt").write_text(shlex.join(command) + "\n")
    (output_dir / "provenance.json").write_text(json.dumps({
        "profile": name,
        "velocity_std_m_s": velocity_std,
        "velocity_variance_m2_s2": velocity_std * velocity_std,
        "dataset_config_sha256": config_sha,
        "profile_config_sha256": profile_config_sha,
        "shared_config_without_velocity_std_sha256": shared_config_identity(profile_config),
        "dataset_config_snapshot": "dataset_config_used.yaml",
        "runtime_reference_or_gt_access": False,
    }, indent=2) + "\n")
    environment = os.environ.copy()
    system_library_dir = "/usr/lib/x86_64-linux-gnu"
    existing = environment.get("LD_LIBRARY_PATH", "")
    environment["LD_LIBRARY_PATH"] = system_library_dir + (os.pathsep + existing if existing else "")
    (output_dir / "runtime_environment.txt").write_text(
        f"LD_LIBRARY_PATH={environment['LD_LIBRARY_PATH']}\n")
    with (output_dir / "run.log").open("w") as log:
        completed = subprocess.run(command, cwd=WORKSPACE, stdout=log,
                                   stderr=subprocess.STDOUT, text=True, env=environment)
    lines = (output_dir / "run.log").read_text().splitlines()
    result = {"profile": name, "velocity_std_m_s": velocity_std,
              "velocity_variance_m2_s2": velocity_std * velocity_std,
              "profile_config_sha256": profile_config_sha,
              "shared_config_without_velocity_std_sha256": shared_config_identity(profile_config),
              "return_code": completed.returncode, "frames": 0,
              "diagnostics_valid": False, "error": None}
    if completed.returncode != 0:
        result["error"] = next((line for line in reversed(lines)
                                if "FIRST_BAD_TX=" in line),
                               f"runner_exit_code_{completed.returncode}")
        return result

    cov_lines = [line for line in lines if line.startswith("INITIAL_STATE_COVARIANCE_APPLIED")]
    injection_lines = [line for line in lines if line.startswith("INITIAL_VELOCITY_INJECTION ")]
    injection_count_lines = [line for line in lines if line.startswith("INITIAL_VELOCITY_INJECTION_COUNT=")]
    gravity_count_lines = [line for line in lines if line.startswith("INITIAL_GRAVITY_INJECTION_COUNT=")]
    if len(cov_lines) != 1 or len(injection_lines) != 1 or \
            injection_count_lines != ["INITIAL_VELOCITY_INJECTION_COUNT=1"] or \
            gravity_count_lines != ["INITIAL_GRAVITY_INJECTION_COUNT=1"]:
        result["error"] = "initial_state_injection_diagnostics_missing_or_repeated"
        return result
    gravity_lines = [line for line in lines if line.startswith("INITIAL_GRAVITY_INJECTION ")]
    reanchor_lines = [line for line in lines if line.startswith("REANCHOR timestamp_ns=")]
    state_vector_lines = [line for line in lines if line.startswith("position=")]
    if len(gravity_lines) != 1 or len(reanchor_lines) != 1 or len(state_vector_lines) != 1:
        result["error"] = "initial_state_runtime_values_missing_or_repeated"
        return result
    velocity_log = injection_lines[0]
    gravity_log = gravity_lines[0]
    reanchor = reanchor_lines[0]
    state_vector_log = state_vector_lines[0]
    actual_velocity = logged_vector(velocity_log, "velocity_world_m_s", "count")
    actual_gravity = logged_vector(gravity_log, "gravity_map_m_s2", "count")
    actual_velocity_state = logged_vector(state_vector_log, "velocity", "gyro_bias")
    actual_gyro_bias = logged_vector(state_vector_log, "gyro_bias", "accel_bias")
    actual_accel_bias = logged_vector(state_vector_log, "accel_bias", "gravity_map_effective")
    actual_state_gravity = logged_vector(state_vector_log, "gravity_map_effective")
    expected_velocity = np.asarray(base_config["initial_state"]["velocity_m_s"], dtype=float)
    expected_gravity = np.asarray(base_config["initial_state"]["gravity_m_s2"], dtype=float)
    expected_anchor = int(base_config["initial_state"]["anchor_timestamp_ns"])
    injected_timestamp = int(re.search(r"timestamp_ns=(\d+)", velocity_log).group(1))
    reanchor_timestamp = int(re.search(r"timestamp_ns=(\d+)", reanchor).group(1))
    if injected_timestamp != expected_anchor or reanchor_timestamp != expected_anchor or \
            int(re.search(r"timestamp_ns=(\d+)", gravity_log).group(1)) != expected_anchor:
        result["error"] = "runtime_initial_state_timestamp_mismatch"
        return result
    if not np.allclose(actual_velocity, expected_velocity, rtol=0.0, atol=1e-11) or \
            not np.allclose(actual_velocity_state, expected_velocity, rtol=0.0, atol=1e-11):
        result["error"] = "runtime_initial_velocity_mismatch"
        return result
    if not np.allclose(actual_gravity, expected_gravity, rtol=0.0, atol=1e-11) or \
            not np.allclose(actual_state_gravity, expected_gravity, rtol=0.0, atol=1e-11):
        result["error"] = "runtime_initial_gravity_mismatch"
        return result
    if not np.allclose(actual_gyro_bias, np.zeros(3), rtol=0.0, atol=1e-12) or \
            not np.allclose(actual_accel_bias, np.zeros(3), rtol=0.0, atol=1e-12):
        result["error"] = "runtime_bias_means_are_not_zero"
        return result
    cov = cov_lines[0]
    expected_variance = velocity_std * velocity_std
    if not np.isclose(float(re.search(r"velocity_std_m_s=([^ ]+)", cov).group(1)),
                      velocity_std, rtol=0.0, atol=1e-12) or \
            not np.isclose(float(re.search(r"velocity_variance_m2_s2=([^ ]+)", cov).group(1)),
                           expected_variance, rtol=0.0, atol=max(1e-12, expected_variance * 1e-12)):
        result["error"] = "configured_velocity_prior_diagnostic_mismatch"
        return result
    if not np.allclose(logged_vector(cov, "P_v", "gyro_bias_std_rad_s"),
                       np.full(3, expected_variance), rtol=1e-10, atol=1e-12):
        result["error"] = "runtime_initial_Pv_does_not_match_sigma_squared"
        return result
    result["initial_pv_diag_m2_s2"] = logged_vector(cov, "P_v", "gyro_bias_std_rad_s").tolist()
    result["initial_pbg_diag_rad2_s2"] = logged_vector(cov, "P_bg", "accel_bias_std_m_s2").tolist()
    result["initial_pba_diag_m2_s4"] = logged_vector(cov, "P_ba").tolist()
    result["initial_velocity_world_m_s"] = initial_velocity = actual_velocity.tolist()
    result["initial_speed_m_s"] = float(np.linalg.norm(initial_velocity))
    result["runtime_injection_verified"] = {
        "anchor_timestamp_ns": expected_anchor,
        "velocity_world_m_s": actual_velocity.tolist(),
        "gravity_map_m_s2": actual_gravity.tolist(),
        "gyro_bias_rad_s": actual_gyro_bias.tolist(),
        "accel_bias_m_s2": actual_accel_bias.tolist(),
        "velocity_injection_count": 1,
        "gravity_injection_count": 1,
    }
    if not np.allclose(result["initial_pbg_diag_rad2_s2"],
                       np.full(3, EXPECTED_BIAS_STD["gyro_bias_std_rad_s"] ** 2),
                       rtol=1e-10, atol=1e-12) or \
            not np.allclose(result["initial_pba_diag_m2_s4"],
                            np.full(3, EXPECTED_BIAS_STD["accel_bias_std_m_s2"] ** 2),
                            rtol=1e-10, atol=1e-12):
        result["error"] = "bias_covariance_changed_across_velocity_profiles"
        return result
    if (output_dir / "trajectory.csv").is_file() and (output_dir / "registration.csv").is_file():
        trajectory = read_csv(output_dir / "trajectory.csv")
        registration = read_csv(output_dir / "registration.csv")
        tx = list(range(666, 766))
        if [int(row["transaction_id"]) for row in trajectory] != tx or \
                [int(row["transaction_id"]) for row in registration] != tx:
            result["error"] = "replay_transaction_sequence_not_TX666_TX765"
            result["frames"] = len(registration)
            return result
        if "ndt_delta_velocity_x" not in trajectory[0] or \
                "p_v_pre_update_x" not in trajectory[0]:
            result["error"] = "runner_velocity_update_diagnostics_missing"
            return result
        result["frames"] = len(registration)
        result["diagnostics_valid"] = True
    else:
        result["error"] = "replay_csv_outputs_missing"
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--executable", type=Path, default=DEFAULT_EXECUTABLE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--profile", choices=tuple(name for name, _ in PROFILES),
                        action="append", help="select profiles; defaults to all four")
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    config_sha = validate_contract(config, args.config, args.executable, args.output)
    input_fingerprints_before = fingerprint_inputs(config, args.config, args.executable)
    shared_config_sha = shared_config_identity(config)
    selected = PROFILES if not args.profile else tuple(
        profile for profile in PROFILES if profile[0] in args.profile)
    results = [run_profile(name, std, config, config_sha, args.executable, args.output)
               for name, std in selected]
    input_fingerprints_after = fingerprint_inputs(config, args.config, args.executable)
    input_fingerprints_stable = input_fingerprints_before == input_fingerprints_after
    profile_shared_config_hashes = {
        profile["profile"]: profile.get("shared_config_without_velocity_std_sha256")
        for profile in results
    }
    profile_config_contract = all(value == shared_config_sha
                                  for value in profile_shared_config_hashes.values())
    summary = {"workspace": str(WORKSPACE), "config_sha256": config_sha,
               "profiles": results,
               "shared_config_without_velocity_std_sha256": shared_config_sha,
               "profile_shared_config_hashes": profile_shared_config_hashes,
               "only_profile_config_field_swept": "initial_state.velocity_std_m_s",
               "runtime_input_fingerprints_before": input_fingerprints_before,
               "runtime_input_fingerprints_after": input_fingerprints_after,
               "runtime_inputs_unchanged_during_sweep": input_fingerprints_stable,
               "profile_config_contract_pass": profile_config_contract,
               "reference_used_at_runtime": False, "gt_updates_after_initialization": 0}
    (args.output / "run_manifest.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    if not input_fingerprints_stable or not profile_config_contract or \
            any(not profile["diagnostics_valid"] or profile["frames"] != 100
                for profile in results):
        raise SystemExit("one or more requested profiles failed validation; see run_manifest.json")


if __name__ == "__main__":
    main()
