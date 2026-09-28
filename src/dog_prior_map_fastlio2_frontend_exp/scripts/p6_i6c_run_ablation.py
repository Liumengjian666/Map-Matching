#!/usr/bin/env python3
"""Dispatch one I6C ablation profile to the shared closed-loop core."""

from __future__ import annotations

import argparse
import csv
import hashlib
import subprocess
from pathlib import Path

import yaml


HERE = Path(__file__).resolve().parent
DEFAULT_CONFIG = HERE.parent / "config/ablation.yaml"
DEFAULT_EXECUTABLE = Path("/tmp/p6-i6c-build/p6_i6b_closed_loop")
MODE_BY_FLAGS = {
    (False, False): "STRICT_BASELINE",
    (True, False): "UOBS_ONLY",
    (False, True): "UNONLOCAL_ONLY",
    (True, True): "DUAL_RELIABILITY",
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", choices=("B0", "B1", "B2", "B3", "B4"), required=True)
    parser.add_argument("--dataset", choices=("Floor01", "Corridor01"), required=True)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--executable", type=Path, default=DEFAULT_EXECUTABLE)
    parser.add_argument("--imu", type=Path, required=True)
    parser.add_argument("--filter-scans", type=Path, required=True)
    parser.add_argument("--scans", type=Path, required=True)
    parser.add_argument("--clouds", type=Path, required=True)
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--params", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frame-limit", type=int)
    parser.add_argument("--init-stamp-ns", type=int)
    args = parser.parse_args()

    config = yaml.safe_load(args.config.read_text())
    if config.get("schema_version") != 1:
        raise RuntimeError("unsupported ablation config schema")
    profile = config["profiles"][args.profile]
    flags = tuple(bool(profile[key]) for key in ("enable_u_obs", "enable_u_nonlocal"))
    runner_mode = MODE_BY_FLAGS[flags]
    if profile["runner_mode"] != runner_mode:
        raise RuntimeError(f"{args.profile}: config flags disagree with runner_mode")

    availability = config["module_availability"]
    unavailable = []
    if profile["enable_u_obs"] and not availability["u_obs"]:
        unavailable.append(availability["u_obs_reason"])
    if profile["enable_u_nonlocal"] and not availability["u_nonlocal"]:
        unavailable.append("U_NONLOCAL_NOT_VALIDATED")
    if profile["enable_vision"] and not availability["vision"]:
        unavailable.append(availability["vision_reason"])
    if profile["enable_vision"]:
        unavailable.append("VISION_FACTOR_NOT_CONSUMED_BY_FILTER_CORE")

    args.output.mkdir(parents=True, exist_ok=True)
    status_path = args.output / "ablation_status.txt"
    if unavailable:
        status_path.write_text(
            f"profile={args.profile}\nstatus=NOT_AVAILABLE\n"
            f"reasons={';'.join(unavailable)}\nndt_align_calls=0\n")
        print(f"{args.profile} NOT_AVAILABLE: {'; '.join(unavailable)}")
        return 3

    dataset = config["dataset_maps"][args.dataset]
    actual_map_hash = sha256(args.map)
    if actual_map_hash != dataset["map_sha256"]:
        raise RuntimeError(f"{args.dataset} map SHA-256 mismatch")
    if not args.executable.is_file():
        raise FileNotFoundError(args.executable)

    args.output.mkdir(parents=True, exist_ok=True)
    profile_prefix = args.output / args.profile
    command = [str(args.executable), runner_mode, str(args.imu),
               str(args.filter_scans), str(args.scans), str(args.clouds),
               str(args.map), str(args.params),
               str(profile_prefix)+"_trajectory.csv",
               str(profile_prefix)+"_reliability.csv",
               str(profile_prefix)+"_runtime.csv"]
    with args.scans.open(newline="") as stream:
        available_scans = sum(1 for _ in csv.DictReader(stream))
    frame_limit = available_scans if args.frame_limit is None else args.frame_limit
    if frame_limit <= 0 or frame_limit > available_scans:
        raise RuntimeError("frame limit must lie within the prepared scan table")
    init_stamp = 0 if args.init_stamp_ns is None else args.init_stamp_ns
    command.extend((str(frame_limit), str(init_stamp), dataset["map_profile"]))
    provenance = [f"profile={args.profile}", f"label={profile['label']}",
                  "status=RUNNING", f"runner_mode={runner_mode}",
                  f"enable_u_obs={int(profile['enable_u_obs'])}",
                  f"enable_u_nonlocal={int(profile['enable_u_nonlocal'])}",
                  f"enable_vision={int(profile['enable_vision'])}",
                  f"map_profile={dataset['map_profile']}",
                  f"map_sha256={actual_map_hash}",
                  f"config_sha256={sha256(args.config)}",
                  "algorithm_core=p6_i1_branched_recovery shared core"]
    status_path.write_text("\n".join(provenance)+"\n")
    subprocess.run(command, check=True)
    with status_path.open("a") as stream:
        stream.write("status=COMPLETE\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
