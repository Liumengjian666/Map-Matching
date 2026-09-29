#!/usr/bin/env python3
"""Run one I6D profile through the shared offline FAST-LIO2/NDT core."""

from __future__ import annotations

import argparse
import csv
import hashlib
import os
import subprocess
import time
from pathlib import Path

import yaml


REPO = Path(__file__).resolve().parents[3]
PACKAGE = REPO / "src/dog_prior_map_fastlio2_frontend_exp"
CONFIG = PACKAGE / "config/ablation.yaml"
DEFAULT_EXE = Path("/tmp/p6-i6c-build/p6_i6b_closed_loop")
FLOOR_ROOT = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01")
FLOOR_INPUT = Path("/home/jian/livox_ws/p6_i1_recovery_assets_20260927/input")
CORRIDOR_ROOT = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01")
CORRIDOR_INPUT = CORRIDOR_ROOT / "results/p6_i6c_framework/input"
CORRIDOR_MANIFEST_SHA = "6d722ec6946570cc09d984c1a8ac7ebaaff799f012f9386ae169e3d47747a043"
CORRIDOR_I6D_PARAMS_SHA = "7e42752ff8b84eae2b2da8d7d9fe179db0bb8f364a923e12236d2e91336e357d"
FLOOR_MAP_SHA = "2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570"
FLOOR_MANIFEST_SHA = "75e71ba365b9c2d0b247f4bf3944008994d398f8f5d87fcb2d49e9e7ffd2a09e"
FLOOR_INPUT_SHAS = {
    "imu.csv": "faf83b341d6084df5a84e431d61a15f6680a7465d83e9fc7bba13a03961722c2",
    "filter_scans.csv": "6cf05c6fb23571969263bb0e011c12adb09dcf73d8707420203459f9d41b1ec0",
    "scans.csv": "9371e593c0e625611f053e3ef0a581c52481ccaed392aa8d74d74313caa9938f",
    "request_xyz_f32.bin": "f7b5262552fe8d52f383e50813de2b568fa8a5990c69e2bba55231df8547188f",
    "params.txt": "84819fc178c70a80de075ba02f5e05938b22879ae4238049f537631e7b121c8b",
}
CORRIDOR_MAP_SHA = "103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f"
EVAL_START_NS = 1_517_157_224_188_979_000
MODE_FOR_PROFILE = {
    "B0": "STRICT_BASELINE",
    "B1": "UOBS_ONLY",
    "B2": "UNONLOCAL_ONLY",
    "B3": "DUAL_RELIABILITY",
    "B4": "FULL_ALGORITHM_V1",
}
R2_POLICIES = ("LEGACY_BASE_NO_GATE", "ADAPTIVE_NO_GATE",
               "BASE_SELECTED_NIS", "ADAPTIVE_SELECTED_NIS")
R3_MODES = ("LEGACY", "EXACT_RESIDUAL")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_manifest(path: Path) -> dict[str, str]:
    return dict(line.split("=", 1) for line in path.read_text().splitlines() if "=" in line)


def mode_counts(path: Path, visual_path: Path | None) -> dict[str, int]:
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    result = {
        "frames": len(rows),
        "uobs_valid_frames": sum(row.get("uobs_valid") == "1" for row in rows),
        "uobs_used_frames": sum(row.get("local_curvature_used") == "1" for row in rows),
        "unonlocal_used_frames": sum(row.get("nonlocal_response_used") == "1" for row in rows),
        "nonlocal_probe_frames": sum(row.get("probe_executed") == "1" for row in rows),
        "ndt_align_calls": sum(int(row.get("ndt_call_count", "0")) for row in rows),
        "m0_nonconverged_frames": sum(row.get("M0_converged") != "1" for row in rows),
    }
    if visual_path and visual_path.is_file():
        with visual_path.open(newline="") as stream:
            updates = list(csv.DictReader(stream))
        applied = [row for row in updates
                   if row.get("status") == "APPLIED_PROJECTED_COMPLEMENTARY_TRANSLATION"]
        result["visual_factor_events"] = len(updates)
        result["visual_filter_updates"] = len(applied)
        result["visual_nonzero_state_updates"] = sum(
            float(row["position_correction_norm_m"]) > 1e-12 or
            float(row["velocity_correction_norm_m"]) > 1e-12
            for row in applied)
    else:
        result.update({"visual_factor_events": 0, "visual_filter_updates": 0,
                       "visual_nonzero_state_updates": 0})
    return result


def validate_inputs(dataset: str, input_dir: Path, map_path: Path,
                    params_path: Path) -> tuple[str, int, int, dict[str, str]]:
    if dataset == "Floor01":
        source_bag = FLOOR_ROOT / "results/p3_r10b_fix1_floor01_full_rerun_20260926/floor01_fix1_runtime_topics.bag"
        frozen_input = input_dir / "input_manifest.txt"
        manifest = read_manifest(frozen_input)
        if sha256(frozen_input) != FLOOR_MANIFEST_SHA:
            raise RuntimeError("Floor01 frozen input manifest SHA mismatch")
        if manifest.get("bag_sha256") != "860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db":
            raise RuntimeError("Floor01 frozen source bag identity mismatch")
        for name, expected_sha in FLOOR_INPUT_SHAS.items():
            if sha256(input_dir / name) != expected_sha:
                raise RuntimeError(f"Floor01 frozen input artifact SHA mismatch: {name}")
        if manifest.get("map_sha256") != FLOOR_MAP_SHA or sha256(map_path) != FLOOR_MAP_SHA:
            raise RuntimeError("Floor01 exact frozen map SHA mismatch")
        if sha256(source_bag) != manifest["bag_sha256"]:
            raise RuntimeError("Floor01 frozen raw bag SHA mismatch")
        if int(manifest.get("scan_count", "0")) != 4127 or int(manifest.get("imu_count", "0")) != 83342:
            raise RuntimeError("Floor01 frozen input counts changed")
        init_stamp = 0
        profile = "floor01"
        expected_scans = 4127
    else:
        frozen_input = CORRIDOR_INPUT / "input_manifest.txt"
        if sha256(frozen_input) != CORRIDOR_MANIFEST_SHA:
            raise RuntimeError("Corridor01 I6C v2 input manifest SHA mismatch")
        manifest = read_manifest(frozen_input)
        for key in ("imu_csv_sha256", "filter_scans_csv_sha256", "scans_csv_sha256",
                    "request_xyz_f32_sha256"):
            name = {
                "imu_csv_sha256": "imu.csv",
                "filter_scans_csv_sha256": "filter_scans.csv",
                "scans_csv_sha256": "scans.csv",
                "request_xyz_f32_sha256": "request_xyz_f32.bin",
            }[key]
            if sha256(input_dir / name) != manifest.get(key):
                raise RuntimeError(f"Corridor01 v2 frozen input hash mismatch: {key}")
        if manifest.get("map_sha256") != CORRIDOR_MAP_SHA or sha256(map_path) != CORRIDOR_MAP_SHA:
            raise RuntimeError("Corridor01 normalized map SHA mismatch")
        if int(manifest.get("scan_count_replayed", "0")) != 2726:
            raise RuntimeError("Corridor01 v2 replay scan count changed")
        if manifest.get("evaluation_start_header_stamp_ns") != str(EVAL_START_NS):
            raise RuntimeError("Corridor01 fixed initialization/evaluation epoch changed")
        profile = "corridor01"
        init_stamp = EVAL_START_NS
        expected_scans = 2726
        if sha256(params_path) != CORRIDOR_I6D_PARAMS_SHA:
            raise RuntimeError("Corridor01 I6D official-calibration parameter SHA mismatch")
        # I6D's derived params file must differ from the frozen bundle only in
        # the final seven T_imu_lidar scalars, pinned from official calibration.
        frozen_values = [float(x) for x in (CORRIDOR_INPUT / "params.txt").read_text().split()]
        run_values = [float(x) for x in params_path.read_text().split()]
        if len(frozen_values) != 27 or len(run_values) != 27 or run_values[:20] != frozen_values[:20]:
            raise RuntimeError("I6D Corridor01 params changed non-extrinsic values")
    for filename in ("imu.csv", "filter_scans.csv", "scans.csv", "request_xyz_f32.bin"):
        if not (input_dir / filename).is_file():
            raise FileNotFoundError(input_dir / filename)
    if not params_path.is_file() or not map_path.is_file():
        raise FileNotFoundError(params_path if not params_path.is_file() else map_path)
    return profile, init_stamp, expected_scans, manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", choices=("Floor01", "Corridor01"), required=True)
    parser.add_argument("--profile", choices=tuple(MODE_FOR_PROFILE), required=True)
    parser.add_argument("--input-dir", type=Path)
    parser.add_argument("--map", type=Path)
    parser.add_argument("--params", type=Path)
    parser.add_argument("--visual-csv", type=Path)
    parser.add_argument("--executable", type=Path, default=DEFAULT_EXE)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frame-limit", type=int)
    parser.add_argument("--task-label", default="PAPER-P6-I6D-FULL-ALGORITHM-FIRST")
    parser.add_argument("--r2-policy", choices=R2_POLICIES,
                        default="LEGACY_BASE_NO_GATE")
    parser.add_argument("--r3-mode", choices=R3_MODES, default="LEGACY")
    args = parser.parse_args()
    if args.profile != "B4" and args.r2_policy != "LEGACY_BASE_NO_GATE":
        parser.error("non-LEGACY --r2-policy is only allowed with B4")
    if args.profile != "B4" and args.r3_mode != "LEGACY":
        parser.error("EXACT_RESIDUAL --r3-mode is only allowed with B4")

    config = yaml.safe_load(CONFIG.read_text())
    profile = config["profiles"][args.profile]
    mode = MODE_FOR_PROFILE[args.profile]
    if profile["runner_mode"] != mode:
        raise RuntimeError("ablation config runner mode mismatch")
    if args.profile == "B4" and not profile["enable_vision"]:
        raise RuntimeError("B4 must enable metric visual fusion")
    default_input = FLOOR_INPUT if args.dataset == "Floor01" else CORRIDOR_INPUT
    default_map = (FLOOR_ROOT / "map/frozen/floor01_h1_map_p5_frozen.pcd"
                   if args.dataset == "Floor01" else
                   CORRIDOR_ROOT / "map/derived/corridor01_map_normalized.pcd")
    default_params = (default_input / "params.txt" if args.dataset == "Floor01" else
                      PACKAGE / "docs/p6_i6d_full_algorithm/corridor01_params_official_calibration.txt")
    input_dir = args.input_dir or default_input
    map_path = args.map or default_map
    params_path = args.params or default_params
    map_profile, init_stamp, expected_scans, manifest = validate_inputs(
        args.dataset, input_dir, map_path, params_path)
    frame_count = expected_scans if args.frame_limit is None else args.frame_limit
    if frame_count <= 0 or frame_count > expected_scans:
        raise RuntimeError("frame-limit outside pinned input frame count")
    if not args.executable.is_file():
        raise FileNotFoundError(args.executable)
    if args.profile == "B4":
        if args.visual_csv is None or not args.visual_csv.is_file():
            raise RuntimeError("B4 requires a prepared causal metric visual CSV")
        visual_path = args.visual_csv
    else:
        visual_path = None

    args.output.mkdir(parents=True, exist_ok=True)
    prefix = args.output / f"{args.profile}_{args.dataset}"
    outputs = [prefix.with_name(prefix.name + suffix) for suffix in
               ("_trajectory.csv", "_reliability.csv", "_runtime.csv")]
    if any(path.exists() for path in outputs):
        raise RuntimeError("refusing to overwrite existing I6D run output")
    trajectory, reliability, runtime = outputs
    command = [str(args.executable), mode,
               str(input_dir / "imu.csv"), str(input_dir / "filter_scans.csv"),
               str(input_dir / "scans.csv"), str(input_dir / "request_xyz_f32.bin"),
               str(map_path), str(params_path), str(trajectory), str(reliability),
               str(runtime)]
    if args.profile == "B4":
        command.extend((str(visual_path), str(frame_count), str(init_stamp), map_profile))
        if args.r2_policy != "LEGACY_BASE_NO_GATE" or args.r3_mode != "LEGACY":
            command.append(args.r2_policy)
        if args.r3_mode != "LEGACY":
            command.append(args.r3_mode)
    else:
        command.extend((str(frame_count), str(init_stamp), map_profile))
    env = os.environ.copy()
    env.pop("LD_LIBRARY_PATH", None)
    env.pop("LD_PRELOAD", None)
    compatibility_preload = os.environ.get("P6_I6D_COMPAT_PRELOAD")
    if compatibility_preload:
        preload_path = Path(compatibility_preload)
        if not preload_path.is_absolute() or not preload_path.is_file():
            raise RuntimeError("P6_I6D_COMPAT_PRELOAD must name an existing absolute file")
        env["LD_PRELOAD"] = str(preload_path)
    resource_path = prefix.with_name(prefix.name + "_resource.txt")
    provenance_path = prefix.with_name(prefix.name + "_provenance.txt")
    provenance_path.write_text(
        f"task={args.task_label}\nprofile={args.profile}\nmode={mode}\n"
        f"r2_policy={args.r2_policy}\n"
        f"r3_mode={args.r3_mode}\n"
        f"dataset={args.dataset}\nsource_manifest_sha256={sha256(input_dir / 'input_manifest.txt')}\n"
        f"map_sha256={sha256(map_path)}\nparams_sha256={sha256(params_path)}\n"
        f"visual_csv_sha256={sha256(visual_path) if visual_path else 'DISABLED'}\n"
        f"ablation_config_sha256={sha256(CONFIG)}\nmap_profile={map_profile}\n"
        f"frame_limit={frame_count}\ninitialization_stamp_ns={init_stamp}\n"
        "gt_used_online=NO\n"
        f"compatibility_preload={compatibility_preload or 'DISABLED'}\n"
        + "".join(f"input_{name}_sha256={sha256(input_dir / name)}\n"
                 for name in ("imu.csv", "filter_scans.csv", "scans.csv",
                              "request_xyz_f32.bin", "params.txt"))
        + "command=" + " ".join(command) + "\n",
        encoding="utf-8",
    )
    started = time.perf_counter()
    process = subprocess.run(["/usr/bin/time", "-v", "-o", str(resource_path), *command],
                             env=env, check=False)
    wall_s = time.perf_counter() - started
    if process.returncode != 0:
        provenance_path.write_text(provenance_path.read_text() +
                                   f"status=FAILED\nreturn_code={process.returncode}\n")
        return process.returncode
    with trajectory.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != frame_count:
        raise RuntimeError(f"trajectory has {len(rows)} rows, expected {frame_count}")
    if [int(row["transaction_id"]) for row in rows] != list(range(1, frame_count + 1)):
        raise RuntimeError("trajectory transaction IDs are not contiguous")
    counts = mode_counts(reliability,
                         prefix.with_name(prefix.name + "_reliability.csv.visual_updates.csv")
                         if args.profile == "B4" else None)
    full_replay = frame_count == expected_scans
    if args.profile == "B4" and full_replay and (
            counts["visual_filter_updates"] == 0 or
            counts["visual_nonzero_state_updates"] == 0):
        raise RuntimeError("FULL B4 did not perform a nonzero metric visual update")
    counts_text = "".join(f"{key}={value}\n" for key, value in counts.items())
    visual_gate = ("PASS" if counts["visual_filter_updates"] > 0 and
                   counts["visual_nonzero_state_updates"] > 0 else
                   "NOT_EXERCISED_IN_PARTIAL_SMOKE" if args.profile == "B4" and not full_replay
                   else "NOT_APPLICABLE")
    provenance_path.write_text(provenance_path.read_text() +
                               f"status={'COMPLETE' if full_replay else 'SMOKE_COMPLETE'}\n"
                               f"full_visual_update_gate={visual_gate}\n"
                               f"process_wall_s={wall_s:.6f}\n" +
                               counts_text)
    print(f"{args.task_label}_RUN_COMPLETE profile={args.profile} dataset={args.dataset} "
          f"frames={frame_count} wall_s={wall_s:.3f} counts={counts}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
