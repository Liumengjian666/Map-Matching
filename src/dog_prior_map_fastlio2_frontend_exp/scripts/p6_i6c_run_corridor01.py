#!/usr/bin/env python3
"""Run the I6C Corridor01 STRICT then UNONLOCAL closed-loop comparison."""

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
ROOT = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01")
FORMAL_OUT = ROOT / "results/p6_i6c_framework/formal"
SMOKE_OUT = ROOT / "results/p6_i6c_framework/smoke"
INPUT = ROOT / "results/p6_i6c_framework/input"
MAP = ROOT / "map/derived/corridor01_map_normalized.pcd"
GT = ROOT / "gt/corridor01_gt.txt"
RAW_BAG = ROOT / "raw/Long_Corridor_Rosbag/raw_data_core_2023-07-25-03-01-44.bag"
DERIVED_BAG = ROOT / "derived/corridor01_adapted_full_se3_v2.bag"
CALIB = ROOT / "calibration/corridor01_extrinsics.yaml"
INTRINSICS = ROOT / "calibration/corridor01_intrinsics.yaml"
INIT = Path("/home/jian/livox_ws/superloc_adapter_ws/config/corridor01_init.yaml")
BUILD = Path("/tmp/p6-i6c-build")
EXECUTABLE = BUILD / "p6_i6b_closed_loop"
FASTLIO2 = Path("/media/jian/HIKVISION/comparison algorithm/FAST_LIO2")
DCREG = Path("/home/jian/livox_ws/DCReg/DCReg")
EVAL_START_NS = 1_517_157_224_188_979_000
EXPECTED_INPUT_MANIFEST_SHA256 = "6d722ec6946570cc09d984c1a8ac7ebaaff799f012f9386ae169e3d47747a043"
EXPECTED_SCAN_COUNT = 2726
PINNED_SOURCE_HASHES = {
    "raw_bag_sha256": ("raw_bag", RAW_BAG,
                       "c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811"),
    "derived_bag_sha256": ("derived_bag", DERIVED_BAG,
                           "7c52b3703f2f5f9b7fe291e579187c67c0181e015df6a9a8398cf4795b547ba0"),
    "initialization_pose_sha256": ("initialization_pose_source", INIT,
                                   "d3e6f560895cb4f6a9efb7058bcff8a13313783e799d82c828e51adcd4bafd1e"),
    "extrinsics_sha256": ("extrinsics", CALIB,
                          "59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d"),
    "camera_intrinsics_sha256": ("camera_intrinsics", INTRINSICS,
                                 "083ff73553f6df25734bfddc439fbda7eb7b01c8baeede2b4949e960f72fd370"),
}
PINNED_BUNDLE_HASHES = {
    "imu_csv_sha256": "7dc881d4ebfeacea9354be569a5952e5e466ccdd366637e52a7797a6f37457aa",
    "filter_scans_csv_sha256": "f992626762f1f2321f78212c0b0c8be9e65b196bd1a638b4dc6c4fe51f846450",
    "scans_csv_sha256": "59179734b433a8d81d5c4ead554556f7d650f2865c87f7205a0e3f1e8c7286af",
    "request_xyz_f32_sha256": "beb700449a60a7f31c33e107b938862782d83afd7c93e589c323371db4be1610",
    "params_sha256": "fc7bb4c8758da222771aa0c9d18bb56e554a758a5bf181ea129ef9b86e3b4f48",
}
MODES = ("STRICT_BASELINE", "UNONLOCAL_ONLY")
PROFILE_BY_MODE = {"STRICT_BASELINE": "B0", "UNONLOCAL_ONLY": "B2"}
ABLATION_CONFIG = Path(__file__).resolve().parents[1] / "config/ablation.yaml"


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def read_manifest(path: Path) -> dict[str, str]:
    return dict(line.split("=", 1) for line in path.read_text().splitlines() if "=" in line)


def run_logged(command: list[str], log_path: Path, env: dict[str, str]) -> float:
    print("RUN", " ".join(command), flush=True)
    start = time.perf_counter()
    with log_path.open("w", encoding="utf-8") as log:
        process = subprocess.Popen(command, env=env, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True, bufsize=1)
        assert process.stdout is not None
        for line in process.stdout:
            print(line, end="", flush=True)
            log.write(line)
        code = process.wait()
    if code != 0:
        raise RuntimeError(f"command failed ({code}); see {log_path}")
    return time.perf_counter() - start


def validate_trajectory(path: Path, expected: int) -> list[dict[str, str]]:
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != expected:
        raise RuntimeError(f"{path.name}: expected {expected} rows, got {len(rows)}")
    if [int(row["transaction_id"]) for row in rows] != list(range(1, expected + 1)):
        raise RuntimeError(f"{path.name}: transaction IDs are not contiguous")
    stamps = [int(row["stamp_ns"]) for row in rows]
    if any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise RuntimeError(f"{path.name}: timestamps are not strictly increasing")
    for row in rows:
        for prefix in ("predictor_imu", "corrected_imu"):
            for suffix in ("tx", "ty", "tz", "qx", "qy", "qz", "qw"):
                if not float(row[f"{prefix}_{suffix}"]).__abs__() < float("inf"):
                    raise RuntimeError(f"{path.name}: nonfinite {prefix}_{suffix}")
    return rows


def validate_mode_outputs(mode: str, out_dir: Path, expected: int) -> int:
    trajectory = validate_trajectory(out_dir / f"trajectory_{mode}.csv", expected)
    with (out_dir / f"reliability_{mode}.csv").open(newline="") as stream:
        reliability_rows = list(csv.DictReader(stream))
    with (out_dir / f"runtime_{mode}.csv").open(newline="") as stream:
        runtime_rows = list(csv.DictReader(stream))
    if len(reliability_rows) != len(trajectory) or len(runtime_rows) != len(trajectory):
        raise RuntimeError(f"{mode}: trajectory/reliability/runtime row counts differ")
    failed = sum(row["M0_converged"] != "1" for row in reliability_rows)
    if mode == "STRICT_BASELINE" and failed:
        raise RuntimeError(f"STRICT_BASELINE emitted {failed} nonconverged transactions")
    with (out_dir / f"mode_validation_{mode}.txt").open("w") as stream:
        stream.write(f"trajectory_rows={len(trajectory)}\n")
        stream.write(f"reliability_rows={len(reliability_rows)}\n")
        stream.write(f"runtime_rows={len(runtime_rows)}\n")
        stream.write(f"ndt_nonconverged={failed}\n")
        stream.write(f"prediction_only={sum(row['decision'] == 'PREDICTION_ONLY' for row in reliability_rows)}\n")
    return failed


def verify_bundle() -> dict[str, str]:
    manifest_path = INPUT / "input_manifest.txt"
    if sha256(manifest_path) != EXPECTED_INPUT_MANIFEST_SHA256:
        raise RuntimeError("prepared Corridor01 manifest differs from the frozen input identity")
    manifest = read_manifest(manifest_path)
    expected = {
        "imu_csv_sha256": INPUT / "imu.csv",
        "filter_scans_csv_sha256": INPUT / "filter_scans.csv",
        "scans_csv_sha256": INPUT / "scans.csv",
        "request_xyz_f32_sha256": INPUT / "request_xyz_f32.bin",
        "params_sha256": INPUT / "params.txt",
    }
    for key, path in expected.items():
        actual = sha256(path)
        if manifest.get(key) != actual:
            raise RuntimeError(f"prepared input SHA mismatch for {key}")
    if manifest.get("scan_count_replayed") != str(EXPECTED_SCAN_COUNT):
        raise RuntimeError("Corridor01 scan count differs from the frozen exporter contract")
    if manifest.get("evaluation_start_header_stamp_ns") != str(EVAL_START_NS):
        raise RuntimeError("Corridor01 evaluation/init epoch differs from the frozen P2B contract")
    if manifest.get("cloud_source_index_start_inclusive") != "50" or \
       manifest.get("initialization_clouds_excluded") != "50" or \
       manifest.get("cloud_count_total") != "2776":
        raise RuntimeError("Corridor01 first-50-cloud initialization contract mismatch")
    if manifest.get("map_sha256") != "103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f":
        raise RuntimeError("Corridor01 map SHA differs from the recorded normalized map")
    if sha256(MAP) != manifest["map_sha256"]:
        raise RuntimeError("current Corridor01 map differs from the prepared manifest")
    if manifest.get("gt_used_online") != "NO" or manifest.get("initialization_pose_gt_used") != "NO":
        raise RuntimeError("GT-free replay initialization contract is violated")
    if manifest.get("cloud_topic") != "/superloc_adapter/points_rot_only":
        raise RuntimeError("input is not the rotational-only, no-second-deskew cloud stream")
    for key, (path_key, expected_path, expected_sha) in PINNED_SOURCE_HASHES.items():
        source = Path(manifest[path_key])
        if source != expected_path or manifest.get(key) != expected_sha or \
           not source.is_file() or sha256(source) != expected_sha:
            raise RuntimeError(f"frozen source identity mismatch: {path_key}")
    for key, expected_sha in PINNED_BUNDLE_HASHES.items():
        if manifest.get(key) != expected_sha or sha256(INPUT / {
                "imu_csv_sha256": "imu.csv",
                "filter_scans_csv_sha256": "filter_scans.csv",
                "scans_csv_sha256": "scans.csv",
                "request_xyz_f32_sha256": "request_xyz_f32.bin",
                "params_sha256": "params.txt",
            }[key]) != expected_sha:
            raise RuntimeError(f"prepared bundle differs from pinned bytes: {key}")

    with (INPUT / "filter_scans.csv").open(newline="") as stream:
        filter_rows = list(csv.DictReader(stream))
    with (INPUT / "scans.csv").open(newline="") as stream:
        scan_rows = list(csv.DictReader(stream))
    if len(filter_rows) != EXPECTED_SCAN_COUNT or len(scan_rows) != EXPECTED_SCAN_COUNT:
        raise RuntimeError("prepared scan tables do not match the manifest-bound row count")
    expected_ids = list(range(1, EXPECTED_SCAN_COUNT + 1))
    if [int(row["transaction_id"]) for row in filter_rows] != expected_ids or \
       [int(row["transaction_id"]) for row in scan_rows] != expected_ids:
        raise RuntimeError("prepared scan transaction IDs are not contiguous and aligned")
    previous_stamp = 0
    expected_offset = 0
    for filter_row, scan_row in zip(filter_rows, scan_rows):
        stamp = int(scan_row["stamp_ns"])
        if int(filter_row["stamp_ns"]) != stamp or stamp <= previous_stamp:
            raise RuntimeError("filter/cloud scan timestamps are misaligned or unordered")
        if int(scan_row["cloud_byte_offset"]) != expected_offset:
            raise RuntimeError("packed cloud offsets contain a gap or overlap")
        count = int(scan_row["cloud_point_count"])
        if count <= 0:
            raise RuntimeError("prepared source cloud is empty")
        if scan_row.get("ndt_source_cloud_hash_available") != "0" or \
           scan_row.get("request_cloud_hash_available") != "0":
            raise RuntimeError("Corridor01 adapter does not provide frozen request/source hashes")
        expected_offset += count * 3 * 4
        previous_stamp = stamp
    if (INPUT / "request_xyz_f32.bin").stat().st_size != expected_offset:
        raise RuntimeError("packed cloud binary size does not match the complete scan table")

    with (INPUT / "imu.csv").open(newline="") as stream:
        imu_rows = list(csv.DictReader(stream))
    imu_stamps = [int(row["stamp_ns"]) for row in imu_rows]
    if len(imu_rows) != int(manifest["imu_count"]) or any(
            b <= a for a, b in zip(imu_stamps, imu_stamps[1:])):
        raise RuntimeError("raw IMU table count/order differs from its manifest")
    init_stamp = int(manifest["evaluation_start_header_stamp_ns"])
    init_index = next((i for i, stamp in enumerate(imu_stamps) if stamp >= init_stamp),
                      len(imu_stamps))
    causal_end = init_index + (1 if init_index < len(imu_stamps) and
                               imu_stamps[init_index] == init_stamp else 0)
    if causal_end < 200 or imu_stamps[causal_end - 1] > init_stamp:
        raise RuntimeError("fixed initialization epoch lacks 200 causal IMU samples")
    causal = imu_stamps[causal_end - 200:causal_end]
    periods = sorted(b-a for a, b in zip(causal[-101:-1], causal[-100:]))
    if not periods or init_stamp - causal[-1] > 2 * periods[len(periods)//2]:
        raise RuntimeError("fixed initialization epoch requires excessive IMU look-ahead/extrapolation")
    if imu_stamps[0] > init_stamp or imu_stamps[-1] < int(scan_rows[-1]["stamp_ns"]):
        raise RuntimeError("IMU header-time coverage does not span initialization through final scan")
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--smoke-frames", type=int, default=0,
                        help="optional short smoke only; zero runs the formal full replay")
    parser.add_argument("--report-only", action="store_true",
                        help="skip closed-loop modes and posthoc-evaluate existing formal output")
    parser.add_argument("--output", type=Path,
                        help="optional output directory; smoke defaults to a separate smoke directory")
    args = parser.parse_args()
    manifest = verify_bundle()
    ablation = yaml.safe_load(ABLATION_CONFIG.read_text())
    for mode in MODES:
        profile = ablation["profiles"][PROFILE_BY_MODE[mode]]
        if profile["runner_mode"] != mode or profile["availability"] != "AVAILABLE":
            raise RuntimeError(f"ablation profile {PROFILE_BY_MODE[mode]} is not runnable")
    out_dir = args.output or (SMOKE_OUT if args.smoke_frames else FORMAL_OUT)
    out_dir.mkdir(parents=True, exist_ok=True)
    if not args.report_only and not args.smoke_frames and any(
            (out_dir / f"trajectory_{mode}.csv").exists() for mode in MODES):
        raise RuntimeError("formal outputs already exist; refusing to overwrite them")

    env = os.environ.copy()
    # Avoid the host's Hikvision MVS libusb shadowing the system libusb needed by PCL IO.
    env.pop("LD_LIBRARY_PATH", None)
    env.pop("LD_PRELOAD", None)
    provenance = [
        "PAPER-P6-I6C Corridor01 closed-loop run",
        f"branch={subprocess.check_output(['git','-C',str(REPO),'branch','--show-current'],text=True).strip()}",
        f"code_sha={subprocess.check_output(['git','-C',str(REPO),'rev-parse','HEAD'],text=True).strip()}",
        f"derived_bag_sha256={manifest['derived_bag_sha256']}",
        f"input_manifest_sha256={sha256(INPUT / 'input_manifest.txt')}",
        f"map_sha256={manifest['map_sha256']}",
        f"evaluation_start_stamp_ns={EVAL_START_NS}",
        "ndt=resolution:0.8,step_size:0.08,epsilon:1e-5,max_iterations:80",
        f"ablation_config_sha256={sha256(ABLATION_CONFIG)}",
        "ablation_profiles=B0:STRICT_BASELINE,B2:UNONLOCAL_ONLY; same shared core",
        "initialization=GT-free first-segment pose; static IMU window ends at evaluation timestamp; existing gates unchanged",
        "source_hash_check=per-cloud NDT/request hashes unavailable; packed binary SHA256 and derived bag SHA256 verified; NDT source hash status is emitted per transaction",
        "GT_read_after_all_requested_closed_loop_trajectories_validated=YES",
    ]
    (out_dir / "execution_provenance.txt").write_text("\n".join(provenance) + "\n")

    if not args.report_only:
        subprocess.run(["cmake", "--build", str(BUILD), "--target",
                        "p6_i6b_closed_loop", "--parallel", "2"],
                       cwd=REPO, env=env, check=True)
        if not EXECUTABLE.is_file():
            raise RuntimeError("shared I6B closed-loop executable is missing")
        for mode in MODES:
            trajectory = out_dir / f"trajectory_{mode}.csv"
            reliability = out_dir / f"reliability_{mode}.csv"
            runtime = out_dir / f"runtime_{mode}.csv"
            command = ["/usr/bin/time", "-v", "-o", str(out_dir / f"resource_{mode}.txt"),
                       str(EXECUTABLE), mode,
                       str(INPUT / "imu.csv"), str(INPUT / "filter_scans.csv"),
                       str(INPUT / "scans.csv"), str(INPUT / "request_xyz_f32.bin"),
                       str(MAP), str(INPUT / "params.txt"), str(trajectory),
                       str(reliability), str(runtime)]
            command.extend([str(args.smoke_frames or EXPECTED_SCAN_COUNT), str(EVAL_START_NS)])
            command.append("corridor01")
            wall_s = run_logged(command, out_dir / f"{mode}.log", env)
            rows = validate_trajectory(trajectory, args.smoke_frames or EXPECTED_SCAN_COUNT)
            validate_mode_outputs(mode, out_dir, args.smoke_frames or EXPECTED_SCAN_COUNT)
            with (out_dir / f"wall_{mode}.txt").open("w") as stream:
                stream.write(f"process_wall_s={wall_s:.9f}\n")
            print(f"VALIDATED mode={mode} scans={len(rows)} wall_s={wall_s:.3f}", flush=True)

    if not args.smoke_frames:
        for mode in MODES:
            validate_mode_outputs(mode, out_dir, EXPECTED_SCAN_COUNT)
        report_command = ["python3", str(PACKAGE / "scripts/p6_i6c_report_corridor01.py"),
                          "--out", str(out_dir), "--gt", str(GT),
                          "--start", str(EVAL_START_NS * 1e-9)]
        run_logged(report_command, out_dir / "posthoc_evaluation.log", env)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
