#!/usr/bin/env python3
"""Run the four closed-loop P6-I6B Floor01 modes on frozen prepared inputs."""

from __future__ import annotations

import csv
import hashlib
import os
import subprocess
import time
from pathlib import Path


REPO = Path(__file__).resolve().parents[3]
PACKAGE = REPO / "src/dog_prior_map_fastlio2_frontend_exp"
INPUT = Path("/home/jian/livox_ws/p6_i1_recovery_assets_20260927/input")
ASSETS = INPUT.parent
MAP = ASSETS / "floor01_h1_map_p5_frozen.pcd"
EXTERNAL_MAP = Path(
    "/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/map/frozen/"
    "floor01_h1_map_p5_frozen.pcd"
)
OUT = PACKAGE / "docs/p6_i6b_dual_reliability_v0"
BUILD = Path("/tmp/p6-i6b-build")
EXECUTABLE = BUILD / "p6_i6b_closed_loop"
FASTLIO2 = Path("/media/jian/HIKVISION/comparison algorithm/FAST_LIO2")
DCREG = Path("/home/jian/livox_ws/DCReg/DCReg")
FROZEN_I6A = PACKAGE / "docs/p6_i6a_convergence_control/trajectory_STRICT.csv"
GT = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/gt/floor01_gt.txt")
EXTRINSICS = Path(
    "/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/calibration/"
    "floor01_extrinsics.yaml"
)
BAG = Path(
    "/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/"
    "p3_r10b_fix1_floor01_full_rerun_20260926/floor01_fix1_runtime_topics.bag"
)
CONFIG = BAG.with_name("floor01_superloc_smoke.yaml")

EXPECTED = {
    "map_sha256": "2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570",
    "bag_sha256": "860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db",
    "config_sha256": "4e9584a4c1d5c2ada963700892880cdf2a7f4e75e43f0ff258b5fd4272af7d77",
    "extrinsics_sha256": "fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414",
    "gt_sha256": "b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f",
    "scans_sha256": "9371e593c0e625611f053e3ef0a581c52481ccaed392aa8d74d74313caa9938f",
    "request_xyz_sha256": "f7b5262552fe8d52f383e50813de2b568fa8a5990c69e2bba55231df8547188f",
    "i6a_strict_trajectory_sha256": "3f467b70345a8723fde8a61d260153d1cf5726c1bd574984940b89835e66a3b2",
    "dcreg_commit": "ce7db8220f549a4a4391729e3bf4de4d4ab74635",
    "fastlio2_commit": "7cc4175de6f8ba2edf34bab02a42195b141027e9",
}
MODES = ("STRICT_BASELINE", "UOBS_ONLY", "UNONLOCAL_ONLY", "DUAL_RELIABILITY")
EXPECTED_SCANS = 4127


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git(cwd: Path, *args: str) -> str:
    return subprocess.check_output(["git", "-C", str(cwd), *args], text=True).strip()


def require_sha(name: str, path: Path, expected: str) -> str:
    actual = sha256(path)
    if actual != expected:
        raise RuntimeError(f"{name} SHA mismatch: expected={expected} actual={actual}")
    return actual


def verify_frozen_inputs() -> dict:
    manifest_path = INPUT / "input_manifest.txt"
    manifest = dict(
        line.split("=", 1)
        for line in manifest_path.read_text().splitlines()
        if "=" in line
    )
    if int(manifest.get("imu_count", "0")) != 83342 or int(
        manifest.get("scan_count", "0")
    ) != EXPECTED_SCANS:
        raise RuntimeError("prepared frozen input counts do not match I6A")
    actual = {
        "map_sha256": require_sha("internal map", MAP, EXPECTED["map_sha256"]),
        "external_map_sha256": require_sha(
            "external frozen map", EXTERNAL_MAP, EXPECTED["map_sha256"]
        ),
        "bag_sha256": require_sha("source bag", BAG, EXPECTED["bag_sha256"]),
        "config_sha256": require_sha("source config", CONFIG, EXPECTED["config_sha256"]),
        "extrinsics_sha256": require_sha(
            "official extrinsics", EXTRINSICS, EXPECTED["extrinsics_sha256"]
        ),
        "scans_sha256": require_sha(
            "prepared scans", INPUT / "scans.csv", EXPECTED["scans_sha256"]
        ),
        "request_xyz_sha256": require_sha(
            "prepared point cloud bytes", INPUT / "request_xyz_f32.bin",
            EXPECTED["request_xyz_sha256"],
        ),
        "i6a_strict_trajectory_sha256": require_sha(
            "frozen I6A STRICT trajectory", FROZEN_I6A,
            EXPECTED["i6a_strict_trajectory_sha256"],
        ),
        "imu_sha256": sha256(INPUT / "imu.csv"),
        "filter_scans_sha256": sha256(INPUT / "filter_scans.csv"),
        "params_sha256": sha256(INPUT / "params.txt"),
        "input_manifest_sha256": sha256(manifest_path),
        "gt_expected_sha256": EXPECTED["gt_sha256"],
        "fastlio2_commit": git(FASTLIO2, "rev-parse", "HEAD"),
        "dcreg_commit": git(DCREG, "rev-parse", "HEAD"),
    }
    if actual["fastlio2_commit"] != EXPECTED["fastlio2_commit"]:
        raise RuntimeError("pinned FAST-LIO2 checkout changed")
    if actual["dcreg_commit"] != EXPECTED["dcreg_commit"]:
        raise RuntimeError("pinned DCReg checkout changed")
    for name, root in (("FAST-LIO2", FASTLIO2), ("DCReg", DCREG)):
        if git(root, "status", "--short"):
            raise RuntimeError(f"pinned {name} source tree is dirty")
    return actual


def run_logged(command, log_path: Path):
    env = os.environ.copy()
    env.pop("LD_LIBRARY_PATH", None)
    print("RUN", " ".join(str(part) for part in command), flush=True)
    with log_path.open("w") as log:
        process = subprocess.Popen(
            [str(part) for part in command],
            cwd=REPO,
            env=env,
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
        code = process.wait()
    if code:
        raise RuntimeError(f"command failed exit={code}; see {log_path}")


def validate_trajectory(path: Path) -> list[dict]:
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != EXPECTED_SCANS:
        raise RuntimeError(f"{path.name}: expected {EXPECTED_SCANS} rows, got {len(rows)}")
    if [int(row["transaction_id"]) for row in rows] != list(range(1, EXPECTED_SCANS + 1)):
        raise RuntimeError(f"{path.name}: invalid transaction sequence")
    stamps = [int(row["stamp_ns"]) for row in rows]
    if any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise RuntimeError(f"{path.name}: non-increasing timestamps")
    for row in rows:
        for prefix in ("predictor_imu", "corrected_imu"):
            for suffix in ("tx", "ty", "tz", "qx", "qy", "qz", "qw"):
                value = float(row[f"{prefix}_{suffix}"])
                if not (-float("inf") < value < float("inf")):
                    raise RuntimeError(f"{path.name}: nonfinite {prefix}_{suffix}")
            qnorm = sum(float(row[f"{prefix}_q{axis}"]) ** 2 for axis in "xyzw") ** 0.5
            if abs(qnorm - 1.0) > 1e-5:
                raise RuntimeError(f"{path.name}: invalid quaternion norm {qnorm}")
    return rows


def run_mode(mode: str):
    trajectory = OUT / f"trajectory_{mode}.csv"
    reliability = OUT / f"reliability_{mode}.csv"
    runtime = OUT / f"runtime_{mode}.csv"
    command = [
        EXECUTABLE, mode,
        INPUT / "imu.csv", INPUT / "filter_scans.csv", INPUT / "scans.csv",
        INPUT / "request_xyz_f32.bin", MAP, INPUT / "params.txt",
        trajectory, reliability, runtime,
    ]
    started = time.perf_counter()
    run_logged(command, OUT / f"{mode}.log")
    wall_s = time.perf_counter() - started
    trajectory_rows = validate_trajectory(trajectory)
    with reliability.open(newline="") as stream:
        reliability_rows = list(csv.DictReader(stream))
    with runtime.open(newline="") as stream:
        runtime_rows = list(csv.DictReader(stream))
    if len(reliability_rows) != EXPECTED_SCANS or len(runtime_rows) != EXPECTED_SCANS:
        raise RuntimeError(f"{mode}: diagnostics/replay row count mismatch")
    for row in reliability_rows:
        if int(row["ndt_call_count"]) not in (1, 3):
            raise RuntimeError(f"{mode}: invalid NDT align count")
        calls = int(row["ndt_call_count"])
        if (row["probe_executed"] == "1") != (calls == 3):
            raise RuntimeError(f"{mode}: probe execution and NDT call count disagree")
        if row["probe_executed"] == "1" and row["probe_triggered"] != "1":
            raise RuntimeError(f"{mode}: probes executed without a recorded trigger")
    return trajectory_rows, wall_s


def strict_replay_parity(replayed: list[dict]) -> dict:
    from scipy.spatial.transform import Rotation
    frozen = validate_trajectory(FROZEN_I6A)
    max_t = 0.0
    max_r = 0.0
    for current, reference in zip(replayed, frozen):
        if current["stamp_ns"] != reference["stamp_ns"]:
            raise RuntimeError("STRICT_BASELINE timestamps differ from I6A frozen replay")
        for prefix in ("predictor_imu", "corrected_imu"):
            p = [float(current[f"{prefix}_t{x}"]) for x in "xyz"]
            q = [float(current[f"{prefix}_q{x}"]) for x in "xyzw"]
            rp = [float(reference[f"{prefix}_t{x}"]) for x in "xyz"]
            rq = [float(reference[f"{prefix}_q{x}"]) for x in "xyzw"]
            max_t = max(max_t, float(sum((a - b) ** 2 for a, b in zip(p, rp)) ** 0.5))
            max_r = max(max_r, float((Rotation.from_quat(rq).inv() *
                                      Rotation.from_quat(q)).magnitude() * 180.0 / 3.141592653589793))
    if not (max_t < 0.005 and max_r < 0.05):
        raise RuntimeError(f"STRICT replay parity gate failed: t={max_t} r_deg={max_r}")
    return {"translation_max_delta_m": max_t, "rotation_max_delta_deg": max_r}


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    actual = verify_frozen_inputs()
    actual.update(
        {
            "start_sha": git(REPO, "rev-parse", "HEAD"),
            "branch": git(REPO, "branch", "--show-current"),
            "map_recovery": "EXACT",
            "ndt_profile": "resolution=0.8,step=0.08,epsilon=1e-5,max_iterations=80",
            "probe_trigger": "normalized innovation chi-square > 16.812 or every 25 scans",
            "probe_prior_radius_sigma": "1.0",
            "uobs_score_gradient_gate": "INDETERMINATE; runtime U_obs.valid=false",
            "gt_used_during_replay": "NO",
        }
    )
    (OUT / "input_provenance.txt").write_text(
        "P6-I6B frozen Floor01 input provenance\n" +
        "\n".join(f"{key}={value}" for key, value in actual.items()) + "\n"
    )
    run_logged(
        ["cmake", "-S", PACKAGE / "scripts/p6_i6b", "-B", BUILD,
         "-DCMAKE_BUILD_TYPE=Release", "-DBUILD_I6B_CLOSED_LOOP_TOOLS=ON",
         f"-DFASTLIO2_ROOT={FASTLIO2}", f"-DDCREG_ROOT={DCREG}"],
        OUT / "cmake_configure.log",
    )
    run_logged(["cmake", "--build", BUILD, "--target", "p6_i6b_closed_loop", "--parallel", "2"],
               OUT / "build.log")
    if not EXECUTABLE.is_file():
        raise RuntimeError("I6B closed-loop executable missing")

    wall_rows = []
    strict_rows, wall_s = run_mode("STRICT_BASELINE")
    wall_rows.append({"mode": "STRICT_BASELINE", "process_wall_s": wall_s})
    parity = strict_replay_parity(strict_rows)
    (OUT / "strict_replay_parity.txt").write_text(
        "STRICT_BASELINE_REPLAY_PARITY=PASS\n" +
        "\n".join(f"{key}={value:.17g}" for key, value in parity.items()) + "\n"
    )
    for mode in MODES[1:]:
        _, wall_s = run_mode(mode)
        wall_rows.append({"mode": mode, "process_wall_s": wall_s})
    with (OUT / "mode_wall_times.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["mode", "process_wall_s"],
                                lineterminator="\n")
        writer.writeheader()
        writer.writerows(wall_rows)

    # GT is intentionally checked/opened only after every closed-loop mode is
    # complete and structurally validated.
    run_logged(
        ["python3", PACKAGE / "scripts/p6_i6b_report.py", "--output-dir", OUT],
        OUT / "posthoc_evaluation.log",
    )
    print("PAPER_P6_I6B_ALL_CLOSED_LOOP_MODES_COMPLETE", flush=True)


if __name__ == "__main__":
    main()
