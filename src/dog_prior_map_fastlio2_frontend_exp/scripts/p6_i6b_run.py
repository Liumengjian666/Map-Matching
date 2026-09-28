#!/usr/bin/env python3
"""Run the four closed-loop P6-I6B Floor01 modes on frozen prepared inputs."""

from __future__ import annotations

import csv
import argparse
import hashlib
import os
import shutil
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
OUT = PACKAGE / "docs/p6_i6b_dual_reliability_v1"
V0_OUT = PACKAGE / "docs/p6_i6b_dual_reliability_v0"
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
EXPECTED_BASELINE_TRAJECTORY_SHA256 = (
    "fd9cb3ef78d25fb48361989bdf2f7b15b8f5e0fefa1e0f4fac0362b0911837da"
)


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


def run_logged(command, log_path: Path, resource_path: Path | None = None):
    env = os.environ.copy()
    env.pop("LD_LIBRARY_PATH", None)
    executable_command = [str(part) for part in command]
    if resource_path is not None:
        executable_command = ["/usr/bin/time", "-v", "-o", str(resource_path),
                              *executable_command]
    print("RUN", " ".join(str(part) for part in command), flush=True)
    with log_path.open("w") as log:
        process = subprocess.Popen(
            executable_command,
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


def read_csv_dict(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def ensure_vision_assist_diagnostics(output_dir: Path,
                                     modes: tuple[str, ...] = MODES) -> None:
    """Expose the disabled vision-assist contract in every per-frame table."""
    for mode in modes:
        for prefix in ("reliability", "runtime"):
            path = output_dir / f"{prefix}_{mode}.csv"
            rows = read_csv_dict(path)
            if not rows:
                raise RuntimeError(f"{path.name}: cannot finalize empty diagnostics")
            fields = list(rows[0])
            for field, default in (
                ("vision_assist_trigger_requested", "0"),
                ("vision_assist_trigger_reason", "NOT_REQUESTED"),
            ):
                if field not in fields:
                    fields.append(field)
                for row in rows:
                    row.setdefault(field, default)
            with path.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields,
                                        lineterminator="\n")
                writer.writeheader()
                writer.writerows(rows)


def write_resource_usage(output_dir: Path, modes: tuple[str, ...]) -> None:
    fields = ["mode", "wall_max_rss_kib", "user_cpu_s", "system_cpu_s"]
    rows = []
    for mode in modes:
        text = (output_dir / f"resource_{mode}.txt").read_text()
        values = {}
        for line in text.splitlines():
            if "Maximum resident set size" in line:
                values["wall_max_rss_kib"] = line.rsplit(":", 1)[1].strip()
            elif "User time (seconds)" in line:
                values["user_cpu_s"] = line.rsplit(":", 1)[1].strip()
            elif "System time (seconds)" in line:
                values["system_cpu_s"] = line.rsplit(":", 1)[1].strip()
        if len(values) != 3:
            raise RuntimeError(f"could not parse GNU time resource report for {mode}")
        rows.append({"mode": mode, **values})
    with (output_dir / "resource_usage.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def copy_frozen_baseline_anchor(output_dir: Path) -> str:
    source = V0_OUT / "baseline_anchor.csv"
    rows = read_csv_dict(source)
    if (len(rows) != 1 or
            rows[0].get("source_sha256") != EXPECTED_BASELINE_TRAJECTORY_SHA256 or
            rows[0].get("transaction_id") != "1"):
        raise RuntimeError("frozen V0 common-anchor identity mismatch")
    target = output_dir / "baseline_anchor.csv"
    shutil.copyfile(source, target)
    copied_sha = sha256(target)
    if copied_sha != sha256(source):
        raise RuntimeError("frozen V0 common-anchor copy changed bytes")
    return copied_sha


def validate_trajectory(path: Path, expected_rows: int = EXPECTED_SCANS) -> list[dict]:
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != expected_rows:
        raise RuntimeError(f"{path.name}: expected {expected_rows} rows, got {len(rows)}")
    if [int(row["transaction_id"]) for row in rows] != list(range(1, expected_rows + 1)):
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


def run_mode(mode: str, output_dir: Path, frame_limit: int | None = None):
    output_dir.mkdir(parents=True, exist_ok=True)
    trajectory = output_dir / f"trajectory_{mode}.csv"
    reliability = output_dir / f"reliability_{mode}.csv"
    runtime = output_dir / f"runtime_{mode}.csv"
    command = [
        EXECUTABLE, mode,
        INPUT / "imu.csv", INPUT / "filter_scans.csv", INPUT / "scans.csv",
        INPUT / "request_xyz_f32.bin", MAP, INPUT / "params.txt",
        trajectory, reliability, runtime,
    ]
    if frame_limit is not None:
        command.append(str(frame_limit))
    started = time.perf_counter()
    run_logged(command, output_dir / f"{mode}.log",
               output_dir / f"resource_{mode}.txt")
    wall_s = time.perf_counter() - started
    expected_rows = frame_limit if frame_limit is not None else EXPECTED_SCANS
    trajectory_rows = validate_trajectory(trajectory, expected_rows)
    with reliability.open(newline="") as stream:
        reliability_rows = list(csv.DictReader(stream))
    with runtime.open(newline="") as stream:
        runtime_rows = list(csv.DictReader(stream))
    if len(reliability_rows) != expected_rows or len(runtime_rows) != expected_rows:
        raise RuntimeError(f"{mode}: diagnostics/replay row count mismatch")
    for row in reliability_rows:
        if int(row["ndt_call_count"]) not in (1, 3):
            raise RuntimeError(f"{mode}: invalid NDT align count")
        calls = int(row["ndt_call_count"])
        if (row["probe_executed"] == "1") != (calls == 3):
            raise RuntimeError(f"{mode}: probe execution and NDT call count disagree")
        if row["probe_executed"] == "1" and row["probe_triggered"] != "1":
            raise RuntimeError(f"{mode}: probes executed without a recorded trigger")
        if (row.get("vision_assist_trigger_requested") != "0" or
                row.get("vision_assist_trigger_reason") != "NOT_REQUESTED"):
            raise RuntimeError(f"{mode}: unexpected vision-assist trigger state")
    return trajectory_rows, wall_s


def strict_replay_parity(replayed: list[dict], expected_rows: int = EXPECTED_SCANS) -> dict:
    from scipy.spatial.transform import Rotation
    frozen = validate_trajectory(FROZEN_I6A, EXPECTED_SCANS)[:expected_rows]
    if len(replayed) != expected_rows:
        raise RuntimeError(f"STRICT parity expected {expected_rows} rows, got {len(replayed)}")
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
    actual["frozen_v0_baseline_anchor_sha256"] = copy_frozen_baseline_anchor(OUT)
    actual.update(
        {
            "start_sha": git(REPO, "rev-parse", "HEAD"),
            "branch": git(REPO, "branch", "--show-current"),
            "map_recovery": "EXACT",
            "ndt_profile": "resolution=0.8,step=0.08,epsilon=1e-5,max_iterations=80",
            "probe_trigger": "normalized innovation chi-square > 16.812 or every 25 scans",
            "probe_prior_radius_sigma": "1.0",
            "uobs_score_gradient_gate": "six-coordinate central finite differences at h=1e-4 and 5e-5 on first two normal-coordinate nominal terminals; details per UOBS mode",
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

    smoke_dir = OUT / "smoke_100"
    smoke_wall_rows = []
    smoke_trajectories = {}
    for mode in MODES:
        rows, wall_s = run_mode(mode, smoke_dir, frame_limit=100)
        smoke_trajectories[mode] = rows
        smoke_wall_rows.append({"mode": mode, "process_wall_s": wall_s})
        for row in read_csv_dict(smoke_dir / f"reliability_{mode}.csv"):
            if row["M0_converged"] != "1":
                raise RuntimeError(f"Smoke {mode}: M0 did not converge at tx {row['transaction_id']}")
    smoke_parity = strict_replay_parity(smoke_trajectories["STRICT_BASELINE"], 100)
    first_stamps = [row["stamp_ns"] for row in smoke_trajectories["STRICT_BASELINE"]]
    for mode in MODES[1:]:
        if [row["stamp_ns"] for row in smoke_trajectories[mode]] != first_stamps:
            raise RuntimeError(f"Smoke {mode}: timestamps differ from STRICT_BASELINE")
    with (smoke_dir / "mode_wall_times.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["mode", "process_wall_s"],
                                lineterminator="\n")
        writer.writeheader()
        writer.writerows(smoke_wall_rows)
    write_resource_usage(smoke_dir, MODES)
    (smoke_dir / "smoke_validation.md").write_text(
        "# P6-I6B 100-frame Floor01 smoke\n\n"
        "- All four modes replayed 100 aligned scan/IMU events through the real closed-loop runner.\n"
        "- Each mode has finite normalized predictor/corrected poses, valid quaternions, increasing timestamps and matching event rows.\n"
        "- M0 converged on all 100 frames per mode; NDT calls and probe triggers are recorded per transaction.\n"
        f"- STRICT parity versus the frozen I6A trajectory prefix: max translation delta {smoke_parity['translation_max_delta_m']:.9g} m; max rotation delta {smoke_parity['rotation_max_delta_deg']:.9g} deg.\n"
        "- GT was not loaded or used for smoke validation.\n",
        encoding="utf-8",
    )

    wall_rows = []
    strict_rows, wall_s = run_mode("STRICT_BASELINE", OUT)
    wall_rows.append({"mode": "STRICT_BASELINE", "process_wall_s": wall_s})
    parity = strict_replay_parity(strict_rows)
    (OUT / "strict_replay_parity.txt").write_text(
        "STRICT_BASELINE_REPLAY_PARITY=PASS\n" +
        "\n".join(f"{key}={value:.17g}" for key, value in parity.items()) + "\n"
    )
    for mode in MODES[1:]:
        _, wall_s = run_mode(mode, OUT)
        wall_rows.append({"mode": mode, "process_wall_s": wall_s})
    write_resource_usage(OUT, MODES)
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


def rerun_modes(mode_names: list[str], output_dir: Path,
                frame_limit: int | None = None) -> None:
    """Replace selected full/smoke mode outputs after a narrowly scoped fix."""
    output_dir.mkdir(parents=True, exist_ok=True)
    wall_by_mode = {
        row["mode"]: float(row["process_wall_s"])
        for row in read_csv_dict(output_dir / "mode_wall_times.csv")
    } if (output_dir / "mode_wall_times.csv").is_file() else {}
    expected_rows = frame_limit if frame_limit is not None else EXPECTED_SCANS
    for mode in mode_names:
        rows, wall_s = run_mode(mode, output_dir, frame_limit)
        if len(rows) != expected_rows:
            raise RuntimeError(f"{mode}: selected rerun row count mismatch")
        wall_by_mode[mode] = wall_s
    write_resource_usage(output_dir, MODES)
    with (output_dir / "mode_wall_times.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["mode", "process_wall_s"],
                                lineterminator="\n")
        writer.writeheader()
        writer.writerows({"mode": mode, "process_wall_s": wall_by_mode[mode]}
                         for mode in MODES)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--rerun-modes", nargs="+", choices=MODES)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--frame-limit", type=int)
    parser.add_argument("--augment-diagnostics", action="store_true")
    args = parser.parse_args()
    if args.augment_diagnostics:
        if args.output_dir is None:
            raise SystemExit("--output-dir is required with --augment-diagnostics")
        ensure_vision_assist_diagnostics(args.output_dir)
    elif args.rerun_modes:
        if args.output_dir is None:
            raise SystemExit("--output-dir is required with --rerun-modes")
        rerun_modes(args.rerun_modes, args.output_dir, args.frame_limit)
    else:
        main()
