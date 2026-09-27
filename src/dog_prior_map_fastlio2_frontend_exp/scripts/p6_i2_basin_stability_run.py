#!/usr/bin/env python3
"""Run the bounded P6-I2 offline NDT basin-stability experiment."""

import csv
import hashlib
import os
import subprocess
import sys
import time
from pathlib import Path


WORKSPACE = Path("/home/jian/livox_ws/dog_loc_paper_ws")
PACKAGE = WORKSPACE / "src/dog_prior_map_fastlio2_frontend_exp"
P6I1 = PACKAGE / "docs/p6_i1_branched_recovery"
OUT = PACKAGE / "docs/p6_i2_basin_stability_probe"
ASSETS = Path("/home/jian/livox_ws/p6_i1_recovery_assets_20260927")
INPUT = ASSETS / "input"
BUILD = ASSETS / "build"
EXECUTABLE = BUILD / "p6_i1_branched_recovery"
MAP = ASSETS / "floor01_h1_map_p5_frozen.pcd"
EXTERNAL_MAP = Path(
    "/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/map/frozen/"
    "floor01_h1_map_p5_frozen.pcd"
)
EXPECTED_HEAD = "9ece6bc50b24b9bc8d619fc2ac84fdb74616480c"
EXPECTED_MAP_SHA = "2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570"
EXPECTED_BAG_SHA = "860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db"
EXPECTED_CONFIG_SHA = "4e9584a4c1d5c2ada963700892880cdf2a7f4e75e43f0ff258b5fd4272af7d77"
EXPECTED_SCANS = 4127
EXPECTED_IMU = 83342
P6_CPP = PACKAGE / "scripts/p6_i1_branched_recovery.cpp"


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git(*args):
    return subprocess.check_output(["git", "-C", str(WORKSPACE), *args], text=True).strip()


def rows(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def verify_frozen_inputs():
    branch = git("branch", "--show-current")
    head = git("rev-parse", "HEAD")
    remote = subprocess.check_output(
        ["git", "-C", str(WORKSPACE), "ls-remote", "origin", "refs/heads/paper"],
        text=True,
    ).strip()
    if branch != "paper" or head != EXPECTED_HEAD:
        raise RuntimeError(f"workspace gate failed: branch={branch}, HEAD={head}")
    if not remote or remote.split()[0] != EXPECTED_HEAD:
        raise RuntimeError(f"origin/paper gate failed: {remote}")
    if sha256(MAP) != EXPECTED_MAP_SHA or sha256(EXTERNAL_MAP) != EXPECTED_MAP_SHA:
        raise RuntimeError("exact frozen-map SHA gate failed")
    provenance = EXTERNAL_MAP.with_suffix(".provenance.txt")
    if not provenance.is_file():
        raise RuntimeError("persistent frozen-map provenance missing")
    manifest = dict(
        line.split("=", 1)
        for line in (INPUT / "input_manifest.txt").read_text().splitlines()
        if "=" in line
    )
    if manifest.get("map_sha256") != EXPECTED_MAP_SHA:
        raise RuntimeError("prepared replay map SHA mismatch")
    if manifest.get("bag_sha256") != EXPECTED_BAG_SHA:
        raise RuntimeError("prepared replay bag SHA mismatch")
    if manifest.get("config_sha256") != EXPECTED_CONFIG_SHA:
        raise RuntimeError("prepared Floor01 config SHA mismatch")
    if (int(manifest.get("imu_count", 0)) != EXPECTED_IMU or
            int(manifest.get("scan_count", 0)) != EXPECTED_SCANS):
        raise RuntimeError("prepared replay input-count mismatch")
    if MAP.stat().st_size != EXTERNAL_MAP.stat().st_size:
        raise RuntimeError("internal and persistent frozen-map byte counts differ")
    if not (INPUT / "request_xyz_f32.bin").is_file():
        raise RuntimeError("prepared request-cloud binary missing")
    if not (P6I1 / "trajectory_BASELINE.csv").is_file():
        raise RuntimeError("P6-I1 frozen baseline trajectory missing")
    if not (P6I1 / "trajectory_MULTISTART_OBJECTIVE.csv").is_file():
        raise RuntimeError("P6-I1 objective-reference trajectory missing")
    if OUT.exists():
        raise RuntimeError(f"refusing to overwrite an existing result directory: {OUT}")
    OUT.mkdir(parents=True)
    print(f"P6_I2_GATES_PASS HEAD={head} REMOTE={remote}", flush=True)
    print(f"FROZEN_MAP_SHA256={EXPECTED_MAP_SHA} MAP_RECOVERY=EXACT", flush=True)


def audit_seed7():
    candidates = rows(P6I1 / "candidate_modes_MULTISTART_OBJECTIVE.csv")
    events = rows(P6I1 / "multistart_events_MULTISTART_OBJECTIVE.csv")
    branches = rows(P6I1 / "branch_events_MULTISTART_OBJECTIVE.csv")
    arbitration = rows(P6I1 / "visual_arbitration_MULTISTART_OBJECTIVE.csv")
    trajectory = rows(P6I1 / "trajectory_MULTISTART_OBJECTIVE.csv")
    seed_names = {
        0: "S0_PREDICTED", 1: "S1_PLUS_X", 2: "S2_MINUS_X",
        3: "S3_PLUS_Y", 4: "S4_MINUS_Y", 5: "S5_PLUS_YAW",
        6: "S6_MINUS_YAW", 7: "S7_VISUAL_MOTION",
    }
    winner_counts = {index: 0 for index in seed_names}
    candidate_counts = {index: 0 for index in seed_names}
    converged_counts = {index: 0 for index in seed_names}
    for row in candidates:
        seed = int(row["seed_index"])
        candidate_counts[seed] += 1
        converged_counts[seed] += int(row["converged"])
        winner_counts[seed] += int(row["selected"])
    event_s7 = sum(int(row["selected_seed_index"]) == 7 for row in events)
    branch_s7 = sum(int(row["selected_seed_index"]) == 7 for row in branches)
    if (len(events) != 1802 or len(branches) != EXPECTED_SCANS or
            len(trajectory) != EXPECTED_SCANS or len(arbitration) != 0):
        raise RuntimeError("P6-I1 seed-audit lineage/count gate failed")
    if any(candidate_counts[index] != 1802 for index in seed_names):
        raise RuntimeError(f"incomplete P6-I1 candidate seed rows: {candidate_counts}")
    if winner_counts[7] != event_s7 or event_s7 != branch_s7:
        raise RuntimeError("S7 winner count disagrees across P6-I1 logs")
    with (OUT / "seed7_audit.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=("seed_index", "seed_name", "candidate_frames",
                        "converged_frames", "selected_objective_wins",
                        "multistart_event_s7_wins", "branch_log_s7_wins",
                        "visual_arbitration_rows", "trajectory_rows"),
            lineterminator="\n",
        )
        writer.writeheader()
        for seed, name in seed_names.items():
            writer.writerow({
                "seed_index": seed,
                "seed_name": name,
                "candidate_frames": candidate_counts[seed],
                "converged_frames": converged_counts[seed],
                "selected_objective_wins": winner_counts[seed],
                "multistart_event_s7_wins": event_s7 if seed == 7 else 0,
                "branch_log_s7_wins": branch_s7 if seed == 7 else 0,
                "visual_arbitration_rows": len(arbitration),
                "trajectory_rows": len(trajectory),
            })
    print(f"P6_I1_S7_AUDIT_PASS selected_as_objective_winner={winner_counts[7]}", flush=True)
    return winner_counts[7]


def rss_kib(pid):
    try:
        for line in Path(f"/proc/{pid}/status").read_text().splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1])
    except (FileNotFoundError, ProcessLookupError, PermissionError):
        return 0
    return 0


def run_logged(name, command, log_path):
    environment = os.environ.copy()
    environment.pop("LD_LIBRARY_PATH", None)
    started = time.monotonic()
    peak = 0
    print(f"RUN {name}: {' '.join(str(item) for item in command)}", flush=True)
    with log_path.open("w") as log:
        process = subprocess.Popen(
            [str(item) for item in command], cwd=WORKSPACE, env=environment,
            stdout=log, stderr=subprocess.STDOUT,
        )
        last_progress = started
        while process.poll() is None:
            peak = max(peak, rss_kib(process.pid))
            now = time.monotonic()
            if now - last_progress >= 30:
                tail = subprocess.run(
                    ["tail", "-n", "1", str(log_path)], text=True,
                    capture_output=True, check=False,
                ).stdout.strip()
                print(f"P6_I2_PROGRESS mode={name} elapsed_s={now-started:.0f} "
                      f"rss_peak_mib={peak/1024:.1f} {tail}", flush=True)
                last_progress = now
            time.sleep(1.0)
        status = process.wait()
    peak = max(peak, rss_kib(process.pid))
    if status:
        tail = subprocess.run(
            ["tail", "-n", "30", str(log_path)], text=True,
            capture_output=True, check=False,
        ).stdout
        raise RuntimeError(f"{name} failed with exit={status}:\n{tail}")
    elapsed = time.monotonic() - started
    print(f"P6_I2_RUN_COMPLETE mode={name} elapsed_s={elapsed:.1f} "
          f"peak_rss_mib={peak/1024:.1f}", flush=True)
    return elapsed, peak / 1024.0


def check_baseline_reproduction():
    replay = rows(OUT / "baseline_replay.csv")
    previous = rows(P6I1 / "trajectory_BASELINE.csv")
    if len(replay) != EXPECTED_SCANS or len(previous) != EXPECTED_SCANS:
        raise RuntimeError("baseline replay output is incomplete")
    limits = {key: 0.0 for key in (
        "predictor_t_difference_m", "predictor_r_difference_deg",
        "raw_t_difference_m", "raw_r_difference_deg",
        "used_t_difference_m", "used_r_difference_deg",
        "corrected_t_difference_m", "corrected_r_difference_deg")}
    for row, old in zip(replay, previous):
        if row["transaction_id"] != old["transaction_id"] or row["stamp_ns"] != old["stamp_ns"]:
            raise RuntimeError("baseline replay timestamp/transaction mismatch")
        if row["source_hash_expected"] != row["source_hash_actual"]:
            raise RuntimeError("baseline replay source-cloud hash mismatch")
        if row["replayed_converged"] != "1":
            raise RuntimeError("baseline replay NDT non-convergence")
        for key in limits:
            limits[key] = max(limits[key], float(row[key]))
    t_keys = [key for key in limits if key.endswith("_m")]
    r_keys = [key for key in limits if key.endswith("_deg")]
    if any(limits[key] >= 0.005 for key in t_keys) or any(
        limits[key] >= 0.05 for key in r_keys
    ):
        raise RuntimeError(f"P6-I1 baseline reproduction gate failed: {limits}")
    with (OUT / "baseline_gate.txt").open("w") as stream:
        stream.write("P6-I2 baseline reproduction PASS\n")
        for key, value in limits.items():
            stream.write(f"{key}={value:.17g}\n")
    print(f"P6_I2_BASELINE_REPLAY_PASS {limits}", flush=True)


def main():
    verify_frozen_inputs()
    s7_winners = audit_seed7()
    if s7_winners <= 0:
        raise RuntimeError("P6-I2 scope changed: GEO7 decision unexpectedly absent")

    run_logged(
        "BUILD",
        ["cmake", "--build", BUILD, "--parallel", "2"],
        OUT / "runner_build.log",
    )
    base_command = [
        EXECUTABLE, "baseline", INPUT / "imu.csv", INPUT / "filter_scans.csv",
        INPUT / "scans.csv", INPUT / "request_xyz_f32.bin", MAP,
        INPUT / "params.txt", OUT / "baseline_replay.csv",
        OUT / "trajectory_BASELINE.csv",
    ]
    elapsed_baseline, rss_baseline = run_logged(
        "BASELINE", base_command, OUT / "baseline_replay.log"
    )
    check_baseline_reproduction()

    common = [
        INPUT / "imu.csv", INPUT / "filter_scans.csv", INPUT / "scans.csv",
        INPUT / "request_xyz_f32.bin", MAP, INPUT / "params.txt",
        INPUT / "p4_i3_visual_valid_imu_translation.csv",
    ]
    geo_outputs = [
        OUT / "trajectory_geo7_reference.csv", OUT / "branch_events_GEO7_REFERENCE.csv",
        OUT / "dcreg_events_GEO7_REFERENCE.csv",
        OUT / "multistart_events_GEO7_REFERENCE.csv",
        OUT / "candidate_modes_GEO7_REFERENCE.csv",
        OUT / "visual_arbitration_GEO7_REFERENCE.csv",
        OUT / "runtime_breakdown_GEO7_REFERENCE.csv",
    ]
    elapsed_geo7, rss_geo7 = run_logged(
        "GEO7_REFERENCE",
        [EXECUTABLE, "GEO7_REFERENCE", *common, *geo_outputs],
        OUT / "GEO7_REFERENCE.log",
    )
    if len(rows(geo_outputs[0])) != EXPECTED_SCANS:
        raise RuntimeError("GEO7_REFERENCE closed-loop trajectory incomplete")

    cov_outputs = [
        OUT / "trajectory_cov3.csv", OUT / "branch_events_COV3_OBJECTIVE.csv",
        OUT / "dcreg_events_COV3_OBJECTIVE.csv",
        OUT / "multistart_events_COV3_OBJECTIVE.csv",
        OUT / "candidate_modes_COV3_OBJECTIVE.csv",
        OUT / "visual_arbitration_COV3_OBJECTIVE.csv",
        OUT / "runtime_breakdown_COV3_OBJECTIVE.csv",
        OUT / "basin_events.csv", OUT / "covariance_probe_direction.csv",
    ]
    elapsed_cov3, rss_cov3 = run_logged(
        "COV3_OBJECTIVE",
        [EXECUTABLE, "COV3_OBJECTIVE", *common, *cov_outputs],
        OUT / "COV3_OBJECTIVE.log",
    )
    if len(rows(cov_outputs[0])) != EXPECTED_SCANS:
        raise RuntimeError("COV3_OBJECTIVE closed-loop trajectory incomplete")
    if len(rows(cov_outputs[7])) != EXPECTED_SCANS or len(rows(cov_outputs[8])) != EXPECTED_SCANS:
        raise RuntimeError("COV3 diagnostic stream incomplete")

    with (OUT / "memory_metrics.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=("mode", "peak_rss_mib", "wall_time_s"),
                                 lineterminator="\n")
        writer.writeheader()
        for mode, rss, elapsed in (
            ("BASELINE", rss_baseline, elapsed_baseline),
            ("GEO7_REFERENCE", rss_geo7, elapsed_geo7),
            ("COV3_OBJECTIVE", rss_cov3, elapsed_cov3),
        ):
            writer.writerow({"mode": mode, "peak_rss_mib": f"{rss:.6f}",
                             "wall_time_s": f"{elapsed:.6f}"})
    lineage = [
        f"workspace_head={git('rev-parse', 'HEAD')}",
        f"origin_paper={subprocess.check_output(['git','-C',str(WORKSPACE),'ls-remote','origin','refs/heads/paper'], text=True).strip()}",
        f"frozen_map_sha256={EXPECTED_MAP_SHA}",
        f"floor01_bag_sha256={EXPECTED_BAG_SHA}",
        f"floor01_config_sha256={EXPECTED_CONFIG_SHA}",
        f"input_manifest_sha256={sha256(INPUT / 'input_manifest.txt')}",
        f"baseline_reference_sha256={sha256(P6I1 / 'trajectory_BASELINE.csv')}",
        f"objective_reference_sha256={sha256(P6I1 / 'trajectory_MULTISTART_OBJECTIVE.csv')}",
        f"p6_i1_visual_seed7_winners={s7_winners}",
        "GEO7_REFERENCE=7 seeds S0-S6 at every scan; fixed P6-I1 +/-0.8m predicted-lidar/body-frame x/y and +/-5deg yaw geometry",
        "COV3_OBJECTIVE=nominal plus/minus max-eigenvector of predicted map-frame P_xy; rho=0.8m",
        "GT=NOT OPENED; post-hoc evaluator must run only after all trajectories pass completeness checks",
    ]
    (OUT / "input_lineage.txt").write_text("\n".join(lineage) + "\n")
    print("P6_I2_ALL_PRE_GT_REPLAYS_COMPLETE", flush=True)


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"PAPER_P6_I2_RUN_BLOCKED: {error}", file=sys.stderr, flush=True)
        raise
