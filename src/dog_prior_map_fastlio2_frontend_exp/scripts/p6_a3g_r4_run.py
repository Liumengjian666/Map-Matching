#!/usr/bin/env python3
"""Guarded R4 debugging run; frozen sensor inputs, no GT and no core tuning."""
import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from p6_a3d_r1_replay_guard import INPUTS, ROOT, CsvTail, input_gate
from p6_a3g_soak import HealthAudit, read_csv, require, summarize

START = "45ceeabdc835ebae96ee98a033dd9575a29b94e9"
INIT = 1517157224188979000


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def tracking_summary(directory):
    rows = read_csv(directory / "trajectory.csv.r4_tracking.csv")
    results = [r for r in rows if r["row_type"] == "RESULT"]
    candidates = [r for r in rows if r["row_type"] == "CANDIDATE"]
    streak = longest = 0
    last_commit = None
    for row in results:
        if row["committed"] == "1":
            last_commit = row["transaction_id"]
            streak = 0
        else:
            streak += 1
            longest = max(longest, streak)
    return dict(terminals=len(results), last_LiDAR_commit=last_commit,
        longest_noncommit_streak=longest, trailing_noncommit_streak=streak,
        health_states=dict(Counter(r["health"] for r in results)),
        recovery_NDT_calls=sum(r["seed"] != "WINDOW_M0" for r in candidates),
        recovered_commits=sum(r["committed"] == "1" and r["seed"] != "WINDOW_M0" for r in results),
        key_transactions={r["transaction_id"]:r for r in results
                          if r["transaction_id"] in ("160","166","174","182","183","202","220","366")})


def run(executable, directory, limit, config):
    # Normal descendants only; never switch/reset the shared worktree.
    subprocess.run(["git", "merge-base", "--is-ancestor", START, "HEAD"], cwd=ROOT, check=True)
    identities = input_gate()
    frozen = json.loads((ROOT / "docs/p6_alg_integration_a3f_r1/RUN_P3_200/input_identity.json").read_text())
    require(identities == frozen, "FROZEN_INPUT_IDENTITY_MISMATCH")
    catalog = read_csv(INPUTS[1][0])
    require(limit in (220, 400, len(catalog)), "UNAUTHORIZED_DEBUG_RANGE")
    selected = catalog[:limit]
    post = [r for r in selected if int(r["scan_start_ns"]) > INIT]
    directory.mkdir(parents=True, exist_ok=False)
    (directory / "input_identity.json").write_text(json.dumps(identities, indent=2) + "\n")
    # Pin the actual config used by this run, rather than a mutable shared path.
    run_config = directory / "tracking.conf"
    run_config.write_bytes(config.read_bytes())
    head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    diff = subprocess.check_output(["git", "diff", START, "--"], cwd=ROOT, text=True)
    (directory / "source_diff.patch").write_text(diff)
    env = dict(os.environ, LD_LIBRARY_PATH="/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu",
        P6_A3B_R1_DIAGNOSTICS="0", P6_A3C_R1_MARGINALIZATION_DIAGNOSTICS="0",
        P6_A3F_R1_COVARIANCE_DIAGNOSTICS="0", P6_A3G_HEALTH_DIAGNOSTICS="1")
    for key in ("P6_A3G_R3_CAPTURE_DIR", "P6_A3G_R3_CAPTURE_TRANSACTIONS"):
        env.pop(key, None)
    command = ["/usr/bin/time", "-v", "-o", str(directory / "resource.txt"), str(executable),
        "FULL_FIXED_LAG_V3_EXPERIMENTAL", str(INPUTS[6][0]), str(INPUTS[2][0]), str(INPUTS[1][0]),
        str(INPUTS[0][0]), str(INPUTS[4][0]), str(INPUTS[5][0]), str(directory / "trajectory.csv"),
        str(directory / "events.csv"), str(directory / "runtime.csv"), "NONE", str(limit), str(INIT),
        "corridor01", "ADAPTIVE_SELECTED_NIS", str(INPUTS[1][0]), "NONE", str(run_config)]
    (directory / "command.json").write_text(json.dumps(command, indent=2) + "\n")
    (directory / "source_identity.json").write_text(json.dumps(dict(START_SHA=START, CODE_SHA=head,
        binary_sha256=digest(executable), config_sha256=digest(run_config),
        diff_sha256=hashlib.sha256(diff.encode()).hexdigest(),
        environment={k:v for k,v in env.items() if k.startswith("P6_") or k == "LD_LIBRARY_PATH"},
        raw_scans=limit, expected_terminals=len(post), GT_USED=False, VISUAL="NONE"), indent=2) + "\n")
    tails = {"health":CsvTail(directory / "trajectory.csv.a3g_health.csv"),
        "covariance":CsvTail(directory / "trajectory.csv.a3f_r1_covariance.csv"),
        "marginalization":CsvTail(directory / "trajectory.csv.a3c_r1_marginalization_trace.csv"),
        "events":CsvTail(directory / "events.csv"), "trajectory":CsvTail(directory / "trajectory.csv")}
    tracking = CsvTail(directory / "trajectory.csv.r4_tracking.csv")
    audit = HealthAudit({r["scan_end_ns"]:r["transaction_id"] for r in post})
    streak = 0
    last_tracking = None
    error = ""
    def inspect():
        nonlocal streak, last_tracking
        for kind, tail in tails.items():
            for row in tail.rows():
                audit.consume(kind, row)
        for row in tracking.rows():
            if row["row_type"] != "RESULT":
                continue
            last_tracking = row
            streak = 0 if row["committed"] == "1" else streak + 1
            # A test guard, not a measurement gate: do not continue into the
            # known hundreds-of-frames starvation failure to collect more rows.
            require(streak < 20, "PERSISTENT_LIDAR_STARVATION:" + row["transaction_id"])
    with (directory / "console.txt").open("w") as console:
        process = subprocess.Popen(command, env=env, stdout=console, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            while True:
                inspect()
                if process.poll() is not None:
                    inspect()
                    if process.returncode == 0:
                        audit.finish(len(post))
                    break
                time.sleep(.05)
        except BaseException as exc:
            error = str(exc) or type(exc).__name__
            if not error.startswith("PERSISTENT_LIDAR_STARVATION"):
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    pass
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=2)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
        code = process.wait(timeout=5)
    result = dict(exit_code=code, completed=code == 0 and not error, first_failure=error,
        frame_limit=limit, expected_terminals=len(post), counts=dict(audit.counts),
        last_health_record=audit.last_row, last_tracking=last_tracking, GT_USED=False, VISUAL="NONE")
    (directory / "run_result.json").write_text(json.dumps(result, indent=2) + "\n")
    (directory / "tracking_summary.json").write_text(json.dumps(tracking_summary(directory), indent=2) + "\n")
    (directory / "health_summary.json").write_text(json.dumps(summarize(directory), indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return 0 if result["completed"] else 2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--limit", type=int, required=True)
    parser.add_argument("--config", type=Path, default=ROOT / "config/p6_tracking_recovery.conf")
    args = parser.parse_args()
    raise SystemExit(run(args.executable.resolve(), args.directory.resolve(), args.limit, args.config.resolve()))
