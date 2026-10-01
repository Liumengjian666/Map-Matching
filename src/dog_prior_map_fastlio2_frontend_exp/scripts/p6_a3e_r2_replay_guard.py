#!/usr/bin/env python3
"""One-shot QR engineering replay; no GT, retries or estimator adjustments."""
import argparse
import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from p6_a3d_r1_replay_guard import INPUTS, ROOT, CsvTail, check, input_gate

OLD = ROOT / "docs/p6_alg_integration_a3d_r1/RUN_P3_200"
FIELDS = ["event_type", "window_nodes", "window_span", "optimizer_status",
          "optimizer_cost_before", "optimizer_cost_after", "predicted_px",
          "predicted_py", "predicted_pz", "predicted_qx", "predicted_qy",
          "predicted_qz", "predicted_qw", "lidar_factor_committed", "lidar_selected_rank"]
FLOAT_FIELDS = set(FIELDS[2:3] + FIELDS[4:13])


def compare(row, old):
    check(row, {key: float(old[key]) if key in FLOAT_FIELDS else old[key] for key in FIELDS})


def run(executable, directory):
    identities = input_gate()
    first = next(csv.DictReader((OLD / "trajectory.csv.a3c_r1_marginalization_trace.csv").open()))
    cutoff = int(first["event_stamp_ns"])
    old = {r["timestamp"]: r for r in csv.DictReader((OLD / "events.csv").open())}
    directory.mkdir(parents=True, exist_ok=False)
    (directory / "input_identity.json").write_text(json.dumps(identities, indent=2) + "\n")
    command = ["/usr/bin/time", "-v", "-o", str(directory / "resource.txt"),
        str(executable), "FULL_FIXED_LAG_V3_EXPERIMENTAL", str(INPUTS[6][0]),
        str(INPUTS[2][0]), str(INPUTS[1][0]), str(INPUTS[0][0]), str(INPUTS[4][0]),
        str(INPUTS[5][0]), str(directory / "trajectory.csv"), str(directory / "events.csv"),
        str(directory / "runtime.csv"), "NONE", "200", "1517157224188979000",
        "corridor01", "ADAPTIVE_SELECTED_NIS", str(INPUTS[1][0]), "NONE"]
    environment = dict(os.environ, LD_LIBRARY_PATH="/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu",
        P6_A3B_R1_DIAGNOSTICS="1", P6_A3C_R1_MARGINALIZATION_DIAGNOSTICS="1")
    (directory / "command.json").write_text(json.dumps(command, indent=2) + "\n")
    tail = CsvTail(directory / "events.csv")
    checked, divergence, error = 0, None, ""

    def inspect():
        nonlocal checked, divergence
        for row in tail.rows():
            stamp = row["timestamp"]
            if int(stamp) < cutoff:
                if stamp not in old:
                    raise RuntimeError("REAL_REPLAY_DETERMINISM_DISCREPANCY: unknown pre-marginalization event")
                compare(row, old[stamp])
                checked += 1
            elif divergence is None and stamp in old:
                try:
                    compare(row, old[stamp])
                except RuntimeError:
                    divergence = dict(stamp_ns=stamp, event_type=row["event_type"],
                        explanation="SQUARE_ROOT_QR_BACKEND_AFTER_FIRST_MARGINALIZATION")

    with (directory / "console.txt").open("w") as console:
        process = subprocess.Popen(command, env=environment, stdout=console,
            stderr=subprocess.STDOUT, start_new_session=True)
        try:
            while True:
                inspect()
                if process.poll() is not None:
                    inspect()
                    if checked == 0:
                        raise RuntimeError("PRE_MARGINALIZATION_IDENTITY_NOT_OBSERVED")
                    break
                time.sleep(0.05)
        except BaseException as exc:
            error = str(exc) or type(exc).__name__
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
        exit_code = process.wait(timeout=5)
    result = dict(pre_marginalization_events_checked=checked, first_old_marginalization=first,
        first_backend_divergence=divergence, discrepancy=error, process_exit_code=exit_code,
        real_replay_count=1, retries=0, GT_USED=False)
    (directory / "identity_gate.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return 2 if error else exit_code


def self_test():
    row={k: "0.25" if k in FLOAT_FIELDS else "SAME" for k in FIELDS}
    compare(row,row)
    bad=dict(row, predicted_px="nan")
    try:
        compare(bad,row)
    except RuntimeError:
        pass
    else:
        raise AssertionError("nonfinite identity accepted")
    print("A3E_R2_REPLAY_GUARD_SELF_TEST_PASS")


if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--self-test",action="store_true")
    parser.add_argument("--executable",type=Path)
    parser.add_argument("--run-dir",type=Path)
    args=parser.parse_args()
    if args.self_test:
        self_test()
    elif args.executable and args.run_dir:
        raise SystemExit(run(args.executable.resolve(),args.run_dir.resolve()))
    else:
        parser.error("--executable and --run-dir required")
