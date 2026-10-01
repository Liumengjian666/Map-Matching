#!/usr/bin/env python3
"""One-shot A3F covariance validation. No GT, retries or estimator changes."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from p6_a3d_r1_replay_guard import INPUTS, ROOT, CsvTail, check, input_gate
from p6_a3e_r2_replay_guard import compare

FROZEN = ROOT / "docs/p6_alg_integration_a3e_r2/RUN_P3_200/EXTERNAL_RUN_FILES.json"


class IdentityAudit:
    """Classify events only once the preceding covariance output is consumed."""
    def __init__(self, old, transactions):
        self.old, self.transactions = old, transactions
        self.pending = []
        self.requests = self.comparisons = self.events = self.terminals = self.checked = 0
        self.comparison_stamp = 0
        self.first_output = self.first_decision = self.first_state = None
        self.request_ids, self.comparison_ids, self.terminal_ids, self.event_stamps = set(), set(), set(), set()

    def identity(self, row):
        stamp = row["stamp_ns"]
        old = self.old.get(stamp)
        if not old or old["event_type"] != "LIDAR_SCAN_END" or self.transactions.get(stamp) != row["transaction_id"]:
            raise RuntimeError("COVARIANCE_DIAGNOSTIC_IDENTITY_MISMATCH")
        return (stamp, row["transaction_id"])

    def request(self, row):
        identity = self.identity(row)
        if identity in self.request_ids:
            raise RuntimeError("DUPLICATE_COVARIANCE_REQUEST")
        self.request_ids.add(identity)
        if row["backend"] != "SQUARE_ROOT_QR" or row["valid"] != "1":
            raise RuntimeError("FIRST_PRODUCTION_COVARIANCE_UNAVAILABLE:" + json.dumps(row))
        self.requests += 1

    def comparison(self, row):
        identity = self.identity(row)
        if identity in self.comparison_ids:
            raise RuntimeError("DUPLICATE_COVARIANCE_COMPARISON")
        self.comparison_ids.add(identity)
        differs = row["legacy_valid"] != "1" or any(float(row[k]) > 0 for k in
            ("P15_relative_error", "Pmap_relative_error"))
        if self.first_output is None and differs:
            self.first_output = dict(row)
        decision_diff = row["legacy_valid"] != "1" or any(row[a] != row[b] for a, b in (
            ("production_probe_trigger", "legacy_probe_trigger"),
            ("production_NIS_accepted", "legacy_same_factor_NIS_accepted"),
            ("production_LiDAR_committed", "legacy_same_factor_admission")))
        if self.first_decision is None and decision_diff:
            self.first_decision = dict(row)
        stamp = int(row["stamp_ns"])
        if stamp <= self.comparison_stamp:
            raise RuntimeError("NON_MONOTONIC_COVARIANCE_COMPARISON")
        self.comparison_stamp = stamp
        self.comparisons += 1

    def consume(self, rows):
        self.pending.extend(rows)
        while self.pending and int(self.pending[0]["timestamp"]) <= self.comparison_stamp:
            row = self.pending.pop(0)
            previous = self.old.get(row["timestamp"])
            if previous is None:
                raise RuntimeError("REAL_REPLAY_DETERMINISM_DISCREPANCY: sensor schedule")
            if row["timestamp"] in self.event_stamps:
                raise RuntimeError("DUPLICATE_REAL_EVENT")
            if row["event_type"] == "LIDAR_SCAN_END":
                identity = (row["timestamp"], self.transactions[row["timestamp"]])
                if identity not in self.comparison_ids:
                    raise RuntimeError("MISSING_TERMINAL_COVARIANCE_COMPARISON")
                self.terminal_ids.add(identity)
            self.event_stamps.add(row["timestamp"])
            before = self.first_output is None or int(row["timestamp"]) < int(self.first_output["stamp_ns"])
            # The first changed covariance must still start at the same state.
            if self.first_output and row["timestamp"] == self.first_output["stamp_ns"]:
                check(row, {k: float(previous[k]) for k in previous if k.startswith("predicted_")})
            try:
                compare(row, previous)
                if before:
                    self.checked += 1
            except RuntimeError:
                if before:
                    raise
                if self.first_state is None:
                    self.first_state = dict(transaction_id=self.transactions[row["timestamp"]],
                        stamp_ns=row["timestamp"], event=row["event_type"],
                        explanation="AFTER_FIRST_COVARIANCE_OUTPUT_DIFFERENCE")
            self.events += 1
            self.terminals += row["event_type"] == "LIDAR_SCAN_END"

    def finish(self, expected=149):
        if self.pending or not self.checked or not (
            self.requests == self.comparisons == self.terminals == expected and self.events == 2*expected) or not (
            self.request_ids == self.comparison_ids == self.terminal_ids):
            raise RuntimeError("MISSING_OR_UNMATCHED_REAL_DIAGNOSTIC_STREAM")


def frozen_events():
    entry = next(r for r in json.loads(FROZEN.read_text()) if Path(r["path"]).name == "events.csv")
    data = Path(entry["path"]).read_bytes()
    if hashlib.sha256(data).hexdigest() != entry["sha256"]:
        raise RuntimeError("FROZEN_A3E_R2_REFERENCE_HASH_MISMATCH")
    return {r["timestamp"]: r for r in csv.DictReader(data.decode().splitlines())}


def run(executable, directory):
    source_sha = subprocess.check_output(["git","rev-parse","HEAD"],cwd=ROOT,text=True).strip()
    dirty = subprocess.check_output(["git","status","--short"],cwd=ROOT,text=True)
    if dirty:
        raise RuntimeError("REAL_REPLAY_REQUIRES_COMMITTED_CLEAN_SOURCE")
    subprocess.run(["git","merge-base","--is-ancestor",
        "34ac3f7692ff51efc3c168fb45e080b8573dee28",source_sha],cwd=ROOT,check=True)
    identities = input_gate()
    old = frozen_events()
    sensor_transactions = {r[key]: r["transaction_id"]
        for r in csv.DictReader(INPUTS[1][0].open())
        for key in ("scan_start_ns", "scan_end_ns")}
    # A unique directory forbids accidental overwrite or a second run in it.
    directory.mkdir(parents=True, exist_ok=False)
    (directory / "input_identity.json").write_text(json.dumps(identities, indent=2) + "\n")
    command = ["/usr/bin/time", "-v", "-o", str(directory / "resource.txt"),
        str(executable), "FULL_FIXED_LAG_V3_EXPERIMENTAL", str(INPUTS[6][0]),
        str(INPUTS[2][0]), str(INPUTS[1][0]), str(INPUTS[0][0]), str(INPUTS[4][0]),
        str(INPUTS[5][0]), str(directory / "trajectory.csv"), str(directory / "events.csv"),
        str(directory / "runtime.csv"), "NONE", "200", "1517157224188979000",
        "corridor01", "ADAPTIVE_SELECTED_NIS", str(INPUTS[1][0]), "NONE"]
    environment = dict(os.environ, LD_LIBRARY_PATH="/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu",
        P6_A3B_R1_DIAGNOSTICS="1", P6_A3C_R1_MARGINALIZATION_DIAGNOSTICS="1",
        P6_A3F_R1_COVARIANCE_DIAGNOSTICS="1")
    (directory / "command.json").write_text(json.dumps(command, indent=2) + "\n")
    (directory / "source_identity.json").write_text(json.dumps(dict(
        CODE_SHA=source_sha, executable_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(),
        environment={k:environment[k] for k in ("LD_LIBRARY_PATH","P6_A3B_R1_DIAGNOSTICS",
            "P6_A3C_R1_MARGINALIZATION_DIAGNOSTICS","P6_A3F_R1_COVARIANCE_DIAGNOSTICS")}),indent=2)+"\n")
    cov_tail = CsvTail(directory / "trajectory.csv.a3f_r1_comparison.csv")
    event_tail = CsvTail(directory / "events.csv")
    request_tail = CsvTail(directory / "trajectory.csv.a3f_r1_covariance.csv")
    audit = IdentityAudit(old, sensor_transactions)
    error = ""

    def inspect():
        for row in request_tail.rows():
            audit.request(row)
        for row in cov_tail.rows():
            audit.comparison(row)
        audit.consume(event_tail.rows())

    with (directory / "console.txt").open("w") as console:
        process = subprocess.Popen(command, env=environment, stdout=console,
            stderr=subprocess.STDOUT, start_new_session=True)
        try:
            while True:
                inspect()
                if process.poll() is not None:
                    inspect()
                    if process.returncode == 0:
                        audit.finish()
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
    result = dict(pre_covariance_output_events_checked=audit.checked,
        first_covariance_output_divergence=audit.first_output,
        first_same_state_shadow_decision_divergence=audit.first_decision,
        first_old_run_event_divergence=audit.first_state, discrepancy=error,
        process_exit_code=exit_code, real_replay_count=1, retries=0, GT_USED=False)
    (directory / "identity_gate.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return 2 if error else exit_code


def self_test():
    # Existing strict logical/sensor comparison still rejects NaN and drift.
    check({"stamp": "123", "x": "0.25"}, {"stamp": "123", "x": 0.25})
    for bad in ("nan", "0.3"):
        try:
            check({"x": bad}, {"x": 0.25})
        except RuntimeError:
            pass
        else:
            raise AssertionError("invalid identity accepted")
    from p6_a3e_r2_replay_guard import FIELDS, FLOAT_FIELDS
    start = {k: "0.25" if k in FLOAT_FIELDS else "SAME" for k in FIELDS}
    start.update(timestamp="1", event_type="LIDAR_SCAN_START")
    end = dict(start, timestamp="2", event_type="LIDAR_SCAN_END")
    audit = IdentityAudit({"1": start, "2": end}, {"1": "1", "2": "1"})
    # Event tail outruns comparison tail: buffer, do not classify prematurely.
    audit.consume([start, dict(end, optimizer_cost_after="0.3")])
    assert audit.events == 0 and len(audit.pending) == 2
    row = dict(transaction_id="1", stamp_ns="2", legacy_valid="1", P15_relative_error="0",
        Pmap_relative_error="1e-16", production_probe_trigger="0", legacy_probe_trigger="0",
        production_NIS_accepted="1", legacy_same_factor_NIS_accepted="1",
        production_LiDAR_committed="1", legacy_same_factor_admission="1")
    audit.request(dict(transaction_id="1", stamp_ns="2", backend="SQUARE_ROOT_QR", valid="1"))
    audit.comparison(row)
    audit.consume([])
    assert audit.first_output == row and audit.first_state is not None and audit.checked == 1
    audit.finish(expected=1)
    for broken in (dict(row, stamp_ns="3"), dict(row, transaction_id="2")):
        try:
            IdentityAudit({"1": start, "2": end}, {"1": "1", "2": "1"}).comparison(broken)
        except RuntimeError:
            pass
        else:
            raise AssertionError("mismatched diagnostic identity accepted")
    try:
        audit.consume([end])
    except RuntimeError:
        pass
    else:
        raise AssertionError("duplicate event accepted")
    empty = IdentityAudit({}, {})
    try:
        empty.finish()
    except RuntimeError:
        pass
    else:
        raise AssertionError("empty streams accepted")
    print("A3F_R1_REPLAY_GUARD_SELF_TEST_PASS")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--executable", type=Path)
    parser.add_argument("--run-dir", type=Path)
    args = parser.parse_args()
    if args.self_test:
        self_test()
    elif args.executable and args.run_dir:
        raise SystemExit(run(args.executable.resolve(), args.run_dir.resolve()))
    else:
        parser.error("--executable and --run-dir required")
