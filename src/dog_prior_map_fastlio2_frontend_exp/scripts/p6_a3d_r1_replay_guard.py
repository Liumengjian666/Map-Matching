#!/usr/bin/env python3
"""One-shot A3D-R1 replay launcher with a live frozen tx115 identity guard.

This is engineering validation plumbing, not estimator code or a GT tool.
No retry is implemented. Incomplete CSV lines are never evaluated.
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


ROOT = Path(__file__).resolve().parents[1]
DATA = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01")
RAW = DATA / "results/p6_a3a_v3_input"
INPUTS = [
    (RAW / "raw_timed_points.bin", "4ba09d8ae7004056dcc4e9d63d2ab09915748c7e68bc29d0dc8bd44a24fc95ff"),
    (RAW / "raw_timed_catalog.csv", "fdaf9607bc1933269f5f999ee044d37aa1eefabedc054f522da5078ce708123f"),
    (RAW / "filter_scans.csv", "41d0b2040a5de8a8bd428c382a7b6dc18fabcaa8d3e7d2cc331018e0585edf1d"),
    (RAW / "RAW_TIMED_INPUT_MANIFEST.txt", "fb20125c63aa4c7110851b8bb08ec33e8938d94be18e28e9737d207001377575"),
    (DATA / "map/derived/corridor01_map_normalized.pcd", "103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f"),
    (ROOT / "docs/p6_i6d_full_algorithm/corridor01_params_official_calibration.txt", "7e42752ff8b84eae2b2da8d7d9fe179db0bb8f364a923e12236d2e91336e357d"),
    (DATA / "results/p6_i6c_framework/input/imu.csv", "7dc881d4ebfeacea9354be569a5952e5e466ccdd366637e52a7797a6f37457aa"),
]


def input_gate():
    records = []
    for path, expected in INPUTS:
        digest = hashlib.sha256()
        with path.open("rb") as source:
            for block in iter(lambda: source.read(1024 * 1024), b""):
                digest.update(block)
        actual = digest.hexdigest()
        if actual != expected:
            raise RuntimeError(f"INPUT_HASH_MISMATCH: {path}")
        records.append(dict(path=str(path), sha256=actual))
    return records


class CsvTail:
    def __init__(self, path):
        self.path, self.offset, self.pending, self.header = path, 0, "", None

    def rows(self):
        if not self.path.exists():
            return []
        with self.path.open() as source:
            source.seek(self.offset)
            data = self.pending + source.read()
            self.offset = source.tell()
        split = data.rfind("\n") + 1
        self.pending = data[split:]
        lines = data[:split].splitlines()
        if self.header is None and lines:
            self.header = next(csv.reader([lines.pop(0)]))
        return list(csv.DictReader(lines, fieldnames=self.header)) if self.header else []


def check(row, expected):
    for key, value in expected.items():
        if isinstance(value, float):
            # Identity tolerance only: allows output-rounding last digits,
            # never changes any estimator threshold or candidate acceptance.
            if not math.isclose(float(row[key]), value, rel_tol=5e-13, abs_tol=0.0):
                raise RuntimeError(f"REAL_REPLAY_DETERMINISM_DISCREPANCY: {key}={row[key]}")
        elif row[key] != value:
            raise RuntimeError(f"REAL_REPLAY_DETERMINISM_DISCREPANCY: {key}={row[key]}")


def run(executable, directory):
    identities = input_gate()  # Must complete before the only real launch.
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
    checked = dict(preopt=False, first_candidate=False)
    tails = {flag: CsvTail(directory / f"trajectory.csv.{suffix}.csv")
             for suffix, flag in (("r1_preopt_capsule", "preopt"),
                                  ("r1_optimizer_trace", "first_candidate"))}
    def inspect_identity():
        for flag, tail in tails.items():
            if checked[flag]:
                continue
            matching = [r for r in tail.rows() if r["transaction_id"] == "115"]
            if not matching:
                continue
            expected = dict(stamp_ns="1517157230686328484")
            if flag == "preopt":
                expected.update(window_nodes="41", window_span_s=2.017077208,
                    selected_nis=15.237396865140097, nis_threshold=15.086,
                    lidar_committed="0")
            else:
                expected.update(iteration="0", surrogate_current_cost=81.345724589884568,
                    surrogate_candidate_cost=81.345724589884583,
                    raw_step_norm=7.0568167407415232e-9,
                    applied_step_norm=7.0568167407415232e-9,
                    gradient_inf_norm=2.8457479913868156e-5,
                    damping_before=1e-6, damping_after=1e-5,
                    solver_status="BLOCK_SPARSE_SIMPLICIAL_LDLT",
                    step_clipped="0", accepted="0")
            check(matching[0], expected)
            checked[flag] = True

    error = ""
    with (directory / "console.txt").open("w") as console:
        process = subprocess.Popen(command, env=environment, stdout=console,
            stderr=subprocess.STDOUT, start_new_session=True)
        try:
            while True:
                inspect_identity()
                if process.poll() is not None:
                    inspect_identity()  # Drain rows flushed at process exit.
                    if process.returncode == 0 and not all(checked.values()):
                        raise RuntimeError("REAL_REPLAY_DETERMINISM_DISCREPANCY: TX115_IDENTITY_NOT_OBSERVED")
                    break
                time.sleep(0.01)
        except BaseException as exc:
            error = str(exc) or type(exc).__name__
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    pass
                finally:
                    # Also stop a surviving child if /usr/bin/time has exited.
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
        finally:
            exit_code = process.wait(timeout=5)
    result = dict(identity_checks=checked, discrepancy=error, process_exit_code=exit_code,
                  real_replay_count=1, retries=0, GT_USED=False)
    (directory / "identity_gate.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return 2 if error else exit_code


def self_test():
    check({"stamp": "115", "cost": "200"}, {"stamp": "115", "cost": 200.0})
    for row in ({"stamp": "116", "cost": "200"}, {"stamp": "115", "cost": "nan"}):
        try:
            check(row, {"stamp": "115", "cost": 200.0})
        except RuntimeError:
            pass
        else:
            raise AssertionError("identity discrepancy was not rejected")
    try:
        check({"step": "7.0568176407415234e-9"}, {"step": 7.0568167407415232e-9})
    except RuntimeError:
        pass
    else:
        raise AssertionError("tiny-step relative identity drift was not rejected")
    with tempfile.TemporaryDirectory(prefix="p6_a3d_guard_test_") as directory:
        path = Path(directory) / "stream.csv"
        tail = CsvTail(path)
        assert tail.rows() == []
        path.write_text("transaction_id,value\n115,")
        assert tail.rows() == []
        with path.open("a") as output:
            output.write("200\n116,201\n")
        assert tail.rows() == [{"transaction_id": "115", "value": "200"},
                               {"transaction_id": "116", "value": "201"}]
        assert tail.rows() == []
    print("A3D_R1_REPLAY_GUARD_SELF_TEST_PASS")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--executable", type=Path)
    parser.add_argument("--run-dir", type=Path)
    args = parser.parse_args()
    if args.self_test:
        self_test()
        raise SystemExit(0)
    if args.executable is None or args.run_dir is None:
        parser.error("--executable and --run-dir required for the one-shot replay")
    raise SystemExit(run(args.executable.resolve(), args.run_dir.resolve()))
