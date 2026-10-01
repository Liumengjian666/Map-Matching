#!/usr/bin/env python3
"""Single-use guarded A3G-R3 prefix evidence capture launcher."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
PACKAGE = REPO / "src/dog_prior_map_fastlio2_frontend_exp"
EXPECTED_START = "484da9c88ad5866a58761aeef702a31a168607a2"
BRANCH = "research/p6-i6d-full-algorithm"
RAW_DIR = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p6_a3a_v3_input")
DATA_ROOT = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01")
FROZEN_RUN = DATA_ROOT / "results/p6_a3g_full_corridor"
FROZEN_DOCS = PACKAGE / "docs/p6_alg_integration_a3g/RUN_FULL_FIRST_FAILURE"
TARGET = DATA_ROOT / "results/p6_a3g_r3_capture"
BUILD = Path("/tmp/p6_a2d_build/p6_i6b_closed_loop")
PARAMS = PACKAGE / "docs/p6_i6d_full_algorithm/corridor01_params_official_calibration.txt"
INIT_NS = 1517157224188979000

INPUT_SHA = {
    str(RAW_DIR / "raw_timed_points.bin"): "4ba09d8ae7004056dcc4e9d63d2ab09915748c7e68bc29d0dc8bd44a24fc95ff",
    str(RAW_DIR / "raw_timed_catalog.csv"): "fdaf9607bc1933269f5f999ee044d37aa1eefabedc054f522da5078ce708123f",
    str(RAW_DIR / "filter_scans.csv"): "41d0b2040a5de8a8bd428c382a7b6dc18fabcaa8d3e7d2cc331018e0585edf1d",
    str(RAW_DIR / "RAW_TIMED_INPUT_MANIFEST.txt"): "fb20125c63aa4c7110851b8bb08ec33e8938d94be18e28e9737d207001377575",
    str(DATA_ROOT / "map/derived/corridor01_map_normalized.pcd"): "103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f",
    str(PARAMS): "7e42752ff8b84eae2b2da8d7d9fe179db0bb8f364a923e12236d2e91336e357d",
    str(DATA_ROOT / "results/p6_i6c_framework/input/imu.csv"): "7dc881d4ebfeacea9354be569a5952e5e466ccdd366637e52a7797a6f37457aa",
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=REPO, text=True).strip()


def require_frozen_input_identity(entries, name: str, expected_sha: str) -> None:
    recorded = next((entry["sha256"] for entry in entries if entry["path"] == name), None)
    if recorded != expected_sha:
        raise RuntimeError(f"A3G input identity ledger mismatch: {name}")


def check_preflight(expected_code_sha: str, expected_binary_sha256: str):
    head = git("rev-parse", "HEAD")
    branch = git("branch", "--show-current")
    status = git("status", "--short")
    if branch != BRANCH:
        raise RuntimeError(f"branch mismatch: {branch}")
    if status:
        raise RuntimeError(f"worktree not clean: {status}")
    if head != expected_code_sha:
        raise RuntimeError(f"CODE_SHA mismatch: expected={expected_code_sha} actual={head}")
    subprocess.run(["git", "merge-base", "--is-ancestor", EXPECTED_START, head],
                   cwd=REPO, check=True)
    actual_inputs = {}
    for name, expected in INPUT_SHA.items():
        path = Path(name)
        actual = sha256(path)
        if actual != expected:
            raise RuntimeError(f"frozen input SHA mismatch: {path}")
        actual_inputs[name] = {"bytes": path.stat().st_size, "sha256": actual}
    frozen_identity = json.loads((FROZEN_RUN / "input_identity.json").read_text())
    for name, expected in INPUT_SHA.items():
        require_frozen_input_identity(frozen_identity, name, expected)
    external_ledger_path = FROZEN_DOCS / "EXTERNAL_RUN_FILES.json"
    ledger = json.loads(external_ledger_path.read_text())
    verified_artifacts = []
    for entry in ledger:
        path = Path(entry["path"])
        if not path.is_file() or path.stat().st_size != entry["bytes"] or sha256(path) != entry["sha256"]:
            raise RuntimeError(f"frozen A3G artifact SHA/size mismatch: {path}")
        verified_artifacts.append({"path": str(path), "bytes": entry["bytes"], "sha256": entry["sha256"]})
    expected_ledger_sha = "4992c4f56fa49e2b8837f57733b9edc2bde51efb997e6c475a716be35a3cdf05"
    if sha256(external_ledger_path) != expected_ledger_sha:
        raise RuntimeError("frozen A3G external ledger SHA mismatch")
    if not BUILD.is_file():
        raise RuntimeError(f"closed-loop executable missing: {BUILD}")
    binary_sha = sha256(BUILD)
    if binary_sha != expected_binary_sha256:
        raise RuntimeError(f"release binary SHA mismatch: expected={expected_binary_sha256} actual={binary_sha}")
    if TARGET.exists() or TARGET.is_symlink():
        raise RuntimeError(f"refusing to overwrite external capture target: {TARGET}")
    return {
        "start_sha": EXPECTED_START,
        "code_sha": head,
        "branch": branch,
        "worktree_clean": True,
        "frozen_a3g_external_ledger_sha256": expected_ledger_sha,
        "frozen_a3g_external_artifact_count": len(verified_artifacts),
        "frozen_a3g_external_artifacts": verified_artifacts,
        "inputs": actual_inputs,
        "binary": {"path": str(BUILD), "bytes": BUILD.stat().st_size,
                   "sha256": binary_sha, "verified_against_explicit_build_sha": True},
    }


def frame_inventory(root: Path):
    inventory = []
    for path in sorted(p for p in root.rglob("*") if p.is_file()):
        if path.name == "MANIFEST.json":
            continue
        inventory.append({"path": str(path.relative_to(root)),
                          "bytes": path.stat().st_size, "sha256": sha256(path)})
    return inventory


def finalize_existing_capture(expected_code_sha: str, expected_binary_sha256: str):
    if not TARGET.is_dir():
        raise RuntimeError(f"existing capture directory missing: {TARGET}")
    if (TARGET / "MANIFEST.json").exists():
        raise RuntimeError("refusing to overwrite existing capture manifest")
    preflight = json.loads((TARGET / "PREFLIGHT.json").read_text())
    run_record = json.loads((TARGET / "run_result.json").read_text())
    if preflight.get("code_sha") != expected_code_sha:
        raise RuntimeError("existing capture CODE_SHA does not match requested identity")
    if preflight.get("binary", {}).get("sha256") != expected_binary_sha256:
        raise RuntimeError("existing capture release binary SHA mismatch")
    if (run_record.get("process_invocations") != 1 or run_record.get("retry_count") != 0 or
            run_record.get("process_exit_code") != 0 or not run_record.get("completed")):
        raise RuntimeError("existing capture does not attest exactly one successful replay")
    audit_script = PACKAGE / "scripts/p6_a3g_r3_prefix_audit.py"
    audit = subprocess.run([sys.executable, str(audit_script), "--capture-root", str(TARGET)],
                           cwd=REPO, check=False)
    if audit.returncode != 0:
        raise RuntimeError(f"offline capture/prefix parity audit failed ({audit.returncode}); no replay")
    parity = json.loads((TARGET / "PREFIX_PARITY.json").read_text())
    inventory = frame_inventory(TARGET)
    expected_tx = [159,160,163,165,166,173,174,176,182,183,185,187,188,190]
    actual_tx = sorted(int(p.name[2:]) for p in TARGET.glob("TX[0-9][0-9][0-9][0-9]"))
    if actual_tx != expected_tx:
        raise RuntimeError(f"captured transaction allowlist mismatch: {actual_tx}")
    manifest = {
        "task": "PAPER-P6-ALG-INTEGRATION-A3G-R3",
        "dataset": "SuperLoc Corridor01",
        "replay_type": "PREFIX_TO_TX190_LIDAR_SCAN_END",
        "real_replay_count": 1,
        "retry_count": 0,
        "gt_used": False,
        "production_mathematics_changed": False,
        "diagnostic_observer_only": True,
        "start_sha": EXPECTED_START,
        "code_sha": expected_code_sha,
        "prefix_audit_script_sha256": sha256(audit_script),
        "release_binary_sha256": preflight["binary"]["sha256"],
        "branch": BRANCH,
        "input_identity": preflight["inputs"],
        "frozen_a3g_artifact_ledger_sha256": preflight["frozen_a3g_external_ledger_sha256"],
        "frozen_a3g_artifact_count": preflight["frozen_a3g_external_artifact_count"],
        "frame_limit_includes_pre_handoff_scans": True,
        "initialization_stamp_ns": INIT_NS,
        "mode": preflight["mode"], "policy": preflight["policy"],
        "visual": "NONE", "visual_provenance": "NONE",
        "selected_transactions": expected_tx,
        "selected_capture_count": len(actual_tx),
        "prefix_parity": parity,
        "files_excluding_manifest": inventory,
        "payload_file_count": len(inventory),
        "payload_total_bytes_excluding_manifest": sum(x["bytes"] for x in inventory),
        "process_exit_code": run_record["process_exit_code"],
    }
    (TARGET / "MANIFEST.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(f"A3G_R3_SINGLE_PREFIX_CAPTURE_PASS root={TARGET} selected={len(actual_tx)} "
          f"manifest_sha256={sha256(TARGET / 'MANIFEST.json')}")
    return manifest


def execute(expected_code_sha: str, expected_binary_sha256: str):
    preflight = check_preflight(expected_code_sha, expected_binary_sha256)
    TARGET.mkdir()
    command = [
        "/usr/bin/time", "-v", "-o", str(TARGET / "resource.txt"), str(BUILD),
        "FULL_FIXED_LAG_V3_EXPERIMENTAL",
        str(DATA_ROOT / "results/p6_i6c_framework/input/imu.csv"),
        str(RAW_DIR / "filter_scans.csv"), str(RAW_DIR / "raw_timed_catalog.csv"),
        str(RAW_DIR / "raw_timed_points.bin"),
        str(DATA_ROOT / "map/derived/corridor01_map_normalized.pcd"), str(PARAMS),
        str(TARGET / "trajectory.csv"), str(TARGET / "events.csv"),
        str(TARGET / "runtime.csv"), "NONE", "190", str(INIT_NS), "corridor01",
        "ADAPTIVE_SELECTED_NIS", str(RAW_DIR / "raw_timed_catalog.csv"), "NONE",
    ]
    preflight.update({
        "dataset": "SuperLoc Corridor01",
        "mode": "FULL_FIXED_LAG_V3_EXPERIMENTAL",
        "policy": "ADAPTIVE_SELECTED_NIS",
        "visual": "NONE",
        "visual_provenance": "NONE",
        "frame_limit": 190,
        "initialization_stamp_ns": INIT_NS,
        "real_replay_count": 1,
        "retry_count": 0,
        "gt_used": False,
        "target": str(TARGET),
        "command": command,
        "environment": {
            "LD_LIBRARY_PATH": "/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu",
            "P6_A3B_R1_DIAGNOSTICS": "0",
            "P6_A3C_R1_MARGINALIZATION_DIAGNOSTICS": "0",
            "P6_A3F_R1_COVARIANCE_DIAGNOSTICS": "0",
            "P6_A3G_HEALTH_DIAGNOSTICS": "1",
            "P6_A3G_R3_CAPTURE_DIR": str(TARGET),
        },
    })
    (TARGET / "PREFLIGHT.json").write_text(json.dumps(preflight, indent=2, sort_keys=True) + "\n")
    env = os.environ.copy()
    env.update(preflight["environment"])
    # Exactly one estimator process call; no retry branch exists.
    with (TARGET / "console.txt").open("wb") as output:
        result = subprocess.run(command, cwd=REPO, env=env, stdout=output,
                                stderr=subprocess.STDOUT, check=False)
    run_record = {"process_exit_code": result.returncode, "process_invocations": 1,
                  "retry_count": 0, "completed": result.returncode == 0}
    (TARGET / "run_result.json").write_text(json.dumps(run_record, indent=2) + "\n")
    if result.returncode != 0:
        raise RuntimeError(f"single prefix replay failed with exit {result.returncode}; no retry")
    finalize_existing_capture(expected_code_sha, expected_binary_sha256)


def self_test():
    if len(EXPECTED_START) != 40 or len(INPUT_SHA) != 7:
        raise RuntimeError("A3G-R3 runner identity fixture malformed")
    if BRANCH != "research/p6-i6d-full-algorithm" or INIT_NS != 1517157224188979000:
        raise RuntimeError("A3G-R3 runner contract fixture mismatch")
    if len("a" * 64) != 64:
        raise RuntimeError("A3G-R3 binary identity fixture malformed")
    fixture = [{"path": "/frozen/raw.bin", "sha256": "a" * 64}]
    require_frozen_input_identity(fixture, "/frozen/raw.bin", "a" * 64)
    try:
        require_frozen_input_identity(fixture, "/frozen/other.bin", "a" * 64)
    except RuntimeError:
        pass
    else:
        raise RuntimeError("A3G-R3 frozen input identity self-test failed to reject mismatch")
    print("A3G_R3_CAPTURE_RUNNER_SELF_TEST_PASS")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--audit-existing", action="store_true")
    parser.add_argument("--expected-code-sha")
    parser.add_argument("--expected-binary-sha256")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return 0
    if sum((args.preflight_only, args.execute, args.audit_existing)) != 1:
        parser.error("choose exactly one of --preflight-only, --execute, or --audit-existing")
    if not args.expected_code_sha:
        parser.error("--expected-code-sha is required")
    if not args.expected_binary_sha256 or len(args.expected_binary_sha256) != 64:
        parser.error("--expected-binary-sha256 (64 hex chars) is required")
    if args.preflight_only:
        result = check_preflight(args.expected_code_sha, args.expected_binary_sha256)
        print(json.dumps({k: result[k] for k in (
            "start_sha", "code_sha", "branch", "worktree_clean",
            "frozen_a3g_external_ledger_sha256", "frozen_a3g_external_artifact_count",
            "inputs", "binary")}, indent=2))
        return 0
    if args.audit_existing:
        finalize_existing_capture(args.expected_code_sha, args.expected_binary_sha256)
        return 0
    execute(args.expected_code_sha, args.expected_binary_sha256)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"A3G_R3_RUNNER_STOP: {exc}", file=sys.stderr)
        raise SystemExit(2)
