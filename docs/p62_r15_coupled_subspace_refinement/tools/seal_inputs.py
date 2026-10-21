#!/usr/bin/env python3
"""Freeze/recheck R15 inputs and protected production state, without parsing GT poses."""
import csv
import hashlib
import json
import os
import pathlib
import subprocess
import sys
from datetime import datetime, timezone

ROOT = pathlib.Path(__file__).resolve().parents[3]
ARCHIVE = ROOT / "docs/p62_r15_coupled_subspace_refinement"
R13 = ROOT / "docs/p62_r13_training_guided_multihypothesis"
R14 = ROOT / "docs/p62_r14b_newer_college/conditional_base_pilot"
STABLE = pathlib.Path("/home/jian/livox_ws/dog_visual_loc_ws")
R13_EVENTS = R13 / "EVENT_INPUT_MANIFEST.json"
R14_PREREG = R14 / "TEST_FRAME_PREREGISTRATION.json"
MAP13 = pathlib.Path("/home/jian/rosbag/loop2/loop2mapping/pcd/loop2_simtime_rebuild_2026_08_12_001_all_downsampled_points.pcd")
MAP14 = pathlib.Path("/home/jian/rosbag/public_datasets/newer_college_r14b/conditional_base_pilot_20261021/map/leica_roi50_leaf020.pcd")
BUILD = pathlib.Path("/tmp/p62_r15_build")
EXPECTED_HEAD = "fae8937cd72e8272111186e572c346bb76f92a4e"


def run(*args, cwd=None):
    return subprocess.check_output(args, cwd=cwd, stderr=subprocess.STDOUT).decode()


def sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def identity(path):
    p = pathlib.Path(path)
    if not p.is_file():
        raise RuntimeError(f"required file missing: {p}")
    return {"path": str(p), "bytes": p.stat().st_size, "sha256": sha(p)}


def verify_checksums(archive):
    checksum_file = archive / "checksums.sha256"
    records = []
    for line in checksum_file.read_text().splitlines():
        if not line.strip():
            continue
        expected, rel = line.split(None, 1)
        rel = rel.lstrip(" *")
        path = archive / rel
        actual = sha(path)
        if actual != expected:
            raise RuntimeError(f"archive checksum mismatch: {path}")
        records.append({"path": str(path), "bytes": path.stat().st_size, "sha256": actual})
    return records


def stable_snapshot():
    if not STABLE.is_dir():
        raise RuntimeError(f"stable worktree missing: {STABLE}")
    git = ["git", "-C", str(STABLE)]
    head = run(*git, "rev-parse", "HEAD").strip()
    branch = run(*git, "branch", "--show-current").strip()
    status = run(*git, "status", "--porcelain=v2", "--branch", "--untracked-files=all")
    refs = run(*git, "for-each-ref", "--format=%(objectname) %(refname)")
    reflog = run(*git, "reflog", "show", "--all", "--date=raw", "--format=%H %gD %gs")
    index_path = pathlib.Path(run(*git, "rev-parse", "--git-path", "index").strip())
    if not index_path.is_absolute():
        index_path = STABLE / index_path
    diff = subprocess.check_output(git + ["diff", "--binary", "HEAD"])
    all_files = {}
    for base, dirs, files in os.walk(STABLE, followlinks=False):
        dirs[:] = [d for d in dirs if d != ".git"]
        for name in files:
            path = pathlib.Path(base) / name
            rel = str(path.relative_to(STABLE))
            if path.is_symlink():
                all_files[rel] = {"symlink_target": os.readlink(path)}
            else:
                all_files[rel] = {"bytes": path.stat().st_size, "sha256": sha(path)}
    return {
        "path": str(STABLE), "head": head, "branch": branch,
        "index_sha256": sha(index_path), "status_sha256": hashlib.sha256(status.encode()).hexdigest(),
        "refs_sha256": hashlib.sha256(refs.encode()).hexdigest(),
        "reflog_sha256": hashlib.sha256(reflog.encode()).hexdigest(),
        "status_lines": len(status.splitlines()), "diff_head_sha256": hashlib.sha256(diff).hexdigest(),
        "filesystem_file_count": len(all_files), "filesystem_files_sha256": hashlib.sha256(
            json.dumps(all_files, sort_keys=True, separators=(",", ":")).encode()).hexdigest(),
        "filesystem_files": all_files,
    }


def input_snapshot(require_run_dir):
    if run("git", "-C", str(ROOT), "rev-parse", "HEAD").strip() != EXPECTED_HEAD:
        raise RuntimeError("research HEAD differs from frozen R14B END_SHA")
    if run("git", "-C", str(ROOT), "branch", "--show-current").strip() != "research/p62-r15-coupled-subspace-refinement-r1":
        raise RuntimeError("unexpected research branch")
    run_dir = ROOT / "docs/p62_r15_coupled_subspace_refinement/run_01"
    if require_run_dir and not run_dir.exists():
        raise RuntimeError("run_01 is missing during after-run asset verification")
    if not require_run_dir and run_dir.exists():
        raise RuntimeError("run_01 already exists before source/build freeze")

    r13_obj = json.loads(R13_EVENTS.read_text())
    r14_obj = json.loads(R14_PREREG.read_text())
    expected = {}

    def add(path, want=None):
        current = identity(path)
        if want is not None and current["sha256"] != want:
            raise RuntimeError(f"identity mismatch: {path}")
        expected[current["path"]] = current

    # Frozen R13 archive records and every provenance input named by its manifest.
    r13_archive_files = verify_checksums(R13)
    for entry in r13_obj["inputs"]:
        add(entry["path"], entry["sha256"])
    add(R13_EVENTS)
    for e in r13_obj["events"]:
        add(e["raw_xyz_offset_file"], e["raw_sha256"])
    add(MAP13, "016b7ae3ae494ad5d462a66d888a96b96dda5fe29d6058059aa2fa6bdf6b09ca")

    # Frozen R14 archive records, 20 evaluation PCDs plus anchor PCD, map, and
    # GT/calibration identities. The registered CSV is SHA-checked only here;
    # no pose values are read by this script.
    r14_archive_files = verify_checksums(R14)
    add(R14_PREREG)
    for entry in r14_obj["source_inputs"]:
        add(entry["path"], entry["sha256"])
    for frame in r14_obj["frames"]:
        add(frame["path"], frame["sha256"])
    map_obj = json.loads((R14 / "PRIOR_MAP_MANIFEST.json").read_text())
    add(map_obj["map_path"], map_obj["map_sha256"])

    code_names = ["tools/CMakeLists.txt", "tools/coupled.hpp", "tools/run_coupled.cpp",
                  "tools/test_coupled.cpp", "tools/freeze_outputs.py", "tools/evaluate_reference.py",
                  "tools/seal_inputs.py", "MATHEMATICAL_FORMULATION.md"]
    code = {str(ARCHIVE / name): identity(ARCHIVE / name) for name in code_names}
    for name in ("run_coupled", "test_coupled"):
        code[str(BUILD / name)] = identity(BUILD / name)
    cache = BUILD / "CMakeCache.txt"
    if cache.is_file():
        code[str(cache)] = identity(cache)

    return {
        "research_git": {"head": EXPECTED_HEAD, "branch": "research/p62-r15-coupled-subspace-refinement-r1"},
        "r13_archive_checksums": r13_archive_files,
        "r14b_archive_checksums": r14_archive_files,
        "file_identities": list(expected.values()),
        "source_and_binary_identities": list(code.values()),
        "stable_worktree": stable_snapshot(),
    }


def write_exclusive(path, obj):
    with open(path, "x") as f:
        json.dump(obj, f, indent=2, sort_keys=True)
        f.write("\n")


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ("before", "after"):
        raise RuntimeError("usage: seal_inputs.py before|after")
    phase = sys.argv[1]
    before_path = ARCHIVE / "INPUT_FREEZE.json"
    before_record = ARCHIVE / "ASSET_IDENTITY_BEFORE.json"
    after_path = ARCHIVE / "ASSET_IDENTITY_AFTER.json"
    if phase == "before":
        if before_path.exists() or before_record.exists() or after_path.exists():
            raise RuntimeError("input freeze already exists; refusing overwrite")
        if (ARCHIVE / "run_01").exists():
            raise RuntimeError("formal run directory already exists")
        current = input_snapshot(require_run_dir=False)
        payload = {"task": "P62-R15-COUPLED-SUBSPACE-REFINEMENT-R1",
                   "created_utc": datetime.now(timezone.utc).isoformat(),
                   "phase": "BEFORE_FORMAL_LOCAL_MODEL_RUN",
                   "reference_pose_values_parsed": False,
                   "raw_reference_file_read_for_identity_only": True,
                   "inputs": current}
        write_exclusive(before_path, payload)
        write_exclusive(before_record, payload)
        print(json.dumps({"phase": phase, "files": len(current["file_identities"]),
                          "stable_files": current["stable_worktree"]["filesystem_file_count"]}, indent=2))
    else:
        if not before_path.is_file() or after_path.exists():
            raise RuntimeError("missing pre-run freeze or after snapshot already exists")
        before = json.loads(before_path.read_text())
        current = input_snapshot(require_run_dir=True)
        if current != before["inputs"]:
            # Give a compact diff while preserving all failure evidence.
            old = {x["path"]: x["sha256"] for x in before["inputs"]["file_identities"]}
            new = {x["path"]: x["sha256"] for x in current["file_identities"]}
            diff = {p: {"before": old.get(p), "after": new.get(p)} for p in sorted(set(old)|set(new)) if old.get(p) != new.get(p)}
            raise RuntimeError(f"asset identity drift after run: {json.dumps(diff, sort_keys=True)}")
        payload = {"task": "P62-R15-COUPLED-SUBSPACE-REFINEMENT-R1",
                   "created_utc": datetime.now(timezone.utc).isoformat(),
                   "phase": "AFTER_FORMAL_LOCAL_MODEL_AND_REFERENCE_EVALUATION",
                   "matches_before": True, "inputs": current}
        write_exclusive(after_path, payload)
        print(json.dumps({"phase": phase, "matches_before": True,
                          "stable_files": current["stable_worktree"]["filesystem_file_count"]}, indent=2))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"INPUT_SEAL_FAILED: {exc}", file=sys.stderr)
        raise
