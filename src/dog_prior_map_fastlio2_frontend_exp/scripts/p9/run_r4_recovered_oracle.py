#!/usr/bin/env python3
"""Resume frozen BASE263 on admitted recovered sources; labels only choose prefix size."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import p9_r4_contract as c
import p9_r4_recovered_inputs as inputs
from p9_r4_oracle_labels import FrozenGeometry, extract

ORACLE = c.OUT / "oracle"
WORKERS = 4  # Fixed pre-experiment process parallelism; each frame retains seed0..262 order.
PROTECTED = ("heldout_source_manifest.csv", "heldout_source_audit.csv",
             "topic_vs_same_objective_raw_parity.csv", "input_stop_freeze.json",
             "oracle/engine_96.log", "execution_manifest.json", "source_recovery/results.json",
             "source_recovery/heldout_source_manifest_recovered.csv", "oracle_parity_freeze.json")


def run_batch(commands):
    """Own worker lifetimes: any worker failure stops the entire scientific stage."""
    processes, logs = [], []
    try:
        for command, path in commands:
            log = path.open("x")
            logs.append(log)
            processes.append(subprocess.Popen(list(map(str, command)), cwd=c.ROOT, env=inputs.ENV,
                                              stdout=log, stderr=subprocess.STDOUT))
        while True:
            codes = [p.poll() for p in processes]
            c.require(not any(code is not None and code != 0 for code in codes),
                      "oracle engine failure: stopping ALL workers; preserve started/partial/log receipts")
            if all(code == 0 for code in codes):
                return
            time.sleep(.2)
    finally:
        for process in processes:
            if process.poll() is None:
                process.terminate()
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()
        for log in logs:
            log.close()


def self_test():
    inputs.self_test()
    with tempfile.TemporaryDirectory(prefix="p9_r4_worker_stop_test.") as directory:
        commands = [([sys.executable, "-c", "raise SystemExit(7)"], Path(directory)/"fail.log"),
                    ([sys.executable, "-c", "import time; time.sleep(30)"], Path(directory)/"sibling.log")]
        tick = time.perf_counter()
        try:
            run_batch(commands)
        except RuntimeError:
            c.require(time.perf_counter()-tick < 10, "failed worker did not stop sibling promptly")
        else:
            raise RuntimeError("failed worker was accepted")
        c.require(all(path.is_file() for _, path in commands), "failure logs were lost")
    print("P9_R4_ORACLE_FAIL_FAST_NO_RETRY_SELF_TEST=PASS")


def prepare():
    c.require(not inputs.MANIFEST.exists(), "recovered execution already frozen")
    branch = subprocess.check_output(["git", "branch", "--show-current"], cwd=c.ROOT, text=True).strip()
    c.require(branch == c.BRANCH, "incorrect branch")
    subprocess.run(["git", "merge-base", "--is-ancestor", inputs.CLOSURE_SHA, "HEAD"], cwd=c.ROOT, check=True)
    old = json.loads((c.OUT / "execution_manifest.json").read_text())
    for name in PROTECTED:
        c.pinned(c.OUT / name, inputs.CLOSURE_SHA)
    for group in ("input_sha256", "code_sha256"):
        for path, sha in old[group].items():
            c.require(c.digest(path) == sha, "frozen input/source changed: " + path)
    for name, sha in old["artifacts"].items():
        c.require(c.digest(c.OUT / name) == sha, "frozen prerequisite changed: " + name)
    parity = json.loads((c.OUT / "oracle_parity_freeze.json").read_text())
    c.require(parity["status"] == "PASS" and
              (parity["frames"], parity["major"], parity["no_major"], parity["major_ids"]) == (24, 9, 15, 22),
              "historical oracle parity prerequisite incomplete")
    c.require(old["original_seed_parity"]["original_seed_parity"] == "PASS" and
              old["original_seed_parity"]["candidate_generator_parity"] == "PASS", "proposal prerequisite failed")
    for path, sha in parity["input_sha256"].items():
        c.require(c.digest(path) == sha, "historical parity input changed")
    c.require(c.digest(old["binary"]) == old["binary_sha256"] and
              c.digest(old["oracle_library"]) == parity["frozen_geometry_library_sha256"] and
              c.digest(c.HERE / "p9_r4_oracle_labels.py") == parity["extractor_sha256"], "frozen carriers changed")
    rows, audit = inputs.cache_gate(Path(old["binary"]).parent / "p9_r4_source_audit")
    c.write_csv(c.OUT / "cache_pre_gate.csv", audit)
    receipt = dict(old)
    receipt.update(start_sha=inputs.CLOSURE_SHA, source_closure_commit=inputs.CLOSURE_SHA,
        SOURCE_PROVENANCE="RECOVERED_SAME_OBJECTIVE_SCAN_END_DESKEW", state="RECOVERED_SOURCE_ADMITTED",
        protected_receipts_sha256={str(c.OUT / name): c.digest(c.OUT / name) for name in PROTECTED},
        raw_source_sha256={r["raw_cloud_file"]: r["raw_source_sha256"] for r in rows},
        closure_raw_source_sha256=json.loads((inputs.RECOVERY / "results.json").read_text())["raw_source_sha256"],
        artifacts={name: sha for name, sha in old["artifacts"].items() if name != "heldout_source_manifest.csv"},
        oracle_workers=WORKERS, historical_ndt_calls_repeated=0, baseline_replay_calls=0)
    receipt["artifacts"].update({str(inputs.SOURCE_MANIFEST.relative_to(c.OUT)): c.digest(inputs.SOURCE_MANIFEST),
                                 "cache_pre_gate.csv": c.digest(c.OUT / "cache_pre_gate.csv")})
    receipt["code_sha256"].update({str(c.HERE / name): c.digest(c.HERE / name)
                                  for name in ("p9_r4_recovered_inputs.py", "run_r4_recovered_oracle.py")})
    c.save_json(inputs.MANIFEST, receipt)
    inputs.verify()
    print("RECOVERED_EXECUTION_PRE_GATE=PASS heldout_raw160 prepared160 ALL_RAW192 NEW_NDT_CALLS=0", flush=True)


def seal_pre_align():
    """Retain the initial preflight receipt when review changes orchestration before any align."""
    c.require(not (ORACLE / "recovered_runs").exists() and
              not (c.OUT / "cohort_selection_freeze.json").exists(), "reseal forbidden after oracle start")
    m = json.loads(inputs.MANIFEST.read_text())
    previous = c.OUT / "execution_manifest_recovered_preflight_v1.json"
    c.require(not previous.exists(), "pre-align reseal already performed")
    c.save_json(previous, m)
    path = str(c.HERE / "run_r4_recovered_oracle.py")
    changes = dict(path=path, previous_sha256=m["code_sha256"][path], sha256=c.digest(path),
                   reason="pre-align independent review: fail-fast worker stop and regression test",
                   previous_manifest_sha256=c.digest(previous), ndt_calls_started=0)
    m["code_sha256"][path] = changes["sha256"]
    m["pre_align_review_reseal"] = changes
    inputs.cache_gate(Path(m["binary"]).parent / "p9_r4_source_audit")
    c.save_json(inputs.MANIFEST, m)
    inputs.verify()
    print("RECOVERED_EXECUTION_RESEALED_BEFORE_ANY_ALIGN=PASS", flush=True)


def run_prefix():
    m = inputs.verify()
    c.require(not (c.OUT / "cohort_selection_freeze.json").exists(), "final cohort already frozen")
    directory = ORACLE / "recovered_runs"
    c.require(not directory.exists(), "recovered oracle already started; refusing repeat")
    directory.mkdir()
    frames = c.read_csv(inputs.SOURCE_MANIFEST)
    geometry = FrozenGeometry(Path(m["oracle_library"]))
    labels, clusters, candidates, cost = [], [], [], []
    previous = 0
    for prefix in c.PREFIXES:
        inputs.verify()
        batch = frames[previous:prefix]
        tick = time.perf_counter()
        commands = []
        for worker in range(WORKERS):
            path = ORACLE / f"recovered_batch_{prefix}_worker_{worker}.csv"
            c.require(not path.exists(), "oracle batch previously started")
            c.write_csv(path, batch[worker::WORKERS])
            commands.append(([m["binary"], "--oracle", c.MAP, path,
                              c.OUT / "conditioned_proposal_pool.csv", directory],
                             ORACLE / f"recovered_engine_{prefix}_worker_{worker}.log"))
        # No repeats after process/input failure; started/partial files are durable receipts.
        run_batch(commands)
        for frame in batch:
            tx = int(frame["transaction_id"])
            rows = c.read_csv(directory / f"tx_{tx}.csv")
            c.require(len(rows) == 263 and [int(r["seed_index"]) for r in rows] == list(range(263)) and
                      all(r["source_hash_actual"] == frame["prepared_source_hash"] and
                          r["source_points"] == frame["prepared_source_point_count"] and
                          r["target_points"] == frame["target_point_count"] for r in rows), "oracle shard identity/source fail")
            primary, label = extract(frame, rows, frame["raw_terminal_pose_xyz_q_xyzw"], geometry)
            candidates.extend(rows); clusters.extend(primary); labels.append(label)
            print("RECOVERED_ORACLE_LABEL", tx, label["label"], label["major_count"], flush=True)
        major = sum(r["label"] == "MAJOR" for r in labels)
        cost.append(dict(prefix=prefix, major=major, no_major=prefix-major, new_calls=len(batch)*263,
                         wall_seconds=time.perf_counter()-tick, workers=WORKERS))
        for name, rows in (("oracle_candidates.csv", candidates), ("oracle_clusters.csv", clusters), ("oracle_labels.csv", labels)):
            c.write_csv(ORACLE / name, rows)
        c.write_csv(ORACLE / "prefix_counts.csv", cost)
        sufficient = major >= 12 and prefix-major >= 40
        if sufficient or prefix == 160:
            c.write_csv(c.OUT / "heldout_final_cohort.csv", [dict(transaction_id=f["transaction_id"]) for f in frames[:prefix]])
            c.save_json(c.OUT / "cohort_selection_freeze.json", dict(final_prefix=prefix, major=major,
                no_major=prefix-major, sufficient=sufficient, selection="first sufficient96/128/160 LABEL COUNTS ONLY",
                visual_evidence_loaded=False, gt_loaded=False, source_provenance=m["SOURCE_PROVENANCE"],
                final_cohort_sha256=c.digest(c.OUT / "heldout_final_cohort.csv"),
                oracle_artifact_sha256={name: c.digest(ORACLE / name) for name in
                                       ("oracle_candidates.csv", "oracle_clusters.csv", "oracle_labels.csv", "prefix_counts.csv")},
                oracle_calls=prefix*263, oracle_wall_seconds=sum(r["wall_seconds"] for r in cost), workers=WORKERS))
            print("RECOVERED_ORACLE_PREFIX_FROZEN", prefix, major, prefix-major, sufficient, flush=True)
            return
        previous = prefix


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("stage", choices=("prepare", "seal-pre-align", "run", "self-test"))
    stage = p.parse_args().stage
    if stage == "prepare": prepare()
    elif stage == "seal-pre-align": seal_pre_align()
    elif stage == "run": run_prefix()
    else: self_test()
