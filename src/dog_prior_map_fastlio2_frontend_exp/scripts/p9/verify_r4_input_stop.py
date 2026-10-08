#!/usr/bin/env python3
"""Verify an archived input STOP; compile/self-test only, never run R4 search."""
import argparse
from collections import Counter
import csv
import io
import json
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import p9_r4_contract as c


ENV = dict(os.environ, PYTHONDONTWRITEBYTECODE="1", OPENBLAS_NUM_THREADS="1",
           OMP_NUM_THREADS="1", LD_LIBRARY_PATH="/lib/x86_64-linux-gnu")
VERIFY = c.OUT / "verification"


def logged(command, name, cwd=c.ROOT):
    path = VERIFY / name
    with path.open("w") as log:
        completed = subprocess.run(list(map(str, command)), cwd=cwd, env=ENV,
                                   stdout=log, stderr=subprocess.STDOUT)
    c.require(completed.returncode == 0, "verification failed: " + str(path))
    raw_sha = c.digest(path)
    # CTest's empty Site field includes trailing spaces; normalize archive text
    # without altering status/content, preserving the exact command-output hash.
    path.write_text("".join(line.rstrip()+"\n" for line in path.read_text().splitlines()))
    print("R4_VERIFICATION_PASS", name, flush=True)
    return dict(command=list(map(str, command)), cwd=str(cwd), exit_code=0,
                raw_command_output_sha256=raw_sha, trailing_whitespace_normalized=True,
                log_sha256=c.digest(path))


def assert_hashes(mapping, relative=False):
    for name, sha in mapping.items():
        c.require(c.digest(c.OUT / name if relative else name) == sha,
                  "frozen hash changed: " + name)


def audit(build):
    result = json.loads((c.OUT / "results.json").read_text())
    c.require(result == json.loads((c.OUT / "input_stop_freeze.json").read_text()),
              "stop results/receipt divergence")
    c.require(result["state"] == "STOPPED_BEFORE_FIRST_ALIGNMENT" and
              result["scientific_final_result"] is None and result["final_cohort_prefix"] is None,
              "input STOP falsely classified as scientific outcome")
    c.require(result["oracle_calls"] == result["candidate_calls"] == result["visual_pairs_extracted"] == 0 and
              not result["gt_loaded"] and not result["evidence_oracle_labels_loaded"],
              "research execution was not stopped")
    c.require(not list((c.OUT / "oracle/runs").glob("*")), "unexpected NDT execution artifacts")
    for name in ("cohort_selection_freeze.json", "heldout_final_cohort.csv", "conditioned_weak_runs.csv",
                 "candidate_runs", "candidate_engine.log", "candidate_freeze.json", "probe_manifest.csv",
                 "candidate_source_manifest.csv", "visual_pair_manifest.csv", "visual_measurement_freeze.json",
                 "visual_measurements.csv", "candidate_residuals.csv", "nonoracle_evidence.csv", "evidence_freeze.json"):
        c.require(not (c.OUT / name).exists(), "unexpected unexecuted-stage output: " + name)
    c.require(not (c.OUT / "oracle/oracle_labels.csv").exists(), "unexpected held-out labels")
    log = (c.OUT / "oracle/engine_96.log").read_text()
    c.require("source hash/count mismatch" in log, "original input STOP log missing")
    selection = json.loads((c.OUT / "selection_freeze.json").read_text())
    assert_hashes(selection["artifacts"], True)
    assert_hashes(selection["input_sha256"])
    assert_hashes(selection["construction_source_sha256"])
    pool = c.read_csv(c.OUT / "heldout_ordered_pool.csv")
    expected, eligible = c.ordered_pool(int(r["transaction_id"]) for r in c.read_csv(c.REGISTRATION))
    c.require(pool == [{key: str(value) for key, value in row.items()} for row in expected] and
              eligible == selection["eligible_count"] == 3630 and len(pool) == 160,
              "selection CSV/receipt/algorithm disagreement")
    ids = [int(r["transaction_id"]) for r in pool]
    minimum = min(abs(a-b) for i, a in enumerate(ids) for b in ids[i+1:])
    c.require(minimum >= 16, "target separation violated")
    prepared = json.loads((c.OUT / "execution_manifest.json").read_text())
    for key in ("input_sha256", "code_sha256", "raw_source_sha256"):
        assert_hashes(prepared[key])
    assert_hashes(prepared["artifacts"], True)
    c.require(c.digest(prepared["binary"]) == prepared["binary_sha256"] and
              c.digest(prepared["oracle_library"]) == prepared["oracle_library_sha256"],
              "prepared Release binary/library changed")
    parity = json.loads((c.OUT / "oracle_parity_freeze.json").read_text())
    assert_hashes(parity["input_sha256"])
    assert_hashes(parity["artifacts"], True)
    c.require(parity["status"] == "PASS" and parity["frames"] == 24 and parity["major"] == 9 and
              parity["no_major"] == 15 and parity["major_ids"] == 22 and
              parity["frozen_geometry_library_sha256"] == prepared["oracle_library_sha256"],
              "historical oracle label parity receipt differs")
    labels = c.read_csv(c.OUT / "oracle_parity.csv")
    c.require(len(labels) == 24 and all(r["parity"] == "PASS" for r in labels),
              "historical parity CSV differs")
    c.require(sum(r["label"]=="MAJOR" for r in labels)==9 and
              sum(r["label"]=="NO_MAJOR" for r in labels)==15 and
              sum(int(r["major_count"]) for r in labels)==22 and
              all(r["major_cluster_ids"]==r["expected_major_ids"] for r in labels),
              "historical label/major ID CSV counts differ")
    seeds = c.read_csv(c.OUT / "original_seed_parity.csv")
    c.require(len(seeds) == 8416 and all(r["parity"] == "PASS" for r in seeds),
              "seed parity CSV differs")
    c.require(Counter(int(r["frame"]) for r in seeds) == Counter({tx:263 for tx in c.DEVELOPMENT}),
              "historical seed frame coverage differs")
    sources = c.read_csv(c.OUT / "heldout_source_audit.csv")
    history = c.read_csv(c.OUT / "historical_source_audit.csv")
    assert_hashes(result["diagnostic_sha256"], True)
    c.require(c.digest(build / "p9_r4_source_audit") == result["source_auditor_sha256"],
              "read-only source auditor changed; refuse execution")
    for rows, manifest_path in ((sources,c.OUT/"heldout_source_manifest.csv"),
                                (history,c.ARCHIVE/"frozen/cohort_frozen.csv")):
        manifest_rows=c.read_csv(manifest_path)
        c.require([r["transaction_id"] for r in rows]==[r["transaction_id"] for r in manifest_rows],
                  "source diagnosis identity/order differs from input manifest")
        output=subprocess.check_output([str(build/"p9_r4_source_audit"),str(manifest_path)],
                                       text=True,env=ENV)
        c.require(list(csv.DictReader(io.StringIO(output)))==rows,
                  "read-only source diagnosis re-execution differs from frozen CSV")
        for row, original in zip(rows,manifest_rows):
            numerical_equal=(int(row["expected_points"])==int(row["actual_points"]) and
                             int(row["expected_hash"])==int(row["actual_hash"]))
            c.require(row["parity"]==("PASS" if numerical_equal else "FAIL") and
                      row["expected_hash"]==original["prepared_source_hash"] and
                      row["expected_points"]==original["prepared_source_point_count"] and
                      c.digest(original["raw_cloud_file"])==original["raw_source_sha256"],
                      "source diagnostic flags/counts/hashes do not match actual input")
    c.require([int(r["transaction_id"]) for r in sources] == ids and len(history) == 32 and
              all(r["parity"] == "PASS" for r in history) and all(r["parity"] == "FAIL" for r in sources),
              "source diagnosis CSV coverage/count disagreement")
    c.require(result["first_frame"] == sources[0] and
              result["heldout_sources"] == dict(total=160, passed=0, failed=160) and
              result["historical_sources"] == dict(total=32, passed=32),
              "source CSV/JSON disagreement")
    raw = c.read_csv(c.OUT / "topic_vs_same_objective_raw_parity.csv")
    c.require(len(raw) == 6 and all(r["exact_raw_parity"] == "0" for r in raw),
              "raw source diagnosis CSV differs")
    originals={r["transaction_id"]:r for r in c.read_csv(c.ARCHIVE/"frozen/cohort_frozen.csv")}
    for row in raw:
        c.require(row["archived_raw_sha256"]==originals[row["transaction_id"]]["raw_source_sha256"] and
                  int(row["exact_raw_parity"])==int(row["archived_raw_sha256"]==row["topic_bag_decoded_raw_sha256"]),
                  "raw diagnostic parity flag/hash disagreement")
    proposals = Counter(int(r["frame"]) for r in c.read_csv(c.OUT / "conditioned_proposal_pool.csv"))
    c.require(proposals == Counter({tx:263 for tx in ids}), "held-out proposal geometry coverage differs")
    c.require(len(c.read_csv(c.OUT / "oracle/batch_96.csv")) == 96,
              "attempted pre-alignment batch receipt differs")
    c.require(c.digest(c.OUT / "frozen_p7_source_export_reference.txt") == result["frozen_runner_source"]["sha256"],
              "frozen source export code changed")
    from run_r4_evidence import frontend_guard
    frontend, environment = frontend_guard()
    # Source immutability includes production/frontends; new edits are offline R4 tooling only.
    frozen = {str(c.HERE/name): c.pinned(c.HERE/name) for name in
              ("p9_ndt_energy_contract.cpp", "p9_r2a_predictor_search.cpp", "p9_r3b_pnp_frontend.py",
               "p9_r3a_depth_completion.py", "p9_r3a_visual_frontend.py")}
    return dict(status="PASS", meaning="archive integrity and exact reproduction of the INPUT FAIL, not source admission PASS",
                eligible_count=eligible, ordered_pool_count=len(ids), minimum_target_separation=minimum,
                historical_label_parity=dict(frames=24,major=9,no_major=15,major_ids=22),
                original_seed_rows=8416, heldout_proposal_rows=sum(proposals.values()),
                historical_sources_pass=32, heldout_sources_fail=160, raw_pairs_fail=6,
                NEW_NDT_CALLS=0, GT_LOADED=False, frontend_source_sha256=frontend,
                frontend_numerical_environment=environment, frozen_offline_dependencies=frozen)


def freeze_hash_manifest(build):
    files = {}
    for path in sorted(c.OUT.rglob("*")):
        if path.is_file() and path.name != "artifact_hashes.json":
            c.require(path.stat().st_size < 100_000_000, "artifact exceeds Git hosting size limit")
            files[path.relative_to(c.ROOT).as_posix()] = c.digest(path)
    sources = {path.relative_to(c.ROOT).as_posix(): c.digest(path) for path in sorted(c.HERE.glob("*r4*")) if path.is_file()}
    sources[(c.HERE/"CMakeLists.txt").relative_to(c.ROOT).as_posix()] = c.digest(c.HERE/"CMakeLists.txt")
    binaries = {str(build/name): c.digest(build/name) for name in
                ("p9_r4_ndt", "p9_r4_source_audit", "libp9_r4_oracle_geometry.so", "libp9_r2b_chart.so")}
    receipt = dict(files=files, source_sha256=sources, binary_sha256=binaries,
                   scientific_execution="STOPPED_BEFORE_FIRST_ALIGNMENT", GT_LOADED=False, NEW_NDT_CALLS=0)
    c.save_json(c.OUT / "artifact_hashes.json", receipt)
    for path, sha in files.items(): c.require(c.digest(c.ROOT/path) == sha, "archive hash verification failed")
    return dict(artifact_count=len(files), source_count=len(sources), status="PASS")


def verify(build):
    VERIFY.mkdir(exist_ok=True)
    commands = []
    commands.append(logged(["cmake", "-S", c.HERE, "-B", build, "-DCMAKE_BUILD_TYPE=Release"], "release_configure.log"))
    commands.append(logged(["cmake", "--build", build, "-j", "2"], "release_build.log"))
    commands.append(logged(["ctest", "--output-on-failure", "-T", "Test"], "p9_ctest.log", build))
    tag = (build / "Testing/TAG").read_text().splitlines()[0]
    xml = ET.parse(build / "Testing" / tag / "Test.xml")
    tests = xml.findall(".//Testing/Test")
    c.require(len(tests) == 38 and all(t.get("Status") == "passed" for t in tests), "P9/R4 self-tests incomplete")
    for source, name in ((build/"Testing"/tag/"Test.xml","ctest.xml"),
                         (build/"Testing/Temporary/LastTest.log","ctest_detail.log")):
        (VERIFY/name).write_bytes(source.read_bytes())
    commands.append(logged(["git", "diff", "--check"], "git_diff_check.log"))
    audited = audit(build)
    c.save_json(VERIFY / "audit.json", audited)
    c.save_json(VERIFY / "tests.json", dict(build_type="Release", test_count=len(tests),
                passed=len(tests), commands=commands, NEW_NDT_CALLS=0, GT_LOADED=False))
    hashed = freeze_hash_manifest(build)
    print(json.dumps(dict(Release="PASS", tests="38/38", archive_audit=audited["status"],
                          hashes=hashed, scientific_execution="NOT_RUN_INPUT_BLOCKER"), indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", type=Path, required=True)
    verify(parser.parse_args().build)
