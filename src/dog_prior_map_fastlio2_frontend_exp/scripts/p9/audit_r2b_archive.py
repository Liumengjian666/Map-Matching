#!/usr/bin/env python3
"""Freeze/verify final R2B artifacts, without opening a map or running an optimizer."""
import argparse
import json
from pathlib import Path

import evaluate_r2b_nonoracle_evidence as evaluation
import p9_r2b_nonoracle_evidence as blind


def freeze():
    evaluation.audit_results()
    output=blind.OUT/"artifact_hashes.json"
    blind.require(not output.exists(),"final artifact manifest already frozen")
    cache=Path("/tmp/p9_r2_build/CMakeCache.txt").read_text()
    blind.require("CMAKE_BUILD_TYPE:STRING=Release" in cache,"not a Release build")
    test_path=Path("/tmp/p9_r2_build/Testing/Temporary/LastTest.log")
    test_log=test_path.read_text()
    blind.require(test_log.count("Test Passed.")==25 and "Test Failed." not in test_log,"not a complete 25/25 P9 test run")
    artifacts={p.name:blind.digest(p) for p in sorted(blind.OUT.iterdir()) if p.is_file() and p!=output}
    sources=[blind.HERE/name for name in ("CMakeLists.txt","p9_r2b_chart.cpp","p9_r2b_nonoracle_evidence.py",
                                         "evaluate_r2b_nonoracle_evidence.py","audit_r2b_archive.py")]
    metadata=dict(artifact_sha256=artifacts,source_sha256={str(p):blind.digest(p) for p in sources},
        binary_sha256={str(blind.LIBRARY):blind.digest(blind.LIBRARY)},
        cmake_cache_sha256=blind.digest("/tmp/p9_r2_build/CMakeCache.txt"),
        verification=dict(release_build="PASS",p9_tests="25/25 PASS",test_log=str(test_path),
            test_log_sha256=blind.digest(test_path),csv_json_hash_audit="PASS",NEW_NDT_CALLS=0,
            GT_USED_FOR_EVIDENCE=False,GT_USED_FOR_GATE=False))
    output.write_text(json.dumps(metadata,indent=2,sort_keys=True,allow_nan=False)+"\n")
    print("P9_R2B_FINAL_ARTIFACT_HASH_MANIFEST=FROZEN")


def audit():
    evaluation.audit_results()
    metadata=json.loads((blind.OUT/"artifact_hashes.json").read_text())
    expected=set(metadata["artifact_sha256"])
    actual={p.name for p in blind.OUT.iterdir() if p.is_file() and p.name!="artifact_hashes.json"}
    blind.require(expected==actual,"unhashed or missing final archive artifact")
    for name,h in metadata["artifact_sha256"].items():
        blind.require(blind.digest(blind.OUT/name)==h,"final artifact changed: "+name)
    for category in ("source_sha256","binary_sha256"):
        for path,h in metadata[category].items():blind.require(blind.digest(path)==h,"source/binary changed: "+path)
    initial_path=blind.OUT/"evaluation_snapshot_initial.json"
    latest=json.loads((blind.OUT/"evaluation_snapshot.json").read_text())
    initial=json.loads(initial_path.read_text());revision=latest["source_revision"]
    blind.require(revision["previous_snapshot_sha256"]==blind.digest(initial_path) and
                  revision["previous_source_sha256"]==initial["evaluation_source_sha256"],"source revision lineage invalid")
    for key,value in initial.items():
        if key!="evaluation_source_sha256":blind.require(latest[key]==value,"code-only source revision changed statistics/provenance")
    print("P9_R2B_FINAL_ARCHIVE_AUDIT=PASS")


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("stage",choices=("freeze","audit"))
    args=parser.parse_args();{"freeze":freeze,"audit":audit}[args.stage]()
