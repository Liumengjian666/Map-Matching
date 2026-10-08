"""Recovered-source admission and immutable resume receipt; no held-out label/GT reader."""
import csv
import io
import json
import os
from pathlib import Path
import subprocess

import p9_r4_contract as c

CLOSURE_SHA = "86ec15a279b84cc970530976a7e998af354a6d33"
RECOVERY = c.OUT / "source_recovery"
SOURCE_MANIFEST = RECOVERY / "heldout_source_manifest_recovered.csv"
MANIFEST = c.OUT / "execution_manifest_recovered.json"
CACHE = Path("/tmp/p9_r4_same_objective_source_recovery.l6hlb3lc")
ENV = dict(os.environ, PYTHONDONTWRITEBYTECODE="1", OPENBLAS_NUM_THREADS="1",
           OMP_NUM_THREADS="1", LD_LIBRARY_PATH="/lib/x86_64-linux-gnu")


def validate_sources(rows, expected_raw, pool):
    c.require(len(rows) == len(pool) == 160 and
              [int(r["transaction_id"]) for r in rows] == pool and len(set(pool)) == 160,
              "recovered source identity/order/count changed")
    for row in rows:
        path = Path(row["raw_cloud_file"])
        c.require(path.parent == CACHE / "source_clouds" and path.is_file() and
                  expected_raw.get(str(path)) == row["raw_source_sha256"] and
                  c.digest(path) == row["raw_source_sha256"] and
                  path.stat().st_size == 12 * int(row["raw_point_count"]),
                  "RECOVERED_SOURCE_CACHE_UNAVAILABLE: " + str(path))


def cache_gate(auditor):
    c.require(CACHE.is_dir(), "RECOVERED_SOURCE_CACHE_UNAVAILABLE: directory missing")
    c.pinned(RECOVERY / "results.json", CLOSURE_SHA)
    c.pinned(SOURCE_MANIFEST, CLOSURE_SHA)
    closure = json.loads((RECOVERY / "results.json").read_text())
    c.require(closure["FINAL_RESULT"] == "R4_SOURCE_CLOUD_PROVENANCE_CLOSED" and
              closure["cache"] == str(CACHE), "source closure not pinned")
    expected = closure["raw_source_sha256"]
    c.require(len(expected) == 192, "closure export count changed")
    for path, sha in expected.items():
        c.require(Path(path).is_file() and c.digest(path) == sha,
                  "RECOVERED_SOURCE_CACHE_UNAVAILABLE: " + path)
    pool = [int(r["transaction_id"]) for r in c.read_csv(c.OUT / "heldout_ordered_pool.csv")]
    rows = c.read_csv(SOURCE_MANIFEST)
    validate_sources(rows, expected, pool)
    c.require(c.digest(auditor) == "ccb21b4c9f466ca8283112f59a130f7c038002e858c043fc527028b18ca66907",
              "frozen source auditor changed")
    output = subprocess.check_output([str(auditor), str(SOURCE_MANIFEST)], env=ENV, text=True)
    audit = list(csv.DictReader(io.StringIO(output)))
    c.require(len(audit) == 160 and [int(r["transaction_id"]) for r in audit] == pool and
              all(r["parity"] == "PASS" for r in audit), "recovered prepared source admission failed")
    return rows, audit


def verify(for_evidence=False):
    c.pinned(SOURCE_MANIFEST, CLOSURE_SHA)
    m = json.loads(MANIFEST.read_text())
    c.require(m["source_closure_commit"] == CLOSURE_SHA and
              m["SOURCE_PROVENANCE"] == "RECOVERED_SAME_OBJECTIVE_SCAN_END_DESKEW" and
              not m["gt_loaded"] and m["candidate_calls_per_frame"] == 12 and
              m["oracle_calls_per_frame"] == 263, "invalid recovered execution receipt")
    for key in ("input_sha256", "code_sha256", "raw_source_sha256", "closure_raw_source_sha256", "protected_receipts_sha256"):
        for name, expected in m[key].items():
            # The blind builder need not inspect archived historical oracle label receipts.
            if for_evidence and (Path(name).name.startswith("oracle_") or (c.OUT / "oracle") in Path(name).parents):
                continue
            c.require(c.digest(name) == expected, "recovered input/code/receipt changed: " + name)
    for name, expected in m["artifacts"].items():
        if for_evidence and name.startswith("oracle_"):
            continue
        c.require(c.digest(c.OUT / name) == expected, "recovered artifact changed: " + name)
    c.require(c.digest(m["binary"]) == m["binary_sha256"] and
              c.digest(m["oracle_library"]) == m["oracle_library_sha256"], "Release carrier changed")
    rows = c.read_csv(SOURCE_MANIFEST)
    pool = [int(r["transaction_id"]) for r in c.read_csv(c.OUT / "heldout_ordered_pool.csv")]
    validate_sources(rows, m["raw_source_sha256"], pool)
    return m


def self_test():
    def reject(rows, expected, pool):
        try:
            validate_sources(rows, expected, pool)
        except RuntimeError:
            return
        raise RuntimeError("source guard admitted mutated fixture")
    pool = list(range(160))
    fixture = [dict(transaction_id=str(i), raw_cloud_file="/tmp/not-authorized.xyzf",
                    raw_source_sha256="bad", raw_point_count="1") for i in pool]
    reject(fixture[:159], {}, pool)
    reject(fixture[::-1], {}, pool)
    reject(fixture, {}, pool)
    reject(fixture, {}, [0] * 160)
    print("P9_R4_RECOVERED_SOURCE_ADMISSION_GUARD_SELF_TEST=PASS")
