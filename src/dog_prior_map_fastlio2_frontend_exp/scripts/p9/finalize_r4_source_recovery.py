#!/usr/bin/env python3
"""Archive and independently cross-check completed provenance artifacts; no NDT."""
import argparse
import json
from pathlib import Path

import p9_r4_contract as c
import p9_r4_source_recovery as r


def self_test():
    from unittest.mock import patch
    result=json.loads((r.OUT/"results.json").read_text())
    validate_closure_rows(result)
    original=c.read_csv
    cases=("duplicate_control","hash_last_bit","prepared_hash","extra_manifest_column","duplicate_field","swapped_raw_provenance")
    for case in cases:
        def mutated(path):
            rows=original(path)
            if Path(path)==r.OUT/"historical_raw_parity.csv" and case=="duplicate_control":
                rows[-1]=dict(rows[0])
            elif Path(path)==r.OUT/"full_source_trajectory_parity.csv" and case=="hash_last_bit":
                rows[-1]["recovered_hash"]=str(int(rows[-1]["recovered_hash"])+1)
            elif Path(path)==r.OUT/"heldout_recovered_source_audit.csv" and case=="prepared_hash":
                rows[0]["actual_hash"]=str(int(rows[0]["actual_hash"])+1)
            elif Path(path)==r.OUT/"heldout_source_manifest_recovered.csv" and case=="extra_manifest_column":
                rows[0]["unallowed_column"]="extra"
            elif Path(path)==r.OUT/"manifest_field_parity.csv" and case=="duplicate_field":
                rows[-1]=dict(rows[0])
            elif Path(path)==r.OUT/"heldout_source_manifest_recovered.csv" and case=="swapped_raw_provenance":
                for field in r.SOURCE_FIELDS:rows[0][field],rows[1][field]=rows[1][field],rows[0][field]
            return rows
        with patch.object(c,"read_csv",side_effect=mutated):
            try:
                validate_closure_rows(result)
            except RuntimeError:
                pass
            else:
                raise RuntimeError("archive mutation escaped validator: "+case)
    c.save_json(r.OUT/"archive_mutation_self_test.json",dict(status="PASS",cases=list(cases),
        fixture="in-memory CSV mutations only; all physical source/CSV artifacts unchanged",NEW_NDT_CALLS=0))
    print("SOURCE_RECOVERY_ARCHIVE_MUTATION_SELF_TEST=PASS 6/6")


def validate_closure_rows(result):
    controls=c.read_csv(c.ARCHIVE/"frozen/cohort_frozen.csv")
    raw=c.read_csv(r.OUT/"historical_raw_parity.csv")
    c.require(len(raw)==len(controls)==32,"raw control denominator changed")
    for row,control in zip(raw,controls):
        path=str(Path(result["cache"])/"source_clouds"/("raw_tx_"+control["transaction_id"]+".xyzf"))
        c.require(row["transaction_id"]==control["transaction_id"] and row["parity"]=="PASS" and
            row["historical_raw_cloud_file"]==control["raw_cloud_file"] and row["recovered_raw_cloud_file"]==path and
            row["historical_sha256"]==row["recovered_sha256"]==control["raw_source_sha256"]==result["raw_source_sha256"][path],
            "raw control row identity/hash mismatch")
    expected=c.read_csv(c.REGISTRATION)
    actual=c.read_csv(Path(result["cache"])/"replay_registration.csv")
    rows=c.read_csv(r.OUT/"full_source_trajectory_parity.csv")
    c.require(len(rows)==len(expected)==len(actual)==4127,"trajectory denominator changed")
    c.require([int(row["transaction_id"]) for row in expected]==list(range(1,4128)),"historical trajectory identity changed")
    for row,before,after in zip(rows,expected,actual):
        c.require(row["transaction_id"]==row["recovered_transaction_id"]==before["transaction_id"]==after["transaction_id"] and
            row["expected_stamp_ns"]==row["recovered_stamp_ns"]==before["stamp_ns"]==after["stamp_ns"] and
            int(row["expected_points"])==int(row["recovered_points"])==int(before["source_points"])==int(after["source_points"]) and
            int(row["expected_hash"])==int(row["recovered_hash"])==int(before["source_cloud_hash"])==int(after["source_cloud_hash"]) and
            row["parity"]=="PASS","trajectory row mismatch")
    old=c.read_csv(c.OUT/"heldout_source_manifest.csv")
    new=c.read_csv(r.OUT/"heldout_source_manifest_recovered.csv")
    prepared=c.read_csv(r.OUT/"heldout_recovered_source_audit.csv")
    pool=c.read_csv(c.OUT/"heldout_ordered_pool.csv")
    c.require(len(old)==len(new)==len(prepared)==len(pool)==160 and
              [row["transaction_id"] for row in old]==[row["transaction_id"] for row in pool],"held-out IDs changed")
    for before,after,row in zip(old,new,prepared):
        c.require(tuple(before)==tuple(after) and before["transaction_id"]==after["transaction_id"]==row["transaction_id"] and
            all(before[field]==after[field] for field in before if field not in r.SOURCE_FIELDS) and
            row["parity"]=="PASS" and int(row["expected_points"])==int(row["actual_points"])==int(before["prepared_source_point_count"]) and
            int(row["expected_hash"])==int(row["actual_hash"])==int(before["prepared_source_hash"]),"held-out row/schema mismatch")
        path=Path(after["raw_cloud_file"])
        canonical=Path(result["cache"])/"source_clouds"/("raw_tx_"+before["transaction_id"]+".xyzf")
        c.require(path==canonical and result["raw_source_sha256"][str(path)]==after["raw_source_sha256"]==c.digest(path) and
                  path.stat().st_size==12*int(after["raw_point_count"]),"held-out raw record mismatch")
    fields=c.read_csv(r.OUT/"manifest_field_parity.csv")
    expected_fields=[(before["transaction_id"],field,before[field],after[field]) for before,after in zip(old,new)
                     for field in before if field not in r.SOURCE_FIELDS]
    c.require(len(fields)==len(expected_fields)==2400,"manifest-field denominator changed")
    c.require([(row["transaction_id"],row["field"],row["expected"],row["actual"]) for row in fields]==expected_fields and
              all(row["expected"]==row["actual"] and row["parity"]=="PASS" for row in fields),"manifest field coverage mismatch")
    c.require(result["manifest_non_source_parity"]==dict(passed=2400,total=2400,rows=160,unchanged_fields=15,status="PASS"),
              "manifest JSON gate disagrees with CSV")
    c.require(result["TX2932"]==next(row for row in prepared if row["transaction_id"]=="2932"),
              "TX2932 JSON disagrees with source auditor CSV")


def normalize_delivery_units(result):
    # Keep the unchanged frozen-audit receipt; only normalize delivery ledger labels.
    original=r.OUT/"ordered_gate_audit_result.json"
    if not original.exists():original.write_bytes((r.OUT/"results.json").read_bytes())
    raw_ledger=r.OUT/"replay_cost_frozen_audit.csv"
    if not raw_ledger.exists():raw_ledger.write_bytes((r.OUT/"replay_cost.csv").read_bytes())
    cost=result["cost"]
    for name in ("prediction_and_deskew_ms","cloud_io_ms","ndt_total_ms","ndt_alignment_ms",
                 "ikfom_update_ms","frame_total_ms"):
        if name in cost:cost[name[:-3]+"_seconds"]=cost.pop(name)
    runtime=c.read_csv(Path(result["cache"])/"replay_runtime.csv")
    execution=json.loads((r.OUT/"replay_execution.json").read_text())
    c.require(cost["baseline_ndt_calls"]==4127 and cost["synthetic_p7_test_align_calls"]==2 and
              cost["category"]=="OFFLINE_PROVENANCE_ONLY" and cost["wall_seconds"]==execution["wall_seconds"],
              "delivery call/wall-time accounting differs from frozen execution")
    for name in ("prediction_and_deskew","cloud_io","ndt_total","ndt_alignment","ikfom_update","frame_total"):
        c.require(cost[name+"_seconds"]==sum(float(row[name+"_ms"]) for row in runtime)/1000,
                  "delivery timing unit/value mismatch")
    cost["timing_units"]="seconds; original runtime CSV uses milliseconds"
    c.save_json(r.OUT/"results.json",result)
    c.write_csv(r.OUT/"replay_cost.csv",[dict(metric=key,value="" if value is None else value,
        unit="seconds" if key.endswith("_seconds") else "calls" if key.endswith("_calls") else "description",
        category="OFFLINE_PROVENANCE_ONLY") for key,value in cost.items()])


def main():
    result=json.loads((r.OUT/"results.json").read_text())
    old=r.verify_old_archive()
    r.lineage_anchor()
    c.require(result["R4_ORACLE_CALLS"]==result["R4_CANDIDATE_CALLS"]==result["NEW_VISUAL_EXTRACTION"]==0
              and result["GT_LOADED"] is False,"recovery scope changed")
    c.require(result["R4_SCIENTIFIC_RESULT"]=="NOT_RUN" and result["replay_outputs_are_reference_replacements"] is False,
              "source closure was mislabeled as a scientific result/reference replacement")
    valid_next={"R4_SOURCE_CLOUD_PROVENANCE_CLOSED":"R4_RESUME_HELDOUT_ORACLE_FROM_RECOVERED_SOURCES",
        "SOURCE_REPLAY_RAW_PARITY_FAIL":"STOP_RESOLVE_FROZEN_SOURCE_PROVENANCE",
        "CURRENT_BASELINE_REPLAY_NOT_REPRODUCED":"STOP_RESOLVE_FROZEN_SOURCE_PROVENANCE",
        "HELDOUT_SOURCE_RECONSTRUCTION_INCOMPLETE":"STOP_RESOLVE_FROZEN_SOURCE_PROVENANCE"}
    c.require(result["FINAL_RESULT"] in valid_next and result["NEXT"]==valid_next[result["FINAL_RESULT"]],"illegal result/next pair")
    targets=r.export_targets();cache=Path(result["cache"])
    c.require(set(result["raw_source_sha256"])=={str(cache/"source_clouds"/("raw_tx_"+str(tx)+".xyzf")) for tx in targets},
              "recovered raw hash-map coverage changed")
    expected_outputs=("replay_registration.csv","replay_trajectory.csv","replay_runtime.csv",
                      "replay_dual_u_diagnostic.csv","replay_curvature_diagnostic.csv")
    c.require(set(result["replay_output_sha256"])=={str(cache/name) for name in expected_outputs},"replay output hash-map coverage changed")
    required=("historical_raw_parity.csv","full_source_trajectory_parity.csv",
              "heldout_recovered_source_audit.csv","manifest_field_parity.csv")
    checks={}
    if result["FINAL_RESULT"]=="R4_SOURCE_CLOUD_PROVENANCE_CLOSED":
        validate_closure_rows(result)
        counts=(32,4127,160,2400)
        for name,count in zip(required,counts):
            rows=c.read_csv(r.OUT/name)
            c.require(len(rows)==count and all(row["parity"]=="PASS" for row in rows),"CSV closure mismatch: "+name)
            checks[name]=dict(rows=count,status="PASS")
        for key,total in (("historical_raw_parity",32),("full_source_trajectory_parity",4127),
                          ("heldout_prepared_source_parity",160)):
            c.require(result[key]==dict(passed=total,total=total,evaluated=total),"JSON closure mismatch: "+key)
        old_manifest=c.read_csv(c.OUT/"heldout_source_manifest.csv")
        new_manifest=c.read_csv(r.OUT/"heldout_source_manifest_recovered.csv")
        c.require(len(new_manifest)==160 and all(before["transaction_id"]==after["transaction_id"] and
            all(before[field]==after[field] for field in before if field not in r.SOURCE_FIELDS)
            for before,after in zip(old_manifest,new_manifest)),"manifest original fields changed")
        for row in new_manifest:
            path=Path(row["raw_cloud_file"])
            c.require(c.digest(path)==row["raw_source_sha256"] and path.stat().st_size==12*int(row["raw_point_count"]),
                      "recovered raw manifest SHA/count mismatch")
        c.require(c.digest(r.OUT/"heldout_source_manifest_recovered.csv")==c.digest(r.OUT/"recovered_source_manifest.csv"),
                  "recovered manifest aliases differ")
    for path,sha in result["raw_source_sha256"].items():
        c.require(c.digest(path)==sha,"recovered source changed: "+path)
    for path,sha in result["replay_output_sha256"].items():
        c.require(c.digest(path)==sha,"replay output changed: "+path)
    c.require("100% tests passed, 0 tests failed out of 38" in (r.OUT/"p9_self_tests.log").read_text(),"P9 tests failed")
    c.require("100% tests passed, 0 tests failed out of 3" in (r.OUT/"frozen_p7_self_tests.log").read_text(),"P7 tests failed")
    c.require("192/192 PASS" in (r.OUT/"selection_self_test.log").read_text(),"compiled export selection test failed")
    normalize_delivery_units(result)
    cost=result["cost"];tx=result.get("TX2932")
    binary=json.loads((r.OUT/"replay_binary_hash.json").read_text())
    report=["# R4 source-cloud provenance recovery", "", "FINAL_RESULT = "+result["FINAL_RESULT"],
        "NEXT = "+result["NEXT"], "", "This is provenance closure only. R4 scientific result remains NOT_RUN.",
        "No oracle263, B12 candidates, visual extraction or GT were run/loaded.", "", "## Git and implementation", "",
        "Branch: `"+result["branch"]+"`", "Start: `"+result["start_sha"]+"`",
        "Frozen replay source: `"+result["source_commit"]+"`", "Workspace: `"+result["worktree"]+"`",
        "Detached replay worktree: `"+result["replay_worktree"]+"`", "External cache: `"+result["cache"]+"`",
        "Binary SHA256: `"+binary["binary_sha256"]+"`", "",
        "Only export predicate/192-ID table and expected count changed. No production runner or R4 algorithm was replaced.",
        "End SHA is reported by git rev-parse HEAD after commit (not embedded in its own content-addressed archive).", "",
        "## Ordered parity gates", "", "| Gate | PASS | Full denominator | Evaluated |",
        "| --- | ---: | ---: | ---: |"]
    for key in ("historical_raw_parity","full_source_trajectory_parity","heldout_prepared_source_parity"):
        gate=result[key];report.append(f"| {key} | {gate['passed']} | {gate['total']} | {gate['evaluated']} |")
    report.extend(["", "Manifest non-source parity: "+str(result.get("manifest_non_source_parity","NOT_EVALUATED")), ""])
    if tx:
        report.extend(["TX2932 expected/recovered points: "+tx["expected_points"]+" / "+tx["actual_points"],
            "TX2932 expected/recovered hash: "+tx["expected_hash"]+" / "+tx["actual_hash"],
            "TX2932: "+tx["parity"], ""])
    report.extend(["## Provenance-only cost", "", "| Metric | Seconds / calls |", "| --- | ---: |",
        "| Baseline data NDT calls | 4127 |", "| Frozen P7 synthetic test alignments (separate) | 2 |",
        f"| Wrapper wall | {cost['wall_seconds']:.9f} |",
        f"| Prediction + deskew | {cost['prediction_and_deskew_seconds']:.9f} |",
        f"| NDT alignment | {cost['ndt_alignment_seconds']:.9f} |",
        f"| NDT total including preprocessing | {cost['ndt_total_seconds']:.9f} |",
        f"| Cloud input I/O | {cost['cloud_io_seconds']:.9f} |",
        f"| Export I/O upper bound only | {cost['export_io_upper_bound_seconds']:.9f} |", "",
        "Export I/O is NOT_SEPARATELY_MEASURED; the bound includes diagnostics, setup and polling.",
        "Delivery ledger uses explicit seconds keys/units; original frozen-audit cost receipt is retained unchanged.",
        "The wrapper wall includes progress-polling tail latency. These are not online-method runtimes.", "",
        "## Verification and preservation", "", "Release P7 build; P7 tests3/3; P9 tests38/38; selection192/192;",
        "integer-hash/manifest and canonical-guard self-tests PASS; CSV/JSON/hash checks PASS.",
        "Initial P9 invocation hit two libusb loader errors; fixed only the invocation's library path, not code or parameters.",
        "Original blocker files and R4 code remain unchanged, including the old topic-source manifest.",
        "Recovery manifest retains cloud_data_sha256 as legacy topic payload metadata, not recovered raw SHA.",
        "Raw192 exports and full replay outputs are outside Git in /tmp. Preserve this cache for the next task.",
        "The bundle contains code/receipts/ancestry, not raw clouds. Original T0, initial state, U_obs and score remain authoritative.", ""])
    (r.OUT/"REPORT.md").write_text("\n".join(report))
    c.save_json(r.OUT/"verification.json",dict(status="PASS",csv_json_checks=checks,
        old_artifact_count=len(old),raw_hashes_verified=len(result["raw_source_sha256"]),
        replay_output_hashes_verified=len(result["replay_output_sha256"]),GT_LOADED=False))
    files={str(path.relative_to(c.ROOT)):c.digest(path) for path in r.OUT.rglob("*")
           if path.is_file() and path.name!="artifact_hashes.json"}
    code={str(c.HERE/name):c.digest(c.HERE/name) for name in
          ("p9_r4_source_recovery.py","run_r4_source_recovery.py","audit_r4_source_recovery.py",
           "finalize_r4_source_recovery.py","p9_r4_source_export_selection_test.cpp")}
    c.save_json(r.OUT/"artifact_hashes.json",dict(files=files,source_sha256=code,
        raw_source_sha256=result["raw_source_sha256"],replay_output_sha256=result["replay_output_sha256"],
        historical_artifact_hashes_unchanged=old,GT_LOADED=False,R4_ORACLE_CALLS=0,R4_CANDIDATE_CALLS=0))
    print("SOURCE_RECOVERY_CSV_JSON_HASH_AUDIT=PASS")


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test",action="store_true")
    if parser.parse_args().self_test:self_test()
    else:main()
