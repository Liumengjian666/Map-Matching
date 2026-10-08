#!/usr/bin/env python3
"""Ordered raw/trajectory/prepared-source audits. No oracle, visual, or GT."""
import argparse
import json
import math
from pathlib import Path
import subprocess

import p9_r4_contract as c
import p9_r4_source_recovery as r
import run_r4_source_recovery as runner


class GateFailure(RuntimeError):
    def __init__(self, result, reason):
        super().__init__(reason)
        self.result=result


def stop_if(condition, result, reason):
    if not condition:
        raise GateFailure(result,reason)


def trajectory_parity(expected, actual):
    return (expected["transaction_id"]==actual["transaction_id"] and
            expected["stamp_ns"]==actual["stamp_ns"] and
            int(expected["source_points"])==int(actual["source_points"]) and
            int(expected["source_cloud_hash"])==int(actual["source_cloud_hash"]))


def manifest_pairs(old, new):
    c.require(len(old)==len(new)==160,"recovered manifest row count changed")
    rows=[]
    for before,after in zip(old,new):
        c.require(before.keys()==after.keys(),"recovered manifest schema changed")
        for field in before:
            if field not in r.SOURCE_FIELDS:
                rows.append(dict(transaction_id=before["transaction_id"],field=field,
                    expected=before[field],actual=after[field],parity="PASS" if before[field]==after[field] else "FAIL"))
    return rows


def self_test():
    first=dict(transaction_id="2932",stamp_ns="123",source_points="413",
               source_cloud_hash="3530993910003886054")
    c.require(trajectory_parity(first,dict(first)),"identity trajectory test failed")
    for field in first:
        changed=dict(first);changed[field]=str(int(first[field])+1)
        c.require(not trajectory_parity(first,changed),"integer/order/stamp trajectory test failed: "+field)
    old=[dict(transaction_id=str(i),raw_cloud_file="old",raw_source_sha256="old",raw_point_count="1",
              stamp_ns="1",cloud_data_sha256="legacy topic") for i in range(160)]
    new=[dict(row,raw_cloud_file="new",raw_source_sha256="new",raw_point_count="2") for row in old]
    c.require(all(row["parity"]=="PASS" for row in manifest_pairs(old,new)),"source-only replacement test failed")
    new[0]["cloud_data_sha256"]="changed"
    c.require(sum(row["parity"]=="FAIL" for row in manifest_pairs(old,new))==1,"legacy-field mutation test failed")
    print("R4_SOURCE_RECOVERY_INTEGER_AND_MANIFEST_SELF_TEST=PASS")


def cost_ledger(receipt, execution):
    runtime=c.read_csv(Path(receipt["cache"])/"replay_runtime.csv")
    c.require(len(runtime)==4127 and [int(row["transaction_id"]) for row in runtime]==list(range(1,4128)),
              "replay runtime is not the complete ordered trajectory")
    names=("prediction_and_deskew_ms","cloud_io_ms","ndt_total_ms","ndt_alignment_ms",
           "ikfom_update_ms","frame_total_ms")
    c.require(all(math.isfinite(float(row[name])) and float(row[name])>=0 for row in runtime for name in names),
              "invalid runtime timing")
    totals={name:sum(float(row[name]) for row in runtime)/1000 for name in names}
    targets=set(receipt["targets"])
    untimed=sum(max(0.,float(row["frame_total_ms"])-sum(float(row[name]) for name in
        ("prediction_and_deskew_ms","cloud_io_ms","ndt_total_ms","ikfom_update_ms")))/1000
        for row in runtime if int(row["transaction_id"]) in targets)
    outside=max(0.,execution["wall_seconds"]-totals["frame_total_ms"])
    costs=dict(baseline_ndt_calls=4127,synthetic_p7_test_align_calls=2,
        wall_seconds=execution["wall_seconds"],wall_timing="wrapper elapsed; includes progress polling tail",
        **totals,export_io_seconds=None,export_io_status="NOT_SEPARATELY_MEASURED",
        export_io_upper_bound_seconds=untimed+outside,
        export_io_bound_definition="export-target untimed frame residual + wall outside timed frames; includes diagnostics/setup/polling",
        category="OFFLINE_PROVENANCE_ONLY")
    c.write_csv(r.OUT/"replay_cost.csv",[dict(metric=key,value="" if value is None else value,
        category="OFFLINE_PROVENANCE_ONLY") for key,value in costs.items()])
    return costs


def raw_gate(source_dir, state):
    controls=c.read_csv(c.ARCHIVE/"frozen/cohort_frozen.csv")
    c.require(len(controls)==32 and set(int(row["transaction_id"]) for row in controls)==set(c.DEVELOPMENT),
              "historical raw control identity changed")
    rows=[]
    for control in controls:
        tx=control["transaction_id"];original=Path(control["raw_cloud_file"])
        recovered=source_dir/("raw_tx_"+tx+".xyzf")
        expected=c.digest(original)
        actual=c.digest(recovered) if recovered.is_file() else "MISSING"
        passed=expected==actual==control["raw_source_sha256"]
        rows.append(dict(transaction_id=tx,historical_raw_cloud_file=str(original),recovered_raw_cloud_file=str(recovered),
            historical_sha256=expected,recovered_sha256=actual,parity="PASS" if passed else "FAIL"))
        if not passed:
            c.write_csv(r.OUT/"historical_raw_parity.csv",rows)
            state["historical_raw_parity"]=dict(passed=len(rows)-1,total=32,evaluated=len(rows))
            raise GateFailure("SOURCE_REPLAY_RAW_PARITY_FAIL","raw byte mismatch at TX"+tx)
    c.write_csv(r.OUT/"historical_raw_parity.csv",rows)
    state["historical_raw_parity"]=dict(passed=32,total=32,evaluated=32)


def full_trajectory_gate(cache, state):
    expected=c.read_csv(c.REGISTRATION)
    actual=c.read_csv(cache/"replay_registration.csv")
    c.require(len(expected)==4127,"historical trajectory count changed")
    stop_if(len(actual)==len(expected),"CURRENT_BASELINE_REPLAY_NOT_REPRODUCED","trajectory row count mismatch")
    rows=[]
    for before,after in zip(expected,actual):
        passed=trajectory_parity(before,after)
        rows.append(dict(transaction_id=before["transaction_id"],recovered_transaction_id=after["transaction_id"],
            expected_stamp_ns=before["stamp_ns"],recovered_stamp_ns=after["stamp_ns"],
            expected_points=before["source_points"],recovered_points=after["source_points"],
            expected_hash=before["source_cloud_hash"],recovered_hash=after["source_cloud_hash"],
            parity="PASS" if passed else "FAIL"))
        if not passed:
            c.write_csv(r.OUT/"full_source_trajectory_parity.csv",rows)
            state["full_source_trajectory_parity"]=dict(passed=len(rows)-1,total=len(expected),evaluated=len(rows))
            raise GateFailure("CURRENT_BASELINE_REPLAY_NOT_REPRODUCED","source trajectory mismatch at TX"+before["transaction_id"])
    c.write_csv(r.OUT/"full_source_trajectory_parity.csv",rows)
    state["full_source_trajectory_parity"]=dict(passed=len(rows),total=len(expected),evaluated=len(rows))


def heldout_gate(source_dir, binary, state):
    old=c.read_csv(c.OUT/"heldout_source_manifest.csv")
    pool=c.read_csv(c.OUT/"heldout_ordered_pool.csv")
    c.require(len(old)==160 and [row["transaction_id"] for row in old]==[row["transaction_id"] for row in pool],
              "held-out manifest no longer matches immutable pool order")
    new=[]
    for before in old:
        path=source_dir/("raw_tx_"+before["transaction_id"]+".xyzf")
        stop_if(path.is_file() and path.stat().st_size%12==0,"HELDOUT_SOURCE_RECONSTRUCTION_INCOMPLETE",
                "missing/invalid raw cloud at TX"+before["transaction_id"])
        new.append(dict(before,raw_cloud_file=str(path),raw_source_sha256=c.digest(path),
                        raw_point_count=str(path.stat().st_size//12)))
    recovered=r.OUT/"heldout_source_manifest_recovered.csv"
    c.write_csv(recovered,new,list(old[0]))
    (r.OUT/"recovered_source_manifest.csv").write_bytes(recovered.read_bytes())
    c.require(c.digest(binary["source_auditor"])==binary["source_auditor_sha256"],"source auditor changed")
    with (r.OUT/"heldout_recovered_source_audit.csv").open("w") as output, (r.OUT/"source_auditor_stderr.log").open("w") as errors:
        completed=subprocess.run([binary["source_auditor"],str(recovered)],env=runner.ENV,stdout=output,stderr=errors)
    stop_if(completed.returncode==0,"HELDOUT_SOURCE_RECONSTRUCTION_INCOMPLETE","source auditor did not complete")
    audit=c.read_csv(r.OUT/"heldout_recovered_source_audit.csv")
    valid=(len(audit)==160 and [row["transaction_id"] for row in audit]==[row["transaction_id"] for row in old])
    stop_if(valid,"HELDOUT_SOURCE_RECONSTRUCTION_INCOMPLETE","source auditor omitted/reordered/duplicated frames")
    passed=sum(row["parity"]=="PASS" and
               int(row["expected_points"])==int(row["actual_points"])==int(before["prepared_source_point_count"]) and
               int(row["expected_hash"])==int(row["actual_hash"])==int(before["prepared_source_hash"])
               for row,before in zip(audit,old))
    state["heldout_prepared_source_parity"]=dict(passed=passed,total=160,evaluated=160)
    state["TX2932"]=next(row for row in audit if row["transaction_id"]=="2932")
    stop_if(passed==160,"HELDOUT_SOURCE_RECONSTRUCTION_INCOMPLETE","not all recovered held-out sources are admitted")
    # Gate4 executes only after all160 sources pass gate3; read back actual archived CSV.
    fields=manifest_pairs(old,c.read_csv(recovered))
    c.write_csv(r.OUT/"manifest_field_parity.csv",fields)
    stop_if(all(row["parity"]=="PASS" for row in fields),"HELDOUT_SOURCE_RECONSTRUCTION_INCOMPLETE",
            "non-source manifest fields changed")
    state["manifest_non_source_parity"]=dict(passed=sum(row["parity"]=="PASS" for row in fields),
        total=len(fields),rows=160,unchanged_fields=len(old[0])-len(r.SOURCE_FIELDS),status="PASS")


def audit():
    c.require(not (r.OUT/"results.json").exists(),"source recovery audit already archived")
    receipt,binary=runner.verify_frozen()
    started=json.loads((r.OUT/"replay_started.json").read_text())
    execution=json.loads((r.OUT/"replay_execution.json").read_text())
    c.require(started["state"]=="ONE_AUTHORIZED_REPLAY_STARTED_NO_RETRY" and
              started["authorized_baseline_ndt_calls"]==4127 and
              started["binary_sha256"]==execution["binary_sha256"]==binary["binary_sha256"] and
              started["command"]==binary["command"] and execution["cache"]==receipt["cache"],
              "start/execution receipt is not bound to the authorized replay")
    c.require(all(item[key]==value for item in (started,execution) for key,value in
              dict(R4_ORACLE_CALLS=0,R4_CANDIDATE_CALLS=0,NEW_VISUAL_EXTRACTION=0,GT_LOADED=False).items()),
              "start/execution scope flags changed")
    c.require(execution["returncode"]==0 and execution["state"]=="COMPLETED","replay incomplete; no admission")
    cache=Path(receipt["cache"]);sources=Path(receipt["source_directory"])
    state=dict(branch=c.BRANCH,start_sha=r.START,source_commit=c.HISTORY_SHA,
        input_lineage="PASS",worktree=str(c.ROOT),replay_worktree=receipt["worktree"],cache=receipt["cache"],
        exported_target_count=192,target_file_sha256=receipt["target_file_sha256"],
        R4_ORACLE_CALLS=0,R4_CANDIDATE_CALLS=0,NEW_VISUAL_EXTRACTION=0,GT_LOADED=False,
        R4_SCIENTIFIC_RESULT="NOT_RUN",replay_outputs_are_reference_replacements=False,
        historical_raw_parity=dict(passed=0,total=32,evaluated=0),
        full_source_trajectory_parity=dict(passed=0,total=4127,evaluated=0),
        heldout_prepared_source_parity=dict(passed=0,total=160,evaluated=0))
    state["cost"]=cost_ledger(receipt,execution)
    try:
        raw_gate(sources,state)
        full_trajectory_gate(cache,state)
        heldout_gate(sources,binary,state)
        actual={p.name for p in sources.glob("*.xyzf")}
        stop_if(actual=={"raw_tx_"+str(tx)+".xyzf" for tx in receipt["targets"]},
                "HELDOUT_SOURCE_RECONSTRUCTION_INCOMPLETE","exported raw-file target set mismatch")
        c.require(r.verify_old_archive()==receipt["historical_blocker_sha256"],"historical blocker files changed")
        state["FINAL_RESULT"]="R4_SOURCE_CLOUD_PROVENANCE_CLOSED"
        state["NEXT"]="R4_RESUME_HELDOUT_ORACLE_FROM_RECOVERED_SOURCES"
    except GateFailure as error:
        state["FINAL_RESULT"]=error.result;state["error"]=str(error)
        state["NEXT"]="STOP_RESOLVE_FROZEN_SOURCE_PROVENANCE"
    state["replay_output_sha256"]={str(path):c.digest(path) for path in cache.glob("replay_*.csv")}
    state["raw_source_sha256"]={str(path):c.digest(path) for path in sources.glob("*.xyzf")}
    c.save_json(r.OUT/"results.json",state)
    print(json.dumps({key:state[key] for key in ("FINAL_RESULT","NEXT","historical_raw_parity",
        "full_source_trajectory_parity","heldout_prepared_source_parity")},indent=2))
    if state["FINAL_RESULT"]!="R4_SOURCE_CLOUD_PROVENANCE_CLOSED":
        raise SystemExit(2)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage",choices=("self-test","audit"))
    args=parser.parse_args()
    if args.stage=="self-test":self_test()
    else:audit()
