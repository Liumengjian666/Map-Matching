#!/usr/bin/env python3
"""Archive existing A3G outputs only. Never launch a runner or read GT."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil

from p6_a3g_soak import ROOT, summarize


def digest(path):
    h=hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda:f.read(1024*1024),b""):
            h.update(block)
    return h.hexdigest()


def thin(source,target,columns,last=None):
    with source.open() as f:
        rows=list(csv.DictReader(f))
    if last is not None:
        rows=rows[-last:]
    with target.open("w") as f:
        writer=csv.DictWriter(f,fieldnames=columns,extrasaction="ignore",lineterminator="\n")
        writer.writeheader(); writer.writerows(rows)


def collect(source,destination):
    destination.mkdir(parents=True,exist_ok=False)
    result=json.loads((source/"run_result.json").read_text())
    summary=summarize(source)
    assert result["REAL_REPLAY_COUNT"]==1 and result["RETRY_COUNT"]==0 and not result["GT_USED"]
    summary["attempted_terminals"]=summary["covariance_requests"]
    for name in ("run_result.json","source_identity.json","input_identity.json","command.json",
                 "resource.txt","console.txt","trajectory.csv.a3g_health.csv","trajectory.csv.a3f_r1_covariance.csv",
                 "trajectory.csv"):
        shutil.copyfile(source/name,destination/name)
    (destination/"health_summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    thin(source/"events.csv",destination/"LAST_20_COMPLETED_EVENTS.csv",
         list(csv.DictReader((source/"events.csv").open()).fieldnames),20)
    thin(source/"trajectory.csv.r1_preopt_capsule.csv",destination/"LAST_5_PREOPT.csv",
         list(csv.DictReader((source/"trajectory.csv.r1_preopt_capsule.csv").open()).fieldnames),5)
    thin(source/"trajectory.csv.a3c_r1_marginalization_trace.csv",destination/"QR_REMOVAL_HISTORY.csv",
         ["transaction_id","event_stamp_ns","enforcement_index","attempt_index","oldest_state_stamp_ns",
          "nodes_before_attempt","span_before_attempt_s","nodes_after_attempt","span_after_attempt_s",
          "marginalization_result","marginalization_backend","qr_stack_rows","qr_columns","qr_marginalized_rank",
          "qr_rank_threshold","qr_R_diag_min","qr_R_diag_max","qr_rows_after_compression","qr_ms","legacy_shadow_status"])
    thin(source/"runtime.csv",destination/"runtime.csv",list(csv.DictReader((source/"runtime.csv").open()).fieldnames))
    frozen=ROOT/"docs/p6_alg_integration_a3f_r1/RUN_P3_200/EXTERNAL_RUN_FILES.json"
    entry=next(r for r in json.loads(frozen.read_text()) if Path(r["path"]).name=="events.csv")
    old_path=Path(entry["path"]); assert digest(old_path)==entry["sha256"]
    old=list(csv.DictReader(old_path.open()))
    current={r["timestamp"]:r for r in csv.DictReader((source/"events.csv").open())}
    differences=[dict(stamp=r["timestamp"],field=k,old=r[k],new=current.get(r["timestamp"],{}).get(k))
        for r in old for k in r if k!="qr_marginalization_ms" and r[k]!=current.get(r["timestamp"],{}).get(k)]
    (destination/"FROZEN_PREFIX_PARITY.json").write_text(json.dumps(dict(rows=len(old),
        frozen_reference_sha256=entry["sha256"],excluded_fields=["qr_marginalization_ms"],differences=differences),indent=2)+"\n")
    identity=json.loads((source/"source_identity.json").read_text())
    launcher_key="src/dog_prior_map_fastlio2_frontend_exp/scripts/p6_a3g_soak.py"
    assert digest(source/"LAUNCHER_AS_RUN.py")==identity["diagnostic_source_sha256"][launcher_key]
    files=[dict(path=str(p),bytes=p.stat().st_size,sha256=digest(p)) for p in sorted(source.iterdir()) if p.is_file()]
    (destination/"EXTERNAL_RUN_FILES.json").write_text(json.dumps(files,indent=2)+"\n")
    print(json.dumps(summary,indent=2))


if __name__=="__main__":
    parser=argparse.ArgumentParser(); parser.add_argument("source",type=Path); parser.add_argument("destination",type=Path)
    args=parser.parse_args(); collect(args.source,args.destination)
