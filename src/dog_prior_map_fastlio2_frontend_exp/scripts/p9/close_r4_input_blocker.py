#!/usr/bin/env python3
"""Archive an input STOP using read-only diagnostics. Never resumes NDT/search."""
import argparse
import csv
import hashlib
import io
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import p9_r4_contract as c


def close(binary):
    c.require(not list((c.OUT/"oracle/runs").glob("*")),"unexpected NDT execution artifacts")
    c.require(not (c.OUT/"cohort_selection_freeze.json").exists(),"unexpected completed oracle labels")
    for name in ("candidate_runs","candidate_engine.log","candidate_freeze.json","probe_manifest.csv",
                 "candidate_source_manifest.csv","visual_pair_manifest.csv","visual_measurements.csv",
                 "visual_measurement_freeze.json","evidence_freeze.json"):
        c.require(not (c.OUT/name).exists(),"unexpected execution-stage artifact: "+name)
    for name,path in (("heldout_source_audit.csv",c.OUT/"heldout_source_manifest.csv"),
                      ("historical_source_audit.csv",c.ARCHIVE/"frozen/cohort_frozen.csv")):
        output=subprocess.check_output([str(binary),str(path)],text=True)
        rows=list(csv.DictReader(io.StringIO(output)))
        c.write_csv(c.OUT/name,rows)
    heldout=c.read_csv(c.OUT/"heldout_source_audit.csv")
    historical=c.read_csv(c.OUT/"historical_source_audit.csv")
    c.require(len(heldout)==160 and len(historical)==32 and all(r["parity"]=="PASS" for r in historical),
              "historical source auditor parity failed")
    c.require(heldout[0]["parity"]=="FAIL","first source mismatch not reproduced")
    sys.path.insert(0,str(c.HERE.parent))
    import p4_i3_visual_increment as p4
    import sensor_msgs.point_cloud2 as pc2
    frozen={int(r["transaction_id"]):r for r in c.read_csv(c.ARCHIVE/"frozen/cohort_frozen.csv")}
    selected={120,368,616,2226,2350,3341}
    parity=[]
    for request in p4.requests():
        tx=int(request.transaction_id)
        if tx not in selected:continue
        points=np.asarray(list(pc2.read_points(request.cloud_end_frame,field_names=("x","y","z"),skip_nans=False)),dtype="<f4")
        raw_sha=hashlib.sha256(points.tobytes()).hexdigest()
        original=frozen[tx]
        c.require(c.digest(original["raw_cloud_file"])==original["raw_source_sha256"],"historical raw source changed")
        parity.append(dict(transaction_id=tx,archived_raw_sha256=original["raw_source_sha256"],
            topic_bag_decoded_raw_sha256=raw_sha,exact_raw_parity=int(raw_sha==original["raw_source_sha256"])))
    c.require(len(parity)==6 and not any(r["exact_raw_parity"] for r in parity),"topic/current-baseline source discrepancy not reproduced")
    c.write_csv(c.OUT/"topic_vs_same_objective_raw_parity.csv",parity)
    runner_path="src/dog_prior_map_fastlio2_frontend_exp/scripts/p7/p7_single_state_runner.cpp"
    source=subprocess.check_output(["git","show",c.HISTORY_SHA+":"+runner_path],cwd=c.ROOT)
    (c.OUT/"frozen_p7_source_export_reference.txt").write_bytes(source)
    codegen=json.loads((binary.parent/"r4_frozen/r4_codegen_manifest.json").read_text())
    c.save_json(c.OUT/"frozen_codegen_manifest.json",codegen)
    source_dir=c.ARCHIVE/"source_clouds"
    exports=sorted(int(p.stem.split("_")[-1]) for p in source_dir.glob("raw_tx_*.xyzf"))
    pool=[int(r["transaction_id"]) for r in c.read_csv(c.OUT/"heldout_ordered_pool.csv")]
    result=dict(task="PAPER-P9-R4-LOCAL-CLONE-RECOVERY-AND-RESUME",workspace=str(c.ROOT),local_clone="PASS",
        branch=c.BRANCH,start_sha=c.START_SHA,remote_start_verified=True,state="STOPPED_BEFORE_FIRST_ALIGNMENT",
        scientific_final_result=None,next="R4_SOURCE_CLOUD_PROVENANCE_CLOSURE",
        blocker="SOURCE_HASH_COUNT_MISMATCH_BEFORE_NDT",first_frame=heldout[0],
        attempted_oracle_batch_prefix=96,final_cohort_prefix=None,oracle_calls=0,candidate_calls=0,
        visual_pairs_extracted=0,gt_loaded=False,evidence_oracle_labels_loaded=False,
        heldout_selection=json.loads((c.OUT/"selection_freeze.json").read_text()),
        oracle_parity=dict(frames=24,major=9,no_major=15,major_ids=22,status="PASS"),
        original_seed_parity=json.loads((c.OUT/"execution_manifest.json").read_text())["original_seed_parity"],
        heldout_sources=dict(total=160,passed=sum(r["parity"]=="PASS" for r in heldout),
                             failed=sum(r["parity"]=="FAIL" for r in heldout)),
        historical_sources=dict(total=32,passed=32),topic_vs_same_objective_raw_parity=dict(tested=6,passed=0),
        archived_source_exports=dict(count=len(exports),transaction_ids=exports,heldout_exports_found=len(set(exports)&set(pool))),
        cause="R4 decoded cloud_end_frame from the historical R10B topic bag; SAME_OBJECTIVE T0/U_obs/source hashes belong to the later current-baseline scan-end deskew replay. The frozen P7 runner exports only the32 development source clouds.",
        requires_authority="Recover/export exact SAME_OBJECTIVE heldout scan-end sources with source count/FNV parity. Do not substitute source hashes, W2, T0, source preprocessing, oracle or heldout selection; any additional baseline replay NDT calls require explicit budget/authorization.",
        not_established=["heldout labels","candidate generalization","visual coverage","AUC","LOFO","GT sanity"],
        frozen_runner_source=dict(git_sha=c.HISTORY_SHA,path=runner_path,sha256=hashlib.sha256(source).hexdigest()),
        diagnostic_sha256={name:c.digest(c.OUT/name) for name in ("heldout_source_audit.csv",
            "historical_source_audit.csv","topic_vs_same_objective_raw_parity.csv",
            "frozen_p7_source_export_reference.txt","frozen_codegen_manifest.json")},
        source_auditor_sha256=c.digest(binary),push_executed=False)
    c.save_json(c.OUT/"input_stop_freeze.json",result)
    c.save_json(c.OUT/"results.json",result)
    print(json.dumps(dict(state=result["state"],first_frame=result["first_frame"],
        heldout_sources=result["heldout_sources"],historical_sources=result["historical_sources"],
        archived_source_exports=result["archived_source_exports"],NEW_NDT_CALLS=0,GT_LOADED=False),indent=2))


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary",type=Path,required=True)
    close(parser.parse_args().binary)
