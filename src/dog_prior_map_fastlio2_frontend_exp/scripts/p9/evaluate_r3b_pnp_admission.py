#!/usr/bin/env python3
"""Labels only after PnP SHA freeze; GT only after coverage SHA freeze."""
import argparse
import csv
from collections import Counter
import json
import tempfile
import numpy as np

import p9_r3_visual_contract as c
from run_r3b_pnp_admission import OUT,START,pinned,save,measurements,coverage_contract,quality_contract,receipt,coverage_values,quality_gate
from evaluate_r3a_metric_depth_coverage import statistics,score_rows,grouped_quality,write_scored


def coverage():
    frozen,plan=measurements();c.require(not (OUT/"coverage_freeze.json").exists(),"coverage already frozen")
    selected=c.read_csv(OUT/"selected_measurements.csv")
    label_path=c.OUT/"frame_statistics.csv";sha=pinned(label_path)
    labels={int(row["frame"]):row["label"] for row in c.read_csv(label_path)}
    c.require(set(labels)==set(plan["targets"]) and Counter(labels.values())==dict(MAJOR=9,NO_MAJOR=23),"label/cohort contract mismatch")
    summaries=coverage_values(selected,labels)
    c.require(summaries[0]["major_valid"]==4 and summaries[0]["no_major_valid"]==13,"R3A17 coverage parity FAIL")
    c.require(summaries[1]["valid"]==frozen["unlabelled_coverage"]["valid"],"label loading changed availability")
    c.write_csv(OUT/"coverage_summary.csv",summaries)
    regression=c.read_csv(OUT/"existing_valid_regression.csv")
    lost=sum(row["new_available"]=="0" for row in selected if row["old_available"]=="1")
    parity=c.read_csv(OUT/"parity.csv")
    passed=summaries[1]["major_valid"]>=6 and summaries[1]["no_major_valid"]>=14
    save("coverage_freeze.json",dict(measurement_freeze_sha256=c.digest(OUT/"measurement_freeze.json"),
        label_source=str(label_path),label_sha256=sha,artifacts={"coverage_summary.csv":c.digest(OUT/"coverage_summary.csv")},
        coverage=summaries,coverage_pass=passed,parity_pass=all(row["pass_flag"]=="1" for row in parity),
        lost_valid=lost,regression_pass=len(regression)==17 and lost==0,gt_loaded=False,ambiguity_auc_run=False))
    print("R3B_COVERAGE",summaries,"PASS",passed,"GT_LOADED=NO",flush=True)


def posthoc():
    gates,_=coverage_contract();c.require(not (OUT/"posthoc_quality.json").exists(),"posthoc already frozen")
    # No GT module or raw GT is imported/read before the two frozen stage contracts.
    from p3_r10c_failure_mechanism import read_gt
    path=c.DATA/"gt/floor01_gt.txt";gt_sha=c.digest(path);times,poses=read_gt(path)
    rows=c.read_csv(OUT/"admission_pairs.csv");by={(int(row["frame"]),int(row["lag"])):row for row in rows}
    selected=c.read_csv(OUT/"selected_measurements.csv");take=[]
    for chosen in selected:
        frame=int(chosen["frame"])
        if chosen["old_available"]=="1":
            take.append(dict(by[(frame,int(chosen["old_lag"]))],group="R3A_EXISTING_VALID_ORIGINAL_LAG",mode="R3B20_PARITY"))
            take.append(dict(by[(frame,int(chosen["new_lag"]))],group="R3A_EXISTING_VALID_NEW_SELECTED_LAG",mode="R3B20"))
        elif chosen["new_available"]=="1":
            row=by[(frame,int(chosen["new_lag"]))]
            c.require(20<=int(row["pnp_correspondences"])<30,"new availability not attributable to admission")
            take.append(dict(row,group="NEWLY_AVAILABLE_SELECTED",mode="R3B20"))
    take.extend(dict(row,group="ALL_LOW_SUPPORT_NEWLY_VALID",mode="R3B20") for row in rows
        if row["status"]=="VALID" and 20<=int(row["pnp_correspondences"])<30)
    scored=score_rows(take,None,times,poses);write_scored(OUT/"posthoc_gt.csv",scored)
    groups=grouped_quality(scored);passed=quality_gate(groups)
    overall=gates["coverage_pass"] and gates["parity_pass"] and gates["regression_pass"] and passed
    save("posthoc_quality.json",dict(coverage_freeze_sha256=c.digest(OUT/"coverage_freeze.json"),
        artifacts={"posthoc_gt.csv":c.digest(OUT/"posthoc_gt.csv")},gt_source=str(path),gt_sha256=gt_sha,
        groups=groups,quality_pass=passed,development_pnp_coverage_gate=overall,gt_used="POSTHOC_FRONTEND_ONLY",
        parameters_changed=False,ambiguity_auc_run=False))
    print("R3B_POSTHOC_QUALITY",groups,"PASS",passed,"DEVELOPMENT_GATE",overall,flush=True)


def full_metrics():
    gates,quality,plan=quality_contract();c.require(quality["development_pnp_coverage_gate"],"full metrics dev gate failed")
    frozen=receipt("full_measurement_freeze.json")
    c.require(frozen["execution_manifest_sha256"]==c.digest(OUT/"execution_manifest.json") and
        frozen["posthoc_quality_sha256"]==c.digest(OUT/"posthoc_quality.json") and
        frozen["full_run_started_sha256"]==c.digest(OUT/"full_run_started.json"),"full freeze lineage failed")
    started=json.loads((OUT/"full_run_started.json").read_text())
    c.require(started["posthoc_quality_sha256"]==c.digest(OUT/"posthoc_quality.json") and
        started["execution_manifest_sha256"]==c.digest(OUT/"execution_manifest.json") and started["attempt_min"]==20 and
        started["completion_parameters"]==plan["completion_parameters"],"full start lineage/parameters failed")
    from p3_r10c_failure_mechanism import read_gt
    gt_path=c.DATA/"gt/floor01_gt.txt";c.require(c.digest(gt_path)==quality["gt_sha256"],"GT changed")
    times,poses=read_gt(gt_path)
    rows=c.read_csv(OUT/"full_measurements.csv");c.require(len(rows)==4126,"full denominator invalid")
    counts={row["frame"]:int(row["pnp_correspondences"]) for row in rows}
    valid=[dict(row,group="FULL_ALL_VALID",mode="R3B20") for row in rows if row["status"]=="VALID"]
    scored=score_rows(valid,None,times,poses);write_scored(OUT/"full_gt.csv",scored)
    groups=grouped_quality(scored);summary=groups[0] if groups else None
    low=[dict(row,group="FULL_LOW_SUPPORT_VALID",mode="R3B20") for row in scored
        if 20<=counts[row["frame"]]<30]
    # Historical comparison is descriptive; all VALID image-time increments are the gate denominator.
    old=[row for row in c.read_csv(c.P4/"visual_increment.csv") if row["status"]=="VALID"]
    c.require(len(old)==1802,"historical1802 parity failed")
    old_ids={row["transaction_cur"] for row in old if row["paired_evaluable"]=="1"}
    intersection=[dict(row,group="HISTORICAL_VALID_INTERSECTION",mode="R3B20") for row in scored if row["frame"] in old_ids]
    passed=summary is not None and summary["missing_gt"]==0 and len(valid)/4126>=.60 and summary["translation"]["rmse"]<=.03517
    metric=dict(old_valid=1802,total_pairs=4126,new_valid=len(valid),new_coverage=len(valid)/4126,
        translation_rmse_m=None if summary is None else summary["translation"]["rmse"],
        rotation_rmse_deg=None if summary is None else summary["rotation"]["rmse"],
        low_support_valid=len(low),low_support_valid_fraction=len(low)/len(valid) if valid else 0,
        runtime_seconds=frozen["wall_seconds"],pass_flag=passed)
    c.write_csv(OUT/"full_floor01_metrics.csv",[metric])
    save("full_metrics.json",dict(metric=metric,groups=groups+grouped_quality(low)+grouped_quality(intersection),
        full_measurement_freeze_sha256=c.digest(OUT/"full_measurement_freeze.json"),
        gt_sha256=c.digest(gt_path),artifacts={name:c.digest(OUT/name) for name in ("full_floor01_metrics.csv","full_gt.csv")}))
    print("R3B_FULL_METRICS",metric,flush=True)


def self_test():
    import run_r3b_pnp_admission as engine
    old_out=engine.OUT;old_guard=engine.guard;old_pinned=engine.pinned;old_labels=c.OUT;old_data=c.DATA
    def must_fail(action):
        try:action()
        except (RuntimeError,FileExistsError):return
        raise AssertionError("modified lineage was accepted")
    with tempfile.TemporaryDirectory(prefix="p9_r3b_stage_",dir="/tmp") as directory:
        try:
            fixture=c.Path(directory);engine.OUT=fixture/"out";engine.OUT.mkdir();c.OUT=fixture/"labels";c.OUT.mkdir()
            c.DATA=fixture/"data";(c.DATA/"gt").mkdir(parents=True);(c.DATA/"gt/floor01_gt.txt").write_text("fixture only\n")
            engine.guard=lambda plan:None;engine.pinned=lambda path:c.digest(path)
            plan=dict(targets=list(range(32)),completion_parameters={});engine.save("execution_manifest.json",plan)
            label_path=c.OUT/"frame_statistics.csv"
            labels={frame:"MAJOR" if frame<9 else "NO_MAJOR" for frame in range(32)}
            c.write_csv(label_path,[dict(frame=frame,label=label) for frame,label in labels.items()])
            old_ids=set(range(4))|set(range(9,22));selected=[dict(frame=frame,old_available=int(frame in old_ids),
                old_lag=1 if frame in old_ids else "",new_available=int(frame in old_ids or frame==22),
                new_lag=1 if frame in old_ids or frame==22 else "") for frame in range(32)]
            c.write_csv(engine.OUT/"selected_measurements.csv",selected)
            c.write_csv(engine.OUT/"existing_valid_regression.csv",[dict(frame=frame,lost_valid=0) for frame in sorted(old_ids)])
            c.write_csv(engine.OUT/"parity.csv",[dict(pass_flag=1)])
            c.write_csv(engine.OUT/"admission_pairs.csv",[dict(frame=22,lag=1,status="VALID",pnp_correspondences=20)])
            names=("selected_measurements.csv","existing_valid_regression.csv","parity.csv","admission_pairs.csv")
            engine.save("measurement_freeze.json",dict(artifacts={name:c.digest(engine.OUT/name) for name in names},
                execution_manifest_sha256=c.digest(engine.OUT/"execution_manifest.json")))
            summaries=coverage_values(selected,labels);c.write_csv(engine.OUT/"coverage_summary.csv",summaries)
            gates=dict(artifacts={"coverage_summary.csv":c.digest(engine.OUT/"coverage_summary.csv")},
                measurement_freeze_sha256=c.digest(engine.OUT/"measurement_freeze.json"),label_source=str(label_path),label_sha256=c.digest(label_path),
                coverage=summaries,coverage_pass=False,parity_pass=True,lost_valid=0,regression_pass=True,gt_loaded=False,ambiguity_auc_run=False)
            engine.save("coverage_freeze.json",gates)
            scored=[]
            for frame in sorted(old_ids):
                for group,mode in (("R3A_EXISTING_VALID_ORIGINAL_LAG","R3B20_PARITY"),("R3A_EXISTING_VALID_NEW_SELECTED_LAG","R3B20")):
                    scored.append(dict(frame=frame,lag=1,group=group,mode=mode,gt_evaluable=1,
                        translation_error_m=.01,rotation_error_deg=.1,catastrophic_translation=0))
            for group in ("NEWLY_AVAILABLE_SELECTED","ALL_LOW_SUPPORT_NEWLY_VALID"):
                scored.append(dict(frame=22,lag=1,group=group,mode="R3B20",gt_evaluable=1,
                    translation_error_m=.26,rotation_error_deg=.1,catastrophic_translation=1))
            write_scored(engine.OUT/"posthoc_gt.csv",scored)
            quality=dict(artifacts={"posthoc_gt.csv":c.digest(engine.OUT/"posthoc_gt.csv")},
                coverage_freeze_sha256=c.digest(engine.OUT/"coverage_freeze.json"),gt_source=str(c.DATA/"gt/floor01_gt.txt"),
                gt_sha256=c.digest(c.DATA/"gt/floor01_gt.txt"),groups=grouped_quality(scored),quality_pass=False,
                development_pnp_coverage_gate=False,gt_used="POSTHOC_FRONTEND_ONLY",parameters_changed=False,ambiguity_auc_run=False)
            engine.save("posthoc_quality.json",quality)
            engine.quality_contract()
            original=(engine.OUT/"execution_manifest.json").read_bytes()
            engine.save("execution_manifest.json",dict(changed=True));must_fail(engine.quality_contract)
            (engine.OUT/"execution_manifest.json").write_bytes(original)
            engine.save("coverage_freeze.json",dict(gates,measurement_freeze_sha256="substituted"));must_fail(engine.coverage_contract)
            engine.save("coverage_freeze.json",dict(gates,coverage_pass=True));must_fail(engine.coverage_contract)
            engine.save("coverage_freeze.json",gates)
            engine.save("posthoc_quality.json",dict(quality,quality_pass=True,development_pnp_coverage_gate=True));must_fail(engine.quality_contract)
            engine.save("posthoc_quality.json",quality)
            original=label_path.read_bytes();label_path.write_text("changed\n");must_fail(engine.coverage_contract);label_path.write_bytes(original)
            engine.reserve_full_run(plan);must_fail(lambda:engine.reserve_full_run(plan))
            # Missing marker is not enough to permit overwrite of retained frozen output.
            (engine.OUT/"full_run_started.json").unlink();(engine.OUT/"full_measurements.csv").write_text("retained output\n")
            must_fail(lambda:engine.reserve_full_run(plan))
        finally:engine.OUT=old_out;engine.guard=old_guard;engine.pinned=old_pinned;c.OUT=old_labels;c.DATA=old_data
    sample=dict(group="ALL_LOW_SUPPORT_NEWLY_VALID",missing_gt=0,catastrophic_rate=.1)
    c.require(quality_gate([sample]) and not quality_gate([dict(sample,catastrophic_rate=.11)]),"frozen catastrophic boundary")
    must_fail(lambda:quality_gate([dict(sample,missing_gt=1)]))
    print("P9_R3B_INFORMATION_ORDER_QUALITY_GATE_SELF_TEST=PASS")


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("stage",choices=("coverage","posthoc","full-metrics","self-test"));args=parser.parse_args()
    {"coverage":coverage,"posthoc":posthoc,"full-metrics":full_metrics,"self-test":self_test}[args.stage]()
