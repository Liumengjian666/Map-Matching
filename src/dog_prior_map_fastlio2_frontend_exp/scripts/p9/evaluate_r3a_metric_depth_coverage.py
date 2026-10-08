#!/usr/bin/env python3
"""Coverage/geometry gate first; GT reads only after explicit SHA256 freezes."""
import argparse
import csv
from collections import Counter
import json

import numpy as np
from scipy.spatial.transform import Rotation

import p9_r3_visual_contract as c
from run_r3a_metric_depth_coverage import OUT, START, pinned, save_json, source_guard, verify_freeze, measurement_contract, development_contract


def statistics(values):
    a=np.asarray(values,dtype=float)
    c.require(np.isfinite(a).all(),"nonfinite metric must remain explicit missing")
    if not len(a):return dict(count=0,mean=None,median=None,rmse=None,p95=None,max=None)
    return dict(count=len(a),mean=float(a.mean()),median=float(np.median(a)),
        rmse=float(np.sqrt(np.mean(a*a))),p95=float(np.percentile(a,95)),max=float(a.max()))


def matrix(value):
    return c.vector(value,16).reshape(4,4)


def difference(first,second):
    residual=np.linalg.inv(first)@second
    return float(np.linalg.norm(residual[:3,3])),float(np.degrees(Rotation.from_matrix(residual[:3,:3]).magnitude()))


def geometry_gate(regressions):
    missing=sum(row["new_status"]!="VALID" for row in regressions)
    dt=statistics([row["translation_difference_m"] for row in regressions if row["new_status"]=="VALID"])
    dr=statistics([row["rotation_difference_deg"] for row in regressions if row["new_status"]=="VALID"])
    passed=len(regressions)==12 and missing==0 and dt["median"]<=.02 and dt["p95"]<=.05 and dr["median"]<=.5 and dr["p95"]<=1.
    return dict(pass_flag=bool(passed),lost_old_valid=missing,translation=dt,rotation=dr)


def development():
    receipt,plan=measurement_contract()
    rows=c.read_csv(OUT/"coverage_by_pair.csv");c.require(len(rows)==256,"development measurement count")
    frames=plan["targets"];by={(int(row["frame"]),int(row["lag"]),row["mode"]):row for row in rows}
    selected=[];regression=[]
    for frame in frames:
        modes={mode:c.choose_smallest_valid([by[(frame,lag,mode)] for lag in c.LAGS]) for mode in ("OLD_DIRECT_ONLY","NEW_AUGMENTED")}
        old,new=modes["OLD_DIRECT_ONLY"],modes["NEW_AUGMENTED"]
        selected.append(dict(frame=frame,old_available=int(old is not None),old_lag="" if old is None else old["lag"],
            new_available=int(new is not None),new_lag="" if new is None else new["lag"],
            newly_recovered=int(old is None and new is not None)))
        if old is not None:
            current=by[(frame,int(old["lag"]),"NEW_AUGMENTED")]
            dt,dr=difference(matrix(old["D_vis"]),matrix(current["D_vis"])) if current["status"]=="VALID" else ("","")
            regression.append(dict(frame=frame,lag=old["lag"],old_status=old["status"],new_status=current["status"],
                translation_difference_m=dt,rotation_difference_deg=dr,old_inliers=int(old["pnp_inliers"]),new_inliers=int(current["pnp_inliers"]),
                inlier_change=int(current["pnp_inliers"])-int(old["pnp_inliers"]),
                old_reprojection_rmse_px=old["reprojection_rmse_px"],new_reprojection_rmse_px=current["reprojection_rmse_px"],
                reprojection_change_px=float(current["reprojection_rmse_px"])-float(old["reprojection_rmse_px"]) if current["reprojection_rmse_px"] else ""))
    c.require(len(regression)==12,"historical12 VALID pair parity failed")
    c.write_csv(OUT/"selected_measurements.csv",selected);c.write_csv(OUT/"old_valid_regression.csv",regression)
    safety=geometry_gate(regression)
    # Label access is confined to this stage AFTER image/depth measurements froze.
    label_path=c.OUT/"frame_statistics.csv";label_sha=pinned(label_path)
    labels={int(row["frame"]):row["label"] for row in c.read_csv(label_path)}
    c.require(set(labels)==set(frames) and Counter(labels.values())==dict(MAJOR=9,NO_MAJOR=23),"development label contract invalid")
    coverage=[]
    for mode in ("OLD_DIRECT_ONLY","NEW_AUGMENTED"):
        field="old_available" if mode=="OLD_DIRECT_ONLY" else "new_available"
        counts={label:sum(int(row[field]) for row in selected if labels[row["frame"]]==label) for label in ("MAJOR","NO_MAJOR")}
        coverage.append(dict(mode=mode,total=32,valid=sum(counts.values()),major_valid=counts["MAJOR"],no_major_valid=counts["NO_MAJOR"]))
    c.require(coverage[0]["major_valid"]==2 and coverage[0]["no_major_valid"]==10,"old group parity failed")
    passed=coverage[1]["major_valid"]>=6 and coverage[1]["no_major_valid"]>=14
    c.write_csv(OUT/"coverage_summary.csv",coverage)
    key=[row for row in rows if int(row["frame"]) in (368,616,2226,2350,2846,3341) and row["mode"]=="NEW_AUGMENTED"]
    c.write_csv(OUT/"key_frame_funnel.csv",key)
    augmented=[row for row in rows if row["mode"]=="NEW_AUGMENTED"]
    costs=[]
    fields=("projection_ms","direct_association_ms","neighbor_search_ms","plane_fitting_ms","completion_ms","pnp_ms","total_ms")
    for field in fields:
        for subset,items in (("ALL_128",augmented),("ATTEMPTED",[row for row in augmented if row["attempted"]=="1"])):
            costs.append(dict(component=field,subset=subset,**statistics([float(row[field]) for row in items])))
    early=[];brute=[]
    for frame in frames:
        total=0.
        for lag in c.LAGS:
            row=by[(frame,lag,"NEW_AUGMENTED")];total+=float(row["total_ms"])
            if row["status"]=="VALID":break
        early.append(total);brute.append(sum(float(by[(frame,lag,"NEW_AUGMENTED")]["total_ms"]) for lag in c.LAGS))
    c.write_csv(OUT/"runtime_breakdown.csv",costs)
    names=("coverage_by_pair.csv","depth_completion_correspondences.csv","direct_parity.csv","selected_measurements.csv",
        "old_valid_regression.csv","coverage_summary.csv","key_frame_funnel.csv","runtime_breakdown.csv")
    save_json("development_gate_freeze.json",dict(artifacts={name:c.digest(OUT/name) for name in names},
        measurement_freeze_sha256=c.digest(OUT/"measurement_freeze.json"),coverage=coverage,coverage_pass=passed,
        geometry=safety,geometry_safety_pass=safety["pass_flag"],cohort="DEVELOPMENT",label_source_sha256=label_sha,
        gt_loaded=False,auc_run=False,early_stop_estimate_ms=statistics(early),four_lag_cost_ms=statistics(brute)))
    print("DEVELOPMENT_COVERAGE",coverage,"GEOMETRY_SAFETY",safety,"GT_LOADED=NO",flush=True)


def score_rows(rows,calibration,gt_times,gt_poses):
    # Imported only inside the GT stage; uses the existing image-time IMU convention.
    from p3_r10c_failure_mechanism import interpolate_gt
    scored=[]
    for row in rows:
        ref=interpolate_gt(gt_times,gt_poses,int(row["timestamp_ref_ns"])*1e-9)
        cur=interpolate_gt(gt_times,gt_poses,int(row["timestamp_cur_ns"])*1e-9)
        item=dict(frame=row["frame"],lag=row["lag"],group=row["group"],mode=row["mode"],gt_evaluable=int(ref is not None and cur is not None),
            translation_error_m="",rotation_error_deg="",catastrophic_translation="")
        if ref is not None and cur is not None:
            # D_vis was frozen from T_IC inverse(PnP) inverse(T_IC).
            dt,dr=difference(np.linalg.inv(ref)@cur,matrix(row["D_vis"]))
            item.update(translation_error_m=dt,rotation_error_deg=dr,catastrophic_translation=int(dt>.25))
        scored.append(item)
    return scored


def grouped_quality(rows):
    result=[]
    for group,mode in sorted({(row["group"],row["mode"]) for row in rows}):
        subset=[row for row in rows if (row["group"],row["mode"])==(group,mode)]
        valid=[row for row in subset if row["gt_evaluable"]==1]
        cats=sum(row["catastrophic_translation"] for row in valid)
        result.append(dict(group=group,mode=mode,total=len(subset),gt_evaluable=len(valid),missing_gt=len(subset)-len(valid),
            translation=statistics([row["translation_error_m"] for row in valid]),rotation=statistics([row["rotation_error_deg"] for row in valid]),
            catastrophic_count=cats,catastrophic_rate=cats/len(valid) if valid else None))
    return result


def posthoc():
    gates,_=development_contract()
    c.require(not (OUT/"posthoc_quality.json").exists(),"posthoc stage already frozen")
    from p3_r10c_failure_mechanism import read_gt
    gt_path=c.DATA/"gt/floor01_gt.txt";gt_sha=c.digest(gt_path);times,poses=read_gt(gt_path)
    rows=c.read_csv(OUT/"coverage_by_pair.csv");by={(int(row["frame"]),int(row["lag"]),row["mode"]):row for row in rows}
    selected=c.read_csv(OUT/"selected_measurements.csv");take=[]
    for chosen in selected:
        frame=int(chosen["frame"])
        if chosen["old_available"]=="1":
            for mode in ("OLD_DIRECT_ONLY","NEW_AUGMENTED"):
                row=by[(frame,int(chosen["old_lag"]),mode)]
                if row["status"]=="VALID":take.append(dict(row,group="OLD_VALID"))
        elif chosen["new_available"]=="1":
            take.append(dict(by[(frame,int(chosen["new_lag"]),"NEW_AUGMENTED")],group="NEWLY_RECOVERED"))
    scored=score_rows(take,p4_calibration(),times,poses)
    write_scored(OUT/"newly_recovered_gt.csv",scored)
    groups=grouped_quality(scored);new=[row for row in groups if row["group"]=="NEWLY_RECOVERED"]
    missing=sum(row["missing_gt"] for row in groups)
    quality=missing==0 and (not new or new[0]["catastrophic_rate"]<=.10)
    save_json("posthoc_quality.json",dict(development_gate_sha256=c.digest(OUT/"development_gate_freeze.json"),
        gt_source=str(gt_path),gt_sha256=gt_sha,groups=groups,quality_pass=quality,missing_gt=missing,
        artifacts={"newly_recovered_gt.csv":c.digest(OUT/"newly_recovered_gt.csv")},gt_used="POSTHOC_FRONTEND_ONLY",
        auc_run=False,parameters_changed=False))
    print("POSTHOC_FRONTEND_QUALITY",groups,"PASS",quality,flush=True)


def write_scored(path,rows):
    fields=("frame","lag","group","mode","gt_evaluable","translation_error_m","rotation_error_deg","catastrophic_translation")
    with path.open("w",newline="") as stream:
        writer=csv.DictWriter(stream,fieldnames=fields,lineterminator="\n");writer.writeheader();writer.writerows(rows)


def p4_calibration():
    return c.frontend.load_calibration(c.DATA/"calibration")


def full_metrics():
    gates,plan=development_contract()
    c.require(gates["coverage_pass"] and gates["geometry_safety_pass"],"full metrics require passed development gate")
    quality=verify_freeze("posthoc_quality.json")
    c.require(quality["quality_pass"] and quality["development_gate_sha256"]==c.digest(OUT/"development_gate_freeze.json"),"full quality lineage invalid")
    freeze=verify_freeze("full_measurement_freeze.json")
    c.require(freeze["execution_manifest_sha256"]==c.digest(OUT/"execution_manifest.json") and
        freeze["full_run_started_sha256"]==c.digest(OUT/"full_run_started.json"),"full measurement lineage invalid")
    started=json.loads((OUT/"full_run_started.json").read_text())
    c.require(started["development_gate_sha256"]==c.digest(OUT/"development_gate_freeze.json") and
        started["posthoc_quality_sha256"]==c.digest(OUT/"posthoc_quality.json") and
        started["completion_parameters"]==plan["completion_parameters"],"full run start contract changed")
    from p3_r10c_failure_mechanism import read_gt
    gt_path=c.DATA/"gt/floor01_gt.txt"
    c.require(c.digest(gt_path)==quality["gt_sha256"],"GT changed")
    times,poses=read_gt(gt_path)
    rows=c.read_csv(OUT/"full_measurements.csv");valid=[dict(row,group="FULL_ALL_VALID") for row in rows if row["status"]=="VALID"]
    c.require(len(rows)==4126,"full metrics denominator mismatch")
    scored=score_rows(valid,p4_calibration(),times,poses)
    write_scored(OUT/"full_visual_gt.csv",scored)
    groups=grouped_quality(scored);all_quality=groups[0] if groups else None
    old=[row for row in c.read_csv(c.P4/"visual_increment.csv") if row["status"]=="VALID"]
    c.require(len(old)==1802,"historical full coverage parity fail")
    old_ids={row["transaction_cur"] for row in old if row["paired_evaluable"]=="1"}
    paired=[dict(row,group="HISTORICAL_VALID_INTERSECTION") for row in scored if row["frame"] in old_ids]
    group_paired=grouped_quality(paired)
    passed=full_gate(len(valid),all_quality)
    metric=dict(old_valid=1802,total_pairs=4126,new_valid=len(valid),new_coverage=len(valid)/4126,
        translation_rmse_m=None if all_quality is None else all_quality["translation"]["rmse"],
        rotation_rmse_deg=None if all_quality is None else all_quality["rotation"]["rmse"],pass_flag=passed,
        runtime_seconds=freeze["wall_seconds"])
    c.write_csv(OUT/"full_floor01_metrics.csv",[metric])
    save_json("full_metrics.json",dict(metric=metric,groups=groups+group_paired,
        full_measurement_freeze_sha256=c.digest(OUT/"full_measurement_freeze.json"),
        gt_sha256=c.digest(gt_path),artifacts={name:c.digest(OUT/name) for name in ("full_floor01_metrics.csv","full_visual_gt.csv")}))
    print("FULL_FLOOR01_METRICS",metric,flush=True)


def full_gate(valid_count,quality):
    return bool(quality is not None and quality["missing_gt"]==0 and valid_count/4126>=.60 and
        quality["translation"]["rmse"] is not None and quality["translation"]["rmse"]<=.03517)


def guard_self_test():
    import tempfile
    import run_r3a_metric_depth_coverage as engine
    previous=engine.OUT
    def must_fail(action):
        try:action()
        except RuntimeError:return
        raise AssertionError("altered contract unexpectedly passed")
    with tempfile.TemporaryDirectory(prefix="p9_r3a_guard_",dir="/tmp") as directory:
        try:
            engine.OUT=c.Path(directory)
            pair=engine.OUT/"pair_manifest.csv";pair.write_text("fixture-pair\n")
            source=engine.OUT/"source.py";source.write_text("fixture-source\n")
            plan=dict(completion_parameters=dict(engine.PARAMETERS),construction_source_sha256={str(source):c.digest(source)},
                pair_manifest_sha256=c.digest(pair),numerical_environment=engine.numerical_environment())
            engine.save_json("execution_manifest.json",plan)
            engine.save_json("measurement_freeze.json",dict(artifacts={"pair_manifest.csv":c.digest(pair)},
                execution_manifest_sha256=c.digest(engine.OUT/"execution_manifest.json")))
            engine.save_json("development_gate_freeze.json",dict(artifacts={},
                measurement_freeze_sha256=c.digest(engine.OUT/"measurement_freeze.json")))
            engine.development_contract()
            original=(engine.OUT/"execution_manifest.json").read_bytes()
            engine.save_json("execution_manifest.json",dict(plan,completion_parameters={}))
            must_fail(engine.development_contract)
            (engine.OUT/"execution_manifest.json").write_bytes(original)
            source.write_text("changed-source\n");must_fail(engine.development_contract)
            source.write_text("fixture-source\n")
            pair.write_text("changed-pair\n");must_fail(engine.development_contract)
            pair.write_text("fixture-pair\n")
            changed=dict(plan,numerical_environment=dict(plan["numerical_environment"],versions={}))
            must_fail(lambda:engine.source_guard(changed))
            alternate=dict(plan["numerical_environment"]["binary_sha256"],**{str(source):c.digest(source)})
            changed=dict(plan,numerical_environment=dict(plan["numerical_environment"],binary_sha256=alternate))
            must_fail(lambda:engine.source_guard(changed))  # exists on disk, but not an actually loaded binary
        finally:engine.OUT=previous


def self_test():
    rows=[dict(new_status="VALID",translation_difference_m=.001*i,rotation_difference_deg=.02*i) for i in range(12)]
    c.require(geometry_gate(rows)["pass_flag"],"synthetic geometry gate")
    broken=[dict(row) for row in rows];broken[-1]["new_status"]="PNP_REJECT"
    c.require(not geometry_gate(broken)["pass_flag"],"lost old measurement passed")
    broken=[dict(row) for row in rows];broken[-1]["translation_difference_m"]=.3
    c.require(not geometry_gate(broken)["pass_flag"],"unsafe P95 passed")
    c.require(statistics([])["median"] is None and statistics([1,2])["rmse"]==float(np.sqrt(2.5)),"empty/metric semantics")
    first=np.eye(4);second=np.eye(4);second[:3,3]=[.01,0,0]
    c.require(abs(difference(first,second)[0]-.01)<1e-12,"relative metric units")
    import tempfile
    with tempfile.TemporaryDirectory(prefix="p9_r3a_empty_score_",dir="/tmp") as directory:
        path=c.Path(directory)/"empty.csv";write_scored(path,[])
        c.require(c.read_csv(path)==[] and "gt_evaluable" in path.read_text(),"zero-valid full score archive must preserve header")
    c.require(not full_gate(0,None),"zero-valid full run must FAIL without a division/error")
    guard_self_test()
    print("P9_R3A_COVERAGE_GEOMETRY_STAGE_GATE_SELF_TEST=PASS")


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("stage",choices=("development","posthoc","full-metrics","self-test"));args=parser.parse_args()
    {"development":development,"posthoc":posthoc,"full-metrics":full_metrics,"self-test":self_test}[args.stage]()
