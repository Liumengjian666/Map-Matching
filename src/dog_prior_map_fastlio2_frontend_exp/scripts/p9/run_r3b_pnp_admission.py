#!/usr/bin/env python3
"""Replay frozen R3A correspondences only; no labels/GT/images/depth extraction."""
import argparse
from collections import Counter,defaultdict
import hashlib
import json
import subprocess
import time
import numpy as np

import p9_r3_visual_contract as c
import run_r3a_metric_depth_coverage as r3a
from p9_r3b_pnp_frontend import solve,ATTEMPT_MIN,INLIER_MIN

START="15d2d38aae93201aa376c9f640524172e7900312"
BRANCH="research/p9-r3b-pnp-admission-realignment"
OUT=c.ROOT/"docs/p9_r3b_pnp_admission"
SOURCES=("p9_r3b_pnp_frontend.py","run_r3b_pnp_admission.py","evaluate_r3b_pnp_admission.py","run_r3b_full_floor01.py")


def save(name,value):
    (OUT/name).write_text(json.dumps(value,indent=2,sort_keys=True,allow_nan=False)+"\n")


def pinned(path):
    original=subprocess.check_output(["git","show",START+":"+path.relative_to(c.ROOT).as_posix()],cwd=c.ROOT)
    sha=hashlib.sha256(original).hexdigest()
    c.require(c.digest(path)==sha,"committed R3A input/source changed: "+str(path))
    return sha


def receipt(name):
    value=json.loads((OUT/name).read_text())
    for path,sha in value["artifacts"].items():c.require(c.digest(OUT/path)==sha,"R3B frozen artifact changed: "+path)
    return value


def guard(plan):
    c.require(ATTEMPT_MIN==plan["attempt_min"]==20 and INLIER_MIN==plan["inlier_min"]==20,"PnP contract changed")
    for path,sha in plan["source_sha256"].items():c.require(c.digest(path)==sha,"frozen R3B source changed: "+path)
    for path,sha in plan["input_sha256"].items():c.require(c.digest(path)==sha,"frozen R3B input changed: "+path)
    old=json.loads((r3a.OUT/"execution_manifest.json").read_text());r3a.source_guard(old)


def measurements():
    frozen=receipt("measurement_freeze.json")
    c.require(frozen["execution_manifest_sha256"]==c.digest(OUT/"execution_manifest.json"),"measurement-plan lineage changed")
    plan=json.loads((OUT/"execution_manifest.json").read_text());guard(plan)
    return frozen,plan


def coverage_contract():
    _,plan=measurements();gates=receipt("coverage_freeze.json")
    c.require(gates["measurement_freeze_sha256"]==c.digest(OUT/"measurement_freeze.json"),"coverage lineage changed")
    label_path=c.OUT/"frame_statistics.csv"
    c.require(gates["label_source"]==str(label_path) and gates["label_sha256"]==pinned(label_path),"frozen label input changed")
    labels={int(row["frame"]):row["label"] for row in c.read_csv(label_path)}
    c.require(set(labels)==set(plan["targets"]) and Counter(labels.values())==dict(MAJOR=9,NO_MAJOR=23),"frozen label cohort changed")
    selected=c.read_csv(OUT/"selected_measurements.csv")
    summaries=coverage_values(selected,labels)
    observed=c.read_csv(OUT/"coverage_summary.csv")
    c.require(len(observed)==len(summaries) and all(row=={key:str(value) for key,value in expected.items()}
        for row,expected in zip(observed,summaries)),"coverage CSV does not match frozen availability")
    lost=sum(row["new_available"]=="0" for row in selected if row["old_available"]=="1")
    regression=c.read_csv(OUT/"existing_valid_regression.csv");parity=c.read_csv(OUT/"parity.csv")
    expected=dict(coverage=summaries,coverage_pass=summaries[1]["major_valid"]>=6 and summaries[1]["no_major_valid"]>=14,
        parity_pass=bool(parity) and all(row["pass_flag"]=="1" for row in parity),lost_valid=lost,
        regression_pass=len(regression)==17 and lost==0,gt_loaded=False,ambiguity_auc_run=False)
    c.require(all(gates[key]==value for key,value in expected.items()),"coverage gate metadata disagrees with frozen CSV evidence")
    return gates,plan


def quality_contract():
    gates,plan=coverage_contract();quality=receipt("posthoc_quality.json")
    c.require(quality["coverage_freeze_sha256"]==c.digest(OUT/"coverage_freeze.json"),"quality lineage changed")
    gt_path=c.DATA/"gt/floor01_gt.txt"
    c.require(quality["gt_source"]==str(gt_path) and quality["gt_sha256"]==c.digest(gt_path),"posthoc GT input changed")
    from evaluate_r3a_metric_depth_coverage import grouped_quality
    scored=[]
    for row in c.read_csv(OUT/"posthoc_gt.csv"):
        row["gt_evaluable"]=int(row["gt_evaluable"])
        if row["gt_evaluable"]:
            row["translation_error_m"]=float(row["translation_error_m"]);row["rotation_error_deg"]=float(row["rotation_error_deg"])
            row["catastrophic_translation"]=int(row["catastrophic_translation"])
            c.require(row["catastrophic_translation"]==int(row["translation_error_m"]>.25),"catastrophic flag changed")
        scored.append(row)
    expected=Counter()
    for chosen in c.read_csv(OUT/"selected_measurements.csv"):
        frame=chosen["frame"]
        if chosen["old_available"]=="1":
            expected[(frame,chosen["old_lag"],"R3A_EXISTING_VALID_ORIGINAL_LAG","R3B20_PARITY")]+=1
            expected[(frame,chosen["new_lag"],"R3A_EXISTING_VALID_NEW_SELECTED_LAG","R3B20")]+=1
        elif chosen["new_available"]=="1":expected[(frame,chosen["new_lag"],"NEWLY_AVAILABLE_SELECTED","R3B20")]+=1
    for row in c.read_csv(OUT/"admission_pairs.csv"):
        if row["status"]=="VALID" and 20<=int(row["pnp_correspondences"])<30:
            expected[(row["frame"],row["lag"],"ALL_LOW_SUPPORT_NEWLY_VALID","R3B20")]+=1
    c.require(Counter((row["frame"],row["lag"],row["group"],row["mode"]) for row in scored)==expected,
        "posthoc group denominator does not match frozen PnP/selection")
    groups=grouped_quality(scored);passed=quality_gate(groups)
    overall=gates["coverage_pass"] and gates["parity_pass"] and gates["regression_pass"] and passed
    expected=dict(groups=groups,quality_pass=passed,development_pnp_coverage_gate=overall,
        gt_used="POSTHOC_FRONTEND_ONLY",parameters_changed=False,ambiguity_auc_run=False)
    c.require(all(quality[key]==value for key,value in expected.items()),"quality gate metadata disagrees with frozen CSV evidence")
    return gates,quality,plan


def coverage_values(selected,labels):
    summaries=[]
    for method,field in (("R3A_AUGMENTED30","old_available"),("R3B_AUGMENTED20","new_available")):
        counts={label:sum(int(row[field]) for row in selected if labels[int(row["frame"])]==label) for label in ("MAJOR","NO_MAJOR")}
        summaries.append(dict(method=method,total=32,valid=sum(counts.values()),major_valid=counts["MAJOR"],no_major_valid=counts["NO_MAJOR"]))
    return summaries


def quality_gate(groups):
    c.require(all(row["missing_gt"]==0 for row in groups),"missing GT cannot silently pass")
    new=[row for row in groups if row["group"] in ("NEWLY_AVAILABLE_SELECTED","ALL_LOW_SUPPORT_NEWLY_VALID")]
    return all(row["catastrophic_rate"] is not None and row["catastrophic_rate"]<=.10 for row in new)


def reserve_full_run(plan):
    names=("full_run_started.json","full_measurements.csv","full_measurement_freeze.json","full_gt.csv",
        "full_floor01_metrics.csv","full_metrics.json")
    c.require(not any((OUT/name).exists() for name in names),"full run/output already exists: no repeat/overwriting")
    value=dict(posthoc_quality_sha256=c.digest(OUT/"posthoc_quality.json"),
        execution_manifest_sha256=c.digest(OUT/"execution_manifest.json"),attempt_min=20,
        completion_parameters=plan["completion_parameters"])
    # Exclusive creation prevents concurrent launches even after the output check.
    with (OUT/"full_run_started.json").open("x") as stream:
        json.dump(value,stream,indent=2,sort_keys=True,allow_nan=False);stream.write("\n")


def freeze_plan():
    c.require(not (OUT/"execution_manifest.json").exists(),"R3B already frozen: do not overwrite replay")
    c.require(subprocess.check_output(["git","branch","--show-current"],cwd=c.ROOT,text=True).strip()==BRANCH,"wrong R3B branch")
    c.require(subprocess.check_output(["git","rev-parse","HEAD"],cwd=c.ROOT,text=True).strip()==START,"R3B start SHA changed before experiment")
    old_receipt,old_plan=r3a.measurement_contract()
    files=[r3a.OUT/name for name in ("measurement_freeze.json","execution_manifest.json","artifact_hashes.json",
        "coverage_by_pair.csv","depth_completion_correspondences.csv","pair_manifest.csv","selected_measurements.csv")]
    hashes={str(path):pinned(path) for path in files}
    old_hashes=json.loads((r3a.OUT/"artifact_hashes.json").read_text())
    for path in files:
        if path.name in old_hashes["artifacts"]:
            c.require(hashes[str(path)]==old_hashes["artifacts"][path.name],"R3A final artifact chain changed")
    for path in (c.DATA/"calibration/floor01_intrinsics.yaml",c.DATA/"calibration/floor01_extrinsics.yaml"):
        sha=c.digest(path);c.require(sha==old_plan["input_sha256"][str(path)],"calibration changed");hashes[str(path)]=sha
    sources=[c.HERE/name for name in SOURCES]+[OUT/"THEORY.md"]
    for path in (c.PACKAGE/"scripts/p4_i3_visual_frontend.py",c.PACKAGE/"scripts/p4_i3_visual_increment.py",
        c.PACKAGE/"scripts/p3_r10c_failure_mechanism.py",c.HERE/"p9_r3a_depth_completion.py",
        c.HERE/"p9_r3a_visual_frontend.py",c.HERE/"p9_r3_visual_contract.py",c.HERE/"evaluate_r3a_metric_depth_coverage.py"):
        pinned(path);sources.append(path)
    plan=dict(task="PAPER-P9-R3B-METRIC-PNP-ADMISSION-CONTRACT-REALIGNMENT",branch=BRANCH,start_sha=START,
        attempt_min=20,old_attempt_min=30,inlier_min=20,pnp="UNCHANGED_P4_EPNP100_2PX_0P99_LM_FINITE_POSITIVE_Z",
        lags=list(c.LAGS),selection="SMALLEST_VALID_LAG",cohort="DEVELOPMENT_ONLY",targets=old_plan["targets"],
        completion_parameters=old_plan["completion_parameters"],numerical_environment=old_plan["numerical_environment"],
        input_sha256=hashes,source_sha256={str(path):c.digest(path) for path in sources},
        r3a_measurement_sha256=hashes[str(r3a.OUT/"measurement_freeze.json")],
        new_image_extraction=0,new_depth_completion=0,new_ndt_calls=0,labels_loaded=False,gt_loaded=False)
    save("execution_manifest.json",plan)
    return plan


def carriers(items,count):
    ids=[int(row["feature_index"]) for row in items]
    c.require(ids==sorted(ids) and len(ids)==len(set(ids)),"frozen FB correspondence order invalid")
    accepted=[row for row in items if row["depth_source"] in ("DIRECT","PLANE_COMPLETED")]
    c.require(len(accepted)==count,"frozen depth count mismatch")
    xyz=np.array([c.vector(row["point_xyz"],3) for row in accepted],dtype=float).reshape(-1,3)
    ref=np.array([[float(row["ref_u"]),float(row["ref_v"])] for row in accepted],float).reshape(-1,2)
    cur=np.array([[float(row["cur_u"]),float(row["cur_v"])] for row in accepted],float).reshape(-1,2)
    c.require(np.array_equal(cur,cur.astype(np.float32).astype(float)),"current pixel float32 carrier roundtrip failed")
    c.require(np.array_equal(ref,ref.astype(np.float32).astype(float)),"reference pixel float32 carrier roundtrip failed")
    c.require(all(np.array_equal(c.vector(c.text(point),3),point) for point in xyz),"metric XYZ carrier roundtrip failed")
    sha=hashlib.sha256(xyz.astype("<f8").tobytes()+ref.astype("<f8").tobytes()+cur.astype("<f8").tobytes()).hexdigest()
    return xyz,ref,cur,sha


def parity_control(previous,current):
    c.require(previous["status"]==current["status"] and int(previous["pnp_inliers"])==current["pnp_inliers"],">=30 PnP status/inlier parity FAIL")
    dt=dr=de=""
    if previous["T_Ccur_Cref"]:
        a=c.vector(previous["T_Ccur_Cref"],16).reshape(4,4);b=c.vector(current["T_Ccur_Cref"],16).reshape(4,4)
        dt,dr=c.separation(a,b)
        c.require(dt<=1e-10 and dr<=1e-8,">=30 transform parity FAIL")
    if previous["reprojection_rmse_px"]:
        de=abs(float(previous["reprojection_rmse_px"])-float(current["reprojection_rmse_px"]))
        c.require(de<=1e-10,">=30 reprojection parity FAIL")
    return dict(frame=previous["frame"],lag=previous["lag"],corr=int(previous["pnp_correspondences"]),
        old_status=previous["status"],new_status=current["status"],old_inliers=int(previous["pnp_inliers"]),new_inliers=current["pnp_inliers"],
        translation_diff_m=dt,rotation_diff_deg=dr,reprojection_diff_px=de,pass_flag=1)


def replay():
    started=time.perf_counter();plan=freeze_plan()
    old=[row for row in c.read_csv(r3a.OUT/"coverage_by_pair.csv") if row["mode"]=="NEW_AUGMENTED"]
    c.require(len(old)==128,"R3A augmented pair count invalid")
    groups=defaultdict(list)
    for row in c.read_csv(r3a.OUT/"depth_completion_correspondences.csv"):groups[(int(row["frame"]),int(row["lag"]))].append(row)
    calibration=c.frontend.load_calibration(c.DATA/"calibration");c.frontend.cv2.setNumThreads(1)
    pairs={(int(row["frame"]),int(row["lag"])):row for row in c.read_csv(r3a.OUT/"pair_manifest.csv")}
    output=[];parity=[];low=[]
    for previous in old:
        frame,lag=int(previous["frame"]),int(previous["lag"]);pair=pairs[(frame,lag)]
        count=int(previous["pnp_correspondences"])
        keys=("frame","lag","transaction_ref","timestamp_ref_ns","timestamp_cur_ns","scan_ref_ns","scan_cur_ns")
        row={name:previous[name] for name in keys}
        xyz,ref,cur,sha=carriers(groups[(frame,lag)],count)
        result=solve(xyz,ref,cur,calibration,frame) if previous["attempted"]=="1" else solve(xyz[:0],ref[:0],cur[:0],calibration,frame)
        row.update(result,old_status=previous["status"],old_correspondences=count,carrier_sha256=sha,
            sync_eligible=int(pair["sync_ref_eligible"]==pair["sync_cur_eligible"]=="1"),
            direct_depth_count=previous["direct_depth_count"],plane_completed_count=previous["plane_completed_count"])
        if previous["attempted"]!="1":row["status"]=previous["status"]
        c.require(bool(row["pnp_attempted"])==(count>=20 and previous["attempted"]=="1"),"admission identity failed")
        if count>=30:parity.append(parity_control(previous,row))
        if 20<=count<30 and previous["attempted"]=="1":low.append(row)
        output.append(row)
        print("PNP_REPLAY",frame,lag,"CORR",count,"INLIERS",row["pnp_inliers"],row["status"],flush=True)
    c.write_csv(OUT/"admission_pairs.csv",output);c.write_csv(OUT/"parity.csv",parity);c.write_csv(OUT/"low_support_pnp.csv",low)
    selected=[];regression=[]
    before={int(row["frame"]):row for row in c.read_csv(r3a.OUT/"selected_measurements.csv")}
    by={(int(row["frame"]),int(row["lag"])):row for row in output}
    for frame in plan["targets"]:
        best=c.choose_smallest_valid([by[(frame,lag)] for lag in c.LAGS]);old_selected=before[frame]
        selected.append(dict(frame=frame,old_available=old_selected["new_available"],old_lag=old_selected["new_lag"],
            new_available=int(best is not None),new_lag="" if best is None else best["lag"],
            newly_available=int(old_selected["new_available"]=="0" and best is not None)))
        if old_selected["new_available"]=="1":
            same=by[(frame,int(old_selected["new_lag"]))]
            c.require(same["status"]=="VALID" and int(same["pnp_correspondences"])>=30,"lost R3A valid/original-lag measurement")
            regression.append(dict(frame=frame,old_lag=old_selected["new_lag"],new_lag="" if best is None else best["lag"],original_lag_status=same["status"],lost_valid=0))
    c.require(len(regression)==17,"R3A17 regression denominator failed")
    c.write_csv(OUT/"selected_measurements.csv",selected);c.write_csv(OUT/"existing_valid_regression.csv",regression)
    names=("admission_pairs.csv","parity.csv","low_support_pnp.csv","selected_measurements.csv","existing_valid_regression.csv")
    guard(plan)
    save("measurement_freeze.json",dict(execution_manifest_sha256=c.digest(OUT/"execution_manifest.json"),
        artifacts={name:c.digest(OUT/name) for name in names},unlabelled_coverage=dict(total=32,valid=sum(row["new_available"] for row in selected)),
        pair_count=128,parity_controls=len(parity),low_support_pairs=len(low),existing_valid=17,lost_valid=0,
        new_image_extraction=0,new_depth_completion=0,new_ndt_calls=0,gt_loaded=False,labels_loaded=False,
        replay_wall_seconds=time.perf_counter()-started))
    print("R3B_PNP_MEASUREMENTS_SELECTED_LAGS_UNLABELLED_COVERAGE_FROZEN=YES",flush=True)


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("stage",choices=("replay",));args=parser.parse_args();replay()
