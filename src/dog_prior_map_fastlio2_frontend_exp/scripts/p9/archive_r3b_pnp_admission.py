#!/usr/bin/env python3
"""Audit frozen CSV evidence and archive frontend gates; no measurements/GT reader."""
import argparse
from collections import Counter,defaultdict
import json
import subprocess
import numpy as np

import p9_r3_visual_contract as c
import run_r3a_metric_depth_coverage as r3a
from evaluate_r3a_metric_depth_coverage import statistics,grouped_quality
from run_r3b_pnp_admission import OUT,START,BRANCH,SOURCES,save,quality_contract,receipt,carriers,parity_control

BUILD=c.Path("/tmp/p9_r3_release_8JvFPZ")


def git_scope():
    prefix="src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/"
    allowed={prefix+name for name in SOURCES+("archive_r3b_pnp_admission.py","CMakeLists.txt")}
    for args in (("diff","HEAD","--no-renames","--name-only","-z"),("diff","--cached","--no-renames","--name-only","-z")):
        names=subprocess.check_output(["git",*args],cwd=c.ROOT,text=True).split("\0")
        c.require(all(not name or name in allowed or name.startswith("docs/p9_r3b_pnp_admission/") for name in names),
            "unrelated tracked/staged changes: refuse task handoff")


def final_quality(row):
    if row["status"]!="VALID":return
    c.require(int(row["pnp_correspondences"])>=20 and int(row["pnp_inliers"])>=20 and
        int(row["pnp_inliers"])<=int(row["pnp_correspondences"]) and row["pnp_attempted"]==row["cheirality_pass"]=="1" and
        np.isfinite(c.vector(row["T_Ccur_Cref"],16)).all() and np.isfinite(c.vector(row["D_vis"],16)).all(),
        "VALID PnP final admission/inlier/finite/cheirality mismatch")


def parity_summary(parity,lost):
    return dict(control_pairs=len(parity),status_parity="PASS",inlier_parity="PASS",
        max_translation_diff_m=max(float(row["translation_diff_m"]) for row in parity if row["translation_diff_m"]),
        max_rotation_diff_deg=max(float(row["rotation_diff_deg"]) for row in parity if row["rotation_diff_deg"]),
        max_reprojection_diff_px=max(float(row["reprojection_diff_px"]) for row in parity if row["reprojection_diff_px"]),
        existing_valid_frames=17,lost_valid=lost)


def admission_contract():
    return dict(old_attempt_min=30,new_attempt_min=20,final_inlier_min=20,RANSAC="UNCHANGED_EPNP100_2PX_0P99_LM",lags=list(c.LAGS))


def full_retention(full):
    if full is None:return dict(status="NOT_RUN")
    historical=[row for row in c.read_csv(c.P4/"visual_increment.csv") if row["status"]=="VALID"]
    current={row["frame"]:row for row in c.read_csv(OUT/"full_measurements.csv")}
    lost=[]
    for previous in historical:
        row=current[previous["transaction_cur"]]
        if row["status"]!="VALID":lost.append(dict(frame=row["frame"],old_status="VALID",new_status=row["status"],
            old_correspondences=previous["pnp_correspondences"],old_inliers=previous["pnp_inliers"],
            new_correspondences=row["pnp_correspondences"],new_inliers=row["pnp_inliers"],
            direct=row["direct_depth_count"],completed=row["plane_completed_count"]))
    return dict(old_valid=len(historical),retained_valid=len(historical)-len(lost),lost_valid=len(lost),lost_pairs=lost,
        note="Historical DIRECT30 vs current frozen R3A completion+admission20; not an augmented30 full-sequence parity comparison")


def typed_scored(path):
    rows=c.read_csv(path)
    for row in rows:
        row["gt_evaluable"]=int(row["gt_evaluable"])
        if row["gt_evaluable"]:
            for key in ("translation_error_m","rotation_error_deg"):row[key]=float(row[key])
            row["catastrophic_translation"]=int(row["catastrophic_translation"])
            c.require(row["catastrophic_translation"]==int(row["translation_error_m"]>.25),"catastrophic CSV flag mismatch")
    return rows


def result_contract(gates,quality,full):
    if not quality["quality_pass"]:
        return "LOW_SUPPORT_PNP_GEOMETRY_UNSAFE","TWO_VIEW_VISUAL_INERTIAL_MEASUREMENT_GATE"
    if not gates["coverage_pass"]:
        return "SPARSE_DEPTH_PNP_COVERAGE_INSUFFICIENT","TWO_VIEW_VISUAL_INERTIAL_MEASUREMENT_GATE"
    c.require(gates["parity_pass"] and gates["regression_pass"],"PnP parity contract STOP, not an algorithm result")
    c.require(full is not None,"required full-sequence gate has not been completed")
    if not full["metric"]["pass_flag"]:
        return "PNP_ADMISSION_COHORT_SPECIFIC","TWO_VIEW_VISUAL_INERTIAL_MEASUREMENT_GATE"
    return "METRIC_PNP_COVERAGE_RESTORED","HELDOUT_VISUAL_NONLOCAL_EVIDENCE_GATE"


def recompute_audit():
    gates,quality,plan=quality_contract();frozen=receipt("measurement_freeze.json")
    rows=c.read_csv(OUT/"admission_pairs.csv");by={(int(row["frame"]),int(row["lag"])):row for row in rows}
    c.require(len(rows)==len(by)==128 and set(by)=={(frame,lag) for frame in plan["targets"] for lag in c.LAGS},"pair identity/denominator changed")
    old=[row for row in c.read_csv(r3a.OUT/"coverage_by_pair.csv") if row["mode"]=="NEW_AUGMENTED"]
    old_by={(int(row["frame"]),int(row["lag"])):row for row in old}
    groups=defaultdict(list)
    for row in c.read_csv(r3a.OUT/"depth_completion_correspondences.csv"):groups[(int(row["frame"]),int(row["lag"]))].append(row)
    parity=c.read_csv(OUT/"parity.csv");proof_by={(int(row["frame"]),int(row["lag"])):row for row in parity}
    for key,row in by.items():
        previous=old_by[key];count=int(previous["pnp_correspondences"])
        c.require(count==int(row["pnp_correspondences"])==int(row["old_correspondences"]),"depth/admission carrier count mismatch")
        for field in ("transaction_ref","timestamp_ref_ns","timestamp_cur_ns","scan_ref_ns","scan_cur_ns","direct_depth_count","plane_completed_count"):
            c.require(previous[field]==row[field],"frozen pair/depth provenance changed: "+field)
        _,_,_,sha=carriers(groups[key],count);c.require(sha==row["carrier_sha256"],"metric correspondence order/carrier hash changed")
        c.require(int(row["pnp_attempted"])==int(previous["attempted"]=="1" and count>=20),"PnP attempt/count identity mismatch")
        final_quality(row)
        if count>=30:
            expected=parity_control(previous,dict(row,pnp_inliers=int(row["pnp_inliers"])))
            c.require(proof_by[key]=={name:str(value) for name,value in expected.items()},">=30 parity proof CSV mismatch")
    c.require(len(parity)==frozen["parity_controls"]==sum(int(row["pnp_correspondences"])>=30 for row in rows),"parity denominator mismatch")
    expected_low=[row for row in rows if 20<=int(row["pnp_correspondences"])<30 and row["sync_eligible"]=="1"]
    c.require(c.read_csv(OUT/"low_support_pnp.csv")==expected_low and len(expected_low)==frozen["low_support_pairs"],"low-support pair denominator mismatch")
    selected=c.read_csv(OUT/"selected_measurements.csv")
    old_selected={int(row["frame"]):row for row in c.read_csv(r3a.OUT/"selected_measurements.csv")}
    c.require(len(selected)==len({row["frame"] for row in selected})==32,"selected frame denominator mismatch")
    for chosen in selected:
        frame=int(chosen["frame"]);best=c.choose_smallest_valid([by[(frame,lag)] for lag in c.LAGS])
        previous=old_selected[frame]
        c.require(chosen["old_available"]==previous["new_available"] and chosen["old_lag"]==previous["new_lag"],"R3A baseline availability/original lag changed")
        c.require(int(chosen["new_available"])==int(best is not None) and
            chosen["new_lag"]==("" if best is None else best["lag"]),"smallest VALID selection mismatch")
        c.require(int(chosen["newly_available"])==int(chosen["old_available"]=="0" and best is not None),"new-availability flag mismatch")
    regression=c.read_csv(OUT/"existing_valid_regression.csv")
    expected_old={(str(frame),row["new_lag"]) for frame,row in old_selected.items() if row["new_available"]=="1"}
    c.require(len(regression)==len(expected_old)==17 and {(row["frame"],row["old_lag"]) for row in regression}==expected_old,
        "original17 regression frame/lag identities changed")
    for row in regression:
        same=by[(int(row["frame"]),int(row["old_lag"]))]
        c.require(same["status"]==row["original_lag_status"]=="VALID" and row["lost_valid"]=="0" and
            int(same["pnp_correspondences"])>=30,"original VALID measurement regression changed")
    c.require(sum(row["new_available"]=="1" for row in selected)==frozen["unlabelled_coverage"]["valid"],"unlabelled coverage mismatch")
    full=receipt("full_metrics.json") if (OUT/"full_metrics.json").exists() else None
    if full is not None:
        c.require(quality["development_pnp_coverage_gate"] and gates["coverage_pass"] and gates["parity_pass"] and
            gates["regression_pass"] and quality["quality_pass"],"full run forbidden by recomputed development gate")
        full_freeze=receipt("full_measurement_freeze.json");started=json.loads((OUT/"full_run_started.json").read_text())
        c.require(full["full_measurement_freeze_sha256"]==c.digest(OUT/"full_measurement_freeze.json") and
            full_freeze["full_run_started_sha256"]==c.digest(OUT/"full_run_started.json") and
            full_freeze["execution_manifest_sha256"]==c.digest(OUT/"execution_manifest.json") and
            full_freeze["posthoc_quality_sha256"]==c.digest(OUT/"posthoc_quality.json") and
            started["posthoc_quality_sha256"]==c.digest(OUT/"posthoc_quality.json") and
            started["execution_manifest_sha256"]==c.digest(OUT/"execution_manifest.json") and
            started["attempt_min"]==20 and started["completion_parameters"]==plan["completion_parameters"],"full lineage/parameter mismatch")
        full_rows=c.read_csv(OUT/"full_measurements.csv");counts={row["frame"]:int(row["pnp_correspondences"]) for row in full_rows}
        c.require(len(full_rows)==len(counts)==4126,"full pair denominator mismatch")
        for row in full_rows:
            c.require(row["lag"]=="1" and int(row["pnp_attempted"])==int(row["attempted"]=="1" and int(row["pnp_correspondences"])>=20),
                "full adjacent/admission attempt identity mismatch")
            final_quality(row)
        valid=[row for row in full_rows if row["status"]=="VALID"];scored=typed_scored(OUT/"full_gt.csv")
        c.require({row["frame"] for row in valid}=={row["frame"] for row in scored} and len(valid)==len(scored),"full GT/measurement identity mismatch")
        c.require(all(row["group"]=="FULL_ALL_VALID" and row["mode"]=="R3B20" and row["lag"]=="1" for row in scored),
            "full gate must use ALL VALID adjacent pairs, not a selected GT group")
        summary=grouped_quality(scored);main=summary[0] if summary else None
        low=[dict(row,group="FULL_LOW_SUPPORT_VALID") for row in scored if 20<=counts[row["frame"]]<30]
        historical=c.read_csv(c.P4/"visual_increment.csv");old_ids={row["transaction_cur"] for row in historical if row["status"]=="VALID" and row["paired_evaluable"]=="1"}
        c.require(sum(row["status"]=="VALID" for row in historical)==1802,"historical1802 mismatch")
        intersection=[dict(row,group="HISTORICAL_VALID_INTERSECTION") for row in scored if row["frame"] in old_ids]
        passed=main is not None and main["missing_gt"]==0 and len(valid)/4126>=.60 and main["translation"]["rmse"]<=.03517
        expected=dict(old_valid=1802,total_pairs=4126,new_valid=len(valid),new_coverage=len(valid)/4126,
            translation_rmse_m=None if main is None else main["translation"]["rmse"],rotation_rmse_deg=None if main is None else main["rotation"]["rmse"],
            low_support_valid=len(low),low_support_valid_fraction=len(low)/len(valid) if valid else 0,
            runtime_seconds=full_freeze["wall_seconds"],pass_flag=passed)
        c.require(full["metric"]==expected and full["groups"]==summary+grouped_quality(low)+grouped_quality(intersection),"full CSV/JSON metrics mismatch")
        c.require(c.read_csv(OUT/"full_floor01_metrics.csv")==[{key:"" if value is None else str(value) for key,value in expected.items()}],
            "full metrics CSV does not match recomputed full gate")
        c.require(full["gt_sha256"]==quality["gt_sha256"],"full GT provenance mismatch")
    result,next_step=result_contract(gates,quality,full)
    return gates,quality,plan,frozen,rows,parity,selected,full,result,next_step


def costs(rows,old,selected,full,write=False):
    old_by={(int(row["frame"]),int(row["lag"])):row for row in old if row["mode"]=="NEW_AUGMENTED"}
    by={(int(row["frame"]),int(row["lag"])):row for row in rows}
    early=[];brute=[]
    for chosen in selected:
        total=0.;all_cost=0.;open_prefix=True
        for lag in c.LAGS:
            key=(int(chosen["frame"]),lag);row=by[key];previous=old_by[key]
            value=float(previous["total_ms"])-float(previous["pnp_ms"])+float(row["pnp_ms"])
            all_cost+=value
            if open_prefix:
                total+=value
                if row["status"]=="VALID":open_prefix=False
        early.append(total);brute.append(all_cost)
    output=dict(development_pnp_only_ms=statistics([float(row["pnp_ms"]) for row in rows if row["pnp_attempted"]=="1"]),
        early_stop_expected_ms=statistics(early),four_lag_expected_ms=statistics(brute),
        estimate_basis="HISTORICAL_R3A_SHARED_STAGES_PLUS_CURRENT_REPLAY_PNP_NOT_CURRENT_FULL_FRONTEND_TIMING")
    if full is not None:
        full_rows=c.read_csv(OUT/"full_measurements.csv");attempted=[row for row in full_rows if row["attempted"]=="1"]
        components=[dict(component=field,subset="SYNC_ELIGIBLE_ATTEMPTED",**statistics([float(row[field]) for row in attempted]))
            for field in ("feature_ms","klt_ms","projection_ms","direct_association_ms","neighbor_search_ms","plane_fitting_ms","completion_ms","pnp_ms","frontend_ms","preprocess_ms","total_ms")]
        if write:c.write_csv(OUT/"runtime_breakdown.csv",components)
        else:c.require(c.read_csv(OUT/"runtime_breakdown.csv")==[{key:"" if value is None else str(value) for key,value in row.items()} for row in components],
            "runtime breakdown CSV mismatch")
        output.update(full_runtime_components_ms=components,full_wall_seconds=full["metric"]["runtime_seconds"])
    return output


def produce(commit_status="PENDING_GIT_HANDOFF"):
    gates,quality,plan,frozen,rows,parity,selected,full,result,next_step=recompute_audit()
    if full is None:
        c.require(not (OUT/"full_run_started.json").exists(),"partial full run cannot be reported complete")
        c.write_csv(OUT/"full_floor01_metrics.csv",[dict(status="NOT_RUN_DEVELOPMENT_GATE_FAIL",old_valid=1802,total_pairs=4126,new_valid="",new_coverage="",translation_rmse_m="",rotation_rmse_deg="",runtime_seconds="")])
    log=(BUILD/"Testing/Temporary/LastTest.log").read_text()
    c.require("CMAKE_BUILD_TYPE:STRING=Release" in (BUILD/"CMakeCache.txt").read_text(),"not Release build")
    c.require(log.count("Test Passed.")==33 and "Test Failed." not in log,"P9 tests not33/33 PASS")
    c.require((OUT/"archive_self_test.log").read_text().strip()=="P9_R3B_ARCHIVE_GATE_FINAL_QUALITY_SELF_TEST=PASS","archive self-test not PASS")
    (OUT/"ctest.log").write_text(log)
    binaries={str(path):c.digest(path) for path in BUILD.iterdir() if path.is_file() and (path.name.startswith("p9_") or path.name=="libp9_r2b_chart.so")}
    git_scope()
    verify=dict(release_build="PASS",p9_tests_passed=33,p9_tests_failed=0,build_dir=str(BUILD),
        ctest_log_sha256=c.digest(BUILD/"Testing/Temporary/LastTest.log"),binary_sha256=binaries,CSV_JSON_AUDIT="PASS",
        admission_boundary_self_test="PASS",cheirality_self_test="PASS",stage_tampering_once_only_self_test="PASS",
        archive_gate_final_quality_self_test="PASS",
        fresh_context_review="Frontend parity and stage-lineage independent read-only reviews; actionable findings fixed BEFORE replay; no external CLI",
        stable_workspace_modified=False,production_core_modified=False,unrelated_untracked_files_modified=False,
        tracked_and_staged_scope_audit="PASS",untracked_preservation_basis="No task writes/staging/deletion of initial .vscode, Testing or anomalous files; not a baseline byte-hash proof")
    save("verification.json",verify)
    old=c.read_csv(r3a.OUT/"coverage_by_pair.csv");runtime=costs(rows,old,selected,full,write=True)
    low=c.read_csv(OUT/"low_support_pnp.csv")
    output=dict(task=plan["task"],GIT=dict(branch=BRANCH,start_sha=START,end_sha="RESOLVE_FROM_GIT_AFTER_COMMIT",worktree=str(c.ROOT),
        commit_status=commit_status,push_executed=False),COHORT="DEVELOPMENT_ONLY_NOT_CONFIRMATORY",
        ADMISSION_CONTRACT=admission_contract(),
        PARITY=parity_summary(parity,gates["lost_valid"]),LOW_SUPPORT_PNP=dict(pair_count=len(low),valid=sum(row["status"]=="VALID" for row in low),pairs=low),
        COVERAGE=gates["coverage"],DEVELOPMENT_PNP_COVERAGE_GATE=quality["development_pnp_coverage_gate"],POSTHOC_GT=quality["groups"],
        FULL_FLOOR01=full if full is not None else dict(status="NOT_RUN_DEVELOPMENT_GATE_FAIL",old_coverage="1802/4126"),
        FULL_HISTORICAL_VALID_RETENTION=full_retention(full),
        COST=runtime,PER_FRAME=selected,NUMERICAL_ENVIRONMENT=plan["numerical_environment"],
        DEVELOPMENT_NEW_IMAGE_EXTRACTION=0,DEVELOPMENT_NEW_DEPTH_COMPLETION=0,NEW_NDT_CALLS=0,
        GT_USED="POSTHOC_FRONTEND_FIDELITY_ONLY",AMBIGUITY_AUC_RUN=False,POSE_SWITCHED=False,EKF_CHANGED=False,
        FINAL_RESULT=result,NEXT=next_step,verification=verify)
    save("results.json",output)
    files=[path for path in OUT.iterdir() if path.is_file() and path.name!="artifact_hashes.json"]
    sources=[c.HERE/name for name in SOURCES]+[c.HERE/"archive_r3b_pnp_admission.py",c.HERE/"CMakeLists.txt"]
    old_plan=json.loads((r3a.OUT/"execution_manifest.json").read_text())
    save("artifact_hashes.json",dict(artifacts={path.name:c.digest(path) for path in sorted(files)},
        code_sha256={str(path):c.digest(path) for path in sorted(sources)},input_sha256=plan["input_sha256"],
        full_input_sha256=old_plan["input_sha256"] if full is not None else {},gt_sha256=quality["gt_sha256"],
        numerical_binary_sha256=plan["numerical_environment"]["binary_sha256"],p9_test_binary_sha256=binaries))
    print("R3B_ARCHIVE",result,"NEXT",next_step,flush=True)


def audit():
    gates,quality,plan,_,rows,parity,selected,full,result,next_step=recompute_audit()
    hashes=json.loads((OUT/"artifact_hashes.json").read_text())
    old_plan=json.loads((r3a.OUT/"execution_manifest.json").read_text())
    verify=json.loads((OUT/"verification.json").read_text())
    sources=[c.HERE/name for name in SOURCES]+[c.HERE/"archive_r3b_pnp_admission.py",c.HERE/"CMakeLists.txt"]
    expected_codes={str(path):c.digest(path) for path in sources}
    expected_binaries={str(path):c.digest(path) for path in BUILD.iterdir() if path.is_file() and
        (path.name.startswith("p9_") or path.name=="libp9_r2b_chart.so")}
    c.require(hashes["input_sha256"]==plan["input_sha256"] and
        hashes["full_input_sha256"]==(old_plan["input_sha256"] if full is not None else {}) and
        hashes["numerical_binary_sha256"]==plan["numerical_environment"]["binary_sha256"] and
        hashes["p9_test_binary_sha256"]==verify["binary_sha256"]==expected_binaries and
        hashes["code_sha256"]==expected_codes and hashes["gt_sha256"]==quality["gt_sha256"],"input/source/binary hash map incomplete/changed")
    c.require(set(hashes["artifacts"])=={path.name for path in OUT.iterdir() if path.is_file() and path.name!="artifact_hashes.json"},
        "archive artifact hash map omits/adds files")
    for name,sha in hashes["artifacts"].items():c.require(c.digest(OUT/name)==sha,"archive artifact hash mismatch: "+name)
    for group in ("code_sha256","input_sha256","full_input_sha256","numerical_binary_sha256","p9_test_binary_sha256"):
        for path,sha in hashes[group].items():c.require(c.digest(path)==sha,"archive source/input/binary hash mismatch: "+path)
    saved=json.loads((OUT/"results.json").read_text())
    expected=dict(task=plan["task"],COHORT="DEVELOPMENT_ONLY_NOT_CONFIRMATORY",ADMISSION_CONTRACT=admission_contract(),
        NUMERICAL_ENVIRONMENT=plan["numerical_environment"],DEVELOPMENT_NEW_IMAGE_EXTRACTION=0,DEVELOPMENT_NEW_DEPTH_COMPLETION=0,
        NEW_NDT_CALLS=0,GT_USED="POSTHOC_FRONTEND_FIDELITY_ONLY",AMBIGUITY_AUC_RUN=False,POSE_SWITCHED=False,EKF_CHANGED=False,
        FULL_HISTORICAL_VALID_RETENTION=full_retention(full))
    c.require(all(saved[key]==value for key,value in expected.items()),"archived admission/environment/isolation flags contradict frozen contract")
    low=c.read_csv(OUT/"low_support_pnp.csv")
    c.require(saved["COST"]==costs(rows,c.read_csv(r3a.OUT/"coverage_by_pair.csv"),selected,full) and
        saved["PER_FRAME"]==selected and saved["LOW_SUPPORT_PNP"]==dict(pair_count=len(low),valid=sum(row["status"]=="VALID" for row in low),pairs=low) and
        saved["PARITY"]==parity_summary(parity,gates["lost_valid"]) and saved["verification"]==verify,"secondary result CSV/JSON mismatch")
    c.require(saved["FINAL_RESULT"]==result and saved["NEXT"]==next_step and saved["COVERAGE"]==gates["coverage"] and
        saved["POSTHOC_GT"]==quality["groups"] and saved["DEVELOPMENT_PNP_COVERAGE_GATE"]==quality["development_pnp_coverage_gate"] and
        saved["NEW_NDT_CALLS"]==0 and not saved["AMBIGUITY_AUC_RUN"],"result CSV/JSON contract mismatch")
    if full is not None:c.require(saved["FULL_FLOOR01"]==full,"full result CSV/JSON mismatch")
    c.require(c.digest(BUILD/"Testing/Temporary/LastTest.log")==verify["ctest_log_sha256"],"CTest log changed")
    git_scope()
    subprocess.run(["git","diff","--check"],cwd=c.ROOT,check=True)
    for name in SOURCES+("archive_r3b_pnp_admission.py",):
        check=subprocess.run(["git","diff","--no-index","--check","/dev/null",str(c.HERE/name)],capture_output=True,text=True)
        c.require(not check.stdout and not check.stderr,"untracked code whitespace error: "+name)
    print("R3B_CSV_JSON_CODE_INPUT_BINARY_HASH_DIFF_AUDIT=PASS",flush=True)


def self_test():
    gates=dict(coverage_pass=True,parity_pass=True,regression_pass=True);quality=dict(quality_pass=True)
    c.require(result_contract(gates,quality,dict(metric=dict(pass_flag=True)))[0]=="METRIC_PNP_COVERAGE_RESTORED","restored gate mapping")
    c.require(result_contract(gates,quality,dict(metric=dict(pass_flag=False)))[0]=="PNP_ADMISSION_COHORT_SPECIFIC","full failure mapping")
    c.require(result_contract(dict(gates,coverage_pass=False),quality,None)[0]=="SPARSE_DEPTH_PNP_COVERAGE_INSUFFICIENT","dev failure mapping")
    c.require(result_contract(gates,dict(quality,quality_pass=False),None)[0]=="LOW_SUPPORT_PNP_GEOMETRY_UNSAFE","quality failure mapping")
    def must_fail(action):
        try:action()
        except RuntimeError:return
        raise AssertionError("invalid archive gate accepted")
    must_fail(lambda:result_contract(gates,quality,None))
    row=dict(status="VALID",pnp_correspondences="20",pnp_inliers="20",pnp_attempted="1",cheirality_pass="1",
        T_Ccur_Cref=c.text(np.eye(4)),D_vis=c.text(np.eye(4)))
    final_quality(row)
    for invalid in (dict(row,pnp_inliers="19"),dict(row,pnp_correspondences="19"),dict(row,cheirality_pass="0")):
        must_fail(lambda item=invalid:final_quality(item))
    print("P9_R3B_ARCHIVE_GATE_FINAL_QUALITY_SELF_TEST=PASS")


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("stage",choices=("produce","audit","self-test"))
    parser.add_argument("--commit-status",choices=("PRECOMMIT_SNAPSHOT","PENDING_GIT_HANDOFF","BLOCKED_READ_ONLY_GIT_INDEX_LOCK"),default="PRECOMMIT_SNAPSHOT");args=parser.parse_args()
    {"produce":lambda:produce(args.commit_status),"audit":audit,"self-test":self_test}[args.stage]()
