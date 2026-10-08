#!/usr/bin/env python3
"""Archive/audit computed frontend gates; no extraction, NDT or GT pose reader."""
import argparse
from collections import Counter,defaultdict
import json
import subprocess

import numpy as np

import p9_r3_visual_contract as c
from p9_r3a_depth_completion import PARAMETERS
from p9_r3a_visual_frontend import array_hash
from evaluate_r3a_metric_depth_coverage import geometry_gate,grouped_quality
from run_r3a_metric_depth_coverage import OUT,START,BRANCH,development_contract,verify_freeze,save_json,input_guard

BUILD=c.Path("/tmp/p9_r3_release_8JvFPZ")


def result_contract(gates,quality,full):
    if not gates["geometry_safety_pass"] or not quality["quality_pass"]:
        return "DEPTH_COMPLETION_GEOMETRY_UNSAFE","REASSESS_VISUAL_MEASUREMENT_MODEL"
    if not gates["coverage_pass"]:
        return "DEPTH_COMPLETION_COVERAGE_INSUFFICIENT","REASSESS_VISUAL_MEASUREMENT_MODEL"
    c.require(full is not None,"required full-sequence generalization has not been run")
    if not full["metric"]["pass_flag"]:
        return "DEPTH_COMPLETION_COHORT_SPECIFIC","REASSESS_VISUAL_FRONTEND_GENERALIZATION"
    return "METRIC_VISUAL_COVERAGE_RESTORED","HELDOUT_VISUAL_NONLOCAL_EVIDENCE_GATE"


def recompute_audit():
    gates,plan=development_contract();quality=verify_freeze("posthoc_quality.json")
    c.require(quality["development_gate_sha256"]==c.digest(OUT/"development_gate_freeze.json"),"quality receipt lineage mismatch")
    regressions=c.read_csv(OUT/"old_valid_regression.csv")
    safety=geometry_gate(regressions)
    c.require(safety==gates["geometry"],"CSV/JSON geometry mismatch")
    selected=c.read_csv(OUT/"selected_measurements.csv")
    labels={int(row["frame"]):row["label"] for row in c.read_csv(c.OUT/"frame_statistics.csv")}
    c.require(c.digest(c.OUT/"frame_statistics.csv")==gates["label_source_sha256"],"development labels changed")
    rows=c.read_csv(OUT/"coverage_by_pair.csv")
    by={(int(row["frame"]),int(row["lag"]),row["mode"]):row for row in rows}
    c.require(len(rows)==len(by)==256,"pair rows duplicated/lost")
    for chosen in selected:
        frame=int(chosen["frame"])
        for prefix,mode in (("old","OLD_DIRECT_ONLY"),("new","NEW_AUGMENTED")):
            actual=c.choose_smallest_valid([by[(frame,lag,mode)] for lag in c.LAGS])
            c.require(int(chosen[prefix+"_available"])==int(actual is not None) and
                chosen[prefix+"_lag"]==("" if actual is None else actual["lag"]),"selected smallest VALID mismatch")
        c.require(int(chosen["newly_recovered"])==int(chosen["new_available"]=="1" and chosen["old_available"]=="0"),"new recovery flag mismatch")
    for summary in gates["coverage"]:
        field="old_available" if summary["mode"]=="OLD_DIRECT_ONLY" else "new_available"
        for label,key in (("MAJOR","major_valid"),("NO_MAJOR","no_major_valid")):
            c.require(summary[key]==sum(int(row[field]) for row in selected if labels[int(row["frame"])]==label),"CSV/JSON coverage mismatch")
    c.require(gates["coverage_pass"]==(gates["coverage"][1]["major_valid"]>=6 and gates["coverage"][1]["no_major_valid"]>=14),"coverage gate mismatch")
    records=c.read_csv(OUT/"depth_completion_correspondences.csv");groups=defaultdict(list)
    for row in records:groups[(int(row["frame"]),int(row["lag"]))].append(row)
    parity=c.read_csv(OUT/"direct_parity.csv");c.require(len(parity)==128,"DIRECT proof row count")
    for proof in parity:
        key=(int(proof["frame"]),int(proof["lag"]));items=groups[key]
        new=by[(*key,"NEW_AUGMENTED")];old=by[(*key,"OLD_DIRECT_ONLY")]
        c.require(len(items)==int(new["fb_valid"]),"FB provenance count mismatch")
        sources=Counter(row["depth_source"] for row in items)
        c.require(sources["DIRECT"]==int(old["pnp_correspondences"])==int(new["direct_depth_count"]),"DIRECT count mismatch")
        c.require(sources["PLANE_COMPLETED"]==int(new["plane_completed_count"]) and
            sources["DIRECT"]+sources["PLANE_COMPLETED"]==int(new["pnp_correspondences"]),"completed count mismatch")
        if proof["attempted"]=="1":
            points=[c.vector(row["point_xyz"],3) for row in items if row["depth_source"]=="DIRECT"]
            sha=array_hash(points)
            c.require(sha==proof["direct_3d_sha256"]==proof["historical_3d_sha256"] and proof["pass_flag"]=="1","DIRECT byte/hash audit failed")
        for row in items:
            if row["depth_source"]!="PLANE_COMPLETED":continue
            zmed=float(row["neighbor_depth_median_m"]);z=float(row["estimated_depth_m"])
            c.require(6<=int(row["neighbor_count"])<=12 and float(row["radius_px"])<=16 and
                float(row["planarity_ratio"])<=.02 and float(row["median_plane_residual_m"])<=max(.05,.01*zmed) and
                float(row["depth_spread_m"])<=max(.50,.10*zmed) and float(row["ray_dot"])>=.10 and z>0 and
                .9*float(row["neighbor_depth_min_m"])<=z<=1.1*float(row["neighbor_depth_max_m"]) and row["rejection"]=="",
                "accepted completion violated frozen geometry gate")
    gt_rows=c.read_csv(OUT/"newly_recovered_gt.csv")
    typed=[]
    for row in gt_rows:
        item=dict(row,gt_evaluable=int(row["gt_evaluable"]))
        if item["gt_evaluable"]:
            for key in ("translation_error_m","rotation_error_deg"):item[key]=float(row[key])
            item["catastrophic_translation"]=int(row["catastrophic_translation"])
            c.require(item["catastrophic_translation"]==int(item["translation_error_m"]>.25),"catastrophic flag mismatch")
        typed.append(item)
    c.require(grouped_quality(typed)==quality["groups"],"GT summary CSV/JSON mismatch")
    new_quality=[row for row in quality["groups"] if row["group"]=="NEWLY_RECOVERED"]
    c.require(quality["quality_pass"]==(quality["missing_gt"]==0 and (not new_quality or new_quality[0]["catastrophic_rate"]<=.10)),"posthoc quality gate mismatch")
    full=verify_freeze("full_metrics.json") if (OUT/"full_metrics.json").exists() else None
    result,next_step=result_contract(gates,quality,full)
    return gates,plan,quality,full,result,next_step,records,parity


def produce():
    gates,plan,quality,full,result,next_step,records,parity=recompute_audit()
    if full is None:
        c.require(not (OUT/"full_run_started.json").exists(),"partial full run must not be reported complete")
        c.write_csv(OUT/"full_floor01_metrics.csv",[dict(status="NOT_RUN_PREREQUISITE_GATE_FAIL",old_valid=1802,total_pairs=4126,
            new_valid="",new_coverage="",translation_rmse_m="",rotation_rmse_deg="",runtime_seconds="")])
    log=(BUILD/"Testing/Temporary/LastTest.log").read_text()
    c.require("CMAKE_BUILD_TYPE:STRING=Release" in (BUILD/"CMakeCache.txt").read_text(),"not Release build")
    c.require(log.count("Test Passed.")==31 and "Test Failed." not in log,"P9 actual test log not31/31 PASS")
    (OUT/"ctest.log").write_text(log)
    binaries={str(path):c.digest(path) for path in BUILD.iterdir() if path.is_file() and (path.name.startswith("p9_") or path.name=="libp9_r2b_chart.so")}
    verify=dict(release_build="PASS",p9_tests_passed=31,p9_tests_failed=0,build_dir=str(BUILD),
        ctest_log_sha256=c.digest(BUILD/"Testing/Temporary/LastTest.log"),binary_sha256=binaries,CSV_JSON_AUDIT="PASS",
        NEW_NDT_CALLS=0,GT_USED="POSTHOC_FRONTEND_FIDELITY_ONLY",stable_workspace_modified=False,production_core_modified=False,
        original_vscode_and_anomalous_files_modified=False,
        incidental_untracked_testing_logs="Initial unsupported CTest --test-dir overwrote Testing/Temporary/LastTest.log and CTestCostData.txt; not staged/deleted; subsequent tests only in /tmp",
        review="Fresh-context geometry review plus bounded stage-chain/numerical-binary review; actionable findings fixed before extraction; no external CLI")
    save_json("verification.json",verify)
    output=dict(task=plan["task"],GIT=dict(branch=BRANCH,start_sha=START,end_sha="RESOLVE_FROM_GIT_AFTER_COMMIT",worktree=str(c.ROOT),
        commit_status="BLOCKED_READ_ONLY_GIT_INDEX_LOCK",push_executed=False),
        COHORT="DEVELOPMENT_ONLY_NOT_CONFIRMATORY",DIRECT_PARITY=dict(attempted_pairs=sum(row["attempted"]=="1" for row in parity),
            all_attempted_pass=all(row["pass_flag"]=="1" for row in parity if row["attempted"]=="1"),historical_selected_valid=12,
            point_hash_parity="PASS",PnP_parity="PASS"),COVERAGE=gates["coverage"],COVERAGE_PASS=gates["coverage_pass"],
        GEOMETRY_SAFETY=gates["geometry"],NEWLY_RECOVERED_GT=quality["groups"],GT_QUALITY_PASS=quality["quality_pass"],
        FULL_FLOOR01=full if full is not None else dict(status="NOT_RUN_PREREQUISITE_GATE_FAIL",old_coverage="1802/4126"),
        COST=dict(per_component=c.read_csv(OUT/"runtime_breakdown.csv"),early_stop_expected_ms=gates["early_stop_estimate_ms"],
            four_lag_ms=gates["four_lag_cost_ms"],extraction=json.loads((OUT/"measurement_freeze.json").read_text())),
        KEY_FUNNEL=c.read_csv(OUT/"key_frame_funnel.csv"),PER_FRAME=c.read_csv(OUT/"selected_measurements.csv"),
        DEPTH_PROVENANCE=dict(total_FB_records=len(records),sources=dict(Counter(row["depth_source"] for row in records)),
            rejection_counts=dict(Counter(row["rejection"] for row in records if row["rejection"]))),
        COMPLETION_PARAMETERS=PARAMETERS,NUMERICAL_ENVIRONMENT=plan["numerical_environment"],
        NEW_NDT_CALLS=0,AMBIGUITY_AUC_RUN=False,GT_USED="POSTHOC_FRONTEND_FIDELITY_ONLY",POSE_SWITCHED=False,EKF_CHANGED=False,
        FINAL_RESULT=result,NEXT=next_step,verification=verify)
    save_json("results.json",output)
    files=[path for path in OUT.iterdir() if path.is_file() and path.name!="artifact_hashes.json"]
    source_paths=list(c.HERE.glob("*r3a*.py"))+[c.HERE/"CMakeLists.txt"]
    save_json("artifact_hashes.json",dict(artifacts={path.name:c.digest(path) for path in sorted(files)},
        code_sha256={str(path):c.digest(path) for path in sorted(source_paths)},input_sha256=plan["input_sha256"],
        numerical_binary_sha256=plan["numerical_environment"]["binary_sha256"],p9_test_binary_sha256=binaries))
    print("R3A_ARCHIVE",result,"NEXT",next_step,flush=True)


def audit():
    _,plan,quality,_,result,next_step,_,_=recompute_audit()
    hashes=json.loads((OUT/"artifact_hashes.json").read_text())
    for name,sha in hashes["artifacts"].items():c.require(c.digest(OUT/name)==sha,"archive artifact hash mismatch: "+name)
    for name,sha in hashes["code_sha256"].items():c.require(c.digest(name)==sha,"archive code hash mismatch: "+name)
    for name,sha in hashes["p9_test_binary_sha256"].items():c.require(c.digest(name)==sha,"P9 binary hash mismatch")
    c.require(c.digest(BUILD/"Testing/Temporary/LastTest.log")==json.loads((OUT/"verification.json").read_text())["ctest_log_sha256"],"CTest log changed")
    input_guard(plan)
    c.require(c.digest(quality["gt_source"])==quality["gt_sha256"],"posthoc GT source hash changed")
    saved=json.loads((OUT/"results.json").read_text())
    c.require(saved["FINAL_RESULT"]==result and saved["NEXT"]==next_step and saved["NEW_NDT_CALLS"]==0 and not saved["AMBIGUITY_AUC_RUN"],"machine result contract mismatch")
    subprocess.run(["git","diff","--check"],cwd=c.ROOT,check=True)
    print("R3A_CSV_JSON_SOURCE_INPUT_BINARY_HASH_DIFF_AUDIT=PASS",flush=True)


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("stage",choices=("produce","audit"));args=parser.parse_args()
    {"produce":produce,"audit":audit}[args.stage]()
