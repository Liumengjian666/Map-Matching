#!/usr/bin/env python3
"""Blind R4 B12/visual builder. No oracle-label/canonical/GT reader or imports."""
import argparse
from collections import defaultdict
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import p9_r4_contract as c
import p9_r2b_nonoracle_evidence as lidar
import p9_r3_visual_contract as visual
import p4_i3_visual_increment as p4
from p9_r3b_pnp_frontend import estimate_pair
from run_r2a_predictor_search import farthest
from run_r3a_metric_depth_coverage import numerical_environment

ENV=dict(os.environ,PYTHONDONTWRITEBYTECODE="1",OPENBLAS_NUM_THREADS="1",OMP_NUM_THREADS="1",
         LD_LIBRARY_PATH="/lib/x86_64-linux-gnu")
FRONTEND_NAMES=("p9_r3_visual_contract.py","p9_r3a_visual_frontend.py","p9_r3a_depth_completion.py",
               "p9_r3b_pnp_frontend.py","p9_r2b_nonoracle_evidence.py","p9_r2b_chart.cpp",
               "run_r2a_predictor_search.py","p9_r2a_predictor_search.cpp","p9_ndt_energy_contract.cpp")
READ_TRACE=set()
HISTORICAL_DOC_INPUTS={
    (c.ROOT/"docs/p9_r3b_pnp_admission/execution_manifest.json").resolve(),
    (c.ROOT/"docs/p9_r3_visual_nonlocal_evidence/extraction_manifest.json").resolve(),
    (visual.P4/"sync_stats.csv").resolve(),
}


def information_guard(event,args):
    if event!="open" or not isinstance(args[0],(str,bytes,os.PathLike)):
        return
    path=Path(os.fsdecode(args[0])).resolve()
    mode=args[1]
    if isinstance(mode,str) and "+" not in mode and any(key in mode for key in ("w","a","x")):
        return
    if (c.OUT/"oracle") in path.parents or path.name.startswith("oracle_") and path.suffix==".csv" or \
            path.name=="cohort_selection_freeze.json" or (c.DATA/"gt") in path.parents or \
            path.name in ("frame_projection_statistics.csv","oracle_recovery.csv") or \
            c.ARCHIVE in path.parents and path not in (c.REGISTRATION,c.ARCHIVE/"dual_u.csv") or \
            (c.ROOT/"docs") in path.parents and c.OUT not in path.parents and path not in HISTORICAL_DOC_INPUTS:
        raise RuntimeError("blind evidence denied label/canonical/GT read: "+str(path))
    if c.OUT in path.parents or c.DATA in path.parents:
        READ_TRACE.add(str(path))


def final_ids():
    c.require(not (c.OUT/"input_stop_freeze.json").exists(),
              "R4 input provenance is STOPPED; no candidate, visual or evidence execution")
    path=c.OUT/"heldout_final_cohort.csv"
    rows=c.read_csv(path)
    c.require(rows and set(rows[0])=={"transaction_id"},"evidence cohort must contain IDs only")
    ids=[int(row["transaction_id"]) for row in rows]
    selection=json.loads((c.OUT/"selection_freeze.json").read_text())
    c.require(c.digest(c.OUT/"heldout_ordered_pool.csv")==selection["artifacts"]["heldout_ordered_pool.csv"],
              "frozen held-out pool changed")
    pool=[int(row["transaction_id"]) for row in c.read_csv(c.OUT/"heldout_ordered_pool.csv")]
    c.require(len(ids) in c.PREFIXES and ids==pool[:len(ids)] and not set(ids)&set(c.DEVELOPMENT),
              "evidence cohort differs from frozen held-out prefix")
    return ids


def validate_stage_receipt(receipt,ids,cohort_sha):
    c.require(receipt["frames"]==len(ids) and receipt["cohort_ids"]==ids and
              receipt["artifacts"]["heldout_final_cohort.csv"]==cohort_sha,
              "frozen stage cohort hash/identity/count changed")
    c.require(not receipt["gt_loaded"] and not receipt["oracle_labels_loaded"] and
              not receipt["canonical_inputs_loaded"],"blind input receipt invalid")


def verify_frozen_inputs(receipt,digest=c.digest):
    for key in ("input_sha256","frontend_source_sha256"):
        for path,sha in receipt.get(key,{}).items():
            c.require(digest(path)==sha,"reused measurement input/source changed: "+str(path))


def validate_stage_rows(ids,runs,pairs):
    c.require(set(runs)==set(pairs)==set(ids),"frozen stage frame coverage changed")
    for tx in ids:
        ordered=sorted(runs[tx],key=lambda r:int(r["probe_rank"]))
        c.require(len(ordered)==12 and [int(r["probe_rank"]) for r in ordered]==list(range(1,13)) and
                  int(ordered[0]["seed_index"])==122,"frozen candidate prefix incomplete")
        c.require(len(pairs[tx])==4 and sorted(int(p["lag"]) for p in pairs[tx])==list(c.LAGS),
                  "frozen visual lag/frame coverage changed")


def frontend_guard():
    hashes={str(c.HERE/name):c.pinned(c.HERE/name) for name in FRONTEND_NAMES}
    for name in ("p4_i3_visual_frontend.py","p4_i3_visual_increment.py"):
        path=c.HERE.parent/name;hashes[str(path)]=c.pinned(path)
    old=c.ROOT/"docs/p9_r3b_pnp_admission/execution_manifest.json"
    c.pinned(old)
    expected=json.loads(old.read_text())["numerical_environment"]
    actual=numerical_environment()
    for key in ("versions","environment"):
        c.require(actual[key]==expected[key],"frozen visual numerical environment changed: "+key)
    for name,sha in expected["binary_sha256"].items():
        c.require(actual["binary_sha256"].get(name)==sha,"frozen visual numerical carrier changed: "+name)
    return hashes,actual


def candidate_stage():
    ids=final_ids()
    c.require(not (c.OUT/"candidate_freeze.json").exists(),"candidate results already frozen")
    manifest=json.loads((c.OUT/"execution_manifest.json").read_text())
    c.require(not manifest["gt_loaded"] and c.digest(manifest["binary"])==manifest["binary_sha256"],"prepared engine hash mismatch")
    for name in ("heldout_ordered_pool.csv","heldout_source_manifest.csv","conditioned_proposal_pool.csv"):
        c.require(c.digest(c.OUT/name)==manifest["artifacts"][name],"prepared blind input changed")
    groups=defaultdict(list)
    for row in c.read_csv(c.OUT/"conditioned_proposal_pool.csv"):
        if int(row["frame"]) in ids:groups[int(row["frame"])].append(row)
    probes=[]
    for tx in ids:
        for rank,seed,distance,row in farthest(groups[tx],12):
            probes.append(dict(row,probe_rank=rank,selection_distance=format(distance,".17g")))
    c.write_csv(c.OUT/"probe_manifest.csv",probes)
    source={int(row["transaction_id"]):row for row in c.read_csv(c.OUT/"heldout_source_manifest.csv")}
    c.write_csv(c.OUT/"candidate_source_manifest.csv",[source[tx] for tx in ids])
    directory=c.OUT/"candidate_runs"
    c.require(not directory.exists(),"candidate execution already started; no repeat")
    directory.mkdir()
    tick=time.perf_counter()
    command=[manifest["binary"],"--candidate",str(c.MAP),str(c.OUT/"candidate_source_manifest.csv"),
             str(c.OUT/"probe_manifest.csv"),str(directory)]
    with (c.OUT/"candidate_engine.log").open("w") as log:
        process=subprocess.Popen(command,cwd=c.ROOT,env=ENV,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,bufsize=1)
        for line in process.stdout:
            log.write(line);log.flush();print(line,end="",flush=True)
        c.require(process.wait()==0,"candidate NDT failure; artifacts preserved, no repeat")
    all_runs=[]
    for tx in ids:
        rows=c.read_csv(directory/f"tx_{tx}.csv")
        c.require(len(rows)==12 and [int(r["probe_rank"]) for r in rows]==list(range(1,13)) and
                  int(rows[0]["seed_index"])==122,"candidate B12 identity/order failure")
        planned=[r for r in probes if int(r["frame"])==tx]
        for row,expected in zip(rows,planned):
            c.require(row["seed_index"]==str(expected["seed_index"]) and row["start_pose_matrix16"]==expected["start_pose_matrix16"]
                and row["source_hash"]==source[tx]["prepared_source_hash"] and row["source_points"]==source[tx]["prepared_source_point_count"]
                and row["target_points"]=="549606" and row["converged"] in ("0","1") and 0<=int(row["iterations"])<=80,
                "candidate proposal/source/terminal parity fail")
        all_runs.extend(rows)
    c.write_csv(c.OUT/"conditioned_weak_runs.csv",all_runs)
    c.save_json(c.OUT/"candidate_freeze.json",dict(frames=len(ids),calls=len(all_runs),wall_seconds=time.perf_counter()-tick,
        cohort_ids=ids,
        gt_loaded=False,oracle_labels_loaded=False,canonical_inputs_loaded=False,
        artifacts={name:c.digest(c.OUT/name) for name in ("conditioned_weak_runs.csv","probe_manifest.csv","candidate_source_manifest.csv","heldout_final_cohort.csv")},
        input_read_trace=sorted(READ_TRACE),binary_sha256=manifest["binary_sha256"]))


def measurement_stage():
    ids=final_ids()
    c.require(not (c.OUT/"visual_measurement_freeze.json").exists(),"visual measurements already frozen")
    source_hashes,environment=frontend_guard()
    calibration=p4.load_calibration(c.DATA/"calibration");p4.cv2.setNumThreads(1);p4.sanity(calibration);visual.self_test()
    sync_path=visual.P4/"sync_stats.csv";c.pinned(sync_path)
    sync={int(row["transaction_id"]):row for row in c.read_csv(sync_path)}
    history=c.ROOT/"docs/p9_r3_visual_nonlocal_evidence/extraction_manifest.json"
    c.pinned(history);expected=json.loads(history.read_text())["input_sha256"]
    inputs={str(sync_path):c.digest(sync_path)}
    for path in [c.REGISTRATION,p4.RUNTIME,p4.MANIFEST]+p4.source_shards()+[
            c.DATA/"calibration/floor01_intrinsics.yaml",c.DATA/"calibration/floor01_extrinsics.yaml"]:
        actual=c.digest(path);c.require(actual==expected[str(path)],"frozen visual input changed: "+str(path));inputs[str(path)]=actual
        print("R4_VISUAL_INPUT_HASH_PASS",path.name,flush=True)
    registration={int(row["transaction_id"]):row for row in c.read_csv(c.REGISTRATION)}
    pool=defaultdict(list)
    for row in c.read_csv(c.OUT/"probe_manifest.csv"):pool[int(row["frame"])].append(row)
    inverse_il=np.linalg.inv(calibration["T_imu_lidar"])
    pairs=[]
    def nominal(row):
        c.require(row["converged"]=="1" and row["status"]=="SUCCESS","invalid frozen visual reference nominal")
        return visual.pose_xyzq([float(row["raw_"+key]) for key in ("x","y","z","qx","qy","qz","qw")])
    for tx in ids:
        t0=visual.rigid_matrix(pool[tx][0]["nominal_pose_matrix16"])
        dt,dr=visual.separation(t0,nominal(registration[tx]));c.require(dt<=1e-5 and dr<=1e-4,"nominal map-frame parity failure")
        for lag in c.LAGS:
            ref=tx-lag;available=ref in sync and ref in registration
            for item in ([tx,ref] if available else [tx]):
                c.require(registration[item]["stamp_ns"]==sync[item]["scan_end_ns"],"visual baseline/sync timestamp mismatch")
            pairs.append(dict(frame=tx,lag=lag,transaction_ref=ref,reference_available=int(available),
                image_index_ref=sync[ref]["image_index"] if available else "",image_index_cur=sync[tx]["image_index"],
                timestamp_ref_ns=sync[ref]["image_ns"] if available else "",timestamp_cur_ns=sync[tx]["image_ns"],
                scan_ref_ns=sync[ref]["scan_end_ns"] if available else "",scan_cur_ns=sync[tx]["scan_end_ns"],
                sync_ref_eligible=sync[ref]["eligible"] if available else "0",sync_cur_eligible=sync[tx]["eligible"],
                reference_map_T_imu=visual.text(nominal(registration[ref])@inverse_il) if available else "",
                nominal_map_T_imu=visual.text(t0@inverse_il)))
    c.write_csv(c.OUT/"visual_pair_manifest.csv",pairs)
    needed=set(ids)|{int(p["transaction_ref"]) for p in pairs if p["reference_available"]}
    clouds={};cloud_hashes=[]
    for request in p4.requests():
        tx=int(request.transaction_id)
        if tx not in needed:continue
        c.require(tx not in clouds and request.map_frame=="floor01_map_h1" and request.lidar_frame=="cmu_sp1_velodyne" and
                  str(request.scan_end_ns)==sync[tx]["scan_end_ns"],"reference cloud frame/stamp invalid")
        clouds[tx]=request.cloud_end_frame
        cloud_hashes.append(dict(transaction_id=tx,cloud_data_sha256=hashlib.sha256(bytes(request.cloud_end_frame.data)).hexdigest()))
    c.require(set(clouds)==needed,"reference clouds missing")
    wanted={int(sync[tx]["image_index"]):int(sync[tx]["image_ns"]) for tx in needed if sync[tx]["eligible"]=="1"}
    images={};image_hashes=[]
    for index,message in enumerate(p4.camera_messages(p4.source_shards())):
        if index not in wanted:continue
        c.require(message.header.stamp.to_nsec()==wanted[index] and message.header.frame_id=="d" and
            message._type=="sensor_msgs/Image" and message.encoding=="bgr8" and (message.width,message.height)==calibration["size"],
            "camera metadata/header sync changed")
        tick=time.perf_counter();gray=p4.rectify(message,calibration);ms=(time.perf_counter()-tick)*1000
        images[index]=(gray,ms)
        image_hashes.append(dict(image_index=index,image_ns=wanted[index],raw_sha256=hashlib.sha256(bytes(message.data)).hexdigest(),
                                rectified_sha256=hashlib.sha256(gray.tobytes()).hexdigest(),rectification_ms=ms))
        if len(images)==len(wanted):break
    c.require(set(images)==set(wanted),"selected images missing")
    measurements=[];tick_all=time.perf_counter()
    for pair in pairs:
        row=dict(pair,status="REFERENCE_UNAVAILABLE",attempted=0,detected=0,klt_forward_valid=0,fb_valid=0,
            direct_depth_count=0,plane_completed_count=0,total_depth_count=0,depth_associated=0,pnp_correspondences=0,
            pnp_inliers=0,inlier_ratio="",reprojection_rmse_px="",pnp_attempted=0,inlier_indices="",cheirality_pass="",
            grid_occupancy="",hull_fraction="",median_parallax_px="",feature_ms=0.,klt_ms=0.,projection_ms=0.,
            direct_association_ms=0.,neighbor_search_ms=0.,plane_fitting_ms=0.,completion_ms=0.,depth_ms=0.,
            pnp_ms=0.,frontend_ms=0.,preprocess_ms=0.,total_ms=0.,T_Ccur_Cref="",D_vis="")
        if pair["reference_available"]:
            row["status"]="SYNC_INVALID"
            if pair["sync_ref_eligible"]==pair["sync_cur_eligible"]=="1":
                ref,ref_cost=images[int(pair["image_index_ref"])];cur,cur_cost=images[int(pair["image_index_cur"])]
                c.require(int(pair["image_index_cur"])>int(pair["image_index_ref"]),"non-forward visual pair")
                metrics=estimate_pair(ref,cur,clouds[int(pair["transaction_ref"])],calibration,int(pair["frame"]))
                row.update(metrics,attempted=1,preprocess_ms=ref_cost+cur_cost,total_ms=metrics["frontend_ms"]+ref_cost+cur_cost)
                row["depth_ms"]=sum(row[key] for key in ("projection_ms","direct_association_ms","completion_ms"))
        measurements.append(row)
        if int(pair["lag"])==8:print("R4_VISUAL_TARGET_COMPLETE",pair["frame"],flush=True)
    # Uniform CSV fields include all frozen frontend timing fields, even if a pair is missing.
    fields=list(dict.fromkeys(key for row in measurements for key in row))
    measurements=[{key:row.get(key,"") for key in fields} for row in measurements]
    c.write_csv(c.OUT/"visual_measurements.csv",measurements,fields)
    c.write_csv(c.OUT/"selected_cloud_hashes.csv",cloud_hashes);c.write_csv(c.OUT/"selected_image_hashes.csv",image_hashes)
    c.save_json(c.OUT/"visual_measurement_freeze.json",dict(frames=len(ids),pairs=len(measurements),lags=list(c.LAGS),
        cohort_ids=ids,
        selection="SMALLEST_VALID_LAG",frontend_frozen_sha=c.START_SHA,frontend_source_sha256=source_hashes,
        numerical_environment=environment,input_sha256=inputs,gt_loaded=False,oracle_labels_loaded=False,
        canonical_inputs_loaded=False,input_read_trace=sorted(READ_TRACE),extraction_seconds=time.perf_counter()-tick_all,
        artifacts={name:c.digest(c.OUT/name) for name in ("visual_measurements.csv","visual_pair_manifest.csv",
                   "selected_cloud_hashes.csv","selected_image_hashes.csv","heldout_final_cohort.csv")}))


def freeze_evidence(library):
    ids=final_ids();frontend_guard()
    c.require(not (c.OUT/"evidence_freeze.json").exists(),"nonoracle evidence already frozen")
    for name in ("candidate_freeze.json","visual_measurement_freeze.json"):
        receipt=json.loads((c.OUT/name).read_text())
        validate_stage_receipt(receipt,ids,c.digest(c.OUT/"heldout_final_cohort.csv"))
        verify_frozen_inputs(receipt)
        for artifact,sha in receipt["artifacts"].items():c.require(c.digest(c.OUT/artifact)==sha,"frozen blind input changed")
    chart=lidar.chart_function(library);lidar.self_test(chart);visual.self_test()
    runs=defaultdict(list);pairs=defaultdict(list)
    for row in c.read_csv(c.OUT/"conditioned_weak_runs.csv"):runs[int(row["frame"])].append(row)
    for row in c.read_csv(c.OUT/"visual_measurements.csv"):pairs[int(row["frame"])].append(row)
    validate_stage_rows(ids,runs,pairs)
    calibration=p4.load_calibration(c.DATA/"calibration");inverse_il=np.linalg.inv(calibration["T_imu_lidar"])
    clusters=[];competitors=[];lidar_evidence=[];residuals=[];evidence=[]
    for tx in ids:
        rows=sorted(runs[tx],key=lambda r:int(r["probe_rank"]))
        nominal=rows[0]["nominal_pose_matrix16"]
        cs,ks,ls=lidar.evidence_for_frame(tx,12,nominal,rows,chart)
        clusters.extend(cs);competitors.extend(ks);lidar_evidence.append(ls)
        nominal_imu=visual.rigid_matrix(nominal)@inverse_il
        strict=[k for k in ks if visual.strict_separation(visual.rigid_matrix(nominal),visual.rigid_matrix(k["representative_pose_matrix16"]))[2]]
        chosen=visual.choose_smallest_valid(pairs[tx]);available=chosen is not None
        scores=[];rnom=None;nominal_rotation=""
        if available:
            reference=visual.rigid_matrix(chosen["reference_map_T_imu"]);measurement=visual.rigid_matrix(chosen["D_vis"])
            candidates=[dict(cluster_id="NOMINAL",pose=nominal_imu,nominal=1,translation=0.,rotation=0.)]
            candidates.extend(dict(cluster_id=k["cluster_id"],pose=visual.rigid_matrix(k["representative_pose_matrix16"])@inverse_il,
                nominal=0,translation=k["translation_from_nominal_m"],rotation=k["rotation_from_nominal_deg"]) for k in strict)
            for candidate in candidates:
                rt,rr=visual.residual(reference,candidate["pose"],measurement)
                residuals.append(dict(frame=tx,selected_lag=int(chosen["lag"]),cluster_id=candidate["cluster_id"],
                    is_nominal=candidate["nominal"],map_T_imu=visual.text(candidate["pose"]),r_t_m=rt,r_r_deg=rr,
                    separation_translation_m=candidate["translation"],separation_rotation_deg=candidate["rotation"]))
                if candidate["nominal"]:rnom=rt;nominal_rotation=rr
                else:scores.append(rt)
        scalars=visual.evidence_scalars(available,rnom,scores)
        evidence.append(dict(frame=tx,visual_available=int(available),selected_lag=int(chosen["lag"]) if available else "",
            transaction_ref=int(chosen["transaction_ref"]) if available else "",image_cur_ns=chosen["timestamp_cur_ns"] if available else "",
            inliers=int(chosen["pnp_inliers"]) if available else "",reprojection=chosen["reprojection_rmse_px"] if available else "",
            cluster_count=len(cs),competitive_cluster_count=len(ks),strict_candidate_count=len(strict),
            nominal_rotation_residual_deg=nominal_rotation,U_comp=ls["U_comp"],**scalars))
    c.write_csv(c.OUT/"terminal_clusters.csv",clusters)
    c.write_csv(c.OUT/"competitive_terminals.csv",competitors,list(clusters[0])+["xi","xi_norm","score_advantage_per_source"])
    c.write_csv(c.OUT/"lidar_nonoracle_evidence.csv",lidar_evidence)
    c.write_csv(c.OUT/"candidate_residuals.csv",residuals,["frame","selected_lag","cluster_id","is_nominal","map_T_imu","r_t_m",
              "r_r_deg","separation_translation_m","separation_rotation_deg"])
    c.write_csv(c.OUT/"nonoracle_evidence.csv",evidence)
    files=("conditioned_weak_runs.csv","terminal_clusters.csv","competitive_terminals.csv","lidar_nonoracle_evidence.csv",
           "visual_measurements.csv","candidate_residuals.csv","nonoracle_evidence.csv","heldout_final_cohort.csv")
    c.save_json(c.OUT/"evidence_freeze.json",dict(state="BLIND_EVIDENCE_FROZEN_BEFORE_LABEL_EVALUATION",frames=len(ids),budget=12,
        gt_loaded=False,oracle_labels_loaded_by_evidence=False,canonical_inputs_loaded=False,
        frontend_frozen_sha=c.START_SHA,EPS_SCORE=c.EPS_SCORE,U_visual="max(0,r_nom-min_alt r_alt)",
        input_read_trace=sorted(READ_TRACE),chart_library=str(library),chart_library_sha256=c.digest(library),
        builder_sha256=c.digest(__file__),artifacts={name:c.digest(c.OUT/name) for name in files}))
    print("R4_BLIND_EVIDENCE_FROZEN",c.digest(c.OUT/"nonoracle_evidence.csv"),"GT_LOADED=NO ORACLE_LABELS_LOADED_BY_EVIDENCE=NO",flush=True)


def self_test():
    c.selection_self_test();visual.self_test()
    for path in (c.OUT/"oracle/oracle_labels.csv",c.OUT/"oracle_clusters.csv",c.DATA/"gt/floor01_gt.txt",
                 c.ARCHIVE/"candidates.csv",c.ARCHIVE/"clusters.csv",
                 c.ROOT/"docs/p9_foundation_weak_discovery/h1/major_projection_statistics.csv"):
        for mode in ("r","a+","w+",None):
            try:information_guard("open",(str(path),mode,0))
            except RuntimeError:pass
            else:raise RuntimeError("blind guard admitted forbidden readable input")
    information_guard("open",(str(c.OUT/"heldout_final_cohort.csv"),"r",0))
    ids=[10000,20000]
    receipt=dict(frames=2,cohort_ids=ids,artifacts={"heldout_final_cohort.csv":"frozen"},
                 gt_loaded=False,oracle_labels_loaded=False,canonical_inputs_loaded=False)
    validate_stage_receipt(receipt,ids,"frozen")
    def denied(callback):
        try:callback()
        except RuntimeError:return
        raise RuntimeError("receipt/coverage regression accepted changed input")
    denied(lambda:validate_stage_receipt(receipt,ids[:1],"shrunk"))
    denied(lambda:verify_frozen_inputs({"input_sha256":{"calibration":"frozen"}},lambda _:"changed"))
    runs={tx:[dict(probe_rank=rank,seed_index=122 if rank==1 else rank) for rank in range(1,13)] for tx in ids}
    pairs={tx:[dict(lag=lag) for lag in c.LAGS] for tx in ids}
    validate_stage_rows(ids,runs,pairs)
    denied(lambda:validate_stage_rows(ids[:1],runs,pairs))
    pairs[ids[1]]=pairs[ids[1]][:3]
    denied(lambda:validate_stage_rows(ids,runs,pairs))
    if (c.OUT/"input_stop_freeze.json").exists():denied(final_ids)
    print("P9_R4_LABEL_GT_INFORMATION_ISOLATION_SELF_TEST=PASS")
    print("P9_R4_COHORT_CALIBRATION_FREEZE_CHAIN_SELF_TEST=PASS")


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage",choices=("self-test","candidate","visual","freeze"))
    parser.add_argument("--library",type=Path)
    args=parser.parse_args();sys.addaudithook(information_guard)
    if args.stage=="self-test":self_test()
    elif args.stage=="candidate":candidate_stage()
    elif args.stage=="visual":measurement_stage()
    else:
        c.require(args.library is not None,"chart library required")
        freeze_evidence(args.library)
