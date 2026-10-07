#!/usr/bin/env python3
"""Fixed four-lag P4 extraction and blind visual candidate evidence, no new NDT."""
import argparse
from collections import Counter
import hashlib
import json
import subprocess
import time

import numpy as np

import p9_r3_visual_contract as c
import p4_i3_visual_increment as p4

ADJ_FIELDS=("transaction_ref","transaction_cur","timestamp_ref_ns","timestamp_cur_ns","scan_ref_ns","scan_cur_ns",
    "attempted","status","detected","klt_forward_valid","fb_valid","depth_associated","pnp_correspondences",
    "pnp_inliers","inlier_ratio","reprojection_rmse_px")
METRIC_FIELDS=("status","detected","klt_forward_valid","fb_valid","depth_associated","pnp_correspondences",
    "pnp_inliers","inlier_ratio","reprojection_rmse_px","feature_ms","klt_ms","depth_ms","projection_ms",
    "association_ms","pnp_ms")


def inputs():
    paths=[c.R2A/"predictor_parity.csv",c.R2B/"competitive_terminals.csv",c.R2B/"evidence_freeze.json",
           c.P4/"visual_increment.csv",c.P4/"sync_stats.csv",c.R2B/"terminal_clusters.csv",c.R2B/"nonoracle_evidence.csv",
           c.R2A/"execution_manifest.json"]
    hashes={str(path):c.committed_hash(path) for path in paths}
    nominals={int(row["frame"]):c.rigid_matrix(row["nominal_pose_matrix16"]) for row in c.read_csv(paths[0])}
    c.require(len(nominals)==32,"frozen 32-target cohort invalid")
    frozen=json.loads(paths[2].read_text())
    c.require(c.digest(paths[1])==frozen["frozen_artifact_sha256"]["competitive_terminals.csv"],"R2B competitors changed")
    competitors=[row for row in c.read_csv(paths[1]) if row["budget"]=="12"]
    c.require(all(int(row["frame"]) in nominals and row["is_nominal"]=="0" and row["competitive"]=="1" and
        float(row["raw_score"])>=float(row["nominal_raw_score"])+2.747604276e-4 for row in competitors),"invalid non-oracle competitor input")
    # Project legacy CSV fields immediately; legacy GT/error columns are never used or retained.
    adjacent=[{key:row[key] for key in ADJ_FIELDS} for row in c.read_csv(paths[3]) if int(row["transaction_cur"]) in nominals]
    c.require(len(adjacent)==32 and len({row["transaction_cur"] for row in adjacent})==32 and
              sum(row["status"]=="VALID" for row in adjacent)==12,"VISUAL_ARCHIVE_ALIGNMENT_FAIL")
    sync={int(row["transaction_id"]):row for row in c.read_csv(paths[4])}
    c.require(len(sync)==4127,"P4 sync archive incomplete")
    registration={int(row["transaction_id"]):row for row in c.read_csv(c.REGISTRATION)}
    c.require(len(registration)==4127,"frozen nominal registration archive incomplete")
    # dual_u.csv already has a prior R2A freeze, including reference-only raw nominal poses.
    # Registration itself was not previously whole-file hashed; do not invent that lineage.
    dual_path=c.REGISTRATION.parent/"dual_u.csv"
    expected=json.loads((c.R2A/"execution_manifest.json").read_text())["input_sha256"][str(dual_path)]
    c.require(c.digest(dual_path)==expected,"prior frozen R1 nominal archive changed")
    hashes[str(dual_path)]=expected
    dual={int(row["transaction_id"]):row for row in c.read_csv(dual_path)}
    needed=set(nominals)|{frame-lag for frame in nominals for lag in c.LAGS}
    for tx in sorted(needed):
        if tx not in registration:continue
        c.require(tx in dual and registration[tx]["stamp_ns"]==dual[tx]["stamp_ns"],"R1 nominal reference lineage invalid")
        first=[float(registration[tx]["raw_"+key]) for key in ("x","y","z","qx","qy","qz","qw")]
        second=[float(dual[tx]["raw_"+key]) for key in ("x","y","z","qx","qy","qz","qw")]
        c.require(np.array_equal(first,second),"R1 registration/raw nominal pose differs from prior freeze")
    hashes[str(c.REGISTRATION)]=c.digest(c.REGISTRATION)
    print("ADJACENT_LABEL_FREE_PARITY=PASS VALID=12/32",dict(Counter(row["status"] for row in adjacent)),flush=True)
    return nominals,competitors,adjacent,sync,registration,hashes


def raw_nominal(row):
    c.require(row["converged"]=="1" and row["status"]=="SUCCESS","reference nominal registration invalid")
    return c.pose_xyzq([float(row["raw_"+key]) for key in ("x","y","z","qx","qy","qz","qw")])


def freeze_plan(calibration):
    branch=subprocess.check_output(["git","branch","--show-current"],cwd=c.ROOT,text=True).strip()
    c.require(branch==c.BRANCH,"incorrect R3 branch")
    subprocess.run(["git","merge-base","--is-ancestor",c.START_SHA,"HEAD"],cwd=c.ROOT,check=True)
    c.OUT.mkdir(parents=True,exist_ok=True)
    c.require(not (c.OUT/"extraction_manifest.json").exists(),"R3 extraction plan already exists")
    nominals,competitors,adjacent,sync,registration,hashes=inputs()
    pairs=[];parity=[]
    inverse_il=np.linalg.inv(calibration["T_imu_lidar"])
    for frame in sorted(nominals):
        c.require(registration[frame]["stamp_ns"]==sync[frame]["scan_end_ns"],"target nominal/sync stamp mismatch")
        dt,dr=c.separation(nominals[frame],raw_nominal(registration[frame]))
        c.require(dt<=1e-5 and dr<=1e-4,"nominal reference-frame parity fail")
        parity.append(dict(frame=frame,translation_error_m=dt,rotation_error_deg=dr,pass_flag=1))
        for lag in c.LAGS:
            ref=frame-lag;available=ref in sync and ref in registration
            if available:
                c.require(registration[ref]["stamp_ns"]==sync[ref]["scan_end_ns"],"reference nominal/sync stamp mismatch")
            pairs.append(dict(frame=frame,lag=lag,transaction_ref=ref,reference_available=int(available),
                scan_ref_ns=sync[ref]["scan_end_ns"] if available else "",
                scan_cur_ns=sync[frame]["scan_end_ns"],timestamp_ref_ns=sync[ref]["image_ns"] if available else "",
                timestamp_cur_ns=sync[frame]["image_ns"],image_index_ref=sync[ref]["image_index"] if available else "",
                image_index_cur=sync[frame]["image_index"],sync_ref_eligible=sync[ref]["eligible"] if available else "0",
                sync_cur_eligible=sync[frame]["eligible"],mismatch_ref_ms=sync[ref]["signed_mismatch_ms"] if available else "",
                mismatch_cur_ms=sync[frame]["signed_mismatch_ms"],
                reference_map_T_lidar=c.text(raw_nominal(registration[ref])) if available else "",
                reference_map_T_imu=c.text(raw_nominal(registration[ref])@inverse_il) if available else "",
                nominal_map_T_lidar=c.text(nominals[frame]),nominal_map_T_imu=c.text(nominals[frame]@inverse_il)))
    c.write_csv(c.OUT/"adjacent_visual_audit.csv",adjacent)
    c.write_csv(c.OUT/"nominal_frame_parity.csv",parity)
    c.write_csv(c.OUT/"pair_manifest.csv",pairs)
    paths=p4.source_shards();hash_paths=[p4.RUNTIME,p4.MANIFEST]+paths+[
        c.DATA/"calibration/floor01_intrinsics.yaml",c.DATA/"calibration/floor01_extrinsics.yaml"]
    for path in hash_paths:
        hashes[str(path)]=c.digest(path);print("HASHED",path.name,hashes[str(path)],flush=True)
    c.require(hashes[str(p4.RUNTIME)]=="860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db","P4 runtime bag hash mismatch")
    source_paths=[c.HERE/"p9_r3_visual_contract.py",c.HERE/"run_r3_visual_evidence.py",c.HERE/"build_r3_visual_evidence.py",
                  c.PACKAGE/"scripts/p4_i3_visual_frontend.py",c.PACKAGE/"scripts/p4_i3_visual_increment.py",c.OUT/"THEORY.md"]
    manifest=dict(task="PAPER-P9-R3-VISUAL-INDEPENDENT-NONLOCAL-EVIDENCE-GATE",branch=c.BRANCH,start_sha=c.START_SHA,
        preparation_head=subprocess.check_output(["git","rev-parse","HEAD"],cwd=c.ROOT,text=True).strip(),
        lags=list(c.LAGS),targets=sorted(nominals),pairs=len(pairs),input_sha256=hashes,
        construction_source_sha256={str(path):c.digest(path) for path in source_paths},
        selection="SMALLEST_VALID_LAG_ONLY",opencv_threads=1,pnp_seed="transaction_cur",
        frontend="unchanged P4-I3",name="MAP-INDEPENDENT VISUAL-MOTION EVIDENCE",
        camera_lidar_used=True,labels_loaded=False,canonical_inputs_loaded=False,gt_used_for_evidence=False,
        new_ndt_calls=0,T_imu_camera=c.text(calibration["T_imu_camera"]),T_imu_lidar=c.text(calibration["T_imu_lidar"]),
        pose_frame="map_T_lidar converted to map_T_imu using inverse(T_IL)",
        reference="frozen R1 same-objective registration raw nominal",bag_paths=[str(path) for path in paths],
        plan_artifact_sha256={name:c.digest(c.OUT/name) for name in ("adjacent_visual_audit.csv","nominal_frame_parity.csv","pair_manifest.csv")})
    (c.OUT/"extraction_manifest.json").write_text(json.dumps(manifest,indent=2,sort_keys=True,allow_nan=False)+"\n")
    return pairs


def selected_data(pairs,calibration):
    started=time.perf_counter();ids={int(row["frame"]) for row in pairs}
    ids.update(int(row["transaction_ref"]) for row in pairs if row["reference_available"])
    sync={int(row["transaction_id"]):row for row in c.read_csv(c.P4/"sync_stats.csv")}
    clouds={};cloud_records=[]
    for request in p4.requests():
        tx=int(request.transaction_id)
        if tx not in ids:continue
        c.require(request.map_frame=="floor01_map_h1" and request.lidar_frame=="cmu_sp1_velodyne" and
                  str(request.scan_end_ns)==sync[tx]["scan_end_ns"],"request frame/stamp mismatch")
        c.require(tx not in clouds,"duplicate captured request")
        clouds[tx]=request.cloud_end_frame
        cloud_records.append(dict(transaction_id=tx,scan_end_ns=int(request.scan_end_ns),
            cloud_data_sha256=hashlib.sha256(bytes(request.cloud_end_frame.data)).hexdigest(),
            point_count=int(request.cloud_end_frame.width*request.cloud_end_frame.height),
            lidar_frame=request.lidar_frame,map_frame=request.map_frame))
    c.require(set(clouds)==ids,"selected reference clouds missing")
    wanted={}
    for tx in ids:
        if sync[tx]["eligible"]!="1":continue
        index,stamp=int(sync[tx]["image_index"]),int(sync[tx]["image_ns"])
        c.require(index not in wanted or wanted[index]==stamp,"shared image index has inconsistent sync stamps")
        wanted[index]=stamp
    images={};image_records=[]
    for index,message in enumerate(p4.camera_messages(p4.source_shards())):
        if index not in wanted:continue
        c.require(message.header.stamp.to_nsec()==wanted[index],"archived image synchronization differs from actual header")
        c.require(message.header.frame_id=="d" and message._type=="sensor_msgs/Image" and
                  message.encoding=="bgr8" and (message.width,message.height)==calibration["size"],
                  "P4 camera identity/metadata changed")
        tick=time.perf_counter();gray=p4.rectify(message,calibration);cost=(time.perf_counter()-tick)*1000
        images[index]=(gray,cost)
        image_records.append(dict(image_index=index,image_ns=wanted[index],camera_frame=message.header.frame_id,
            camera_topic=p4.CAM_TOPIC,width=message.width,height=message.height,
            encoding=message.encoding,raw_image_data_sha256=hashlib.sha256(bytes(message.data)).hexdigest(),
            rectified_gray_sha256=hashlib.sha256(gray.tobytes()).hexdigest(),rectification_ms=cost))
        if len(images)==len(wanted):break
    c.require(set(images)==set(wanted),"selected raw images missing")
    c.write_csv(c.OUT/"selected_cloud_hashes.csv",sorted(cloud_records,key=lambda row:row["transaction_id"]))
    c.write_csv(c.OUT/"selected_image_hashes.csv",image_records)
    return clouds,images,time.perf_counter()-started


def extract():
    calibration=c.frontend.load_calibration(c.DATA/"calibration")
    c.frontend.cv2.setNumThreads(1);c.self_test()
    started=time.perf_counter();pairs=freeze_plan(calibration)
    clouds,images,io_seconds=selected_data(pairs,calibration)
    rows=[]
    for pair in pairs:
        row=dict(pair,attempted=0,status="REFERENCE_UNAVAILABLE",detected=0,klt_forward_valid=0,fb_valid=0,
            depth_associated=0,pnp_correspondences=0,pnp_inliers=0,inlier_ratio="",reprojection_rmse_px="",
            feature_ms=0.,klt_ms=0.,depth_ms=0.,projection_ms=0.,association_ms=0.,pnp_ms=0.,
            preprocess_ms=0.,total_ms=0.,T_Ccur_Cref="",D_vis="")
        if pair["reference_available"]:
            row["status"]="SYNC_INVALID"
            if pair["sync_ref_eligible"]==pair["sync_cur_eligible"]=="1":
                ref,ref_cost=images[int(pair["image_index_ref"])];cur,cur_cost=images[int(pair["image_index_cur"])]
                c.require(int(pair["image_index_cur"])>int(pair["image_index_ref"]),"non-forward image pair")
                tick=time.perf_counter();metrics,transform=c.frontend.estimate(ref,cur,clouds[int(pair["transaction_ref"])],calibration,int(pair["frame"]))
                row.update({key:metrics[key] for key in METRIC_FIELDS})
                row.update(attempted=1,preprocess_ms=ref_cost+cur_cost,total_ms=(time.perf_counter()-tick)*1000+ref_cost+cur_cost)
                if transform is not None:
                    c.require(row["status"]=="VALID" and row["pnp_correspondences"]>=30 and row["pnp_inliers"]>=20,"P4 VALID invariant failed")
                    row.update(T_Ccur_Cref=c.text(transform),D_vis=c.text(c.imu_increment(transform,calibration["T_imu_camera"])))
        rows.append(row)
        print("PAIR",pair["frame"],pair["lag"],row["status"],row["pnp_correspondences"],row["pnp_inliers"],flush=True)
    c.write_csv(c.OUT/"multilag_visual_measurements.csv",rows)
    receipt=dict(extraction_manifest_sha256=c.digest(c.OUT/"extraction_manifest.json"),rows=len(rows),
        selected_data_io_seconds=io_seconds,wall_seconds=time.perf_counter()-started,NEW_NDT_CALLS=0,
        measurement_sha256=c.digest(c.OUT/"multilag_visual_measurements.csv"),
        selected_input_hashes={name:c.digest(c.OUT/name) for name in ("selected_image_hashes.csv","selected_cloud_hashes.csv")},
        label_or_gt_evaluation_loaded=False)
    (c.OUT/"extraction_receipt.json").write_text(json.dumps(receipt,indent=2,sort_keys=True,allow_nan=False)+"\n")
    print("P9_R3_FIXED_FOUR_LAG_EXTRACTION=COMPLETE",dict(Counter(row["status"] for row in rows)),flush=True)


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("stage",choices=("self-test","extract"));args=parser.parse_args()
    {"self-test":c.self_test,"extract":extract}[args.stage]()
