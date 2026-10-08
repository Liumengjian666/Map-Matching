#!/usr/bin/env python3
"""Hash-pinned P4/R3 image/depth replay; no label, GT or NDT reader."""
import argparse
import csv
import gzip
import hashlib
import json
import os
import subprocess
import sys
import time

import numpy as np

import p9_r3_visual_contract as c
import p4_i3_visual_increment as p4
from p9_r3a_depth_completion import PARAMETERS
from p9_r3a_visual_frontend import estimate_pair

OUT=c.ROOT/"docs/p9_r3a_metric_depth_coverage"
START="f0e8be52a553fe04b2a3e116f8a54b01089dcd3c"
BRANCH="research/p9-r3a-metric-depth-coverage"
SOURCES=("p9_r3a_depth_completion.py","p9_r3a_visual_frontend.py","run_r3a_metric_depth_coverage.py",
    "evaluate_r3a_metric_depth_coverage.py")


def save_json(name,value):
    (OUT/name).write_text(json.dumps(value,indent=2,sort_keys=True,allow_nan=False)+"\n")


def pinned(path):
    expected=subprocess.check_output(["git","show",START+":"+path.relative_to(c.ROOT).as_posix()],cwd=c.ROOT)
    value=hashlib.sha256(expected).hexdigest()
    c.require(c.digest(path)==value,"pinned historical artifact changed: "+str(path))
    return value


def source_guard(plan):
    c.require(PARAMETERS==plan["completion_parameters"],"frozen depth parameters changed")
    for name,expected in plan["construction_source_sha256"].items():
        c.require(c.digest(name)==expected,"frozen implementation changed: "+name)
    c.require(c.digest(OUT/"pair_manifest.csv")==plan["pair_manifest_sha256"],"copied pair manifest changed")
    expected=plan["numerical_environment"]
    actual=numerical_environment()
    for key in ("versions","environment"):
        c.require(actual[key]==expected[key],"frozen numerical environment changed")
    for name,sha in expected["binary_sha256"].items():
        c.require(actual["binary_sha256"].get(name)==sha,"frozen numerical binary not currently loaded/matching: "+name)


def numerical_environment():
    import scipy
    # Load all math paths used in construction, including their BLAS/LAPACK.
    np.linalg.eigh(np.eye(3));np.linalg.inv(np.eye(3))
    paths={sys.executable}
    for line in c.Path("/proc/self/maps").read_text().splitlines():
        parts=line.split()
        if len(parts)<6 or not parts[-1].startswith("/"):continue
        path=parts[-1];name=c.Path(path).name
        if ".so" in name and (any(key in path for key in ("/numpy/","/scipy/","/cv2","libopencv","libopenblas","libblas","liblapack")) or
                name.startswith(("libm-","libm.so","libgfortran","libstdc++","libgcc_s"))):paths.add(path)
    return dict(versions=dict(python=sys.version,numpy=np.__version__,scipy=scipy.__version__,opencv=p4.cv2.__version__),
        environment={name:os.environ.get(name,"") for name in ("OPENBLAS_NUM_THREADS","OMP_NUM_THREADS","MKL_NUM_THREADS","LD_LIBRARY_PATH")},
        binary_sha256={name:c.digest(name) for name in sorted(paths)})


def verify_freeze(name):
    receipt=json.loads((OUT/name).read_text())
    for artifact,sha in receipt["artifacts"].items():
        c.require(c.digest(OUT/artifact)==sha,"frozen artifact changed: "+artifact)
    return receipt


def measurement_contract():
    receipt=verify_freeze("measurement_freeze.json")
    c.require(receipt["execution_manifest_sha256"]==c.digest(OUT/"execution_manifest.json"),"plan receipt mismatch")
    plan=json.loads((OUT/"execution_manifest.json").read_text());source_guard(plan)
    return receipt,plan


def development_contract():
    _,plan=measurement_contract();gates=verify_freeze("development_gate_freeze.json")
    c.require(gates["measurement_freeze_sha256"]==c.digest(OUT/"measurement_freeze.json"),"development freeze lineage mismatch")
    return gates,plan


def input_guard(plan):
    for name,expected in plan["input_sha256"].items():
        c.require(c.digest(name)==expected,"frozen input changed: "+name)
        print("FROZEN_INPUT_REVERIFIED",name,flush=True)


def freeze_plan():
    c.require(subprocess.check_output(["git","branch","--show-current"],cwd=c.ROOT,text=True).strip()==BRANCH,
        "incorrect R3A branch")
    subprocess.run(["git","merge-base","--is-ancestor",START,"HEAD"],cwd=c.ROOT,check=True)
    c.require(not (OUT/"execution_manifest.json").exists(),"R3A already planned; never overwrite archive")
    history=json.loads((c.OUT/"extraction_manifest.json").read_text())
    artifacts=("extraction_manifest.json","pair_manifest.csv","multilag_visual_measurements.csv",
        "selected_cloud_hashes.csv","selected_image_hashes.csv")
    hashes={str(c.OUT/name):pinned(c.OUT/name) for name in artifacts}
    for path in (c.P4/"sync_stats.csv",c.P4/"visual_increment.csv"):
        hashes[str(path)]=pinned(path)
    raw=[p4.RUNTIME,p4.MANIFEST]+p4.source_shards()+[
        c.DATA/"calibration/floor01_intrinsics.yaml",c.DATA/"calibration/floor01_extrinsics.yaml"]
    for path in raw:
        actual=c.digest(path)
        c.require(actual==history["input_sha256"][str(path)],"raw R3/P4 input hash mismatch")
        hashes[str(path)]=actual
        print("INPUT_HASH_PASS",path.name,flush=True)
    sources=[c.HERE/name for name in SOURCES]+[c.PACKAGE/"scripts/p4_i3_visual_frontend.py",
        c.PACKAGE/"scripts/p4_i3_visual_increment.py",c.PACKAGE/"scripts/p3_r10c_failure_mechanism.py",
        c.HERE/"p9_r3_visual_contract.py",c.HERE/"p9_r2b_nonoracle_evidence.py",OUT/"THEORY.md"]
    for path in sources:
        if path.parent!=OUT and path.name not in SOURCES:pinned(path)
    pairs=c.read_csv(c.OUT/"pair_manifest.csv")
    c.require(len(pairs)==128 and len(set(row["frame"] for row in pairs))==32,"R3 cohort/lag contract invalid")
    c.write_csv(OUT/"pair_manifest.csv",pairs)
    plan=dict(task="PAPER-P9-R3A-GEOMETRY-ASSISTED-METRIC-DEPTH-COVERAGE-GATE",branch=BRANCH,start_sha=START,
        preparation_head=subprocess.check_output(["git","rev-parse","HEAD"],cwd=c.ROOT,text=True).strip(),
        completion_parameters=PARAMETERS,cohort="DEVELOPMENT",targets=history["targets"],lags=list(c.LAGS),
        selection="SMALLEST_VALID_LAG",opencv_threads=1,pnp_seed="transaction_cur",new_ndt_calls=0,
        gt_loaded=False,labels_loaded=False,input_sha256=hashes,
        numerical_environment=numerical_environment(),
        construction_source_sha256={str(path):c.digest(path) for path in sources},
        pair_manifest_sha256=c.digest(OUT/"pair_manifest.csv"))
    save_json("execution_manifest.json",plan)
    return pairs,plan


def selected_data(pairs,calibration):
    started=time.perf_counter()
    ids={int(row["frame"]) for row in pairs}|{int(row["transaction_ref"]) for row in pairs if row["reference_available"]=="1"}
    sync={int(row["transaction_id"]):row for row in c.read_csv(c.P4/"sync_stats.csv")}
    prior_cloud={int(row["transaction_id"]):row for row in c.read_csv(c.OUT/"selected_cloud_hashes.csv")}
    prior_image={int(row["image_index"]):row for row in c.read_csv(c.OUT/"selected_image_hashes.csv")}
    clouds={};records=[]
    for request in p4.requests():
        tx=int(request.transaction_id)
        if tx not in ids:continue
        c.require(request.map_frame=="floor01_map_h1" and request.lidar_frame=="cmu_sp1_velodyne" and
            str(request.scan_end_ns)==sync[tx]["scan_end_ns"] and tx not in clouds,"request identity invalid")
        sha=hashlib.sha256(bytes(request.cloud_end_frame.data)).hexdigest()
        c.require(sha==prior_cloud[tx]["cloud_data_sha256"],"R3 cloud bytes changed")
        clouds[tx]=request.cloud_end_frame
        records.append(dict(transaction_id=tx,cloud_data_sha256=sha,scan_end_ns=int(request.scan_end_ns)))
    c.require(set(clouds)==ids,"selected request cloud missing")
    wanted={int(sync[tx]["image_index"]):int(sync[tx]["image_ns"]) for tx in ids if sync[tx]["eligible"]=="1"}
    images={};image_records=[]
    for index,message in enumerate(p4.camera_messages(p4.source_shards())):
        if index not in wanted:continue
        c.require(message.header.stamp.to_nsec()==wanted[index] and message.header.frame_id=="d" and
            message._type=="sensor_msgs/Image" and message.encoding=="bgr8" and
            (message.width,message.height)==calibration["size"],"camera identity invalid")
        sha=hashlib.sha256(bytes(message.data)).hexdigest()
        c.require(sha==prior_image[index]["raw_image_data_sha256"],"R3 image bytes changed")
        tick=time.perf_counter();gray=p4.rectify(message,calibration);cost=(time.perf_counter()-tick)*1000
        c.require(hashlib.sha256(gray.tobytes()).hexdigest()==prior_image[index]["rectified_gray_sha256"],"MEI replay changed")
        images[index]=(gray,cost)
        image_records.append(dict(image_index=index,raw_image_data_sha256=sha,rectification_ms=cost))
        if len(images)==len(wanted):break
    c.require(set(images)==set(wanted),"selected camera images missing")
    c.write_csv(OUT/"selected_cloud_hashes.csv",records);c.write_csv(OUT/"selected_image_hashes.csv",image_records)
    return clouds,images,time.perf_counter()-started


def blank_measurement(pair,mode):
    keys=("frame","lag","transaction_ref","scan_ref_ns","scan_cur_ns","timestamp_ref_ns","timestamp_cur_ns")
    return dict({key:pair[key] for key in keys},mode=mode,attempted=0,status="SYNC_INVALID",detected=0,
        klt_forward_valid=0,fb_valid=0,direct_depth_count=0,plane_completed_count=0,total_depth_count=0,
        depth_associated=0,pnp_correspondences=0,pnp_inliers=0,inlier_ratio="",reprojection_rmse_px="",
        feature_ms=0.,klt_ms=0.,projection_ms=0.,direct_association_ms=0.,neighbor_search_ms=0.,
        plane_fitting_ms=0.,completion_ms=0.,pnp_ms=0.,frontend_ms=0.,preprocess_ms=0.,total_ms=0.,
        T_Ccur_Cref="",D_vis="")


def extract_development():
    started=time.perf_counter();pairs,plan=freeze_plan()
    calibration=p4.load_calibration(c.DATA/"calibration");p4.cv2.setNumThreads(1);p4.sanity(calibration)
    clouds,images,io=selected_data(pairs,calibration)
    history={(row["frame"],row["lag"]):row for row in c.read_csv(c.OUT/"multilag_visual_measurements.csv")}
    rows=[];correspondences=[];parity=[]
    for pair in pairs:
        old=blank_measurement(pair,"OLD_DIRECT_ONLY");new=blank_measurement(pair,"NEW_AUGMENTED")
        audit=dict(frame=pair["frame"],lag=pair["lag"],attempted=0,count=0,historical_3d_sha256="",
            direct_3d_sha256="",pose_translation_error_m="",pose_rotation_error_deg="",pass_flag="NOT_ATTEMPTED")
        if pair["reference_available"]!="1":old["status"]=new["status"]="REFERENCE_UNAVAILABLE"
        elif pair["sync_ref_eligible"]==pair["sync_cur_eligible"]=="1":
            ref,rcost=images[int(pair["image_index_ref"])];cur,ccost=images[int(pair["image_index_cur"])]
            control,augmented,records,proof=estimate_pair(ref,cur,clouds[int(pair["transaction_ref"])],calibration,int(pair["frame"]))
            old.update(control,attempted=1);new.update(augmented,attempted=1,preprocess_ms=rcost+ccost)
            new["total_ms"]=new["frontend_ms"]+rcost+ccost
            audit.update(proof,attempted=1)
            correspondences.extend(dict(frame=pair["frame"],lag=pair["lag"],transaction_ref=pair["transaction_ref"],**record) for record in records)
        previous=history[(pair["frame"],pair["lag"])]
        for key in ("status","detected","klt_forward_valid","fb_valid","pnp_correspondences","pnp_inliers"):
            c.require(str(old[key])==str(previous[key]),"R3 historical DIRECT parity mismatch: "+key)
        if old["status"]=="VALID":
            dt,dr=c.separation(c.rigid_matrix(old["T_Ccur_Cref"]),c.rigid_matrix(previous["T_Ccur_Cref"]))
            c.require(dt<=1e-10 and dr<=1e-8 and
                abs(float(old["reprojection_rmse_px"])-float(previous["reprojection_rmse_px"]))<=1e-10,"R3 DIRECT PnP mismatch")
            audit.update(historical_pose_translation_error_m=dt,historical_pose_rotation_error_deg=dr)
        else:audit.update(historical_pose_translation_error_m="",historical_pose_rotation_error_deg="")
        rows.extend((old,new));parity.append(audit)
        print("PAIR",pair["frame"],pair["lag"],"DIRECT",old["pnp_correspondences"],"COMPLETED",new["plane_completed_count"],
            "TOTAL",new["pnp_correspondences"],"INLIERS",new["pnp_inliers"],new["status"],flush=True)
    source_guard(plan)
    c.write_csv(OUT/"coverage_by_pair.csv",rows);c.write_csv(OUT/"depth_completion_correspondences.csv",correspondences)
    c.write_csv(OUT/"direct_parity.csv",parity)
    names=("coverage_by_pair.csv","depth_completion_correspondences.csv","direct_parity.csv","selected_cloud_hashes.csv","selected_image_hashes.csv","pair_manifest.csv")
    save_json("measurement_freeze.json",dict(execution_manifest_sha256=c.digest(OUT/"execution_manifest.json"),
        artifacts={name:c.digest(OUT/name) for name in names},rows=len(rows),wall_seconds=time.perf_counter()-started,
        io_seconds=io,gt_loaded=False,labels_loaded=False,new_ndt_calls=0))
    print("DEVELOPMENT_MEASUREMENTS_FROZEN=YES",flush=True)


def full_floor01():
    gates,plan=development_contract()
    c.require(gates["coverage_pass"] and gates["geometry_safety_pass"],"FULL_RUN_FORBIDDEN_DEVELOPMENT_GATE_FAIL")
    quality=verify_freeze("posthoc_quality.json")
    c.require(quality["development_gate_sha256"]==c.digest(OUT/"development_gate_freeze.json"),"quality gate lineage changed")
    c.require(quality["quality_pass"],"FULL_RUN_FORBIDDEN_QUALITY_GATE_FAIL")
    c.require(not (OUT/"full_measurements.csv").exists() and not (OUT/"full_run_started.json").exists(),"full run already started")
    input_guard(plan)
    save_json("full_run_started.json",dict(development_gate_sha256=c.digest(OUT/"development_gate_freeze.json"),
        posthoc_quality_sha256=c.digest(OUT/"posthoc_quality.json"),completion_parameters=PARAMETERS))
    calibration=p4.load_calibration(c.DATA/"calibration");p4.cv2.setNumThreads(1)
    sync=c.read_csv(c.P4/"sync_stats.csv");c.require(len(sync)==4127,"full sync contract")
    images=iter(p4.camera_messages(p4.source_shards()));image_index=-1;previous=None;rows=[]
    started=time.perf_counter();writer=None
    with gzip.open(OUT/"full_depth_correspondences.csv.gz","wt",newline="") as stream:
        for i,request in enumerate(p4.requests()):
            match=sync[i];c.require(str(request.transaction_id)==match["transaction_id"] and
                str(request.scan_end_ns)==match["scan_end_ns"],"full request identity mismatch")
            while image_index<int(match["image_index"]):image=next(images);image_index+=1
            c.require(image.header.stamp.to_nsec()==int(match["image_ns"]) and image.header.frame_id=="d" and
                image._type=="sensor_msgs/Image" and image.encoding=="bgr8" and
                (image.width,image.height)==calibration["size"],"full camera identity mismatch")
            tick=time.perf_counter();gray=p4.rectify(image,calibration) if match["eligible"]=="1" else None
            preprocess=(time.perf_counter()-tick)*1000
            if previous is not None:
                old_request,old_match,old_gray,old_cost=previous
                pair=dict(frame=request.transaction_id,lag=1,transaction_ref=old_request.transaction_id,
                    scan_ref_ns=old_request.scan_end_ns,scan_cur_ns=request.scan_end_ns,
                    timestamp_ref_ns=old_match["image_ns"],timestamp_cur_ns=match["image_ns"])
                row=blank_measurement(pair,"NEW_AUGMENTED")
                if gray is not None and old_gray is not None:
                    c.require(int(match["image_index"])>int(old_match["image_index"]),"non-forward full image pair")
                    _,augmented,records,_=estimate_pair(old_gray,gray,old_request.cloud_end_frame,calibration,request.transaction_id,replay=False)
                    row.update(augmented,attempted=1,preprocess_ms=preprocess+old_cost,
                        total_ms=augmented["frontend_ms"]+preprocess+old_cost)
                    for record in records:
                        record=dict(frame=request.transaction_id,lag=1,transaction_ref=old_request.transaction_id,**record)
                        if writer is None:writer=csv.DictWriter(stream,fieldnames=list(record),lineterminator="\n");writer.writeheader()
                        writer.writerow(record)
                rows.append(row)
            previous=(request,match,gray,preprocess)
            if i and i%200==0:print("FULL",i,"VALID",sum(row["status"]=="VALID" for row in rows),flush=True)
    extraction_wall=time.perf_counter()-started
    c.require(len(rows)==4126,"full pair count mismatch");source_guard(plan);input_guard(plan)
    c.write_csv(OUT/"full_measurements.csv",rows)
    save_json("full_measurement_freeze.json",dict(rows=len(rows),wall_seconds=extraction_wall,
        execution_manifest_sha256=c.digest(OUT/"execution_manifest.json"),full_run_started_sha256=c.digest(OUT/"full_run_started.json"),
        artifacts={name:c.digest(OUT/name) for name in ("full_measurements.csv","full_depth_correspondences.csv.gz")},
        gt_scoring_done=False,parameters_changed=False,new_ndt_calls=0))
    print("FULL_FLOOR01_MEASUREMENTS_FROZEN",len(rows),flush=True)


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("stage",choices=("development","full"));args=parser.parse_args()
    {"development":extract_development,"full":full_floor01}[args.stage]()
