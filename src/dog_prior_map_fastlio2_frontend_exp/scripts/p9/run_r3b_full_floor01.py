#!/usr/bin/env python3
"""Conditional once-only full run; never called before the development gate."""
import json
import time

import p9_r3_visual_contract as c
import run_r3a_metric_depth_coverage as r3a
import p4_i3_visual_increment as p4
from p9_r3b_pnp_frontend import estimate_pair
from run_r3b_pnp_admission import OUT,quality_contract,save,guard,reserve_full_run


def run():
    gates,quality,plan=quality_contract()
    c.require(quality["development_pnp_coverage_gate"] and gates["coverage_pass"] and
        gates["parity_pass"] and gates["regression_pass"] and quality["quality_pass"],"FULL_FORBIDDEN_DEVELOPMENT_GATE_FAIL")
    old_plan=json.loads((r3a.OUT/"execution_manifest.json").read_text());r3a.input_guard(old_plan)
    reserve_full_run(plan)
    calibration=p4.load_calibration(c.DATA/"calibration");p4.cv2.setNumThreads(1)
    sync=c.read_csv(c.P4/"sync_stats.csv");c.require(len(sync)==4127,"full sync4127 parity")
    images=iter(p4.camera_messages(p4.source_shards()));image_index=-1;previous=None;rows=[]
    started=time.perf_counter()
    for index,request in enumerate(p4.requests()):
        match=sync[index]
        c.require(str(request.transaction_id)==match["transaction_id"] and str(request.scan_end_ns)==match["scan_end_ns"],"full request identity mismatch")
        while image_index<int(match["image_index"]):image=next(images);image_index+=1
        c.require(image.header.stamp.to_nsec()==int(match["image_ns"]) and image.header.frame_id=="d" and
            image._type=="sensor_msgs/Image" and image.encoding=="bgr8" and
            (image.width,image.height)==calibration["size"],"full image identity mismatch")
        tick=time.perf_counter();gray=p4.rectify(image,calibration) if match["eligible"]=="1" else None
        rectification_ms=(time.perf_counter()-tick)*1000
        if previous is not None:
            old_request,old_match,old_gray,old_ms=previous
            row=dict(frame=request.transaction_id,lag=1,transaction_ref=old_request.transaction_id,
                timestamp_ref_ns=old_match["image_ns"],timestamp_cur_ns=match["image_ns"],
                scan_ref_ns=old_request.scan_end_ns,scan_cur_ns=request.scan_end_ns,
                attempted=0,status="SYNC_INVALID",pnp_correspondences=0,pnp_inliers=0,inlier_ratio="",
                reprojection_rmse_px="",grid_occupancy="",hull_fraction="",median_parallax_px="",
                pnp_ms=0.,pnp_attempted=0,T_Ccur_Cref="",D_vis="",inlier_indices="",cheirality_pass="",
                detected=0,klt_forward_valid=0,fb_valid=0,direct_depth_count=0,plane_completed_count=0,total_depth_count=0,
                depth_associated=0,feature_ms=0.,klt_ms=0.,projection_ms=0.,direct_association_ms=0.,
                neighbor_search_ms=0.,plane_fitting_ms=0.,completion_ms=0.,frontend_ms=0.,preprocess_ms=0.,total_ms=0.)
            if gray is not None and old_gray is not None:
                c.require(int(match["image_index"])>int(old_match["image_index"]),"non-forward full image pair")
                measured=estimate_pair(old_gray,gray,old_request.cloud_end_frame,calibration,request.transaction_id)
                row.update(measured,attempted=1,preprocess_ms=rectification_ms+old_ms,
                    total_ms=measured["frontend_ms"]+rectification_ms+old_ms)
            rows.append(row)
        previous=(request,match,gray,rectification_ms)
        if index and index%200==0:print("R3B_FULL",index,"VALID",sum(row["status"]=="VALID" for row in rows),flush=True)
    wall=time.perf_counter()-started
    c.require(len(rows)==4126,"full pair count mismatch");guard(plan);r3a.input_guard(old_plan)
    c.write_csv(OUT/"full_measurements.csv",rows)
    save("full_measurement_freeze.json",dict(artifacts={"full_measurements.csv":c.digest(OUT/"full_measurements.csv")},
        rows=len(rows),wall_seconds=wall,execution_manifest_sha256=c.digest(OUT/"execution_manifest.json"),
        posthoc_quality_sha256=c.digest(OUT/"posthoc_quality.json"),full_run_started_sha256=c.digest(OUT/"full_run_started.json"),
        gt_scoring_done=False,new_ndt_calls=0,parameters_changed=False))
    print("R3B_FULL_MEASUREMENTS_FROZEN",len(rows),flush=True)


if __name__=="__main__":run()
