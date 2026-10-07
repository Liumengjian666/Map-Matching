#!/usr/bin/env python3
"""Blind construction only: no oracle labels, canonical poses, GT or optimizer."""
import argparse
import json
from unittest.mock import patch

import numpy as np

import p9_r3_visual_contract as c


def audit_extraction():
    manifest=json.loads((c.OUT/"extraction_manifest.json").read_text())
    receipt=json.loads((c.OUT/"extraction_receipt.json").read_text())
    c.require(receipt["extraction_manifest_sha256"]==c.digest(c.OUT/"extraction_manifest.json"),"extraction plan changed")
    c.require(receipt["rows"]==128 and receipt["NEW_NDT_CALLS"]==0 and
              not receipt["label_or_gt_evaluation_loaded"],"invalid extraction stage")
    for path,h in manifest["construction_source_sha256"].items():
        c.require(c.digest(path)==h,"extraction source changed: "+path)
    for name,h in {**manifest["plan_artifact_sha256"],**receipt["selected_input_hashes"],
                   "multilag_visual_measurements.csv":receipt["measurement_sha256"]}.items():
        c.require(c.digest(c.OUT/name)==h,"extracted artifact changed: "+name)
    # Small committed inputs are rechecked now. Raw bags were fully hashed before extraction;
    # selected cloud/image bytes are additionally archived, not re-read for each audit stage.
    for path,h in manifest["input_sha256"].items():
        if not path.endswith(".bag"):
            c.require(c.digest(path)==h,"construction input changed: "+path)
    return manifest


def construct(manifest):
    frames=manifest["targets"]
    nominals={int(row["frame"]):c.rigid_matrix(row["nominal_pose_matrix16"])
              for row in c.read_csv(c.R2A/"predictor_parity.csv")}
    c.require(sorted(nominals)==frames,"nominal cohort changed")
    clusters=[row for row in c.read_csv(c.R2B/"terminal_clusters.csv") if row["budget"]=="12"]
    nominal_scores={int(row["frame"]):float(row["nominal_raw_score"]) for row in clusters if row["is_nominal"]=="1"}
    c.require(sorted(nominal_scores)==frames,"nominal score coverage invalid")
    inherited=[row for row in c.read_csv(c.R2B/"competitive_terminals.csv") if row["budget"]=="12"]
    measurements=c.read_csv(c.OUT/"multilag_visual_measurements.csv")
    c.require(len(measurements)==128 and len({(row["frame"],row["lag"]) for row in measurements})==128,
              "fixed multi-lag keys invalid")
    inverse_il=np.linalg.inv(c.vector(manifest["T_imu_lidar"],16).reshape(4,4))
    candidates=[];selected_rows=[];residuals=[];evidence=[]
    for frame in frames:
        selected=c.choose_smallest_valid([row for row in measurements if int(row["frame"])==frame])
        available=selected is not None
        measurement=dict(frame=frame,visual_available=int(available),selected_lag="" if not available else selected["lag"],
            status="VISUAL_UNAVAILABLE" if not available else "VALID")
        fields=("transaction_ref","scan_ref_ns","scan_cur_ns","timestamp_ref_ns","timestamp_cur_ns","detected",
                "klt_forward_valid","fb_valid","depth_associated","pnp_correspondences","pnp_inliers","inlier_ratio",
                "reprojection_rmse_px","T_Ccur_Cref","D_vis","reference_map_T_imu","nominal_map_T_imu")
        measurement.update({key:"" if not available else selected[key] for key in fields})
        selected_rows.append(measurement)
        nominal=nominals[frame]
        points=[dict(frame=frame,cluster_id="NOMINAL",is_nominal=1,raw_score=nominal_scores[frame],
            score_advantage=0.,translation_from_nominal_m=0.,rotation_from_nominal_deg=0.,
            strict_center_eligible=0,map_T_lidar=c.text(nominal),map_T_imu=c.text(nominal@inverse_il))]
        for row in inherited:
            if int(row["frame"])!=frame:continue
            pose=c.rigid_matrix(row["representative_pose_matrix16"])
            dt,dr,eligible=c.strict_separation(nominal,pose)
            points.append(dict(frame=frame,cluster_id=row["cluster_id"],is_nominal=0,raw_score=float(row["raw_score"]),
                score_advantage=float(row["score_advantage"]),translation_from_nominal_m=dt,rotation_from_nominal_deg=dr,
                strict_center_eligible=int(eligible),map_T_lidar=c.text(pose),map_T_imu=c.text(pose@inverse_il)))
        candidates.extend(points)
        frame_residuals=[]
        for point in points:
            rt=rr=""
            if available:
                reference=c.vector(selected["reference_map_T_imu"],16).reshape(4,4)
                increment=c.vector(selected["D_vis"],16).reshape(4,4)
                rt,rr=c.residual(reference,c.vector(point["map_T_imu"],16).reshape(4,4),increment)
                c.require(np.isfinite([rt,rr]).all(),"non-finite visual candidate residual")
            row=dict(point,visual_available=int(available),selected_lag=measurement["selected_lag"],
                     translation_residual_m=rt,rotation_residual_deg=rr)
            residuals.append(row);frame_residuals.append(row)
        alternatives=[row for row in frame_residuals if row["strict_center_eligible"]]
        best=min(alternatives,key=lambda row:(row["translation_residual_m"],row["cluster_id"])) if available and alternatives else None
        scalars=c.evidence_scalars(available,frame_residuals[0]["translation_residual_m"],
                                  [row["translation_residual_m"] for row in alternatives] if available else [])
        evidence.append(dict(frame=frame,visual_available=int(available),selected_lag=measurement["selected_lag"],
            pnp_inliers=measurement["pnp_inliers"],reprojection_rmse_px=measurement["reprojection_rmse_px"],
            inherited_competitive_count=len(points)-1,strict_competitive_count=len(alternatives),
            inside_center_excluded_count=len(points)-1-len(alternatives),**scalars,
            best_alt_cluster_id="" if best is None else best["cluster_id"],
            r_nom_rotation_deg=frame_residuals[0]["rotation_residual_deg"],
            r_best_alt_rotation_deg="" if best is None else best["rotation_residual_deg"],
            rotation_margin_deg="" if best is None else frame_residuals[0]["rotation_residual_deg"]-best["rotation_residual_deg"],
            best_alt_translation_separation_m="" if best is None else best["translation_from_nominal_m"],
            best_alt_rotation_separation_deg="" if best is None else best["rotation_from_nominal_deg"]))
    return candidates,selected_rows,residuals,evidence


def build():
    c.require(not (c.OUT/"evidence_freeze.json").exists(),"visual evidence already frozen")
    manifest=audit_extraction();rows=construct(manifest)
    names=("visual_candidates.csv","visual_measurements.csv","visual_candidate_residuals.csv","visual_nonoracle_evidence.csv")
    for name,data in zip(names,rows):c.write_csv(c.OUT/name,data)
    frozen=dict(stage="NONORACLE_VISUAL_EVIDENCE_FROZEN_BEFORE_LABELS_GT",targets=manifest["targets"],
        extraction_manifest_sha256=c.digest(c.OUT/"extraction_manifest.json"),
        extraction_receipt_sha256=c.digest(c.OUT/"extraction_receipt.json"),NEW_NDT_CALLS=0,
        GT_USED_FOR_EVIDENCE=False,labels_loaded=False,canonical_inputs_loaded=False,
        builder_source_sha256=c.digest(__file__),frozen_artifact_sha256={name:c.digest(c.OUT/name) for name in names})
    (c.OUT/"evidence_freeze.json").write_text(json.dumps(frozen,indent=2,sort_keys=True,allow_nan=False)+"\n")
    print("P9_R3_NONORACLE_VISUAL_EVIDENCE_FROZEN",frozen["frozen_artifact_sha256"],flush=True)


def audit_frozen():
    manifest=audit_extraction();frozen=json.loads((c.OUT/"evidence_freeze.json").read_text())
    c.require(frozen["stage"]=="NONORACLE_VISUAL_EVIDENCE_FROZEN_BEFORE_LABELS_GT" and not frozen["labels_loaded"] and
        not frozen["GT_USED_FOR_EVIDENCE"] and frozen["builder_source_sha256"]==c.digest(__file__),"evidence boundary/source changed")
    c.require(c.digest(c.OUT/"extraction_manifest.json")==frozen["extraction_manifest_sha256"] and
              c.digest(c.OUT/"extraction_receipt.json")==frozen["extraction_receipt_sha256"],"extraction freeze chain changed")
    names=("visual_candidates.csv","visual_measurements.csv","visual_candidate_residuals.csv","visual_nonoracle_evidence.csv")
    for name,expected in zip(names,construct(manifest)):
        c.require(c.digest(c.OUT/name)==frozen["frozen_artifact_sha256"][name],"frozen evidence changed: "+name)
        c.require(c.read_csv(c.OUT/name)==[{key:str(value) for key,value in row.items()} for row in expected],
                  "blind CSV recomputation mismatch: "+name)
    return frozen


def self_test():
    c.self_test()
    identity=c.text(np.eye(4));far=np.eye(4);far[0,3]=.3
    fields=("transaction_ref","scan_ref_ns","scan_cur_ns","timestamp_ref_ns","timestamp_cur_ns","detected",
            "klt_forward_valid","fb_valid","depth_associated","pnp_correspondences","pnp_inliers","inlier_ratio",
            "reprojection_rmse_px","T_Ccur_Cref","D_vis","reference_map_T_imu","nominal_map_T_imu")
    measurements=[]
    frames=list(range(1,33))
    for frame in frames:
        for lag in c.LAGS:
            row={key:0 for key in fields}
            row.update(frame=frame,lag=lag,status="VALID" if frame==1 and lag in (2,8) else "INVALID",
                       D_vis=identity,reference_map_T_imu=identity,nominal_map_T_imu=identity)
            measurements.append(row)
    fixtures={
        c.R2A/"predictor_parity.csv":[dict(frame=f,nominal_pose_matrix16=identity) for f in frames],
        c.R2B/"terminal_clusters.csv":[dict(frame=f,budget="12",is_nominal="1",nominal_raw_score="100") for f in frames],
        c.R2B/"competitive_terminals.csv":[dict(frame="2",budget="12",cluster_id="C001",representative_pose_matrix16=c.text(far),
            raw_score="101",score_advantage="1")],
        c.OUT/"multilag_visual_measurements.csv":measurements}
    with patch.object(c,"read_csv",side_effect=lambda path:fixtures[path]):
        candidates,selected,residuals,evidence=construct(dict(targets=frames,T_imu_lidar=identity))
    c.require(len(candidates)==33 and len(residuals)==33 and selected[0]["selected_lag"]==2,
              "blind construction candidate/availability test failed")
    c.require(evidence[0]["U_visual"]==0. and evidence[0]["r_best_alt"]=="" and
              evidence[1]["strict_competitive_count"]==1 and evidence[1]["U_visual"]=="" and
              all(row["translation_residual_m"]=="" for row in residuals if row["frame"]==2),
              "construction conflated unavailable with zero evidence")
    print("P9_R3_BLIND_CONSTRUCTION_MISSING_EMPTY_SET_SELF_TEST=PASS")


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("stage",choices=("build","audit","self-test"));args=parser.parse_args()
    {"build":build,"audit":audit_frozen,"self-test":self_test}[args.stage]()
