"""Post-freeze recovery, shadow parity, cost and fixed-anchor GT diagnostics."""
import argparse
from collections import Counter
import json
import sys
import numpy as np
from scipy.spatial.transform import Rotation
from run_budgeted import ARCHIVE, ROOT, SAME, read, csv_write, json_write, sha


def matrix(text):
    result = np.array(list(map(float, text.split(";"))))
    if result.size != 16 or not np.isfinite(result).all():
        raise RuntimeError("nonfinite or malformed pose")
    return result.reshape(4, 4)


def distance(a, b):
    return float(np.linalg.norm(a[:3, 3]-b[:3, 3])), float(np.degrees(
        (Rotation.from_matrix(a[:3, :3]).inv()*Rotation.from_matrix(b[:3, :3])).magnitude()))


def statistics(rows):
    def mean(key): return float(np.mean([float(r[key]) for r in rows]))
    def p95(key): return float(np.percentile([float(r[key]) for r in rows], 95))
    return dict(frames=len(rows), calls=sum(int(r["full_ndt_calls"]) for r in rows),
        max_calls=max(int(r["full_ndt_calls"]) for r in rows), max_previews=max(int(r["preview_count"]) for r in rows),
        mean_previews=mean("preview_count"), expanded=sum(r["expanded"]=="1" for r in rows),
        recommended_alternatives=sum(int(r["recommended_id"])>=0 for r in rows),
        recommended_nonlocal=sum(float(r["recommended_dt_m"])>.2 or float(r["recommended_dr_deg"])>2 for r in rows),
        mean_total_ms=mean("frame_total_ms"), P95_total_ms=p95("frame_total_ms"),
        mean_frame_cpu_ms=mean("frame_cpu_ms"), P95_frame_cpu_ms=p95("frame_cpu_ms"),
        mean_increment_ms=mean("shadow_ms"), P95_increment_ms=p95("shadow_ms"),
        mean_increment_cpu_ms=mean("shadow_cpu_ms"), P95_increment_cpu_ms=p95("shadow_cpu_ms"),
        mean_nominal_align_ms=mean("nominal_alignment_ms"),
        phase_mean_ms={key:mean(key) for key in ("model_ms", "jet_ms", "solve_ms", "preview_ms", "refinement_ms")},
        status=dict(Counter(r["shadow_status"] for r in rows)))


def parity(archive):
    rows=[]
    for method in "ABC":
        for name in ("registration.csv", "trajectory.csv"):
            original, shadow = read(archive/"continuous_control"/name), read(archive/f"continuous_{method}"/name)
            if len(original)!=200 or len(shadow)!=200:
                raise RuntimeError("continuous ledger must contain all 200 frames")
            for a,b in zip(original,shadow):
                keys=[k for k in a if k not in ("alignment_ms",)]
                mismatched=[k for k in keys if a[k]!=b[k]]
                rows.append(dict(method=method, transaction_id=a["transaction_id"], table=name,
                                 exact_nominal_state_parity=int(not mismatched), mismatched_fields=";".join(mismatched)))
    csv_write(archive/"nominal_state_parity.csv",rows)
    if any(not r["exact_nominal_state_parity"] for r in rows):
        raise RuntimeError("shadow changed nominal/source/state")
    historical={r["transaction_id"]:r for r in read(SAME/"registration.csv")}
    diagnostic=[]
    for row in read(archive/"integration_C/frames.csv"):
        old=historical[row["transaction_id"]]
        def old_pose(prefix):
            out=np.eye(4); out[:3,3]=[float(old[prefix+"_"+a]) for a in "xyz"]
            out[:3,:3]=Rotation.from_quat([float(old[prefix+"_q"+a]) for a in "xyzw"]).as_matrix(); return out
        dt,dr=distance(matrix(row["nominal_pose"]),old_pose("raw"))
        ok=(row["source_hash"]==old["source_cloud_hash"] and row["source_count"]==old["source_points"]
            and dt<=1e-5 and dr<=1e-4 and abs(float(row["nominal_probability"])-float(old["transformation_probability"]))<=1e-6)
        diagnostic.append(dict(transaction_id=row["transaction_id"], translation_m=dt, rotation_deg=dr, pass_parity=int(ok)))
    csv_write(archive/"historical_nominal_parity.csv",diagnostic)
    if not all(r["pass_parity"] for r in diagnostic): raise RuntimeError("historical nominal input parity failed")


def recovery(archive):
    canonical=ROOT/"docs/p9_r1a_true_profile_closure/canonical_oracle.csv"
    if sha(canonical)!="e3b955a700a6205eb48e473186daec22f44b711bb357a2d682f31cbaeb745458":
        raise RuntimeError("canonical evaluation reference changed")
    targets={(616,"P02"),(616,"P03"),(616,"P10"),(616,"P12"),(616,"P13"),(616,"P17"),(2226,"P09")}
    result=[]
    for method in "ABC":
        candidates=read(archive/f"two_{method}/candidates.csv")
        frames={int(r["transaction_id"]):r for r in read(archive/f"two_{method}/frames.csv")}
        for target in read(canonical):
            tx, cid=int(target["transaction_id"]),target["cluster_id"]
            if (tx,cid) not in targets: continue
            pose=matrix(target["closed_pose_matrix16"])
            c=[r for r in candidates if int(r["transaction_id"])==tx]
            preview=[distance(matrix(r["preview_pose"]),pose) for r in c if r["finite"]=="1"]
            refined=[distance(matrix(r["refined_pose"]),pose) for r in c if r["refined_successful"]=="1"]
            selected=distance(matrix(frames[tx]["recommended_pose"]),pose)
            result.append(dict(method=method, transaction_id=tx, canonical_id=cid,
                preview_recovered=int(any(dt<=.2 and dr<=2 for dt,dr in preview)),
                refined_recovered=int(any(dt<=.2 and dr<=2 for dt,dr in refined)),
                recommended_recovered=int(selected[0]<=.2 and selected[1]<=2),
                recommended_dt_m=selected[0],recommended_dr_deg=selected[1],
                evaluation_only="overlapping canonical neighborhoods, not independent localization successes"))
    csv_write(archive/"case_results.csv",result)
    return {m:{k:sum(r[k] for r in result if r["method"]==m) for k in
                   ("preview_recovered","refined_recovered","recommended_recovered")} for m in "ABC"}


def posthoc_gt(archive):
    # No GT file can be opened until every candidate recommendation is frozen.
    sys.path.insert(0,str(ROOT/"src/dog_prior_map_fastlio2_frontend_exp/scripts"))
    import p5_i1_posthoc_gt as contract
    import yaml
    if sha(contract.GT)!=contract.EXPECTED_GT_SHA or sha(contract.EXTRINSICS)!=contract.EXPECTED_EXTR_SHA:
        raise RuntimeError("historical GT/calibration digest mismatch")
    receipt=json.loads((ROOT/"docs/p9_r2a_predictor_conditioned_search/results.json").read_text())["gt_contract"]
    old_path=SAME/"posthoc_gt/frame_mode_metrics.csv"
    if sha(old_path)!=receipt["input_sha256"][str(old_path)]:
        raise RuntimeError("historical GT parity metrics changed")
    json_write(archive/"gt_contract_receipt.json",dict(fixed_reference_alignment=receipt["fixed_reference_alignment"],
        anchor_stamp=receipt["anchor_stamp"], GT_USED="POSTHOC_ONLY", new_alignment_fitted=False,
        input_sha256={str(p):sha(p) for p in (contract.GT,contract.EXTRINSICS,old_path,
        ROOT/"docs/p9_r2a_predictor_conditioned_search/results.json", ROOT/"src/dog_prior_map_fastlio2_frontend_exp/scripts/p5_i1_posthoc_gt.py")}))
    anchor=matrix(receipt["fixed_reference_alignment"])
    extr=np.array(yaml.safe_load(contract.EXTRINSICS.read_text())["laser_to_imu"]["data"]).reshape(4,4)
    extr[:3,:3]=Rotation.from_matrix(extr[:3,:3]).as_matrix(); lidar_to_imu=np.linalg.inv(extr)
    times,poses=contract.gt_data(contract.GT)
    old={r["transaction_id"]:r for r in read(old_path)}
    rows=[]; gt_parity=[]
    for job in ["integration_C"]+[f"continuous_{m}" for m in "ABC"]+[f"two_{m}" for m in "ABC"]:
        for row in read(archive/job/"frames.csv"):
            raw=contract.interpolate_gt(times,poses,int(row["stamp_ns"])/1e9)
            if raw is None: raise RuntimeError("GT does not bracket frame")
            reference=anchor@raw
            nt,nr=contract.pose_error(matrix(row["nominal_pose"])@lidar_to_imu,reference)
            st,sr=contract.pose_error(matrix(row["recommended_pose"])@lidar_to_imu,reference)
            if job=="integration_C":
                previous=old[row["transaction_id"]]
                dt=abs(nt-float(previous["baseline_raw_translation_gt_error_m"]))
                dr=abs(nr-float(previous["baseline_raw_rotation_gt_error_deg"]))
                gt_parity.append(dict(transaction_id=row["transaction_id"], translation_diff_m=dt,rotation_diff_deg=dr,
                                      pass_parity=int(dt<=1e-4 and dr<=1e-3)))
            rows.append(dict(job=job,transaction_id=row["transaction_id"],nominal_translation_m=nt,
                recommended_translation_m=st,nominal_rotation_deg=nr,recommended_rotation_deg=sr,
                recommended_id=row["recommended_id"],translation_change_m=st-nt,rotation_change_deg=sr-nr,
                translation_outcome="SAME" if abs(st-nt)<=1e-6 else "IMPROVED" if st<nt else "WORSE"))
    csv_write(archive/"gt_contract_parity.csv",gt_parity)
    if not all(r["pass_parity"] for r in gt_parity): raise RuntimeError("map/GT fixed-anchor parity failed")
    csv_write(archive/"posthoc_gt.csv",rows)
    summaries={}
    for job in sorted({r["job"] for r in rows}):
        subset=[r for r in rows if r["job"]==job]
        summaries[job]=dict(count=len(subset),outcomes=dict(Counter(r["translation_outcome"] for r in subset)),
            nominal_translation_RMSE_m=float(np.sqrt(np.mean([r["nominal_translation_m"]**2 for r in subset]))),
            recommended_translation_RMSE_m=float(np.sqrt(np.mean([r["recommended_translation_m"]**2 for r in subset]))),
            nominal_rotation_RMSE_deg=float(np.sqrt(np.mean([r["nominal_rotation_deg"]**2 for r in subset]))),
            recommended_rotation_RMSE_deg=float(np.sqrt(np.mean([r["recommended_rotation_deg"]**2 for r in subset]))))
    return summaries


def evaluate(attempt):
    archive=ARCHIVE/f"attempt_{attempt}"
    freeze=json.loads((archive/"blind_outputs_freeze.json").read_text())
    for name,digest in freeze["output_sha256"].items():
        if sha(archive/name)!=digest: raise RuntimeError("frozen output changed: "+name)
    jobs=["integration_C"]+[f"two_{m}" for m in "ABC"]+["continuous_control"]+[f"continuous_{m}" for m in "ABC"]
    summaries={}
    for job in jobs:
        rows=read(archive/job/"frames.csv")
        expected=32 if job=="integration_C" else 2 if job.startswith("two_") else 200
        if len(rows)!=expected: raise RuntimeError("missing frame records")
        for row in rows:
            if int(row["full_ndt_calls"])>3 or int(row["preview_count"])>16 or row["nonfinite_recommended"]!="0":
                raise RuntimeError("hard engineering budget/finite guard failed")
            matrix(row["recommended_pose"]); matrix(row["nominal_pose"])
        summaries[job]=statistics(rows)
        summaries[job]["peak_rss_kib"]=int(read(archive/job/"resources.csv")[0]["peak_rss_kib"])
    parity(archive)
    # Controlled A/B/C compare identical weak proposal IDs and visitation.
    pools=[]
    for method in "ABC":
        pools.append([(r["transaction_id"],r["candidate_id"],r["u"]) for r in read(archive/f"two_{method}/candidates.csv")])
    if pools[0]!=pools[1] or pools[1]!=pools[2]: raise RuntimeError("controlled weak pools/order differed")
    result=dict(attempt=attempt,hard_engineering="PASS",nominal_state_parity="1200/1200 EXACT",statistics=summaries,
                recovery=recovery(archive),GT=posthoc_gt(archive),GT_USED_FOR_SELECTION=False,POSE_SWITCHED=False)
    json_write(archive/"evaluation.json",result)
    print(json.dumps(result,indent=2))


if __name__=="__main__":
    parser=argparse.ArgumentParser(); parser.add_argument("--attempt",type=int,default=0)
    evaluate(parser.parse_args().attempt)
