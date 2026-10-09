"""Independent R7 covariance/budget/causal audit, then fixed post-hoc GT."""
import hashlib
import json
from collections import Counter
from pathlib import Path
import subprocess
import numpy as np
from scipy.spatial.transform import Rotation
from run_budgeted import ROOT,read,sha,csv_write,json_write
from run_directional import ARCHIVE,MODES,R6
from run_admission import CONTROL
from evaluate_admission import load
from evaluate_budgeted import matrix,distance
from evaluate_anchored import carrier_quaternion,chart
from evaluate_weak_refinement import vector,imu_pose,statistics,posthoc
from evaluate_event import moments

def skew(v):
    x,y,z=v;return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])

def left_jacobian(phi):
    angle=np.linalg.norm(phi);K=skew(phi)
    if angle<1e-6:return np.eye(3)+.5*K+K@K/6
    return np.eye(3)+(1-np.cos(angle))/angle**2*K+(angle-np.sin(angle))/angle**3*K@K

def covariance_check(c,r,extr):
    Q=vector(r["eigenvectors"]).reshape(6,6);l=vector(r["eigenvalues"]);k=int(r["weak_dimension"])
    W=Q[:,:k];S=Q[:,k:];J=vector(c["jacobian"]).reshape(6,6)
    R0=vector(c["baseline_covariance"]).reshape(6,6);Rnew=vector(c["covariance"]).reshape(6,6)
    pred=matrix(c["predicted_imu_pose"]);measured=matrix(c["candidate_imu_pose"])
    nominal=matrix(r["nominal_pose"])
    Rnom=Rotation.from_quat(carrier_quaternion(nominal[:3,:3])).as_matrix()
    Rlid=measured[:3,:3]@extr[:3,:3]
    theta=Rotation.from_matrix(Rlid@Rnom.T).as_rotvec()
    phi=Rotation.from_matrix(pred[:3,:3].T@measured[:3,:3]).as_rotvec()
    expected=np.zeros((6,6));expected[:3,:3]=.8*np.eye(3)
    expected[:3,3:]=skew(measured[:3,:3]@extr[:3,3])@left_jacobian(theta)
    expected[3:,3:]=np.linalg.solve(left_jacobian(phi),pred[:3,:3].T)@left_jacobian(theta)
    chart0=np.linalg.solve(J,np.linalg.solve(J,R0).T).T
    chart_new=np.linalg.solve(J,np.linalg.solve(J,Rnew).T).T
    m=np.clip(l[k]/np.maximum(l[:k],1e-6*l[k]),1,20)
    variances=np.diag(W.T@chart0@W)
    rebuilt=R0.copy()
    for i in range(k):rebuilt+=(m[i]-1)*variances[i]*np.outer(J@W[:,i],J@W[:,i])
    error=max(np.max(np.abs(J-expected)),np.max(np.abs(Rnew-rebuilt)),
        np.max(np.abs(m-vector(c["multipliers"]))),np.max(np.abs(variances-vector(c["weak_variances"]))),
        np.max(np.abs(S.T@(chart_new-chart0)@S)),
        np.max(np.abs(np.diag(W.T@chart_new@W)-m*variances)))
    spd=np.linalg.eigvalsh(Rnew).min()>0
    psd=np.linalg.eigvalsh(Rnew-R0).min()>-1e-10
    ok=(error<=1e-8 and spd and psd and abs(float(c["epsilon"])-1e-6*l[k])<1e-12 and
        np.max(np.abs(R0-np.diag([.04]*3+[.01]*3)))<1e-14 and
        np.max(np.abs(Rnew-Rnew.T))<1e-10)
    return dict(covariance_pass=int(ok),max_reconstruction_error=float(error),
        minimum_eigenvalue=float(np.linalg.eigvalsh(Rnew).min()),
        weak_relative_gap=float((l[1]-l[0])/max(l[1],1e-30)) if k==2 else 1.)

def verify(directory):
    freeze=json.loads((directory/"execution_freeze.json").read_text())
    blind=json.loads((directory/"blind_outputs_freeze.json").read_text())
    if blind["GT_LOADED"] or blind["ORACLE_LOADED"]:raise RuntimeError("isolation failed")
    for p,h in blind["output_sha256"].items():
        if sha(directory/p)!=h:raise RuntimeError("runtime output changed")
    for p,h in freeze["inherited_sha256"].items():
        if sha(ROOT/p)!=h:raise RuntimeError("historical outcome changed")
    for p,h in freeze["source_sha256"].items():
        data=subprocess.check_output(["git","-C",str(ROOT),"show",freeze["code_sha"]+":"+p])
        if hashlib.sha256(data).hexdigest()!=h:raise RuntimeError("code provenance failed")
    if sha(freeze["binary_path"])!=freeze["binary_sha256"] or sha(ARCHIVE/"THEORY.md")!=freeze["theory_sha256"]:
        raise RuntimeError("binary/rules changed")
    lineage=json.loads((ROOT/"docs/p9_r4_heldout_visual_evidence/source_recovery/replay_input_hashes.json").read_text())
    params=Path(lineage["inputs"]["params_txt"]["path"])
    values=np.array([float(x) for x in params.read_text().split()])
    if len(values)!=27 or sha(params)!=freeze["input_sha256"][str(params)]:raise RuntimeError("parameter identity failed")
    extr=np.eye(4);extr[:3,3]=values[-7:-4];extr[:3,:3]=Rotation.from_quat(values[-4:]).as_matrix()
    jobs={"control":load(CONTROL)}
    for n in MODES:
        jobs[n]=load(directory/n);jobs[n]["weak"]=read(directory/n/"weak_refinement.csv")
        jobs[n]["directional"]=read(directory/n/"directional_covariance.csv")
    parity=[]
    for table in ("registration","trajectory"):
        for a,b in zip(jobs["control"][table],jobs[MODES[0]][table]):
            different=[k for k in a if k!="alignment_ms" and a[k]!=b[k]]
            parity.append(dict(transaction_id=a["transaction_id"],table=table,exact=int(not different),different_fields=";".join(different)))
    csv_write(directory/"nominal_state_parity.csv",parity)
    if len(parity)!=8254 or not all(r["exact"] for r in parity):raise RuntimeError("shadow nominal/state changed")
    guards=[];covs=[];causal=[];pairs=[]
    for n in MODES:
        job=jobs[n];first=None;previous=None
        for rows in job.values():
            if isinstance(rows,list) and len(rows)==4127 and "transaction_id" in rows[0]:
                if [x["transaction_id"] for x in rows]!=[str(i) for i in range(1,4128)]:raise RuntimeError("ordered denominator failed")
        for i,(r,c,f,t,nom) in enumerate(zip(job["weak"],job["directional"],job["frames"],job["trajectory"],job["registration"])):
            used=r["alternative_used"]=="1";valid=c["valid"]=="1";rec=r["recommended"]=="1"
            quiet=r["triggered"]!="1" or r["anchor_valid"]!="1"
            actual=matrix(c["actual_imu_pose"]);cov=vector(c["consumed_covariance"]).reshape(6,6)
            R0=np.diag([.04]*3+[.01]*3)
            ok=(int(r["jet_calls"])<=2 and int(r["value_calls"])<=3 and r["extra_align_calls"]=="0" and f["full_ndt_calls"]=="1" and
                (not quiet or r["jet_calls"]==r["value_calls"]=="0") and used==(n!=MODES[0] and rec and valid) and
                c["alternative_used"]==r["alternative_used"] and r["update_success"]==nom["effective"] and
                np.isfinite(actual).all() and np.isfinite(imu_pose(t,"corrected_imu_")).all() and
                (np.max(np.abs(cov-vector(c["covariance"]).reshape(6,6)))<1e-14 if used else np.max(np.abs(cov-R0))<1e-14))
            expected_lidar=matrix(r["candidate_pose"] if used else r["nominal_pose"])
            expected_lidar[:3,:3]=Rotation.from_quat(carrier_quaternion(expected_lidar[:3,:3])).as_matrix()
            ok=ok and np.max(np.abs(actual-expected_lidar@np.linalg.inv(extr)))<1e-8
            if valid:
                check=covariance_check(c,r,extr);covs.append(dict(mode=n,transaction_id=r["transaction_id"],**check))
                ok=ok and bool(check["covariance_pass"]) and rec
                if n==MODES[0]:ok=ok and c["weak_paired_valid"]=="1"
            if used:
                ok=ok and r["status"]=="LOCAL_REGULARIZED_REFINEMENT" and r["weak_quality_valid"]=="1"
                if first is None:first=i+1
            if rec:
                k=int(r["weak_dimension"]);W=vector(r["eigenvectors"]).reshape(6,6)[:,:k]
                eta=vector(r["candidate_eta"]);u=vector(r["u_anchor"]);rho=float(r["rho"])
                count=float(f["source_count"]);s0=float(r["nominal_score"]);s=float(r["candidate_score"])
                objective=-s/count+.5*rho*np.linalg.norm(W.T@eta-u)**2
                objective0=-s0/count+.5*rho*np.dot(u,u)
                ok=ok and objective<objective0-1e-8*max(1,abs(objective0)) and -s/count<=-s0/count+.05*max(1,abs(s0/count))
                dt,dr=distance(matrix(r["candidate_pose"]),matrix(r["nominal_pose"]))
                ok=ok and dt<=.15+1e-6 and dr<=2+1e-4
                if n==MODES[0]:
                    delta_t,delta_r=distance(matrix(r["weak_pose"]),matrix(r["candidate_pose"]))
                    pairs.append(dict(transaction_id=r["transaction_id"],strong_selected=int(r["strong_selected"]),
                        weak_score=float(r["weak_score"]),coupled_score=s,score_gain_per_point=(s-float(r["weak_score"]))/count,
                        pose_difference_m=delta_t,pose_difference_deg=delta_r,paired_covariance_valid=int(c["weak_paired_valid"])))
            if r["anchor_valid"]=="1":
                ok=ok and r["anchor_after_origin"]==r["anchor_origin"] and r["anchor_after_prediction"]==r["anchor_prediction"] and r["anchor_after_origin_stamp_ns"]==r["anchor_origin_stamp_ns"]
                if previous and previous["anchor_after_valid"]=="1" and previous["anchor_after_origin_stamp_ns"]==r["anchor_origin_stamp_ns"]:
                    prior=matrix(previous["anchor_prediction"] if previous["anchor_valid"]=="1" else r["anchor_origin"])
                    interval=np.linalg.inv(imu_pose(job["trajectory"][i-1],"corrected_imu_")@extr)@(imu_pose(t,"predicted_imu_")@extr)
                    ok=ok and np.max(np.abs(prior@interval-matrix(r["anchor_prediction"])))<1e-8
            if n==MODES[1]:ok=ok and int(r["jet_calls"])<=1 and r["strong_selected"]=="0"
            if nom["effective"]!="1":ok=ok and not used and all(t["corrected_imu_"+a]==t["predicted_imu_"+a] for a in ("x","y","z","qx","qy","qz","qw"))
            changed=any(t[a]!=jobs[MODES[0]]["trajectory"][i][a] for a in t if a.startswith("predicted_imu_"))
            source_changed=nom["source_cloud_hash"]!=jobs[MODES[0]]["registration"][i]["source_cloud_hash"]
            causal.append(dict(mode=n,transaction_id=r["transaction_id"],first_feedback_tx=first or "",prediction_differs=int(changed),source_differs=int(source_changed)))
            guards.append(dict(mode=n,transaction_id=r["transaction_id"],pass_guards=int(ok),used=int(used)))
            previous=r
        if n!=MODES[0] and first and first<4127 and not any(x["mode"]==n and int(x["transaction_id"])==first+1 and x["prediction_differs"] for x in causal):
            raise RuntimeError("feedback did not affect causal next prediction")
    csv_write(directory/"engineering_guards.csv",guards);csv_write(directory/"covariance_parity.csv",covs)
    csv_write(directory/"causal_feedback_parity.csv",causal);csv_write(directory/"paired_candidates.csv",pairs)
    if not all(r["pass_guards"] for r in guards):raise RuntimeError("engineering failed BEFORE GT")
    return freeze,jobs

def evaluate():
    directory=ARCHIVE/"attempt_0";freeze,jobs=verify(directory)
    stats={n:statistics(n,t,directory) for n,t in jobs.items()}
    for n in MODES:
        rows=jobs[n]["directional"];s=stats[n]
        s["covariance_attempts"]=sum(r["attempted"]=="1" for r in rows)
        s["covariance_valid"]=sum(r["valid"]=="1" for r in rows)
        s["covariance_statuses"]=dict(Counter(r["status"] for r in rows))
        s["multipliers"]=moments([x for r in rows if r["valid"]=="1" for x in vector(r["multipliers"])])
        s["covariance_ms_all_frames"]=moments([float(r["covariance_ms"]) for r in rows])
        s["covariance_ms_attempted"]=moments([float(r["covariance_ms"]) for r in rows if r["attempted"]=="1"])
        s["filter_correction_translation_m"]=moments([float(r["correction_t_m"]) for r in rows if r["alternative_used"]=="1"])
        s["filter_correction_rotation_deg"]=moments([float(r["correction_r_deg"]) for r in rows if r["alternative_used"]=="1"])
        s["abnormal_updates"]=sum(r["update_success"]!=nom["effective"] for r,nom in zip(rows,jobs[n]["registration"]))
        s["curvature_degenerate_weak_rows"]=sum(r["mode"]==n and float(r["weak_relative_gap"])<=1e-6 for r in read(directory/"covariance_parity.csv"))
        s["candidate_score_better"]=sum(float(r["candidate_score"])>float(r["nominal_score"]) for r in jobs[n]["weak"] if r["recommended"]=="1")
        s["candidate_score_worse"]=sum(float(r["candidate_score"])<float(r["nominal_score"]) for r in jobs[n]["weak"] if r["recommended"]=="1")
    json_write(directory/"pre_GT_audit_freeze.json",dict(GT_LOADED=False,engineering="PASS",evaluator_sha256=sha(Path(__file__)),
        audit_code_sha=subprocess.check_output(["git","-C",str(ROOT),"rev-parse","HEAD"],text=True).strip(),
        engineering_statistics=stats,output_sha256={p.name:sha(p) for p in directory.glob("*.csv")}))
    gt=posthoc(directory,jobs)  # FIRST GT access: runtime and all engineering outputs frozen.
    old=json.loads((R6/"evaluation.json").read_text())
    summaries=[]
    for n in ("control","R6_weak_only","R6_coupled",*MODES):
        historic=n.startswith("R6_")
        key={"R6_weak_only":"weak_only_feedback","R6_coupled":"coupled_weak_feedback"}.get(n,n)
        g=old["GT"][key] if historic else gt[key];s=old["statistics"][key] if historic else stats[key]
        summaries.append(dict(method=n,frames=4127,GT_frames=g["count"],feedback=s.get("alternative_used",0),
            translation_RMSE_m=g["corrected_translation_m"]["RMSE"],translation_P95_m=g["corrected_translation_m"]["P95"],translation_max_m=g["corrected_translation_m"]["max"],
            rotation_RMSE_deg=g["corrected_rotation_deg"]["RMSE"],rotation_P95_deg=g["corrected_rotation_deg"]["P95"],rotation_max_deg=g["corrected_rotation_deg"]["max"],
            large_jumps=s["large_jumps"],mean_ms=s["processing_ms"]["mean"],P95_ms=s["processing_ms"]["P95"],max_ms=s["processing_ms"]["max"],peak_rss_kib=s["peak_rss_kib"]))
    csv_write(ARCHIVE/"method_summary.csv",summaries)
    gain={n:1-gt[n]["corrected_translation_m"]["RMSE"]/gt["control"]["corrected_translation_m"]["RMSE"] for n in MODES[1:]}
    paired=read(directory/"paired_candidates.csv")
    result=dict(task=freeze["task"],start_sha=freeze["start_sha"],code_sha=freeze["code_sha"],engineering="PASS",statistics=stats,GT=gt,
        method_summary=summaries,gain_vs_control=gain,
        weak_drift_relief_vs_R6=1-gt[MODES[1]]["corrected_translation_m"]["RMSE"]/old["GT"]["weak_only_feedback"]["corrected_translation_m"]["RMSE"],
        coupled_vs_directional_weak=1-gt[MODES[2]]["corrected_translation_m"]["RMSE"]/gt[MODES[1]]["corrected_translation_m"]["RMSE"],
        paired_candidates=dict(total=len(paired),strong_changed=sum(r["strong_selected"]=="1" for r in paired),
            score_gain_per_point=moments([float(r["score_gain_per_point"]) for r in paired]),
            no_extra_paired_objective_calls=True),
        accuracy_goal_pass=gain[MODES[2]]>=.01,
        performance_goal_pass=all(stats[n]["processing_ms"]["mean"]<=100 and stats[n]["processing_ms"]["P95"]<=150 for n in MODES),
        full_ndt_calls=sum(stats[n]["full_ndt_calls"] for n in MODES),extra_full_ndt_calls=0,
        GT_FOR_ADMISSION=False,production_changed=False,independent_validation=False,targeted_runtime_repairs=0)
    json_write(directory/"evaluation.json",result)
    print(json.dumps(result,indent=2))

if __name__=="__main__":evaluate()
