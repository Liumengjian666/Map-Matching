"""Post-rejection attribution only. NEVER produces an accepted moving state."""
import csv
import json
import pathlib
import shutil
import sys
import time
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from preintegration import integrate, log, tangent, motion_system, profile_diagnostics
from prepare_bootstrap import ARCHIVE, INPUT, OUTPUT, sha, write_json, write_csv, verify_input

def stage_imu(rows, horizon):
    selected=[r for r in rows if int(r["stamp_ns"])<=horizon]
    return (np.array([int(r["stamp_ns"]) for r in selected], dtype=np.int64),
            np.array([[float(r[k]) for k in ["ax","ay","az","gx","gy","gz"]] for r in selected]))

def fit(poses, stamps, imu_stamps, imu, pose_sigma, rotation_sigma, cfg):
    def integrals(bg):
        return [integrate(imu_stamps, imu, int(a), int(b), bg,
                          cfg["maximum_imu_gap_s"], cfg["maximum_endpoint_hold_s"])
                for a,b in zip(stamps[:-1],stamps[1:])]
    def gyro_residual(bg):
        return np.concatenate([log(d[0].T @ (a[:3,:3].T @ b[:3,:3]))
                               for a,b,d in zip(poses[:-1],poses[1:],integrals(bg))])
    gyro=least_squares(gyro_residual, np.zeros(3), xtol=1e-10, ftol=1e-10, gtol=1e-10, max_nfev=100)
    terms=integrals(gyro.x); dt=np.diff(stamps)*1e-9
    n=len(poses); sigma=[]
    # Conservative marginal uncertainty, including lever-arm translation and
    # reference rotation acting on preintegrated force. Across-block correlation
    # is bounded by Nblocks*diag(Ci), not treated as independent observations.
    for i,(_,dv,dp,_,_) in enumerate(terms):
        sp=np.sqrt(pose_sigma[i]**2+pose_sigma[i+1]**2+(rotation_sigma[i]*np.linalg.norm(dp))**2)
        sv=max(cfg["imu_accel_noise"]*dt[i], rotation_sigma[i]*np.linalg.norm(dv), 1e-6)
        sigma.append([sp,sv])
    sigma=np.array(sigma)*np.sqrt(2*(n-1))
    A,y,rawA,rawy=motion_system(poses,stamps,terms,sigma)
    x=np.linalg.lstsq(A,y,rcond=cfg["svd_relative_rank_tolerance"])[0]
    g=x[3*n:3*n+3]; norm=np.linalg.norm(g)
    if norm<1e-8: raise RuntimeError("gravity tentative solution has zero norm")
    g=g/norm*cfg["gravity_norm_m_s2"]; x[3*n:3*n+3]=g
    for _ in range(10):
        basis=tangent(g)
        J=np.column_stack((A[:,:3*n], A[:,3*n:3*n+3]@basis, A[:,3*n+3:]))
        delta=np.linalg.lstsq(J,y-A@x,rcond=cfg["svd_relative_rank_tolerance"])[0]
        x[:3*n]+=delta[:3*n]; x[3*n+3:]+=delta[3*n+2:]
        g+=basis@delta[3*n:3*n+2]; g=g/np.linalg.norm(g)*cfg["gravity_norm_m_s2"]
        x[3*n:3*n+3]=g
        if np.linalg.norm(delta)<1e-9:break
    scales=[cfg["gravity_norm_m_s2"]*np.deg2rad(cfg["gravity_direction_std_max_deg"])]*2+[cfg["accel_bias_std_max_m_s2"]]*3
    d=profile_diagnostics(A,n,g,scales,np.maximum(rotation_sigma[:-1],rotation_sigma[1:]),dt,sigma,
                          cfg["svd_relative_rank_tolerance"])
    residual=(rawA@x-rawy).reshape(-1,6)
    d.update({"gyro_bias_tentative":gyro.x.tolist(),"accel_bias_tentative":x[3*n+3:].tolist(),
              "gravity_tentative":g.tolist(),"last_velocity_tentative":x[3*n-3:3*n].tolist(),
              "gyro_residual_rmse_deg":float(np.rad2deg(np.sqrt(np.mean(gyro.fun.reshape(-1,3)**2)*3))),
              "position_residual_rmse_m":float(np.sqrt(np.mean(np.sum(residual[:,:3]**2,axis=1)))),
              "velocity_residual_rmse_m_s":float(np.sqrt(np.mean(np.sum(residual[:,3:]**2,axis=1)))),
              "fit_start_ns":int(stamps[0]),"fit_end_ns":int(stamps[-1]),
              "IMU_max_loaded_stamp_ns":int(imu_stamps[-1]),"solver_success":bool(gyro.success),
              "state_accepted":False,"covariance":"NOT_ACCEPTED; full coupled covariance NOT_RUN after upstream quality rejection"})
    pre=[]
    for i,(DR,dv,dp,_,_) in enumerate(terms):
        row={"start_ns":int(stamps[i]),"end_ns":int(stamps[i+1]),"dt_s":float(dt[i]),
             "endpoint_hold_ns":max(0,int(stamps[i+1]-imu_stamps[-1])),"role":"DIAGNOSTIC_FIT_ONLY"}
        row.update({"dR"+str(k):float(v) for k,v in enumerate(log(DR))})
        row.update({"dv"+str(k):float(v) for k,v in enumerate(dv)})
        row.update({"dp"+str(k):float(v) for k,v in enumerate(dp)})
        pre.append(row)
    return d,x,pre

def diagnose():
    started=time.monotonic()
    freeze=json.loads((ARCHIVE/"bootstrap_freeze.json").read_text())
    if sha(ARCHIVE/"bootstrap_config.json")!=freeze["config_sha256"]:
        raise RuntimeError("bootstrap gates changed after freeze")
    manifest=json.loads((INPUT/"input_manifest.json").read_text())
    verify_input(manifest,freeze["input_manifest_sha256"])
    if any(manifest["input_files"][name]!=receipt for name,receipt in freeze["input_hashes"].items()
           if name in manifest["input_files"]):
        raise RuntimeError("input hash receipts changed")
    sources={str(p):sha(p) for p in pathlib.Path(__file__).parent.glob("*.py")}
    sources[str(pathlib.Path(__file__).parent/"bootstrap_lidar.cpp")]=sha(pathlib.Path(__file__).parent/"bootstrap_lidar.cpp")
    write_json(ARCHIVE/"diagnostic_provenance.json",{
        "mode":"DIAGNOSTIC_ONLY; never eligible for state injection",
        "gate_freeze_sha256":sha(ARCHIVE/"bootstrap_freeze.json"),
        "input_hash_recheck":"PASS", "sources":sources,
        "diagnostic_executable_sha256":sha(freeze["binary"]),
        "diagnostic_binary_receipt_timing":"POST_RUN before subsequent build; original freeze binds first rejected execution",
        "raw_odometry_sha256":sha(OUTPUT/"diagnostic_lidar_odometry.csv"),
        "first_rejected_odometry_sha256":sha(OUTPUT/"bootstrap_lidar_odometry.csv"),
        "command":"bootstrap_lidar INPUT diagnostic_lidar_odometry.csv --diagnostic-only",
        "GT_LOADED":False,"NDT_CALLS":0})
    rows=list(csv.DictReader((OUTPUT/"diagnostic_lidar_odometry.csv").open()))
    if len(rows)!=freeze["bootstrap_scan_count"] or any(r["solver_converged"]!="1" for r in rows):
        raise RuntimeError("incomplete/unconverged diagnostic odometry; no interpolation permitted")
    if max(int(r["stamp_ns"]) for r in rows)>freeze["boot_stamp_ns"]:
        raise RuntimeError("future scan in bootstrap")
    poses=np.array([[float(r["T"+str(i)+str(j)]) for i in range(4) for j in range(4)] for r in rows]).reshape(-1,4,4)
    # Correct small float-carrier orthogonality error only; no pose/trajectory fit.
    for T in poses:T[:3,:3]=Rotation.from_matrix(T[:3,:3]).as_matrix()
    ext=np.array(freeze["T_imu_lidar"]); imu_poses=poses@np.linalg.inv(ext)
    stamps=np.array([int(r["stamp_ns"]) for r in rows],dtype=np.int64)
    all_imu=list(csv.DictReader((INPUT/"imu.csv").open()))
    cfg=json.loads((ARCHIVE/"bootstrap_config.json").read_text())
    c=cfg["preintegration"]; lc=cfg["lidar"]
    train=(stamps>=freeze["estimation_start_ns"])&(stamps<freeze["estimation_end_ns"])
    valid=(stamps>=freeze["estimation_end_ns"])&(stamps<=freeze["boot_stamp_ns"])
    ts,measurements=stage_imu(all_imu,freeze["estimation_end_ns"])
    rotation_sigma=[]; position_sigma=[]
    for i,r in enumerate(rows):
        duration=(int(r["stamp_ns"])-int(r["scan_start_ns"]))*1e-9
        local=[s for s in all_imu if int(r["scan_start_ns"])<=int(s["stamp_ns"])<=int(r["stamp_ns"])]
        speed=max(np.linalg.norm([float(s[k]) for k in ["gx","gy","gz"]]) for s in local)
        sr=max(np.deg2rad(lc["pose_rotation_sigma_deg"]),(speed+c["gyro_bias_std_max_rad_s"])*duration)
        pair_dt=(stamps[i]-stamps[i-1])*1e-9 if i else duration
        sp=max(lc["pose_position_sigma_m"],float(r["pair_translation_m"])*duration/pair_dt)
        # T_map_imu translation carries uncertain R*lever_arm, including its
        # contribution to marginal position sigma. No direct covariance copying.
        position_sigma.append(np.hypot(sp,np.linalg.norm(ext[:3,3])*sr)); rotation_sigma.append(sr)
    sr=np.array(rotation_sigma); sp=np.array(position_sigma)
    d,x,pre=fit(imu_poses[train],stamps[train],ts,measurements,sp[train],sr[train],c)
    splits=[]
    for lower,upper in [(5.,6.5),(6.5,8.)]:
        mask=(stamps>=freeze["first_sensor_stamp_ns"]+int(lower*1e9))&(stamps<freeze["first_sensor_stamp_ns"]+int(upper*1e9))
        # Causal split: do not load even the other split's later IMU.
        split_ts,split_imu=stage_imu(all_imu,freeze["first_sensor_stamp_ns"]+int(upper*1e9))
        sd,_,_=fit(imu_poses[mask],stamps[mask],split_ts,split_imu,sp[mask],sr[mask],c)
        splits.append(sd)
    val_ts,val_imu=stage_imu(all_imu,freeze["boot_stamp_ns"])
    initial=imu_poses[train][-1]; initial_time=int(stamps[train][-1]); vi=x[3*np.count_nonzero(train)-3:3*np.count_nonzero(train)]
    g=np.array(d["gravity_tentative"]); ba=np.array(d["accel_bias_tentative"]); bg=np.array(d["gyro_bias_tentative"])
    validation=[]
    for T,t in zip(imu_poses[valid],stamps[valid]):
        DR,dv,dp,Jv,Jp=integrate(val_ts,val_imu,initial_time,int(t),bg,c["maximum_imu_gap_s"],c["maximum_endpoint_hold_s"])
        dt=(t-initial_time)*1e-9
        predicted=initial[:3,3]+vi*dt+.5*g*dt*dt+initial[:3,:3]@(dp-Jp@ba)
        validation.append({"stamp_ns":int(t),"fit_end_ns":initial_time,"translation_error_m":float(np.linalg.norm(T[:3,3]-predicted)),
                           "rotation_error_deg":float(np.rad2deg(np.linalg.norm(log((initial[:3,:3]@DR).T@T[:3,:3])))),
                           "status":"DIAGNOSTIC_ONLY_UNTRUSTED_RAW_SCAN_POSES","state_accepted":False})
    d["split_diagnostics"]=splits
    d["split_bg_change_rad_s"]=float(np.linalg.norm(np.array(splits[0]["gyro_bias_tentative"])-splits[1]["gyro_bias_tentative"]))
    d["split_ba_change_m_s2"]=float(np.linalg.norm(np.array(splits[0]["accel_bias_tentative"])-splits[1]["accel_bias_tentative"]))
    d["split_gravity_angle_deg"]=float(np.rad2deg(np.arccos(np.clip(np.dot(splits[0]["gravity_tentative"],splits[1]["gravity_tentative"])/c["gravity_norm_m_s2"]**2,-1,1))))
    d["validation_translation_rmse_m"]=float(np.sqrt(np.mean([v["translation_error_m"]**2 for v in validation])))
    d["validation_rotation_rmse_deg"]=float(np.sqrt(np.mean([v["rotation_error_deg"]**2 for v in validation])))
    d["raw_odometry_pairs"]=len(rows)-1
    d["raw_odometry_quality_pass_pairs"]=sum(r["converged"]=="1" for r in rows[1:])
    d["raw_odometry_solver_converged_pairs"]=sum(r["solver_converged"]=="1" for r in rows[1:])
    d["first_quality_failure_tx"]=next(int(r["transaction_id"]) for r in rows if r["converged"]!="1")
    d["formal_initialization_status"]="REJECTED_AT_BOOTSTRAP_REGISTRATION_QUALITY_GATE"
    d["offline_diagnostics_wall_s"]=time.monotonic()-started
    d["new_NDT_calls"]=0; d["GT_LOADED"]=False
    write_json(ARCHIVE/"motion_diagnostics.json",d)
    write_json(ARCHIVE/"initial_state_estimate.json",{"status":"NOT_ACCEPTED", "stamp_ns":freeze["boot_stamp_ns"],
               "reason":"bootstrap registration quality failed; diagnostic fits do not constitute an accepted initial state",
               "velocity":None,"bg":None,"ba":None,"gravity":None,"covariance":"NOT_RUN"})
    write_csv(ARCHIVE/"bootstrap_validation.csv",validation)
    write_csv(ARCHIVE/"imu_preintegration.csv",pre)
    write_csv(ARCHIVE/"observability_diagnostics.csv",[{
        "profile":"gravity_tangent_and_accel_bias", "rank":d["profile_rank"],"dimension":5,
        "condition":d["condition"],"sigma_min":d["singular_values"][-1],
        "uncertainty_bound":d["perturbation_frobenius_bound"],"sigma_min_lower_bound":d["sigma_min_lower_bound"],
        "acceptance":"FAIL", "interpretation":d["rank_interpretation"]}])
    shutil.copyfile(OUTPUT/"diagnostic_lidar_odometry.csv",ARCHIVE/"bootstrap_lidar_odometry.csv")
    shutil.copyfile(OUTPUT/"bootstrap_lidar_odometry.csv",ARCHIVE/"first_rejected_odometry.csv")
    print(json.dumps(d,allow_nan=False))

if __name__=="__main__":diagnose()
