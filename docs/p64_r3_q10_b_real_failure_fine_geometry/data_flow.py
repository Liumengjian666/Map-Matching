"""Join an audited baseline with the actual IMU stream and offline IMU reference."""
import csv
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

prior=Path(__file__).resolve().parents[1]/"p64_r3_q10_a_r1_complete_hilti"
sys.path.insert(0,str(prior))
from reference_checks import interpolate, poses, read_reference


def mat(r,k):
    return np.array([[float(r[k+str(i)+str(j)]) for j in range(4)] for i in range(4)])


def vec(x):
    return ";".join(f"{v:.12g}" for v in np.asarray(x).ravel())


def deg(a,b):
    return float(np.rad2deg(Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).magnitude()))


def pose_from_state(s):
    m=np.eye(4)
    m[:3,3]=s[:3]
    m[:3,:3]=Rotation.from_quat(s[[7,8,9,6]]).as_matrix()
    return m


def rows(p):
    return list(csv.DictReader(p.open()))


def write(path,seq):
    with path.open("w") as f:
        w=csv.DictWriter(f,list(dict.fromkeys(k for x in seq for k in x)),lineterminator="\n")
        w.writeheader()
        w.writerows(seq)


def audit(trace, baseline, input_, ref, out):
    native=rows(trace/"trajectory.csv")
    control=rows(baseline/"trajectory.csv")
    states=rows(trace/"states.csv")
    diag=rows(trace/"diagnostics.csv")
    gt,ss=read_reference(ref)
    gm=poses(ss)
    il=np.array(json.loads((input_/"input_summary.json").read_text())["T_I_L"])
    imu=[]
    with (input_/"imu.tsv").open() as f:
        for row in f:
            tick,*vals=row.split()
            imu.append((int(tick),list(map(float,vals))))
    ticks=np.array([x[0] for x in imu],dtype=np.int64)
    signal=np.array([x[1] for x in imu])
    receipt=[]
    last=0
    for i,(a,b,s,d) in enumerate(zip(native,control,states,diag)):
        ns=int(a["stamp_ns"])
        assert ns==int(b["stamp_ns"])==int(s["stamp_ns"])==int(d["stamp_ns"])
        assert all(np.array_equal(mat(a,k),mat(b,k)) for k in ("pred","ndt","state"))
        p,n,q=(mat(a,k) for k in ("pred","ndt","state"))
        rp=pose_from_state(np.fromstring(s["prev"],sep=";"))
        t0,t1=np.searchsorted(ticks,[last,ns],side="right")
        part=signal[t0:t1]
        assert len(part)==int(a["imu_count"])
        actual_dt=(ticks[t1-1]-(ticks[t0-1] if t0 else ticks[0]))*1e-9
        g=interpolate(gt,gm,ns)
        g=None if g is None else g@il
        r=dict(frame=i,stamp_ns=ns,scanning_elapsed_s=(ns-int(native[0]["stamp_ns"]))*1e-9,
          t_prev=vec(rp),t_pred=vec(p),t_ndt=vec(n),t_post=vec(q),t_reference=vec(g) if g is not None else "",
          prediction_rot_deg=deg(rp,p),ndt_correction_rot_deg=deg(p,n),ukf_post_vs_ndt_rot_deg=deg(n,q),
          ukf_post_vs_predict_rot_deg=deg(p,q),ndt_translation_correction_m=np.linalg.norm(p[:3,3]-n[:3,3]),
          ukf_post_vs_ndt_translation_m=np.linalg.norm(n[:3,3]-q[:3,3]),imu_dt_s=actual_dt,imu_count=len(part),
          imu_acc_mean_l=vec(part[:,:3].mean(0)),imu_gyro_mean_l=vec(part[:,3:].mean(0)),
          imu_gyro_peak_rad_s=np.linalg.norm(part[:,3:],axis=1).max(),
          reference_rot_pred_deg=deg(g,p) if g is not None else "",reference_rot_ndt_deg=deg(g,n) if g is not None else "",
          reference_rot_post_deg=deg(g,q) if g is not None else "",
          reference_implied_specific_force_world=vec(g[:3,:3]@part[:,:3].mean(0)) if g is not None else "",
          velocity_pre=vec(np.fromstring(s["pred"],sep=";")[3:6]),
          velocity_post=vec(np.fromstring(s["post"],sep=";")[3:6]),
          bias_acc_pre=vec(np.fromstring(s["pred"],sep=";")[10:13]),
          bias_gyro_pre=vec(np.fromstring(s["pred"],sep=";")[13:16]),
          bias_acc_post=vec(np.fromstring(s["post"],sep=";")[10:13]),
          bias_gyro_post=vec(np.fromstring(s["post"],sep=";")[13:16]),
          source=a["source"],associated=a["supported"],iterations=a["iterations"],native_reward=a["native_reward"],
          native_hessian=d["native_hessian"],ndt_psd_proxy=d["proxy_matrix"],ndt_psd_eigenvalues=d["spectrum"],
          covariance_pred=d["covariance_before"],covariance_post=d["covariance_after"],weak_flag=d["weak"])
        receipt.append(r)
        last=ns
    write(out,receipt)


if __name__=="__main__":
    audit(*(Path(x) for x in sys.argv[1:]))
