#!/usr/bin/env python3
"""Map-independent visual motion contract. No label/GT reader or optimizer."""
import hashlib
from pathlib import Path
import subprocess
import sys

import numpy as np
from scipy.spatial.transform import Rotation

from p9_r2b_nonoracle_evidence import require,read_csv,write_csv,text,vector

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
PACKAGE=ROOT/"src/dog_prior_map_fastlio2_frontend_exp"
sys.path.insert(0,str(PACKAGE/"scripts"))
import p4_i3_visual_frontend as frontend

OUT=ROOT/"docs/p9_r3_visual_nonlocal_evidence"
START_SHA="4957015ce21c5c9071f17746c172f6fe13a9bde6"
BRANCH="research/p9-r3-visual-nonlocal-evidence"
LAGS=(1,2,4,8)
DATA=Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01")
REGISTRATION=DATA/"results/dual_u_r1_closure_20261003/same_objective/registration.csv"
P4=PACKAGE/"docs/p4_i3_visual_increment"
R2B=ROOT/"docs/p9_r2b_nonoracle_evidence"
R2A=ROOT/"docs/p9_r2a_predictor_conditioned_search"


def digest(path):
    h=hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda:stream.read(8*1024*1024),b""):h.update(block)
    return h.hexdigest()


def committed_hash(path):
    expected=subprocess.check_output(["git","show",START_SHA+":"+path.relative_to(ROOT).as_posix()],cwd=ROOT)
    value=hashlib.sha256(expected).hexdigest()
    require(digest(path)==value,"committed input changed: "+str(path))
    return value


def pose_xyzq(values):
    values=np.asarray(values,dtype=float);require(values.shape==(7,) and np.isfinite(values).all(),"invalid pose xyzq")
    m=np.eye(4);m[:3,3]=values[:3];m[:3,:3]=Rotation.from_quat(values[3:]).as_matrix()
    return m


def rigid_matrix(value):
    m=vector(value,16).reshape(4,4)
    require(np.max(abs(m[3]-[0,0,0,1]))<1e-9 and
            np.max(abs(m[:3,:3].T@m[:3,:3]-np.eye(3)))<1e-5 and
            np.linalg.det(m[:3,:3])>0,"invalid matrix pose carrier")
    m=m.copy();m[:3,:3]=Rotation.from_matrix(m[:3,:3]).as_matrix()
    return m


def separation(first,second):
    return float(np.linalg.norm(first[:3,3]-second[:3,3])),float(np.degrees(
        (Rotation.from_matrix(first[:3,:3]).inv()*Rotation.from_matrix(second[:3,:3])).magnitude()))


def imu_increment(pnp_transform,t_ic):
    return t_ic@np.linalg.inv(pnp_transform)@np.linalg.inv(t_ic)


def residual(reference_imu,candidate_imu,visual_increment):
    error=np.linalg.inv(visual_increment)@np.linalg.inv(reference_imu)@candidate_imu
    return float(np.linalg.norm(error[:3,3])),float(np.degrees(Rotation.from_matrix(error[:3,:3]).magnitude()))


def choose_smallest_valid(rows):
    require(sorted(int(row["lag"]) for row in rows)==list(LAGS),"incomplete fixed four-lag measurement set")
    valid=[row for row in rows if row["status"]=="VALID"]
    return min(valid,key=lambda row:int(row["lag"])) if valid else None


def strict_separation(first,second):
    dt,dr=separation(first,second)
    return dt,dr,dt>.2 or dr>2.


def evidence_scalars(available,nominal_residual,eligible_residuals):
    """Missing is unknown; an available measurement with no alternative is zero."""
    if not available:
        return dict(r_nom="",r_best_alt="",G_visual="",U_visual="")
    require(np.isfinite(nominal_residual),"invalid available nominal residual")
    if not eligible_residuals:
        return dict(r_nom=float(nominal_residual),r_best_alt="",G_visual=0.,U_visual=0.)
    require(np.isfinite(eligible_residuals).all(),"invalid competing residual")
    alternative=float(min(eligible_residuals));gap=float(nominal_residual-alternative)
    return dict(r_nom=float(nominal_residual),r_best_alt=alternative,G_visual=gap,U_visual=max(0.,gap))


def self_test():
    calibration=frontend.load_calibration(DATA/"calibration")
    frontend.cv2.setNumThreads(1);frontend.sanity(calibration)
    reference=pose_xyzq([1,-2,.4,.1,.2,-.1,.96])
    increment=pose_xyzq([.24,-.02,.03,-.02,.03,.01,.99]);current=reference@increment
    t_ic=calibration["T_imu_camera"];t_il=calibration["T_imu_lidar"]
    pnp=np.linalg.inv(current@t_ic)@(reference@t_ic)
    measured=imu_increment(pnp,t_ic)
    require(np.max(abs(measured-increment))<1e-12,"PnP IMU increment direction wrong")
    ref_lidar=reference@t_il;cur_lidar=current@t_il
    rt,rr=residual(ref_lidar@np.linalg.inv(t_il),cur_lidar@np.linalg.inv(t_il),measured)
    require(rt<1e-12 and rr<1e-10,"LiDAR/IMU visual residual frame mismatch")
    displaced=current.copy();displaced[:3,3]+=[.1,0,0]
    rt,_=residual(reference,displaced,measured);require(abs(rt-.1)<1e-12,"translation residual unit mismatch")
    rows=[dict(lag=lag,status="VALID" if lag in (2,8) else "INVALID",residual=99 if lag==2 else 0) for lag in LAGS]
    require(choose_smallest_valid(rows)["lag"]==2,"pair selection used residual instead of availability")
    require(choose_smallest_valid([dict(lag=lag,status="INVALID") for lag in LAGS]) is None,
            "all-invalid visual was not missing")
    require(all(value=="" for value in evidence_scalars(False,None,[]).values()),
            "missing visual manufactured zero evidence")
    empty=evidence_scalars(True,.2,[])
    require(empty["U_visual"]==0. and empty["r_best_alt"]=="" and empty["r_nom"]==.2,
            "available empty competing set mishandled")
    require(evidence_scalars(True,.2,[.3])["U_visual"]==0. and
            abs(evidence_scalars(True,.2,[.3,.1])["U_visual"]-.1)<1e-12,
            "translation evidence sign/minimum convention wrong")
    inside=reference.copy();inside[:3,3]+=[.1,0,0]
    require(not strict_separation(reference,inside)[2],"inside-center competitor admitted")
    outside=reference.copy();outside[:3,:3]=Rotation.from_rotvec([.05,0,0]).as_matrix()@reference[:3,:3]
    require(strict_separation(reference,outside)[2],"rotation-only separated competitor excluded")
    print("P9_R3_PNP_DIRECTION_MEI_TRANSFORM_SELECTION_SELF_TEST=PASS")


if __name__=="__main__":self_test()
