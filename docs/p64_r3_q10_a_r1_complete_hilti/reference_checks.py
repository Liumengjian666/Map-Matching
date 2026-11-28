"""World-frame-invariant reference checks, kept separate from fixed-world APE."""
import argparse
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"p64_r3_q10_public_mechanism_validation"))
from evaluate_exp18 import read_reference, write


def interpolate(ts, states, query, max_gap=150_000_000):
    right = np.searchsorted(ts,query)
    if right < len(ts) and ts[right] == query:
        return states[right]
    if not 0<right<len(ts) or ts[right]-ts[right-1]>max_gap:
        return None
    w = (query-ts[right-1])/(ts[right]-ts[right-1])
    result = np.eye(4)
    result[:3,3]=(1-w)*states[right-1,:3,3]+w*states[right,:3,3]
    result[:3,:3]=Slerp([0,1],Rotation.from_matrix(states[right-1:right+1,:3,:3]))(w).as_matrix()
    return result


def poses(values):
    states=np.repeat(np.eye(4)[None],len(values),axis=0)
    states[:,:3,3]=values[:,:3]
    states[:,:3,:3]=Rotation.from_quat(values[:,3:]).as_matrix()
    return states


def report(dense, sparse, estimate, output):
    td,rd=read_reference(dense)
    ts,rs=read_reference(sparse)
    te,re=read_reference(estimate)
    dr,er=poses(rd),poses(re)
    target=[interpolate(td,dr,i) for i in te]
    tip=np.array([.059,-.00855,.1964])
    rs_est=[interpolate(td,dr,i) for i in ts]
    surveys=[]
    for i,left in enumerate(rs_est):
        if left is None:
            continue
        for j in range(i+1,len(rs_est)):
            right=rs_est[j]
            if right is None:
                continue
            att=right[:3,:3]-left[:3,:3]
            delta=right[:3,3]-left[:3,3]
            dist=np.linalg.norm(rs[j,:3]-rs[i,:3])
            d=np.linalg.norm(delta+att@tip)
            surveys.append(dict(left_ns=ts[i],right_ns=ts[j],survey_pair_m=dist,dense_tip_pair_m=d,dense_imu_pair_m=np.linalg.norm(delta),distance_difference_m=d-dist))
    write(output/"SPARSE_REFERENCE_DISTANCE_AUDIT.csv",surveys)
    metrics=[]
    for lag in (1,5,10,50):
        for left in range(len(te)-lag):
            right=left+lag
            if target[left] is None or target[right] is None:
                continue
            elapsed_ns=te[right]-te[left]
            if not abs(elapsed_ns-lag*100_000_000)<50_000_000:
                continue
            reference_delta=np.linalg.solve(target[left],target[right])
            est_delta=np.linalg.solve(er[left],er[right])
            e=np.linalg.solve(reference_delta,est_delta)
            metrics.append(dict(lag_frames=lag,start_ns=te[left],end_ns=te[right],
                trans_m=np.linalg.norm(e[:3,3]),rotation_deg=np.degrees(Rotation.from_matrix(e[:3,:3]).magnitude())))
    write(output/"RELATIVE_REFERENCE_DIAGNOSTICS.csv",metrics)


if __name__=="__main__":
    p=argparse.ArgumentParser()
    for n in ("dense","sparse","estimate","output"):
        p.add_argument(n,type=Path)
    a=p.parse_args()
    report(a.dense,a.sparse,a.estimate,a.output)
