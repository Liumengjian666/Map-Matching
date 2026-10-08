"""P4-equivalent tracking/PnP with offline depth completion and DIRECT replay."""
import hashlib
import time

import numpy as np

from p9_r3a_depth_completion import associate, p4
from p9_r3_visual_contract import require, text, imu_increment, separation


def tracking(ref, cur, calibration):
    """Literal P4 Shi-Tomasi/KLT/FB rules, shared by both depth modes."""
    tick = time.perf_counter()
    features = p4.cv2.goodFeaturesToTrack(ref, 500, .01, 10, mask=calibration["mask"])
    metrics = dict(detected=0 if features is None else len(features), klt_forward_valid=0,
        fb_valid=0, feature_ms=(time.perf_counter()-tick)*1000, klt_ms=0.)
    if features is None:
        return np.empty((0,2)), np.empty((0,2)), metrics
    tick = time.perf_counter()
    options = dict(winSize=(21,21), maxLevel=3,
        criteria=(p4.cv2.TERM_CRITERIA_EPS | p4.cv2.TERM_CRITERIA_COUNT,30,.01))
    forward, s1, _ = p4.cv2.calcOpticalFlowPyrLK(ref,cur,features,None,**options)
    backward, s2, _ = p4.cv2.calcOpticalFlowPyrLK(cur,ref,forward,None,**options)
    a, b = features.reshape(-1,2), forward.reshape(-1,2)
    w, h = calibration["size"]
    valid = (s1.ravel()!=0) & (s2.ravel()!=0) & (np.linalg.norm(backward.reshape(-1,2)-a,axis=1)<=1.)
    valid &= np.isfinite(b).all(axis=1) & (b[:,0]>=0) & (b[:,0]<w) & (b[:,1]>=0) & (b[:,1]<h)
    metrics.update(klt_forward_valid=int(np.count_nonzero(s1)), fb_valid=int(valid.sum()),
        klt_ms=(time.perf_counter()-tick)*1000)
    return a[valid], b[valid], metrics


def solve(points, pixels, calibration, seed):
    row = dict(pnp_correspondences=len(points), pnp_inliers=0, inlier_ratio="",
        reprojection_rmse_px="", pnp_ms=0., status="INSUFFICIENT_CORRESPONDENCES",
        T_Ccur_Cref="", D_vis="")
    if len(points)<30:
        return row
    tick = time.perf_counter()
    pose, count, rmse, _ = p4.pnp(points,pixels.astype(float),calibration["Krect"],seed)
    row.update(pnp_inliers=count,inlier_ratio=count/len(points),
        reprojection_rmse_px=rmse if np.isfinite(rmse) else "",pnp_ms=(time.perf_counter()-tick)*1000,
        status="VALID" if pose is not None else "PNP_REJECT")
    if pose is not None:
        row.update(T_Ccur_Cref=text(pose),D_vis=text(imu_increment(pose,calibration["T_imu_camera"])))
    return row


def array_hash(array):
    return hashlib.sha256(np.asarray(array,dtype="<f8").tobytes()).hexdigest()


def estimate_pair(ref, cur, cloud, calibration, seed, replay=True):
    tick = time.perf_counter()
    a, b, metrics = tracking(ref,cur,calibration)
    direct, direct_points, good, points, records, timing = associate(cloud,a,calibration)
    shared_ms = (time.perf_counter()-tick)*1000
    old = solve(direct_points,b[direct],calibration,seed) if replay else None
    new = solve(points,b[good],calibration,seed)
    if not metrics["detected"]:
        new["status"]="NO_FEATURES"
        if old is not None:old["status"]="NO_FEATURES"
    completed = int(good.sum()-direct.sum())
    new.update(metrics,**timing,direct_depth_count=int(direct.sum()),plane_completed_count=completed,
        total_depth_count=int(good.sum()),depth_associated=int(good.sum()),
        frontend_ms=shared_ms+new["pnp_ms"])
    for i, record in enumerate(records):
        record.update(cur_u=float(b[i,0]),cur_v=float(b[i,1]),
            point_xyz="" if not good[i] else text(points[np.searchsorted(np.flatnonzero(good),i)]))
    parity = None
    if replay:
        old_good, old_points, _, _ = p4.associate_depth(cloud,a,calibration)
        require(np.array_equal(old_good,direct) and np.array_equal(old_points,direct_points),
            "DIRECT point/feature parity failed")
        legacy, pose = p4.estimate(ref,cur,cloud,calibration,seed)
        for field in ("status","detected","klt_forward_valid","fb_valid","pnp_correspondences","pnp_inliers"):
            require(legacy[field]==(metrics[field] if field in metrics else old[field]),"DIRECT replay field mismatch: "+field)
        dt=dr=0.
        if pose is not None:
            measured=np.array([float(x) for x in old["T_Ccur_Cref"].split(";")]).reshape(4,4)
            dt,dr=separation(pose,measured)
            require(dt<=1e-10 and dr<=1e-8,"DIRECT PnP pose replay mismatch")
            require(abs(float(old["reprojection_rmse_px"])-legacy["reprojection_rmse_px"])<=1e-10,
                "DIRECT PnP reprojection replay mismatch")
        parity=dict(count=int(direct.sum()),historical_3d_sha256=array_hash(old_points),
            direct_3d_sha256=array_hash(direct_points),pose_translation_error_m=dt,
            pose_rotation_error_deg=dr,pass_flag=1)
        old.update(metrics,direct_depth_count=int(direct.sum()),plane_completed_count=0,
            total_depth_count=int(direct.sum()),depth_associated=int(direct.sum()))
    return old,new,records,parity


def self_test():
    from types import SimpleNamespace
    w,h=240,180
    yy,xx=np.indices((h,w));gray=(((xx//10+yy//10)%2)*255).astype(np.uint8)
    k=np.array([[150.,0,120],[0,150.,90],[0,0,1.]])
    calibration=dict(Krect=k,T_camera_lidar=np.eye(4),T_imu_camera=np.eye(4),
        size=(w,h),mask=np.full((h,w),255,dtype=np.uint8))
    pixels=np.array([[x,y] for y in range(9,h-1,10) for x in range(9,w-1,10)],dtype=float)
    xyz=(np.c_[pixels,np.ones(len(pixels))]@np.linalg.inv(k).T*5).astype("<f4")
    cloud=SimpleNamespace(fields=[SimpleNamespace(name=n,datatype=7,count=1,offset=i*4) for i,n in enumerate("xyz")],
        is_bigendian=False,point_step=12,height=1,width=len(xyz),row_step=12*len(xyz),data=xyz.tobytes())
    old,new,records,parity=estimate_pair(gray,gray,cloud,calibration,1)
    require(old["status"]==new["status"]=="VALID" and parity["pass_flag"]==1,"synthetic DIRECT PnP parity")
    require(new["direct_depth_count"]>=30 and len(records)==new["fb_valid"],"synthetic tracking/provenance")
    pose=np.array(list(map(float,new["D_vis"].split(";")))).reshape(4,4)
    require(np.max(abs(pose-np.eye(4)))<1e-5,"synthetic identity visual increment")
    print("P9_R3A_TRACKING_DIRECT_PNP_REPLAY_SELF_TEST=PASS")


if __name__=="__main__":
    p4.cv2.setNumThreads(1)
    self_test()
