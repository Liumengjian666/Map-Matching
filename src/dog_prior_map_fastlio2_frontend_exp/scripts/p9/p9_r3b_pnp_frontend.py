"""Offline PnP admission=20; the original P4 solver/quality gate is unchanged."""
import time
import numpy as np

from p9_r3a_visual_frontend import tracking, p4
from p9_r3a_depth_completion import associate
from p9_r3_visual_contract import require, text, imu_increment

ATTEMPT_MIN=20
INLIER_MIN=20  # immutable P4 pnp() acceptance, not a second tuned gate


def solve(points,reference_pixels,current_pixels,calibration,seed):
    require(points.shape==(len(current_pixels),3) and reference_pixels.shape==current_pixels.shape,
        "PnP correspondence carriers/order invalid")
    require(np.isfinite(points).all() and np.isfinite(current_pixels).all() and
        np.isfinite(reference_pixels).all() and (points[:,2]>0).all(),"invalid metric input")
    row=dict(pnp_correspondences=len(points),pnp_inliers=0,inlier_ratio="",reprojection_rmse_px="",
        grid_occupancy="",hull_fraction="",median_parallax_px="",pnp_ms=0.,pnp_attempted=0,
        status="INSUFFICIENT_CORRESPONDENCES",T_Ccur_Cref="",D_vis="",inlier_indices="",cheirality_pass="")
    if len(points)<ATTEMPT_MIN:return row
    tick=time.perf_counter()
    pose,count,rmse,take=p4.pnp(points,current_pixels.astype(float),calibration["Krect"],seed)
    row.update(pnp_ms=(time.perf_counter()-tick)*1000,pnp_attempted=1,pnp_inliers=count,
        inlier_ratio=count/len(points),reprojection_rmse_px=rmse if np.isfinite(rmse) else "",
        status="VALID" if pose is not None else "PNP_REJECT",inlier_indices=";".join(map(str,take)))
    if pose is not None:
        require(count>=INLIER_MIN and len(take)==count and np.isfinite(pose).all() and
            ((points[take]@pose[:3,:3].T+pose[:3,3])[:,2]>0).all(),"P4 quality contract violated")
        occupancy,hull=p4.feature_distribution(current_pixels[take],calibration["size"])
        row.update(T_Ccur_Cref=text(pose),D_vis=text(imu_increment(pose,calibration["T_imu_camera"])),
            grid_occupancy=occupancy,hull_fraction=hull,cheirality_pass=1,
            median_parallax_px=float(np.median(np.linalg.norm(current_pixels[take]-reference_pixels[take],axis=1))))
    return row


def estimate_pair(ref,cur,cloud,calibration,seed):
    """Full-run-only wrapper: exactly frozen R3A tracking/depth plus solve20."""
    started=time.perf_counter()
    a,b,metrics=tracking(ref,cur,calibration)
    direct,_,good,points,_,timing=associate(cloud,a,calibration)
    shared_ms=(time.perf_counter()-started)*1000
    row=solve(points,a[good].astype(float),b[good].astype(float),calibration,seed)
    row.update(metrics,**timing,direct_depth_count=int(direct.sum()),
        plane_completed_count=int(good.sum()-direct.sum()),total_depth_count=int(good.sum()),
        depth_associated=int(good.sum()),frontend_ms=shared_ms+row["pnp_ms"])
    if not metrics["detected"]:row["status"]="NO_FEATURES"
    return row


def self_test():
    k=np.array([[150.,0,120],[0,150.,90],[0,0,1.]])
    xyz=np.array([[.1*(i%5-2),.1*(i//5-2),3+.05*i] for i in range(40)],float)
    calibration=dict(Krect=k,T_imu_camera=np.eye(4),size=(240,180))
    rvec=np.array([.01,-.02,.005]);tvec=np.array([.1,-.03,.02])
    pixels=p4.cv2.projectPoints(xyz,rvec,tvec,k,None)[0].reshape(-1,2)
    reference=(xyz@k.T)[:,:2]/xyz[:,2:3]
    for count in (19,20,29,30,40):
        row=solve(xyz[:count],reference[:count],pixels[:count],calibration,1)
        if count<20:
            require(not row["pnp_attempted"] and row["status"]=="INSUFFICIENT_CORRESPONDENCES","19 admission")
        else:
            require(row["status"]=="VALID" and row["pnp_inliers"]==count and row["cheirality_pass"]==1,"20 exact-boundary quality")
        if count>=30:
            from p9_r3a_visual_frontend import solve as old_solve
            previous=old_solve(xyz[:count],pixels[:count],calibration,1)
            for field in ("status","pnp_inliers","T_Ccur_Cref","reprojection_rmse_px"):
                require(row[field]==previous[field],">=30 synthetic parity "+field)
            pixels32=pixels[:count].astype(np.float32)
            canonical=solve(xyz[:count],reference[:count],pixels32,calibration,1)
            previous32=old_solve(xyz[:count],pixels32,calibration,1)
            for field in ("status","pnp_inliers","T_Ccur_Cref","reprojection_rmse_px"):
                require(canonical[field]==previous32[field],">=30 float32 pixel canonicalization parity "+field)
    # Force a post-LM behind-camera solution through the unchanged P4 pnp guard.
    old_ransac=p4.cv2.solvePnPRansac;old_lm=p4.cv2.solvePnPRefineLM
    try:
        take=np.arange(20,dtype=np.int32).reshape(-1,1)
        p4.cv2.solvePnPRansac=lambda *a,**kw:(True,np.zeros((3,1)),np.array([[0.],[0.],[-20.]]),take)
        p4.cv2.solvePnPRefineLM=lambda *a,**kw:(np.zeros((3,1)),np.array([[0.],[0.],[-20.]]))
        row=solve(xyz[:20],reference[:20],pixels[:20],calibration,1)
        require(row["status"]=="PNP_REJECT" and not row["T_Ccur_Cref"],"cheirality rejection lost")
    finally:p4.cv2.solvePnPRansac=old_ransac;p4.cv2.solvePnPRefineLM=old_lm
    old_tracking,old_associate=globals()["tracking"],globals()["associate"]
    try:
        empty=np.empty((0,2));mask=np.zeros(0,dtype=bool)
        globals()["tracking"]=lambda *a:(empty,empty,dict(detected=0,fb_valid=0))
        globals()["associate"]=lambda *a:(mask,np.empty((0,3)),mask,np.empty((0,3)),[],{})
        row=estimate_pair(None,None,None,calibration,1)
        require(row["status"]=="NO_FEATURES" and row["depth_associated"]==0,"frozen zero-feature wrapper semantics")
    finally:globals()["tracking"]=old_tracking;globals()["associate"]=old_associate
    print("P9_R3B_ADMISSION_BOUNDARY_PARITY_CHEIRALITY_SELF_TEST=PASS")


if __name__=="__main__":
    p4.cv2.setNumThreads(1);self_test()
