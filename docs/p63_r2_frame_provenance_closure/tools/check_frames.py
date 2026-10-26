#!/usr/bin/env python3
"""Read frozen R1/producer artifacts only. No ROS, optimizer, fitting or asset writes."""
import hashlib,json,math,subprocess
from pathlib import Path
import numpy as np
A=Path(__file__).resolve().parents[1]
W=A.parents[1]

def angle(a,b):
    return float(np.degrees(np.arccos(np.clip(np.dot(a,b)/(np.linalg.norm(a)*np.linalg.norm(b)),-1,1))))

def align(a,b):
    # Closed-form FromTwoVectors for these fixed non-antipodal vectors, not a fit.
    a=a/np.linalg.norm(a);b=b/np.linalg.norm(b)
    c=float(a@b);v=np.cross(a,b)
    assert c>-1+1e-10
    K=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    return np.eye(3)+K+K@K/(1+c)

def transform(R,t):
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=t
    return T

# Freeze the actual inputs used by this analysis without re-reading the bag.
ids=json.loads((A/'evidence/INPUT_IDENTITIES.json').read_text())
r1path=Path(ids['R1_gravity_reconstruction']['path'])
assert hashlib.sha256(r1path.read_bytes()).hexdigest()==ids['R1_gravity_reconstruction']['sha256']
r1=json.loads(r1path.read_text());u=np.array(r1['mean_raw_accel']);u/=np.linalg.norm(u)
z=np.array([0.,0.,1.]);G=9.7946;gI=-G*u
Rg=align(gI,-G*z)
yaw=math.atan2(Rg[1,0],Rg[0,0]);cy,sy=math.cos(-yaw),math.sin(-yaw)
Rz=np.array([[cy,-sy,0],[sy,cy,0],[0,0,1.]])
Rpre=Rz@Rg  # R1 manual pose is identity; remove yaw exactly as the native initializer.
Tpre=transform(Rpre,[0,0,0]);Tfinal=np.array(r1['logged_ICP_T_map_imu_rounded'])
Rfinal=Tfinal[:3,:3];Tdelta=Tfinal@np.linalg.inv(Tpre)
fmap=Rfinal@u;fmap/=np.linalg.norm(fmap)
gmap=-G*fmap;ares=G*fmap-G*z
older=json.loads((A/'evidence/OLDER_MAPPING_LOG_EXCERPTS.json').read_text())
gravity_line=next(x['text'] for x in older['selected'] if 'IMU Initials: Gravity:' in x)
oldg=np.array([float(x) for x in gravity_line.split('IMU Initials: Gravity:',1)[1].split()[:3]])
states=json.loads((A/'evidence/NEIGHBOR_STATE_PREFIX.json').read_text())
p0=np.array([float(x) for x in states['pose_first10'][0].split()])
q=p0[4:8];qnorm=float(np.linalg.norm(q));q=q/qnorm
x,y,zz,ww=q
Rpose=np.array([[1-2*(y*y+zz*zz),2*(x*y-zz*ww),2*(x*zz+y*ww)],
 [2*(x*y+zz*ww),1-2*(x*x+zz*zz),2*(y*zz-x*ww)],
 [2*(x*zz-y*ww),2*(y*zz+x*ww),1-2*(x*x+y*y)]])
checks={
 'Rpre_orthogonality_error':float(np.linalg.norm(Rpre.T@Rpre-np.eye(3))),
 'Rpre_det':float(np.linalg.det(Rpre)),
 'Rpre_specific_force_to_positive_Z_error':float(np.linalg.norm(Rpre@u-z)),
 'Rpre_gravity_to_negative_Z_error':float(np.linalg.norm(Rpre@gI+G*z)),
 'logged_R_orthogonality_error_due_to_print_precision':float(np.linalg.norm(Rfinal.T@Rfinal-np.eye(3))),
 'logged_R_det':float(np.linalg.det(Rfinal)),
 'combined_delta_reconstructs_final_error':float(np.linalg.norm(Tdelta@Tpre-Tfinal)),
 'combined_delta_maps_positive_Z_to_final_force_error':float(np.linalg.norm(Tdelta[:3,:3]@z-Rfinal@u)),
 'gravity_and_force_angles_agree_error_deg':abs(angle(gmap,-z)-angle(fmap,z)),
 'first_logged_pose_quaternion_norm_before_normalization':qnorm,
}
assert checks['Rpre_orthogonality_error']<1e-12
assert checks['Rpre_specific_force_to_positive_Z_error']<1e-12
assert checks['Rpre_gravity_to_negative_Z_error']<1e-11
assert checks['combined_delta_reconstructs_final_error']<1e-12
assert checks['gravity_and_force_angles_agree_error_deg']<1e-10
assert np.isfinite(Tdelta).all() and abs(angle(fmap,z)-38.02105923169883)<1e-8
r={
 'task':'P63-R2-FROZEN-MAP-FRAME-PROVENANCE-CLOSURE',
 'analysis_CODE_SHA':subprocess.check_output(['git','rev-parse','HEAD'],cwd=W,text=True).strip(),
 'inputs':'frozen R1 mean and six-digit ICP log;adjacent producer pose;explicitly older different-run gravity log',
 'no_new_NDT_ICP_ESKF_calls':True,'no_transform_applied_to_assets':True,'no_rotation_fitted':True,
 'specific_force_unit_I':u.tolist(),'gravity_I_Super_units':gI.tolist(),
 'input_force_tilt_from_I_positive_Z_deg':angle(u,z),
 'R_gravity_alignment_S_I':Rg.tolist(),'native_yaw_removed_rad':yaw,
 'T_reconstructed_pre_registration_numeric':Tpre.tolist(),
 'T_logged_final_ICP_M_I_rounded':Tfinal.tolist(),
 'T_combined_optimizer_delta_not_an_independent_map_calibration':Tdelta.tolist(),
 'NDT_intermediate_transform':'NOT_RECORDED;cannot separate NDT contribution from ICP without a prohibited rerun',
 'final_force_unit_M':fmap.tolist(),'final_implied_gravity_M':gmap.tolist(),
 'final_force_vs_Super_positive_Z_deg':angle(fmap,z),
 'final_stationary_acceleration_residual_mps2':ares.tolist(),
 'final_stationary_acceleration_residual_norm_mps2':float(np.linalg.norm(ares)),
 'older_Aug8_logged_gravity':oldg.tolist(),
 'older_gravity_tilt_from_negative_Z_deg':angle(oldg,-z),
 'older_gravity_norm_mps2':float(np.linalg.norm(oldg)),
 'older_force_vs_R1_input_force_deg':angle(-oldg,u),
 'final_force_vs_older_map_positive_force_deg_conditional':angle(fmap,-oldg),
 'ICP_rotation_from_identity_deg_approx':float(np.degrees(np.arccos(np.clip((np.trace(Rfinal)-1)/2,-1,1)))),
 'neighbor_pose_first':p0.tolist(),
 'neighbor_first_rotation_from_identity_deg':float(np.degrees(2*np.arccos(abs(q[3])))),
 'neighbor_first_force_tilt_if_same_stationary_IMU_deg_conditional':angle(Rpose@u,z),
 'algebra_checks':checks,
 'interpretation':'23.62deg raw-frame tilt and 38.02deg final world inconsistency are distinct non-additive angles;the combined registration rotation is not an independently certified map-frame transform;all physical inferences remain conditional on quasi-stationarity and artifact identity',
}
(A/'evidence/FRAME_NUMERICS.json').write_text(json.dumps(r,indent=2,allow_nan=False)+'\n')
print(json.dumps({k:r[k] for k in ['analysis_CODE_SHA','input_force_tilt_from_I_positive_Z_deg','final_force_vs_Super_positive_Z_deg','older_gravity_tilt_from_negative_Z_deg','older_force_vs_R1_input_force_deg','final_force_vs_older_map_positive_force_deg_conditional','neighbor_first_rotation_from_identity_deg','final_stationary_acceleration_residual_norm_mps2']},indent=2))
