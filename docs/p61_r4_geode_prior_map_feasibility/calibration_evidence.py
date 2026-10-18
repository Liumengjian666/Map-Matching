"""Compare published numeric evidence only; never executes downloaded code or applies calibration."""
import hashlib
import json
from pathlib import Path
import re
import sys
import numpy as np


def opencv_matrix(text, name):
    matches = re.findall(r'^'+re.escape(name)+r':.*?data:\s*\[(.*?)\]', text, re.M|re.S)
    if len(matches)!=1: raise ValueError('matrix must occur exactly once: '+name)
    values=[float(x) for x in matches[0].split(',')]
    if len(values)!=16: raise ValueError('4x4 required')
    return np.array(values).reshape(4,4)


def audit(evidence, output):
    evidence=Path(evidence)
    drive=(evidence/'drive_gamma_config_response.bin').read_text()
    git=(Path(__file__).resolve().parent.parent/'p61_geode_native_tunnel_entry/official/gamma_config.yaml').read_text()
    fast=(evidence/'fastlio_carol.yaml').read_text()
    T=opencv_matrix(drive,'T_IMU_LiDAR');old=opencv_matrix(git,'T_IMU_LiDAR')
    R=np.array([float(x) for x in re.search(r'extrinsic_R:\s*\[(.*?)\]',fast,re.S)[1].split(',')]).reshape(3,3)
    t=np.array([float(x) for x in re.search(r'extrinsic_T:\s*\[(.*?)\]',fast,re.S)[1].split(',')])
    if not np.array_equal(T[:3,:3],R) or not np.array_equal(T[:3,3],t):
        raise ValueError('Drive versus author FAST-LIO extrinsic differs')
    new_fields={k:re.search(r'^'+k+r':\s*(.*?)\s*(?:#.*)?$',drive,re.M)[1].strip().strip('"') for k in
                ('imu_topic','pointcloud_topic','VERT_RES')}
    old_fields={k:re.search(r'^'+k+r':\s*(.*?)\s*(?:#.*)?$',git,re.M)[1].strip().strip('"') for k in new_fields}
    q=np.array([.999901,-.00492765,.00575961,.0117651]) # exact script constants, wxyz
    w,x,y,z=q
    A=np.eye(4)
    A[:3,:3]=[[1-2*y*y-2*z*z,2*x*y-2*z*w,2*x*z+2*y*w],
              [2*x*y+2*z*w,1-2*x*x-2*z*z,2*y*z-2*x*w],
              [2*x*z-2*y*w,2*y*z+2*x*w,1-2*x*x-2*y*y]]
    A[:3,3]=[.00947221,-.308202,-.365733]
    result={'drive_fields':new_fields,'old_git_fields':old_fields,
            'drive_sha256':hashlib.sha256((evidence/'drive_gamma_config_response.bin').read_bytes()).hexdigest(),
            'drive_matches_actual_lidar_and_xsens_topics':new_fields=={'imu_topic':'/imu/data','pointcloud_topic':'/livox/lidar','VERT_RES':'6'},
            'drive_matrix_equals_old_git_numeric_matrix':bool(np.array_equal(T,old)),
            'drive_matrix_equals_author_fastlio_matrix':True,'T_I_L':T.tolist(),
            'T_I_L_inverse':np.linalg.solve(T,np.eye(4)).tolist(),
            'rotation_orthogonality_fro':float(np.linalg.norm(T[:3,:3].T@T[:3,:3]-np.eye(3))),
            'rotation_determinant':float(np.linalg.det(T[:3,:3])),
            'direction_evidence':'author FAST_LIO pointBodyToWorld maps p_L through offset_R_L_I and offset_T_L_I, then IMU pose',
            'imu_instance':'external Xsens /imu/data, not integrated /livox/imu',
            'applied_to_runtime':False,'empirically_validated_on_this_bag':False,
            'leica_script_T_raw':A.tolist(),'leica_script_quaternion_norm':float(np.linalg.norm(q)),
            'leica_script_operation':'T_eval = T_device @ solve(T, I); reference-point right composition, not T_GT_map',
            'T_GT_map_available':False,'GT_rotation_available':False}
    with Path(output).open('x') as stream:json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in result.items() if not isinstance(v,list)},indent=2))


if __name__=='__main__':audit(*sys.argv[1:])
