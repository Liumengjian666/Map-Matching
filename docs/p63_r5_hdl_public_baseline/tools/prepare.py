#!/usr/bin/env python3
"""Prepare only the certified map, 160 input identities and the old raw4 anchor."""
import hashlib, json, os, subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from data import pcd_xyz, sha, write_xyzi_pcd

A=Path(__file__).resolve().parents[1]; W=A.parents[1]
D=Path('/home/jian/livox_ws/p63_r5_hdl_public_baseline_data')
old=json.loads(subprocess.check_output(['git','show','HEAD:docs/p62_r29_full_sequence_benchmark/input/DATASETS.json'],cwd=W,text=True))['datasets']['EASY']
assert os.statvfs(D).f_bavail*os.statvfs(D).f_frsize >= 20*1024**3
assert old['anchor_raw_ordinal']==4 and old['scan_count']==2023
adapter=Path(old['scan_directory']).parent/'manifest.json'
assert sha(adapter)==old['adapter_manifest_sha256']
frames=old['frames'][:160]
assert [f['raw_ordinal'] for f in frames]==list(range(5,165))
for f in frames:
    assert sha(f['path'])==f['sha256']
    xyz,header=pcd_xyz(f['path'])
    assert len(xyz)==f['points'] and f['frame_id']=='depth_camera_link'
    f['xyz_sha256']=hashlib.sha256(xyz.tobytes()).hexdigest()
    f['finite_xyz_actual']=int(np.isfinite(xyz).all(axis=1).sum())
    f['pcd_fields']=header['FIELDS']
manifest_path=Path(old['tile_manifest']); manifest=json.loads(manifest_path.read_text())
clouds=[]
for tile in manifest['tiles']:
    assert sha(tile['tile_path'])==tile['sha256']
    xyz,_=pcd_xyz(tile['tile_path']);assert len(xyz)==tile['point_count']
    clouds.append(xyz)
target=np.concatenate(clouds);assert len(target)==manifest['global_target_point_count']==93146
map_path=D/'r29_all_tiles_exact_xyzi.pcd';write_xyzi_pcd(map_path,target)
anchor=np.array(old['initializations']['C0']['T_map_lidar'])
assert np.allclose(anchor[3],[0,0,0,1]) and np.allclose(anchor[:3,:3].T@anchor[:3,:3],np.eye(3),atol=1e-12)
extrinsic=np.array(old['T_lidar_imu']);assert np.allclose(anchor@extrinsic,np.array(old['initializations']['C0']['T_map_imu']))
online={'dataset':'Koide indoor_easy_01 / Azure Kinect depth snapshots','raw_anchor':4,
        'anchor_stamp_ns':old['anchor_stamp_ns'],'T_map_lidar_anchor':anchor.tolist(),
        'anchor_quaternion_xyzw':Rotation.from_matrix(anchor[:3,:3]).as_quat().tolist(),
        'oracle_start':True,'frames':frames,'map_path':str(map_path),'map_sha256':sha(map_path),
        'map_points':len(target),'map_xyz_sha256':hashlib.sha256(target.tobytes()).hexdigest(),
        'source_tiles':manifest['tiles'],'tile_manifest_sha256':sha(manifest_path),
        'adapter_manifest_sha256':sha(adapter),'input_frame':'depth_camera_link',
        'reference_pose_source':'Previously frozen R29 raw4 anchor ONLY; no evaluation reference opened.',
        'use_imu':False,'map_coordinates_transformed':False}
(A/'INPUT_FREEZE.json').write_text(json.dumps(online,indent=2)+'\n')
# Separated metadata for post-seal evaluation; no evaluation pose values are read here.
offline={k:old[k] for k in ('reference_path','reference_sha256','T_lidar_imu','T_imu_lidar')}
offline['reference_kind']='MAP_ANCHORED_OFFLINE_GICP_IMU_REFERENCE_NOT_MOCAP'
(A/'REFERENCE_METADATA.json').write_text(json.dumps(offline,indent=2)+'\n')
print('PASS exact 160 PCD identities/XYZ, 8 tiles/93146 points, raw4 SE3 and source extrinsic closure; no test reference poses read.')
