#!/usr/bin/env python3
"""Zero-align Hard admission. Only raw4 reference pose is parsed before trials."""
import hashlib,json,os,subprocess,zipfile
from pathlib import Path
from decimal import Decimal
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores,get_typestore
from data import sha,pcd_xyz,write_xyzi_pcd
A=Path(__file__).resolve().parents[1];W=A.parents[1]
D=Path('/home/jian/livox_ws/p63_r8_indoor_hard_holdout_data')
ROOT=Path('/home/jian/rosbag/public_datasets/koide_p62_r29')
def git_json(p):return json.loads(subprocess.check_output(['git','show','HEAD:'+p],cwd=W,text=True))
assert os.statvfs(D).f_bavail*os.statvfs(D).f_frsize>=20*1024**3
official=git_json('docs/p62_r29_full_sequence_benchmark/sources/zenodo_record.json')
history=git_json('docs/p62_r29_full_sequence_benchmark/logs/download_receipts.json')
receipts=[]
for name in ['indoor_hard_01.zip','map_indoor_hard.ply','gt.zip']:
    p=ROOT/name;info=next(f for f in official['files'] if f['key']==name);old=next(f for f in history if f['path']==str(p))
    assert p.is_file() and p.stat().st_size==info['size']==old['bytes']
    md5=hashlib.md5();s=hashlib.sha256()
    with p.open('rb') as stream:
        for b in iter(lambda:stream.read(4*1024*1024),b''):md5.update(b);s.update(b)
    assert 'md5:'+md5.hexdigest()==info['checksum'] and s.hexdigest()==old['sha256']
    receipts.append({'path':str(p),'bytes':p.stat().st_size,'md5':md5.hexdigest(),'sha256':s.hexdigest(),'publisher_and_history_match':True})
    print('VERIFIED',name,flush=True)
adapter_path=ROOT/'indoor_hard_01_adapter/manifest.json';adapter=json.loads(adapter_path.read_text())
assert adapter['selected_message_counts']['/points2/decompressed']==len(adapter['frames'])==2379
assert adapter['xyz_bitwise_roundtrip']=='PASS_ALL_POINTS_ALL_SCANS'
frames=adapter['frames'][:164];assert [f['raw_ordinal'] for f in frames]==list(range(1,165))
selected=frames[3:];assert [f['raw_ordinal'] for f in selected]==list(range(4,165))
store=get_typestore(Stores.ROS2_HUMBLE)
with Reader(Path(adapter['source_metadata']).parent) as reader:
    connection=next(c for c in reader.connections if c.topic=='/points2/decompressed')
    assert connection.msgcount==2379
    for index,(_,bagtime,cdr) in enumerate(reader.messages(connections=[connection]),1):
        if index>164:break
        if index<4:continue
        f=frames[index-1];message=store.deserialize_cdr(cdr,connection.msgtype)
        ns=int(message.header.stamp.sec)*10**9+int(message.header.stamp.nanosec)
        assert ns==f['stamp_ns'] and int(bagtime)==f['bag_stamp_ns'] and message.header.frame_id==f['frame_id']=='depth_camera_link'
        assert not message.is_bigendian and not any(v.name in ('t','time','timestamp') for v in message.fields)
        fields={v.name:v for v in message.fields}
        assert all(fields[k].datatype==7 and fields[k].count==1 for k in ('x','y','z'))
        dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[fields[k].offset for k in ('x','y','z')],'itemsize':message.point_step})
        view=np.ndarray((message.height,message.width),dtype=dtype,buffer=message.data,strides=(message.row_step,message.point_step))
        raw=np.column_stack([view[k].reshape(-1) for k in ('x','y','z')]);converted,_=pcd_xyz(f['path'])
        assert sha(f['path'])==f['sha256'] and converted.tobytes()==raw.tobytes() and np.isfinite(raw).all()
        f['xyz_sha256']=hashlib.sha256(raw.tobytes()).hexdigest();f['finite_xyz_actual']=len(raw)
metadata_path=Path(adapter['source_metadata']);oldmeta=next(x for x in git_json('docs/p62_r29_full_sequence_benchmark/sources/EXTRACTED_BAG_IDENTITIES.json') if x['path']==str(metadata_path))
assert sha(metadata_path)==oldmeta['sha256']
tiles_path=ROOT/'map_indoor_hard_tiles/tile_manifest.json';tiles=json.loads(tiles_path.read_text());clouds=[]
for tile in tiles['tiles']:
    assert sha(tile['tile_path'])==tile['sha256'];xyz,_=pcd_xyz(tile['tile_path']);assert len(xyz)==tile['point_count'];clouds.append(xyz)
target=np.concatenate(clouds);assert len(target)==tiles['global_target_point_count']==96324
map_path=D/'r29_hard_tiles_exact_xyzi.pcd';write_xyzi_pcd(map_path,target)
gt=ROOT/'gt/gt/traj_lidar_indoor_hard_01.txt'
with zipfile.ZipFile(ROOT/'gt.zip') as z:assert z.read('gt/traj_lidar_indoor_hard_01.txt')==gt.read_bytes()
# Read timestamp strings until the raw4 bracket; do not parse evaluation poses.
ns=selected[0]['stamp_ns'];previous=None;bracket=None
with gt.open() as stream:
    for line in stream:
        if not line.strip() or line.startswith('#'):continue
        tokens=line.split();t=int(Decimal(tokens[0])*10**9)
        if t==ns:bracket=((t,tokens),(t,tokens));break
        if t>ns:
            assert previous is not None;bracket=(previous,(t,tokens));break
        previous=(t,tokens)
assert bracket is not None
(ta,va),(tb,vb)=bracket;assert tb-ta<=150000000 and min(abs(ns-ta),abs(ns-tb))<=50000000
pa=np.array(list(map(float,va[1:])));pb=np.array(list(map(float,vb[1:])))
w=(ns-ta)/(tb-ta) if ta!=tb else 0.;anchor=np.eye(4);anchor[:3,3]=(1-w)*pa[:3]+w*pb[:3]
anchor[:3,:3]=Rotation.from_quat(pa[3:]).as_matrix() if ta==tb else Slerp([0.,1.],Rotation.from_quat([pa[3:],pb[3:]]))([w]).as_matrix()[0]
assert np.isfinite(anchor).all()
F={'dataset':'Koide indoor_hard_01 / Azure Kinect depth snapshots','raw_anchor':4,'anchor_stamp_ns':ns,
   'T_map_lidar_anchor':anchor.tolist(),'anchor_quaternion_xyzw':Rotation.from_matrix(anchor[:3,:3]).as_quat().tolist(),
   'oracle_start':True,'anchor_reference_bracket_ns':[ta,tb],'frames':selected[1:],
   'map_path':str(map_path),'map_sha256':sha(map_path),'map_points':len(target),
   'source_tiles':tiles['tiles'],'tile_manifest_sha256':sha(tiles_path),'adapter_manifest_sha256':sha(adapter_path),
   'input_frame':'depth_camera_link','use_imu':False,'map_coordinates_transformed':False,'candidate_raws':[30,55,80,105,130,155]}
(A/'INPUT_FREEZE.json').write_text(json.dumps(F,indent=2)+'\n')
(A/'REFERENCE_METADATA.json').write_text(json.dumps({'reference_path':str(gt),'reference_sha256':sha(gt),
   'reference_kind':'MAP_ANCHORED_OFFLINE_GICP_IMU_REFERENCE_NOT_MOCAP'},indent=2)+'\n')
(A/'ASSET_PREFLIGHT.json').write_text(json.dumps({'payloads':receipts,'adapter_manifest_sha256':sha(adapter_path),
   'raw_CDR_to_PCD_bitwise_verified_range':[4,164],'verified_clouds':161,'raw4_anchor_valid':True,
   'evaluation_reference_poses_parsed':False,'historical_formal_Hard_runs':0,'source_map_sha256':sha(ROOT/'map_indoor_hard.ply')},indent=2)+'\n')
print('ADMISSION_PASS 161 original CDR/XYZ/header identities; raw4 anchor only; new NDT=0',flush=True)
