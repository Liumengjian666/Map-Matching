#!/usr/bin/env python3
"""Anchor-only initialization; no later reference pose values parsed."""
import argparse,bisect,csv,hashlib,json,math
from pathlib import Path
from decimal import Decimal
import numpy as np,yaml
from scipy.spatial.transform import Rotation
P=Path(__file__).resolve().parents[1];ROOT=P.parents[1]
D=Path('/home/jian/rosbag/public_datasets/koide_p62_r29')
a=argparse.ArgumentParser();a.add_argument('sequence',choices=['EASY','HARD']);args=a.parse_args()
seq={'EASY':'indoor_easy_01','HARD':'indoor_hard_01'}[args.sequence]
manifest=json.loads((D/(seq+'_adapter')/'manifest.json').read_text());frames=manifest['frames'];imu=[]
with Path(manifest['imu_csv']).open() as f:
 for r in csv.reader(f):
  if len(r)==9 and not r[0].startswith('#'):imu.append((int(r[1])*10**9+int(r[2]),[float(v) for v in r[3:]]))
ts=[r[0] for r in imu]
gt=list((D/'gt').rglob('traj_lidar_'+seq+'.txt'));assert len(gt)==1
# Choose the first sensor-valid anchor from point/IMU times only.
start=next(i for i,r in enumerate(frames) if bisect.bisect_right(ts,r['stamp_ns'])>0 and r['stamp_ns']-ts[bisect.bisect_right(ts,r['stamp_ns'])-1]<=20000000)
anchor=frames[start]['stamp_ns'];boundary=imu[bisect.bisect_right(ts,anchor)-1];assert anchor-boundary[0]<=20000000
# Read only the adjacent reference rows needed for this declared anchor.
before=after=None
with gt[0].open() as f:
 for line in f:
  if not line.strip() or line.startswith('#'):continue
  row=line.split();stamp=int(Decimal(row[0])*10**9)
  if stamp<=anchor:before=(stamp,row)
  if stamp>=anchor:after=(stamp,row);break
assert before and after and after[0]-before[0]<=150000000 and min(anchor-before[0],after[0]-anchor)<=50000000
from scipy.spatial.transform import Slerp
va=np.array([float(x) for x in before[1][1:]]);vb=np.array([float(x) for x in after[1][1:]])
w=(anchor-before[0])/(after[0]-before[0]) if after[0]>before[0] else 0.
ref=np.eye(4);ref[:3,3]=(1-w)*va[:3]+w*vb[:3]
ref[:3,:3]=Slerp([0.,1.],Rotation.from_quat([va[3:],vb[3:]]))([w]).as_matrix()[0]
ref_stamp=anchor
# Full tail must have an actual post-exposure IMU sample. Never add synthetic IMU.
end=len(frames)
while end>start+1 and frames[end-1]['stamp_ns']>=ts[-1]:end-=1
assert end>start+1
q=np.array([-.476,.474,.524,.525]);norm=float(np.linalg.norm(q));t_li=np.eye(4);t_li[:3,:3]=Rotation.from_quat(q/norm).as_matrix();t_li[:3,3]=[.003,.004,-.051];t_il=np.linalg.inv(t_li)
assert np.linalg.norm(t_li@t_il-np.eye(4))<1e-12
out=P/'input';out.mkdir(exist_ok=True)
runtime=yaml.safe_load((ROOT/'src/dog_prior_map_localization/config/p41_newer_college_runtime.yaml').read_text());runtime.pop('initialization',None)
for key in ['frames']:
 runtime[key]['map_frame']='benchmark_map';runtime[key]['base_frame']='benchmark_lidar'
runtime['lidar_update']['T_imu_lidar_rotation_row_major']=t_il[:3,:3].reshape(-1).tolist()
runtime['deskew']['T_imu_lidar_rotation_row_major']=t_il[:3,:3].reshape(-1).tolist();runtime['deskew']['T_imu_lidar_translation_m']=t_il[:3,3].tolist();runtime['deskew']['expected_lidar_frame']='benchmark_lidar'
rt=out/(args.sequence+'_runtime.yaml');assert not rt.exists();rt.write_text(yaml.safe_dump(runtime,sort_keys=False))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
initials={}
for arm in ['C0','C3']:
 ml=ref.copy()
 if arm=='C3':ml[:3,:3]=Rotation.from_euler('z',8,degrees=True).as_matrix()@ml[:3,:3]
 mi=ml@t_li
 initial={'explicit_state_enable':True,'state_stamp_sec':anchor//10**9,'state_stamp_nsec':anchor%10**9,
  'map_T_imu_translation_m':mi[:3,3].tolist(),'map_T_imu_rotation_row_major':mi[:3,:3].reshape(-1).tolist(),
  'velocity_map_mps':[0.,0.,0.],'gyro_bias_imu_radps':[0.,0.,0.],'accel_bias_imu_mps2':[0.,0.,0.],'gravity_map_mps2':[0.,0.,-9.80665],
  'boundary_imu_stamp_sec':boundary[0]//10**9,'boundary_imu_stamp_nsec':boundary[0]%10**9,
  'boundary_imu_gyro_radps':boundary[1][:3],'boundary_imu_accel_mps2':boundary[1][3:],
  'ndt_seed_enable':True,'ndt_seed_map_T_lidar_translation_m':ml[:3,3].tolist(),'ndt_seed_map_T_lidar_rotation_row_major':ml[:3,:3].reshape(-1).tolist()}
 fn=out/(args.sequence+'_'+arm+'_initialization.yaml');assert not fn.exists();fn.write_text(yaml.safe_dump({'initialization':initial},sort_keys=False))
 initials[arm]={'path':str(fn),'sha256':sha(fn),'T_map_lidar':ml.tolist(),'T_map_imu':mi.tolist(),'closure_error':float(np.linalg.norm(mi@t_il-ml))}
key={'name':seq,'anchor_raw_ordinal':start+1,'anchor_stamp_ns':anchor,'reference_anchor_stamp_ns':ref_stamp,'anchor_delta_ns':anchor-ref_stamp,
 'frames':frames[start+1:end],'scan_count':end-start-1,'raw_total':len(frames),'prefix_not_evaluated_including_anchor':start+1,'unbracketed_suffix_count':len(frames)-end,
 'publisher_start_stamp_ns':frames[start+1]['stamp_ns'],'imu_replay_start_stamp_ns':anchor,'expected_boundary_imu_stamp_ns':boundary[0],
 'scan_directory':str(D/(seq+'_adapter')/'scans'),'imu_csv':manifest['imu_csv'],'tile_manifest':str(D/('map_indoor_'+('easy' if args.sequence=='EASY' else 'hard')+'_tiles')/'tile_manifest.json'),
 'runtime_config':str(rt),'runtime_sha256':sha(rt),'initializations':initials,'instantaneous_scan':True,
 'T_lidar_imu':t_li.tolist(),'T_imu_lidar':t_il.tolist(),'publisher_quaternion_norm_before_normalization':norm,
 'reference_path':str(gt[0]),'reference_sha256':sha(gt[0]),'reference_values_read':'ANCHOR_BRACKET_ONLY_NO_TEST_REFERENCE',
 'adapter_manifest_sha256':sha(D/(seq+'_adapter')/'manifest.json')}
contract=out/'DATASETS.json';d=json.loads(contract.read_text()) if contract.exists() else {'datasets':{}}
assert args.sequence not in d['datasets'];d['datasets'][args.sequence]=key;contract.write_text(json.dumps(d,indent=2)+'\n')
print('INPUT_FROZEN',args.sequence,key['scan_count'],'anchor',start+1,'suffix',len(frames)-end,'initial q norm',norm)
