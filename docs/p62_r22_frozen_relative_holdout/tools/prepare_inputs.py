#!/usr/bin/env python3
import csv,json,hashlib,subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parents[1];REPO=HERE.parents[1]
DATA=Path('/home/jian/livox_ws/p59_r3_newer_college_official_internal_recovery_data/01_short_experiment')
GT=DATA/'ground_truth/registered_poses.csv'
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
def save(p,d):
 with p.open('x') as f:json.dump(d,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n')
def snapshot():
 p=Path('/home/jian/livox_ws/dog_visual_loc_ws')
 def git(*a):return subprocess.check_output(['git','--no-optional-locks','-C',str(p),*a])
 return dict(HEAD=git('rev-parse','HEAD').decode().strip(),branch=git('branch','--show-current').decode().strip(),status_sha256=hashlib.sha256(git('status','--porcelain=v1','-uall')).hexdigest(),index_sha256=sha(p/'.git/index'))
def main():
 inv=list(csv.DictReader((REPO/'docs/p62_r14b_newer_college/LOCAL_SCAN_INVENTORY.csv').open()));assert len(inv)==15301
 selected={f'W{i+1}':inv[a-1:a+60] for i,a in enumerate((2600,4800,6500,9400))}
 # Timestamp fields only for test rows; pose fields parsed for exactly four anchors.
 assert sha(GT)=='b2fdfab253ce19ae58e1566c4ef0b66fa04f6feb3798d11d41ca5e809cc4363e'
 wanted={int(r['filename_stamp_ns']) for rs in selected.values() for r in rs};anchors={int(rs[0]['filename_stamp_ns']) for rs in selected.values()}
 matched=set();values={}
 with GT.open() as f:
  next(f)
  for line in f:
   sec,nsec,remaining=line.rstrip().split(',',2);ns=int(sec)*10**9+int(nsec)
   if ns in wanted:matched.add(ns)
   if ns in anchors:values[ns]=np.array([float(x) for x in remaining.split(',')])
 assert matched==wanted and len(values)==4
 tbl=np.array(json.loads((REPO/'docs/p62_r14b_newer_college/conditional_base_pilot/TEST_FRAME_PREREGISTRATION.json').read_text())['T_B_L'])
 doc={'start_sha':'cfeb7288ce0a5cf00ae58d94ece26cb745a9b57c','windows':{},'T_B_L':tbl.tolist(),'reference_sha256':sha(GT),'reference_pose_values_read':'FOUR_ANCHORS_ONLY','evaluation_values_read':False,'timestamps_exact_match':len(matched),'stable_before':snapshot(),'protocol_sha256':sha(HERE/'PRE_REGISTRATION.md')}
 for role,rs in selected.items():
  folder=HERE/role.lower();folder.mkdir();frames=[]
  for j,r in enumerate(rs):
   path=Path(r['path']);content=path.read_bytes();offset=content.index(b'DATA binary\n')+len(b'DATA binary\n');assert offset==250 and len(content)==int(r['size_bytes'])
   ordinal=int(r['inventory_index_zero_based'])+1
   frames.append(dict(local_index=j,raw_ordinal=ordinal,stamp_ns=int(r['filename_stamp_ns']),path=str(path),points=65536,header_bytes=offset,point_step=29,pcd_bytes=len(content),pcd_sha256=hashlib.sha256(content).hexdigest()))
  assert all(frames[j]['stamp_ns']<frames[j+1]['stamp_ns'] for j in range(60))
  with (folder/'INPUT_SCANS.csv').open('x',newline='') as f:
   w=csv.DictWriter(f,fieldnames=frames[0],lineterminator='\n');w.writeheader();w.writerows(frames)
  v=values[frames[0]['stamp_ns']];assert len(v)==7 and np.isfinite(v).all() and abs(np.linalg.norm(v[3:])-1)<2e-5
  t=np.eye(4);t[:3,:3]=Rotation.from_quat(v[3:]).as_matrix();t[:3,3]=v[:3];t=t@tbl
  with (folder/'ANCHOR_MATRIX.txt').open('x') as f:f.write(' '.join(format(x,'.17g') for x in t.flat)+'\n')
  doc['windows'][role]={'frames':frames,'anchor_T_M_L':t.tolist(),'anchor_sha256':sha(folder/'ANCHOR_MATRIX.txt')}
 save(HERE/'INPUT_PREPARATION.json',doc)
 print('INPUT_PREPARATION_PASS four anchors only; 244 timestamp matches; no align')
if __name__=='__main__':main()
