"""GT-free original archive features. No reference/evaluated trajectory input, ROS or matching."""
import csv,gzip,hashlib,io,json,subprocess
from collections import defaultdict
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
A=Path(__file__).resolve().parents[1];W=A.parents[1];OLD=W/'docs/p63_r10_hdl_imu_full_hard';D=OLD/'raw/full_01'
START='b5acf73081917d5d0280818bc96ab5c729ca50b6'
sha=lambda b:hashlib.sha256(b).hexdigest()
def dump(name,v):(A/name).write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def write(name,v):
 with (A/name).open('w',newline='') as f:
  wr=csv.DictWriter(f,fieldnames=list(v[0]),lineterminator='\n');wr.writeheader();wr.writerows(v)
manifest=json.loads((OLD/'RAW_MANIFEST.json').read_text());seal=json.loads((D/'ONLINE_SEAL.json').read_text());inputs={}
for m in manifest:
 p=OLD/m['archive'];b=p.read_bytes();assert sha(b)==m['archive_sha256'];content=gzip.decompress(b) if p.suffix=='.gz' else b
 assert sha(content)==m['source_sha256']
 name=p.name.removesuffix('.gz') if hasattr(str,'removesuffix') else (p.name[:-3] if p.suffix=='.gz' else p.name)
 if name in seal['files']:assert sha(content)==seal['files'][name]
 inputs[str(p.relative_to(W))]={'file_sha256':sha(b),'uncompressed_sha256':sha(content)}
allowed={
 'ONLINE_RESULTS.csv':['raw_ordinal','stamp_ns','imu_consumed','imu_active_predict','last_imu_stamp_ns','imu_age_ms','imu_integrated_s','prior_ukf_correction_stamp_ns','inlier_fraction','native_inlier_error_m2','px','py','pz','qx','qy','qz','qw'],
 'NATIVE_NDT.csv':['stamp_ns','source_points','target_points','source_xyz_sha256','target_xyz_sha256','map_frozen','converged','probability','fitness_all_nn_m2','iterations','tx','ty','tz','qx','qy','qz','qw']+['guess_'+str(i)+str(j) for i in range(4) for j in range(4)],
 'IMU_PREDICT.csv':['scan_stamp_ns','imu_stamp_ns','prev_stamp_before_ns','last_correction_ns','active_predict','dt_s'],
 'IMU_RECEIVED.csv':['stamp_ns'], 'CALLBACKS.csv':['stamp_ns']}
def read(name):
 with gzip.open(D/(name+'.gz'),'rt') as f:
  rs=list(csv.DictReader(f));assert set(allowed[name])<=rs[0].keys()
  return [{k:r[k] for k in allowed[name]} for r in rs]
online=read('ONLINE_RESULTS.csv');native=read('NATIVE_NDT.csv');imu=read('IMU_PREDICT.csv');rx=read('IMU_RECEIVED.csv');cb=read('CALLBACKS.csv')
assert len(online)==len(native)==len(cb)==2375 and len(imu)==len(rx)==44267
assert [int(x['raw_ordinal']) for x in online]==list(range(5,2380))
freeze=json.loads((OLD/'INPUT_FREEZE.json').read_text());command=json.loads((D/'COMMAND.json').read_text());receipt=json.loads((D/'RUN_RECEIPT.json').read_text())
assert command['CODE_SHA']=='4d0ba895314eb80aae191c3bc63a37fa28861b1d' and not command['test_GT_access'] and not seal['test_reference_values_read']
assert receipt['reason']=='COMPLETED_2375' and receipt['same_ukf_instance']
assert subprocess.check_output(['git','show',START+':docs/p63_r10_hdl_imu_full_hard/raw/full_01/ONLINE_SEAL.json'],cwd=W)==(D/'ONLINE_SEAL.json').read_bytes()
for name in ['tools/run_full_once.py','tools/imu_geometry.py']:
 assert subprocess.check_output(['git','show',command['CODE_SHA']+':docs/p63_r10_hdl_imu_full_hard/'+name],cwd=W)==(OLD/name).read_bytes()
for o,n,f,c in zip(online,native,freeze['frames'],cb):
 assert int(o['stamp_ns'])==int(n['stamp_ns'])==int(c['stamp_ns'])==f['stamp_ns']
 assert n['source_xyz_sha256']==f['source_xyz_sha256'] and n['target_xyz_sha256']==freeze['target_xyz_sha256'] and n['map_frozen']=='1'
for a,b in zip(imu,rx):assert a['imu_stamp_ns']==b['stamp_ns'] and int(a['imu_stamp_ns'])<=int(a['scan_stamp_ns'])
assert all(int(b['imu_stamp_ns'])>int(a['imu_stamp_ns']) for a,b in zip(imu,imu[1:]))
# Integer raw indexing only. No reference values or reference error columns exist here.
O={int(x['raw_ordinal']):x for x in online};N={i:n for i,n in zip(O,native)};I=defaultdict(list)
for i,r in enumerate(imu):
 r=dict(r);r['real_gap_s']=(int(r['imu_stamp_ns'])-int(imu[i-1]['imu_stamp_ns']))/1e9 if i else None
 I[int(r['scan_stamp_ns'])].append(r)
def pospose(r,pk):
 t=np.eye(4);t[:3,3]=[float(r[k]) for k in pk];q=[float(r[k]) for k in ['qx','qy','qz','qw']]
 assert np.isfinite(t).all() and np.isfinite(q).all() and np.linalg.norm(q)>0
 t[:3,:3]=Rotation.from_quat(q).as_matrix();return t
def P(k):
 t=np.array([float(N[k]['guess_'+str(i)+str(j)]) for i in range(4) for j in range(4)]).reshape(4,4);assert np.isfinite(t).all();return t
def T(k):return pospose(N[k],['tx','ty','tz'])
def U(k):return pospose(O[k],['px','py','pz'])
def diff(a,b):return float(np.linalg.norm(a[:3,3]-b[:3,3])),float(np.degrees(Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).magnitude()))
def dt(k):return (int(O[k]['stamp_ns'])-int(O[k-1]['stamp_ns']))/1e9
def ratio(a,b):return a/b if b!=0 else None
def num(s):
 v=float(s);return v if np.isfinite(v) else None

assert not (A/'SHADOW_FEATURES.csv').exists(), 'Do not overwrite formal features'
r11=set(range(180,210))|set(range(299,329))|set(range(505,535))
r12=set(range(240,270))|set(range(350,380))|set(range(450,480))|set(range(550,580))
for name,expected in [('p63_r11_harmful_ndt_observation/CAUSAL_FEATURES.csv',r11),('p63_r12_risk_shadow_ranking/SHADOW_FEATURES.csv',r12)]:
 with (W/'docs'/name).open() as z: actual={int(x['raw_ordinal']) for x in csv.DictReader(z)}
 assert actual==expected
formal=sorted(set(O)-r11-r12)
assert len(r11)==90 and len(r12)==120 and len(formal)==2165
assert not (r11&r12 or r11&set(formal) or r12&set(formal))
assert r11|r12|set(formal)==set(range(5,2380))
segments=[('S1',5,179),('S2',210,239),('S3',270,298),('S4',329,349),('S5',380,449),('S6',480,504),('S7',535,549),('S8',580,2379)]
assert sorted(k for _,a,b in segments for k in range(a,b+1))==formal
corrections={k:diff(P(k),T(k)) for k in O} # raw online only; no GT, no optimization.
result=[];poses=[]
for k in formal:
 o,n=O[k],N[k];p,t=P(k),T(k);s=int(o['stamp_ns']);d,dr=corrections[k]
 window=next(name for name,a,b in segments if a<=k<=b)
 r={'window':window,'raw_ordinal':k,'stamp_ns':s,'d_t_m':d,'d_R_deg':dr,
 'gaussian':num(n['probability']),'native_inlier':num(o['inlier_fraction']),'nn_fitness_m2':num(n['fitness_all_nn_m2']),
 'source_points':int(n['source_points']),'source_xyz_sha256':n['source_xyz_sha256'],'target_xyz_sha256':n['target_xyz_sha256']}
 for feature in ['d_t_m','d_R_deg','gaussian','native_inlier','nn_fitness_m2']:
  r[feature+'_missing_reason']='' if r[feature] is not None and np.isfinite(r[feature]) else 'NONFINITE'
 for h in [1,3,5]:
  hist=list(range(k-h,k));complete=all(i in corrections for i in hist)
  assert all(int(O[i]['stamp_ns'])<s for i in hist if i in O)
  r[f'h{h}_history_available_n']=sum(i in O for i in hist)
  r[f'h{h}_first_stamp_ns']=int(O[k-h]['stamp_ns']) if complete else None
  r[f'h{h}_last_stamp_ns']=int(O[k-1]['stamp_ns']) if complete else None
  r[f'h{h}_current_minus_first_s']=(s-int(O[k-h]['stamp_ns']))/1e9 if complete else None
  for j,tag,cur in [(0,'t',d),(1,'R',dr)]:
   med=float(np.median([corrections[i][j] for i in hist])) if complete else None
   val=ratio(cur,med) if med is not None and np.isfinite(med) else None
   reason='HISTORY_INSUFFICIENT' if not complete else 'NONFINITE' if not np.isfinite(med) else 'ZERO_DENOMINATOR' if med==0 else 'NONFINITE' if val is None or not np.isfinite(val) else ''
   r[f'h{h}_{tag}_median']=med
   r[f'h{h}_{tag}_ratio']=val if not reason else None
   r[f'h{h}_{tag}_ratio_missing_reason']=reason
 result.append(r);poses.append({'window':window,'raw_ordinal':k,'stamp_ns':s,'prediction':p.tolist(),'ndt':t.tolist()})
write('SHADOW_FEATURES.csv',result);dump('ONLINE_POSES.json',poses);dump('ALLOWLIST.json',allowed)
dump('PARTITION.json',{'r11':sorted(r11),'r12':sorted(r12),'r13':formal,'original_count':len(O),'r11_count':len(r11),'r12_count':len(r12),'r13_count':len(formal),'pairwise_disjoint':True,'union_exact':True,'segments':segments})
root=subprocess.check_output(['git','status','--porcelain=v1','-uall'],cwd='/home/jian/livox_ws/dog_loc_paper_ws')
dump('INPUT_IDENTITY.json',{'START_SHA':START,'R10_CODE_SHA':command['CODE_SHA'],'R10_raw_files':inputs,'NDT_rows':len(native),'UKF_outputs':len(online),'IMU_consumed':len(imu),'root_status_sha256':sha(root),'root_status_entries':len(root.splitlines()),'reference_read_by_feature_program':False,'new_NDT':0,'new_UKF':0,'new_ROS':0})
files=['SHADOW_FEATURES.csv','ONLINE_POSES.json','PARTITION.json','ALLOWLIST.json','INPUT_IDENTITY.json','PRE_REGISTERED_CONTRACT.md','tools/extract_features.py']
dump('FEATURE_SEAL.json',{'CODE_SHA':subprocess.check_output(['git','rev-parse','HEAD'],cwd=W,text=True).strip(),'files':{p:sha((A/p).read_bytes()) for p in files},'reference_pose_opened':False,'analysis_frames':2165,'history':[1,3,5],'current_UKF_used_as_prefusion_feature':False})
print('PASS original identities and partition; 2165 GT-free rows sealed; 0 NDT/UKF/ROS')
