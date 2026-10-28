"""GT-free original archive features. No reference/evaluated trajectory input, ROS or matching."""
import csv,gzip,hashlib,io,json,subprocess
from collections import defaultdict
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
A=Path(__file__).resolve().parents[1];W=A.parents[1];OLD=W/'docs/p63_r10_hdl_imu_full_hard';D=OLD/'raw/full_01'
START='e25754fd99e81144376e9717c23063c5b575e667'
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
windows={'A':range(180,210),'B':range(299,329),'C':range(505,535)};result=[];poses=[]
for window,ids in windows.items():
 for k in ids:
  o,n=O[k],N[k];p,t,u,prev=P(k),T(k),U(k),U(k-1);s=int(o['stamp_ns']);batch=I[s]
  assert all(int(x['last_correction_ns'])==int(O[k-1]['stamp_ns']) for x in batch)
  assert len(batch)==int(o['imu_consumed']) and sum(int(x['active_predict']) for x in batch)==int(o['imu_active_predict'])
  assert abs(sum(float(x['dt_s']) for x in batch)-float(o['imu_integrated_s']))<1e-8
  assert int(o['last_imu_stamp_ns'])<=s and int(o['prior_ukf_correction_stamp_ns'])==int(O[k-1]['stamp_ns'])
  d,dr=diff(p,t);ap,ar=diff(prev,p);bp,br=diff(prev,t);step=dt(k);delta=np.linalg.inv(p)@t
  cp=t[:3,3]-p[:3,3];mp=p[:3,3]-prev[:3,3];den=np.linalg.norm(cp)*np.linalg.norm(mp)
  active=[x for x in batch if x['active_predict']=='1']
  r={'window':window,'raw_ordinal':k,'stamp_ns':s,'dt_scan_s':step,'d_t_m':d,'d_R_deg':dr,'correction_rate_m_s':d/step,'correction_rate_deg_s':dr/step,'pred_motion_m':ap,'pred_motion_deg':ar,'ndt_jump_m':bp,'ndt_jump_deg':br,'pred_motion_rate_m_s':ap/step,'pred_motion_rate_deg_s':ar/step,'ndt_jump_rate_m_s':bp/step,'ndt_jump_rate_deg_s':br/step,'correction_pred_motion_cos':float(np.dot(cp,mp)/den) if den else None,'cos_defined':bool(den),'hasConverged':int(n['converged']),'gaussian':num(n['probability']),'nn_fitness_m2':num(n['fitness_all_nn_m2']),'native_inlier':num(o['inlier_fraction']),'native_matching_error_m2':num(o['native_inlier_error_m2']),'iterations':int(n['iterations']),'imu_consumed':len(batch),'imu_active':len(active),'last_imu_ns':int(o['last_imu_stamp_ns']),'imu_age_ms':(s-int(o['last_imu_stamp_ns']))/1e6,'imu_integrated_s':float(o['imu_integrated_s']),'max_real_imu_gap_ms':max(x['real_gap_s'] for x in batch if x['real_gap_s'] is not None)*1000 if batch else None,'last_map_correction_ns':int(o['prior_ukf_correction_stamp_ns']),'cross_correction_overlap_ms':max(0,int(active[0]['last_correction_ns'])-int(active[0]['prev_stamp_before_ns']))/1e6 if active else None,'source_points':int(n['source_points']),'source_xyz_sha256':n['source_xyz_sha256'],'target_xyz_sha256':n['target_xyz_sha256'],'history_complete':True}
  for h in [1,3,5]:
   hist=list(range(k-h,k));values=np.array([diff(P(i),T(i)) for i in hist]);span=sum(dt(i) for i in hist)
   r['h'+str(h)+'_duration_s']=span
   for j,tag,cur in [(0,'t',d),(1,'R',dr)]:
    med=float(np.median(values[:,j]));rate=float(values[:,j].sum()/span)
    r[f'h{h}_{tag}_median']=med;r[f'h{h}_{tag}_max']=float(values[:,j].max());r[f'h{h}_{tag}_ratio']=ratio(cur,med);r[f'h{h}_{tag}_rate_ratio']=ratio(cur/step,rate)
    r[f'h{h}_{tag}_denominator_nonzero']=med!=0 and rate!=0
   a,b=diff(U(k-h),p);r[f'h{h}_pred_from_past_ukf_m']=a;r[f'h{h}_pred_from_past_ukf_deg']=b;r[f'h{h}_pred_elapsed_s']=(s-int(O[k-h]['stamp_ns']))/1e9
  for i in range(4):
   for j in range(4):r[f'delta_{i}{j}']=float(delta[i,j])
  result.append(r);poses.append({'window':window,'raw_ordinal':k,'stamp_ns':s,'prediction':p.tolist(),'ndt':t.tolist(),'ukf_post_fusion_DIAGNOSTIC_ONLY':u.tolist()})
assert len(result)==90
write('CAUSAL_FEATURES.csv',result);dump('ONLINE_POSES.json',poses);dump('ALLOWLIST.json',allowed)
root=subprocess.check_output(['git','status','--porcelain=v1','-uall'],cwd='/home/jian/livox_ws/dog_loc_paper_ws')
dump('INPUT_IDENTITY.json',{'START_SHA':START,'R10_CODE_SHA':command['CODE_SHA'],'R10_raw_files':inputs,'NDT_rows':len(native),'UKF_outputs':len(online),'IMU_consumed':len(imu),'root_status_sha256':sha(root),'root_status_entries':len(root.splitlines()),'reference_read_by_feature_program':False,'new_NDT':0,'new_UKF':0,'new_ROS':0})
files=['CAUSAL_FEATURES.csv','ONLINE_POSES.json','ALLOWLIST.json','INPUT_IDENTITY.json','INPUT_AND_CAUSAL_FEATURE_CONTRACT.md','tools/extract_features.py']
dump('FEATURE_SEAL.json',{'CODE_SHA':subprocess.check_output(['git','rev-parse','HEAD'],cwd=W,text=True).strip(),'files':{p:sha((A/p).read_bytes()) for p in files},'reference_pose_opened':False,'analysis_frames':90,'history':[1,3,5],'current_UKF_used_as_prefusion_feature':False})
print('PASS original archive identities; 90 GT-free rows sealed; no NDT/UKF/ROS; covariance unavailable, NIS not computed')
