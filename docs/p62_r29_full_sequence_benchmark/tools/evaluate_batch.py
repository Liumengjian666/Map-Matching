#!/usr/bin/env python3
"""Seal complete online outputs first, then evaluate reference; no alignment or feedback."""
import argparse,bisect,csv,hashlib,json,math
from collections import defaultdict
from decimal import Decimal
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
P=Path(__file__).resolve().parents[1]

def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
def rows(p):
 with Path(p).open() as f:return list(csv.DictReader(f))
def flag(s):return str(s).lower() in ['1','true']
def num(s):
 try:v=float(s);return v if math.isfinite(v) else None
 except (TypeError,ValueError):return None
def pose(r):
 t=np.eye(4);q=np.array([float(r[k]) for k in ['qx','qy','qz','qw']]);t[:3,3]=[float(r[k]) for k in ['px','py','pz']]
 if not np.isfinite(t).all() or not np.isfinite(q).all() or np.linalg.norm(q)<1e-10:return None
 t[:3,:3]=Rotation.from_quat(q).as_matrix();return t
def mat(r,prefix):
 try:t=np.array([float(r[prefix+str(i)+str(j)]) for i in range(4) for j in range(4)]).reshape(4,4)
 except (KeyError,ValueError):return None
 if not np.isfinite(t).all() or abs(np.linalg.det(t[:3,:3])-1)>1e-3:return None
 return t
def error(a,b):
 if a is None or b is None:return None,None
 return float(np.linalg.norm(a[:3,3]-b[:3,3])),float(Rotation.from_matrix(b[:3,:3].T@a[:3,:3]).magnitude()*180/math.pi)
def stats(v):
 a=np.array([x for x in v if x is not None and math.isfinite(x)])
 return {'n':len(a),'rmse':float(np.sqrt(np.mean(a*a))) if len(a) else None,'mean':float(np.mean(a)) if len(a) else None,'p95':float(np.percentile(a,95)) if len(a) else None,'max':float(np.max(a)) if len(a) else None}
def write(p,rs):
 if not rs:return
 with p.open('w',newline='') as f:
  w=csv.DictWriter(f,fieldnames=list(rs[0]));w.writeheader();w.writerows(rs)
def longest(bs):
 best=run=0
 for x in bs:run=run+1 if x else 0;best=max(best,run)
 return best
def reference_episodes(records):
 episodes=[];active=None;high_run=low_run=0;last_high=None
 for i,r in enumerate(records):
  if not r['reference_valid']:
   high_run=low_run=0
   continue # Unknown evidence cannot close an existing failure or prove recovery.
  if r['reference_high_error']:
   high_run+=1;low_run=0;last_high=i
   if active is None and high_run>=3:active=i-2
  else:
   low_run+=1;high_run=0
   if active is not None and low_run>=3:
    recovered=i-2
    episodes.append({'first_raw':records[active]['raw_ordinal'],'last_raw':records[last_high]['raw_ordinal'],
     'startup':active==0,'recovery_raw':records[recovered]['raw_ordinal'],
     'recovery_seconds':(records[recovered]['scan_stamp_ns']-records[active]['scan_stamp_ns'])/1e9})
    active=None
 if active is not None:
  episodes.append({'first_raw':records[active]['raw_ordinal'],'last_raw':records[last_high]['raw_ordinal'],
   'startup':active==0,'recovery_raw':None,'recovery_seconds':None})
 return episodes

class Reference:
 def __init__(self,config):
  f=Path(config['reference_path']);assert sha(f)==config['reference_sha256']
  self.ts=[];self.v=[];self.cache={};self.extrinsic=np.array(config.get('reference_to_lidar',np.eye(4)))
  with f.open() as lines:
   for line in lines:
    if not line.strip() or line.startswith('#'):continue
    x=line.strip().replace(',',' ').split()
    if len(x)==8:t=int(Decimal(x[0])*10**9);v=[float(y) for y in x[1:]]
    elif len(x)==9:
     try:t=int(x[0])*10**9+int(x[1]);v=[float(y) for y in x[2:]]
     except ValueError:continue
    else:raise RuntimeError('unsupported reference row')
    self.ts.append(t);self.v.append(v)
  assert all(a<b for a,b in zip(self.ts,self.ts[1:]))
 def at(self,t):
  if t in self.cache:return self.cache[t]
  i=bisect.bisect_left(self.ts,t)
  if i==0:a=b=0
  elif i==len(self.ts):a=b=len(self.ts)-1
  elif self.ts[i]==t:a=b=i
  else:a,b=i-1,i
  ta,tb=self.ts[a],self.ts[b]
  if tb-ta>150000000 or min(abs(t-ta),abs(t-tb))>50000000:return None
  va,vb=np.array(self.v[a]),np.array(self.v[b]);out=np.eye(4)
  w=(t-ta)/(tb-ta) if tb>ta else 0.
  out[:3,3]=(1-w)*va[:3]+w*vb[:3]
  out[:3,:3]=Rotation.from_quat(va[3:]).as_matrix() if a==b else Slerp([0.,1.],Rotation.from_quat([va[3:],vb[3:]]))([w]).as_matrix()[0]
  out=out@self.extrinsic;self.cache[t]=out;return out

def main():
 a=argparse.ArgumentParser();a.add_argument('--dataset',required=True);a.add_argument('--arm',required=True);a.add_argument('--tag',default='full_02');args=a.parse_args()
 plan=json.loads((P/'logs'/('BATCH_PLAN_'+args.dataset+'_'+args.arm+'_'+args.tag+'.json')).read_text())['runs']
 config=json.loads((P/'input/DATASETS.json').read_text())['datasets'][args.dataset]
 # Phase1: no reference pose access. Every online file is immutable after supervision.
 seal=[]
 for run in plan:
  d=Path(run['path']);receipt=json.loads((d/'RUN_RECEIPT.json').read_text());assert receipt['input_completed']
  online=['publisher_receipts.csv','localization_trajectory.csv','r23_ndt_lineage.csv','r23_admission.csv','r23_candidates.csv','r29_schedule.csv','oosm_statistics.csv','ekf_prediction_diagnostics.csv','local_map_diagnostics.csv','ndt_diagnostics.csv','ndt_runtime.csv','RUN_RECEIPT.json','RUN_COMMAND.json']
  for name in online:
   f=d/name
   if f.exists():seal.append({'path':str(f),'bytes':f.stat().st_size,'sha256':sha(f)})
  sf=d/'ONLINE_OUTPUT_SEAL.json';assert not sf.exists();sf.write_text(json.dumps({'files':[x for x in seal if Path(x['path']).parent==d],'reference_pose_access':'NONE_DURING_SEAL'},indent=2)+'\n')
 (P/'logs'/('ONLINE_BATCH_SEAL_'+args.dataset+'_'+args.arm+'.json')).write_text(json.dumps(seal,indent=2)+'\n')
 print('ONLINE_OUTPUTS_SEALED_BEFORE_REFERENCE',flush=True)
 # Phase2: offline reference only.
 ref=Reference(config);summaries=[];events=[];output=P/'FULL_SEQUENCE_TRAJECTORIES';output.mkdir(exist_ok=True)
 for run in plan:
  d=Path(run['path']);rr=json.loads((d/'RUN_RECEIPT.json').read_text());receipts=rows(d/'publisher_receipts.csv');assert len(receipts)==config['scan_count']
  traj=rows(d/'localization_trajectory.csv');idx=defaultdict(list)
  for r in traj:idx[(r['source'],int(r['stamp_ns']))].append(r)
  corrected=sorted({int(r['stamp_ns']) for r in traj if r['source']=='EKF_CORRECTED'})
  lineage={int(r['scan_ns']):r for r in rows(d/'r23_ndt_lineage.csv')}
  admissions={int(r['scan_ns']):r for r in rows(d/'r23_admission.csv')};candidates=rows(d/'r23_candidates.csv');candidate_index=defaultdict(list)
  for r in candidates:candidate_index[int(r['scan_ns'])].append(r)
  schedule=rows(d/'r29_schedule.csv');assert len(schedule)==len(receipts)
  blocks=defaultdict(int);frame_index={int(x['scan_ns']):int(x['frame_one_based']) for x in schedule}
  for r in admissions.values():blocks[(frame_index[int(r['scan_ns'])]-1)//60]+=int(r['extra_aligns'])
  assert all(n<=12 for n in blocks.values()) and all(int(x['extra_aligns'])<=2 for x in admissions.values())
  result=[];native_et=[];native_er=[];rpe_t=[];rpe_r=[];ts=[];estimates=[];refs=[];harmful=missed=0
  for row in receipts:
   scan=int(row['scan_stamp_ns']);stamp=int(row['ekf_imu_watermark_ns']);begin=float(row['wall_publish_sec']);ack=num(row['wall_ekf_ack_sec']);applied=flag(row['ekf_correction_ack']);published=flag(row['ndt_measurement_published'])
   if applied:
    lo=bisect.bisect_left(corrected,stamp-1000);hi=bisect.bisect_right(corrected,stamp+1000)
    match=[r for t in corrected[lo:hi] for r in idx[('EKF_CORRECTED',t)] if begin<=float(r['receipt_wall_sec'])<=ack]
   else:match=[r for r in idx[('EKF_HIGH_RATE',stamp)] if float(r['receipt_wall_sec'])<=begin]
   assert len(match)==1,(run,scan,len(match))
   state=match[0];assert state['frame_id']=='benchmark_map' and state['child_frame_id']=='benchmark_lidar'
   est=pose(state);truth=ref.at(int(state['stamp_ns']));et,er=error(est,truth);ts.append(int(state['stamp_ns']));estimates.append(est);refs.append(truth)
   li=lineage[scan];ad=admissions[scan];ne_t,ne_r=error(mat(li,'raw'),ref.at(scan));native_et.append(ne_t);native_er.append(ne_r)
   pred=mat(li,'tp');pt,pr=error(pred,ref.at(scan));selected_id=int(ad['selected_id']);selected=next((c for c in candidate_index[scan] if int(c['id'])==selected_id),None)
   se_t,se_r=error(mat(selected,'pose'),ref.at(scan)) if selected else (None,None)
   is_harm=bool(published and se_t is not None and pt is not None and ((se_t>.5 and se_t-pt>.03) or (se_r>5 and se_r-pr>.3)))
   beneficial=bool(not published and pt is not None and any((lambda e:e[0] is not None and e[0]<=pt+1e-9 and e[1]<=pr+1e-9 and (pt-e[0]>.03 or pr-e[1]>.3))(error(mat(c,'pose'),ref.at(scan))) for c in candidate_index[scan] if flag(c['finite']) and flag(c['converged'])))
   harmful+=is_harm;missed+=beneficial
   rec={'dataset':run['dataset'],'arm':run['arm'],'mode':run['mode'],'scan_index':int(row['scan_index']),'raw_ordinal':config['anchor_raw_ordinal']+int(row['scan_index']),'scan_stamp_ns':scan,'state_stamp_ns':int(state['stamp_ns']),'state_source':state['source'],'map_published':published,'ekf_update_applied':applied,'nominal_attempted':flag(ad['nominal_attempted']),'extra_calls':int(ad['extra_aligns']),'reason':ad['reason'],'translation_error_m':et,'rotation_error_deg':er,'reference_valid':truth is not None,'reference_high_error':(et>.5 or er>5) if et is not None else None,'harmful_admission_proxy':is_harm,'missed_beneficial_proxy':beneficial,**{k:state[k] for k in ['px','py','pz','qx','qy','qz','qw']}}
   result.append(rec)
   if flag(ad['quality_event']) or int(ad['extra_aligns']) or selected_id>0:events.append({k:rec[k] for k in ['dataset','arm','mode','raw_ordinal','scan_stamp_ns','extra_calls','map_published','ekf_update_applied','reason','translation_error_m','rotation_error_deg','reference_high_error']})
  # Relative pose error over one actual sensor second, no trajectory fitting.
  for i,t in enumerate(ts):
   j=bisect.bisect_left(ts,t+10**9)
   options=[k for k in [j-1,j] if i<k<len(ts) and abs(ts[k]-t-10**9)<=50000000]
   if not options:continue
   j=min(options,key=lambda k:abs(ts[k]-t-10**9))
   if any(x is None for x in [estimates[i],estimates[j],refs[i],refs[j]]):continue
   delta_est=np.linalg.inv(estimates[i])@estimates[j];delta_ref=np.linalg.inv(refs[i])@refs[j]
   e=np.linalg.inv(delta_ref)@delta_est;rpe_t.append(float(np.linalg.norm(e[:3,3])));rpe_r.append(float(Rotation.from_matrix(e[:3,:3]).magnitude()*180/math.pi))
  write(output/(run['dataset']+'_'+run['arm']+'_'+run['mode']+'.csv'),result)
  no_map=[not r['map_published'] for r in result];best_time=0.;begin_none=None
  for r in result:
   if not r['map_published']:begin_none=begin_none if begin_none is not None else r['scan_stamp_ns'];best_time=max(best_time,(r['scan_stamp_ns']-begin_none)/1e9)
   else:begin_none=None
  high=[bool(r['reference_high_error']) for r in result];episodes=reference_episodes(result)
  cb=[num(r['callback_wall_ms']) for r in lineage.values()];event_times=[num(lineage[s]['callback_wall_ms']) for s,r in admissions.items() if flag(r['quality_event'])]
  conv=sum(flag(r['converged']) for r in candidates);extra=sum(int(r['extra_aligns']) for r in admissions.values());oosm=rows(d/'oosm_statistics.csv')
  summary={'dataset':run['dataset'],'arm':run['arm'],'mode':run['mode'],'status':'COMPLETE_ADMITTED_SPAN','raw_total':config['raw_total'],'anchor_raw':config['anchor_raw_ordinal'],'evaluated_scans':len(result),'reference_n':sum(r['reference_valid'] for r in result),'coverage_fraction':len(result)/(config['raw_total']-1),'finite_estimates':sum(x is not None for x in estimates),'translation':stats([r['translation_error_m'] for r in result]),'rotation':stats([r['rotation_error_deg'] for r in result]),'ATE_definition':'unaligned map-frame position RMSE','RPE_1s_translation':stats(rpe_t),'RPE_1s_rotation':stats(rpe_r),'raw_native_translation':stats(native_et),'raw_native_rotation':stats(native_er),'nominal_calls':sum(flag(r['nominal_attempted']) for r in admissions.values()),'extra_calls':extra,'native_calls_total':len(candidates),'native_converged_total':conv,'published_map_updates':sum(r['map_published'] for r in result),'rejected_map_observations':sum(no_map),'actual_ekf_updates':sum(r['ekf_update_applied'] for r in result),'oosm_statuses':{s:sum(r['oosm_result']==s for r in oosm) for s in {r['oosm_result'] for r in oosm}},'longest_no_map_frames':longest(no_map),'longest_no_map_span_sec':best_time,'reference_high_error_frames':sum(high),'sustained_reference_episodes':episodes,'harmful_admission_proxy':harmful,'missed_beneficial_proxy':missed,'callback_ms':stats(cb),'event_callback_ms':stats(event_times),'wall_seconds':rr['wall_seconds'],'processing_scans_per_wall_sec':len(result)/rr['wall_seconds'],'sensor_span_sec':(config['frames'][-1]['stamp_ns']-config['anchor_stamp_ns'])/1e9,'ndt_peak_rss_KiB':rr['ndt_peak_rss_KiB'],'ekf_peak_rss_KiB':rr['ekf_peak_rss_KiB'],'simultaneous_ndt_ekf_peak_rss_KiB':rr['simultaneous_ndt_ekf_peak_rss_KiB'],'target_points':stats([num(r['target_points']) for r in lineage.values()]),'target_hash_switches':sum(a['target_hash']!=b['target_hash'] for a,b in zip(list(lineage.values()),list(lineage.values())[1:])),'max_extra_in_60frame_block':max(blocks.values()),'nominal_basic_quality_passes':sum(flag(r['basic_quality']) for r in admissions.values()),'nominal_absolute_passes':sum(flag(r['absolute_pass']) for r in admissions.values())}
  assert len(candidates)==summary['nominal_calls']+extra
  summaries.append(summary);print(run['mode'],'t/r RMSE',summary['translation']['rmse'],summary['rotation']['rmse'],'map updates',summary['published_map_updates'],flush=True)
  # Check raw output seal again after reference evaluation; it must not change.
  for f in json.loads((d/'ONLINE_OUTPUT_SEAL.json').read_text())['files']:assert sha(f['path'])==f['sha256']
 (P/('RESULT_'+args.dataset+'_'+args.arm+'.json')).write_text(json.dumps(summaries,indent=2,allow_nan=False)+'\n')
 write(P/('EVENTS_'+args.dataset+'_'+args.arm+'.csv'),events)
 print('OFFLINE_EVALUATION_COMPLETE',flush=True)

if __name__=='__main__':main()
