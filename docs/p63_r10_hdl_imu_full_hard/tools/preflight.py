"""Full raw input verification only; zero NDT, no evaluation pose parsing."""
import bisect,csv,hashlib,json,shutil,subprocess,time
from decimal import Decimal
from pathlib import Path
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores,get_typestore
from data import sha,pcd_xyz
from imu_geometry import convert
A=Path(__file__).resolve().parents[1];W=A.parents[1];R9=W/'docs/p63_r9_hdl_real_imu_prediction'
ROOT=Path('/home/jian/rosbag/public_datasets/koide_p62_r29');begin_wall=time.monotonic()
def dump(name,x):(A/name).write_text(json.dumps(x,indent=2)+'\n')
def write(name,r):
 with (A/name).open('w',newline='') as s:
  w=csv.DictWriter(s,fieldnames=list(r[0]));w.writeheader();w.writerows(r)
def stats(v):
 a=np.asarray(v,float);return dict(n=len(a),min=float(a.min()),mean=float(a.mean()),p95=float(np.percentile(a,95)),max=float(a.max()))
assert shutil.disk_usage(W).free>=20*1024**3
assert sha(A/'tools/imu_geometry.py')==sha(R9/'tools/imu_geometry.py')
B=json.loads((R9/'BUILD.json').read_text());F=json.loads((R9/'INPUT_FREEZE.json').read_text())
assert sha(F['map_path'])==F['map_sha256']
for p,v in B['native_binaries'].items():assert sha(p)==v['sha256']
assert sha(B['trace_binary'])==B['trace_binary_sha256'] and sha(B['trace_source'])==B['trace_source_sha256']
manifest=ROOT/'indoor_hard_01_adapter/manifest.json';M=json.loads(manifest.read_text());frames=M['frames']
assert len(frames)==2379 and [f['raw_ordinal'] for f in frames]==list(range(1,2380))
assert all(a['stamp_ns']<b['stamp_ns'] for a,b in zip(frames,frames[1:]))
imu_csv=Path(M['imu_csv']);imus=list(csv.DictReader(imu_csv.open()));times=np.array([int(r['sec'])*10**9+int(r['nsec']) for r in imus],dtype=np.int64)
assert len(times)==44398 and np.all(np.diff(times)>0)
store=get_typestore(Stores.ROS2_HUMBLE);hashes={k:hashlib.sha256() for k in ['/imu','/points2/decompressed']};count={k:0 for k in hashes};imu_hash=hashlib.sha256();verified=[]
with Reader(Path(M['source_metadata']).parent) as reader:
 connections=[c for c in reader.connections if c.topic in hashes]
 assert {c.topic:c.msgcount for c in connections}==M['selected_message_counts']=={'/imu':44398,'/points2/decompressed':2379}
 for connection,bagtime,cdr in reader.messages(connections=connections):
  topic=connection.topic;i=count[topic];count[topic]+=1;hashes[topic].update(cdr)
  msg=store.deserialize_cdr(cdr,connection.msgtype);ns=int(msg.header.stamp.sec)*10**9+int(msg.header.stamp.nanosec)
  if topic=='/imu':
   vals=[getattr(msg.angular_velocity,k) for k in 'xyz']+[getattr(msg.linear_acceleration,k) for k in 'xyz']
   assert msg.header.frame_id=='imu_link' and ns==times[i] and np.isfinite(vals).all()
   assert np.array_equal(vals,[float(imus[i][k]) for k in ['gx','gy','gz','ax','ay','az']])
   imu_hash.update(np.array([ns],dtype='<i8').tobytes()+np.array(vals,dtype='<f8').tobytes())
   verified.append({'id':i,'stamp_ns':ns,'bag_stamp_ns':int(bagtime),'cdr_sha256':hashlib.sha256(cdr).hexdigest(),**dict(zip(['gx','gy','gz','ax','ay','az'],vals))})
  else:
   f=frames[i];assert ns==f['stamp_ns'] and int(bagtime)==f['bag_stamp_ns'] and msg.header.frame_id==f['frame_id']=='depth_camera_link'
   fields={v.name:v for v in msg.fields};assert not msg.is_bigendian and not any(k in fields for k in ['t','time','timestamp'])
   assert all(fields[k].datatype==7 and fields[k].count==1 for k in ['x','y','z'])
   dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[fields[k].offset for k in ['x','y','z']],'itemsize':msg.point_step})
   view=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step))
   xyz=np.column_stack([view[k].reshape(-1) for k in ['x','y','z']]);converted,_=pcd_xyz(f['path'])
   assert sha(f['path'])==f['sha256'] and xyz.tobytes()==converted.tobytes() and np.isfinite(xyz).all() and len(xyz)==f['points']
   f['xyz_sha256']=hashlib.sha256(xyz.tobytes()).hexdigest()
   if count[topic]%400==0:print('RAW_CHECK',count[topic],flush=True)
assert count==M['selected_message_counts'] and {k:h.hexdigest() for k,h in hashes.items()}==M['cdr_topic_sha256'] and imu_hash.hexdigest()==M['imu_value_sha256']
write('evidence/RAW_IMU_VERIFIED.csv',verified)
F['frames']=frames[4:];F['raw_last']=2379;F['evaluation_count']=2375;F['dataset_total_clouds']=2379;F['use_imu']=True;F.pop('candidate_raws',None)
assert F['anchor_stamp_ns']==frames[3]['stamp_ns'];anchor=F['anchor_stamp_ns'];end=frames[-1]['stamp_ns'];idx=int(np.searchsorted(times,anchor,side='right'));last=int(np.searchsorted(times,end,side='right'));start=idx
assert 0<idx<last<len(times) and times[0]<anchor and times[-1]>=end
prev=None;init=None;last_scan=anchor;timeline=[];preview=[]
for f in F['frames']:
 s=f['stamp_ns'];batch=[];integrated=[];overlap=0;maxdt=0
 while idx<last and times[idx]<=s:
  ts=int(times[idx]);dt=(ts-int(times[idx-1]))/1e9;ir=imus[idx];old=imus[idx-1]
  af,wf,lever=convert([float(ir[k]) for k in ['ax','ay','az']],[float(ir[k]) for k in ['gx','gy','gz']],[float(old[k]) for k in ['gx','gy','gz']],dt)
  preview.append({'id':idx,'stamp_ns':ts,'lever_norm_m_s2':float(np.linalg.norm(lever))})
  if init is None:init=ts
  if prev is not None and ts-init>=2*10**9:
   integrated.append((ts-prev)/1e9);overlap+=max(0,min(ts,last_scan)-prev)
  prev=ts;batch.append(ts);maxdt=max(maxdt,dt);idx+=1
 timeline.append({'raw_ordinal':f['raw_ordinal'],'scan_stamp_ns':s,'prior_correction_stamp_ns':last_scan,'scheduled_imu_calls':len(batch),'model_active_predict_calls':len(integrated),'last_imu_stamp_ns':prev,'scan_minus_last_imu_ns':s-prev,'model_integrated_s':sum(integrated),'model_post_correction_overlap_ns':overlap,'max_consumed_imu_gap_s':maxdt})
 last_scan=s
assert idx==last
write('evidence/NATIVE_TIME_PREFLIGHT.csv',timeline)
gaps=[{'previous_stamp_ns':int(times[i-1]),'stamp_ns':int(times[i]),'dt_s':(int(times[i])-int(times[i-1]))/1e9} for i in range(1,len(times)) if times[i]-times[i-1]>50_000_000]
write('evidence/IMU_GAPS_OVER_50MS.csv',gaps)
# Read only first token (timestamp) of reference rows. Pose values are not parsed.
ref=json.loads((A/'REFERENCE_METADATA.json').read_text());assert sha(ref['reference_path'])==ref['reference_sha256']
rts=[]
with Path(ref['reference_path']).open() as stream:
 for line in stream:
  if line.strip() and not line.startswith('#'):rts.append(int(Decimal(line.split(maxsplit=1)[0])*10**9))
assert all(a<b for a,b in zip(rts,rts[1:]))
def time_ok(ts,seq):
 i=bisect.bisect_left(seq,ts)
 if i==0:a=b=0
 elif i==len(seq):a=b=len(seq)-1
 elif seq[i]==ts:a=b=i
 else:a,b=i-1,i
 return seq[b]-seq[a]<=150_000_000 and min(abs(ts-seq[a]),abs(ts-seq[b]))<=50_000_000,a,b
coverage=[];scan_times=[f['stamp_ns'] for f in F['frames']];pairs=[]
for f in F['frames']:
 ok,a,b=time_ok(f['stamp_ns'],rts);coverage.append({'raw_ordinal':f['raw_ordinal'],'reference_time_supported':ok,'a_ns':rts[a],'b_ns':rts[b]})
 target=f['stamp_ns']+1_000_000_000
 if target>scan_times[-1]:continue
 ok,a,b=time_ok(target,scan_times)
 if not ok:continue
 pairs.append({'start_raw':f['raw_ordinal'],'start_ns':f['stamp_ns'],'end_ns':target,'end_bracket_raw_a':F['frames'][a]['raw_ordinal'],'end_bracket_raw_b':F['frames'][b]['raw_ordinal'],'end_weight':(target-scan_times[a])/(scan_times[b]-scan_times[a]) if a!=b else 0,'reference_time_supported':time_ok(f['stamp_ns'],rts)[0] and time_ok(target,rts)[0]})
write('REFERENCE_TIME_COVERAGE.csv',coverage);write('RPE_PAIRS.csv',pairs)
dump('INPUT_FREEZE.json',F)
P={'stage':'ZERO_NDT_FULL_INPUT_PASS_PENDING_NATIVE_PREPROCESS','NDT_calls':0,'test_reference_pose_values_read':False,'raw_count':count,'cdr_sha256':{k:h.hexdigest() for k,h in hashes.items()},'imu_value_sha256':imu_hash.hexdigest(),'adapter_manifest_sha256':sha(manifest),'imu_csv_sha256':sha(imu_csv),'raw_first_stamp_ns':frames[0]['stamp_ns'],'raw_last_stamp_ns':end,'evaluation_raw':[5,2379],'evaluation_count':2375,'evaluation_span_s':(end-frames[4]['stamp_ns'])/1e9,'anchor_to_last_s':(end-anchor)/1e9,'imu_first_ns':int(times[0]),'imu_last_ns':int(times[-1]),'scheduled_imu':last-start,'model_active_predict':sum(x['model_active_predict_calls'] for x in timeline),'model_skipped_predict':last-start-sum(x['model_active_predict_calls'] for x in timeline),'imu_full_dt_ms':stats(np.diff(times)/1e6),'imu_gaps_gt50ms':len(gaps),'scan_imu_age_ms':stats([x['scan_minus_last_imu_ns']/1e6 for x in timeline]),'zero_new_imu_raws':[x['raw_ordinal'] for x in timeline if not x['scheduled_imu_calls']],'zero_active_predict_raws':[x['raw_ordinal'] for x in timeline if not x['model_active_predict_calls']],'causal_lever_correction_m_s2':stats([x['lever_norm_m_s2'] for x in preview]),'reference_rows':len(rts),'reference_time_supported':sum(x['reference_time_supported'] for x in coverage),'reference_coverage_fraction':sum(x['reference_time_supported'] for x in coverage)/2375,'reference_time_first_last_ns':[rts[0],rts[-1]],'rpe_pairs':len(pairs),'rpe_reference_time_supported':sum(x['reference_time_supported'] for x in pairs),'r9_adapter_sha256':sha(A/'tools/imu_geometry.py'),'map_file_sha256':sha(F['map_path']),'free_bytes':shutil.disk_usage(W).free,'elapsed_s':time.monotonic()-begin_wall}
dump('PREFLIGHT.json',P);print(json.dumps(P,indent=2),flush=True)
