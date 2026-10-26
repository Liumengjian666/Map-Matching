#!/usr/bin/env python3
"""Finalize the single completed trial; no align or filter update, no reference data."""
from pathlib import Path
import csv,json,numpy as np,hashlib,gzip,io
import rosbag
A=Path(__file__).resolve().parents[1];D=Path('/home/jian/livox_ws/p63_r3_hdl_data/trial_01')
r=json.loads((D/'RUN_RECEIPT.json').read_text());n=list(csv.DictReader((D/'NATIVE_NDT.csv').open()));c=list(csv.DictReader((D/'CALLBACKS.csv').open()));b=list(csv.DictReader((D/'ADAPTER.csv').open()));identity=json.loads((A/'WINDOW_IDENTITY.json').read_text())
assert len(n)==len(c)==len(b)==len(r['records'])==400
for x,y,z,f in zip(n,c,b,identity['frames']):
 assert int(x['stamp_ns'])==int(y['stamp_ns'])==int(z['stamp_ns'])==f['stamp_ns']
 assert z['xyz_sha256']==f['xyz_sha256'] and int(z['points'])==f['points']
 assert x['map_frozen']=='1' and x['target_xyz_sha256']==r['map_messages'][0]['xyz_sha256']
# rospy/topics.py increments publication seq before serialize_message, which replaces Header.seq.
# Verify every full received raw payload differs ONLY by this transport sequence, not point/time data.
seq=[]
with rosbag.Bag(identity['bag'],'r') as bag:
 for i,(_,m,_) in enumerate(bag.read_messages(topics=['/livox/lidar'])):
  if i>=400:break
  original_seq=m.header.seq;m.header.seq=i+1;s=io.BytesIO();m.serialize(s)
  assert hashlib.sha256(s.getvalue()).hexdigest()==b[i]['raw_message_sha256']
  seq.append({'ordinal':i+1,'bag_seq':original_seq,'transport_seq':i+1})
(A/'evidence/HEADER_SEQ_TRANSPORT.json').write_text(json.dumps({'all400_received_payloads_match_after_only_Header_seq_replacement':True,'source':'/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py:1065 and rospy/msg.py:145','original_XYZ_count_stamps_and_all_other_fields_preserved':True,'rows':seq},indent=2)+'\n')
stat=lambda a:{'mean':float(np.mean(a)),'p95':float(np.percentile(a,95)),'max':float(np.max(a)),'min':float(np.min(a))}
summary={'NDT_compute_ms':stat([float(x['ndt_compute_ms']) for x in n]),'callback_ms_including_diagnostics':stat([float(x['callback_ms']) for x in c]),'diagnostics_ms':stat([float(x['diagnostic_ms']) for x in n]),'bridge_ms':stat([float(x['bridge_ms']) for x in b]),'all_point_NN_fitness_m2':stat([float(x['fitness_all_nn_m2']) for x in n]),'native_inlier_fraction':stat([x['inlier_fraction'] for x in r['records']]),'native_inlier_mean_squared_distance_m2':stat([x['native_matching_error'] for x in r['records']]),'translation_steps_m':stat([x['translation_step_m'] for x in r['records']]),'rotation_steps_deg':stat([x['rotation_step_deg'] for x in r['records']]),'iterations':stat([int(x['iterations']) for x in n]),'first_output':r['records'][0],'last_output':r['records'][-1],'all_converged':all(x['converged']=='1' for x in n),'target':r['map_messages'][0],'RSS_KiB_conservative':max(r['manager_peak_RSS_KiB_sampled'],max(int(x['peak_RSS_KiB']) for x in n)),'wall_s':r['wall_s'],'ATE_RPE':'NOT_EVALUABLE_NO_INDEPENDENT_GT','RViz_original_capture':'Noetic warning dialog;not a localization view'}
summary['resource_200MiB_pass']=summary['RSS_KiB_conservative']<=200*1024;summary['callback_p95_under_100ms']=summary['callback_ms_including_diagnostics']['p95']<100
(A/'evidence/MEASURED_SUMMARY.json').write_text(json.dumps(summary,indent=2)+'\n')
(A/'NDT_RESULTS.csv').write_bytes((D/'NATIVE_NDT.csv').read_bytes());(A/'PROCESS_MEMORY.csv').write_bytes((D/'MEMORY.csv').read_bytes())
with (A/'RUNTIME_MEMORY.csv').open('w',newline='') as f:
 w=csv.writer(f,lineterminator='\n');w.writerow(['ordinal','stamp_ns','ndt_compute_ms','callback_ms_instrumented','diagnostic_ms','bridge_ms','manager_getrusage_peak_KiB'])
 for i,(x,y,z) in enumerate(zip(n,c,b),1):w.writerow([i,x['stamp_ns'],x['ndt_compute_ms'],y['callback_ms'],x['diagnostic_ms'],z['bridge_ms'],x['peak_RSS_KiB']])
with (A/'TRAJECTORY.csv').open('w',newline='') as f:
 w=csv.writer(f,lineterminator='\n');w.writerow(['ordinal_1based','stamp_ns','x','y','z','qx','qy','qz','qw','valid_screen','inlier_fraction','translation_step_m','rotation_step_deg'])
 for q in r['records']:w.writerow([q['ordinal_1based'],q['stamp_ns'],*q['output_pose_xyzw'],q['valid_update_screen'],q['inlier_fraction'],q['translation_step_m'],q['rotation_step_deg']])
raw=A/'raw';raw.mkdir(exist_ok=True);manifest=[]
for p in sorted(D.iterdir()):
 if p.is_file():
  payload=p.read_bytes();out=raw/(p.name+'.gz');out.write_bytes(gzip.compress(payload,mtime=0));manifest.append({'source':str(p),'bytes':len(payload),'sha256':hashlib.sha256(payload).hexdigest(),'archive':str(out.relative_to(A)),'archive_sha256':hashlib.sha256(out.read_bytes()).hexdigest()})
(A/'evidence/RAW_MANIFEST.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps({k:summary[k] for k in ['NDT_compute_ms','callback_ms_including_diagnostics','native_inlier_fraction','RSS_KiB_conservative','resource_200MiB_pass','callback_p95_under_100ms']},indent=2))
