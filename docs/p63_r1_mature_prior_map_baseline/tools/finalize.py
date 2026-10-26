#!/usr/bin/env python3
"""Archive the failed smoke as recorded; no new estimator or reference evaluation."""
import csv,json,hashlib,gzip,shutil,subprocess,sys
from pathlib import Path
import numpy as np
P=Path(__file__).resolve().parents[1];R=P.parents[1];U=Path('/home/jian/livox_ws/p63_r1_super_lio_data/Super-LIO');D=U.parent/'loop2_smoke_01'
def sha(f):return hashlib.sha256(Path(f).read_bytes()).hexdigest()
def stat(a):
 a=np.asarray(a,dtype=float);a=a[np.isfinite(a)]
 return {'n':int(len(a)),'mean':float(a.mean()) if len(a) else None,'p95':float(np.percentile(a,95)) if len(a) else None,'max':float(a.max()) if len(a) else None}
rows=list(csv.DictReader((D/'ORIGINAL_TRAJECTORY.csv').open()));node=json.loads((D/'OBSERVER_RECEIPT.json').read_text());run=json.loads((D/'RUN_RECEIPT.json').read_text());input_=json.loads((D/'INPUT_RECEIPT.json').read_text());g=json.loads((P/'evidence/INITIALIZATION_GRAVITY_DIAGNOSTIC.json').read_text());contract=json.loads((P/'config/SENSOR_CONTRACT.json').read_text())
assert len(rows)==node['tracked_outputs']==95 and input_['counts']['/livox/lidar']==107
assert node['map_frozen'] and node['map_before_sha256']==node['map_after_sha256']
assert node['tracking_failure_stop'] and all(int(x['plane_matches'])<4 for x in rows[-20:])
for x in rows:assert np.isfinite([float(x[k]) for k in ['px','py','pz','qx','qy','qz','qw','vx','vy','vz']]).all()
result={'task':'P63-R1-ESTABLISH-MATURE-PRIOR-MAP-BASELINE','START_SHA':'3f0d24f1a930a2365e7c918c8444afc44c039998','author_ros1_SHA':'60b57aaac8dc397f80c56364e7ccb008c300cc29','author_tracked_patch_count':0,'run_harness_CODE_SHA':'5ae6d0c600e1140b996e83a237d81cffb956969d','END_SHA':'final containing archive commit;full SHA in handoff','branch':'research/p63-r1-mature-prior-map-baseline','push_executed':False,'source_and_release':'PASS_LOCAL_ENV_ISOLATION','external_PCD_import':'PASS','frozen_prior_map':'PASS','sensor_frame_contract':'CONDITIONAL_USER_AUTHORIZED_HISTORICAL_IDENTITY','physical_calibration':'NOT_AVAILABLE','original_tracking':'SMOKE_TEST_FAILED_EARLY','research_backbone':'NOT_YET_ACCEPTED','requested_scans':400,'published_sensor_inputs':input_['counts'],'tracked_outputs':len(rows),'outputs_with_at_least_4_plane_matches':sum(int(x['plane_matches'])>=4 for x in rows),'consecutive_unsupported_at_stop':20,'first_unsupported_tracking_output_one_based':next(i+1 for i,x in enumerate(rows) if int(x['plane_matches'])<4),'native_initialization_NDT_calls':1,'native_initialization_ICP_calls':1,'ICP_fitness':.10959,'ICP_score_is_pose_error':False,'native_iterated_ESKF_observe_calls':95,'full_tracking_duration_sensor_s':float(rows[-1]['stamp_s'])-float(rows[0]['stamp_s']),'first_pose':[float(rows[0][k]) for k in ['px','py','pz']],'last_pose':[float(rows[-1][k]) for k in ['px','py','pz']],'estimated_displacement_not_error_m':float(np.linalg.norm([float(rows[-1][k])-float(rows[0][k]) for k in ['px','py','pz']])),'process_ms':stat([float(x['process_ms']) for x in rows]),'spin_plus_process_ms':stat([float(x['spin_and_process_ms']) for x in rows]),'plane_matches':stat([int(x['plane_matches']) for x in rows]),'final_cached_plane_rms_m':stat([float(x['final_cached_plane_rms']) for x in rows]),'initialization_NDT_plus_ICP_log_interval_s':3.289708,'gravity_disagreement_diagnostic_deg':g['angle_to_author_positive_map_Z_deg'],'observer':node,'supervision':run,'actual_input_receipt':input_,'formal_ATE_RPE':'NOT_EVALUABLE_NO_INDEPENDENT_GT_UNQUALIFIED_EXTRINSIC','RViz':'STARTED_DURING_FAILED_SMOKE;NO_CAPTURE_BEFORE_EARLY_STOP;ONECLICK_COMPOSITION_PARSE_ONLY','unique_NEXT':'Controller to commission one bounded external-map gravity/initialization frame closure using independent mapping provenance;then decide a frame adapter or framework switch.No new recovery or repeated smoke in this task.'}
(P/'results.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
shutil.copyfile(D/'ORIGINAL_TRAJECTORY.csv',P/'ORIGINAL_TRAJECTORY.csv')
manifest=[];raw=P/'raw';raw.mkdir(exist_ok=True)
for directory in [D,U.parent/'map_probe_01']:
 dest=raw/directory.name;dest.mkdir(exist_ok=True)
 for f in directory.iterdir():
  if not f.is_file() or f.suffix not in ['.csv','.json','.log']:continue
  b=f.read_bytes()
  if f.name=='ROSLAUNCH.log':assert not any(x in b for x in [b'local environment:',b'API_KEY=',b'api_key='])
  z=dest/(f.name+'.gz');z.write_bytes(gzip.compress(b,mtime=0));assert gzip.decompress(z.read_bytes())==b
  manifest.append({'path':str(f),'sha256':sha(f),'bytes':len(b),'portable':str(z.relative_to(P)),'portable_sha256':sha(z)})
(P/'evidence/RAW_OUTPUT_MANIFEST.json').write_text(json.dumps(manifest,indent=2)+'\n')
up=P/'upstream_source'
for source in ['LICENSE','README.md','src/super_lio/src/lio/super_lio_reloc.cpp','src/super_lio/src/lio/super_lio.cpp','src/super_lio/src/lio/ESKF.cpp','src/super_lio/src/ros/ROSWrapper.cpp','src/super_lio/include/lio/super_lio_reloc.h','src/super_lio/include/OctVoxMap/OctVoxMap.hpp']:
 target=up/source;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(U/source,target)
old=Path('/home/jian/livox_ws/p62_r32_nonmutating_observation_ws/docs/p62_r32_nonmutating_observation')
(P/'evidence/R32_ACTUAL_COMPLETION.json').write_text(json.dumps({'actual_R32_END':'66494205f7dd48d193d08936f10784f49f653939','archive_path':str(old),'source_sha256':sha(old/'results.json'),'status':'completed;not unexecuted as older planning text said','actual_metrics':{'indoor_translation_rmse_m':5440.6130295998455,'indoor_rotation_rmse_deg':27.34121035504169,'native_calls':806,'actual_EKF_updates':67},'original_branch_preserved':True},indent=2)+'\n')
print(json.dumps({k:result[k] for k in ['original_tracking','tracked_outputs','process_ms','spin_plus_process_ms','gravity_disagreement_diagnostic_deg','estimated_displacement_not_error_m']},indent=2))
