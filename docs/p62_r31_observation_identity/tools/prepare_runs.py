#!/usr/bin/env python3
"""Freeze existing inputs, no evaluation reference pose parsing."""
import hashlib,json,shutil,importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
P=Path(__file__).resolve().parents[1];R=P.parents[1]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
old=R/'docs/p62_r29_full_sequence_benchmark';easy=json.loads((old/'input/DATASETS.json').read_text())['datasets']['EASY']
for label in ['C0','C3']:
 x=easy['initializations'][label];assert sha(x['path'])==x['sha256']
assert sha(easy['runtime_config'])==easy['runtime_sha256']
contract={'datasets':{'EASY':easy},'reference_pose_values_read':False,'strict_policy':'STRICT_FAIL_CLOSED'}
r23=R/'docs/p62_r23_real_imu_closure';old_nc=json.loads((r23/'input/INITIALIZATION_AND_TIME_CONTRACT.json').read_text());w=old_nc['windows']['W1'];r22=json.loads((R/'docs/p62_r22_frozen_relative_holdout/WINDOW_IDENTITY.json').read_text())
try:
 runtime=R/'src/dog_prior_map_localization/config/p41_newer_college_runtime.yaml';assert sha(runtime)==old_nc['runtime_config_sha256']
 assert sha(old_nc['imu_source']['path'])==old_nc['imu_source']['sha256']
 assert sha(old_nc['map']['tile_manifest_path'])==old_nc['map']['tile_manifest_sha256']
 frames=[x for x in w['frames'] if x['role']=='EVALUATION'];assert len(frames)==60
 for f in frames:assert sha(f['path'])==f['pcd_sha256']
 initials={}
 for arm in ['C0','C3']:
  src=w['initializations'][arm];assert sha(src['path'])==src['sha256']
  dest=P/'input'/('NC_W1_'+arm+'_initialization.yaml');shutil.copyfile(src['path'],dest)
  initials[arm]={**src,'path':str(dest)}
 nc={'anchor_raw_ordinal':2600,'scan_count':60,'raw_total':15301,'frames':frames,
  'publisher_start_stamp_ns':w['publisher_start_stamp_ns'],'imu_replay_start_stamp_ns':w['imu_replay_start_stamp_ns'],
  'expected_boundary_imu_stamp_ns':w['expected_boundary_imu_stamp_ns'],'scan_directory':str(Path(frames[0]['path']).parent),
  'imu_csv':old_nc['imu_source']['path'],'tile_manifest':old_nc['map']['tile_manifest_path'],
  'runtime_config':str(runtime),'runtime_sha256':sha(runtime),'initializations':initials,'instantaneous_scan':False,
  'reference_path':'/home/jian/livox_ws/p59_r3_newer_college_official_internal_recovery_data/01_short_experiment/ground_truth/registered_poses.csv',
  'reference_sha256':r22['reference_sha256'],'reference_to_lidar':r22['T_B_L'],'coordinate_status':'CONDITIONAL_BASE_MAP_ANCHORED_REFERENCE'}
 contract['datasets']['NC_W1']=nc;contract['NC_regression_status']='INPUT_IDENTITIES_PASS_CONDITIONAL_REFERENCE'
except (AssertionError,FileNotFoundError) as e:
 contract['NC_regression_status']='BLOCKED_INPUT_CONTRACT: '+str(e)
(P/'input/DATASETS.json').write_text(json.dumps(contract,indent=2)+'\n')
spec=importlib.util.spec_from_file_location('snap',r23/'tools/prepare_inputs.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
(P/'evidence/STABLE_BEFORE.json').write_text(json.dumps(m.stable_snapshot(),indent=2)+'\n')
print(contract['NC_regression_status'])
