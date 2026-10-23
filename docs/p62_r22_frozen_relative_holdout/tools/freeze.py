#!/usr/bin/env python3
import json,csv,re,subprocess,sys
from pathlib import Path
from prepare_inputs import HERE,REPO,DATA,GT,sha,save,snapshot
BUILD=Path('/home/jian/livox_ws/p62_r22_relative_holdout_build')
def rows(p):return list(csv.DictReader(Path(p).open()))
def before():
 prep=json.loads((HERE/'INPUT_PREPARATION.json').read_text());assert snapshot()==prep['stable_before']
 exposure=json.loads((HERE/'HISTORICAL_EXPOSURE.json').read_text());windows={}
 source=DATA/'prior_map/new-college-29-01-2020-5cm-resolution.ply'
 assert sha(source)=='e8bf6c7f2ddacde4f6f65ba7759b00bdca5c6bf5e706344ce8c3592529f04ab0'
 for role,w in prep['windows'].items():
  folder=HERE/role.lower();scans=rows(folder/'SCANS.csv');receipt=rows(folder/'map_preflight_01/MAP_BUILD.csv')[0]
  assert len(scans)==61 and float(receipt['anchor_overlap_0p5m'])>=.5
  for f,r in zip(w['frames'],scans):
   assert str(f['stamp_ns'])==r['stamp_ns'] and f['pcd_sha256']==sha(f['path'])
   f['source_fnv']=r['expected_source_fnv'];f['source_points']=int(r['expected_source_points'])
  map_path=Path('/home/jian/livox_ws/p62_r22_relative_holdout_data')/(role.lower()+'_roi.pcd')
  windows[role]={**w,'history_exposure':exposure[role],'scan_table_sha256':sha(folder/'SCANS.csv'),'map':{'path':str(map_path),'sha256':sha(map_path),'bytes':map_path.stat().st_size,'receipt':receipt,'receipt_sha256':sha(folder/'map_preflight_01/MAP_BUILD.csv')}}
 save(HERE/'WINDOW_IDENTITY.json',{'windows':windows,'T_B_L':prep['T_B_L'],'reference_sha256':prep['reference_sha256'],'evaluation_values_read':False,'official_map_sha256':sha(source),'stable_snapshot':prep['stable_before'],'pre_registration_sha256':sha(HERE/'PRE_REGISTRATION.md')})
 files={}
 def add(p):
  p=Path(p).resolve()
  if str(p) in files:return
  files[str(p)]=sha(p)
  if p.suffix in ('.cpp','.hpp'):
   for x in re.findall(r'^#include "([^"]+)"',p.read_text(),re.M):add(p.parent/x)
 for p in (HERE/'tools').iterdir():
  if p.is_file():add(p)
 for p in [HERE/'PRE_REGISTRATION.md',Path('/usr/include/pcl-1.10/pcl/registration/ndt.h'),Path('/usr/include/pcl-1.10/pcl/registration/impl/ndt.hpp')]:add(p)
 old=json.loads((REPO/'docs/p62_r19_native_ndt_basin_stress_test/NDT_INPUT_FREEZE.json').read_text())
 for f,h in old['r19_code_and_frozen_headers_sha256'].items():assert sha(REPO/f)==h
 for f in ['docs/p62_r20_quality_gated_recovery/tools/policy.hpp','docs/p62_r21_relative_temporal_admission/tools/policy.hpp']:
  assert sha(REPO/f)==__import__('hashlib').sha256(subprocess.check_output(['git','show','cfeb7288ce0a5cf00ae58d94ece26cb745a9b57c:'+f],cwd=REPO)).hexdigest()
 save(HERE/'ALGORITHM_FREEZE.json',{'start_sha':'cfeb7288ce0a5cf00ae58d94ece26cb745a9b57c','code_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip(),'files_sha256':files,'binary_path':str(BUILD/'run_holdout'),'binary_sha256':sha(BUILD/'run_holdout'),'release':True,'window_identity_sha256':sha(HERE/'WINDOW_IDENTITY.json'),'max_nominal':1920,'max_extra':192,'max_total':2112,'aligns_at_freeze':0,'method_semantics_changed':False})
 print('R22_INPUT_AND_ALGORITHM_FREEZE_PASS; align=0')
def after():
 a=json.loads((HERE/'ALGORITHM_FREEZE.json').read_text());w=json.loads((HERE/'WINDOW_IDENTITY.json').read_text());assert sha(HERE/'WINDOW_IDENTITY.json')==a['window_identity_sha256']
 for f,h in a['files_sha256'].items():assert sha(f)==h,f
 assert sha(a['binary_path'])==a['binary_sha256'] and snapshot()==w['stable_snapshot']
 output={};nom=extra=0
 for role,win in w['windows'].items():
  folder=HERE/role.lower()/'formal_run_01';calls=rows(folder/'ALIGN_CALLS.csv');causal=rows(folder/'CAUSAL_RESULTS.csv');candidates=rows(folder/'CANDIDATE_AND_ADMISSION_LOG.csv')
  assert len(causal)==480 and len(calls)==len(candidates)
  for r in causal:
   f=win['frames'][int(r['frame'])];assert r['source_fnv']==f['source_fnv'] and int(r['stamp_ns'])==f['stamp_ns'] and int(r['raw_ordinal'])==f['raw_ordinal']
   assert int(r['extra_aligns']) in (0,2) and int(r['budget_after'])<=12
   assert int(r['history_last_stamp_ns'])<int(r['stamp_ns'])
   assert r['history_after_stamp_ns']==(r['stamp_ns'] if r['accepted_observation']=='1' else r['history_last_stamp_ns'])
   if r['nominal_gate_accepted']=='1' or r['policy'] in ('BASELINE_NATIVE_NDT','QUALITY_GATE_ONLY'):assert r['extra_aligns']=='0'
  for arm in ['C0','C3']:
   for policy in ['BASELINE_NATIVE_NDT','QUALITY_GATE_ONLY','QUALITY_TRIGGERED_MULTISTART','SINGLE_FRAME_RELATIVE_ADMISSION']:
    rs=[r for r in causal if r['arm']==arm and r['policy']==policy];assert len(rs)==60 and sum(int(r['extra_aligns']) for r in rs)<=12
  nom+=sum(r['kind']=='NOMINAL_FULL' for r in calls);extra+=sum(r['kind']=='EXTRA_FULL' for r in calls)
  assert sha(win['map']['path'])==win['map']['sha256']
  for f in win['frames']:assert sha(f['path'])==f['pcd_sha256']
  for p in folder.iterdir():
   if p.is_file():output[str(p.relative_to(HERE))]=sha(p)
  log=HERE/role.lower()/'FORMAL_RUN_01.log';assert 'R22_NATIVE_COMPLETE '+role in log.read_text();output[str(log.relative_to(HERE))]=sha(log)
 assert nom==1920 and extra<=192
 save(HERE/'NATIVE_OUTPUT_FREEZE.json',{'native_outputs_frozen_before_reference':True,'nominal_full':nom,'extra_full':extra,'total_full':nom+extra,'files_sha256':output,'algorithm_freeze_sha256':sha(HERE/'ALGORITHM_FREEZE.json'),'input_and_production_unchanged':True,'test_reference_values_read':False})
 print('R22_OUTPUT_SEAL_PASS',nom,extra)
if __name__=='__main__':{'before':before,'after':after}[sys.argv[1]]()
