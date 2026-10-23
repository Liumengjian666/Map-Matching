#!/usr/bin/env python3
"""Reference access is permitted only after all four native output seals exist."""
import csv,json,math,sys
from collections import defaultdict
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from prepare_inputs import HERE,REPO,GT,sha,save
sys.path.insert(0,str(REPO/'docs/p62_r20_quality_gated_recovery/tools'))
from baseline_stats import errors,distribution,longest_run
POLICIES=['BASELINE_NATIVE_NDT','QUALITY_GATE_ONLY','QUALITY_TRIGGERED_MULTISTART','SINGLE_FRAME_RELATIVE_ADMISSION']
A,B,C,D=POLICIES

def merge(a,b):return dict(a,**b)
def rows(p):return list(csv.DictReader(Path(p).open()))
def write(p,data):
 assert data
 with p.open('x',newline='') as f:
  w=csv.DictWriter(f,fieldnames=data[0],lineterminator='\n');w.writeheader();w.writerows({k:'NA' if v is None else v for k,v in r.items()} for r in data)
def pose(r,p):
 m=np.array([float(r[f'{p}{i}{j}']) if r[f'{p}{i}{j}']!='NA' else math.nan for i in range(4) for j in range(4)]).reshape(4,4)
 return m if np.isfinite(m).all() else None
def error(p,q):
 if p is None:return None,None
 return float(np.linalg.norm(p[:3,3]-q[:3,3])),math.degrees(math.acos(float(np.clip((np.trace(q[:3,:3].T@p[:3,:3])-1)/2,-1,1))))
def key(r):return r['role'],r['arm'],r['policy'],int(r['frame'])
def count(rs,k):return sum(r[k]=='1' for r in rs)
def main():
 seal=json.loads((HERE/'NATIVE_OUTPUT_FREEZE.json').read_text());assert seal['native_outputs_frozen_before_reference'] and seal['nominal_full']==1920 and seal['total_full']<=2112
 for p,h in seal['files_sha256'].items():assert sha(HERE/p)==h,p
 freeze=json.loads((HERE/'ALGORITHM_FREEZE.json').read_text());assert sha(HERE/'ALGORITHM_FREEZE.json')==seal['algorithm_freeze_sha256']
 ident=json.loads((HERE/'WINDOW_IDENTITY.json').read_text());assert sha(HERE/'WINDOW_IDENTITY.json')==freeze['window_identity_sha256']
 merged={}
 for name in ['CAUSAL_RESULTS.csv','CANDIDATE_AND_ADMISSION_LOG.csv','ALIGN_CALLS.csv','RUNTIME_AND_MEMORY.csv','TRAJECTORY_RECEIPT.csv']:
  merged[name]=[r for role in ident['windows'] for r in rows(HERE/role.lower()/'formal_run_01'/name)]
 native=merged['CAUSAL_RESULTS.csv'];candidates=merged['CANDIDATE_AND_ADMISSION_LOG.csv'];times=merged['RUNTIME_AND_MEMORY.csv'];calls=merged['ALIGN_CALLS.csv']
 assert len(native)==len({key(r) for r in native})==1920 and len(candidates)==len(calls)==seal['total_full']
 cm=defaultdict(list)
 for r in candidates:cm[key(r)].append(r)
 # Verify decisive flags without reference values.
 for r in native:
  cs=cm[key(r)];assert len(cs)==int(r['candidate_count']);nom=cs[0];assert nom['candidate_id']=='0'
  assert nom['basic_quality']==r['nominal_basic_quality'] and nom['quality_gate_accepted']==r['nominal_gate_accepted']
  assert r['policy'] in POLICIES and int(r['extra_aligns'])==len(cs)-1
  if r['relative_only_admitted']=='1':
   assert r['policy']==D and r['accepted_observation']=='1' and not any(c['quality_gate_accepted']=='1' for c in cs)
   selected=next(c for c in cs if c['chosen']=='1');assert selected['pareto_dominates']=='1' and selected['basic_quality']=='1'
  for c in cs:
   if c['candidate_id']=='0':continue
   dom=(c['basic_quality']=='1' and c['nominal_comparable']=='1' and float(c['native_score'])>float(nom['native_score']) and float(c['overlap_0p5'])>float(nom['overlap_0p5']) and float(c['heldout_mean'])<float(nom['heldout_mean']) and (float(c['translation_from_nominal_m'])>.10 or float(c['rotation_from_nominal_deg'])>1)) if c['nominal_comparable']=='1' else False
   assert dom==(c['pareto_dominates']=='1')
  if r['accepted_observation']=='0':assert pose(r,'selected') is None and np.array_equal(pose(r,'tp'),pose(r,'output'))
 # Only now read test reference poses (no alignment/offset fitting).
 assert sha(GT)==ident['reference_sha256'];wanted={int(r['stamp_ns']) for r in native};assert len(wanted)==240
 gt={};tbl=np.array(ident['T_B_L'])
 with GT.open() as f:
  next(f)
  for line in f:
   sec,ns,rest=line.rstrip().split(',',2);stamp=int(sec)*10**9+int(ns)
   if stamp not in wanted:continue
   v=np.array([float(x) for x in rest.split(',')]);assert np.isfinite(v).all() and len(v)==7 and abs(np.linalg.norm(v[3:])-1)<2e-5
   t=np.eye(4);t[:3,:3]=Rotation.from_quat(v[3:]).as_matrix();t[:3,3]=v[:3];gt[stamp]=t@tbl
 assert set(gt)==wanted
 er={};error_rows=[];groups=defaultdict(list);tg=defaultdict(list)
 for t in times:
  if t['policy']!='MAP_INIT':tg[t['role'],t['arm'],t['policy']].append(t)
 for r in native:
  k=key(r);er[k]={};groups[k[:3]].append(r)
  for prefix in ['tp','raw','selected','output']:
   a,b=error(pose(r,prefix),gt[int(r['stamp_ns'])]);er[k][prefix]=(a,b)
   error_rows.append(merge({x:r[x] for x in ['role','arm','policy','frame','raw_ordinal','stamp_ns']},{'estimate':prefix,'valid':int(a is not None),'translation_m':a,'rotation_deg':b,'reference_type':'MAP_ANCHORED_BENCHMARK_REFERENCE; CONDITIONAL_BASE'}))
 summary={};follow=[]
 for k,rs in groups.items():
  assert len(rs)==60 and [int(r['frame']) for r in rs]==list(range(1,61));tm=tg[k];cs=[c for r in rs for c in cm[key(r)]];ns=[cm[key(r)][0] for r in rs]
  errs=[er[key(r)]['output'] for r in rs];bad=[a>.5 or b>5 for a,b in errs];low=[a<=.3 and b<=3 for a,b in errs];falls=[r['accepted_observation']=='0' for r in rs]
  badfirst=next((j for j,b in enumerate(bad) if b),None);recovery=next((j for j in range((badfirst+1) if badfirst is not None else 60,58) if all(low[j:j+3])),None)
  out={'samples':60,'translation_m':errors([a for a,b in errs]),'rotation_deg':errors([b for a,b in errs]),'converged_nominal':count(ns,'converged'),'convergence_rate':count(ns,'converged')/60,'basic_quality_failed':60-count(rs,'nominal_basic_quality'),'absolute_quality_alarm_frames':count(rs,'quality_event'),'basic_pass_but_mean_over_0p30':sum(r['nominal_basic_quality']=='1' and r['nominal_gate_accepted']=='0' for r in rs),'rejected_observations':sum(falls),'accepted_observations':60-sum(falls),'relative_dominance_candidates':count(cs,'pareto_dominates'),'relative_only_candidates':count(cs,'relative_only_candidate'),'relative_only_admissions':count(rs,'relative_only_admitted'),'accepted_extra':count(rs,'selected_extra'),'fallback_count':sum(falls),'longest_fallback':longest_run(falls),'extra_ndt':sum(int(r['extra_aligns']) for r in rs),'high_error_output_frames':sum(bad),'longest_high_error':longest_run(bad),'first_high_error_raw':int(rs[badfirst]['raw_ordinal']) if badfirst is not None else None,'first_three_low_after_high_raw':int(rs[recovery]['raw_ordinal']) if recovery is not None else None,'high_error_after_recovery':sum(bad[recovery:]) if recovery is not None else None,'frame_wall_ms':distribution([r['frame_wall_ms'] for r in tm]),'event_wall_ms':distribution([r['frame_wall_ms'] for r in tm if r['event']=='1']),'extra_align_ms':sum(float(r['extra_align_ms']) for r in tm),'peak_rss_KiB':max(int(r['peak_rss_KiB']) for r in tm),'nominal_native_score':distribution([c['native_score'] for c in ns]),'nominal_overlap':distribution([c['overlap_0p5'] for c in ns]),'nominal_coverage_consistency_mean':distribution([c['heldout_mean'] for c in ns]),'nominal_full_calls':60}
  summary['/'.join(k)]=out
  for j,r in enumerate(rs):
   if r['relative_only_admitted']!='1':continue
   cr=next(x for x in groups.get((k[0],k[1],C),[]) if x['frame']==r['frame'])
   follow.append(merge({x:r[x] for x in ['role','arm','policy','frame','raw_ordinal','selected_id']},{'output_translation_m':errs[j][0],'output_rotation_deg':errs[j][1],'nominal_translation_m':er[key(r)]['raw'][0],'nominal_rotation_deg':er[key(r)]['raw'][1],'C_output_translation_m':er[key(cr)]['output'][0],'C_output_rotation_deg':er[key(cr)]['output'][1],'subsequent_high_error_frames':sum(bad[j+1:]),'remaining_frames':59-j}))
 comparisons=[];side=[];support=set();events=set();counter=False
 for role in ident['windows']:
  for arm in ['C0','C3']:
   c=summary[f'{role}/{arm}/{C}'];d=summary[f'{role}/{arm}/{D}'];a=summary[f'{role}/{arm}/{A}']
   dt=d['translation_m']['rmse']-c['translation_m']['rmse'];dr=d['rotation_deg']['rmse']-c['rotation_deg']['rmse'];event=d['relative_only_admissions']>0
   net=event and dt< -1e-9 and dr< -1e-9 and d['high_error_output_frames']<=c['high_error_output_frames'] and d['longest_fallback']<=c['longest_fallback']
   safety=event and (dt>.03 or dr>.3)
   comparisons.append({'role':role,'arm':arm,'D_relative_events':d['relative_only_admissions'],'D_minus_C_translation_rmse_m':dt,'D_minus_C_rotation_rmse_deg':dr,'D_minus_C_high_error_frames':d['high_error_output_frames']-c['high_error_output_frames'],'D_minus_C_longest_fallback':d['longest_fallback']-c['longest_fallback'],'D_minus_C_extra_calls':d['extra_ndt']-c['extra_ndt'],'D_minus_C_mean_ms':d['frame_wall_ms']['mean']-c['frame_wall_ms']['mean'],'net_benefit':net,'event_safety_counterexample':safety,'same_output_matrices':all(np.array_equal(pose(x,'output'),pose(y,'output')) for x,y in zip(groups[role,arm,C],groups[role,arm,D]))})
   if event:events.add(role)
   if net and ident['windows'][role]['history_exposure']['confirmatory_eligible']:support.add(role)
   counter|=safety
   if arm=='C0':
    for policy in [B,C,D]:
     x=summary[f'{role}/{arm}/{policy}'];dt0=x['translation_m']['rmse']-a['translation_m']['rmse'];dr0=x['rotation_deg']['rmse']-a['rotation_deg']['rmse'];fail=dt0>.03 or dr0>.3
     side.append({'role':role,'policy':policy,'translation_rmse_increase_m':dt0,'rotation_rmse_increase_deg':dr0,'side_effect_failure':fail})
     if policy==D:counter|=fail
 if counter:status='RELATIVE_ADMISSION_SAFETY_COUNTEREXAMPLE'
 elif not events:status='MECHANISM_NOT_EXERCISED_ON_HOLDOUT'
 elif len(support)>=2:status='HOLDOUT_MECHANISM_BENEFIT_SUPPORTED'
 elif len(support)==1:status='LIMITED_HOLDOUT_SUPPORT'
 else:status='HOLDOUT_BENEFIT_NOT_ESTABLISHED'
 for name,data in merged.items():write(HERE/name,data)
 write(HERE/'REFERENCE_ERROR.csv',error_rows);write(HERE/'D_VS_C.csv',comparisons);write(HERE/'C0_SIDE_EFFECTS.csv',side)
 if follow:write(HERE/'RELATIVE_EVENT_FOLLOWUP.csv',follow)
 else:(HERE/'RELATIVE_EVENT_FOLLOWUP.csv').write_text('role,arm,policy,frame,raw_ordinal,selected_id\n')
 save(HERE/'results.json',{'task':'P62-R22-FROZEN-RELATIVE-ADMISSION-HOLDOUT-VALIDATION','status':status,'start_sha':freeze['start_sha'],'code_sha':freeze['code_sha'],'binary_sha256':freeze['binary_sha256'],'end_sha_location':'final archive commit; provided in handoff','native_output_seal_sha256':sha(HERE/'NATIVE_OUTPUT_FREEZE.json'),'calls':{k:seal[k] for k in ['nominal_full','extra_full','total_full']},'reference_samples_unique':240,'reference_type':'MAP_ANCHORED_BENCHMARK_REFERENCE; CONDITIONAL_BASE','coverage_label':'Coverage Consistency Mean; Full includes these points; not independent holdout evidence','history_exposure':{r:w['history_exposure'] for r,w in ident['windows'].items()},'trajectories':summary,'D_vs_C':comparisons,'C0_side_effects':side,'relative_event_windows':sorted(events),'net_benefit_confirmatory_windows':sorted(support),'relative_event_followups':follow,'peak_rss_KiB':max(int(r['peak_rss_KiB']) for r in times),'runtime_all_frames_ms':distribution([r['frame_wall_ms'] for r in times if r['policy']!='MAP_INIT']),'production_and_input_unchanged':seal['input_and_production_unchanged'],'limitations':['same sequence temporal windows, not cross-dataset generalization','input geometry had prior screening; no documented error-driven method selection exposure','no independent IMU prediction','admission is not correctness or recovery; fallback included in errors','fixed order execution; timings not randomized','map-anchored conditional Base reference, no posthoc alignment'],'next':'Stop offline prototype expansion; research lead decides existing dog system LiDAR-IMU engineering closure. D is core only if holdout support is established; otherwise experimental ablation only.'})
 print(status)
 for r in comparisons:print(json.dumps(r))
if __name__=='__main__':main()
