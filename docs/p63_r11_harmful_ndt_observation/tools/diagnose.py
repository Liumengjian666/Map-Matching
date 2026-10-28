"""Post-feature-seal reference diagnosis; descriptive distributions, no detector tuning."""
import csv,hashlib,json,subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
A=Path(__file__).resolve().parents[1];W=A.parents[1];OLD=W/'docs/p63_r10_hdl_imu_full_hard'
def dump(n,v):(A/n).write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def write(n,v):
 with (A/n).open('w',newline='') as f:
  d=csv.DictWriter(f,fieldnames=list(v[0]),lineterminator='\n');d.writeheader();d.writerows(v)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
seal=json.loads((A/'FEATURE_SEAL.json').read_text());assert all(sha(A/p)==h for p,h in seal['files'].items())
features=list(csv.DictReader((A/'CAUSAL_FEATURES.csv').open()));poses=json.loads((A/'ONLINE_POSES.json').read_text())
module={'__name__':'r10_frozen_reference','__file__':str(W/'docs/p62_r29_full_sequence_benchmark/tools/evaluate_batch.py')}
src=subprocess.check_output(['git','show','e25754fd99e81144376e9717c23063c5b575e667:docs/p62_r29_full_sequence_benchmark/tools/evaluate_batch.py'],cwd=W)
exec(compile(src,module['__file__'],'exec'),module)
meta=json.loads((OLD/'REFERENCE_METADATA.json').read_text());reference=module['Reference'](meta);error=module['error']
rows=[]
for f,p in zip(features,poses):
 assert int(f['raw_ordinal'])==p['raw_ordinal'];truth=reference.at(p['stamp_ns']);assert truth is not None
 ep_t,ep_r=error(np.array(p['prediction']),truth);en_t,en_r=error(np.array(p['ndt']),truth);eu_t,eu_r=error(np.array(p['ukf_post_fusion_DIAGNOSTIC_ONLY']),truth)
 follow_t,follow_r=error(np.array(p['ukf_post_fusion_DIAGNOSTIC_ONLY']),np.array(p['ndt']))
 dt,dr=en_t-ep_t,en_r-ep_r
 row={'window':f['window'],'raw_ordinal':p['raw_ordinal'],'stamp_ns':p['stamp_ns'],'reference_valid':True,'pred_t_m':ep_t,'pred_R_deg':ep_r,'ndt_t_m':en_t,'ndt_R_deg':en_r,'ukf_t_m':eu_t,'ukf_R_deg':eu_r,'delta_error_t_m':dt,'delta_error_R_deg':dr,'ukf_minus_ndt_m':follow_t,'ukf_minus_ndt_deg':follow_r,'normal_ndt_0_15m_2deg':en_t<=.15 and en_r<=2,'both_improve':dt<0 and dr<0,'both_worsen':dt>0 and dr>0,'mixed_or_zero_change':not ((dt<0 and dr<0) or (dt>0 and dr>0)),'prediction_high':ep_t>.5 or ep_r>5,'ndt_high':en_t>.5 or en_r>5}
 rows.append(row)
write('REFERENCE_DIAGNOSIS.csv',rows)
joined=[dict(f,**r) for f,r in zip(features,rows)]
# Only frozen groups; no candidate rule, classification threshold or fitted score.
groups={'A_fixed_normal_control':[r for r in joined if r['window']=='A'],'ALL90':joined,'NDT_NORMAL_0_15m_2deg':[r for r in joined if r['normal_ndt_0_15m_2deg']],'BOTH_DIMENSIONS_IMPROVE':[r for r in joined if r['both_improve']]}
metrics=['d_t_m','d_R_deg','gaussian','nn_fitness_m2','native_inlier','native_matching_error_m2','correction_rate_m_s','correction_rate_deg_s','pred_motion_m','pred_motion_deg','ndt_jump_m','ndt_jump_deg','correction_pred_motion_cos','imu_age_ms','max_real_imu_gap_ms','cross_correction_overlap_ms']+[f'h{h}_{tag}_{kind}' for h in [1,3,5] for tag in ['t','R'] for kind in ['ratio','rate_ratio']]
def stat(v):
 v=[float(x) for x in v if x is not None and x!=''];assert np.isfinite(v).all()
 if not v:return {'n':0,'min':None,'median':None,'p90':None,'p95':None,'max':None}
 return dict(n=len(v),min=min(v),median=float(np.median(v)),p90=float(np.percentile(v,90)),p95=float(np.percentile(v,95)),max=max(v))
dists=[];positions=[];keys={k:next(r for r in joined if r['raw_ordinal']==k) for k in [309,515]}
for group,data in groups.items():
 for metric in metrics:
  s=stat([r[metric] for r in data]);dists.append(dict(group=group,metric=metric,**s))
  for raw,key in keys.items():
   vals=[float(r[metric]) for r in data if r[metric]!=''];value=float(key[metric]) if key[metric]!='' else None
   positions.append({'group':group,'metric':metric,'key_raw':raw,'value':value,'n':len(vals),'empirical_fraction_le':sum(v<=value for v in vals)/len(vals) if vals and value is not None else None,'not_a_p_value':True})
write('DISTRIBUTIONS.csv',dists);write('KEY_EMPIRICAL_POSITIONS.csv',positions)
# Evaluate overlap with each observed key magnitude as descriptive anchors, never deploy thresholds.
overlap=[]
for group,data in groups.items():
 if group=='ALL90':continue
 for raw,key in keys.items():
  for metric in ['d_t_m','d_R_deg']:
   members=[r['raw_ordinal'] for r in data if float(r[metric])>=float(key[metric])]
   overlap.append({'group':group,'key_raw':raw,'metric':metric,'key_value':float(key[metric]),'group_n':len(data),'at_or_above_n':len(members),'raws':members})
window_summary={w:{'n':sum(r['window']==w for r in rows),'normal_ndt':sum(r['window']==w and r['normal_ndt_0_15m_2deg'] for r in rows),'both_improve':sum(r['window']==w and r['both_improve'] for r in rows),'both_worsen':sum(r['window']==w and r['both_worsen'] for r in rows),'mixed':sum(r['window']==w and r['mixed_or_zero_change'] for r in rows)} for w in ['A','B','C']}
dump('DESCRIPTIVE_RESULTS.json',{'START_SHA':'e25754fd99e81144376e9717c23063c5b575e667','FEATURE_CODE_SHA':seal['CODE_SHA'],'EVALUATION_CODE_SHA':subprocess.check_output(['git','rev-parse','HEAD'],cwd=W,text=True).strip(),'n':90,'reference_n':90,'reference_kind':meta['reference_kind'],'reference_sha256':meta['reference_sha256'],'window_summary':window_summary,'groups':{k:[r['raw_ordinal'] for r in v] for k,v in groups.items()},'keys':keys,'overlap':overlap,'new_NDT':0,'new_UKF':0,'new_ROS':0,'runtime_algorithm_changes':0,'gates_implemented':0,'retrospective_known_failure_analysis':True,'NIS_available':False})
assert all(sha(A/p)==h for p,h in seal['files'].items())
print(json.dumps({'windows':window_summary,'keys':{k:{x:r[x] for x in ['pred_t_m','pred_R_deg','ndt_t_m','ndt_R_deg','ukf_t_m','ukf_R_deg','delta_error_t_m','delta_error_R_deg','d_t_m','d_R_deg','gaussian','native_inlier']} for k,r in keys.items()}},indent=2))
