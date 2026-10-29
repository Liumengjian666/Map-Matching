"""Independent post-seal checks of reference identity, labels, rank metrics and provenance."""
import csv,hashlib,json,math,subprocess
from collections import Counter
from pathlib import Path
import numpy as np
from scipy.stats import rankdata
P=Path(__file__).resolve().parents[1];W=P.parents[1]
def rows(f):
 with f.open() as z:return list(csv.DictReader(z))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
checks=[]
def check(ok,name):
 assert ok,name
 checks.append(name);print('PASS',name)
for sealname in ['SHADOW_SEAL.json','EVALUATION_SEAL.json']:
 s=json.loads((P/sealname).read_text());files=s['files'] if 'files' in s else s
 check(all(sha(P/f)==h for f,h in files.items()),sealname+' unchanged')
features=rows(P/'SHADOW_FEATURES.csv');labels=rows(P/'REFERENCE_LABELS.csv');rankings=rows(P/'SHADOW_RANKINGS.csv');metrics=rows(P/'RANKING_METRICS.csv')
old={int(r['raw_ordinal']):r for r in rows(W/'docs/p63_r10_hdl_imu_full_hard/FULL_TRAJECTORY.csv')}
byraw={int(r['raw_ordinal']):r for r in labels};fs={int(r['raw_ordinal']):r for r in features}
invalid=[int(r['raw_ordinal']) for r in labels if r['reference_valid']=='False']
check(invalid==[748,1691,1692] and len(labels)==2165,'2165 rows retained; exactly2162 valid references, missing three not zero-filled')
for r in labels:
 if r['reference_valid']=='False':
  assert r['delta_e_t_m']==r['delta_e_R_deg']=='' and r['label']=='REFERENCE_UNAVAILABLE';continue
 for nk,ok in [('e_pred_t_m','pred_translation_error_m'),('e_pred_R_deg','pred_rotation_error_deg'),('e_ndt_t_m','ndt_translation_error_m'),('e_ndt_R_deg','ndt_rotation_error_deg')]:
  assert abs(float(r[nk])-float(old[int(r['raw_ordinal'])][ok]))<1e-10
 dt=float(r['delta_e_t_m']);dr=float(r['delta_e_R_deg'])
 h=dt>=.10 or dr>=2;b=dt<=-.10 or dr<=-2
 assert r['label']==('MIXED_MATERIAL' if h and b else 'MATERIAL_HARM' if h else 'MATERIAL_BENEFIT' if b else 'SMALL_OR_AMBIGUOUS')
check(True,'all reference values reproduce R10 after seal; exact inclusive label priority')
for m in metrics:
 scope=m['scope'];group=m['prediction_group'];feature=m['feature']
 eligible=[r for r in rankings if r['scope']==scope and r['feature']==feature and r['rank_highest_is_1'] and byraw[int(r['raw_ordinal'])]['reference_valid']=='True' and (group=='ALL' or byraw[int(r['raw_ordinal'])]['prediction_group']==group)]
 eligible.sort(key=lambda r:int(r['rank_highest_is_1']));n=len(eligible);top=eligible[:math.ceil(n/10)]
 counts=Counter(byraw[int(r['raw_ordinal'])]['label'] for r in eligible);tc=Counter(byraw[int(r['raw_ordinal'])]['label'] for r in top)
 assert n==int(m['eligible_n']) and len(top)==int(m['top_k'])
 for lab in ['MATERIAL_HARM','MATERIAL_BENEFIT','MIXED_MATERIAL','SMALL_OR_AMBIGUOUS']:assert int(m[lab.lower()+'_n'])==counts[lab]
 assert int(m['top_harm_n'])==tc['MATERIAL_HARM'] and int(m['top_benefit_n'])==tc['MATERIAL_BENEFIT']
 hb=[r for r in eligible if byraw[int(r['raw_ordinal'])]['label'] in ['MATERIAL_HARM','MATERIAL_BENEFIT']]
 nh=counts['MATERIAL_HARM'];nb=counts['MATERIAL_BENEFIT']
 if nh and nb:
  ranks=rankdata([float(r['oriented_risk']) for r in hb],method='average')
  total=sum(x for x,r in zip(ranks,hb) if byraw[int(r['raw_ordinal'])]['label']=='MATERIAL_HARM')
  auc=(total-nh*(nh+1)/2)/(nh*nb)
  assert abs(auc-float(m['harm_vs_benefit_auroc']))<1e-12
 else:assert m['harm_vs_benefit_auroc']==''
 for delta,col in [('delta_e_t_m','spearman_delta_t'),('delta_e_R_deg','spearman_delta_R')]:
  x=[float(r['oriented_risk']) for r in eligible];y=[float(byraw[int(r['raw_ordinal'])][delta]) for r in eligible]
  if len(set(x))>1 and len(set(y))>1:
   rho=np.corrcoef(rankdata(x),rankdata(y))[0,1];assert abs(rho-float(m[col]))<1e-12
  else:assert m[col]==''
 if nh:assert abs(tc['MATERIAL_HARM']/nh-float(m['harm_recall_at_top10']))<1e-12
 else:assert m['harm_recall_at_top10']==''
 sufficient=nh>=10 and nb>=10
 passed=sufficient and float(m['harm_vs_benefit_auroc'])>=.75 and float(m['harm_enrichment'])>=2 and float(m['top_benefit_fraction'])<=.1
 assert (m['passes_exploratory_criteria']=='True')==passed
check(len(metrics)==297,'297 cohorts: eligibility/Top/labels/AUROC independent rank-sum/Spearman and frozen qualification pass')
runs=rows(P/'TEMPORAL_RUNS.csv')
covered=[]
for r in runs:
 ids=list(range(int(r['first_raw']),int(r['last_raw'])+1));covered+=ids
 assert len(ids)==int(r['n'])
 for k in ids:assert byraw[k]['label']==r['label'] and byraw[k]['prediction_group']==r['prediction_group']
 assert abs(float(r['span_seconds'])-(int(r['end_stamp_ns'])-int(r['start_stamp_ns']))/1e9)<1e-12
check(covered==[int(r['raw_ordinal']) for r in labels],'continuous descriptive runs cover all rows and never bridge exclusions or reference gaps')
benefits=rows(P/'BENEFICIAL_CORRECTION_COUNTEREXAMPLES.csv')
check({int(r['raw_ordinal']) for r in benefits}=={int(r['raw_ordinal']) for r in labels if r['label']=='MATERIAL_BENEFIT'},'every beneficial observation retained, including non-Top cases')
for r in benefits:
 m=next(m for m in metrics if m['scope']==r['scope'] and m['prediction_group']==r['prediction_group'] and m['feature']==r['feature'])
 assert (r['in_top10']=='True')==(int(r['cohort_rank'])<=int(m['top_k']))
check(True,'beneficial Top membership uses same frozen cohort ranking')
ident=json.loads((P/'INPUT_IDENTITY.json').read_text())
status=subprocess.check_output(['git','status','--porcelain=v1','-uall'],cwd='/home/jian/livox_ws/dog_loc_paper_ws')
check(hashlib.sha256(status).hexdigest()==ident['root_status_sha256'],'root pre-existing status untouched')
check(not subprocess.check_output(['git','diff',ident['START_SHA'],'--','docs/p63_r9_hdl_real_imu_prediction','docs/p63_r10_hdl_imu_full_hard','docs/p63_r11_harmful_ndt_observation','docs/p63_r12_risk_shadow_ranking'],cwd=W),'historical R9-R12 unchanged')
check([m['feature'] for m in metrics if m['scope']=='GLOBAL' and m['prediction_group']=='ALL' and m['passes_exploratory_criteria']=='True']==['d_R_deg'],'only rotation correction satisfies all global frozen criteria; no reinterpretation')
(P/'EVALUATION_CHECKS.json').write_text(json.dumps({'passed':checks,'NDT':0,'UKF':0,'ROS':0},indent=2)+'\n')
