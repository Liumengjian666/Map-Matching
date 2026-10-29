"""Read-only evidence checks; no matching, propagation or new risk computation rules."""
import csv, gzip, hashlib, json, math, subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
P=Path(__file__).resolve().parents[1]; W=P.parents[1]; OLD=W/'docs/p63_r10_hdl_imu_full_hard'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def rows(p):
 with p.open() as f:return list(csv.DictReader(f))
checks=[]
def check(ok,name):
 assert ok,name
 checks.append(name);print('PASS',name)
for sealname in ['SHADOW_SEAL.json','EVALUATION_SEAL.json']:
 seal=json.loads((P/sealname).read_text())
 files=seal['files'] if 'files' in seal else seal
 check(all(sha(P/n)==s for n,s in files.items()),sealname+' hashes unchanged')
f=rows(P/'SHADOW_FEATURES.csv'); labs=rows(P/'REFERENCE_LABELS.csv'); rankings=rows(P/'SHADOW_RANKINGS.csv'); metrics=rows(P/'RANKING_METRICS.csv')
expected=list(range(240,270))+list(range(350,380))+list(range(450,480))+list(range(550,580))
check([int(r['raw_ordinal']) for r in f]==expected==[int(r['raw_ordinal']) for r in labs],'all and only 120 fixed formal frames retained')
check(not(set(expected)&(set(range(180,210))|set(range(299,329))|set(range(505,535)))),'no R11 formal sample enters R12')
check(not any('ukf' in k.lower() and 'past_ukf' not in k.lower() for k in f[0]),'current posterior absent from causal feature columns')
with gzip.open(OLD/'raw/full_01/ONLINE_RESULTS.csv.gz','rt') as z:o={int(r['raw_ordinal']):r for r in csv.DictReader(z)}
with gzip.open(OLD/'raw/full_01/NATIVE_NDT.csv.gz','rt') as z:n={int(r['stamp_ns']):r for r in csv.DictReader(z)}
def correction(k):
 r=n[int(o[k]['stamp_ns'])]
 a=np.array([float(r['guess_'+str(i)+str(j)]) for i in range(4) for j in range(4)]).reshape(4,4)
 b=np.eye(4);b[:3,3]=[float(r[x]) for x in ['tx','ty','tz']]
 b[:3,:3]=Rotation.from_quat([float(r[x]) for x in ['qx','qy','qz','qw']]).as_matrix()
 return np.linalg.norm(a[:3,3]-b[:3,3]),np.degrees(Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).magnitude())
for r in f:
 raw=int(r['raw_ordinal']);val=correction(raw)
 assert np.allclose(val,[float(r['d_t_m']),float(r['d_R_deg'])],atol=1e-12,rtol=0)
 for h in [1,3,5]:
  history=np.array([correction(k) for k in range(raw-h,raw)])
  assert all(int(o[k]['stamp_ns'])<int(r['stamp_ns']) for k in range(raw-h,raw))
  for j,tag in enumerate(['t','R']):
   median=float(np.median(history[:,j]));assert median!=0
   assert abs(val[j]/median-float(r[f'h{h}_{tag}_ratio']))<1e-12
check(True,'120 corrections and six history ratios independently match original matrices with strictly past history')
specs=json.loads((P/'RANKING_RULES.json').read_text())['features']
for scope in ['GLOBAL','D','E','F','G']:
 for spec in specs:
  group=[r for r in rankings if r['scope']==scope and r['feature']==spec['name']]
  valid=[r for r in group if r['rank_highest_is_1']]
  ordered=sorted(valid,key=lambda r:(-spec['direction']*float(r['feature_value']),int(r['raw_ordinal'])))
  assert [int(r['rank_highest_is_1']) for r in ordered]==list(range(1,len(valid)+1))
  assert all((r['in_top10']=='True')==(int(r['rank_highest_is_1'])<=math.ceil(len(valid)/10)) for r in valid)
check(len(rankings)==2640,'all 11 global/window ranking directions, tie rules, Top cardinality exact')
oldref={int(r['raw_ordinal']):r for r in rows(OLD/'FULL_TRAJECTORY.csv')}
for r in labs:
 old=oldref[int(r['raw_ordinal'])]
 for new,key in [('e_pred_t_m','pred_translation_error_m'),('e_pred_R_deg','pred_rotation_error_deg'),('e_ndt_t_m','ndt_translation_error_m'),('e_ndt_R_deg','ndt_rotation_error_deg')]:
  assert abs(float(r[new])-float(old[key]))<1e-10
 dt=float(r['delta_e_t_m']);dr=float(r['delta_e_R_deg'])
 h=dt>=.1 or dr>=2;b=dt<=-.1 or dr<=-2
 assert r['label']==('MIXED_MATERIAL' if h and b else 'MATERIAL_HARM' if h else 'MATERIAL_BENEFIT' if b else 'SMALL_OR_AMBIGUOUS')
check(True,'post-seal reference errors agree with R10; labels independently satisfy exact inclusive boundaries')
for m in metrics:
 cohort=[r for r in labs if (m['scope']=='GLOBAL' or r['window']==m['scope']) and (m['prediction_group']=='ALL' or r['prediction_group']==m['prediction_group'])]
 assert len(cohort)==int(m['eligible_n'])
 assert int(m['top_k'])==math.ceil(len(cohort)/10)
 assert int(m['material_harm_n'])==sum(r['label']=='MATERIAL_HARM' for r in cohort)
 assert int(m['material_benefit_n'])==sum(r['label']=='MATERIAL_BENEFIT' for r in cohort)
 assert m['harm_vs_benefit_auroc']=='' and m['harm_recall_at_top10']=='' and m['harm_enrichment']==''
 assert m['auroc_unavailable_reason']=='MISSING_HARM_OR_BENEFIT_CLASS'
check(len(metrics)==165,'all 165 scope/group/feature cohorts retained; undefined AUROC/recall/lift not fabricated')
benefits=rows(P/'BENEFICIAL_CORRECTION_COUNTEREXAMPLES.csv')
check(set(int(r['raw_ordinal']) for r in benefits)=={266,551,552,560} and len(benefits)==176,'all four beneficial observations retained for every feature and applicable cohort')
identity=json.loads((P/'INPUT_IDENTITY.json').read_text())
status=subprocess.check_output(['git','status','--porcelain=v1','-uall'],cwd='/home/jian/livox_ws/dog_loc_paper_ws')
check(hashlib.sha256(status).hexdigest()==identity['root_status_sha256'],'root pre-existing local changes unchanged')
check(not subprocess.check_output(['git','diff',identity['START_SHA'],'--','docs/p63_r10_hdl_imu_full_hard','docs/p63_r11_harmful_ndt_observation'],cwd=W),'R10 and R11 archives unchanged')
check(json.loads((P/'RESULTS.json').read_text())['verdict']=='EVALUATION_INSUFFICIENT','preregistered coverage stop respected; no unsupported detector claim')
(P/'CHECKS.json').write_text(json.dumps(dict(passed=checks,additional_NDT=0,additional_UKF=0,ROS_replays=0),indent=2)+'\n')
