"""Freeze all 11 single-feature orders; no reference input or score training."""
import csv,hashlib,json,math,subprocess
from pathlib import Path
A=Path(__file__).resolve().parents[1];W=A.parents[1]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
s=json.loads((A/'FEATURE_SEAL.json').read_text());assert all(sha(A/p)==h for p,h in s['files'].items())
f=list(csv.DictReader((A/'SHADOW_FEATURES.csv').open()))
expected=list(range(240,270))+list(range(350,380))+list(range(450,480))+list(range(550,580))
assert [int(r['raw_ordinal']) for r in f]==expected
spec=[('gaussian',-1),('native_inlier',-1),('nn_fitness_m2',1),('d_t_m',1),('d_R_deg',1)]+[(f'h{h}_{axis}_ratio',1) for h in [1,3,5] for axis in ['t','R']]
assert len(spec)==11
out=[]
for scope in ['GLOBAL','D','E','F','G']:
 group=[r for r in f if scope=='GLOBAL' or r['window']==scope]
 for feature,direction in spec:
  valid=[r for r in group if r[feature]!='' and math.isfinite(float(r[feature]))]
  order=sorted(valid,key=lambda r:(-direction*float(r[feature]),int(r['raw_ordinal'])))
  ranks={int(r['raw_ordinal']):i+1 for i,r in enumerate(order)};top=math.ceil(len(valid)/10)
  for r in group:
   k=int(r['raw_ordinal']);rank=ranks.get(k)
   out.append({'scope':scope,'window':r['window'],'raw_ordinal':k,'stamp_ns':r['stamp_ns'],'feature':feature,'direction':direction,'feature_value':float(r[feature]) if rank else None,'oriented_risk':direction*float(r[feature]) if rank else None,'rank_highest_is_1':rank,'eligible_n':len(valid),'top_k':top,'in_top10':rank<=top if rank else None,'missing_reason':'' if rank else 'R11_MISSING_NONFINITE_ZERO_DENOMINATOR_OR_HISTORY'})
with (A/'SHADOW_RANKINGS.csv').open('w',newline='') as z:
 wr=csv.DictWriter(z,fieldnames=list(out[0]),lineterminator='\n');wr.writeheader();wr.writerows(out)
(A/'RANKING_RULES.json').write_text(json.dumps({'features':[{'name':n,'direction':d} for n,d in spec],'tie':'raw ordinal ascending','top':'ceil(valid_n/10)','no_normalization':True,'missing_by_feature':{n:sum(r[n]=='' or not math.isfinite(float(r[n])) for r in f) for n,_ in spec}},indent=2)+'\n')
files=list(s['files'])+['FEATURE_SEAL.json','SHADOW_RANKINGS.csv','RANKING_RULES.json','REUSE_IDENTITY.json','tools/rank_and_seal.py']
(A/'SHADOW_SEAL.json').write_text(json.dumps({'CODE_SHA':subprocess.check_output(['git','rev-parse','HEAD'],cwd=W,text=True).strip(),'files':{p:sha(A/p) for p in files},'reference_values_or_labels_read':False,'frames':120,'rankings':11,'ranking_rows':len(out),'current_UKF_used_as_prefusion_feature':False},indent=2)+'\n')
print('PASS:120 features; 11 rankings; GLOBAL + four fixed window orders; all sealed before reference; 0 NDT/UKF/ROS')
