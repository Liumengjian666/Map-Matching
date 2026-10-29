"""Verify frozen online-only features and ordering without opening any reference values."""
import csv,gzip,hashlib,json,math
from collections import Counter
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
P=Path(__file__).resolve().parents[1];W=P.parents[1];D=W/'docs/p63_r10_hdl_imu_full_hard/raw/full_01'
def rows(path):
 with path.open() as f:return list(csv.DictReader(f))
checks=[]
def check(ok,name):
 assert ok,name
 checks.append(name);print('PASS',name)
s=json.loads((P/'SHADOW_SEAL.json').read_text())
check(all(hashlib.sha256((P/name).read_bytes()).hexdigest()==h for name,h in s['files'].items()),'pre-reference feature/source/input SHA seal')
part=json.loads((P/'PARTITION.json').read_text());a,b,c=[set(part[k]) for k in ['r11','r12','r13']]
check([len(a),len(b),len(c)]==[90,120,2165] and not(a&b or a&c or b&c) and a|b|c==set(range(5,2380)),'disjoint90+120+2165=2375 exact partition')
f=rows(P/'SHADOW_FEATURES.csv');rank=rows(P/'SHADOW_RANKINGS.csv')
check([int(r['raw_ordinal']) for r in f]==sorted(c),'all2165 formal rows, none excluded for missing history')
with gzip.open(D/'NATIVE_NDT.csv.gz','rt') as z:native=list(csv.DictReader(z))
with gzip.open(D/'ONLINE_RESULTS.csv.gz','rt') as z:online=list(csv.DictReader(z))
vals={};stamps={}
for n,o in zip(native,online):
 k=int(o['raw_ordinal']);stamps[k]=int(n['stamp_ns'])
 p=np.array([float(n[f'guess_{i}{j}']) for i in range(4) for j in range(4)]).reshape(4,4)
 t=np.array([float(n[x]) for x in ['tx','ty','tz']]);r=Rotation.from_quat([float(n[x]) for x in ['qx','qy','qz','qw']]).as_matrix()
 vals[k]=(float(np.linalg.norm(p[:3,3]-t)),float(np.degrees(Rotation.from_matrix(p[:3,:3].T@r).magnitude())))
for r in f:
 k=int(r['raw_ordinal'])
 assert vals[k]==(float(r['d_t_m']),float(r['d_R_deg']))
 for h in [1,3,5]:
  ids=list(range(k-h,k));enough=all(x in vals for x in ids)
  for j,tag in enumerate(['t','R']):
   name=f'h{h}_{tag}_ratio';v=r[name];reason=r[name+'_missing_reason']
   if not enough:assert v=='' and reason=='HISTORY_INSUFFICIENT';continue
   assert all(stamps[i]<stamps[k] for i in ids)
   med=float(np.median([vals[i][j] for i in ids]))
   if med==0:assert v=='' and reason=='ZERO_DENOMINATOR'
   else:assert float(v)==vals[k][j]/med and reason==''
check(True,'independent raw SO3/translation and fixed medians; missing history and zero denominator not filled')
spec=json.loads((P/'RANKING_RULES.json').read_text())['features']
for scope in ['GLOBAL']+[x[0] for x in part['segments']]:
 for x in spec:
  group=[r for r in rank if r['scope']==scope and r['feature']==x['name']]
  valid=[r for r in group if r['rank_highest_is_1']]
  order=sorted(valid,key=lambda r:(-x['direction']*float(r['feature_value']),int(r['raw_ordinal'])))
  assert [int(r['rank_highest_is_1']) for r in order]==list(range(1,len(valid)+1))
  assert all((r['in_top10']=='True')==(int(r['rank_highest_is_1'])<=math.ceil(len(valid)/10)) for r in valid)
  assert all(r['missing_reason'] and not r['in_top10'] for r in group if not r['rank_highest_is_1'])
check(len(rank)==2165*11*2,'all global/natural-segment rankings, ties, missing records and Topceil exact')
check(all('ukf' not in k.lower() and 'reference' not in k.lower() for k in f[0]),'current posterior and reference excluded from risk columns')
missing={x['name']:dict(Counter(r[x['name']+'_missing_reason'] for r in f if r[x['name']+'_missing_reason'])) for x in spec}
(P/'FEATURE_CHECKS.json').write_text(json.dumps({'passed':checks,'missing_reasons':missing,'reference_values_read':False,'new_NDT':0},indent=2)+'\n')
