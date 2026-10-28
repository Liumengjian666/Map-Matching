"""No replay: verify raw algebra, historical causality, seals and existing evaluation."""
import csv,gzip,hashlib,json,subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
A=Path(__file__).resolve().parents[1];W=A.parents[1];OLD=W/'docs/p63_r10_hdl_imu_full_hard'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
checks=[]
def check(b,label):assert b,label;checks.append(label);print('PASS',label)
s=json.loads((A/'FEATURE_SEAL.json').read_text());check(all(sha(A/p)==h for p,h in s['files'].items()),'all causal features, formulas and code hashes unchanged after reference diagnosis')
f=list(csv.DictReader((A/'CAUSAL_FEATURES.csv').open()));r=list(csv.DictReader((A/'REFERENCE_DIAGNOSIS.csv').open()));p=json.loads((A/'ONLINE_POSES.json').read_text())
expected=list(range(180,210))+list(range(299,329))+list(range(505,535))
check([int(x['raw_ordinal']) for x in f]==expected and [int(x['raw_ordinal']) for x in r]==expected,'exactly 90 fixed frames retained in both stages')
check(not any('ukf' in k.lower() and 'past_ukf' not in k.lower() for k in f[0]),'current post-fusion UKF output absent from pre-fusion feature columns')
with gzip.open(OLD/'raw/full_01/ONLINE_RESULTS.csv.gz','rt') as z:o={int(x['raw_ordinal']):x for x in csv.DictReader(z)}
with gzip.open(OLD/'raw/full_01/NATIVE_NDT.csv.gz','rt') as z:n={int(x['stamp_ns']):x for x in csv.DictReader(z)}
def diff(a,b):return np.linalg.norm(a[:3,3]-b[:3,3]),np.degrees(Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).magnitude())
def poses(k):
 x=n[int(o[k]['stamp_ns'])];a=np.array([float(x['guess_'+str(i)+str(j)]) for i in range(4) for j in range(4)]).reshape(4,4);b=np.eye(4);b[:3,3]=[float(x[v]) for v in ['tx','ty','tz']];b[:3,:3]=Rotation.from_quat([float(x[v]) for v in ['qx','qy','qz','qw']]).as_matrix();return a,b
for x,pr in zip(f,p):
 k=int(x['raw_ordinal']);a,b=poses(k);t,ang=diff(a,b)
 assert abs(float(x['d_t_m'])-t)<1e-12 and abs(float(x['d_R_deg'])-ang)<1e-12
 assert np.array_equal(a,np.array(pr['prediction'])) and np.array_equal(b,np.array(pr['ndt']))
 for h in [1,3,5]:
  hist=np.array([diff(*poses(j)) for j in range(k-h,k)])
  assert all(int(o[j]['stamp_ns'])<int(x['stamp_ns']) for j in range(k-h-1,k))
  for i,tag in enumerate(['t','R']):assert abs(float(x[f'h{h}_{tag}_median'])-np.median(hist[:,i]))<1e-12
  span=(int(o[k-1]['stamp_ns'])-int(o[k-h-1]['stamp_ns']))/1e9
  assert abs(float(x[f'h{h}_duration_s'])-span)<1e-12
check(True,'all corrections independently match raw matrices; 1/3/5 history uses only preceding frames and real ns')
# Evaluation equivalence checked only after feature seal, never used to construct features.
history={int(x['raw_ordinal']):x for x in csv.DictReader((OLD/'FULL_TRAJECTORY.csv').open())}
for row in r:
 old=history[int(row['raw_ordinal'])]
 for newkey,oldkey in [('pred_t_m','pred_translation_error_m'),('pred_R_deg','pred_rotation_error_deg'),('ndt_t_m','ndt_translation_error_m'),('ndt_R_deg','ndt_rotation_error_deg'),('ukf_t_m','translation_error_m'),('ukf_R_deg','rotation_error_deg')]:assert abs(float(row[newkey])-float(old[oldkey]))<1e-10
check(True,'all 90 newly calculated reference diagnoses reproduce frozen R10 evaluation')
ident=json.loads((A/'INPUT_IDENTITY.json').read_text())
root=subprocess.check_output(['git','status','--porcelain=v1','-uall'],cwd='/home/jian/livox_ws/dog_loc_paper_ws')
check(hashlib.sha256(root).hexdigest()==ident['root_status_sha256'],'root workspace existing changes untouched')
check(not subprocess.check_output(['git','diff',ident['START_SHA'],'--','docs/p63_r10_hdl_imu_full_hard','docs/p63_r9_hdl_real_imu_prediction'],cwd=W),'historical R9/R10 archive bytes unchanged')
check(all(x['reference_valid']=='True' for x in r),'90/90 valid reference samples; no zero-filled missing values')
check(not any(k in str(f[0].keys()).lower() for k in ['mahalanobis','nis_','covariance']),'no invented covariance/NIS/statistical confidence')
(A/'CHECKS.json').write_text(json.dumps({'passed':checks,'additional_NDT':0,'additional_UKF':0,'ROS_replays':0},indent=2)+'\n')
