"""Bounded read-only evidence checks, no ROS/NDT calls or fresh reference parsing."""
import csv,gzip,hashlib,json,subprocess,shutil
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
A=Path(__file__).resolve().parents[1];W=A.parents[1];D=Path('/home/jian/livox_ws/p63_r9_real_imu_prediction_data/imu_on_01')
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
rows=lambda p:list(csv.DictReader(Path(p).open()))
checks=[]
def check(ok,name):
 assert ok,name
 checks.append(name);print('PASS',name)
R=json.loads((A/'RESULTS.json').read_text());F=json.loads((A/'INPUT_FREEZE.json').read_text());B=json.loads((A/'BUILD.json').read_text());C=json.loads((A/'CONTRACT_DECISION.json').read_text());P=json.loads((A/'PREFLIGHT.json').read_text())
check(all(sha(A/n)==v for n,v in C['files_sha256'].items()),'pre-run contract source/binary/input seal unchanged')
check(all(sha(p)==v['sha256'] for p,v in B['native_binaries'].items()),'six frozen native ELFs unchanged')
check(sha(B['trace_source'])==B['trace_source_sha256'] and sha(B['trace_binary'])==B['trace_binary_sha256'],'actual trace source and binary')
s=json.loads((D/'ONLINE_SEAL.json').read_text());check(all(sha(D/p)==h for p,h in s['files'].items()),'online outputs sealed before reference evaluation')
for f in json.loads((A/'RAW_MANIFEST.json').read_text()):
 p=A/f['archive'];b=p.read_bytes();b=gzip.decompress(b) if p.suffix=='.gz' else b
 assert hashlib.sha256(b).hexdigest()==f['source_sha256'] and sha(p)==f['archive_sha256']
check(True,'all archived raw content byte-identical')
n=rows(D/'NATIVE_NDT.csv');u=rows(D/'IMU_PREDICT.csv');rx=rows(D/'IMU_RECEIVED.csv');a=rows(D/'ADAPTER.csv');out=rows(A/'IMU_ON_TRAJECTORY.csv')
check(len(n)==len(out)==160 and [int(x['raw_ordinal']) for x in out]==list(range(5,165)),'one run 160 frames and align calls, all original IDs')
check(len(u)==len(rx)==len(a)==8625 and sum(int(x['active_predict']) for x in u)==5358,'8625 actual IMU receipts/consumptions and 5358 active predictions')
check(all(int(p['imu_stamp_ns'])==int(r['stamp_ns'])==int(v['stamp_ns'])<=int(p['scan_stamp_ns']) for p,r,v in zip(u,rx,a)),'ordered actual IMU no future input or missing receipt')
check(all(p['finite_state']=='1' and float(p['dt_s'])>=0 and p['prev_stamp_after_ns']==p['imu_stamp_ns'] for p in u),'actual native state/time finite and forward')
check(all(p['source_xyz_sha256']==f['source_xyz_sha256'] and p['target_xyz_sha256']==F['target_xyz_sha256'] and p['map_frozen']=='1' for p,f in zip(n,F['frames'])),'all actual Source and frozen Target identities')
by_scan={}
for p in u:by_scan[int(p['scan_stamp_ns'])]=p
for p in n:
 if int(p['stamp_ns']) not in by_scan:continue
 last=by_scan[int(p['stamp_ns'])];guess=np.array([float(p['guess_'+str(i)+str(j)]) for i in range(4) for j in range(4)]).reshape(4,4)
 assert np.allclose(guess[:3,3],[float(last[k]) for k in ['px','py','pz']],atol=1e-7,rtol=0)
 q=Rotation.from_quat([float(last[k]) for k in ['qx','qy','qz','qw']]).as_matrix()
 assert np.max(np.abs(q-guess[:3,:3]))<1e-6
check(True,'last real IMU predictor state is actual NDT guess on 159 nonempty batches')
t=rows(A/'evidence/NATIVE_TIME_PREFLIGHT.csv')
check(all(int(x['imu_consumed'])==int(y['scheduled_imu_calls']) and int(x['imu_active_predict'])==int(y['model_active_predict_calls']) and int(x['last_imu_stamp_ns'])==int(y['last_imu_stamp_ns']) for x,y in zip(out,t)),'preflight clock matches native calls including raw68 and cool-time')
check(all(v for v in R['H2_checks'].values()) and R['new_high_frames']==0 and R['reference_coverage']==1,'complete reference evaluation and unchanged H2 boundaries')
check(R['local_worsening']['rotation_error_deg']['count_delta_above_1e_minus6']==41 and R['local_worsening']['translation_error_m']['count_delta_above_1e_minus6']==7,'local worsening retained')
for name in ['MASTER_RESEARCH_PLAN_AND_LOG.md','INNOVATION_RESEARCH_LOG.md']:
 path='docs/paper_research_control/'+name;old=subprocess.check_output(['git','show',R['START_SHA']+':'+path],cwd=W);assert (W/path).read_bytes().startswith(old)
check(True,'master and innovation ledger append only')
root=Path('/home/jian/livox_ws/dog_loc_paper_ws');dirty=subprocess.check_output(['git','status','--porcelain=v1','-uall'],cwd=root)
check(len(dirty.splitlines())==444 and hashlib.sha256(dirty).hexdigest()=='d9ff40ea7f732beb59d0e595a8e1fd7a47d9750d0f839749eb9790d39392659b','root pre-existing staged/untracked work unchanged')
for name,inf in B['upstream'].items():
 path=Path('/home/jian/livox_ws/p63_r3_hdl_data/catkin_ws/src')/name
 assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=path,text=True).strip()==inf['SHA']
 assert not subprocess.check_output(['git','status','--porcelain'],cwd=path)
check(True,'four upstream source versions pristine')
check(shutil.disk_usage(W).free>=20*1024**3,'20 GiB disk reserve')
check(sha(F['map_path'])==F['map_sha256'],'map input file unchanged after run')
print('TOTAL',len(checks),'NDT added by checks=0')
(A/'FINAL_CHECKS.json').write_text(json.dumps({'passed':checks,'additional_NDT':0,'free_bytes':shutil.disk_usage(W).free,'root_status_sha256':hashlib.sha256(dirty).hexdigest()},indent=2)+'\n')
