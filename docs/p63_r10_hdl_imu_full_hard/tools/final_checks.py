"""Bounded evidence checks. No reference parsing, ROS replay or NDT calls."""
import csv,gzip,hashlib,json,shutil,subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
A=Path(__file__).resolve().parents[1];W=A.parents[1];D=Path('/home/jian/livox_ws/p63_r10_imu_full_hard_data/full_01')
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
rows=lambda p:list(csv.DictReader(Path(p).open()))
checks=[]
def check(ok,name):
    assert ok,name
    checks.append(name);print('PASS',name)
R=json.loads((A/'FULL_RESULTS.json').read_text());F=json.loads((A/'INPUT_FREEZE.json').read_text());B=json.loads((A/'BUILD.json').read_text());C=json.loads((A/'CONTRACT_DECISION.json').read_text())
check(all(sha(A/n)==v for n,v in C['files_sha256'].items()),'pre-run input/source/parameter/RPE contract seal unchanged')
check((A/'tools/imu_geometry.py').read_bytes()==(W/'docs/p63_r9_hdl_real_imu_prediction/tools/imu_geometry.py').read_bytes(),'R9 physical adapter byte-identical')
check(all(sha(p)==v['sha256'] for p,v in B['native_binaries'].items()),'six frozen native runtime ELF files unchanged')
check(sha(B['trace_source'])==B['trace_source_sha256'] and sha(B['trace_binary'])==B['trace_binary_sha256'],'R9 actual read-only trace source and binary unchanged')
s=json.loads((D/'ONLINE_SEAL.json').read_text());check(all(sha(D/p)==h for p,h in s['files'].items()),'online output seal unchanged after reference evaluation')
for f in json.loads((A/'RAW_MANIFEST.json').read_text()):
    p=A/f['archive'];b=p.read_bytes();b=gzip.decompress(b) if p.suffix=='.gz' else b
    assert hashlib.sha256(b).hexdigest()==f['source_sha256'] and sha(p)==f['archive_sha256']
check(True,'all raw archives are byte-identical to original run content')
n=rows(D/'NATIVE_NDT.csv');u=rows(D/'IMU_PREDICT.csv');rx=rows(D/'IMU_RECEIVED.csv');a=rows(D/'ADAPTER.csv');out=rows(A/'FULL_TRAJECTORY.csv');cb=rows(D/'CALLBACKS.csv')
check(len(n)==len(out)==len(cb)==2375 and [int(x['raw_ordinal']) for x in out]==list(range(5,2380)),'single continuous complete raw5-2379; 2375 actual align/callback/output rows')
check(R['run']['formal_trials']==1 and R['run']['same_ukf_instance'] and R['extra_align']==0 and not R['run']['test_GT_access'],'one run, original state instance, no extra align or GT feedback')
check(len(u)==len(rx)==len(a)==44267 and sum(int(x['active_predict']) for x in u)==41000,'actual 44267 IMU receipts and predict calls; 41000 active')
check(all(int(p['imu_stamp_ns'])==int(r['stamp_ns'])==int(v['stamp_ns'])<=int(p['scan_stamp_ns']) for p,r,v in zip(u,rx,a)) and all(int(y['imu_stamp_ns'])>int(x['imu_stamp_ns']) for x,y in zip(u,u[1:])),'actual ordered consumption: no future IMU or duplicated sample')
check(all(p['finite_state']=='1' and float(p['dt_s'])>=0 and p['prev_stamp_after_ns']==p['imu_stamp_ns'] for p in u) and all(x['core_finite']=='True' for x in out),'observed position/velocity/quaternion states finite and propagation clock forward')
check(all(p['source_xyz_sha256']==f['source_xyz_sha256'] and p['target_xyz_sha256']==F['target_xyz_sha256'] and p['map_frozen']=='1' for p,f in zip(n,F['frames'])),'all actual Source and frozen Target match preflight identities')
by_scan={}
for p in u:by_scan[int(p['scan_stamp_ns'])]=p
nonempty=0
for p in n:
    if int(p['stamp_ns']) not in by_scan:continue
    last=by_scan[int(p['stamp_ns'])];guess=np.array([float(p['guess_'+str(i)+str(j)]) for i in range(4) for j in range(4)]).reshape(4,4)
    assert np.allclose(guess[:3,3],[float(last[k]) for k in ['px','py','pz']],atol=1e-7,rtol=0)
    q=Rotation.from_quat([float(last[k]) for k in ['qx','qy','qz','qw']]).as_matrix()
    assert np.max(np.abs(q-guess[:3,:3]))<1e-6
    nonempty+=1
check(nonempty==2373,'2373 nonempty batches: actual last IMU state feeds actual NDT guess')
t=rows(A/'evidence/NATIVE_TIME_PREFLIGHT.csv')
check(all(int(x['imu_consumed'])==int(y['scheduled_imu_calls']) and int(x['imu_active_predict'])==int(y['model_active_predict_calls']) and int(x['last_imu_stamp_ns'])==int(y['last_imu_stamp_ns']) and abs(float(x['imu_integrated_s'])-float(y['model_integrated_s']))<1e-8 for x,y in zip(out,t)),'full preflight clock model matches native counts/time, including raw68/1451 and cool-time')
check(R['reference_invalid_raws']==[748,1691,1692] and sum(x['reference_valid']=='True' for x in out)==2372 and R['reference_coverage']==2372/2375,'2372 valid reference rows, three missing rows retained without zero filling')
check(R['verdict']=='FULL_SEQUENCE_LOCALIZATION_FAIL' and R['high_error_frames']==1868 and [(e['first_raw'],e['recovery_raw']) for e in R['sustained_episodes']]==[(309,314),(515,None)],'frozen thresholds classify two sustained failure episodes, not success')
check(R['first160_vs_R9']['position']['max']==0 and R['first160_vs_R9']['rotation']['max']==0,'all first 160 UKF poses reproduce R9 exactly')
rpe=rows(A/'RPE_1S.csv');pairs=rows(A/'RPE_PAIRS.csv')
check(len(rpe)==len(pairs)==2281 and sum(r['valid']=='True' for r in rpe)==2278 and all(all(r[k]==p[k] for k in p) for r,p in zip(rpe,pairs)),'1-second RPE uses only pre-run frozen pair/time rules')
for name in ['MASTER_RESEARCH_PLAN_AND_LOG.md','INNOVATION_RESEARCH_LOG.md']:
    path='docs/paper_research_control/'+name;old=subprocess.check_output(['git','show',R['START_SHA']+':'+path],cwd=W);assert (W/path).read_bytes().startswith(old)
check(True,'master and innovation records append only')
root=Path('/home/jian/livox_ws/dog_loc_paper_ws');dirty=subprocess.check_output(['git','status','--porcelain=v1','-uall'],cwd=root)
check(len(dirty.splitlines())==444 and hashlib.sha256(dirty).hexdigest()=='d9ff40ea7f732beb59d0e595a8e1fd7a47d9750d0f839749eb9790d39392659b','root pre-existing 444 staged/untracked entries unchanged')
for name,inf in B['upstream'].items():
    path=Path('/home/jian/livox_ws/p63_r3_hdl_data/catkin_ws/src')/name
    assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=path,text=True).strip()==inf['SHA']
    assert not subprocess.check_output(['git','status','--porcelain'],cwd=path)
check(True,'four original upstream source versions remain pristine')
check(shutil.disk_usage(W).free>=20*1024**3,'at least 20 GiB storage reserve')
check(sha(F['map_path'])==F['map_sha256'],'original map file unchanged after run')
print('TOTAL',len(checks),'additional NDT=0')
(A/'FINAL_CHECKS.json').write_text(json.dumps({'passed':checks,'additional_NDT':0,'free_bytes':shutil.disk_usage(W).free,'root_status_sha256':hashlib.sha256(dirty).hexdigest()},indent=2)+'\n')
