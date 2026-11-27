"""Seal actual trajectories before reading the legacy independent reference."""
import csv
import hashlib
import json
from pathlib import Path
import runpy
import sys
from collections import Counter
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from check_gate import inspect,read

HERE=Path(__file__).resolve().parent
Q1=HERE.parent/'p64_r2_q1_candidate_search'/'prepare_inputs.py'
POSE=[f'{i}{j}' for i in range(4) for j in range(4)]

def digest(f):
    h=hashlib.sha256()
    with f.open('rb') as s:
        for data in iter(lambda:s.read(1<<20),b''):h.update(data)
    return h.hexdigest()

def write(p,rows):
    fields=list(dict.fromkeys(k for r in rows for k in r))
    with p.open('x') as f:
        w=csv.DictWriter(f,fieldnames=fields,lineterminator='\n')
        w.writeheader();w.writerows(rows)

def pose(row,mode):
    return np.array([float(row[mode+s]) for s in POSE]).reshape(4,4)

def error(p,q):
    return np.linalg.norm(p[:3,3]-q[:3,3]),np.rad2deg(Rotation.from_matrix(p[:3,:3].T@q[:3,:3]).magnitude())

def resolve(runs,out):
    dirs=sorted(p for p in runs.iterdir() if p.is_dir() and (p/'trajectory.csv').exists())
    if not dirs:raise ValueError('no causal runs')
    logged={p.name:read(p/'trajectory.csv') for p in dirs}
    for run,rows in logged.items():
        n=72 if run.startswith('w2') else 50 if run.startswith('dog') else 30
        assert len(rows)==n,(run,len(rows),n)
    parity={}
    for name in ('omni_OFF','omni_B','omni_C','omni_SHADOW','omni_TIMEOUT'):
        parity[name]=inspect(runs/'omni_A',runs/name)
    parity['w2_SHADOW']=inspect(runs/'w2_A',runs/'w2_SHADOW')
    parity['dog_SHADOW']=inspect(runs/'dog_A',runs/'dog_SHADOW')
    (out/'baseline_parity.json').write_text(json.dumps(parity,indent=2)+'\n')
    seal={str(f):digest(f) for p in dirs for f in p.glob('*.csv')}
    (out/'online_seal.json').write_text(json.dumps(seal,indent=2)+'\n')
    assert all(digest(Path(p))==sha for p,sha in seal.items())
    latest=max(int(r['stamp_ns']) for k,z in logged.items() if not k.startswith('dog') for r in z)
    original=runpy.run_path(str(Q1))
    tracking=original['read_omni_reference'](latest)
    def ref(name,stamp):
        return None if name.startswith('dog') else original['interpolate'](*tracking,int(stamp),True)
    shadow=[];summary=[];rank=[];refinements=[];filters=[];runtime=[];deg=[]
    fig,axes=plt.subplots(1,3,figsize=(14,4),layout='constrained')
    for p in dirs:
        name=p.name
        trace=logged[name]
        decisions=read(p/'decisions.csv')
        diagnostics=read(p/'diagnostics.csv')
        if name.endswith(('_A','_ACTIVE','_SHADOW')) and 'TIMEOUT' not in name:
            ax=axes[2 if name.startswith('dog') else 1 if name.startswith('w2') else 0]
            xy=np.stack([pose(r,'state')[:2,3] for r in trace])
            ax.plot(*xy.T,label=name)
            ax.set_title('Dog engineering' if name.startswith('dog') else 'Omni W2' if name.startswith('w2') else 'Omni normal')
            ax.set_aspect('equal')
        assert len(decisions)==len(trace)==len(diagnostics)
        errs=[]; rpe_pairs=[]; synced=[]; bad_start=None; loss=0.;bad_n=0;prev=None
        candidates=read(p/'candidates.csv')
        candidate_frames={r['frame'] for r in candidates}
        counts=Counter();loss_count=0
        for r,d,h in zip(trace,decisions,diagnostics):
            assert r['stamp_ns']==d['stamp_ns']==h['stamp_ns'] and int(r['latest_imu_ns'])<=int(r['stamp_ns'])
            actual=int(r['updates'])
            assert actual in (0,1)
            assert actual==int(d['actual']!='PROPAGATE')
            if actual:assert r['last_update_ns']==r['stamp_ns']
            if not actual and prev is not None:assert r['last_update_ns']==prev['last_update_ns']
            prev=r
            gt=ref(name,r['stamp_ns'])
            if gt is not None:
                e=error(pose(r,'state'),gt)
                errs.append(e)
                synced.append((int(r['stamp_ns']),pose(r,'state'),gt))
                native_error=error(pose(r,'ndt'),gt)
                native_ok=native_error[0]<=.15 and native_error[1]<=2
                counts['wrong_native_accept']+=int(d['actual']=='NATIVE' and not native_ok)
                counts['beneficial_native_reject']+=int(d['actual']=='PROPAGATE' and native_ok)
                if e[0]>.15 or e[1]>2:
                    if bad_n==0:bad_start=int(r['stamp_ns'])
                    bad_n+=1
                else:
                    if bad_n>=3:loss+=(int(r['stamp_ns'])-bad_start)*1e-9;loss_count+=1
                    bad_n=0
            if d['trigger']!='NONE' and d['geometry_state']=='NOT_RUN' and d['score_calls']=='0':
                d=dict(d, budget_outcome='NOT_RUN_DETECT_ONLY' if name.endswith('_B') else 'TRIGGER_NO_SEARCH_BUDGET')
            if int(d['candidate_fits'])>0:
                counts['events']+=1
            counts['triggers']+=int(d['trigger']!='NONE')
            counts['propagate']+=1-actual
            counts['late']+=int(d['geometry_state']=='SEARCH_TIMEOUT')
            counts['low_support']+=int(d['geometry_state']=='LOW_SUPPORT')
            filters.append(dict(run=name,frame=r['frame'],stamp_ns=r['stamp_ns'],latest_imu_ns=r['latest_imu_ns'],
                imu_count=r['imu_count'],updates=actual,last_update_ns=r['last_update_ns'],proposed=d['decision'],actual=d['actual'],
                prior_xyz=';'.join(r[k] for k in ('pred03','pred13','pred23')),
                native_xyz=';'.join(r[k] for k in ('ndt03','ndt13','ndt23')),
                state_xyz=';'.join(r[k] for k in ('state03','state13','state23')),
                covariance_diagonal=';'.join(str(float(h['covariance_after'].split(';')[17*i])) for i in range(16)),
                reference_translation_error_m=e[0] if gt is not None else '',reference_rotation_error_deg=e[1] if gt is not None else ''))
            shadow.append(dict(run=name,**d))
            if not name.endswith('_A') and not name.endswith('_OFF'):
                deg.append(dict(run=name,stage='NDT_ADAPTED',frame=r['frame'],stamp_ns=r['stamp_ns'],weak=h['weak'],
                                weak_rank=h['local_weak_rank'],risk=h['risk'],spectrum=h['spectrum'],weak_subspace=h['weak_subspace']))
        if bad_n>=3:loss+=(int(trace[-1]['stamp_ns'])-bad_start)*1e-9;loss_count+=1
        for k in sorted(candidate_frames,key=int):
            data=[r for r in candidates if r['frame']==k]
            gt=ref(name,data[0]['stamp_ns'])
            vals=[error(np.fromstring(r['end_matrix'],sep=';').reshape(4,4),gt) for r in data] if gt is not None else [None]*len(data)
            cover=None if gt is None else any(x<=.15 and y<=2 for x,y in vals)
            counts['coverage_n']+=int(gt is not None);counts['covered']+=int(cover is True)
            for method in ('full_native','max_native','mean_native','park','shared_native'):
                if method=='shared_native' and int(data[0]['n_points'])==0:continue
                order=sorted(range(len(data)),key=lambda i:(-float(data[i][method]),i))
                if method=='full_native' and gt is not None:counts['native_top1']+=int(vals[order[0]][0]<=.15 and vals[order[0]][1]<=2)
                for num,i in enumerate(order,1):
                    e=vals[i]
                    rank.append(dict(run=name,method=method,rank=num,**data[i],
                        score=float(data[i][method]),coverage=cover,pose_translation_error_m=None if not e else e[0],
                        pose_rotation_error_deg=None if not e else e[1]))
        for r in read(p/'refinements.csv'):
            gt=ref(name,r['stamp_ns'])
            if gt is not None:
                for g0 in ('R0','R1','R2'):
                    e=error(np.fromstring(r['reference_free_'+g0],sep=';').reshape(4,4),gt)
                    r[g0+'_translation_m'],r[g0+'_rotation_deg']=e
            refinements.append(dict(run=name,**r))
        t=np.array([float(r['total_s']) for r in trace])
        a=np.array([float(r['normal_s']) for r in trace])
        active=np.array([int(r['score_calls'])>0 or r['geometry_state']=='SEARCH_TIMEOUT' for r in decisions])
        wall,user,system,mem=(runs/(name+'.wall')).read_text().split()
        runtime.append(dict(run=name,n=len(t),wall_s=wall,user_s=user,system_s=system,peak_rss_kib=mem,
             total_mean_s=t.mean(),total_p95_s=np.quantile(t,.95),total_max_s=t.max(),
             native_mean_s=a.mean(),native_p95_s=np.quantile(a,.95),enhanced_frames=active.sum(),
             enhanced_mean_s=t[active].mean() if active.any() else None,enhanced_max_s=t[active].max() if active.any() else None,
             trigger_fraction=counts['triggers']/len(t),fixed_grid_scores=sum(int(r['score_calls']) for r in decisions),
             local_seed_fits=sum(int(r['candidate_fits']) for r in decisions)))
        for i,(stamp,state,gt) in enumerate(synced):
            match=next(((x,y,z) for x,y,z in synced[i+1:] if x>=stamp+950_000_000),None)
            if match and match[0]-stamp<=1_050_000_000:
                rpe_pairs.append(error(np.linalg.inv(state)@match[1],np.linalg.inv(gt)@match[2]))
        meas=np.array(errs)
        rpe=np.array(rpe_pairs)
        row=dict(run=name,frames=len(trace),reference_frames=len(errs),tracking_coverage=1.0,ukf_commits=len(trace)-counts['propagate'],
                 duplicate=0,propagate=counts['propagate'],low_support=counts['low_support'],timeout=counts['late'],
                 candidate_covered=counts['covered'],candidate_reference_event_count=counts['coverage_n'],native_top1=counts['native_top1'],
                 loss_s=loss if len(errs) else '',loss_events=loss_count if len(errs) else '',
                 rpe_pairs=len(rpe_pairs),rpe_1s_translation_rmse_m=np.sqrt(np.mean(rpe[:,0]**2)) if len(rpe) else '',
                 rpe_1s_rotation_rmse_deg=np.sqrt(np.mean(rpe[:,1]**2)) if len(rpe) else '',
                 wrong_native_accept_count=counts['wrong_native_accept'] if len(errs) else '',
                 beneficial_native_reject_count=counts['beneficial_native_reject'] if len(errs) else '',
                 wrong_extended_accept='NOT_EVALUABLE_WITHOUT_RELIABLE_PROPOSALS')
        if len(errs):
            row.update(ate_rmse_m=np.sqrt(np.mean(meas[:,0]**2)),translation_mean_m=meas[:,0].mean(),
                       rotation_mean_deg=meas[:,1].mean(),translation_p95_m=np.quantile(meas[:,0],.95),
                       translation_max_m=meas[:,0].max(),rotation_p95_deg=np.quantile(meas[:,1],.95),rotation_max_deg=meas[:,1].max(),
                       accurate_frames=np.count_nonzero((meas[:,0]<=.15)&(meas[:,1]<=2)))
        summary.append(row)
    for ax in axes:
        ax.legend(fontsize=6);ax.set_xlabel('Map x (m)');ax.set_ylabel('Map y (m)')
    fig.savefig(out/'trajectory_comparison.png',dpi=130)
    for name,rows in [('ACTIVE_LOCALIZATION_RESULTS',summary),('candidate_ranking',rank),('SHADOW_DECISIONS',shadow),
                      ('filter_update_log',filters),('runtime_breakdown',runtime),('degeneracy_runtime',deg)]:
        write(out/(name+'.csv'),rows)
    write(out/'refinement_results.csv',refinements or [dict(status='NOT_RUN',reason='No real candidate passed frozen geometry support/reliability gate')])

if __name__=='__main__':resolve(*map(Path,sys.argv[1:3]))
