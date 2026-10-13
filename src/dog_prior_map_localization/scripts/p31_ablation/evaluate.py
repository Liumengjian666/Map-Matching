#!/usr/bin/env python3
"""P31 bounded posthoc evaluation: a shared frozen transform, never refit."""
import argparse
import importlib.util
import json
import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'p29_feedback'))
from prepare import rows,sha,write_csv
from analyze import frozen,G,summary
spec=importlib.util.spec_from_file_location('p30_frozen_evaluation',Path(__file__).resolve().parents[1]/'p30_deskew/evaluate.py')
p30=importlib.util.module_from_spec(spec);spec.loader.exec_module(p30)
ndt_durations=p30.ndt_durations

CACHE='.p31_r1_deskew_contribution_visual_ablation'

def freeze(repo):
    cache=repo/CACHE;path=cache/'pre_gt_freeze.json'
    if path.exists(): raise ValueError('P31_FREEZE_EXISTS')
    for stage in ('D_short','D','E'):
        gate=json.loads((cache/('gate_'+stage+'.json')).read_text())
        if gate['status']!='PASS' or gate['GT_LOADED']: raise ValueError('P31_GATE_REQUIRED')
    paths=[p for p in (cache/'runs').rglob('*') if p.is_file()]
    paths+=list(cache.glob('*.yaml'))+list(cache.glob('gate_*.json'))+[cache/'input_freeze.json']
    data=dict(protocol='P31_PRE_GT_OUTPUT_FREEZE',GT_LOADED=False,
              completed=True,files_sha256={str(p):sha(p) for p in sorted(paths)})
    path.write_text(json.dumps(data,indent=2)+'\n'); print('P31_PRE_GT_FREEZE_PASS')

def evaluate(repo):
    cache=repo/CACHE
    receipt=json.loads((cache/'pre_gt_freeze.json').read_text())
    if receipt['protocol']!='P31_PRE_GT_OUTPUT_FREEZE' or receipt['GT_LOADED'] or not receipt['completed']:
        raise ValueError('P31_OUTPUT_FREEZE_REQUIRED')
    inputs=json.loads((cache/'input_freeze.json').read_text())
    for inventory in (inputs['files_sha256'],receipt['files_sha256']):
        for path,digest in inventory.items():
            if sha(path)!=digest: raise ValueError('P31_FROZEN_ARTIFACT_CHANGED:'+path)
    contract_path=repo/'docs/p10_corridor01_deterministic_oosm_replay_r1/posthoc_metrics.json'
    contract=json.loads(contract_path.read_text());origin=contract['evaluation_origin_unix_s']
    R=np.array(contract['fit_transform_rotation']);t=np.array(contract['fit_transform_translation_m'])
    gt=repo/'.p18_map_recovery/posthoc_inputs/frozen.YOuGEy/corridor01_gt.txt';ext=gt.parent/'corridor01_extrinsics.yaml'
    if sha(gt)!='3cabcc78ecea4d991aa6e3eddb811cefc4fdacf5f3387b98950fa09ad338dd03' or sha(ext)!='59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d':
        raise ValueError('P31_GT_OR_EXTRINSIC_CHANGED')
    gt_times,gt_poses=G.load_gt(gt);Til=G.load_t_imu_lidar(ext)
    prior=json.loads((repo/'docs/p30_r1_scan_local_deskew/results.json').read_text())
    metrics=list(prior['metrics']);rpes=list(prior['rpe']);stability=list(prior['stability'])
    runtime=list(prior['runtime']);resources=list(prior['resources']);samples=[]
    roots={m:repo/'.p29_r1_fixed_source_feedback/runs'/m for m in ('A','B')}
    roots.update(C=repo/'.p30_r1_scan_local_deskew/runs/full',D=cache/'runs/D',E=cache/'runs/E')
    for method in ('D','E'):
        root=roots[method];ndt=rows(root/'ndt_diagnostics.csv')
        layers={'EKF_HighRate':frozen.state_trajectory(root/'ekf_lineage.csv')[0]}
        for name,prefix in [('Initial_Guess','initial_guess_'),('Raw_NDT','raw_ndt_'),('Final_Used','final_used_')]:
            layers[name]=frozen.measurement_trajectory(ndt,prefix,Til,origin)
        for layer,trajectory in layers.items():
            pairs=frozen.pair_with_gt(trajectory,gt_times,gt_poses,origin,35)
            times,et,er,_=frozen.summarize_window(pairs,R,t)
            for window,lo,hi in [('3_35',3,35),('26_35',26,35),('34_35',34,35)]:
                mask=(times>=lo)&(times<=hi);tr,rr=summary(et[mask]),summary(er[mask])
                metrics.append(dict(method=method,layer=layer,window=window,count=int(mask.sum()),
                    translation_rmse_m=tr['rmse'],translation_p95_m=tr['p95'],translation_max_m=tr['maximum'],
                    rotation_rmse_deg=rr['rmse'],rotation_p95_deg=rr['p95'],rotation_max_deg=rr['maximum'],
                    over_1m=int((et[mask]>1).sum()),over_2m=int((et[mask]>2).sum()),over_5m=int((et[mask]>5).sum())))
            if layer!='EKF_HighRate': continue
            for i in np.flatnonzero(times>=3):
                samples.append(dict(method=method,stamp=origin+times[i],relative_s=times[i],translation_error_m=et[i],rotation_error_deg=er[i]))
            te=[];re=[];interval=[]
            for i,record in enumerate(pairs):
                if times[i]<3: continue
                j=int(np.searchsorted(times,times[i]+1))
                if j>=len(pairs) or abs(times[j]-times[i]-1)>0.0051: continue
                pe,Re=record[1];pg,Rg=record[2];qe,Qe=pairs[j][1];qg,Qg=pairs[j][2]
                te.append(np.linalg.norm(Re.T@(qe-pe)-Rg.T@(qg-pg)))
                re.append(G.rotation_angle_deg((Rg.T@Qg).T@(Re.T@Qe)));interval.append(times[j]-times[i])
            tr,rr=summary(te),summary(re)
            rpes.append(dict(method=method,count=len(te),nominal_sec=1.,actual_min_sec=min(interval),actual_max_sec=max(interval),
                translation_rmse_m=tr['rmse'],translation_p95_m=tr['p95'],translation_max_m=tr['maximum'],
                rotation_rmse_deg=rr['rmse'],rotation_p95_deg=rr['p95'],rotation_max_deg=rr['maximum']))
            jumps=np.linalg.norm(np.diff(np.array([p[1][0] for p in pairs]),axis=0),axis=1)
            peak=int(np.argmax(et));stability.append(dict(method=method,max_error_time_s=float(times[peak]),
                max_error_m=float(et[peak]),max_adjacent_jump_m=float(jumps.max()),jumps_gt2m=int((jumps>2).sum()),
                persistent_1m=frozen.persistent(times,et,1),persistent_2m=frozen.persistent(times,et,2)))
        for file,field,label in [('deskew.csv','deskew_processing_ms','Deskew'),('msckf_diagnostics.csv','update_ms','MSCKF_processing')]:
            records=rows(root/file)
            if label=='MSCKF_processing': records=[r for r in records if r['status']=='APPLIED_REPROJECTION_SELECTED_GAIN']
            values=np.array([float(r[field]) for r in records])
            runtime.append(dict(method=method,stage=label,count=len(values),mean_ms=float(values.mean()) if len(values) else None,
                p95_ms=float(np.percentile(values,95)) if len(values) else None,max_ms=float(values.max()) if len(values) else None,
                timing_measured=bool(len(values))))
        for key,values in ndt_durations(root).items():
            runtime.append(dict(method=method,stage='NDT_align' if key=='align_time_ms' else 'NDT_callback',count=len(values),
                mean_ms=float(np.mean(values)),p95_ms=float(np.percentile(values,95)),max_ms=float(np.max(values))))
        execution=json.loads((root/'execution.json').read_text())
        for name,resource in execution['resources'].items():
            resources.append(dict(method=method,process=name,peak_rss_bytes=resource['peak_rss_bytes'],cpu_sec=resource['cpu_sec'],replay_wall_sec=execution['wall_sec']))
    comparisons=[]
    for left,right,label in [('B','D','propagation_convention'),('D','C','scan_start_continuity'),('E','C','visual_update')]:
        for a in [r for r in metrics if r['method']==left]:
            b=next(r for r in metrics if r['method']==right and r['layer']==a['layer'] and r['window']==a['window'])
            if b['count']!=a['count']: raise ValueError('P31_UNPAIRED_COUNTS')
            for key in ('translation_rmse_m','translation_p95_m','translation_max_m','rotation_rmse_deg','rotation_p95_deg','rotation_max_deg'):
                comparisons.append(dict(contrast=label,left=left,right=right,layer=a['layer'],window=a['window'],metric=key,
                    before=a[key],after=b[key],delta=b[key]-a[key],relative_percent=(b[key]-a[key])/a[key]*100,count=a['count']))
    divergence=[];ndts={m:rows(root/'ndt_diagnostics.csv') for m,root in roots.items()}
    for left,right in [('B','D'),('D','C'),('E','C')]:
        for a,b in zip(ndts[left],ndts[right]):
            if a['cloud_hash']!=b['cloud_hash']:
                divergence.append(dict(contrast=left+right,layer='Source',frame_index=a['frame_index'],stamp=a['lidar_header_stamp'],left_hash=a['cloud_hash'],right_hash=b['cloud_hash']));break
        for label,prefix in [('Initial','initial_guess_'),('Raw','raw_ndt_'),('Used','final_used_')]:
            for a,b in zip(ndts[left],ndts[right]):
                ap,ar=frozen.pose_from_fields(a,prefix);bp,br=frozen.pose_from_fields(b,prefix)
                ap,ar=G.lidar_pose_to_imu(dict(position=ap,rotation=ar),Til);bp,br=G.lidar_pose_to_imu(dict(position=bp,rotation=br),Til)
                dp=float(np.linalg.norm(bp-ap));dr=float(np.degrees((Rotation.from_matrix(ar).inv()*Rotation.from_matrix(br)).magnitude()))
                if dp>1e-9 or dr>1e-8:
                    divergence.append(dict(contrast=left+right,layer=label,frame_index=a['frame_index'],stamp=a['lidar_header_stamp'],translation_delta_m=dp,rotation_delta_deg=dr));break
    for name,data in [('accuracy_metrics',metrics),('accuracy_comparison',comparisons),('rpe_summary',rpes),('stability_summary',stability),
                      ('per_sample_errors',samples),('runtime_summary',runtime),('resource_summary',resources),('first_divergence',divergence)]:
        write_csv(cache/(name+'.csv'),data)
    result=dict(TASK='P31-R1-DESKEW-CONTRIBUTION-VISUAL-ABLATION',START_SHA=inputs['START_SHA'],GT_LOADED=True,
        gates={s:json.loads((cache/('gate_'+s+'.json')).read_text()) for s in ('D_short','D','E')},
        metrics=metrics,comparisons=comparisons,rpe=rpes,stability=stability,runtime=runtime,resources=resources,
        first_divergence=divergence,evaluation_contract_sha256=sha(contract_path),GT_sha256=sha(gt),extrinsic_sha256=sha(ext),
        shared_Control_V2_transform=True,historical_reference='P30 archived metrics A/B/C, unchanged',
        short_runs=1,full_runs=2,GT_parameter_search=False,independent_contribution_fractions_claimed=False,
        FINAL_RESULT='DESKEW_CONTRIBUTION_AND_VISUAL_ABLATION_COMPLETE')
    (cache/'analysis_results.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(primary=[r for r in metrics if r['layer']=='EKF_HighRate' and r['window']=='3_35'],stability=stability,rpe=rpes),indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True);p.add_argument('--freeze',action='store_true')
    a=p.parse_args();freeze(a.repo.resolve()) if a.freeze else evaluate(a.repo.resolve())
