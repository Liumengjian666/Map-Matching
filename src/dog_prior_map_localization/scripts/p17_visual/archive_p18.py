"""Archive measured P18 receipts/CSVs, not bags, maps or invented outputs."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def read(path):return json.loads(path.read_text())


def rows(path):
    with path.open(newline='') as f:return list(csv.DictReader(f))


def copy_text(source,target):
    target.parent.mkdir(parents=True,exist_ok=True)
    # Formatting-only LF normalization; raw cache and pre-GT hashes remain.
    target.write_bytes(source.read_bytes().replace(b'\r\n',b'\n'))


def main():
    p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True)
    p.add_argument('--cache',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();root=args.output;root.mkdir(parents=True,exist_ok=True)
    for run in ('control_short','visual_short','visual_full'):
        for source in (args.cache/'runs'/run).glob('*'):
            if source.suffix in ('.csv','.json','.yaml'):copy_text(source,root/'runs'/run/source.name)
    for folder in ('control_parity','visual_smoke_audit','visual_smoke_audit_checked','visual_full_audit','runtime_audit','posthoc_evaluation'):
        for source in (args.cache/folder).glob('*'):
            if source.suffix in ('.csv','.json'):copy_text(source,root/folder/source.name)
    copy_text(args.cache/'pre_gt_freeze.json',root/'pre_gt_freeze.json')
    for folder,name in [('control_parity','control_parity.csv'),('visual_full_audit','visual_application_summary.csv'),
                        ('posthoc_evaluation','accuracy_comparison.csv'),('runtime_audit','runtime_summary.csv'),
                        ('runtime_audit','resource_summary.csv')]:
        copy_text(args.cache/folder/name,root/name)
    post=read(args.cache/'posthoc_evaluation/posthoc_metrics.json')
    audit=read(args.cache/'visual_full_audit/visual_application_audit.json')
    safety={};ndt={}
    for label,path in [('Control',args.repo/'docs/p10_corridor01_deterministic_oosm_replay_r1/runs/control_v2'),
                       ('Visual',args.cache/'runs/visual_full')]:
        states=[r for r in rows(path/'ekf_lineage.csv') if r['event_type']=='IMU_HIGH_RATE_PUBLISH']
        pp=np.array([[float(r[k]) for k in ('p_x','p_y','p_z')] for r in states])
        qq=Rotation.from_quat(np.array([[float(r[k]) for k in ('q_x','q_y','q_z','q_w')] for r in states]))
        dt=np.linalg.norm(np.diff(pp,axis=0),axis=1);dr=np.degrees((qq[:-1].inv()*qq[1:]).magnitude())
        safety[label]=dict(max_highrate_translation_jump_m=float(dt.max()),
                          max_highrate_rotation_jump_deg=float(dr.max()),
                          jumps_over_existing_2m_or_20deg=int(((dt>2)|(dr>20)).sum()))
        diag=rows(path/'ndt_diagnostics.csv')
        ndt[label]=dict(frames=len(diag),converged=sum(r['ndt_has_converged']=='1' for r in diag),
            iterations_mean=float(np.mean([float(r['ndt_iterations']) for r in diag])),
            iterations_max=max(int(r['ndt_iterations']) for r in diag),
            fitness_mean=float(np.mean([float(r['ndt_fitness']) for r in diag])),
            step_limited=sum(r['translation_limited']=='1' or r['rotation_limited']=='1' for r in diag))
    groups=[]
    for method,components in post['methods'].items():
        for component in ('Initial_Guess','Raw_NDT','Final_Used'):
            for label in ('applied_frames','not_applied_frames'):
                m=components[component][label]
                groups.append(dict(method=method,component=component,group=label,count=m['translation_m']['count'],
                    translation_rmse=m['translation_m']['rmse'],translation_p95=m['translation_m']['p95'],
                    translation_max=m['translation_m']['max'],rotation_rmse=m['rotation_deg']['rmse']))
    with (root/'application_conditioned_measurements.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(groups[0]),lineterminator='\n');w.writeheader();w.writerows(groups)
    # A high-rate state has no new camera observation at each IMU tick.
    # Classify by its latest accepted NDT measurement, using the same
    # Visual-derived time partition for both trajectories, never GT labels.
    origin=post['origin']
    app=rows(args.cache/'visual_full_audit/visual_application_summary.csv')
    applied={float(r['relative_time_s'])+origin for r in app if r['applied']=='1'}
    first=min(applied);partition={}
    for r in rows(args.cache/'runs/visual_full/ekf_lineage.csv'):
        if r['event_type']!='IMU_HIGH_RATE_PUBLISH':continue
        rel=float(r['state_stamp'])-origin
        if not 3<=rel<=35:continue
        latest=float(r['last_ndt_observation_time'])
        partition[rel]=('LATEST_MEASUREMENT_VISUAL' if latest in applied else
                        'NO_DIRECT_VISUAL_WITH_PAST_VISUAL' if latest>first else 'NO_VISUAL_HISTORY')
    errors=rows(args.cache/'posthoc_evaluation/per_sample_errors.csv');ekf_groups=[]
    for method in ('Control','Visual'):
        for group in ('LATEST_MEASUREMENT_VISUAL','NO_DIRECT_VISUAL_WITH_PAST_VISUAL','NO_VISUAL_HISTORY'):
            selected=[r for r in errors if r['method']==method and r['component']=='EKF_HighRate'
                      and float(r['relative_time_s']) in partition and partition[float(r['relative_time_s'])]==group]
            entry=dict(method=method,group=group,count=len(selected))
            for field in ('translation_error_m','rotation_error_deg'):
                x=np.array([float(r[field]) for r in selected])
                for stat in ('rmse','p95','max'):
                    entry[field+'_'+stat]=(float(np.sqrt(np.mean(x*x))) if stat=='rmse' else
                                          float(np.percentile(x,95)) if stat=='p95' else float(x.max())) if len(x) else None
            ekf_groups.append(entry)
    with (root/'application_conditioned_ekf.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(ekf_groups[0]),lineterminator='\n');w.writeheader();w.writerows(ekf_groups)
    result=dict(TASK='P18-FROZEN-MAP-RECOVERY-AND-VISUAL-NDT-REPLAY',
        FINAL_RESULT=post['FINAL_RESULT'],EXPERIMENT_VALID=True,
        NEXT='REPLICATE_FIXED_VISUAL_REPLAY_AND_VERIFY_TAIL_RISK',
        original_START_SHA='3b3d53da8fe755c5f3c24dd8c99008579625b7a0',
        resumed_START_SHA='e43b07006c52a08eba0aeb7c0e651e1813d5c844',
        CODE_SHA=read(args.cache/'pre_gt_freeze.json')['code_sha'],PUSH_EXECUTED=False,
        storage=read(root/'input_freeze.json')['map'],
        control_parity=read(args.cache/'control_parity/control_parity.json'),
        visual_smoke=read(args.cache/'visual_smoke_audit_checked/visual_application_audit.json'),
        visual_full=audit,posthoc=post,ndt=ndt,safety=safety,
        runtime=read(args.cache/'runtime_audit/runtime_metrics.json'),EKF_application_groups=ekf_groups,
        important_limit='Overall RMSE/P95 improved but primary translation maximum worsened; no direct vision in 26-35s',
        novelty_claim=False,realtime_camera_frontend=False)
    (root/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    artifacts={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()
               for p in root.rglob('*') if p.is_file() and p.name!='artifact_hashes.json'}
    (root/'artifact_hashes.json').write_text(json.dumps(dict(files_sha256=artifacts,
        raw_cache_retained=str(args.cache),archive_text_normalization='CRLF to LF only'),indent=2)+'\n')
    print('ARCHIVED',len(artifacts),'files',sum(p.stat().st_size for p in root.rglob('*') if p.is_file()),'bytes')


if __name__=='__main__':main()
