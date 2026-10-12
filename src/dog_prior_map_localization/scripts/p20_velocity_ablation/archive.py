"""Bounded P20 evidence archive, preserving all previous runs."""
import argparse
import csv
import json
import shutil
from pathlib import Path
from audit import sha,rows


def archive(repo,cache,code_sha):
    out=repo/'docs/p20_r1_visual_velocity_feedback_ablation';inventory=[]
    load=lambda p:json.loads(p.read_text())
    complete=load(cache/'audit_checked/completeness.json');mechanism=load(cache/'mechanism_checked/mechanism.json')
    metrics=load(cache/'posthoc_final/posthoc_metrics.json')
    if complete['EXPERIMENT_CONTRACT']!='EXPERIMENT_CONTRACT_PASS' or mechanism['VELOCITY_INTERVENTION']!='VELOCITY_INTERVENTION_CONFIRMED':
        raise ValueError('ARCHIVE_VALIDITY_GATE')
    def copy(source,name):
        target=out/name
        if target.exists():raise ValueError('TARGET_EXISTS:'+str(target))
        target.parent.mkdir(parents=True,exist_ok=True)
        if source.suffix=='.csv':
            with source.open(newline='') as f:data=list(csv.reader(f))
            with target.open('w',newline='') as f:csv.writer(f,lineterminator='\n').writerows(data)
        elif source.name=='launch.log':
            # Archive hygiene only; raw replay output and freeze unchanged.
            target.write_text('\n'.join(line.rstrip() for line in source.read_text().splitlines())+'\n')
        else:shutil.copyfile(source,target)
        inventory.append(dict(source=str(source.resolve()),source_sha256=sha(source),archive=name,archive_sha256=sha(target)))
    for name in ('ndt_diagnostics.csv','ekf_lineage.csv','deskew.csv','oosm.csv','oosm_fixed_lag.csv','ekf_runtime.csv',
                 'resources.csv','execution.json','rosparams.yaml','parameter_parity.json','launch.log','master.log','record.log','playback.log'):
        copy(cache/'runs/visual_blend0'/name,'runs/visual_blend0/'+name)
    for name in ('input_freeze.json','pre_gt_freeze.json','config_diff.txt'):copy(cache/name,name)
    groups={'audit_checked':['completeness.json'],'mechanism_checked':['mechanism.json','velocity_lineage.csv','first_divergence.csv'],
        'visual_audit':['visual_application_audit.json','visual_application_summary.csv'],
        'runtime_analysis':['runtime_metrics.json','runtime_summary.csv','resource_summary.csv']}
    for folder,names in groups.items():
        for name in names:copy(cache/folder/name,name)
    copy(cache/'runs/visual_blend0/parameter_parity.json','parameter_parity.json')
    for source in (cache/'posthoc_final').glob('*'):
        if source.is_file():copy(source,source.name)
    diag=rows(cache/'runs/visual_blend0/ndt_diagnostics.csv')
    quality=dict(total_NDT=396,extra_align=0,one_iteration=sum(r['ndt_iterations']=='1' for r in diag),
        iterations_mean=sum(int(r['ndt_iterations']) for r in diag)/len(diag),iterations_max=max(int(r['ndt_iterations']) for r in diag),
        translation_limited=sum(r['translation_limited']=='1' for r in diag),rotation_limited=sum(r['rotation_limited']=='1' for r in diag),
        step_limited=sum(r['translation_limited']=='1' or r['rotation_limited']=='1' for r in diag))
    engineering=metrics['engineering_screen']
    if engineering['rmse_within_5percent'] or engineering['p95_within_5percent']:
        raise ValueError('RESULT_CLASSIFICATION_REQUIRES_REVIEW')
    result=dict(TASK='P20-R1-VISUAL-VELOCITY-FEEDBACK-CAUSAL-ABLATION',FINAL_RESULT='BLEND_ZERO_REGRESSION',
        EXPERIMENT_CONTRACT=complete['EXPERIMENT_CONTRACT'],VELOCITY_INTERVENTION=mechanism['VELOCITY_INTERVENTION'],
        START_SHA='389243ae84cedb9a75964c8d43584cf60e233961',CODE_SHA=code_sha,
        runtime_source_base_sha='389243ae84cedb9a75964c8d43584cf60e233961',
        PUSH_EXECUTED=False,completeness=complete,ndt_quality=quality,
        mechanisms=mechanism,posthoc=metrics,runtime=load(cache/'runtime_analysis/runtime_metrics.json'),
        limitations=['One controlled-switch closed-loop replay; later state/source histories diverge',
            'Global max small improvement coexists with large RMSE/P95 regression',
            'NDT motion resumes forward but wrong relative positions and excessive EKF velocity remain',
            'Not proof beta=.6 optimal, unique cause, independent generalization, or originality',
            'Offline visual CSV; not a complete online visual resource benchmark'],
        NEXT='Proposal only: retain beta=.6 and test the no-visual NDT initial-translation source (historical NDT increment vs existing causal EKF prediction), one factor/one bounded run, no grid; requires decision AI authorization')
    (out/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    (out/'source_inventory.json').write_text(json.dumps(inventory,indent=2)+'\n')
    hashes={str(p.relative_to(out)):sha(p) for p in out.rglob('*') if p.is_file() and p.name!='artifact_hashes.json'}
    (out/'artifact_hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
    print(json.dumps(dict(files=len(hashes),quality=quality),indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True);p.add_argument('--cache',type=Path,required=True)
    p.add_argument('--code-sha',required=True);archive(**vars(p.parse_args()))
