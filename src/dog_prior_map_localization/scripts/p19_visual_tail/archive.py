"""Archive only bounded P19 evidence, never bag/PCD or earlier results."""
import argparse
import csv
import hashlib
import json
import shutil
from pathlib import Path
from compare import rows, write_csv


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(1<<20),b''):h.update(b)
    return h.hexdigest()


def archive(repo, cache, code_sha):
    out=repo/'docs/p19_r1_visual_replication_tail_risk'
    replication=json.loads((cache/'replication_final/replication.json').read_text())
    complete=json.loads((cache/'visual_full_audit/visual_application_audit.json').read_text())
    if replication['REPLICATION_RESULT']!='REPLAY_REPLICATION_PASS' or complete['result']!='PASS':
        raise ValueError('ARCHIVE_SUCCESS_GATE_FAILED')
    if len(rows(cache/'tail_analysis/tail_peak_events.csv'))!=30:
        raise ValueError('TAIL_PEAK_EVIDENCE_INCOMPLETE')
    inventory=[]
    def copy(source,destination):
        target=out/destination
        if target.exists():raise ValueError('ARCHIVE_TARGET_EXISTS:'+str(target))
        target.parent.mkdir(parents=True,exist_ok=True)
        if source.suffix=='.csv':
            with source.open(newline='') as f:data=list(csv.reader(f))
            with target.open('w',newline='') as f:csv.writer(f,lineterminator='\n').writerows(data)
        else:shutil.copyfile(source,target)
        inventory.append(dict(source=str(source.resolve()),archive=destination,
            source_sha256=digest(source),archive_sha256=digest(target),size_bytes=target.stat().st_size))
    for name in ('ndt_diagnostics.csv','ekf_lineage.csv','oosm.csv','oosm_fixed_lag.csv','deskew.csv',
                 'ekf_runtime.csv','resources.csv','execution.json','rosparams.yaml','launch.log','master.log','record.log','playback.log'):
        copy(cache/'runs/repeat_visual'/name,'runs/repeat_visual/'+name)
    groups={'replication_final':['replication.json','replication_comparison.csv'],
        'tail_analysis':['tail_metrics.json','tail_error_timeline.csv','tail_window_summary.csv','tail_peak_events.csv'],
        'lineage_final':['lineage_metrics.json','causal_lineage_summary.csv'],
        'runtime_analysis':['runtime_metrics.json','runtime_summary.csv','resource_summary.csv'],
        'visual_full_audit':['visual_application_audit.json','visual_application_summary.csv']}
    for folder,names in groups.items():
        for name in names:copy(cache/folder/name,name)
    for name in ('pre_gt_freeze.json','evaluation_replay_receipt.json'):copy(cache/name,name)
    for source in (cache/'frozen_analysis_code').glob('*.py'):copy(source,'frozen_analysis_code/'+source.name)
    for source in (cache/'posthoc_evaluation').glob('*'):
        if source.is_file():copy(source,'posthoc_evaluation/'+source.name)
    p18=repo/'docs/p18_frozen_map_visual_ndt_replay/resume_20261012_1004/posthoc_evaluation'
    metrics=[]
    for path in (p18/'method_summary.csv',cache/'posthoc_evaluation/method_summary.csv'):
        for r in rows(path):
            if path.parent==p18:r['method']='P18_Visual' if r['method']=='Visual' else 'Control'
            elif r['method']=='Visual':r['method']='P19_Repeat'
            else:continue
            metrics.append(r)
    write_csv(out/'combined_method_summary.csv',metrics)
    load=lambda p:json.loads(p.read_text())
    result=dict(TASK='P19-R1-VISUAL-REPLAY-REPLICATION-AND-TAIL-RISK',
        FINAL_RESULT='VISUAL_REPLICATED_TAIL_RISK_CHARACTERIZED',
        REPLICATION='PASS',TAIL_RISK_ANALYSIS='PASS',
        START_SHA='5df7f8c729dd2ff3d53ffe44f25d183963be87fb',CODE_SHA=code_sha,
        PUSH_EXECUTED=False,replication=replication,
        completeness=complete,
        metrics_equal_to_P18=(load(p18/'posthoc_metrics.json')['methods']==load(cache/'posthoc_evaluation/posthoc_metrics.json')['methods']),
        runtime=load(cache/'runtime_analysis/runtime_metrics.json'),tail=load(cache/'tail_analysis/tail_metrics.json'),
        lineage=load(cache/'lineage_final/lineage_metrics.json'),
        limitations=['Prefix-aligned relative drift, not map-frame absolute ATE or independent generalization',
            'Distinct feedback histories including deskewed sources; not isolated framewise causal effects',
            'Tail stall and propagation observed; unique cause/true Hessian weak direction not established',
            'Offline visual CSV injection; full online vision resource budget not measured'],
        NEXT='Decision only: one Visual velocity_blend=0 causal ablation against frozen .6 to isolate the NDT-derived velocity feedback channel; no other change; not executed')
    (out/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    (out/'source_inventory.json').write_text(json.dumps(inventory,indent=2)+'\n')
    hashes={str(p.relative_to(out)):digest(p) for p in out.rglob('*') if p.is_file() and p.name!='artifact_hashes.json'}
    (out/'artifact_hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
    print(json.dumps(dict(archived_files=len(hashes),archive_bytes=sum(p.stat().st_size for p in out.rglob('*') if p.is_file())),indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True);p.add_argument('--cache',type=Path,required=True)
    p.add_argument('--code-sha',required=True);archive(**vars(p.parse_args()))
