#!/usr/bin/env python3
"""Archive light real P30 evidence, never Sources/bags/maps/build products."""
import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'p29_feedback'))
from prepare import sha, rows, write_csv

def main(repo):
    cache=repo/'.p30_r1_scan_local_deskew';doc=repo/'docs/p30_r1_scan_local_deskew'
    result=json.loads((cache/'analysis_results.json').read_text())
    result.update(CODE_SHA=subprocess.check_output(['git','-C',str(repo),'rev-parse','HEAD'],text=True).strip(),
        PUSH_EXECUTED=False,END_SHA='See final delivery commit/chat; not self-referential in this committed artifact.',
        NEXT='Decision AI to freeze a minimal comparison separating ZOH convention alignment from historical-node isolation; not executed.')
    result['external_cli_review']=dict(invocations=1,exit_code=0,model='gpt-6.1-sol',reasoning_effort='xhigh',
        input_sha256='1e27be61ffb7c37adbcc31f6c47237d8470b421ee682a9516670d603e7d994e6',
        scope='P30 runtime/math source only; not final GT evaluation',result='No substantive issue.')
    for name in ('input_freeze.json','pre_gt_freeze.json','gate_short.json','gate_full.json','short.yaml','full.yaml'):
        shutil.copy2(cache/name,doc/name)
    for name in ('source_comparison_short.csv','source_comparison_full.csv','point_comparison_short.csv','point_comparison_full.csv',
                 'pose_comparison_short.csv','pose_comparison_full.csv','motion_summary_short.csv','motion_summary_full.csv',
                 'accuracy_metrics.csv','accuracy_comparison.csv','rpe_summary.csv','per_sample_errors.csv',
                 'runtime_summary.csv','resource_summary.csv','first_divergence.csv'):
        (doc/name).write_bytes((cache/name).read_bytes().replace(b'\r\n',b'\n'))
    module_counts=[]
    for stage in ('short','full'):
        source=cache/'runs'/stage;dest=doc/'runs'/stage;dest.mkdir(parents=True,exist_ok=True)
        for name in ('execution.json','rosparams.yaml','ndt_diagnostics.csv','mature_diagnostics.csv','ekf_lineage.csv',
                     'deskew.csv','oosm.csv','oosm_fixed_lag.csv','msckf_diagnostics.csv','resources.csv','scan_local_motion.csv',
                     'sqrt_covariance_summary.csv','sqrt_covariance_scale.csv','deskew_capture.json'):
            payload=(source/name).read_bytes()
            if name.endswith('.csv'):payload=payload.replace(b'\r\n',b'\n')
            (dest/name).write_bytes(payload)
        meta=doc/'source_metadata'/stage;meta.mkdir(parents=True,exist_ok=True)
        for path in (cache/'sources'/stage).glob('*.source.json'):shutil.copy2(path,meta/path.name)
        m=rows(source/'mature_diagnostics.csv')
        module_counts.append(dict(stage=stage,DCReg_valid=sum(r['dcreg_status']=='DCREG_DETECTION_ADAPTED' for r in m),
            quick_scores=sum(int(r['quick_score_calls']) for r in m),extra_aligns=sum(int(r['extra_aligns']) for r in m),
            replacements=sum(int(r['selected_candidate'])>=0 for r in m)))
    write_csv(doc/'module_counts.csv',module_counts)
    front=repo/'docs/p28_r2_mature_visual_module/visual_inputs/extraction.json'
    result['historical_frontend_cost']=json.loads(front.read_text())
    result['module_counts']=module_counts
    (doc/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    external=doc/'external_review';external.mkdir(exist_ok=True)
    for name in ('review_input.md','review_result.md'):shutil.copy2(Path('/tmp/p30-cli-review.1IHEyo')/name,external/name)
    if sha(external/'review_input.md')!=result['external_cli_review']['input_sha256']:
        raise ValueError('AUTHORIZED_EXTERNAL_INPUT_CHANGED')
    if (external/'review_result.md').read_text().strip()!='No substantive issue.':
        raise ValueError('EXTERNAL_VERDICT_NOT_AS_RECORDED')
    files=sorted(p for p in doc.rglob('*') if p.is_file() and p.name!='artifact_hashes.json')
    (doc/'artifact_hashes.json').write_text(json.dumps({str(p.relative_to(repo)):sha(p) for p in files},indent=2)+'\n')
    print(json.dumps(dict(FINAL_RESULT=result['FINAL_RESULT'],CODE_SHA=result['CODE_SHA'],files=len(files),module_counts=module_counts)))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True);main(**vars(p.parse_args()))
