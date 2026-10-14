#!/usr/bin/env python3
"""Check the blocked P32 receipt and unchanged source; never starts an experiment."""
import argparse
import csv
import hashlib
import json
import subprocess
from pathlib import Path

def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1<<20),b''): h.update(block)
    return h.hexdigest()

def main(seal=False):
    doc=Path(__file__).resolve().parent;repo=doc.parents[1]
    result=json.loads((doc/'results.json').read_text())
    assert result['FINAL_RESULT']=='DATA_NOT_READY' and result['selected_scene'] is None
    assert not result['algorithm_math_modified'] and not result['algorithm_parameters_modified']
    assert not result['runtime_source_modified'] and not result['GT_evaluator_executed']
    assert result['short_replays_executed']==result['full_replays_executed']==result['NDT_align_calls_executed']==0
    for group in ('F','G'):
        assert result[group]['status']=='NOT_RUN_DATA_CONTRACT_FAILED'
        for field in ('NDT','OOSM','MSCKF_updates','translation_metrics','rotation_metrics','RPE'):
            assert result[group][field] is None
    assert result['absolute_map_to_GT_transform_is_mandatory'] is False
    assert result['literal_initial_timestamp_field_is_mandatory'] is False
    assert result['official_only_semantic_proof_is_mandatory'] is False
    review=json.loads((doc/'external_review'/'execution.json').read_text())
    assert result['external_review']['authorized'] is True
    assert result['external_review']['invocations']==review['invocations']==1
    assert result['external_review']['exit_code']==review['exit_code']==0
    for field in ('input','result'):
        assert sha(doc/'external_review'/review[field+'_file'])==review[field+'_SHA256']
    for filename in ('accuracy_comparison.csv','visual_update_summary.csv','runtime_summary.csv','resource_summary.csv'):
        with (doc/filename).open(newline='') as stream:
            records=list(csv.DictReader(stream))
        assert [r['group'] for r in records]==['F','G']
        for row in records:
            assert None not in row,filename
            assert row['status'].startswith(('NOT_RUN','NOT_MEASURED')),filename
            assert all(v=='' for k,v in row.items() if k not in ('group','status','reason')),filename
    freeze=json.loads((doc/'algorithm_freeze.json').read_text())
    for relative,digest in freeze['files_sha256'].items(): assert sha(repo/relative)==digest,relative
    changed=subprocess.check_output(['git','-C',str(repo),'diff',result['CODE_SHA'],'--name-only','--','src'],text=True)
    assert not changed,'Runtime/source changed during data gate'
    evidence=json.loads((doc/'asset_evidence.json').read_text())
    assert sum(evidence['Corridor02']['bag']['counts'].values())==evidence['Corridor02']['bag']['message_count']
    assert len(evidence['Corridor02']['first_messages'])==3
    assert evidence['Corridor02']['bag']['full_SHA256_rechecked'] is False
    assert evidence['Floor01']['ssd_map']['hash_rechecked'] is True
    paths=sorted(p for p in doc.rglob('*') if p.is_file() and p.name!='artifact_hashes.json')
    assert not any(p.suffix in ('.bag','.pcd','.xyz','.f64','.o','.so') for p in paths)
    target=doc/'artifact_hashes.json'
    if seal:
        target.write_text(json.dumps({str(p.relative_to(repo)):sha(p) for p in paths},indent=2)+'\n')
    else:
        hashes=json.loads(target.read_text())
        assert set(hashes)=={str(p.relative_to(repo)) for p in paths},'Manifest file set differs'
        for relative,digest in hashes.items(): assert sha(repo/relative)==digest,relative
    print('P32_BLOCKED_ARCHIVE_CHECK_PASS: honest NOT_RUN metrics, unchanged algorithms, evidence and light hashes')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--seal',action='store_true');main(p.parse_args().seal)
