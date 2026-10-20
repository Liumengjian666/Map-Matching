#!/usr/bin/env python3
"""Read-only blocked-state checks. No registration, bag replay, or new selection."""
import csv
import hashlib
import json
from pathlib import Path

DOC=Path(__file__).resolve().parent
REPO=DOC.parents[1]


def read_csv(name):
    with (DOC/name).open() as stream:return list(csv.DictReader(stream))


def main():
    result=json.loads((DOC/'results.json').read_text());pre=read_csv('input_preflight.csv')
    assert len(pre)==100 and sum(r['evaluable']=='0' for r in pre)==96
    assert all((int(r['training_points'])>=300 and all(int(r['val'+str(b)])>=10 for b in range(8)))==(r['evaluable']=='1') for r in pre)
    assert result['counts']['natural_trigger_count'] is None and result['counts']['new_native_aligns']==0
    assert result['scientific_classification']=='NOT_EVALUABLE_A_B_C_D_PREREQUISITES_NOT_MET'
    assert len(read_csv('NATURAL_SHADOW_RESULTS.csv'))==100
    diag=read_csv('DIAGNOSTIC_PROBE_RESULTS.csv')
    assert [int(r['scan_index']) for r in diag]==[20,40,60,80]
    assert all(r['status']=='NOT_RUN_FORMAL_ZERO_TRIGGER_CONDITION_NOT_ESTABLISHED' for r in diag)
    for name in ['CANDIDATE_POOL.csv','HELDOUT_BLOCK_LOSSES.csv','SELECTOR_COMPARISON.csv']:assert read_csv(name)==[]
    manifest=json.loads((DOC/'REAL_INPUT_MANIFEST.json').read_text())
    for row,source in zip(pre,manifest['raw_scans']):
        assert row['source_fnv']==source['expected_processed_fnv'] and int(row['source_points'])==source['expected_processed_points']
    prereg=DOC/'WINDOW_PRE_REGISTRATION.json'
    assert hashlib.sha256(prereg.read_bytes()).hexdigest()==result['pre_registration_SHA256']
    freeze=json.loads((DOC/'output_freeze.json').read_text())
    for row in freeze['files']:
        path=Path(row['path']);assert path.stat().st_size==row['bytes']
        assert hashlib.sha256(path.read_bytes()).hexdigest()==row['sha256']
    checksum=DOC/'checksums.sha256'
    if checksum.exists():
        for line in checksum.read_text().splitlines():
            digest,path=line.split('  ',1);assert hashlib.sha256((REPO/path).read_bytes()).hexdigest()==digest,path
    print('BLOCKED_STATE_READONLY_CHECKS_PASS real_aligns=0 natural_events=NOT_EVALUABLE diagnostics=NOT_RUN')


if __name__=='__main__':main()
