#!/usr/bin/python3
"""Read-only R11 asset gate: inherited inputs plus frozen R5/R6/R9/R10 evidence."""
import argparse,hashlib,json,subprocess,sys
from pathlib import Path

HERE=Path(__file__).resolve().parent;REPO=HERE.parents[1]
sys.path.insert(0,str(REPO/'docs/p61_r9_geode_anomaly_readonly_attribution'))
from asset_gate import sha,verify
BASE='a0275decc7862030c3581038ab87d09efe5aaa4e'
R6=Path('/home/jian/livox_ws/p61_r6_geode_runs')


def main(label):
    inherited=verify(HERE/('r8_assets_'+label+'.json'));records=list(inherited['assets'])
    def check(name,path,expected,source,size=None):
        path=Path(path);actual=sha(path);n=path.stat().st_size
        row=dict(asset=name,path=str(path.resolve()),sha256=actual,expected_sha256=expected,
            size_bytes=n,frozen_source=source,status='PASS' if actual==expected and (size is None or n==size) else 'FAIL')
        records.append(row)
        if row['status']!='PASS':raise RuntimeError('INPUT_ASSET_MISMATCH:'+str(path))
    prefixes=['docs/p61_r9_geode_anomaly_readonly_attribution','docs/p61_r10_geode_gyro_increment_audit',
        'docs/p61_r6_geode_causal_init_bridge','docs/p61_r5_geode_frozen_prior_map']
    names=subprocess.check_output(['git','ls-tree','-r','--name-only',BASE,'--']+prefixes,cwd=REPO,text=True).splitlines()
    for name in names:
        blob=subprocess.check_output(['git','show',BASE+':'+name],cwd=REPO)
        check('frozen_Git_archive',REPO/name,hashlib.sha256(blob).hexdigest(),BASE,len(blob))
    m=json.loads((REPO/'docs/p61_r6_geode_causal_init_bridge/build_manifest.json').read_text())
    for f in m['source_files']:
        check('actual_R6_author_source',R6/'author_ws/src/FAST_LIO'/f['path'],f['sha256'],'R6 build_manifest',f['bytes'])
    check('R6_binary_hash_only',m['binary'],m['binary_sha256'],'R6 build_manifest')
    # Protect R10 input pin explicitly, independent of generic Git checks.
    check('R10_input_pin',REPO/'docs/p61_r10_geode_gyro_increment_audit/inputs_frozen.json',
        'a2f3b61260e727addc935fff38cdfee587a88ba806d4a3992e094af9bb8af690','R11 task')
    result=dict(status='PASS',comparisons=len(records),assets=records,method='streaming SHA256; Git blobs; declared author source; no binary execution')
    with (HERE/('asset_gate_'+label+'.json')).open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print('R11 gate PASS:',len(records),'comparisons')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('label',choices=['before','after']);main(p.parse_args().label)
