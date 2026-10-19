#!/usr/bin/python3
"""Extend the unchanged R9 asset gate; write exclusively to the R10 directory."""
import argparse,hashlib,json,subprocess,sys
from pathlib import Path

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[1]
R9=REPO/'docs/p61_r9_geode_anomaly_readonly_attribution'
sys.path.insert(0,str(R9))
from asset_gate import sha,verify


def main(label):
    inherited=HERE/('r8_assets_'+label+'.json')
    base=verify(inherited)
    records=list(base['assets'])
    names=subprocess.check_output(['git','ls-tree','-r','--name-only','d1f9fa9464dac835598e72d52ea2484fd9aa801c','--',str(R9.relative_to(REPO))],cwd=REPO,text=True).splitlines()
    for name in names:
        expected=hashlib.sha256(subprocess.check_output(['git','show','d1f9fa9464dac835598e72d52ea2484fd9aa801c:'+name],cwd=REPO)).hexdigest()
        path=REPO/name;actual=sha(path)
        records.append(dict(asset='R9_frozen_archive',path=str(path),size_bytes=path.stat().st_size,
            expected_sha256=expected,sha256=actual,status='PASS' if actual==expected else 'FAIL',
            frozen_source='R9 Git d1f9fa9464dac835598e72d52ea2484fd9aa801c'))
        assert actual==expected,name
    result=dict(status='PASS',comparisons=len(records),assets=records,inherited_snapshot_sha256=sha(inherited),
        method='read-only streaming SHA256; R9 archive against immutable Git blobs; no nodes/binaries executed')
    with (HERE/('asset_gate_'+label+'.json')).open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print('R10 asset gate PASS:',len(records),'comparisons')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('label',choices=['before','after']);main(p.parse_args().label)
