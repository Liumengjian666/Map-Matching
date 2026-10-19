#!/usr/bin/python3
"""Check frozen input identity/admission, not trajectory performance."""
import csv,json
from asset_gate import HERE,REPO,R8,RUN,sha


def main():
    m=json.loads((R8/'input_manifest.json').read_text())
    with (REPO/'docs/p61_r2_geode_real_data_run/scan_ledger.csv').open() as f:
        ledger={int(r['scan_index']):r for r in csv.DictReader(f)}
    with (R8/'scan_manifest.csv').open() as f:scans=list(csv.DictReader(f))
    assert [int(r['scan_index']) for r in scans]==list(range(2986,5427))
    keys=('stamp_ns','points','frame_id','offset_min','offset_max','serialized_sha256','timebase')
    for r in scans:
        original=ledger[int(r['scan_index'])]
        assert all(r[k]==original[k] for k in keys)
        first=int(r['stamp_ns'])+int(r['offset_min']);end=int(r['stamp_ns'])+int(r['offset_max'])
        assert end==int(r['end_ns']) and m['test_start_ns']<=first<=end<=m['test_end_ns']
    for b in m['boundary']:
        r=ledger[int(b['scan_index'])];assert all(r[k]==b[k] for k in keys)
        admitted=m['test_start_ns']<=int(r['stamp_ns'])+int(r['offset_min']) and int(r['stamp_ns'])+int(r['offset_max'])<=m['test_end_ns']
        assert admitted==b['admitted']
    with (RUN/'input/imu.csv').open() as f:imu=list(csv.DictReader(f))
    ns=[int(x['stamp_ns']) for x in imu]
    assert len(ns)==m['imu_count'] and all(a<b for a,b in zip(ns,ns[1:]))
    assert ns[0]==m['imu_first_ns'] and ns[-1]==m['imu_last_ns']
    assert ns[0]<int(scans[0]['stamp_ns']) and ns[-1]>int(scans[-1]['end_ns'])
    out=dict(status='PASS',scan_count=len(scans),first_index=2986,last_index=5426,
        first_header_ns=int(scans[0]['stamp_ns']),last_end_ns=int(scans[-1]['end_ns']),
        imu_count=len(ns),boundary_admission={b['scan_index']:b['admitted'] for b in m['boundary']},
        source_manifest_sha256=sha(R8/'input_manifest.json'),
        scope='all input identities only; no out-of-scope trajectory analysis',gt_read=False)
    with (HERE/'input_semantic_check.json').open('x') as f:json.dump(out,f,indent=2);f.write('\n')
    print(json.dumps(out,indent=2))


if __name__=='__main__':main()
