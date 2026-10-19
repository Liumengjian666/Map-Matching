#!/usr/bin/python3
"""Read-only proof checks against the one frozen run, without ROS execution."""
import csv,json
from pathlib import Path
from prepare_inputs import ROOT,sha
from analyze_run import HERE,read


def main():
    frozen=json.loads((HERE/'output_freeze.json').read_text())
    for name,value in frozen.items():
        p=ROOT/name
        assert p.stat().st_size==value['size'] and sha(p)==value['sha256'],name
    rows=read(HERE/'frame_results.csv');r=ROOT/'formal'
    assert len(rows)==2441 and [int(x['scan_index']) for x in rows]==list(range(2986,5427))
    lineage=read(r/'ekf_lineage.csv')
    events={name:[x for x in lineage if x['event_type']==name] for name in
            ('NDT_CALLBACK_ENTER','OOSM_ROLLBACK','NDT_CORRECTION_APPLIED','OOSM_REPLAY_COMPLETE','CORRECTED_PUBLISH')}
    assert all(len(v)==2441 for v in events.values())
    for i,x in enumerate(rows):
        assert x['terminal']=='APPLIED' and x['evidence_complete']=='1' and x['ndt_align_calls']=='1'
        assert int(x['prior_ndt_update_count'])==int(x['state_revision_before_update'])==i
        assert int(x['state_revision_after_replay'])==i+1
        assert float(x['deskew_max_imu_source_stamp'])<=float(x['deskew_reference_sec'])
        assert x['deskew_post_end_imu']=='0'
        order=[int(events[name][i]['event_index']) for name in events]
        assert order==sorted(order) and len(set(order))==5
        assert float(events['NDT_CORRECTION_APPLIED'][i]['state_stamp'])==float(events['NDT_CORRECTION_APPLIED'][i]['ndt_measurement_stamp'])
        assert float(events['OOSM_REPLAY_COMPLETE'][i]['state_stamp'])==float(events['NDT_CALLBACK_ENTER'][i]['state_stamp'])
    parity=read(HERE/'r7_first20_parity.csv')
    assert len(parity)==20
    for x in parity:
        assert x['source_hash_equal']==x['iterations_equal']=='1'
        assert all(float(v)==0 for k,v in x.items() if 'difference' in k)
    # CSV serialization may normalize line endings only; all cell strings match.
    for name in ('live_trajectories.csv','frame_terminals.csv','deskew.csv','fusion.csv','oosm.csv','geometry.csv'):
        with (r/name).open(newline='') as a,(HERE/name).open(newline='') as b:
            assert list(csv.reader(a))==list(csv.reader(b)),name
    print('PASS: frozen raw hashes; 2441 unique causal chains; first20 R7 parity; six cell-identical archives')


if __name__=='__main__':main()
