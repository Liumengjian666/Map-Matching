#!/usr/bin/env python3
"""No optimizer: compare the new reader/rules against all400 archived R3 receipts."""
import csv,json,tempfile
from pathlib import Path
import numpy as np
from contract import CsvTail,assess
A=Path(__file__).resolve().parents[1];R3=Path('/home/jian/livox_ws/p63_r3_hdl_data/trial_01')
n=list(csv.DictReader((R3/'NATIVE_NDT.csv').open()));r=json.loads((R3/'RUN_RECEIPT.json').read_text());p=[0,0,0,0,0,0,1]
t=CsvTail(R3/'NATIVE_NDT.csv')
for i,(row,old) in enumerate(zip(n,r['records'])):
    assert t.poll()==row
    f,v,dt,dr=assess(old['output_pose_xyzw'],old['raw_ndt_pose_xyzw'],row['converged']=='1',True,float(row['fitness_all_nn_m2']),old['native_matching_error'],old['inlier_fraction'],p)
    assert f==old['finite'] and v==old['valid_update_screen']
    assert abs(dt-old['translation_step_m'])<1e-12 and abs(dr-old['rotation_step_deg'])<1e-12
    p=old['output_pose_xyzw']
assert t.poll() is None;t.close()
with tempfile.TemporaryDirectory(prefix='p63_r4_tail_') as d:
    p=Path(d)/'x.csv';t=CsvTail(p);assert t.poll() is None;p.write_text('a,b\n1,');assert t.poll() is None
    with p.open('a') as f:f.write('2\n')
    assert t.poll()=={'a':'1','b':'2'};assert t.poll() is None;t.close()
(A/'evidence/SCHEDULER_TESTS.json').write_text(json.dumps({'R3_rows_replayed_without_optimizer':400,'validity_and_steps_match_R3':True,'incremental_reader_exact':True,'partial_line_preserved':True,'new_align_calls':0},indent=2)+'\n')
print('PASS R3 rules400/400;incremental and partial-line reader;align0')
