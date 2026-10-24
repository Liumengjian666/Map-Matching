#!/usr/bin/env python3
"""Exactly 12 exclusive causal ROS runs; no reference or tuning logic."""
import subprocess,json,hashlib
from pathlib import Path
HERE=Path(__file__).resolve().parents[1];REPO=HERE.parents[1]
OUT=Path('/home/jian/livox_ws/p62_r24_geometry_oosm_data')
freeze=json.loads((HERE/'CODE_FREEZE.json').read_text())
for name,h in freeze['files_sha256'].items():
 assert hashlib.sha256((REPO/name).read_bytes()).hexdigest()==h,name
for mode in ('G','F','GF'):
 for window in ('W1','W3'):
  for arm in ('C0','C3'):
   dest=OUT/(mode.lower()+'_'+window.lower()+'_'+arm.lower()+'_01')
   print('START',mode,window,arm,flush=True)
   subprocess.run(['python3',str(HERE/'tools/run_ros.py'),'--mode',mode,'--window',window,'--arm',arm,'--run-dir',str(dest)],check=True,cwd=REPO)
   # Basic completion/budget checks, no reference values.
   import csv
   with (dest/'r23_admission.csv').open() as f: records=list(csv.DictReader(f))
   assert len(records)==60 and sum(int(x['extra_aligns']) for x in records)<=12
   assert not any(int(x['relative_only']) for x in records)
   print('DONE',mode,window,arm,'attempts',sum(int(x['nominal_attempted']) for x in records),'extra',sum(int(x['extra_aligns']) for x in records),'published',sum(int(x['measurement_published']) for x in records),flush=True)
