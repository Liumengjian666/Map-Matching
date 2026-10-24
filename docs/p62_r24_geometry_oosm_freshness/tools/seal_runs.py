#!/usr/bin/env python3
"""Seal all 12 outputs and identities, with no evaluation reference access."""
import csv,json,hashlib,shutil,subprocess
from pathlib import Path
P=Path(__file__).resolve().parents[1];R=P.parents[1];D=Path('/home/jian/livox_ws/p62_r24_geometry_oosm_data')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def rows(p):
 with p.open() as f:return list(csv.DictReader(f))
freeze=json.loads((P/'CODE_FREEZE.json').read_text())
for name,h in freeze['files_sha256'].items():assert sha(R/name)==h,name
out=P/'formal_runs';out.mkdir(exist_ok=False);runs=[]
nominal=extra=fused=0
for mode in ('G','F','GF'):
 for window in ('W1','W3'):
  for arm in ('C0','C3'):
   src=D/(mode.lower()+'_'+window.lower()+'_'+arm.lower()+'_01');dst=out/src.name;dst.mkdir()
   receipt=json.loads((src/'RUN_RECEIPT.json').read_text())
   assert receipt['process_exit_code']==0 and receipt['input_completed'] and not receipt['timed_out'] and not receipt['interrupted']
   a=rows(src/'r23_admission.csv');c=rows(src/'r23_candidates.csv')
   assert len(a)==60 and all(x['policy']=='MULTISTART_C' and x['relative_only']=='0' for x in a)
   n=sum(int(x['nominal_attempted']) for x in a);e=sum(int(x['extra_aligns']) for x in a);f=sum(int(x['measurement_published']) for x in a)
   assert n<=60 and e<=12 and len(c)==n+e
   files={}
   for file in src.iterdir():
    if file.is_file() and file.suffix in ('.csv','.json','.log'):
     shutil.copy2(file,dst/file.name);files[file.name]=sha(dst/file.name)
   runs.append({'window':window,'arm':arm,'strategy':mode,'path':str(dst),'runtime_path':str(src),'nominal_full':n,'extra_full':e,'fused':f,'files_sha256':files})
   nominal+=n;extra+=e;fused+=f
assert nominal<=720 and extra<=144 and len(runs)==12
seal={'all_formal_outputs_frozen_before_reference':True,'reference_values_read':False,'CODE_SHA':freeze['CODE_SHA'],'nominal_full':nominal,'extra_full':extra,'fused':fused,'formal_runs':runs}
with (P/'NATIVE_OUTPUT_FREEZE.json').open('x') as f:json.dump(seal,f,indent=2);f.write('\n')
print('SEALED',len(runs),'runs, nominal',nominal,'extra',extra,'fused',fused,'seal SHA',sha(P/'NATIVE_OUTPUT_FREEZE.json'))
