#!/usr/bin/env python3
import json,csv,hashlib,shutil
from pathlib import Path
P=Path(__file__).resolve().parents[1];R=P.parents[1]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
C=json.loads((P/'CODE_FREEZE.json').read_text())
for n,h in C['files_sha256'].items():assert sha(R/n)==h,n
src=Path('/home/jian/livox_ws/p62_r25_results_paper_data/g_only_w1_c3_01');dst=P/'formal_runs/g_only_w1_c3_01';dst.mkdir(parents=True,exist_ok=False)
receipt=json.loads((src/'RUN_RECEIPT.json').read_text());assert receipt['process_exit_code']==0 and receipt['input_completed'] and not receipt['timed_out']
with (src/'r23_admission.csv').open() as f:a=list(csv.DictReader(f))
with (src/'r23_candidates.csv').open() as f:c=list(csv.DictReader(f))
assert len(a)==60 and all(int(x['extra_aligns'])==0 and x['policy']=='MULTISTART_C' and int(x['relative_only'])==0 for x in a)
n=sum(int(x['nominal_attempted']) for x in a);accepted=sum(int(x['measurement_published']) for x in a);assert len(c)==n<=60
h={}
for f in src.iterdir():
 if f.is_file() and f.suffix in ('.csv','.json','.log'):
  shutil.copy2(f,dst/f.name);h[f.name]=sha(dst/f.name)
seal={'all_formal_outputs_frozen_before_reference':True,'reference_values_read':False,'CODE_SHA':C['CODE_SHA'],'nominal_full':n,'extra_full':0,'formal_runs':[{'window':'W1','arm':'C3','strategy':'G_ONLY','path':str(dst),'nominal_full':n,'extra_full':0,'fused':accepted,'files_sha256':h}]}
with (P/'NATIVE_OUTPUT_FREEZE.json').open('x') as f:json.dump(seal,f,indent=2);f.write('\n')
print('SEALED: nominal',n,'extra0; fused',accepted,'SHA',sha(P/'NATIVE_OUTPUT_FREEZE.json'))
