#!/usr/bin/env python3
"""Run only the predeclared R31 causal trajectories, with exclusive outputs."""
import json,subprocess,sys
from pathlib import Path
P=Path(__file__).resolve().parents[1];R=P.parents[1];data=Path('/home/jian/livox_ws/p62_r31_observation_identity_data')
runs=[{'dataset':'EASY','arm':'C0','mode':m,'path':str(data/('EASY_C0_'+m+'_strict_01'))} for m in 'BC']
config=json.loads((P/'input/DATASETS.json').read_text())
if 'NC_W1' in config['datasets']:runs.append({'dataset':'NC_W1','arm':'C3','mode':'C','path':str(data/'NC_W1_C3_C_strict_01')})
plan=P/'evidence/RUN_PLAN.json';assert not plan.exists();assert all(not Path(x['path']).exists() for x in runs)
plan.write_text(json.dumps({'code_sha':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'runs':runs,'reference_evaluation_values_read':False},indent=2)+'\n')
for run in runs:
 cmd=[sys.executable,str(P/'tools/run_sequence.py'),'--window',run['dataset'],'--arm',run['arm'],'--mode',run['mode'],'--run-dir',run['path']]
 with (P/'logs'/(Path(run['path']).name+'.supervisor.log')).open('x') as f:r=subprocess.run(cmd,cwd=R,stdout=f,stderr=subprocess.STDOUT)
 receipt=Path(run['path'])/'RUN_RECEIPT.json'
 assert r.returncode==0 and receipt.exists() and json.loads(receipt.read_text())['input_completed'],'Stop; no retry: '+run['path']
 print('INPUT_COMPLETE',run['path'],flush=True)
print('ALL_ONLINE_COMPLETE_REFERENCE_NOT_EVALUATED',flush=True)
