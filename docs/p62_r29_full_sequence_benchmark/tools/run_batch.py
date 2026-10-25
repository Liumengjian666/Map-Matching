#!/usr/bin/env python3
"""One frozen matched batch; no GT, no automatic failed-run retries."""
import argparse,json,subprocess,sys,time
from pathlib import Path
P=Path(__file__).resolve().parents[1];ROOT=P.parents[1]
a=argparse.ArgumentParser();a.add_argument('--dataset',required=True);a.add_argument('--arm',required=True,choices=['C0','C3']);a.add_argument('--modes',default='ABCD');a.add_argument('--tag',default='full_02');args=a.parse_args()
plan=[]
for mode in args.modes:
 d=Path('/home/jian/livox_ws/p62_r29_full_sequence_data')/(args.dataset+'_'+args.arm+'_'+mode+'_'+args.tag)
 assert not d.exists()
 plan.append({'dataset':args.dataset,'arm':args.arm,'mode':mode,'path':str(d)})
planfile=P/'logs'/('BATCH_PLAN_'+args.dataset+'_'+args.arm+'_'+args.tag+'.json');assert not planfile.exists()
planfile.write_text(json.dumps({'code_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'runs':plan,'test_reference_values_read':False},indent=2)+'\n')
for run in plan:
 command=[sys.executable,str(P/'tools/run_sequence.py'),'--window',run['dataset'],'--arm',run['arm'],'--mode',run['mode'],'--run-dir',run['path']]
 log=P/'logs'/('full_'+run['dataset']+'_'+run['arm']+'_'+run['mode']+'_'+args.tag+'.log')
 with log.open('x') as f:p=subprocess.run(command,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
 receipt=Path(run['path'])/'RUN_RECEIPT.json'
 assert receipt.exists() and json.loads(receipt.read_text())['input_completed'], 'Incomplete run; stop batch without retry: '+run['path']
 print('FULL_INPUT_COMPLETE',run['dataset'],run['arm'],run['mode'],flush=True)
print('BATCH_ONLINE_COMPLETE_REFERENCE_NOT_READ',flush=True)
