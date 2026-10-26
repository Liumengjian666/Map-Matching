#!/usr/bin/env python3
"""Continue only the unprocessed input after a proven zero-input startup monitor abort."""
import csv,json,subprocess,sys
from pathlib import Path
P=Path(__file__).resolve().parents[1];R=P.parents[1]
plan=json.loads((P/'evidence/RUN_PLAN.json').read_text());failed=Path(plan['runs'][1]['path'])
receipt=json.loads((failed/'RUN_RECEIPT.json').read_text());assert not receipt['input_completed']
for name in ['publisher_receipts.csv','localization_trajectory.csv','ekf_prediction_diagnostics.csv','oosm_statistics.csv']:
 with (failed/name).open() as f:assert len(list(csv.DictReader(f)))==0
assert not (failed/'r23_candidates.csv').exists()
proof={'failed_path':str(failed),'input_scans':0,'native_calls':0,'IMU_propagations':0,'cause':'supervisor required both node PID sets as soon as NDT was first seen; EKF had not yet appeared','change':'wait until both components have been seen before requiring both to remain alive','localization_source_or_parameters_changed':False,'completed_B_rerun':False}
(P/'evidence/ZERO_INPUT_STARTUP_ABORT.json').write_text(json.dumps(proof,indent=2)+'\n')
new_runs=[plan['runs'][0],{**plan['runs'][1],'path':str(failed.with_name('EASY_C0_C_strict_02'))}]+plan['runs'][2:]
(P/'evidence/FINAL_RUN_PLAN.json').write_text(json.dumps({'runs':new_runs,'original_plan_preserved':'RUN_PLAN.json','resumed_supervisor_code':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'no_reference_evaluation':True},indent=2)+'\n')
for run in new_runs[1:]:
 assert not Path(run['path']).exists()
 cmd=[sys.executable,str(P/'tools/run_sequence.py'),'--window',run['dataset'],'--arm',run['arm'],'--mode',run['mode'],'--run-dir',run['path']]
 with (P/'logs'/(Path(run['path']).name+'.supervisor.log')).open('x') as f:r=subprocess.run(cmd,cwd=R,stdout=f,stderr=subprocess.STDOUT)
 receipt=Path(run['path'])/'RUN_RECEIPT.json'
 assert r.returncode==0 and receipt.exists() and json.loads(receipt.read_text())['input_completed'],'Stop, no retry: '+run['path']
 print('INPUT_COMPLETE',run['path'],flush=True)
print('ALL_REQUESTED_ONLINE_INPUT_COMPLETE_NO_REFERENCE_EVALUATION',flush=True)
