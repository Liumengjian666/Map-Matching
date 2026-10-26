#!/usr/bin/env python3
"""Compile existing ROS-free foundation tests and the one targeted grid regression."""
import concurrent.futures,json,os,subprocess,time
from pathlib import Path
P=Path(__file__).resolve().parents[1];R=P.parents[1];PKG=R/'src/dog_prior_map_localization';BUILD=R/'.r30_diagnostic_build'
common=['g++','-O3','-DNDEBUG','-std=c++14','-I/usr/include/eigen3','-I'+str(PKG/'include')]
plans={
 'ndt_target_resolution_contract_test':['tests/ndt_target_resolution_contract_test.cpp'],
 'frame_conversion_contract_test':['tests/frame_conversion_contract_test.cpp'],
 'causal_pose_prediction_test':['tests/causal_pose_prediction_test.cpp'],
 'imu_interval_numeric_test':['tests/imu_interval_numeric_test.cpp','src/core/imu_propagation.cpp'],
 'imu_deskew_contract_test':['tests/imu_deskew_contract_test.cpp','src/core/state_history.cpp','src/core/imu_propagation.cpp','src/core/imu_deskew.cpp'],
 'oosm_replay_planner_contract_test':['tests/oosm_replay_planner_contract_test.cpp','src/core/oosm_replay_planner.cpp','src/core/state_history.cpp']}
def run(item):
 name,files=item;cmd=common+[str(PKG/f) for f in files]+['-o',str(BUILD/name)]
 if name.startswith('ndt_target'):cmd+=subprocess.check_output(['pkg-config','--cflags','--libs','pcl_registration-1.10'],text=True).split()
 start=time.monotonic()
 with (P/'logs'/(name+'.build.log')).open('w') as f:c=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
 result={'test':name,'compile_argv':cmd,'build_exit':c.returncode,'build_s':time.monotonic()-start,'native_align_calls':0}
 if c.returncode==0:
  r=subprocess.run([str(BUILD/name)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True);(P/'logs'/(name+'.run.log')).write_text(r.stdout);result['run_exit']=r.returncode;result['output']=r.stdout.strip()
 return result
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:results=list(pool.map(run,plans.items()))
(P/'evidence/CORE_REGRESSION_RESULTS.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps(results,indent=2));assert all(x['build_exit']==0 and x.get('run_exit')==0 for x in results)
