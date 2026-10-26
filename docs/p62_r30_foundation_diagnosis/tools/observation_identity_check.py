#!/usr/bin/env python3
"""Reproduce the archived post-admission limiter at raw62, without NDT or EKF."""
import csv,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
sys.dont_write_bytecode=True
P=Path(__file__).resolve().parents[1];R=P.parents[1];OLD=R/'docs/p62_r29_full_sequence_benchmark'
sys.path.insert(0,str(OLD/'tools'));from evaluate_batch import mat,Reference,error,flag
cfg=json.loads((OLD/'input/DATASETS.json').read_text())['datasets']['EASY'];ref=Reference(cfg)
results=[]
for mode in 'BC':
 run=Path('/home/jian/livox_ws/p62_r29_full_sequence_data')/f'EASY_C0_{mode}_full_03'
 with (run/'r23_ndt_lineage.csv').open() as f:lines=list(csv.DictReader(f))
 i=62-5;current=lines[i];prev_index=max(j for j in range(i) if flag(lines[j]['measurement_published']))
 previous=mat(lines[prev_index],'published');raw=mat(current,'raw');pub=mat(current,'published');reproduced=raw.copy()
 dp=raw[:3,3]-previous[:3,3]
 if np.linalg.norm(dp)>.5:reproduced[:3,3]=previous[:3,3]+dp/np.linalg.norm(dp)*.5
 angle=Rotation.from_matrix(previous[:3,:3].T@raw[:3,:3]).magnitude()*180/np.pi
 if angle>5:reproduced[:3,:3]=Slerp([0,1],Rotation.from_matrix([previous[:3,:3],raw[:3,:3]]))([5/angle]).as_matrix()[0]
 delta=float(np.linalg.norm(reproduced[:3,3]-pub[:3,3]));assert delta<1e-12
 rotation_difference=float(np.linalg.norm(reproduced[:3,:3]-pub[:3,:3]))
 scan=int(current['scan_ns']);before=error(raw,ref.at(scan));after=error(pub,ref.at(scan))
 with (run/'r23_candidates.csv').open() as f:c=next(x for x in csv.DictReader(f) if int(x['scan_ns'])==scan and x['id']=='0')
 results.append({'mode':mode,'raw':62,'previous_published_raw':prev_index+5,'previous_observation_age_s':(scan-int(lines[prev_index]['scan_ns']))/1e9,'candidate_basic':flag(c['basic_quality']),'candidate_absolute_pass':flag(c['absolute_pass']),'candidate_selected':flag(c['selected']),'raw_to_published_translation_m':float(np.linalg.norm(raw[:3,3]-pub[:3,3])),'translation_reproduction_error_m':delta,'scipy_vs_eigen_rotation_matrix_difference':rotation_difference,'verified_scope':'exact translation limiter; rotation normalization uses different libraries and is not asserted identical','raw_reference_translation_m':before[0],'raw_reference_rotation_deg':before[1],'published_reference_translation_m':after[0],'published_reference_rotation_deg':after[1],'original_ekf_update_applied':True,'scope':'Archived limiter transformation is causal for this pose change; not proof of all later trajectory failure, no repaired run'})
(P/'evidence/OBSERVATION_IDENTITY_COUNTEREXAMPLE.json').write_text(json.dumps(results,indent=2)+'\n');print(json.dumps(results,indent=2))
