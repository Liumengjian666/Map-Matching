#!/usr/bin/python3
"""Summarize real frozen ROS outputs; no GT or synthetic localization."""
import csv
import hashlib
import json
from decimal import Decimal
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path('/home/jian/livox_ws/p61_r7_geode_runs')
HERE=Path(__file__).resolve().parent


def read(p):
    with open(p) as f: return list(csv.DictReader(f))


def sha(p):
    h=hashlib.sha256()
    with open(p,'rb') as f:
        for b in iter(lambda:f.read(1<<20),b''): h.update(b)
    return h.hexdigest()


def stats(x):
    a=np.asarray(x,dtype=float)
    return dict(mean=float(a.mean()),p95=float(np.percentile(a,95)),max=float(a.max()),count=len(a))


def pose(row,prefix=''):
    if prefix:
        return np.array([float(row[prefix+'_'+s]) for s in ('tx','ty','tz')]),Rotation.from_quat([float(row[prefix+'_'+s]) for s in ('qx','qy','qz','qw')])
    return np.array([float(row[s]) for s in ('px','py','pz')]),Rotation.from_quat([float(row[s]) for s in ('qx','qy','qz','qw')])


def main():
    r=ROOT/'formal'
    # Freeze raw outputs before any derived interpretation. Never read GT.
    frozen={str(p):sha(p) for p in sorted(r.glob('*.csv'))}
    freeze_path=ROOT/'output_freeze.json'
    if freeze_path.exists(): assert json.loads(freeze_path.read_text())==frozen
    else: freeze_path.write_text(json.dumps(frozen,indent=2)+'\n')
    m=json.loads((ROOT/'input/manifest.json').read_text())
    ndt=read(r/'ndt_diagnostics.csv'); deskew=read(r/'deskew.csv'); fusion=read(r/'fusion.csv')
    oosm=read(r/'oosm.csv'); receipts=read(r/'publisher_receipts.csv'); performance=read(r/'ndt_performance.csv')
    live=read(r/'live_trajectories.csv'); memory=read(r/'memory.csv'); lineage=read(r/'ekf_lineage.csv')
    corrected=[x for x in live if x['stream']=='CORRECTED']
    predictions=[x for x in live if x['stream']=='PREDICTION']
    assert all(len(x)==20 for x in (ndt,deskew,fusion,oosm,receipts,performance,corrected,predictions))
    assert all(x['status']=='PUBLISHED' and x['post_scan_end_imu_used']=='0' for x in deskew)
    assert all(x['oosm_result']=='APPLIED' for x in oosm)
    assert all(x['initial_guess_source']=='same_scan_end_ekf_deskew_prediction' for x in ndt)
    assert len({x['cloud_hash'] for x in ndt})==20
    assert all(int(x['coupled_jet_calls'])==int(x['coupled_value_calls'])==0 for x in ndt)
    assert all(int(x['coupled_feedback_applied'])==0 for x in ndt)
    corrections=[x for x in lineage if x['event_type']=='NDT_CORRECTION_APPLIED']
    assert len(corrections)==20
    reference_errors=[]; table=[]
    for i,(n,d,f,p,c,a) in enumerate(zip(ndt,deskew,fusion,predictions,corrected,corrections)):
        source=m['scans'][i]; end=int(source['end_ns'])
        assert int(f['measurement_ns'])==int(p['stamp_ns'])==end
        assert abs(float(a['state_stamp'])-end/1e9)<5e-7
        assert abs(float(a['state_stamp'])-float(a['ndt_measurement_stamp']))<1e-9
        pp,pr=pose(p); np_,nr=pose(n,'raw_ndt'); cp,cr=pose(c)
        ip,ir=pose(n,'initial_guess')
        assert np.linalg.norm(ip-pp)<1e-9 and (ir.inv()*pr).magnitude()<1e-9
        reference_errors.append(float(abs(Decimal.from_float(float(a['state_stamp']))*Decimal(10**9)-Decimal(end))))
        table.append(dict(scan_index=source['scan_index'],start_ns=source['stamp_ns'],end_ns=end,
            source_points=n['cloud_size_after_filter'],source_hash=n['cloud_hash'],deskew=d['status'],
            ndt_converged=n['ndt_has_converged'],iterations=n['ndt_iterations'],fitness=n['ndt_fitness'],
            correction_m=float(np.linalg.norm(np_-pp)),correction_deg=float(np.rad2deg((pr.inv()*nr).magnitude())),
            fusion=oosm[i]['oosm_result'],actual_correction_sec=a['state_stamp'],raw_rollback_sec=oosm[i]['rollback_stamp'],
            rollback_fragment_ms=oosm[i]['alignment_error_ms'],corrected_publish_ns=c['stamp_ns'],
            prediction_xyz=pp.tolist(),raw_ndt_xyz=np_.tolist(),ekf_xyz=cp.tolist(),
            translation_limited=n['translation_limited'],rotation_limited=n['rotation_limited']))
    with open(HERE/'frame_results.csv','w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(table[0]),lineterminator='\n');w.writeheader();w.writerows(table)
    cp=np.array([pose(x)[0] for x in corrected]); rotations=Rotation.from_quat([[float(x[k]) for k in ('qx','qy','qz','qw')] for x in corrected])
    jumps=np.linalg.norm(np.diff(cp,axis=0),axis=1)
    turns=np.rad2deg((rotations[:-1].inv()*rotations[1:]).magnitude())
    resources={}
    for name in sorted({x['process'] for x in memory}):
        samples=[x for x in memory if x['process']==name]
        resources[name]={'peak_rss_mib':max(int(x['rss_kib']) for x in samples)/1024,
            'vmhwm_mib':max(int(x['hwm_kib']) for x in samples)/1024,
            'sample_count':len(samples),'cpu_ticks_start':int(samples[0]['utime_ticks'])+int(samples[0]['stime_ticks']),
            'cpu_ticks_end':int(samples[-1]['utime_ticks'])+int(samples[-1]['stime_ticks'])}
    durations={'deskew_ms':stats([float(x['deskew_processing_ms']) for x in deskew]),
        'ndt_align_ms':stats([float(x['align_time_ms']) for x in performance]),
        'ndt_callback_ms':stats([float(x['localization_time_ms']) for x in performance]),
        'ekf_update_ms':stats([float(x['update_ms']) for x in fusion]),
        'publisher_to_fusion_ack_ms':stats([float(x['publish_to_ack_ms']) for x in receipts])}
    durations['steady_ndt_align_ms_excluding_first']=stats([float(x['align_time_ms']) for x in performance[1:]])
    durations['sum_deskew_ndt_callback_fusion_ms']=stats([float(d['deskew_processing_ms'])+
        float(n['localization_time_ms'])+float(f['update_ms']) for d,n,f in zip(deskew,performance,fusion)])
    result={'TASK':'P61-R7-GEODE-CONTINUOUS-BASELINE-SMOKE',
        'FINAL_RESULT':'BASELINE_20_FRAME_READY' if all(x['ndt_has_converged']=='1' for x in ndt) else 'BASELINE_RUNNING_BUT_NDT_FAILED',
        'scans':20,'deskew_success':20,'ndt_align_calls':20,'ndt_converged':sum(x['ndt_has_converged']=='1' for x in ndt),
        'oosm_applied':20,'oosm_rejected':0,'non_gt':True,'gt_read':False,'extra_align_calls':0,
        'coupled_jet_calls':0,'coupled_value_calls':0,'candidate_feedback':0,
        'map_sha256':sha(Path('/home/jian/livox_ws/p61_r5_geode_runs/formal_mapping/frozen_map.pcd')),
        'initialization_sha256':sha(HERE.parent/'p61_r6_geode_causal_init_bridge/initialization.json'),
        'covariance_policy':'existing fixed 15D restart prior; R6 23D covariance NOT imported',
        'map_points':321053,'target_points':sorted({int(x['map_points']) for x in performance}),
        'sensor_window_ns':[int(m['scans'][0]['stamp_ns']),int(m['scans'][-1]['end_ns'])],
        'initial_T_M_L':json.loads((HERE.parent/'p61_r6_geode_causal_init_bridge/initialization.json').read_text())['initial_state']['T_M_L'],
        'final_corrected_lidar_pose':{k:corrected[-1][k] for k in ('stamp_ns','px','py','pz','qx','qy','qz','qw')},
        'corrected_path_length_19_intervals_m':float(jumps.sum()),'max_adjacent_translation_m':float(jumps.max()),
        'max_adjacent_rotation_deg':float(turns.max()),'max_raw_ndt_correction_m':max(x['correction_m'] for x in table),
        'max_raw_ndt_correction_deg':max(x['correction_deg'] for x in table),
        'maximum_native_double_time_representation_error_ns':max(reference_errors),
        'durations':durations,'resources':resources,'PUSH_EXECUTED':False,'accuracy_evaluated':False,
        'hessian_diagnosis':'not enabled in existing fusion runtime; no new jets added',
        'replay':json.loads((r/'replay_summary.json').read_text()),'raw_output_sha256':frozen}
    identity=json.loads((ROOT/'run_identity.json').read_text())
    result.update({key:identity[key] for key in ('START_SHA','CODE_SHA')})
    result['binaries']=identity['binaries']
    result['visualization_post_map_publish_resources']=json.loads((ROOT/'visualization_resources.json').read_text())
    result['realtime_validation']='NOT_REALTIME_VALIDATED'
    result['runtime_recorder_note']='Frozen receipts label 1..20 transport seq as scan_index; original indices 2986..3005 are bound by input SHA and frame_results. Archival reproducer fixes the log label only; no rerun.'
    (HERE/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(8,5))
    for name in ('PREDICTION','NDT_USED','CORRECTED'):
        data=np.array([pose(x)[0] for x in live if x['stream']==name])
        ax.plot(data[:,0],data[:,1],'.-',label=name)
    ax.set(xlabel='map X / m',ylabel='map Y / m',title='Real 20-frame GEODE smoke — no GT accuracy claim')
    ax.axis('equal');ax.legend();fig.tight_layout();fig.savefig(HERE/'live_trajectories.png',dpi=160);plt.close(fig)
    print(json.dumps({k:v for k,v in result.items() if k!='raw_output_sha256'},indent=2))


if __name__=='__main__':main()
