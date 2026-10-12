"""Read-only P18 application/completeness audit. No GT or parameter changes."""
import argparse
import collections
import csv
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def rows(path):
    with Path(path).open(newline='') as f:return list(csv.DictReader(f))


def pose(row,prefix):
    p=np.array([float(row[prefix+k]) for k in ('tx','ty','tz')])
    q=np.array([float(row[prefix+k]) for k in ('qx','qy','qz','qw')])
    if not np.isfinite(p).all() or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>1e-5:
        raise ValueError('NONFINITE_OR_INVALID_POSE')
    return p,Rotation.from_quat(q)


def sensor_sec(ns):
    return int(ns)//1000000000+(int(ns)%1000000000)*1e-9


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True)
    p.add_argument('--visual',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--full',action='store_true');args=p.parse_args()
    diag=rows(args.run/'ndt_diagnostics.csv');vision=rows(args.visual)
    lookup={sensor_sec(r['scan_ns']):r for r in vision}
    apply_reasons={'NO_VISUAL_ROW','INVALID_VISUAL_MEASUREMENT','PREVIOUS_SCAN_MISMATCH',
                   'NONFINITE_BASE_POSE','NONFINITE_VISUAL_GUESS','P4_VISUAL_TRANSLATION_SHORT_TIME_BRIDGE'}
    counts=collections.Counter();failures=[];applications=[];last=None;maxjump=0.;maxrot=0.
    for r in diag:
        v=lookup[float(r['lidar_header_stamp'])]
        attempted=r['initial_guess_reason'] in apply_reasons
        applied=r['initial_guess_source']=='p4_visual_translation_imu_rotation'
        counts['valid_input']+=v['valid']=='1';counts['apply_calls']+=attempted
        counts['applied']+=applied;counts['rejected']+=attempted and not applied
        if attempted and not applied: counts['reject_'+r['initial_guess_reason']]+=1
        translation_change=rotation_change=composition_error=0.
        for prefix in ('initial_guess_','raw_ndt_','final_used_'):pose(r,prefix)
        cur=pose(r,'final_used_')
        if last is not None:
            maxjump=max(maxjump,float(np.linalg.norm(cur[0]-last[0])))
            maxrot=max(maxrot,float(np.degrees((last[1].inv()*cur[1]).magnitude())))
        last=cur
        if applied:
            guess,rg=pose(r,'initial_guess_');imu,ri=pose(r,'imu_rotation_guess_')
            previous,rp=pose(r,'previous_pose_')
            d=np.array([float(v[k]) for k in ('dx','dy','dz')])
            composition_error=float(np.linalg.norm(guess-(previous+rp.apply(d))))
            translation_change=float(np.linalg.norm(guess-imu))
            rotation_change=float(np.degrees((ri.inv()*rg).magnitude()))
            previous_ns=int(v['prev_scan_ns']);current_ns=int(v['scan_ns'])
            causal=(v['valid']=='1' and int(v['ref_image_ns'])<=previous_ns
                    and int(v['cur_image_ns'])<=current_ns
                    and int(v['depth_max_point_ns'])<=int(v['ref_image_ns'])
                    and int(v['max_imu_ns'])<=current_ns)
            # appendPoseCsv normalizes a quaternion derived from the raw PCL
            # float matrix. Its reconstructed rotation is not that original
            # matrix. Keep this residual diagnostic-only; direct C++ tests
            # check apply() against the actual input matrix without transport.
            if not causal or rotation_change>1e-8:
                failures.append(dict(frame=r['frame_index'],reason='VISUAL_CAUSAL_OR_ROTATION'))
        applications.append(dict(frame=r['frame_index'],scan_ns=v['scan_ns'],
            relative_time_s=float(r['lidar_header_stamp'])-1517157224.188979,
            valid_input=v['valid'],apply_called=int(attempted),applied=int(applied),
            reason=r['initial_guess_reason'],translation_change_m=translation_change,
            rotation_change_deg=rotation_change,composition_error_m=composition_error))
    oosm=rows(args.run/'oosm.csv');fixed=rows(args.run/'oosm_fixed_lag.csv')
    lineage=rows(args.run/'ekf_lineage.csv')
    states=[r for r in lineage if r['event_type'] in ('IMU_HIGH_RATE_PUBLISH','CORRECTED_PUBLISH')]
    nonfinite=sum(not all(np.isfinite(float(r[k])) for k in ('p_x','p_y','p_z','q_x','q_y','q_z','q_w','v_x','v_y','v_z')) for r in states)
    converged=sum(r['ndt_has_converged']=='1' for r in diag)
    applied_oosm=sum(r['oosm_result']=='APPLIED' for r in oosm)
    late=sum('LATE' in r['status'] for r in fixed)
    if not counts['applied'] or nonfinite or late:failures.append(dict(reason='NO_APPLICATION_OR_NONFINITE_OR_LATE'))
    highrate_count=sum(r['event_type']=='IMU_HIGH_RATE_PUBLISH' for r in states)
    corrected_count=sum(r['event_type']=='CORRECTED_PUBLISH' for r in states)
    if args.full and (len(diag)!=396 or converged!=396 or applied_oosm!=396 or
                      highrate_count!=8004 or corrected_count!=396):
        failures.append(dict(reason='FULL_REPLAY_INCOMPLETE'))
    args.output.mkdir(parents=True,exist_ok=True)
    with (args.output/'visual_application_summary.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(applications[0]));w.writeheader();w.writerows(applications)
    result=dict(result='PASS' if not failures else 'FAIL',GT_LOADED=False,
                ndt_rows=len(diag),ndt_converged=converged,oosm_applied=applied_oosm,
                counters=dict(counts),late=late,nonfinite_states=nonfinite,
                highrate_states=highrate_count,corrected_states=corrected_count,
                max_used_scan_translation_jump_m=maxjump,max_used_scan_rotation_jump_deg=maxrot,
                csv_composition_residual_scope='diagnostic only: normalized quaternion loses raw float-matrix entries; direct C++ apply test covers formula',
                extra_align_calls=0,extra_align_basis='CONTROL mode, single existing ndt.align call in source',
                failures=failures)
    (args.output/'visual_application_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2));return 0 if not failures else 1


if __name__=='__main__':raise SystemExit(main())
