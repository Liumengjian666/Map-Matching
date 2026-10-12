"""P18 pre-GT prefix parity, reusing the frozen P10 tolerances."""
import argparse
import csv
import json
import math
from pathlib import Path
import numpy as np
import rosbag
from scipy.spatial.transform import Rotation


def rows(path):
    with Path(path).open(newline='') as f: return list(csv.DictReader(f))


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--repo',type=Path,required=True)
    p.add_argument('--run',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--duration',type=float,default=18.5)
    args=p.parse_args()
    old=args.repo/'docs/p10_corridor01_deterministic_oosm_replay_r1/runs/control_v2'
    bag_path=args.repo/'.p10_corridor01_deterministic_oosm_replay_r1/input_v2/corridor01_p10_causal_startup5_eval35_fixedlag_v2.bag'
    with rosbag.Bag(str(bag_path)) as bag:
        cutoff=bag.get_start_time()+args.duration if args.duration else bag.get_end_time()
        covered_imu=[m.header.stamp.to_sec() for _,m,t in bag.read_messages(topics=['/imu/data'])
                     if t.to_sec()<=cutoff]
    if not covered_imu: raise ValueError('NO_COVERED_IMU_INPUT')
    last_input_imu=max(covered_imu)
    a,b=rows(old/'ndt_diagnostics.csv'),rows(args.run/'ndt_diagnostics.csv')
    failures=[]; evidence=[]; count=180
    def check(stage,index,field,x,y,tolerance=0):
        if tolerance:
            equal=math.isfinite(float(x)) and math.isfinite(float(y)) and abs(float(x)-float(y))<=tolerance
        else: equal=x==y
        evidence.append([stage,index+1,field,x,y,'PASS' if equal else 'FAIL'])
        if not equal: failures.append(dict(stage=stage,row=index+1,field=field,control=x,current=y))
    if len(b)<count: raise ValueError('INSUFFICIENT_PREFIX_NDT')
    identity=('frame_index','lidar_header_stamp','cloud_seq','cloud_size_raw',
              'cloud_size_after_filter','cloud_hash','initial_guess_source','initial_guess_reason',
              'local_imu_prior_used','scan_start_stamp','scan_mid_stamp','scan_end_stamp',
              'scan_min_offset_sec','scan_max_offset_sec','ndt_has_converged','ndt_iterations',
              'translation_limited','rotation_limited')
    for i,(left,right) in enumerate(zip(a[:count],b[:count])):
        for field in identity:
            check('NDT',i,field,left[field],right[field],1e-12 if 'stamp' in field or 'offset_sec' in field else 0)
        for prefix in ('initial_guess_','raw_ndt_','used_before_step_limit_','final_used_'):
            x=np.array([float(left[prefix+k]) for k in ('tx','ty','tz','qx','qy','qz','qw')])
            y=np.array([float(right[prefix+k]) for k in ('tx','ty','tz','qx','qy','qz','qw')])
            if not np.isfinite(x).all() or not np.isfinite(y).all():
                check('NDT',i,prefix+'finite',True,False); continue
            dt=float(np.linalg.norm(x[:3]-y[:3]))
            dr=float(np.degrees((Rotation.from_quat(x[3:]).inv()*Rotation.from_quat(y[3:])).magnitude()))
            check('NDT',i,prefix+'translation_difference_m',0,dt,1e-9)
            check('NDT',i,prefix+'rotation_difference_deg',0,dr,1e-8)
        check('NDT',i,'ndt_fitness',left['ndt_fitness'],right['ndt_fitness'],
              1e-12*max(1,abs(float(left['ndt_fitness']))))
    for name,excluded in (('oosm.csv',()),('deskew.csv',('deskew_processing_ms',))):
        left,right=rows(old/name)[:count],rows(args.run/name)[:count]
        check(name,0,'prefix_count',count,len(right))
        for i,(x,y) in enumerate(zip(left,right)):
            for key in x:
                if key not in excluded: check(name,i,key,x[key],y.get(key))
    left=[r for r in rows(old/'oosm_fixed_lag.csv') if r['status']=='APPLIED'][:count]
    right=[r for r in rows(args.run/'oosm_fixed_lag.csv') if r['status']=='APPLIED'][:count]
    check('fixed_lag',0,'prefix_applied',count,len(right))
    for i,(x,y) in enumerate(zip(left,right)):
        for key in ('observation_id','ndt_stamp','deadline_stamp','release_state_stamp',
                    'replay_imu_count','correction_applied','status'):
            check('fixed_lag',i,key,x[key],y[key])
    for event in ('IMU_HIGH_RATE_PUBLISH','CORRECTED_PUBLISH'):
        left=[r for r in rows(old/'ekf_lineage.csv') if r['event_type']==event
              and float(r['state_stamp'])<=last_input_imu]
        right=[r for r in rows(args.run/'ekf_lineage.csv') if r['event_type']==event]
        check(event,0,'expected_prefix_count',len(left),len(right))
        if not right: check(event,0,'nonempty',True,False)
        if left and right: check(event,0,'expected_last_state_stamp',left[-1]['state_stamp'],right[-1]['state_stamp'])
        for i,(x,y) in enumerate(zip(left,right)):
            for key in ('state_stamp','p_x','p_y','p_z','q_x','q_y','q_z','q_w',
                        'v_x','v_y','v_z','ndt_update_count','state_revision'):
                check(event,i,key+'_finite',True,math.isfinite(float(y[key])))
                check(event,i,key,x[key],y[key])
    late=sum('LATE' in r['status'] for r in rows(args.run/'oosm_fixed_lag.csv'))
    check('fixed_lag',0,'late_count',0,late)
    args.output.mkdir(parents=True,exist_ok=True)
    summary={}
    for stage,index,field,x,y,status in evidence:
        entry=summary.setdefault((stage,field),[0,0,None,0.0])
        entry[0]+=1
        if status=='FAIL':
            entry[1]+=1
            if entry[2] is None: entry[2]=index
        try:
            difference=abs(float(x)-float(y))
            if math.isfinite(difference): entry[3]=max(entry[3],difference)
        except (TypeError,ValueError): pass
    with (args.output/'control_parity.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['stage','field','checks','failures','first_failure_index','max_numeric_difference'])
        for key,value in summary.items(): w.writerow([*key,*value])
    result=dict(result='PASS' if not failures else 'FAIL',GT_LOADED=False,
                frozen_transaction_count=count,actual_ndt_rows=len(b),failure_count=len(failures),
                first_failures=failures[:20],late_count=late,input_cutoff=cutoff,last_input_imu=last_input_imu)
    (args.output/'control_parity.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return 0 if not failures else 1


if __name__=='__main__': raise SystemExit(main())
