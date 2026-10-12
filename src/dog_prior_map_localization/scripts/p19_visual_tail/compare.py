"""Full P18/P19 physical-output comparison; no GT, no runtime edits."""
import argparse
import csv
import json
import math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import yaml


def rows(path):
    with Path(path).open(newline='') as f:
        return list(csv.DictReader(f))


def write_csv(path, records):
    if not records:
        raise ValueError('EMPTY_EVIDENCE:' + str(path))
    with Path(path).open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(records[0]), lineterminator='\n')
        w.writeheader(); w.writerows(records)


def pose_delta(a, b):
    a, b = np.array(a, dtype=float), np.array(b, dtype=float)
    if not np.isfinite(a).all() or not np.isfinite(b).all():
        return float('inf'), float('inf')
    if np.linalg.norm(a[3:]) < 1e-9 or np.linalg.norm(b[3:]) < 1e-9:
        return float('inf'), float('inf')
    dr = np.degrees((Rotation.from_quat(a[3:]).inv() * Rotation.from_quat(b[3:])).magnitude())
    return float(np.linalg.norm(a[:3]-b[:3])), float(dr)


class Checks:
    def __init__(self):
        self.summary = {}; self.failures = []

    def check(self, stage, index, stamp, field, left, right, tolerance=0):
        difference = None
        try:
            x, y = float(left), float(right)
            difference = abs(x-y)
            # IDs/counters must never pass through float (19-digit nanosecond IDs).
            equal = left == right if not tolerance else (math.isfinite(x) and math.isfinite(y) and difference <= tolerance)
        except (TypeError, ValueError):
            equal = left == right
        key = (stage, field)
        out = self.summary.setdefault(key, dict(stage=stage, field=field, checks=0,
            failures=0, first_failure_index=None, first_failure_stamp=None, max_difference=0.0))
        out['checks'] += 1
        if difference is not None and math.isfinite(difference):
            out['max_difference'] = max(out['max_difference'], difference)
        if not equal:
            out['failures'] += 1
            if out['first_failure_index'] is None:
                out.update(first_failure_index=index, first_failure_stamp=stamp)
            self.failures.append(dict(stage=stage, index=index, stamp=stamp, field=field,
                previous=left, repeat=right, difference=difference))

    def table(self, stage, left, right, expected, excluded=()):
        self.check(stage, 0, None, 'P18_count', expected, len(left))
        self.check(stage, 0, None, 'P19_count', expected, len(right))
        for i, (a, b) in enumerate(zip(left, right), 1):
            stamp = a.get('state_stamp', a.get('lidar_header_stamp', a.get('ndt_stamp', a.get('reference_stamp'))))
            self.check(stage, i, stamp, 'schema', sorted(a), sorted(b))
            for key in a:
                if key in excluded:
                    continue
                tolerance = 1e-12*max(1, abs(float(a[key]))) if key == 'ndt_fitness' else 0
                if key.endswith('stamp') or key in ('local_imu_t0', 'local_imu_t1'):
                    tolerance = 0 if a[key]=='nan' else 1e-12
                self.check(stage, i, stamp, key, a[key], b.get(key), tolerance)


def normalized_parameters(path):
    params = yaml.safe_load(Path(path).read_text())
    params.pop('run_id', None); params.pop('roslaunch', None)
    for key in ('ekf_prediction_diagnostics_csv_path', 'imu_deskew_csv_path',
                'ndt_diagnostics_csv_path', 'oosm_csv_path', 'oosm_fixed_lag_csv_path', 'runtime_csv_path'):
        params['output'][key] = '<independent output>/' + Path(params['output'][key]).name
    return params


def schedule_valid(row):
    values=[float(row[k]) for k in ('ndt_stamp','deadline_stamp','arrival_state_stamp')]
    if not all(math.isfinite(v) for v in values):return False
    valid=values[2]<values[1] and abs(values[1]-values[0]-.15)<3e-7
    if row['status']=='APPLIED':
        release=float(row['release_state_stamp'])
        valid=valid and math.isfinite(release) and release>=values[1]
    return valid


def compare(previous, repeat, output):
    c = Checks()
    ndt_a, ndt_b = rows(previous/'ndt_diagnostics.csv'), rows(repeat/'ndt_diagnostics.csv')
    prefixes = ('baseline_guess_', 'imu_rotation_guess_', 'initial_guess_', 'previous_pose_',
                'delta_pose_', 'raw_ndt_', 'used_before_step_limit_', 'final_used_')
    pose_keys = {p+k for p in prefixes for k in ('tx','ty','tz','qx','qy','qz','qw')}
    timing = {'ros_now','wall_time','ndt_align_complete_wall_sec','ndt_odom_publish_wall_sec',
              'local_imu_history_first_stamp','local_imu_history_last_stamp','pending_lidar_count'}
    c.table('NDT', ndt_a, ndt_b, 396, timing | pose_keys)
    for label,table in [('P18',ndt_a),('P19',ndt_b)]:
        for i,row in enumerate(table,1):
            for key in ('lidar_header_stamp','ndt_fitness'):
                c.check(label+'_NDT',i,row['lidar_header_stamp'],key+'_finite',True,math.isfinite(float(row[key])))
            if i>1:
                c.check(label+'_NDT',i,row['lidar_header_stamp'],'strictly_increasing_stamp',True,
                        float(row['lidar_header_stamp'])>float(table[i-2]['lidar_header_stamp']))
    for i, (a, b) in enumerate(zip(ndt_a, ndt_b), 1):
        for p in prefixes:
            x, y = [[float(row[p+k]) for k in ('tx','ty','tz','qx','qy','qz','qw')] for row in (a,b)]
            if np.isnan(x).all() and np.isnan(y).all():
                # Unused first-frame stages only, not valid output poses.
                if i == 1 and p in ('imu_rotation_guess_', 'previous_pose_', 'delta_pose_'):
                    continue
            dt, dr = pose_delta(x, y)
            c.check('NDT', i, a['lidar_header_stamp'], p+'position_m', 0, dt, 1e-9)
            c.check('NDT', i, a['lidar_header_stamp'], p+'orientation_deg', 0, dr, 1e-8)
    c.table('Deskew', rows(previous/'deskew.csv'), rows(repeat/'deskew.csv'), 396, {'deskew_processing_ms'})
    c.table('OOSM', rows(previous/'oosm.csv'), rows(repeat/'oosm.csv'), 396)
    fa, fb = rows(previous/'oosm_fixed_lag.csv'), rows(repeat/'oosm_fixed_lag.csv')
    c.table('FixedLag', [r for r in fa if r['status']=='APPLIED'],
            [r for r in fb if r['status']=='APPLIED'], 396,
            {'arrival_state_stamp','arrival_wall_sec','processing_wall_sec'})
    for label, table in [('P18',fa),('P19',fb)]:
        c.check('FixedLag',0,None,label+'_queued',396,sum(r['status']=='QUEUED' for r in table))
        c.check('FixedLag',0,None,label+'_total',792,len(table))
        c.check('FixedLag',0,None,label+'_late_or_other',0,sum(r['status'] not in ('QUEUED','APPLIED') for r in table))
        queued=[r for r in table if r['status']=='QUEUED'];applied=[r for r in table if r['status']=='APPLIED']
        c.check('FixedLag',0,None,label+'_unique_queued_ID',396,len({r['observation_id'] for r in queued}))
        c.check('FixedLag',0,None,label+'_unique_applied_ID',396,len({r['observation_id'] for r in applied}))
        for i,(q,a) in enumerate(zip(queued,applied),1):
            for key in ('observation_id','ndt_stamp','deadline_stamp','arrival_state_stamp','arrival_wall_sec'):
                c.check('FixedLag',i,a['ndt_stamp'],label+'_queued_applied_'+key,q[key],a[key])
        for i,row in enumerate(table,1):
            c.check('FixedLag',i,row['ndt_stamp'],label+'_deadline_contract',True,schedule_valid(row))
    ea, eb = rows(previous/'ekf_lineage.csv'), rows(repeat/'ekf_lineage.csv')
    for event, expected in [('IMU_HIGH_RATE_PUBLISH',8004),('CORRECTED_PUBLISH',396),
                            ('OOSM_ROLLBACK',396),('NDT_CORRECTION_APPLIED',396),('OOSM_REPLAY_COMPLETE',396)]:
        a, b = [[r for r in table if r['event_type']==event] for table in (ea,eb)]
        c.table(event, a, b, expected, {'event_index','event_ros_stamp','p_x','p_y','p_z','q_x','q_y','q_z','q_w'})
        for label,table in [('P18',a),('P19',b)]:
            for i,row in enumerate(table,1):
                for key in ('state_stamp','v_x','v_y','v_z'):
                    c.check(event,i,row['state_stamp'],label+'_'+key+'_finite',True,math.isfinite(float(row[key])))
                if event in ('IMU_HIGH_RATE_PUBLISH','CORRECTED_PUBLISH') and i>1:
                    c.check(event,i,row['state_stamp'],label+'_strictly_increasing_stamp',True,
                            float(row['state_stamp'])>float(table[i-2]['state_stamp']))
        for i,(x,y) in enumerate(zip(a,b),1):
            dt,dr=pose_delta([x[k] for k in ('p_x','p_y','p_z','q_x','q_y','q_z','q_w')],
                            [y[k] for k in ('p_x','p_y','p_z','q_x','q_y','q_z','q_w')])
            c.check(event,i,x['state_stamp'],'position_m',0,dt,1e-9)
            c.check(event,i,x['state_stamp'],'orientation_deg',0,dr,1e-8)
    c.check('Configuration',0,None,'algorithm_parameters',normalized_parameters(previous/'rosparams.yaml'),
            normalized_parameters(repeat/'rosparams.yaml'))
    for key in ('input_sha256','visual_sha256','dataset_config_sha256','binaries','completed','enable'):
        a,b=[json.loads((p/'execution.json').read_text())[key] for p in (previous,repeat)]
        c.check('Execution',0,None,key,a,b)
    for label,table in [('P18',ndt_a),('P19',ndt_b)]:
        c.check('Visual',0,None,label+'_applications',169,sum(r['initial_guess_source']=='p4_visual_translation_imu_rotation' for r in table))
    output.mkdir(parents=True,exist_ok=False)
    write_csv(output/'replication_comparison.csv',list(c.summary.values()))
    chronological=sorted(c.failures,key=lambda r:float(r['stamp']) if r['stamp'] is not None else -math.inf)
    result=dict(REPLICATION_RESULT='REPLAY_REPLICATION_PASS' if not c.failures else 'REPLAY_REPLICATION_FAILED',
                GT_LOADED=False,failure_count=len(c.failures),first_differences=chronological[:20],
                tolerance_translation_m=1e-9,tolerance_rotation_deg=1e-8,
                counts=dict(P18_NDT=len(ndt_a),P19_NDT=len(ndt_b)),
                exclusions=dict(NDT=sorted(timing),callback_arrival='legal arrival snapshots are not fixed sensor-time outputs'))
    (output/'replication.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--previous',type=Path,required=True)
    p.add_argument('--repeat',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();r=compare(a.previous,a.repeat,a.output)
    raise SystemExit(0 if r['failure_count']==0 else 1)
