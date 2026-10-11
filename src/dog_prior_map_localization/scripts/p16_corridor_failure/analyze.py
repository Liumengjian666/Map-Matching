#!/usr/bin/env python3
"""Frozen-output diagnosis only: no registration, replay, fitting, or tuning."""
import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np

PACKAGE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PACKAGE / 'scripts/p10_corridor01_deterministic_oosm'))
import evaluate_frozen_v2 as frozen
geo = frozen.geometry


def vector(row, prefix):
    return np.array([float(row[prefix + axis]) for axis in 'xyz'])


def pose(row, prefix=None):
    if prefix is not None:
        return frozen.pose_from_fields(row, prefix)
    return vector(row, 'p_'), geo.quat_to_rotation([float(row['q_' + a]) for a in 'xyzw'])


def lidar_to_imu(value, extrinsic):
    return geo.lidar_pose_to_imu(dict(position=value[0], rotation=value[1]), extrinsic)


def delta(a, b):
    return float(np.linalg.norm(a[0] - b[0])), geo.rotation_angle_deg(a[1].T @ b[1])


def limited(raw, previous):
    """Exact frozen ROS limiter (0.5m, 5deg), not a new admission rule."""
    p, r = raw[0].copy(), raw[1].copy()
    dp = p - previous[0]
    dist, angle = delta(raw, previous)
    if dist > .5:
        p = previous[0] + dp * (.5 / dist)
    if angle > 5:
        r = geo.quat_to_rotation(geo.slerp(geo.rotation_to_quaternion(previous[1]),
                                         geo.rotation_to_quaternion(r), 5 / angle))
    return p, r


def next_position(current, previous):
    # previous_pose * delta_pose, delta_pose = previous_before.inverse() * current.
    return current[0] + current[1] @ previous[1].T @ (current[0] - previous[0])


def stats(values):
    a = np.asarray(values, dtype=float)
    assert len(a) and np.isfinite(a).all()
    return dict(count=len(a), mean=float(a.mean()), rmse=float(np.sqrt(np.mean(a*a))),
                p95=float(np.percentile(a, 95)), max=float(a.max()))


def write_csv(path, rows):
    with path.open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]), lineterminator='\n')
        w.writeheader()
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--repo', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--gt', type=Path, required=True)
    ap.add_argument('--extrinsics', type=Path, required=True)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    archive = args.repo / 'docs/p10_corridor01_deterministic_oosm_replay_r1'
    metrics = json.loads((archive / 'posthoc_metrics.json').read_text())
    assert frozen.sha256(args.gt) == metrics['input_hashes']['GT_sha256']
    assert frozen.sha256(args.extrinsics) == metrics['input_hashes']['extrinsics_sha256']
    gt_times, gt_poses = geo.load_gt(args.gt)
    ext = geo.load_t_imu_lidar(args.extrinsics)
    origin = metrics['evaluation_origin_unix_s']
    align_r = np.array(metrics['fit_transform_rotation'])
    align_p = np.array(metrics['fit_transform_translation_m'])
    # Keep the historical transform, including its prefix-conditioning limitations.
    def error(value, stamp):
        truth = geo.interpolate_gt(gt_times, gt_poses, stamp)
        assert truth is not None, stamp
        trans, rot = geo.errors_for([value], [truth], align_r, align_p)
        return float(trans[0]), float(rot[0]), align_r @ value[0] + align_p - truth[0]

    stages, events, methods, inputs = [], [], {}, {}
    for method in ('control_v2', 'weak_only_feedback_v2'):
        root = archive / 'runs' / method
        ds = frozen.read_csv(root / 'ndt_diagnostics.csv')
        records, ls = frozen.state_trajectory(root / 'ekf_lineage.csv')
        assert len(ds) == 396
        inputs[str(root.relative_to(args.repo) / 'ndt_diagnostics.csv')] = frozen.sha256(root / 'ndt_diagnostics.csv')
        inputs[str(root.relative_to(args.repo) / 'ekf_lineage.csv')] = frozen.sha256(root / 'ekf_lineage.csv')
        grouped = {}
        for r in ls:
            if r['event_type'] in ('OOSM_ROLLBACK', 'NDT_CORRECTION_APPLIED', 'OOSM_REPLAY_COMPLETE'):
                key = r['ndt_measurement_stamp']
                assert r['event_type'] not in grouped.setdefault(key, {})
                grouped[key][r['event_type']] = r
        assert len(grouped) == 396
        hr = [dict(time=stamp-origin, translation=error(value, stamp)[0], rotation=error(value, stamp)[1])
              for stamp, value in records if 0 <= stamp-origin <= 35]
        primary = [r for r in hr if r['time'] >= 3]
        actual = stats([r['translation'] for r in primary])
        assert abs(actual['rmse'] - metrics['corrected_imu_trajectory_metrics'][method]['metrics']['primary_3_35s']['translation_m']['rmse']) < 1e-9
        method_rows, next_guess_reconstruction_errors = [], []
        for i, d in enumerate(ds):
            stamp = float(d['lidar_header_stamp'])
            group = grouped[d['lidar_header_stamp']]
            before = group['OOSM_ROLLBACK']
            after = group['NDT_CORRECTION_APPLIED']
            replay = group['OOSM_REPLAY_COMPLETE']
            assert before['state_stamp'] == after['state_stamp']
            assert abs(float(replay['state_stamp']) - float(replay['state_now_before_ndt'])) < 1e-9
            assert abs(float(before['state_stamp']) - float(before['rollback_stamp'])) < 1e-9
            if not 0 <= stamp-origin <= 35:
                continue  # startup is recorded, but is outside frozen GT evaluation
            row = dict(method=method, frame=int(d['frame_index']), time_s=stamp-origin,
                       ndt_stamp=stamp, rollback_stamp=float(before['state_stamp']),
                       release_stamp=float(replay['state_stamp']),
                       rollback_alignment_ms=(stamp-float(before['state_stamp']))*1000,
                       fitness=float(d['ndt_fitness']), iterations=int(d['ndt_iterations']),
                       converged=int(d['ndt_has_converged']),
                       translation_limited=int(d['translation_limited']), rotation_limited=int(d['rotation_limited']),
                       raw_to_guess_m=float(d['raw_delta_from_guess_translation']),
                       raw_to_guess_deg=float(d['raw_delta_from_guess_rotation_deg']),
                       feedback=int(d['coupled_feedback_changed_observation']))
            for name, value, t in (
                ('guess', lidar_to_imu(pose(d, 'initial_guess_'), ext), stamp),
                ('raw', lidar_to_imu(pose(d, 'raw_ndt_'), ext), stamp),
                ('prelimit', lidar_to_imu(pose(d, 'used_before_step_limit_'), ext), stamp),
                ('used', lidar_to_imu(pose(d, 'final_used_'), ext), stamp),
                ('rollback', pose(before), float(before['state_stamp'])),
                ('corrected', pose(after), float(after['state_stamp'])),
                ('replayed', pose(replay), float(replay['state_stamp']))):
                et, er, ev = error(value, t)
                row[name+'_error_m'], row[name+'_error_deg'] = et, er
                for axis, v in zip('xyz', ev): row[name+'_error_gt_'+axis] = float(v)
            row['raw_to_used_m'], row['raw_to_used_deg'] = delta(pose(d, 'raw_ndt_'), pose(d, 'final_used_'))
            row['prelimit_to_used_m'], row['prelimit_to_used_deg'] = delta(pose(d, 'used_before_step_limit_'), pose(d, 'final_used_'))
            row['state_correction_m'], row['state_correction_deg'] = delta(pose(before), pose(after))
            row['replay_rewrite_m'] = float(replay['oosm_rewrite_translation_m'])
            row['replay_rewrite_mps'] = float(replay['oosm_rewrite_velocity_mps'])
            used_imu = lidar_to_imu(pose(d, 'final_used_'), ext)
            assert delta(used_imu, pose(after))[0] < 1e-10
            assert delta(used_imu, pose(after))[1] < 1e-5
            if i:
                previous = pose(ds[i-1], 'final_used_')
                nominal_limited = limited(pose(d, 'raw_ndt_'), previous)
                if not row['feedback']:
                    assert delta(nominal_limited, pose(d, 'final_used_'))[0] < 1e-10
                    assert delta(nominal_limited, pose(d, 'final_used_'))[1] < 1e-5
                previous_imu = lidar_to_imu(previous, ext)
                dt = stamp-float(ds[i-1]['lidar_header_stamp'])
                v_ndt = (used_imu[0]-previous_imu[0])/dt
                v_pre = vector(before, 'v_')  # correction-applied event precedes velocity blend
                v_blended = .4*v_pre+.6*v_ndt
                for label, v in [('v_ndt', v_ndt), ('v_rollback', v_pre), ('v_blended', v_blended)]:
                    for axis, x in zip('xyz', v): row[label+'_'+axis] = float(x)
                row['velocity_injection_mps'] = float(np.linalg.norm(v_blended-v_pre))
                if i+1 < len(ds):
                    actual_next = pose(ds[i+1], 'initial_guess_')[0]
                    reconstruction_error = float(np.linalg.norm(next_position(pose(d,'final_used_'), previous)-actual_next))
                    next_guess_reconstruction_errors.append(reconstruction_error)
                    # CSV quaternions normalize the original PCL float rotations.
                    # This is a float representation guard, NOT a changed replay tolerance.
                    assert reconstruction_error < 8*np.finfo(np.float32).eps
                if row['feedback']:
                    nominal_imu = lidar_to_imu(nominal_limited, ext)
                    actual_pose = pose(d, 'final_used_')
                    change = used_imu[0]-nominal_imu[0]
                    v_change = .6*change/dt
                    event = dict(frame=row['frame'], time_s=row['time_s'], weak_dimension=int(d['coupled_weak_dimension']),
                        translation_limited=row['translation_limited'], rotation_limited=row['rotation_limited'],
                        nominal_same_history_error_m=error(nominal_imu,stamp)[0],
                        feedback_same_history_error_m=row['used_error_m'],
                        measured_rotation_change_deg=delta(nominal_limited,actual_pose)[1],
                        changed_measurement_m=float(np.linalg.norm(change)),
                        direct_velocity_change_mps=float(np.linalg.norm(v_change)),
                        next_ndt_guess_change_m=float(np.linalg.norm(next_position(actual_pose,previous)-next_position(nominal_limited,previous))),
                        rollback_error_m=row['rollback_error_m'], corrected_error_m=row['corrected_error_m'],
                        replayed_error_m=row['replayed_error_m'], state_correction_m=row['state_correction_m'],
                        replay_rewrite_m=row['replay_rewrite_m'])
                    for label, v in [('measurement_delta_map_imu',change), ('velocity_delta_map',v_change),
                                     ('weak_chart_translation_map_m',np.array([float(d['coupled_weak_eta'+str(k)]) for k in range(3)])*.8)]:
                        for axis,x in zip('xyz',v): event[label+'_'+axis] = float(x)
                    events.append(event)
            else:
                for label in ('v_ndt','v_rollback','v_blended'):
                    for axis in 'xyz': row[label+'_'+axis] = None
                row['velocity_injection_mps'] = None
            method_rows.append(row)
            stages.append(row)
        windows = {}
        for label, lo, hi in [('prefix',0,3),('early',3,10),('A',6,12),('B',26,30),('C',30,35),('primary',3,35)]:
            selected = [r for r in method_rows if lo <= r['time_s'] < hi]
            windows[label] = {name:stats([r[name+'_error_m'] for r in selected])
                              for name in ('guess','raw','prelimit','used','rollback','corrected','replayed')}
            windows[label]['limited_frames'] = sum(r['rotation_limited'] or r['translation_limited'] for r in selected)
            windows[label]['feedback_frames'] = sum(r['feedback'] for r in selected)
            windows[label]['iteration_one_frames'] = sum(r['iterations']==1 for r in selected)
            windows[label]['raw_error_over_1m_fitness_under_0p05'] = sum(r['raw_error_m']>1 and r['fitness']<.05 for r in selected)
            published = [r for r in hr if lo <= r['time'] < hi]
            windows[label]['published_high_rate'] = stats([r['translation'] for r in published])
        methods[method] = dict(high_rate_primary=actual, windows=windows,
            complete_scans=len(ds), evaluated_scans=len(method_rows),
            limit_translation=sum(int(r['translation_limited']) for r in ds),
            limit_rotation=sum(int(r['rotation_limited']) for r in ds),
            used_equals_after_pose_but_stamps_differ=True,
            next_guess_reconstruction_max_m=max(next_guess_reconstruction_errors),
            first_raw_crossings={str(t):next((r['time_s'] for r in method_rows if r['time_s']>=3 and r['raw_error_m']>t),None) for t in (.5,1,2)},
            first_high_rate_crossings={str(t):next((r['time'] for r in hr if r['time']>=3 and r['translation']>t),None) for t in (.5,1,2)})
    assert len(events)==9
    write_csv(args.out/'frame_stages.csv', stages)
    write_csv(args.out/'weak_feedback_events.csv', events)
    summary = dict(task='P16-CORRIDOR01-LOCALIZATION-FAILURE-ROOT-R1', methods=methods,
        fixed_alignment=True, new_registration_calls=0, new_replays=0,
        evaluation_origin_unix_s=origin,
        evaluation='frozen Control_V2 0-3s full-pose alignment; primary 3-35s relative drift',
        stage_time_definition='measurement stages at scan-start; state stages at their own logged state_stamp',
        covariance_or_filter_math_changed=False,
        counterfactual='nine same-history algebraic limiter/velocity/next-guess substitutions only; NOT counterfactual trajectories',
        frozen_config_sha256=frozen.sha256(args.repo/'src/dog_prior_map_localization/config/p10_corridor01_prospective_causal.yaml'),
        inputs_sha256=inputs, gt_sha256=frozen.sha256(args.gt), extrinsics_sha256=frozen.sha256(args.extrinsics),
        frozen_posthoc_metrics_sha256=frozen.sha256(archive/'posthoc_metrics.json'))
    (args.out/'analysis.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    print(json.dumps(summary,indent=2,allow_nan=False))


if __name__=='__main__':
    main()
