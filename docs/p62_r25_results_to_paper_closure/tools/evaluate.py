#!/usr/bin/env python3
"""Post-seal R25 G-only evaluation. Never imported by the ROS localization nodes.

EKF error is evaluated at its actual published current-state timestamp, not at
LiDAR scan start. Raw/selected/published NDT poses use scan-start timestamps.
Reference is the conditional Base, map-anchored benchmark; no alignment fit.
"""
import argparse
import bisect
import csv
import hashlib
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

HERE = Path(__file__).resolve().parents[1]
REPO = HERE.parents[1]
GT = Path('/home/jian/livox_ws/p59_r3_newer_college_official_internal_recovery_data/01_short_experiment/ground_truth/registered_poses.csv')
BASIS = 'MAP_ANCHORED_BENCHMARK_REFERENCE; CONDITIONAL_BASE'
POLICIES = ('G_ONLY',)
EXPECTED = {('W1', 'C3', 'G_ONLY')}
REQUIRED = ('publisher_receipts.csv', 'localization_trajectory.csv',
            'r23_ndt_lineage.csv', 'oosm_statistics.csv',
            'ekf_prediction_diagnostics.csv', 'deskew_diagnostics.csv',
            'r23_admission.csv', 'r23_candidates.csv',
            'local_map_diagnostics.csv', 'ndt_diagnostics.csv', 'RUN_RECEIPT.json', 'r24_geometry.csv', 'r24_revision.csv')
STAMP_TOL_NS = 1000  # existing publisher double-seconds -> ROS stamp tolerance


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1048576), b''):
            h.update(block)
    return h.hexdigest()


def rows(path):
    with Path(path).open(newline='') as f:
        return list(csv.DictReader(f))


def number(value):
    if value is None or value in ('NA', ''):
        return None
    x = float(value)
    return x if math.isfinite(x) else None


def truth(value):
    return str(value).lower() in ('1', 'true')


def stat(values, rmse=False):
    v = [float(x) for x in values if x is not None and math.isfinite(float(x))]
    if not v:
        return {'n': 0, 'mean': None, 'p95': None, 'max': None,
                **({'rmse': None, 'median': None} if rmse else {})}
    result = {'n': len(v), 'mean': float(np.mean(v)),
              'p95': float(np.percentile(v, 95)), 'max': max(v)}
    if rmse:
        result.update(rmse=float(np.sqrt(np.mean(np.square(v)))),
                      median=float(np.median(v)))
    return result


def longest(flags):
    best = run = 0
    for value in flags:
        run = run + 1 if value else 0
        best = max(best, run)
    return best


def matrix(row, prefix):
    values = [number(row.get(prefix + str(i) + str(j)))
              for i in range(4) for j in range(4)]
    if any(x is None for x in values):
        return None
    pose = np.array(values).reshape(4, 4)
    if (np.linalg.norm(pose[3] - [0, 0, 0, 1]) > 1e-6 or
            np.linalg.norm(pose[:3, :3].T @ pose[:3, :3] - np.eye(3)) > 1e-4 or
            abs(np.linalg.det(pose[:3, :3]) - 1) > 1e-4):
        raise RuntimeError('invalid SE3 in sealed matrix record')
    return pose


def odom_pose(row):
    p = np.array([float(row[x]) for x in ('px', 'py', 'pz')])
    q = np.array([float(row[x]) for x in ('qx', 'qy', 'qz', 'qw')])
    if not np.isfinite(p).all() or not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > 1e-5:
        raise RuntimeError('nonfinite/invalid actual ROS pose')
    out = np.eye(4)
    out[:3, :3] = Rotation.from_quat(q).as_matrix()
    out[:3, 3] = p
    return out


def flatten_pose(pose, prefix):
    return {prefix + str(i) + str(j): None if pose is None else float(pose[i, j])
            for i in range(4) for j in range(4)}


def pose_error(estimate, reference):
    if estimate is None or reference is None:
        return None, None
    translation = float(np.linalg.norm(estimate[:3, 3] - reference[:3, 3]))
    cosine = float(np.clip((np.trace(reference[:3, :3].T @ estimate[:3, :3]) - 1) / 2, -1, 1))
    return translation, math.degrees(math.acos(cosine))


def write_csv(path, records):
    if not records:
        raise RuntimeError('refuse an unlabelled empty evaluation output')
    fields = list(dict.fromkeys(k for r in records for k in r))
    with path.open('x', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields, lineterminator='\n')
        writer.writeheader()
        for r in records:
            writer.writerow({k: 'NA' if r.get(k) is None else r.get(k) for k in fields})


class Reference:
    """Bounded posthoc interpolation of the original Base-frame reference."""
    def __init__(self, path, expected_sha, t_bl):
        if sha(path) != expected_sha:
            raise RuntimeError('official reference identity changed')
        self.stamps, self.lines = [], []
        self.t_bl = np.asarray(t_bl)
        with path.open() as f:
            next(f)
            for line in f:
                sec, nsec, remaining = line.rstrip().split(',', 2)
                self.stamps.append(int(sec) * 1000000000 + int(nsec))
                self.lines.append(remaining)
        if any(a >= b for a, b in zip(self.stamps, self.stamps[1:])):
            raise RuntimeError('reference timestamps not strictly increasing')
        self.cache = {}

    def base(self, i):
        if i not in self.cache:
            v = np.array([float(x) for x in self.lines[i].split(',')])
            if len(v) != 7 or not np.isfinite(v).all() or abs(np.linalg.norm(v[3:]) - 1) > 2e-5:
                raise RuntimeError('invalid reference record')
            self.cache[i] = v
        return self.cache[i]

    def at(self, stamp):
        pos = bisect.bisect_left(self.stamps, stamp)
        receipt = {'reference_type': BASIS, 'reference_valid': False,
                   'reference_status': 'NO_BRACKET', 'reference_before_ns': None,
                   'reference_after_ns': None, 'reference_before_distance_ms': None,
                   'reference_after_distance_ms': None, 'reference_gap_ms': None}
        if pos < len(self.stamps) and self.stamps[pos] == stamp:
            a = b = pos
        elif pos == 0 or pos == len(self.stamps):
            return None, receipt
        else:
            a, b = pos - 1, pos
        ta, tb = self.stamps[a], self.stamps[b]
        da, db = (stamp - ta) / 1e6, (tb - stamp) / 1e6
        receipt.update(reference_before_ns=ta, reference_after_ns=tb,
                       reference_before_distance_ms=da, reference_after_distance_ms=db,
                       reference_gap_ms=(tb - ta) / 1e6)
        if tb - ta > 150000000 or min(da, db) > 50:
            receipt['reference_status'] = 'GAP_OR_NEAREST_LIMIT'
            return None, receipt
        va, vb = self.base(a), self.base(b)
        out = np.eye(4)
        if a == b:
            out[:3, :3] = Rotation.from_quat(va[3:]).as_matrix()
            out[:3, 3] = va[:3]
        else:
            weight = (stamp - ta) / (tb - ta)
            out[:3, 3] = (1 - weight) * va[:3] + weight * vb[:3]
            out[:3, :3] = Slerp([0., 1.], Rotation.from_quat([va[3:], vb[3:]]))([weight]).as_matrix()[0]
        receipt.update(reference_valid=True, reference_status='EXACT' if a == b else 'BOUNDED_INTERPOLATION')
        return out @ self.t_bl, receipt


def seal_runs(path):
    seal = json.loads(path.read_text())
    if not (seal.get('all_formal_outputs_frozen_before_reference') is True or
            seal.get('all_native_outputs_frozen_before_reference') is True):
        raise RuntimeError('all formal outputs must be sealed before reference access')
    runs = seal.get('formal_runs', seal.get('runs'))
    if isinstance(runs, dict):
        runs = list(runs.values())
    if not isinstance(runs, list) or len(runs) != 1:
        raise RuntimeError('exactly one approved G-only formal run must be frozen')
    actual = set()
    for run in runs:
        key = run['window'], run['arm'], run['strategy']
        if key in actual:
            raise RuntimeError('duplicate formal trajectory')
        actual.add(key)
        folder = Path(run.get('path', run.get('run_dir', '')))
        if not folder.is_absolute():
            raise RuntimeError('formal run must have an explicit absolute path')
        digests = run.get('files_sha256', {})
        global_digests = seal.get('files_sha256', {})
        for name in REQUIRED:
            f = folder / name
            expected = digests.get(name, global_digests.get(str(f)))
            if expected is None or sha(f) != expected:
                raise RuntimeError('missing/changed sealed output: ' + str(f))
        run['_folder'] = folder
    if actual != EXPECTED:
        raise RuntimeError('formal set differs from W1/C3/G_ONLY trajectory')
    return seal, runs


def wall_matched_output(receipt, trajectory):
    stamp = int(receipt['ekf_imu_watermark_ns'])
    begin, ack = float(receipt['wall_publish_sec']), number(receipt['wall_ekf_ack_sec'])
    published = truth(receipt['ndt_measurement_published'])
    if published:
        if ack is None or not truth(receipt['ekf_correction_ack']):
            raise RuntimeError('published measurement lacks actual EKF correction acknowledgement')
        candidates = [r for r in trajectory if r['source'] == 'EKF_CORRECTED' and
                      begin <= float(r['receipt_wall_sec']) <= ack and
                      abs(int(r['stamp_ns']) - stamp) <= STAMP_TOL_NS]
    else:
        if truth(receipt['ekf_correction_ack']):
            raise RuntimeError('rejected measurement has a fake correction acknowledgement')
        # The input publisher waits for this exact high-rate watermark before cloud publication.
        candidates = [r for r in trajectory if r['source'] == 'EKF_HIGH_RATE' and
                      int(r['stamp_ns']) == stamp and float(r['receipt_wall_sec']) <= begin]
    if len(candidates) != 1:
        raise RuntimeError('cannot identify unique same-frame actual EKF output: ' + str((receipt['scan_index'], published, len(candidates))))
    selected = candidates[0]
    if selected['frame_id'] != 'newer_college_map' or selected['child_frame_id'] != 'os1_lidar':
        raise RuntimeError('actual EKF output frame mismatch')
    return selected


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seal', type=Path, default=HERE / 'NATIVE_OUTPUT_FREEZE.json')
    args = parser.parse_args()
    seal, runs = seal_runs(args.seal)  # No GT values are read above this line.
    contract = json.loads((HERE / 'input/INITIALIZATION_AND_TIME_CONTRACT.json').read_text())
    r22 = json.loads((REPO / 'docs/p62_r22_frozen_relative_holdout/WINDOW_IDENTITY.json').read_text())
    reference = Reference(GT, r22['reference_sha256'], r22['T_B_L'])
    causal_rows, error_rows, lineage_rows, admission_rows, runtime_rows = [], [], [], [], []
    summaries, extra_count, nominal_count = {}, 0, 0
    run_errors = {}
    for run in runs:
        window, arm, policy = run['window'], run['arm'], run['strategy']
        meta = {'window': window, 'arm': arm, 'strategy': policy}
        folder = run['_folder']
        receipts, traj, ndt = (rows(folder / n) for n in ('publisher_receipts.csv', 'localization_trajectory.csv', 'r23_ndt_lineage.csv'))
        admissions, candidates = rows(folder / 'r23_admission.csv'), rows(folder / 'r23_candidates.csv')
        oosm, events = rows(folder / 'oosm_statistics.csv'), rows(folder / 'ekf_prediction_diagnostics.csv')
        deskew, maps = rows(folder / 'deskew_diagnostics.csv'), rows(folder / 'local_map_diagnostics.csv')
        process = json.loads((folder / 'RUN_RECEIPT.json').read_text())
        if len(receipts) != 60 or len(ndt) != 60 or len(admissions) != 60 or len(deskew) != 60:
            raise RuntimeError('incomplete formal60-frame trajectory: ' + str(folder))
        if process['process_exit_code'] != 0 or process['timed_out'] or process['interrupted']:
            raise RuntimeError('formal process did not complete legally')
        frames = contract['windows'][window]['frames'][1:]
        ndt_by_stamp = {int(r['scan_ns']): r for r in ndt}
        admission_by_stamp = {int(r['scan_ns']): r for r in admissions}
        map_by_stamp = {int(r['scan_ns']): r for r in maps}
        cand_by_stamp = defaultdict(list)
        for c in candidates:
            cand_by_stamp[int(c['scan_ns'])].append(c)
        accepted, relative, event_count, used = 0, 0, 0, 0
        bad, no_map, et, er = [], [], [], []
        per_estimate = defaultdict(list)
        callbacks, roundtrips, event_times = [], [], []
        advancing_without_map = 0
        previous_output_stamp = int(contract['windows'][window]['imu_replay_start_stamp_ns'])
        run_errors[(window, arm, policy)] = []
        for index, receipt in enumerate(receipts, 1):
            stamp = int(receipt['scan_stamp_ns'])
            frozen = frames[index - 1]
            if int(receipt['scan_index']) != index or stamp != int(frozen['stamp_ns']):
                raise RuntimeError('scan identity/order differs from frozen input')
            if not truth(receipt['ndt_diagnostic_ack']) or not truth(receipt['ndt_measurement_published_known']):
                raise RuntimeError('missing explicit admission acknowledgement')
            n, a = ndt_by_stamp[stamp], admission_by_stamp[stamp]
            if int(n['reference_ns']) != stamp:
                raise RuntimeError('Deskew/NDT reference is not the frozen scan start')
            published = truth(receipt['ndt_measurement_published'])
            if published != truth(n['measurement_published']) or published != truth(a['measurement_published']):
                raise RuntimeError('measurement publication status disagrees')
            if int(n['ekf_sample_ns']) > stamp or not 0 <= float(n['prediction_age_ms']) <= 20:
                raise RuntimeError('prediction time is noncausal/stale')
            attempted = int(a['nominal_attempted'])
            if float(n['ekf_sample_accept_wall']) > float(n['prediction_query_wall']):
                raise RuntimeError('prediction delivery/query wall ordering invalid')
            if attempted and not (float(n['prediction_query_wall']) <= float(n['ndt_start_wall']) <= float(n['ndt_complete_wall'])):
                raise RuntimeError('prediction/native wall ordering invalid')
            if not attempted and (published or int(a['extra_aligns']) != 0):
                raise RuntimeError('pre-NDT rejection published a fake measurement or candidates')
            if int(receipt['imu_last_ns']) < stamp + int(receipt['max_point_offset_ns']):
                raise RuntimeError('actual IMU delivery fails scan-end coverage')
            d = deskew[index - 1]
            if d['status'] != 'PUBLISHED' or truth(d['current_scan_ndt_leakage']) or truth(d['post_scan_end_imu_used']):
                raise RuntimeError('Deskew causality contract failed')
            current = wall_matched_output(receipt, traj)
            current_stamp = int(current['stamp_ns'])
            if current_stamp <= previous_output_stamp:
                raise RuntimeError('real EKF state time failed to advance')
            advancing_without_map += not published
            previous_output_stamp = current_stamp
            out_pose = odom_pose(current)
            chosen = [c for c in cand_by_stamp[stamp] if truth(c['selected'])]
            if len(chosen) > 1:
                raise RuntimeError('multiple selected raw candidates')
            selected_pose = matrix(chosen[0], 'pose') if chosen else None
            estimates = {'IMU_EKF_PREDICTION': (matrix(n, 'tp'), stamp),
                         'NDT_NOMINAL_RAW': (matrix(n, 'raw') if attempted else None, stamp),
                         'NDT_SELECTED_RAW': (selected_pose, stamp),
                         'NDT_PUBLISHED': (matrix(n, 'published') if published else None, stamp),
                         'EKF_CAUSAL_OUTPUT': (out_pose, int(current['stamp_ns']))}
            errors_here = {}
            for label, (pose, when) in estimates.items():
                gt, match = reference.at(when)
                translation, rotation = pose_error(pose, gt)
                per_estimate[label].append((translation, rotation))
                errors_here[label] = (translation, rotation)
                error_rows.append({**meta, 'frame': index, 'raw_ordinal': frozen['raw_ordinal'],
                                   'scan_ns': stamp, 'estimate': label, 'estimate_stamp_ns': when,
                                   'estimate_valid': pose is not None, 'translation_m': translation,
                                   'rotation_deg': rotation, **match})
            t, r = errors_here['EKF_CAUSAL_OUTPUT']
            et.append(t); er.append(r)
            is_bad = t is not None and (t > .5 or r > 5)
            bad.append(is_bad); no_map.append(not published)
            run_errors[(window, arm, policy)].append((t, r))
            frame_extras = int(a['extra_aligns']); used += frame_extras
            attempted = int(a['nominal_attempted'])
            nominal_count += attempted; extra_count += frame_extras
            if frame_extras not in (0, 2) or int(a['budget_used']) != used or used > 12:
                raise RuntimeError('extra native budget/identity failed')
            if truth(a['absolute_pass']) and frame_extras:
                raise RuntimeError('normal absolute-pass frame performed extra search')
            if policy == 'BASELINE' and frame_extras:
                raise RuntimeError('baseline performed candidate search')
            if len(cand_by_stamp[stamp]) != attempted + frame_extras:
                raise RuntimeError('native candidate/call count differs')
            was_relative = truth(a['relative_only'])
            if was_relative and policy != 'RELATIVE_D_EXPERIMENTAL':
                raise RuntimeError('relative admission leaked into nonexperimental policy')
            accepted += published; relative += was_relative; event_count += truth(a['quality_event'])
            matching_oosm = [o for o in oosm if abs(round(float(o['ndt_stamp']) * 1e9) - stamp) <= STAMP_TOL_NS]
            if published and len(matching_oosm) != 1:
                raise RuntimeError('published observation lacks a unique OOSM record')
            if not published and matching_oosm:
                raise RuntimeError('rejected observation was still sent to OOSM')
            o = matching_oosm[0] if matching_oosm else {}
            if o and (float(o['rollback_stamp']) > float(o['ndt_stamp']) + 1e-6 or
                      float(o['ndt_stamp']) > float(o['state_now_stamp']) + 1e-6 or
                      float(o['alignment_error_ms']) > 20.001):
                raise RuntimeError('historical application time invalid')
            m = map_by_stamp.get(stamp, {})
            if m and int(m['target_points']) != int(n['target_points']):
                raise RuntimeError('map target record disagrees with actual NDT')
            frame_meta = {**meta, 'frame': index, 'raw_ordinal': frozen['raw_ordinal'], 'scan_ns': stamp}
            causal_rows.append({**frame_meta, 'scan_end_ns': stamp + int(receipt['max_point_offset_ns']),
                                'deskew_reference_ns': int(n['reference_ns']),
                                'prediction_sample_ns': int(n['ekf_sample_ns']),
                                'ekf_output_stamp_ns': int(current['stamp_ns']), 'ekf_output_source': current['source'],
                                'measurement_published': published, 'relative_only': was_relative,
                                'target_hash': n['target_hash'], 'target_points': n['target_points'],
                                'loaded_tile_ids': m.get('loaded_tile_ids'),
                                'ndt_update_result': o.get('oosm_result'),
                                **flatten_pose(out_pose, 'ekf'), **flatten_pose(matrix(n, 'tp'), 'tp')})
            nominal_error = errors_here['NDT_NOMINAL_RAW']
            chosen_error = errors_here['NDT_SELECTED_RAW']
            comparable_error = all(x is not None for x in nominal_error + chosen_error)
            admission_rows.append({**frame_meta, **a,
                                   'reference_selected_raw_translation_m': chosen_error[0],
                                   'reference_selected_raw_rotation_deg': chosen_error[1],
                                   'reference_improvement_vs_nominal_translation_m': nominal_error[0] - chosen_error[0] if comparable_error else None,
                                   'reference_improvement_vs_nominal_rotation_deg': nominal_error[1] - chosen_error[1] if comparable_error else None,
                                   'actual_EKF_output_stamp_ns': int(current['stamp_ns']),
                                   'actual_EKF_translation_error_m': t, 'actual_EKF_rotation_error_deg': r})
            lineage_rows.append({**frame_meta, 'record_type': 'NDT_FUSION_LINK',
                                 'scan_end_ns': stamp + int(receipt['max_point_offset_ns']),
                                 'deskew_reference_ns': int(n['reference_ns']),
                                 'ekf_sample_ns': n['ekf_sample_ns'], 'prediction_query_wall': n['prediction_query_wall'],
                                 'ndt_start_wall': n['ndt_start_wall'], 'ndt_complete_wall': n['ndt_complete_wall'],
                                 'measurement_published': published, 'measurement_time_ns': stamp,
                                 'ekf_receive_wall': next((e['event_ros_stamp'] for e in events if
                                     e['event_type'] == 'NDT_CALLBACK_ENTER' and number(e['ndt_measurement_stamp']) is not None and
                                     abs(round(float(e['ndt_measurement_stamp']) * 1e9) - stamp) <= STAMP_TOL_NS), None),
                                 'actual_historical_application_stamp_sec': o.get('rollback_stamp'),
                                 'state_now_stamp_sec': o.get('state_now_stamp'),
                                 'alignment_error_ms': o.get('alignment_error_ms'),
                                 'oosm_result': o.get('oosm_result'), 'replay_imu_count': o.get('replay_imu_count')})
            callback = float(n['callback_wall_ms']); callbacks.append(callback)
            ack_wall = float(receipt['wall_ekf_ack_sec']) if published else float(receipt['wall_ndt_ack_sec'])
            roundtrip = (ack_wall - float(receipt['wall_publish_sec'])) * 1000
            roundtrips.append(roundtrip)
            if truth(a['quality_event']):
                event_times.append(callback)
            runtime_rows.append({**frame_meta, 'quality_event': a['quality_event'], 'extra_aligns': frame_extras,
                                 'ndt_callback_wall_ms': callback, 'input_to_final_ack_ms': roundtrip,
                                 'diagnostic_align_ms': receipt['ndt_align_ms'],
                                 'nominal_native_align_ms': next((c['align_ms'] for c in cand_by_stamp[stamp] if int(c['id']) == 0), None),
                                 'extra_native_align_ms': sum(float(c['align_ms']) for c in cand_by_stamp[stamp] if int(c['id']) > 0),
                                 'target_build_ms': m.get('target_build_ms'),
                                 'target_rebuild_ms': m.get('target_rebuild_ms'),
                                 'tile_io_ms': m.get('tile_io_ms'),
                                 'ndt_node_peak_rss_KiB': process['ndt_peak_rss_KiB'],
                                 'ekf_node_peak_rss_KiB': process['ekf_peak_rss_KiB'],
                                 'simultaneous_ndt_ekf_peak_rss_KiB': process['simultaneous_ndt_ekf_peak_rss_KiB']})
        event_counts = Counter(e['event_type'] for e in events)
        for e in events:
            lineage_rows.append({**meta, 'record_type': 'EKF_EVENT', 'state_frame': 'IMU', **e})
        attempts = sum(int(a['nominal_attempted']) for a in admissions)
        converged_attempts = sum(truth(a['nominal_attempted']) and truth(a['nominal_converged']) for a in admissions)
        pre_ndt_rejections = sum(not truth(a['nominal_attempted']) for a in admissions)
        native_events = sum(truth(a['nominal_attempted']) and truth(a['quality_event']) for a in admissions)
        estimate_summary = {label: {'translation_m': stat([x[0] for x in es], True),
                                    'rotation_deg': stat([x[1] for x in es], True)}
                            for label, es in per_estimate.items()}
        rejected_errors = [e for e, missing in zip(run_errors[(window, arm, policy)], no_map) if missing]
        nominals = [c for c in candidates if int(c['id']) == 0]
        summaries['/'.join((window, arm, policy))] = {
            'samples': 60, 'reference_relative_errors': estimate_summary,
            'nominal_ndt_attempts': attempts,
            'nominal_ndt_converged': converged_attempts,
            'nominal_ndt_convergence_rate': converged_attempts / attempts if attempts else None,
            'all_native_converged': sum(truth(c['converged']) for c in candidates),
            'pre_ndt_rejected_frames': pre_ndt_rejections,
            'pre_ndt_guard_rejections': pre_ndt_rejections,
            'rejected_after_native_attempt_frames': sum(truth(a['nominal_attempted']) and not truth(a['measurement_published']) for a in admissions),
            'all_native_calls': len(candidates), 'extra_full_ndt': used,
            'basic_quality_failures': sum(truth(a['nominal_attempted']) and not truth(a['basic_quality']) for a in admissions),
            'nominal_basic_quality_failure_rate': sum(truth(a['nominal_attempted']) and not truth(a['basic_quality']) for a in admissions) / attempts if attempts else None,
            'absolute_quality_rejections_after_basic_pass': sum(truth(a['nominal_attempted']) and truth(a['basic_quality']) and not truth(a['absolute_pass']) for a in admissions),
            'quality_events': native_events,
            'all_scan_quality_events_including_pre_ndt_guard': event_count, 'map_observations_published': accepted,
            'map_observations_rejected': 60 - accepted, 'relative_only_admissions': relative,
            'selected_extra_candidates': sum(int(a['selected_id']) > 0 for a in admissions),
            'imu_high_rate_propagations': event_counts['IMU_HIGH_RATE_PUBLISH'],
            'imu_raw_message_count': max(int(e['imu_msg_count']) for e in events),
            'imu_propagations_including_oosm_replay': event_counts['IMU_HIGH_RATE_PUBLISH'] + sum(int(o['replay_imu_count']) for o in oosm),
            'ndt_callbacks_into_ekf': event_counts['NDT_CALLBACK_ENTER'],
            'ndt_corrections_applied': event_counts['NDT_CORRECTION_APPLIED'],
            'oosm_results': dict(Counter(o['oosm_result'] for o in oosm)),
            'replayed_imu_count': sum(int(o['replay_imu_count']) for o in oosm),
            'oosm_alignment_error_ms': stat([number(o['alignment_error_ms']) for o in oosm]),
            'no_map_observation_frames': sum(no_map), 'longest_no_map_run': longest(no_map),
            'no_map_frames_with_real_imu_state_time_advancing': advancing_without_map,
            'ekf_high_error_frames': sum(bad), 'longest_high_error_run': longest(bad),
            'no_map_ekf_translation_m': stat([x[0] for x in rejected_errors], True),
            'no_map_ekf_rotation_deg': stat([x[1] for x in rejected_errors], True),
            'ndt_callback_wall_ms': stat(callbacks), 'event_frame_callback_ms': stat(event_times),
            'scan_input_to_final_ack_ms': stat(roundtrips), 'process_wall_seconds': process['wall_seconds'],
            'ndt_node_peak_rss_KiB': process['ndt_peak_rss_KiB'],
            'ekf_node_peak_rss_KiB': process['ekf_peak_rss_KiB'],
            'simultaneous_node_peak_rss_KiB': process['simultaneous_ndt_ekf_peak_rss_KiB'],
            'actual_target_hashes': sorted(set(n['target_hash'] for n in ndt)),
            'actual_target_points': sorted(set(int(n['target_points']) for n in ndt)),
            'native_nominal_score': stat([number(c['native_score']) for c in nominals]),
            'native_nominal_overlap': stat([number(c['overlap']) for c in nominals]),
            'coverage_consistency_mean': stat([number(c['coverage_mean']) for c in nominals]),
            'recovery_success_not_equated_to_admission': True}
    if not 0 <= nominal_count <= 60 or extra_count > 0 or nominal_count + extra_count > 60:
        raise RuntimeError('formal experiment native budget violated')
    comparisons = []
    baseline = json.loads((REPO/'docs/p62_r23_real_imu_closure/EVALUATION_SUMMARY.json').read_text())['trajectories']
    for key, v in summaries.items():
        window, arm, mode = key.split('/')
        base = baseline['/'.join((window,arm,'MULTISTART_C'))]
        e,b = v['reference_relative_errors']['EKF_CAUSAL_OUTPUT'],base['reference_relative_errors']['EKF_CAUSAL_OUTPUT']
        dt=e['translation_m']['rmse']-b['translation_m']['rmse'];dr=e['rotation_deg']['rmse']-b['rotation_deg']['rmse']
        comparisons.append({'window':window,'arm':arm,'mode':mode,'delta_translation_rmse_m':dt,'delta_rotation_rmse_deg':dr,
            'C0_side_effect':arm=='C0' and (dt>.03 or dr>.3),'W1_C3_target_met':window=='W1' and arm=='C3' and e['translation_m']['rmse']<=1. and e['rotation_deg']['rmse']<=5.})
    geometry_rows, revision_rows = [], []
    for run in runs:
        meta={k:run[k] for k in ('window','arm','strategy')}
        folder=run['_folder'];ad={int(a['scan_ns']):a for a in rows(folder/'r23_admission.csv')}
        ge=rows(folder/'r24_geometry.csv');rev=rows(folder/'r24_revision.csv');key='/'.join(meta[k] for k in ('window','arm','strategy'))
        assert len(ge)==60
        for g in ge:
            a=ad[int(g['scan_ns'])]
            assert float(g['median_limit'])==1. and float(g['p90_limit'])==3. and float(g['overlap_limit'])==.5
            bypass=not truth(g['original_gate_passed']) and truth(g['diagnostic_only'])
            if bypass:assert truth(a['nominal_attempted'])
            geometry_rows.append({**meta,**g,'actually_attempted':a['nominal_attempted'],'measurement_published':a['measurement_published']})
        for q in rev:
            if q['event']=='QUERY':
                assert int(q['message_ns'])<=int(q['scan_ns']) and 0<=float(q['prediction_age_ms'])<=20
            if q['event']=='CORRECTED' and truth(q['accepted']):
                assert int(q['message_ns'])==int(q['latest_history_ns']) and q['reason']=='REPLACED_EXACT_LATEST'
            revision_rows.append({**meta,**q})
        v=summaries[key]
        v['original_geometry_gate_rejections']=sum(not truth(g['original_gate_passed']) for g in ge)
        v['originally_rejected_but_attempted']=sum(not truth(g['original_gate_passed']) and truth(ad[int(g['scan_ns'])]['nominal_attempted']) for g in ge)
        v['corrected_messages_received']=sum(q['event']=='CORRECTED' for q in rev)
        v['corrected_latest_replacements']=sum(q['event']=='CORRECTED' and truth(q['accepted']) for q in rev)
        v['queries_using_replaced_sample']=sum(q['event']=='QUERY' and int(q['sample_corrected_version'])>0 for q in rev)
        v['corrected_rejection_reasons']=dict(Counter(q['reason'] for q in rev if q['event']=='CORRECTED' and not truth(q['accepted'])))
    outputs = {'CAUSAL_RESULTS.csv': causal_rows, 'REFERENCE_ERRORS.csv': error_rows,
               'IMU_EKF_LINEAGE.csv': lineage_rows, 'NDT_ADMISSION_LOG.csv': admission_rows,
               'RUNTIME_MEMORY.csv': runtime_rows,'GEOMETRY_GATE_LOG.csv':geometry_rows,'EKF_REVISION_FRESHNESS.csv':revision_rows}
    for name, records in outputs.items():
        write_csv(HERE / name, records)
    result = {'task': 'P62-R25-RESULTS-TO-PAPER-CLOSURE', 'formal_run_count': 1,
              'nominal_full_calls': nominal_count, 'extra_full_calls': extra_count,
              'total_formal_full_calls': nominal_count + extra_count,
              'reference_type': BASIS, 'reference_sha256': sha(GT),
              'native_output_seal_sha256': sha(args.seal), 'trajectories': summaries,
              'vs_R23_C': comparisons,
              'reference_time_rule': 'exact or bracket gap<=150ms and nearest endpoint<=50ms; translation linear, rotation SLERP in Base then fixed T_B_L; no fitted offset/alignment',
              'EKF_time_rule': 'same-frame actual corrected current-state publication after accepted map measurement; otherwise actual high-rate watermark publication; never compared against scan-start reference',
              'limitations': ['known development windows', 'conditional Base reference exported by map registration, not independent mocap',
                              'Coverage Consistency Mean points participate in Full NDT',
                              'existing zero-velocity/bias anchor priors; no independent calibration or GT velocity',
                              'F only replaces exact latest corrected timestamp; actual query consumption separately measured',
                              'all native candidates precede reference evaluation; admission does not imply correctness',
                              'pre-NDT geometry guard rejections are separate from optimizer/basic/absolute quality failures; zero attempts gives unavailable convergence rate']}
    with (HERE / 'EVALUATION_SUMMARY.json').open('x') as f:
        json.dump(result, f, indent=2, ensure_ascii=False, allow_nan=False)
        f.write('\n')
    print(json.dumps({'formal_runs': 1, 'nominal': nominal_count, 'extra': extra_count, 'vs_R23_C': comparisons}, ensure_ascii=False))


if __name__ == '__main__':
    main()
