#!/usr/bin/env python3
"""Summarize the fixed small offline experiment; never use location GT."""
import argparse
import csv
import hashlib
import json
import shutil
from collections import Counter
from pathlib import Path

import numpy as np


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def statistics(values):
    values = np.asarray(values, dtype=float)
    assert len(values) and np.isfinite(values).all()
    return dict(mean=float(values.mean()), p95=float(np.quantile(values, .95)),
                max=float(values.max()), min=float(values.min()))


def write_csv(path, records):
    with path.open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(records)


def main(experiment, archive, code_sha):
    samples = rows(experiment/'samples.csv')
    registration, timing, parity = [], [], []
    for directory in ('smoke', 'cohort'):
        registration.extend(rows(experiment/directory/'registration.csv'))
        timing.extend(rows(experiment/directory/'timing.csv'))
        parity.extend(rows(experiment/directory/'source_parity.csv'))
    expected = [int(r['frame_index']) for r in samples]
    assert [int(r['frame_index']) for r in timing] == expected
    assert len(registration) == 96 and len(parity) == 24
    assert all(r['parity'] == 'PASS' for r in parity)
    assert len({r['target_hash'] for r in parity}) == 1
    assert len({r['target_points'] for r in parity}) == 1
    for frame in expected:
        assert sorted(int(r['rank']) for r in registration if int(r['frame_index']) == frame) == [0, 1, 2, 3]
    previous_queries = {int(r['stamp_ns']): r for r in rows(Path('/home/jian/rosbag/p11_solid_r1_20261011/solid_shadow/solid.csv'))}
    for r in registration:
        if int(r['rank']):
            old = previous_queries[int(r['stamp_ns'])]
            assert r['reference_id'] == old[f"top{r['rank']}_id"]
            assert abs(float(r['similarity'])-float(old[f"top{r['rank']}_similarity"])) < 1e-12
            assert r['coordinate_status'] == 'COARSE_POSITION_HYPOTHESIS'
    local = [r for r in registration if r['rank'] == '0']
    candidates = [r for r in registration if r['rank'] != '0']

    def metrics(records):
        valid = [r for r in records if r['valid_se3'] == '1']
        residuals = [float(r['rmse_before_last_update_m']) for r in valid]
        finite_residuals = [v for v in residuals if np.isfinite(v)]
        return dict(calls=len(records), converged=sum(r['converged'] == '1' for r in records),
                    valid_se3=len(valid), nonfinite_pose=sum(not all(np.isfinite(float(r[f'final_{field}'])) for field in ('tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw')) for r in records),
                    large_excursions=sum(r['large_excursion'] == '1' for r in records),
                    degenerate_any_iteration=sum('1' in r['degenerate_mask_rpy_xyz'] for r in records),
                    failure_reasons=dict(Counter(r['failure_reason'] for r in records if r['failure_reason'])),
                    iterations=statistics([r['iterations'] for r in records]),
                    native_preupdate_rmse_m=statistics(finite_residuals) if finite_residuals else None,
                    native_fitness=statistics([r['fitness_correspondence_fraction'] for r in records]),
                    call_ms=statistics([r['call_wall_ms'] for r in records]))

    output = dict(task='P12-DCREG-SOLID-MINIMAL-INTEGRATION-R1',
                  final_result='MINIMAL_INTEGRATION_PARTIAL',
                  start_sha='578291c999ea375ff72538da7b6a34fb54dc85bb', code_sha=code_sha,
                  protocol=json.loads((experiment/'freeze.json').read_text()),
                  actual_scans=24, references=389, source_parity=24,
                  target_points=int(parity[0]['target_points']), target_hash=parity[0]['target_hash'],
                  solid_valid_queries=24, top3_prior_query_parity=72,
                  local=metrics(local), solid_candidates=metrics(candidates),
                  exact_reference_to_lidar_conversions=0, coarse_hypotheses=72,
                  new_full_ndt_calls=0, dcreg_calls=96, feedback_calls=0, gt_loaded=False,
                  accuracy_result='ACCURACY_GAIN_NOT_EVALUATED',
                  next='ESTABLISH_REFERENCE_TO_LIDAR_POSE_CONTRACT',
                  stage_ms={name: statistics([r[name] for r in timing]) for name in
                            ('preprocessing_ms', 'descriptor_ms', 'search_ms', 'query_wall_ms',
                             'local_dcreg_ms', 'top3_dcreg_ms', 'combination_ms', 'total_probe_ms')},
                  peak_rss_kib=max(int(r['peak_rss_kib']) for r in timing),
                  limits=['No exact historical reference-state-to-LiDAR/time transform proof',
                          'No independent place labels or localization accuracy test',
                          'Straight-motion strata do not certify physical corridor annotation',
                          'Original baseline deskew=false remains unchanged'])
    if (not output['local']['converged'] or not output['solid_candidates']['converged'] or
            output['local']['nonfinite_pose'] or output['solid_candidates']['nonfinite_pose']):
        output['final_result'] = 'MINIMAL_INTEGRATION_FAIL'
        output['next'] = 'REPAIR_MINIMAL_INTEGRATION_INTERFACE'
    archive.mkdir(parents=True, exist_ok=True)
    for name, records in [('registration.csv', registration), ('timing.csv', timing), ('source_parity.csv', parity)]:
        write_csv(archive/name, records)
    # Git archive uses LF; original external freeze bytes/hash remain immutable.
    for name in ('samples.csv', 'extraction.csv'):
        write_csv(archive/name, rows(experiment/name))
    shutil.copyfile(experiment/'freeze.json', archive/'freeze.json')
    with (archive/'results.json').open('x') as stream:
        json.dump(output, stream, indent=2, allow_nan=False)
    print(json.dumps(output, indent=2, allow_nan=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('experiment', type=Path)
    parser.add_argument('archive', type=Path)
    parser.add_argument('code_sha')
    args = parser.parse_args()
    main(args.experiment, args.archive, args.code_sha)
