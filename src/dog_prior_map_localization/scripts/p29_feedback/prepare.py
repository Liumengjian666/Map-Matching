#!/usr/bin/env python3
"""Freeze the prescribed P28 sensor identities and inputs, without opening GT."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import yaml

REQUESTS = ['7:1517157219794140100', '193:1517157238552942991',
            '270:1517157246318675041', '320:1517157251361368895',
            '340:1517157253378448009', '347:1517157254084407091',
            '365:1517157255899825096', '396:1517157259026242018']


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def rows(path):
    with Path(path).open(newline='') as f:
        return list(csv.DictReader(f))


def write_csv(path, records):
    if not records:
        raise ValueError('EMPTY_RESULT:' + str(path))
    with Path(path).open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(dict.fromkeys(k for row in records for k in row)))
        writer.writeheader()
        writer.writerows(records)


def main(repo):
    repo = repo.resolve()
    cache = repo / '.p29_r1_fixed_source_feedback'
    if cache.exists():
        raise ValueError('P29_CACHE_ALREADY_EXISTS_NO_AUTOMATIC_REPLAY')
    original = repo / 'docs/p28_r2_mature_visual_module'
    freeze = json.loads((original / 'input_freeze_runtime.json').read_text())
    for path, expected in freeze['files_sha256'].items():
        if sha(path) != expected:
            raise ValueError('P28_FROZEN_INPUT_CHANGED:' + path)
    selection = []
    for method in ('A', 'B'):
        mature = {int(r['frame_index']): r for r in rows(original/'runs'/method/'mature_diagnostics.csv')}
        diag = {int(r['frame_index']): r for r in rows(original/'runs'/method/'ndt_diagnostics.csv')}
        for request in REQUESTS:
            frame, ns = request.split(':')
            row = diag[int(frame)]
            if mature[int(frame)]['scan_ns'] != ns or row['local_imu_prior_used'] != '1':
                raise ValueError('P29_FROZEN_FRAME_ID_OR_IMU_PRIOR_FAILED')
            selection.append(dict(method=method, frame_index=frame, scan_ns=ns,
                                  p28_source_hash=row['cloud_hash'],
                                  points=row['cloud_size_after_filter'],
                                  p28_selected_candidate=mature[int(frame)]['selected_candidate']))
    cache.mkdir()
    for method in ('A', 'B'):
        (cache/'sources'/method).mkdir(parents=True)
        (cache/'runs'/method).mkdir(parents=True)
        overlay = yaml.safe_load((repo/'src/dog_prior_map_localization/config/p28_mature_visual.yaml').read_text())
        overlay['output'].update(experimental_ndt_source_capture_enable=True,
            experimental_ndt_source_capture_directory=str(cache/'sources'/method),
            experimental_ndt_source_capture_requests=REQUESTS)
        (cache/('capture_'+method+'.yaml')).write_text(yaml.safe_dump(overlay, sort_keys=False))
    write_csv(cache/'frame_selection.csv', selection)
    result = dict(TASK='P29-R1-FIXED-SOURCE-NDT-FEEDBACK-ISOLATION', GT_LOADED=False,
                  START_SHA='9b6f32616fae7f408f1950ebba094f1ce72cadc1',
                  requests=REQUESTS, max_pair_aligns=32, selection=selection,
                  files_sha256=freeze['files_sha256'])
    for method in ('A', 'B'):
        path=cache/('capture_'+method+'.yaml')
        result['files_sha256'][str(path)]=sha(path)
    (cache/'input_freeze.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(dict(result='P29_INPUT_FREEZE_PASS',frames_per_run=8,GT_LOADED=False)))


if __name__ == '__main__':
    p=argparse.ArgumentParser(); p.add_argument('--repo', type=Path, required=True)
    main(**vars(p.parse_args()))
