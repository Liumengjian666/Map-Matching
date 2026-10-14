#!/usr/bin/env python3
"""Run the unchanged legacy bootstrap once; its unconditional PASS is not a gate."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

MAP_SHA = '6c39e243556eddbef7a8b4bd2de25cf7c962c4ff0f52c94cfc1d994f178ae228'
PRIOR_SHA = 'fc319ad99737a717456e11eb70383d4848eadf7e46587fee0643d6732cf1c3a1'
LEGACY_SHA = 'deb58aeb3bd76f802568469f50d33f91b8026924b3fe299fa93b8fa8f9dc266d'


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def run(command, name, output, env):
    resource = output / (name + '_resource.txt')
    timed = ['/usr/bin/time', '-f',
             'user_cpu_sec=%U\nsystem_cpu_sec=%S\npeak_rss_kib=%M\nwall_sec=%e',
             '-o', str(resource)] + list(map(str, command))
    with (output / (name + '.log')).open('x') as log:
        code = subprocess.call(timed, stdout=log, stderr=subprocess.STDOUT, env=env)
    receipt = dict(command=list(map(str, command)), exit_code=code,
                   resource_text=resource.read_text())
    (output / (name + '_execution.json')).write_text(json.dumps(receipt, indent=2) + '\n')
    return code


def main(asset, inputs, build, output, protocol):
    asset, inputs, build, output = (p.resolve() for p in (asset, inputs, build, output))
    # This directory is the one-shot guard. Never replace or reuse it.
    output.mkdir(parents=True, exist_ok=False)
    extraction = json.loads((inputs / 'extraction.json').read_text())
    for filename, expected in extraction['files_sha256'].items():
        if sha(Path(filename)) != expected:
            raise ValueError('EXTRACTED_INPUT_HASH_CHANGED: ' + filename)
    if extraction['GT_LOADED'] or extraction['post_cutoff_sensor_measurement_used']:
        raise ValueError('INPUT_CAUSAL_CONTRACT')
    sources = Path(__file__).resolve().parent
    map_path = asset / 'map/corridor02.pcd'
    prior = asset / 'initial_pose/corridor02.yaml'
    hashes = {str(path): sha(path) for path in
              (map_path, prior, sources / 'legacy/superloc_first_segment_initializer.cpp')}
    if list(hashes.values()) != [MAP_SHA, PRIOR_SHA, LEGACY_SHA]:
        raise ValueError('FROZEN_INPUT_OR_LEGACY_MISMATCH')
    for binary in ('math_test', 'input_bridge', 'initializer'):
        hashes[str(build / binary)] = sha(build / binary)
    hashes[str(protocol.resolve())] = sha(protocol)
    for path in sorted(sources.rglob('*')):
        if path.is_file() and '__pycache__' not in path.parts:
            hashes[str(path)] = sha(path)
    freeze = dict(created_unix_ns=str(time.time_ns()), map_sha256=MAP_SHA,
                  prior_sha256=PRIOR_SHA, hashes=hashes, extraction=extraction,
                  raw_bag_path=str(asset / 'raw/corridor02.bag'),
                  raw_bag_size_bytes=(asset / 'raw/corridor02.bag').stat().st_size,
                  raw_bag_full_sha256_recomputed=False, GT_loaded=False)
    (output / 'input_freeze.json').write_text(json.dumps(freeze, indent=2) + '\n')
    env = os.environ.copy()
    env['LD_PRELOAD'] = '/lib/x86_64-linux-gnu/libusb-1.0.so.0'
    env['ROS_LOG_DIR'] = str(output / 'ros_logs')
    if run([build / 'math_test', prior], 'math_test', output, env):
        raise ValueError('MATH_OR_ACTIVE_PRIOR_TEST_FAILED')
    if run([build / 'input_bridge', inputs], 'input_bridge', output, env):
        raise ValueError('BOOTSTRAP_INPUT_BRIDGE_FAILED')
    freeze['hashes'][str(inputs / 'init_rotation_input.bag')] = sha(inputs / 'init_rotation_input.bag')
    freeze['hashes'][str(inputs / 'bridge.csv')] = sha(inputs / 'bridge.csv')
    (output / 'input_freeze.json').write_text(json.dumps(freeze, indent=2) + '\n')
    command = [build / 'initializer', inputs / 'init_rotation_input.bag', map_path, prior, output, '5']
    code = run(command, 'initializer', output, env)
    with (output / 'initializer_candidates.csv').open() as stream:
        ranked = list(csv.DictReader(stream))
    with (output / 'candidate_diagnostics.csv').open() as stream:
        diagnostics = {int(r['candidate_id']): r for r in csv.DictReader(stream)}
    if len(ranked) != 24 or len(diagnostics) != 24:
        raise ValueError('ORIGINAL_CANDIDATE_BUDGET_NOT_COMPLETED')
    best = diagnostics[int(ranked[0]['candidate_id'])]
    checks = dict(program_exit_zero=code == 0, converged=best['converged'] == '1',
                  legal_seed=best['seed_SE3_float_tolerance'] == '1',
                  fitness=float(best['fitness']) <= 10,
                  nn_median=float(best['nn_median_m']) <= 1,
                  nn_p90=float(best['nn_p90_m']) <= 3,
                  inlier_0_5m=float(best['inlier_0_5m']) >= .5)
    with (output / 'initializer_registration.csv').open() as stream:
        local = list(csv.DictReader(stream))
    result = dict(program_exit_code=code, legacy_ranked_best=best, checks=checks,
                  initialization_admitted=all(checks.values()),
                  map_icp_calls=len(diagnostics), local_icp_calls=len(local),
                  local_scans_used=1 + sum(int(row['used']) for row in local),
                  method='UNCHANGED_LEGACY_FIRST_SEGMENT_24_PROPER_AXES',
                  candidate_rule='R_seed=R_yaml*A_i; t_seed=t_yaml',
                  legacy_yaml_status='NOT_A_RUNTIME_INITIALIZATION_UNLESS_ADMITTED',
                  GT_loaded=False, independent_validation_executed=False,
                  normalized_map_executed=False, F_executed=False, G_executed=False)
    (output / 'admission.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    for key in ('asset', 'inputs', 'build', 'output', 'protocol'):
        parser.add_argument('--' + key, type=Path, required=True)
    main(**vars(parser.parse_args()))
