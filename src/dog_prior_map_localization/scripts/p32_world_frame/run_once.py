#!/usr/bin/env python3
"""One pre-registered four-Q experiment on actual R3 artifacts, with no GT access."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess

REPO = Path(__file__).resolve().parents[4]
EXPECTED = {
    'local': '8c2302f575f12b0decfdcecaf369eaa0a8429b6bdb1673006ec519d7d248e0cd',
    'map': '6c39e243556eddbef7a8b4bd2de25cf7c962c4ff0f52c94cfc1d994f178ae228',
    'prior': 'fc319ad99737a717456e11eb70383d4848eadf7e46587fee0643d6732cf1c3a1',
    'legacy': 'deb58aeb3bd76f802568469f50d33f91b8026924b3fe299fa93b8fa8f9dc266d'}


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def main(asset, binary, output):
    asset, binary, output = (path.resolve() for path in (asset, binary, output))
    output.mkdir(parents=True, exist_ok=False)  # guard, not a reusable cache
    code = Path(__file__).resolve().parent
    files = dict(local=REPO / '.p32_r3_corridor02_init_transfer/initializer_once/first_segment_local_submap.pcd',
                 map=asset / 'map/corridor02.pcd', prior=asset / 'initial_pose/corridor02.yaml',
                 legacy=code.parent / 'p32_init_transfer/legacy/superloc_first_segment_initializer.cpp')
    hashes = {role: dict(path=str(path), SHA256=sha(path)) for role, path in files.items()}
    if any(hashes[role]['SHA256'] != expected for role, expected in EXPECTED.items()):
        raise ValueError('FROZEN_INPUT_CHANGED')
    protocol = REPO / 'docs/p32_r4_corridor02_world_frame/PROTOCOL.md'
    freeze = dict(inputs=hashes, source_SHA256={str(path): sha(path) for path in
                  (code / 'world_test.cpp', code / 'run_once.py', code / 'CMakeLists.txt', files['legacy'])},
                  binary_SHA256=sha(binary), protocol_SHA256=sha(protocol),
                  reference_ns='1645999726479818106', GT_LOADED=False,
                  local_reconstructed=False, raw_bag_replayed=False,
                  Q_yaw_deg=[0, 90, -90, 180], rule='T_M_L=[Q,0]*T_yaml',
                  map_voxel_m=.20, source_voxel_already_applied_m=.25,
                  ICP=dict(max_distance_m=8, max_iterations=60, epsilon=1e-7, fitness_epsilon=1e-6),
                  budgets=dict(NN_screens=4, map_ICP=2))
    (output / 'input_freeze.json').write_text(json.dumps(freeze, indent=2) + '\n')
    env = os.environ.copy()
    env['LD_PRELOAD'] = '/lib/x86_64-linux-gnu/libusb-1.0.so.0'
    with (output / 'math_test.log').open('x') as log:
        code_math = subprocess.call([str(binary), '--math-test'], stdout=log, stderr=subprocess.STDOUT, env=env)
    if code_math:
        raise ValueError('WORLD_TRANSFORM_MATH_FAILED')
    command = [str(binary), str(files['local']), str(files['map']), str(files['prior']), str(output)]
    timed = ['/usr/bin/time', '-f', 'user_cpu_sec=%U\nsystem_cpu_sec=%S\npeak_rss_kib=%M\nwall_sec=%e',
             '-o', str(output / 'resource.txt')] + command
    with (output / 'world_test.log').open('x') as log:
        returncode = subprocess.call(timed, stdout=log, stderr=subprocess.STDOUT, env=env)
    receipt = dict(command=command, returncode=returncode, math_test_returncode=code_math,
                   environment_overrides=dict(LD_PRELOAD=env['LD_PRELOAD']), GT_LOADED=False)
    (output / 'execution.json').write_text(json.dumps(receipt, indent=2) + '\n')
    if returncode:
        raise ValueError('WORLD_TEST_PROGRAM_FAILED')
    with (output / 'world_candidates.csv').open() as stream:
        screens = list(csv.DictReader(stream))
    with (output / 'map_icp.csv').open() as stream:
        registered = list(csv.DictReader(stream))
    assert len(screens) == 4 and len(registered) <= 2
    eligible = sorted((float(row['nn_median_m']), int(row['candidate_id'])) for row in screens
                      if row['screen_pass'] == '1')[:2]
    assert [int(row['candidate_id']) for row in registered] == [row[1] for row in eligible]
    admitted = [row for row in registered if row['admitted'] == '1']
    result = dict(GT_LOADED=False, NN_screens=4, map_ICP_calls=len(registered),
                  admitted_candidates=admitted, holdout_executed=False,
                  normalized_map_executed=False, F_executed=False, G_executed=False)
    if not eligible:
        result['stage_result'] = 'WORLD_FRAME_HYPOTHESIS_NOT_SUPPORTED'
    elif not admitted:
        result['stage_result'] = 'WORLD_FRAME_ICP_GEOMETRIC_ADMISSION_FAILED'
    else:
        result['stage_result'] = 'INITIAL_CANDIDATE_REQUIRES_INDEPENDENT_HOLDOUT'
    (output / 'admission.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    for name in ('asset', 'binary', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    main(**vars(parser.parse_args()))
