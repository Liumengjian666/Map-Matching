#!/usr/bin/env python3
"""Seal existing blocked evidence locally. Never accesses sensor payload or network."""
import csv
import hashlib
import json
from pathlib import Path
import runpy
import shutil
import subprocess
import sys

DOC = Path(__file__).resolve().parent

def write(name, value):
    with (DOC / name).open('x') as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')

def main():
    # Never revise a prior completed result, including failed gates.
    if any((DOC / name).exists() for name in ['results.json', 'FINAL_PROTECTION_RECEIPT.json', 'checksums.sha256']):
        raise RuntimeError('Finalization already attempted/completed; do not overwrite')
    preflight = runpy.run_path(str(DOC / 'preflight.py'), run_name='offline_receipt_reader')
    receipt = json.loads((DOC / 'PREFLIGHT_RECEIPT.json').read_text())
    before = json.loads((DOC / 'ASSET_IDENTITY_BEFORE.json').read_text())
    after = json.loads((DOC / 'ASSET_IDENTITY_AFTER.json').read_text())
    if before != after or receipt['result'] != 'BLOCKED_ASSET_ACQUISITION':
        raise RuntimeError('Unexpected original gate result')

    # Bounded second filename inventory only; does not parse bags or renamed files.
    command = ['rg', '--files', '/home/jian/livox_ws', '/home/jian/rosbag', '/home/jian/geode',
               '-g', '*[Mm]3[Dd][Gg][Rr]*', '-g', '*[Ww]heel*[Ff]loat*', '-g', '*[Ww]heel-float*',
               '-g', '!build/**', '-g', '!devel/**', '-g', '!**/.git/**']
    inventory = subprocess.run(command, capture_output=True, text=True, timeout=30)
    write('LOCAL_INVENTORY.json', dict(command=command, returncode=inventory.returncode,
          matches=inventory.stdout.splitlines(), stderr=inventory.stderr,
          scope='Filename search only, including this R14 directory after creation; not proof against renamed files',
          first_inventory_before_worktree='Same bounded filename search found no matching filenames (rg exit1)',
          payload_files_locally_verified=False))
    if inventory.returncode not in [0, 1] or inventory.stderr:
        raise RuntimeError('Local inventory incomplete; preserve receipt and stop')

    tests = subprocess.run([sys.executable, str(DOC / 'tests/test_storage.py')], capture_output=True, text=True)
    with (DOC / 'TEST_OUTPUT.log').open('x') as stream:
        stream.write(tests.stdout + tests.stderr)
    if tests.returncode:
        raise RuntimeError('Storage tests failed')

    write('MAP_MANIFEST.json', dict(status='NOT_BUILT', reason='Asset admission blocked', map_path=None,
          map_sha256=None, map_points=None, map_build_time_range_ns=None, gt_build_time_range_ns=None,
          map_frame='W (planned only)', map_type='Oracle-Built Prior Map (planned only)', builds_executed=0))
    write('TEST_FRAME_PRE_REGISTRATION.json', dict(status='NOT_SELECTED_AFTER_ASSET_BLOCK',
          planned_rule='20 time-equally-spaced LiDAR frames in whole-sequence 65%-95%; only after all input gates',
          selected_frames=[], selected_frame_count=None, valid_reference_count=None,
          reason='No bag/GT loaded; no timestamps/source hashes available; not a completed test registration'))

    for name in ['NDT_COMPARISON.csv', 'ACCEPTANCE_AND_EVENTS.csv', 'GT_ERROR_COMPARISON.csv', 'RUNTIME_AND_MEMORY.csv']:
        with (DOC / name).open() as stream:
            rows = list(csv.reader(stream))
        if len(rows) != 1 or not rows[0]:
            raise RuntimeError('NOT_RUN outputs must have a header only: ' + name)

    final_snapshot = preflight['snapshot']()
    unchanged = final_snapshot == before
    free = shutil.disk_usage(DOC).free
    write('FINAL_PROTECTION_RECEIPT.json', dict(protected_assets_equal=unchanged,
          final_snapshot=final_snapshot, free_bytes=free,
          minimum_reserve_bytes=preflight['RESERVE'], reserve_preserved=free >= preflight['RESERVE'],
          note='Read-only current protection snapshot, not a historical preservation/backup rerun'))
    if not unchanged or free < preflight['RESERVE']:
        raise RuntimeError('Final production/baseline/storage protection failed; no passing results')

    write('results.json', dict(task='P62-R14-M3DGR-MOCAP-HELDOUT-LOCALIZATION-PILOT',
          result=receipt['result'], storage_status=receipt['storage']['admission'],
          start_sha=preflight['BASE'], preflight_code_sha=receipt['CODE_SHA'],
          official_revision=preflight['REV'], dataset='M3DGR', sequence='Wheel-float01',
          actual_sensor_asset_identity_verified=False, downloaded_sensor_payload_bytes=0,
          coordinate_status='NOT_EVALUATED_AFTER_ASSET_BLOCK', time_status='NOT_EVALUATED_AFTER_ASSET_BLOCK',
          reference_status='NOT_EVALUATED_AFTER_ASSET_BLOCK', prediction_status='NOT_EVALUATED_AFTER_ASSET_BLOCK',
          map_builds=0, new_ndt_align=0, GT_pose_files_read=0, ekf_runs=0, rosbag_play=0,
          ros_localization_nodes=0, new_trajectories=0, production_changes=0,
          ndt_binary_path=None, ndt_binary_sha256=None, test_frame_count=None,
          natural_events=None, admitted_alternatives=None, accepted_recoveries=None,
          translation_error_statistics=None, rotation_error_statistics=None,
          algorithm_runtime_ms=None, algorithm_peak_rss_bytes=None,
          S1='NOT_EVALUABLE', S2='NOT_EVALUABLE', S3='NOT_EVALUABLE', S4='NOT_EVALUABLE', S5='NOT_EVALUABLE', S6='NOT_EVALUABLE',
          storage_tests_passed=6, protected_assets_unchanged=unchanged, push_executed=False,
          novelty='NOT_ESTABLISHED',
          unique_next='Controller to resolve the same Wheel-float01 asset acquisition/storage admission and decide whether to authorize resumption; no automatic retry/dataset switch/R15'))

    files = sorted(path for path in DOC.rglob('*') if path.is_file() and '__pycache__' not in path.parts)
    with (DOC / 'checksums.sha256').open('x') as stream:
        for path in files:
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            stream.write(f'{digest}  {path.relative_to(DOC)}\n')
    print(json.dumps(dict(result=receipt['result'], files_sealed=len(files), protected_assets_equal=unchanged,
          storage_tests_passed=6, payload_downloaded_bytes=0, NDT_align=0), indent=2))

if __name__ == '__main__':
    main()
