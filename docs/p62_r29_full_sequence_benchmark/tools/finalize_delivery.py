#!/usr/bin/env python3
"""Package and verify existing R29 evidence; never starts localization."""
import csv
import gzip
import hashlib
import importlib.util
import json
import shutil
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

sys.dont_write_bytecode = True
P = Path(__file__).resolve().parents[1]
REPO = P.parents[1]
DATA = Path('/home/jian/livox_ws/p62_r29_full_sequence_data')
CODE = 'f66a55457ba3e63d55ad8d588fb2cf5b1ead169b'

def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda: f.read(1048576), b''):
            h.update(b)
    return h.hexdigest()

def rows(path):
    with path.open(newline='') as f:
        return list(csv.DictReader(f))

def save(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n')

manifest = []
checks = {'online_code_sha': CODE, 'runs': {}}
freeze = json.loads((P/'BUILD_FREEZE.json').read_text())
for mode in 'ABCD':
    run = DATA/f'EASY_C0_{mode}_full_03'
    dest = P/'runs'/run.name
    dest.mkdir(parents=True, exist_ok=True)
    seal = json.loads((run/'ONLINE_OUTPUT_SEAL.json').read_text())
    for item in seal['files']:
        file = Path(item['path'])
        assert file.stat().st_size == item['bytes'] and sha(file) == item['sha256'], str(file)
        manifest.append(item)
    command = json.loads((run/'RUN_COMMAND.json').read_text())
    assert command['git_head_at_start'] == CODE
    for label, binary in [('ndt','dog_prior_map_ndt_node_cpp'),('ekf','dog_prior_map_ekf_node_cpp')]:
        expected = freeze['binaries']['devel/lib/dog_prior_map_localization/'+binary]['sha256']
        assert command['binaries_sha256'][label] == expected
    receipt = json.loads((run/'RUN_RECEIPT.json').read_text())
    assert receipt['input_completed']
    trace = rows(P/'FULL_SEQUENCE_TRAJECTORIES'/f'EASY_C0_{mode}.csv')
    assert len(trace) == 2023
    assert [int(r['raw_ordinal']) for r in trace] == list(range(5,2028))
    assert sum(r['reference_valid'] == 'True' for r in trace) == 2022
    unknown = [r for r in trace if r['reference_valid'] != 'True']
    assert len(unknown) == 1 and int(unknown[0]['raw_ordinal']) == 1336
    assert unknown[0]['translation_error_m'] == '' and unknown[0]['reference_high_error'] == ''
    admission = rows(run/'r23_admission.csv')
    schedule = rows(run/'r29_schedule.csv')
    indices = {int(r['scan_ns']):int(r['frame_one_based']) for r in schedule}
    blocks = defaultdict(int)
    for r in admission:
        extra = int(r['extra_aligns'])
        assert 0 <= extra <= 2
        blocks[(indices[int(r['scan_ns'])]-1)//60] += extra
    assert max(blocks.values(), default=0) <= 12
    candidates = rows(run/'r23_candidates.csv')
    assert len(candidates) == sum(r['native_calls_total'] for r in json.loads((P/'RESULT_EASY_C0.json').read_text()) if r['mode']==mode)
    checks['runs'][mode] = {'online_seal_matches':True, 'input_completed':True,
        'frames':len(trace), 'valid_reference':2022, 'missing_reference_raw':1336,
        'max_extra_in_60frame_block':max(blocks.values(),default=0),
        'extra_calls':sum(blocks.values()), 'native_calls':len(candidates),
        'binary_identity_matches':True}
    for name in ['RUN_COMMAND.json','RUN_RECEIPT.json','ONLINE_OUTPUT_SEAL.json','COMPLETED_INPUT_SHUTDOWN.json']:
        shutil.copyfile(run/name, dest/name)
    # Preserve exact raw CSV/log bytes in portable compressed form; original high-rate data remains external.
    for name in ['r23_ndt_lineage.csv','r23_admission.csv','r23_candidates.csv','r29_schedule.csv',
                 'publisher_receipts.csv','oosm_statistics.csv','local_map_diagnostics.csv',
                 'geometry_diagnostics.csv','ndt_runtime.csv','ROSLAUNCH.log']:
        source = run/name
        if not source.is_file():
            continue
        with source.open('rb') as fi, (dest/(name+'.gz')).open('wb') as fo:
            with gzip.GzipFile(fileobj=fo, mode='wb', mtime=0, filename='') as compressed:
                shutil.copyfileobj(fi, compressed)
        with gzip.open(dest/(name+'.gz'),'rb') as f:
            assert hashlib.sha256(f.read()).hexdigest() == sha(source)

b = rows(P/'FULL_SEQUENCE_TRAJECTORIES/EASY_C0_B.csv')
c = rows(P/'FULL_SEQUENCE_TRAJECTORIES/EASY_C0_C.csv')
fields = json.loads((P/'logs/B_C_TRAJECTORY_EQUALITY.json').read_text())['fields']
assert all(all(x[k] == y[k] for k in fields) for x,y in zip(b,c))
checks['B_C_all_pose_time_admission_fusion_fields_equal'] = True
checks['formal_calls'] = sum(x['native_calls'] for x in checks['runs'].values())
checks['preparatory_calls'] = sum(x['native_calls'] for x in json.loads((P/'logs/ADAPTER_RUN_ACCOUNTING.json').read_text()))
assert checks['formal_calls'] == 3625 and checks['preparatory_calls'] == 28
checks['all_new_native_calls'] = 3653
assert not subprocess.check_output(['git','diff',CODE,'--','src'],cwd=REPO)
checks['online_source_unchanged_since_CODE'] = True
for file, digest in freeze['model_sources_unchanged'].items():
    assert sha(REPO/file) == digest
checks['EKF_IMU_Deskew_observation_map_model_sources_unchanged'] = True
for file, identity in freeze['binaries'].items():
    assert sha(REPO/file) == identity['sha256']
checks['final_binaries_match_frozen_build'] = True

spec = importlib.util.spec_from_file_location('r23_snapshot', REPO/'docs/p62_r23_real_imu_closure/tools/prepare_inputs.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
after = module.stable_snapshot()
save(P/'STABLE_AFTER.json',after)
assert after == json.loads((P/'STABLE_BEFORE.json').read_text())
checks['stable_HEAD_branch_index_status_protected_files_unchanged'] = True
changed = subprocess.check_output(['git','diff','--name-only','fa08c12ac965206e576643dae578b63d49719d6f'],cwd=REPO,text=True).splitlines()
allowed = {'src/dog_prior_map_localization/launch/p62_r29_full_sequence.launch',
 'src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp',
 'src/dog_prior_map_localization/src/newer_college_first_scan_publisher.cpp'}
assert all(f.startswith('docs/p62_r29_full_sequence_benchmark/') or f in allowed for f in changed)
checks['tracked_change_scope_R29_only'] = True
usage = shutil.disk_usage('/home/jian/rosbag/public_datasets/koide_p62_r29')
assert usage.free >= 20*1024**3
checks['storage'] = {'free_bytes':usage.free,'total_bytes':usage.total,'minimum_reserve_bytes':20*1024**3}
save(P/'RAW_ARTIFACT_MANIFEST.json', {'original_online_files':manifest,
 'high_rate_data_preserved_external':True,'portable_raw_subset':'runs/*/*.gz; gzip decompresses byte-exact',
 'large_asset_identity':'sources/EXTRACTED_BAG_IDENTITIES.json and logs/download_receipts.json',
 'reference_evaluation_follows_online_seal':True})
save(P/'FINAL_VALIDATION.json',checks)
print(json.dumps(checks,indent=2))
