#!/usr/bin/env python3
"""Seal actual blocked disposition and verify unchanged production/history."""
import csv
import hashlib
import importlib.util
import json
import re
import subprocess
import sys
from pathlib import Path

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parents[1]
REPO = HERE.parents[1]
START = 'c8a02bf1cf16413569f940ef532f14ae19a38d9a'
CODE = '9e5d41528c12228b613bce79b31b95fcc159c813'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(name, value):
    (HERE / name).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


spec = importlib.util.spec_from_file_location('r23_prepare', REPO / 'docs/p62_r23_real_imu_closure/tools/prepare_inputs.py')
r23 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(r23)
stable = r23.stable_snapshot()
assert stable == json.loads((HERE / 'STABLE_BEFORE.json').read_text()), 'production state changed'
save('STABLE_AFTER.json', stable)
for row in json.loads((HERE / 'INPUT_IDENTITIES.json').read_text()):
    assert sha(row['path']) == row['sha256'], row['path']
for row in json.loads((HERE / 'sources/manifest.json').read_text()):
    assert 'error' not in row
    assert sha(HERE / 'sources' / row['name']) == row['sha256']

original = (HERE.parent / 'p62_r27_submission_package/PAPER_SUBMISSION_DRAFT.tex').read_text()
manuscript = (HERE / 'UPDATED_MANUSCRIPT.tex').read_text()
assert original.split(r'\section{Experimental Protocol}')[0] == manuscript.split(r'\section{Experimental Protocol}')[0]
abstract = manuscript.split(r'\begin{abstract}')[1].split(r'\end{abstract}')[0]
keys = {k for group in re.findall(r'\\cite\{([^}]+)\}', manuscript) for k in group.split(',')}
bbl = (HERE / 'build/UPDATED_MANUSCRIPT.bbl').read_text()
assert not keys - set(re.findall(r'\\bibitem\{([^}]+)\}', bbl))
log = (HERE / 'build/UPDATED_MANUSCRIPT.log').read_text()
assert not re.search(r'(Citation|Reference).*undefined|Overfull|Missing character', log)
rows = list(csv.DictReader((HERE / 'INDEPENDENT_IMU_RESULTS.csv').open()))
assert len(rows) == 12 and all(r['status'] == 'NOT_RUN_G0_BLOCKED' and r['translation_rmse_m'] == '' for r in rows)
assert not (HERE / 'EXTERNAL_BASELINE_RESULTS.csv').exists()
changes = subprocess.check_output(['git', 'diff', START, '--name-only'], cwd=REPO, text=True).splitlines()
assert all(x.startswith('docs/p62_r28_independent_validation/') for x in changes)
info = subprocess.check_output(['pdfinfo', str(HERE / 'UPDATED_MANUSCRIPT.pdf')], text=True)
pages = int(re.search(r'Pages:\s*(\d+)', info).group(1))
save('BUILD_RECEIPT.json', {
    'command': '/home/jian/livox_ws/p62_r27_typesetting_assets/tectonic -k --keep-logs --outdir build UPDATED_MANUSCRIPT.tex',
    'cwd': str(HERE), 'exit_code': 0, 'engine_sha256': sha('/home/jian/livox_ws/p62_r27_typesetting_assets/tectonic'),
    'tex_sha256': sha(HERE / 'UPDATED_MANUSCRIPT.tex'), 'pdf_sha256': sha(HERE / 'UPDATED_MANUSCRIPT.pdf'),
    'pages': pages, 'abstract_whitespace_words': len(abstract.split()), 'resolved_citations': len(keys),
    'warnings': 'Tectonic bbl consistency/6-pass notice; inherited TU/ptm font substitution and underfull boxes; no undefined citation/reference, missing glyph or overfull box. Fonts embedded; rendered pages visually checked.',
    'formal_localization_experiments': 0})
save('results.json', {
    'task': 'P62-R28-INDEPENDENT-VALIDATION-AND-MATCHED-BASELINE', 'status': 'COORDINATE_CONTRACT_BLOCKED',
    'start_sha': START, 'audit_code_sha': CODE, 'delivery_end_sha': 'resolve final local HEAD; see final handoff',
    'branch': subprocess.check_output(['git', 'branch', '--show-current'], cwd=REPO, text=True).strip(),
    'workspace': str(REPO), 'pushed': False, 'new_ndt_calls': 0, 'new_ekf_runs': 0,
    'internal_trajectories_planned': 12, 'internal_trajectories_executed': 0, 'external_trajectories_executed': 0,
    'csv_version_identity_verified': True, 'csv_reference_axes_certified': False,
    'matrix_algebra_pass': True, 'historical_frame_error_proven': False,
    'new_error_metrics': None, 'independent_recovery': 'NOT_EVALUABLE', 'normal_safety': 'NOT_EVALUABLE',
    'matched_external_comparison': 'NOT_ESTABLISHED', 'paper_pages': pages,
    'stable_workspace_unchanged': True, 'historical_and_algorithm_diff_empty': True,
    'next': 'Authoritative binding of exact CSV payload to reference axes before further accuracy experiments',
    'advisor_review_recommended': True, 'formal_submission_recommended': False})
vfs = __import__('os').statvfs(HERE)
save('logs/final_verification.json', {'stable_unchanged': True, 'critical_input_hashes_match': True,
     'archive_source_hashes_match': True, 'old_manuscript_front_matter_and_method_unchanged': True,
     'new_result_matrix_rows': len(rows), 'external_results_file_absent': True,
     'free_bytes': vfs.f_bavail * vfs.f_frsize, 'formal_native_calls': 0,
     'scope': 'new R28 docs only', 'validation_tool_sha256': sha(Path(__file__)),
     'note': 'No repeated full dataset hashing; no future reference pose values decoded'})
print('Delivery checks PASS; scientific G0 remains BLOCKED; no experimental success inferred.')
