#!/usr/bin/env python3
"""Mechanical Git text normalization, preserving original machine receipts.

Never changes persistent sensor files or their input manifest. CSV parsed values
must remain identical; machine logs change only line-ending/trailing whitespace.
"""
import csv
import io
import json
from pathlib import Path
import shutil
import subprocess

from run_protocol import OUTPUT, check, digest, save


def main():
    repo = Path(subprocess.check_output(['git', 'rev-parse', '--show-toplevel'], text=True).strip())
    archive = repo / 'docs/p9_r7_cross_dataset_nearoptimal/prospective_scanend_v1'
    backup = OUTPUT / 'archive_receipts_before_formatting'
    check(not backup.exists(), 'format receipt backup already exists')
    shutil.copytree(archive, backup)
    old_hashes = json.loads((backup / 'artifact_hashes.json').read_text())
    changes = []
    for p in sorted(archive.iterdir()):
        if p.suffix not in ('.csv', '.log'):
            continue
        original = p.read_bytes()
        text = original.decode('utf-8')
        formatted = '\n'.join(line.rstrip(' \t\r') for line in text.splitlines()) + '\n'
        if p.suffix == '.csv':
            check(list(csv.reader(io.StringIO(text))) == list(csv.reader(io.StringIO(formatted))),
                  'CSV values changed: ' + p.name)
        old = digest(p)
        p.write_text(formatted)
        changes.append({'file': p.name, 'before_sha256': old, 'after_sha256': digest(p),
                        'values_unchanged': True})
    save(archive / 'formatting_receipt.json', {
        'original_archive_backup': str(backup), 'original_hash_book_sha256': digest(backup / 'artifact_hashes.json'),
        'reason': 'git diff --check rejects CRLF and log trailing spaces', 'changes': changes,
        'persistent_sensor_files_changed': False, 'experiment_reexecuted': False})
    # Replace this turn's unfinished delivery hash book, keeping the prior one in
    # the persistent backup. No earlier experiment freeze is touched.
    paths = [p for p in archive.iterdir() if p.is_file() and p.name != 'artifact_hashes.json']
    paths += [p for p in Path(__file__).resolve().parent.iterdir() if p.is_file()]
    with open(archive / 'artifact_hashes.json', 'w') as f:
        json.dump({str(p.relative_to(repo)): digest(p) for p in sorted(paths)}, f, indent=2)
        f.write('\n')
    check(bool(old_hashes), 'original freeze missing')
    print('TEXT_NORMALIZATION_PASS; persistent original receipts retained')


if __name__ == '__main__':
    main()
