#!/usr/bin/env python3
"""Extract the ten verified official ZIPs without changing their PCD payloads."""
import fcntl
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import tempfile
import time
import zipfile

from download_assets import ROOT, check_mount, validate


def validate_member(member, names):
    if (not re.fullmatch(r'ouster_scan/cloud_\d+_\d{9}\.pcd', member.filename) or
            stat.S_ISLNK(member.external_attr >> 16)):
        raise RuntimeError('Unexpected archive entry: ' + member.filename)
    if member.filename in names:
        raise RuntimeError('Duplicate scan across official archives')
    names.add(member.filename)


def main():
    plan = json.loads(Path(__file__).resolve().with_name('download_plan.json').read_text())
    check_mount()
    if any(p.is_symlink() for p in [ROOT, *ROOT.parents]):
        raise RuntimeError('Root symlink is not allowed')
    os.chdir(str(ROOT))
    device_id = os.stat('.').st_dev
    check_mount(device_id)
    lock = open('.prepare.lock', 'a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    state = json.loads(Path('downloads/download_status.json').read_text())
    if state['status'] != 'DOWNLOADS_COMPLETE_EXTRACTION_NOT_RUN':
        raise RuntimeError('Download verification has not completed')
    receipts = {entry['path']: entry for entry in state['files']}
    if (len(receipts) != len(state['files']) or
            set(receipts) != {entry['path'] for entry in plan['files']}):
        raise RuntimeError('Receipt does not cover the complete explicit download plan')
    for entry in plan['files']:
        path = Path(entry['path'])
        if any(p.is_symlink() for p in [path, *path.parents]) or path.stat().st_dev != device_id:
            raise RuntimeError('Verified asset left the external filesystem')
        receipt = receipts[entry['path']]
        if receipt['drive_id'] != entry['drive_id'] or receipt['bytes'] != entry['bytes']:
            raise RuntimeError('Receipt identity differs from the explicit download plan')
        if validate(path, entry)['sha256'] != receipt['sha256']:
            raise RuntimeError('Verified asset content changed before extraction')
    metadata = []
    names = set()
    total_bytes = 0
    for i in range(1, 11):
        archive = Path('downloads/ouster_scan-%03d.zip' % i)
        if archive.is_symlink():
            raise RuntimeError('Archive symlink is not allowed')
        with zipfile.ZipFile(str(archive)) as z:
            for member in z.infolist():
                validate_member(member, names)
                total_bytes += member.file_size
            metadata.append({'path': str(archive), 'scan_count': len(z.infolist()),
                             'uncompressed_bytes': sum(x.file_size for x in z.infolist())})
    raw = Path('01_short_experiment/raw_format')
    if any(p.is_symlink() for p in [raw, *raw.parents]) or raw.stat().st_dev != device_id:
        raise RuntimeError('Extraction directory is not on the external drive')
    output = raw / 'ouster_scan'
    if output.exists() or output.is_symlink():
        raise RuntimeError('Refuse to overwrite or merge existing extracted scans')
    free = os.statvfs('.').f_bavail * os.statvfs('.').f_frsize
    if free < total_bytes + 10 * 1024 ** 3:
        raise RuntimeError('Insufficient space for actual uncompressed ZIP totals')
    started = time.monotonic()
    for archive in metadata:
        check_mount(device_id)
        # unzip verifies entry CRC while writing; -n additionally forbids overwrite.
        subprocess.run(['unzip', '-nq', archive['path'], '-d', str(raw)], check=True)
        print('EXTRACTED_CRC_PASS', archive['path'], archive['scan_count'], flush=True)
    files = sorted(output.glob('*.pcd'))
    if len(files) != len(names):
        raise RuntimeError('Extracted scan count differs from official ZIP inventory')
    schemas = {}
    point_total = 0
    for path in files:
        header = {}
        with path.open('rb') as f:
            for _ in range(40):
                line = f.readline().decode('ascii').strip()
                if not line or line.startswith('#'):
                    continue
                key, value = line.split(' ', 1)
                header[key] = value
                if key == 'DATA':
                    break
            else:
                raise RuntimeError('PCD header is not bounded')
        if header['DATA'] != 'binary':
            raise RuntimeError('Unexpected PCD encoding')
        schema = json.dumps({k: header[k] for k in ['FIELDS', 'SIZE', 'TYPE', 'COUNT', 'WIDTH', 'HEIGHT', 'DATA']}, sort_keys=True)
        schemas[schema] = schemas.get(schema, 0) + 1
        point_total += int(header['POINTS'])
    stamps = sorted(int(re.search(r'cloud_(\d+)_(\d+)\.pcd', p.name).group(1)) * 1000000000 +
                    int(re.search(r'cloud_(\d+)_(\d+)\.pcd', p.name).group(2)) for p in files)
    result = {'status': 'PCD_EXTRACTION_AND_ZIP_CRC_PASS', 'scan_count': len(files),
              'point_count': point_total, 'first_ns': stamps[0], 'last_ns': stamps[-1],
              'duration_s': (stamps[-1] - stamps[0]) / 1e9,
              'uncompressed_bytes': total_bytes, 'schemas': schemas,
              'archives': metadata, 'elapsed_s': time.monotonic() - started}
    check_mount(device_id)
    report_dir = Path('downloads')
    if report_dir.is_symlink() or report_dir.stat().st_dev != device_id:
        raise RuntimeError('Report directory is not on the external filesystem')
    with tempfile.NamedTemporaryFile(mode='w', dir=str(report_dir),
                                     prefix='pointcloud_validation.', suffix='.tmp',
                                     delete=False) as report:
        json.dump(result, report, indent=2)
        report.write('\n')
    os.replace(report.name, str(report_dir / 'pointcloud_validation.json'))
    print(json.dumps(result, indent=2), flush=True)


if __name__ == '__main__':
    main()
