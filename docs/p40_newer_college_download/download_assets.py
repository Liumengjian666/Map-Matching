#!/usr/bin/env python3
"""Sequential, resumable acquisition of explicitly inventoried public assets.

Only download into the mounted external drive. No registration, credentials,
extraction, localization or algorithm changes are performed by this script.
"""
import hashlib
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from urllib.parse import urlencode, urlparse

import requests
from requests.adapters import HTTPAdapter
from bs4 import BeautifulSoup
from urllib3.util.retry import Retry


MOUNT = Path('/media/jian/HIKVISION')
ROOT = MOUNT / 'paper rosbag/NewerCollege'


def check_mount(device_id=None):
    actual = subprocess.check_output(
        ['findmnt', '-n', '-o', 'UUID', '-M', str(MOUNT)], text=True).strip()
    if actual != 'F627-4133' or not os.path.ismount(str(MOUNT)):
        raise RuntimeError('Expected HIKVISION filesystem is not mounted')
    if device_id is not None and (os.stat('.').st_dev != device_id or
                                  MOUNT.stat().st_dev != device_id):
        raise RuntimeError('Pinned working directory is on another filesystem')


def digest(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(4 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def content_url(file_id):
    url = 'https://drive.usercontent.google.com/uc?' + urlencode(
        {'id': file_id, 'export': 'download'})
    session = requests.Session()
    session.mount('https://', HTTPAdapter(max_retries=Retry(
        total=2, backoff_factor=2, status_forcelist=[429, 500, 502, 503, 504])))
    with session, session.get(url, stream=True, timeout=(15, 45)) as response:
        response.raise_for_status()
        if 'text/html' not in response.headers.get('Content-Type', ''):
            return url
        soup = BeautifulSoup(response.text, 'html.parser')
    form = soup.find('form', id='download-form')
    if not form or form.get('method', '').lower() != 'get':
        raise RuntimeError('Access requires login, quota recovery, or unknown form; stopped')
    action = form.get('action', '')
    parsed = urlparse(action)
    if parsed.scheme != 'https' or parsed.netloc != 'drive.usercontent.google.com':
        raise RuntimeError('Unexpected download confirmation destination')
    values = {n['name']: n.get('value', '') for n in form.find_all('input') if n.get('name')}
    if values.get('id') != file_id or values.get('confirm') != 't':
        raise RuntimeError('Unexpected confirmation fields')
    return action + '?' + urlencode(values)


def validate(path, entry, previous=None):
    size = path.stat().st_size
    if size != entry['bytes']:
        raise RuntimeError('Size mismatch: %s: %s != %s' % (path, size, entry['bytes']))
    with open(path, 'rb') as f:
        prefix = f.read(256)
    if b'<html' in prefix.lower() or b'<!doctype html' in prefix.lower():
        raise RuntimeError('HTML is not a dataset payload: ' + str(path))
    if path.name.startswith('ouster_scan-') and not prefix.startswith(b'PK\x03\x04'):
        raise RuntimeError('Expected ZIP signature')
    if '.ply' in path.name and not prefix.startswith(b'ply\n'):
        raise RuntimeError('Expected PLY signature')
    result = {'bytes': size, 'sha256': digest(path), 'publisher_sha256': None,
              'validation': 'OFFICIAL_LISTING_SIZE_AND_LOCAL_SHA256'}
    if previous is not None and (previous['drive_id'] != entry['drive_id'] or
                                 previous['bytes'] != size or
                                 previous['sha256'] != result['sha256']):
        raise RuntimeError('Recorded file identity or content changed: ' + str(path))
    return result


def main(plan_path):
    plan = json.loads(Path(plan_path).read_text())
    check_mount()
    if any(p.is_symlink() for p in [ROOT, *ROOT.parents]):
        raise RuntimeError('External destination ancestors must not be symlinks')
    ROOT.mkdir(parents=True, exist_ok=True)
    os.chdir(str(ROOT))  # Pin cwd to external filesystem even if its mount disappears.
    device_id = os.stat('.').st_dev
    if device_id != MOUNT.stat().st_dev:
        raise RuntimeError('Download root is not on the external filesystem')
    lock = open('.download.lock', 'a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    reserve = 10 * 1024 ** 3
    remaining = sum(max(0, e['bytes'] - (Path(e['path'] + '.part').stat().st_size
                    if Path(e['path'] + '.part').is_file() else 0))
                    for e in plan['files'] if not Path(e['path']).exists())
    free = os.statvfs('.').f_bavail * os.statvfs('.').f_frsize
    if free < remaining + reserve:
        raise RuntimeError('Insufficient space for the explicit download plan')
    state_path = Path('downloads/download_status.json')
    state_path.parent.mkdir(parents=True, exist_ok=True)
    if (state_path.is_symlink() or any(p.is_symlink() for p in state_path.parents) or
            state_path.parent.stat().st_dev != device_id):
        raise RuntimeError('Existing receipt path is not a safe external destination')
    receipts = {}
    if state_path.exists():
        previous_state = json.loads(state_path.read_text())
        if previous_state['root'] != str(ROOT):
            raise RuntimeError('Existing receipt belongs to a different download root')
        for receipt in previous_state['files']:
            if receipt['path'] in receipts:
                raise RuntimeError('Duplicate entry in existing download receipt')
            receipts[receipt['path']] = receipt
    state = {'status': 'RUNNING', 'started_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
             'root': str(ROOT), 'files': list(receipts.values())}

    def save():
        check_mount(device_id)
        if any(p.is_symlink() for p in state_path.parents):
            raise RuntimeError('Status directory symlink is not allowed')
        if state_path.parent.stat().st_dev != device_id:
            raise RuntimeError('Status directory crossed a filesystem boundary')
        with tempfile.NamedTemporaryFile(mode='w', dir=str(state_path.parent),
                                         prefix='download_status.', suffix='.tmp',
                                         delete=False) as temp:
            json.dump(state, temp, indent=2)
            temp.write('\n')
        os.replace(temp.name, str(state_path))

    try:
        for entry in plan['files']:
            check_mount(device_id)
            dest = Path(entry['path'])
            if dest.is_absolute() or '..' in dest.parts:
                raise RuntimeError('Unsafe destination')
            dest.parent.mkdir(parents=True, exist_ok=True)
            if any(p.is_symlink() for p in [dest, *dest.parents]):
                raise RuntimeError('Destination symlink is not allowed')
            if dest.parent.stat().st_dev != device_id:
                raise RuntimeError('Destination crossed a filesystem boundary')
            started = time.monotonic()
            print('START', entry['path'], entry['bytes'], flush=True)
            if not dest.exists():
                part = dest.with_name(dest.name + '.part')
                if part.is_symlink():
                    raise RuntimeError('Partial file symlink is not allowed')
                if not part.exists() or part.stat().st_size != entry['bytes']:
                    url = content_url(entry['drive_id'])
                    subprocess.run([
                        'curl', '--fail', '--location', '--continue-at', '-',
                        '--connect-timeout', '20', '--speed-limit', '1024',
                        '--speed-time', '180', '--retry', '2', '--retry-delay', '10',
                        '--output', str(part), url], check=True)
                check_mount(device_id)
                result = validate(part, entry, receipts.get(entry['path']))
                if dest.exists():
                    raise RuntimeError('Refuse to overwrite an existing final file')
                os.rename(str(part), str(dest))
            else:
                result = validate(dest, entry, receipts.get(entry['path']))
            receipts[entry['path']] = dict(entry, **result, elapsed_s=time.monotonic() - started)
            state['files'] = list(receipts.values())
            save()
            print('VERIFIED', str(dest), result['sha256'], flush=True)
        state['status'] = 'DOWNLOADS_COMPLETE_EXTRACTION_NOT_RUN'
    except Exception as error:
        state['status'] = 'STOPPED_ERROR'
        state['error'] = str(error)
        raise
    finally:
        state['updated_utc'] = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
        try:
            save()
        finally:
            lock.close()


if __name__ == '__main__':
    main(sys.argv[1])
