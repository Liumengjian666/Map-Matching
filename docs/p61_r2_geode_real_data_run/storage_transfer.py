#!/usr/bin/env python3
"""Read-only external transfer/hash with bounded kernel monitoring; no disk repair."""
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

SOURCE = Path('/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02/raw/Shield_tunnel1_gamma.bag')
DEST = Path('/home/jian/geode/Shield_tunnel1_gamma')
SIZE = 4500527001
RATE = 16 * 1024 * 1024
ERROR = re.compile(r'usb 4-2.*(?:reset|disconnect)|(?:sda|sd 0:0:0:0|host0:).*(?:error|offline|failed|timeout|uas_eh)|'
                   r'(?:I/O error|Buffer I/O error).*sda|(?:exfat|exFAT).*(?:error|fail)|lost async page write', re.I)


def hash_worker():
    digest = hashlib.sha256(); count = 0; start = time.monotonic()
    with SOURCE.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block); count += len(block)
            time.sleep(max(0., count / RATE - (time.monotonic() - start)))
            if count % (64 * 1024 * 1024) == 0:
                print('HASH_PROGRESS', count, flush=True)
    if count != SIZE: raise ValueError('source size changed during independent hash')
    print('SOURCE_SHA256', digest.hexdigest(), flush=True)


def kernel_since(epoch):
    return subprocess.check_output(['journalctl', '-k', '-b', '--since', '@' + str(epoch), '--no-pager'],
                                   text=True, timeout=8)


def evidence_path(phase, attempt=None):
    if phase not in ('copy', 'source_hash'): raise ValueError('unsupported phase')
    if attempt is not None and not re.fullmatch(r'[A-Za-z0-9_-]{1,32}', attempt):
        raise ValueError('invalid explicit attempt label')
    return DEST / 'storage_evidence' / (phase if attempt is None else phase + '_' + attempt)


def monitor(phase, attempt=None):
    if phase not in ('copy', 'source_hash'): raise ValueError('unsupported phase')
    if SOURCE.stat().st_size != SIZE: raise ValueError('unexpected source size')
    mount = subprocess.check_output(['findmnt', '-rn', '-o', 'UUID', '/media/jian/HIKVISION'], text=True, timeout=8).strip()
    if mount != 'F627-4133': raise ValueError('unexpected external UUID')
    if (DEST / 'Shield_tunnel1_gamma.bag').exists(): raise FileExistsError('verified destination must never be overwritten')
    # Explicit new labels allow append-verify resume without erasing old evidence.
    root = evidence_path(phase, attempt); root.mkdir(parents=True, exist_ok=False)
    epoch = int(time.time()); kernel_since(epoch)
    partial = DEST / 'Shield_tunnel1_gamma.bag.part'
    command = (['rsync', '--partial', '--append-verify', '--info=progress2', '--bwlimit=16384', str(SOURCE), str(partial)]
               if phase == 'copy' else [sys.executable, str(Path(__file__).resolve()), '--hash-worker'])
    start = time.monotonic(); reason = None; last_progress = start; previous = -1
    with (root / 'worker.log').open('x') as output, (root / 'monitor.jsonl').open('x') as events:
        process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        while True:
            try:
                logs = kernel_since(epoch)
                failures = [line for line in logs.splitlines() if ERROR.search(line)]
                if failures: reason = 'NEW_STORAGE_ERROR: ' + ' | '.join(failures)
            except (subprocess.SubprocessError, OSError) as error:
                reason = 'MONITOR_UNAVAILABLE: ' + str(error); logs = ''
            progress = partial.stat().st_size if phase == 'copy' and partial.exists() else (root / 'worker.log').stat().st_size
            if progress != previous: previous = progress; last_progress = time.monotonic()
            state = Path(f'/proc/{process.pid}/stat')
            try: stat = state.read_text().rsplit(') ', 1)[1].split()[0]
            except FileNotFoundError: stat = 'EXITED'
            events.write(json.dumps({'elapsed_sec': time.monotonic() - start, 'progress': progress,
                                     'worker_state': stat, 'fault': reason}) + '\n'); events.flush()
            if time.monotonic() - last_progress > 90: reason = reason or 'NO_PROGRESS_90_SECONDS'
            if time.monotonic() - start > 1200: reason = reason or 'PHASE_DEADLINE'
            if reason:
                if process.poll() is None: os.killpg(process.pid, signal.SIGTERM)
                break
            if process.poll() is not None: break
            time.sleep(1.)
        (root / 'kernel.log').write_text(logs)
        try: code = process.wait(timeout=5)
        except subprocess.TimeoutExpired: code = None; reason = reason or 'WORKER_STILL_BLOCKED'
    result = {'phase': phase, 'source': str(SOURCE), 'partial': str(partial), 'expected_bytes': SIZE,
              'rate_limit_bytes_sec': RATE, 'wall_sec': time.monotonic() - start, 'exit_code': code,
              'stop_reason': reason, 'success': reason is None and code == 0, 'command': command}
    with (root / 'receipt.json').open('x') as stream: json.dump(result, stream, indent=2)
    print(json.dumps(result, indent=2))
    if not result['success']: sys.exit(1)


if __name__ == '__main__':
    hash_worker() if sys.argv[1] == '--hash-worker' else monitor(sys.argv[1], sys.argv[2] if len(sys.argv)>2 else None)
