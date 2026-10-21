#!/usr/bin/env python3
"""Delete only an explicit, SHA-verified duplicate plan. No directory deletion."""
import argparse
import csv
import fcntl
import json
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import time
from audit import OUT, ROOT, PROTECTED, sha, signature, hash_record, write

def mountpoints():
    # ismount alone cannot identify same-device bind mounts.
    return {Path(re.sub(r'\\([0-7]{3})',lambda m:chr(int(m[1],8)),line.split()[4]))
            for line in Path('/proc/self/mountinfo').read_text().splitlines()}

def confined_file(path, root=ROOT):
    p=Path(path)
    if not p.is_absolute() or p==root or root not in p.parents or p.resolve()!=p:
        raise RuntimeError('Path not explicit/inside root, or has symlink: '+str(p))
    ancestry=[p]+list(p.parents)[:len(p.parts)-len(root.parts)]
    mounts=mountpoints()
    if any(a in mounts or os.path.ismount(a) for a in ancestry) or not stat.S_ISREG(p.lstat().st_mode) or p.stat().st_dev!=root.stat().st_dev:
        raise RuntimeError('Not same-filesystem regular non-mount file: '+str(p))
    return p

def validate_pair(target, keeper, expected, records, root=ROOT, protected=PROTECTED):
    if str(target) in protected:raise RuntimeError('Protected target')
    target=confined_file(target,root);keeper=confined_file(keeper,root)
    if target==keeper or target.stat().st_ino==keeper.stat().st_ino:
        raise RuntimeError('Same path/inode, not an independently retained copy')
    for p in [target,keeper]:
        record=records[str(p)]
        if record['sha256']!=expected or signature(p)!=record['signature']:
            raise RuntimeError('Frozen hash/stat identity changed: '+str(p))
    if target.stat().st_size!=keeper.stat().st_size:raise RuntimeError('Size mismatch')
    return target,keeper

def unused(paths):
    records=[]
    for argv in [['fuser','-I']+[str(p) for p in paths],
                 ['lsof','-nP','-e','/tmp/fuse','-Fp','--']+[str(p) for p in paths]]:
        r=subprocess.run(argv,capture_output=True,text=True,timeout=30)
        records.append(dict(argv=argv,returncode=r.returncode,stdout=r.stdout,stderr=r.stderr))
        if r.returncode not in [0,1]:raise RuntimeError('Open-file check failed: '+str(records[-1]))
        if argv[0]=='fuser':
            pids=set(int(p) for p in r.stdout.split())
            noise=r.stderr
            for p in paths:noise=noise.replace(str(p)+':','')
            if noise.strip():raise RuntimeError('fuser scan warning: '+r.stderr)
            # fuser labels open-file names on stderr only when it found an owner.
            if r.returncode==1 and (r.stdout or r.stderr):raise RuntimeError('fuser uncertainty')
            if r.returncode==0 and not pids:raise RuntimeError('Unparseable fuser result')
        else:
            if r.stderr:raise RuntimeError('lsof coverage uncertainty: '+r.stderr)
            pids=set(int(p) for p in re.findall(r'^p(\d+)$',r.stdout,re.M))
            if r.returncode==0 and not pids:raise RuntimeError('Unparseable lsof result')
        if pids-{os.getpid()}:raise RuntimeError('File used by another process: '+str(records[-1]))
    return records

def delete_one(entry, records, root=ROOT, protected=PROTECTED, journal=None):
    if entry['classification']!='SAFE_TO_DELETE' or entry['basis']!='A_EXACT_SHA_DUPLICATE':
        raise RuntimeError('Deletion lacks approved exact duplicate evidence')
    target,keeper=validate_pair(entry['path'],entry['retained_copy'],entry['sha256'],records,root,protected)
    # This process holds read-only verification descriptors until after unlink.
    descriptors=[]
    try:
        for p in [target,keeper]:
            fd=os.open(str(p),os.O_RDONLY|os.O_NOFOLLOW)
            descriptors.append(fd)
            fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
            s=os.fstat(fd)
            if (s.st_dev,s.st_ino,s.st_size,s.st_mtime_ns,s.st_ctime_ns)!=(
                records[str(p)]['signature']['dev'],records[str(p)]['signature']['inode'],
                records[str(p)]['signature']['size'],records[str(p)]['signature']['mtime_ns'],records[str(p)]['signature']['ctime_ns']):
                raise RuntimeError('Opened inode changed')
        checks=unused([target,keeper])
        # Rehash the exact opened target version, not merely its earlier stat.
        fresh=hash_record(target)
        if fresh['sha256']!=entry['sha256'] or fresh['signature']!=records[str(target)]['signature']:
            raise RuntimeError('Target digest/stat changed immediately before deletion')
        validate_pair(target,keeper,entry['sha256'],records,root,protected)
        checks+=unused([target,keeper])
        before=shutil.disk_usage(root).free
        directory=os.open(str(target.parent),os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW)
        try:
            if os.readlink('/proc/self/fd/'+str(directory))!=str(target.parent):
                raise RuntimeError('Parent directory was renamed')
            confined_file(target,root);confined_file(keeper,root)
            s=os.stat(target.name,dir_fd=directory,follow_symlinks=False)
            expected=records[str(target)]['signature']
            if (s.st_ino,s.st_dev,s.st_size,s.st_mtime_ns,s.st_ctime_ns)!=(expected['inode'],expected['dev'],expected['size'],expected['mtime_ns'],expected['ctime_ns']):
                raise RuntimeError('Final directory entry changed')
            validate_pair(target,keeper,entry['sha256'],records,root,protected)
            if journal is None:raise RuntimeError('Durable journal callback required')
            journal(dict(phase='INTENT',path=str(target),retained_copy=str(keeper),sha256=entry['sha256'],at_ns=time.time_ns()))
            os.unlink(target.name,dir_fd=directory)
            journal(dict(phase='UNLINKED',path=str(target),at_ns=time.time_ns()))
            os.fsync(directory)
        finally:os.close(directory)
        if target.exists() or signature(keeper)!=records[str(keeper)]['signature']:
            raise RuntimeError('Unexpected post-unlink state')
        return dict(path=str(target),retained_copy=str(keeper),sha256=entry['sha256'],
             logical_bytes=records[str(target)]['signature']['size'],
             allocated_bytes_before=records[str(target)]['signature']['allocated'],
             deleted_at_ns=time.time_ns(),free_bytes_before=before,
             open_file_checks=checks,status='DELETED_VERIFIED_REDUNDANT_COPY')
    finally:
        for fd in reversed(descriptors):os.close(fd)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--apply',action='store_true');args=parser.parse_args()
    if not args.apply:raise RuntimeError('Explicit --apply required')
    if (OUT/'DELETE_STARTED.json').exists():raise RuntimeError('Do not repeat deletion attempt')
    plan=json.loads((OUT/'APPROVED_DUPLICATES.json').read_text())
    records={r['path']:r for r in json.loads((OUT/'HASHES_BEFORE.json').read_text())}
    paths={r['path'] for r in plan}
    if len(paths)!=len(plan) or paths & {r['retained_copy'] for r in plan}:
        raise RuntimeError('Duplicate deletion entries or keeper scheduled for deletion')
    # Complete all non-destructive checks before deleting anything.
    for entry in plan:
        target,keeper=validate_pair(entry['path'],entry['retained_copy'],entry['sha256'],records)
        unused([target,keeper])
    for keeper in sorted(set(r['retained_copy'] for r in plan)):
        fresh=hash_record(keeper)
        if fresh['sha256']!=records[keeper]['sha256'] or fresh['signature']!=records[keeper]['signature']:
            raise RuntimeError('Retained copy SHA/stat changed')
    write('DELETE_STARTED.json',dict(at_ns=time.time_ns(),plan_sha256=sha(OUT/'APPROVED_DUPLICATES.json'),
          paths=sorted(paths),free_bytes=shutil.disk_usage(ROOT).free))
    with (OUT/'DELETE_STARTED.json').open('rb') as marker:os.fsync(marker.fileno())
    fields=['path','retained_copy','sha256','logical_bytes','allocated_bytes_before','deleted_at_ns','status']
    with (OUT/'DELETED_FILES.csv').open('x',newline='') as f, (OUT/'DELETE_JOURNAL.jsonl').open('x') as log:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();f.flush();os.fsync(f.fileno())
        log.flush();os.fsync(log.fileno())
        directory=os.open(str(OUT),os.O_RDONLY|os.O_DIRECTORY)
        try:os.fsync(directory)
        finally:os.close(directory)
        def journal(record):
            log.write(json.dumps(record,ensure_ascii=False)+'\n');log.flush();os.fsync(log.fileno())
        for entry in plan:
            try:result=delete_one(entry,records,journal=journal)
            except BaseException as exc:
                journal(dict(phase='ERROR',path=entry['path'],target_still_exists=Path(entry['path']).exists(),error=repr(exc),at_ns=time.time_ns()))
                raise
            journal(dict(phase='VERIFIED_DELETED',**result))
            writer.writerow({k:result[k] for k in fields});f.flush();os.fsync(f.fileno())
            print('DELETED '+entry['path'],flush=True)
    write('DELETE_FINISHED.json',dict(deleted_files=len(plan),at_ns=time.time_ns(),free_bytes=shutil.disk_usage(ROOT).free))

if __name__=='__main__':main()
