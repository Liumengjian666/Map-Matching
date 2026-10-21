#!/usr/bin/env python3
"""Bounded disk inventory, read-only reference evidence and streaming SHA256."""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import runpy
import stat
import subprocess
import time

OUT = Path(__file__).resolve().parent
ROOT = Path('/home/jian/rosbag')
PROTECTED = {
 str(ROOT/'loop2/raw_restore_20260914_stage0/loop2_raw.bag'): 'd17bee3f8697dcb01cb4b9f6ea2483aa42a1a31ae277659c8a7869d9c69f13ae',
 str(ROOT/'loop2/loop2mapping/pcd/loop2_simtime_rebuild_2026_08_12_001_all_downsampled_points.pcd'): '016b7ae3ae494ad5d462a66d888a96b96dda5fe29d6058059aa2fa6bdf6b09ca',
 str(ROOT/'loop2/final_delivery_run_A_20260922/ndt_determinism.csv'): 'e5f749cbbc7fc514326cc61626945f4ac6347722e4ad769e676984e1d214102a',
}

def write(name, value):
    with (OUT/name).open('x') as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n')

def csvout(name, rows, fields):
    with (OUT/name).open('x', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

def signature(p):
    s = Path(p).lstat()
    return dict(size=s.st_size, allocated=s.st_blocks*512, dev=s.st_dev, inode=s.st_ino,
                nlink=s.st_nlink, mtime_ns=s.st_mtime_ns, ctime_ns=s.st_ctime_ns, mode=s.st_mode)

def hash_record(p):
    before = signature(p)
    if not stat.S_ISREG(before['mode']) or Path(p).resolve() != Path(p):
        raise RuntimeError('Nonregular/symlink component: '+str(p))
    fd = os.open(str(p), os.O_RDONLY | os.O_NOFOLLOW)
    h = hashlib.sha256()
    with os.fdopen(fd, 'rb') as f:
        s=os.fstat(f.fileno())
        if (s.st_dev,s.st_ino,s.st_size,s.st_mtime_ns,s.st_ctime_ns)!=(before['dev'],before['inode'],before['size'],before['mtime_ns'],before['ctime_ns']):
            raise RuntimeError('Opened hash descriptor differs: '+str(p))
        for chunk in iter(lambda: f.read(8*1024**2), b''):
            h.update(chunk)
    if before != signature(p):
        raise RuntimeError('File changed during hash: '+str(p))
    return dict(path=str(p),sha256=h.hexdigest(),signature=before)

def sha(p):
    return hash_record(p)['sha256']

def command_file(name, argv):
    with (OUT/name).open('x') as f:
        result = subprocess.run(argv, stdout=f, stderr=subprocess.PIPE, text=True)
    if result.returncode:
        raise RuntimeError(f'{argv}: {result.returncode} {result.stderr}')

def protections():
    # Only reads a named snapshot function; never executes the historical runner.
    helper='/home/jian/livox_ws/p62_r12_feasibility_pool_ws/docs/p62_r12_feasibility_preserving_pool/execute.py'
    stable = runpy.run_path(helper, run_name='cleanup_readonly')['stable_snapshot']()
    projects = []
    for stage, directory in [(13,'p62_r13_multihypothesis_ws'), (14,'p62_r14_m3dgr_pilot_ws')]:
        repo=Path('/home/jian/livox_ws')/directory
        doc=repo/'docs'/('p62_r13_training_guided_multihypothesis' if stage==13 else 'p62_r14_m3dgr_mocap_pilot')
        argv=['git','--no-optional-locks','-C',str(repo)]
        head=subprocess.check_output(argv+['rev-parse','HEAD'],text=True).strip()
        status=subprocess.check_output(argv+['status','--porcelain=v1'],text=True)
        verified=[]
        for line in (doc/'checksums.sha256').read_text().splitlines():
            expected, relative=line.split('  ',1)
            actual=sha(doc/relative)
            if actual!=expected:raise RuntimeError('Archive SHA mismatch '+str(doc/relative))
            verified.append(dict(path=str(doc/relative),sha256=actual))
        projects.append(dict(stage=stage,HEAD=head,status=status,verified=verified))
    return dict(stable=stable,projects=projects)

def inventory():
    write('PROTECTION_BEFORE.json', protections())
    command_file('DF_BEFORE.txt',['df','-B1',str(ROOT)])
    command_file('DF_BEFORE_HUMAN.txt',['df','-h',str(ROOT)])
    command_file('DU_BEFORE.txt',['du','-xhd','2',str(ROOT)])
    rows=[]; errors=[]; device=ROOT.stat().st_dev
    for folder, dirs, files in os.walk(ROOT, followlinks=False, onerror=lambda e: errors.append(str(e))):
        dirs[:]=[d for d in dirs if not (Path(folder)/d).is_symlink()
                 and (Path(folder)/d).stat().st_dev==device and not os.path.ismount(Path(folder)/d)]
        for name in files:
            p=Path(folder)/name
            s=p.lstat()
            if not stat.S_ISREG(s.st_mode) or s.st_dev!=device or s.st_size<=300*1024**2:continue
            kind=subprocess.check_output(['file','-b','--',str(p)],text=True).strip()
            rows.append(dict(path=str(p),**signature(p),type=p.suffix or 'NO_EXTENSION',magic=kind,
                             mtime_utc=datetime.fromtimestamp(s.st_mtime,timezone.utc).isoformat()))
    if errors:raise RuntimeError(str(errors))
    rows.sort(key=lambda r:(-r['allocated'],-r['size'],r['path']))
    write('INVENTORY_BEFORE.json',rows)
    write('SIZE_COUNTS.json',{str(n)+'_GiB':sum(r['size']>n*1024**3 for r in rows) for n in [1,5,10]})
    print(json.dumps(dict(large_files=len(rows),above_1GiB=sum(r['size']>1024**3 for r in rows),
                         above_5GiB=sum(r['size']>5*1024**3 for r in rows),above_10GiB=sum(r['size']>10*1024**3 for r in rows)),indent=2),flush=True)

def hashes():
    rows=json.loads((OUT/'INVENTORY_BEFORE.json').read_text())
    groups={}
    for r in rows:groups.setdefault(r['size'],[]).append(r['path'])
    paths=set(PROTECTED)
    for group in groups.values():
        if len(group)>1:paths.update(group)
    records=[]
    for p in sorted(paths):
        print('HASH_BEGIN '+p,flush=True);start=time.monotonic()
        record=hash_record(p);value=record['sha256']
        record['seconds']=time.monotonic()-start
        if p in PROTECTED and value!=PROTECTED[p]:raise RuntimeError('Protected hash mismatch '+p)
        records.append(record)
        with (OUT/'HASH_PROGRESS.jsonl').open('a') as f:f.write(json.dumps(record)+'\n')
        print('HASH_OK '+value+' '+p,flush=True)
    write('HASHES_BEFORE.json',records)

def references():
    rows=json.loads((OUT/'INVENTORY_BEFORE.json').read_text())
    # Exact paths and specific parent directory names, never inference from a basename alone.
    pairs=[(r['path'],Path(r['path']).parent.name) for r in rows]
    tokens=sorted(set([p for p,_ in pairs]+[parent for _,parent in pairs if len(parent)>12]))
    with (OUT/'REFERENCE_PATTERNS_V2.txt').open('x') as f:f.write('\n'.join(tokens)+'\n')
    roots=[str(p) for p in Path('/home/jian/livox_ws').glob('p62_r*_ws') if int(p.name.split('_')[1][1:])>=6]
    roots+=['/home/jian/livox_ws/dog_visual_loc_ws/src','/home/jian/livox_ws/dog_light_loc_ws/src',
            '/home/jian/livox_ws/dog_loc_paper_ws']
    roots=[p for p in roots if Path(p).exists()]
    argv=['rg','--files-with-matches','--hidden','-F','-f',str(OUT/'REFERENCE_PATTERNS_V2.txt'),
          '-g','!.git/**','-g','!**/.git/**','-g','!**/build/**','-g','!**/devel/**',
          '-g','*.{py,sh,cpp,hpp,yaml,yml,launch,json,md,csv,txt}']+roots
    found=subprocess.run(argv,capture_output=True,text=True)
    if found.returncode not in [0,1] or found.stderr:raise RuntimeError(found.stderr)
    matches=[];skipped=[]
    for name in sorted(set(found.stdout.splitlines())):
        p=Path(name)
        if p.stat().st_size>32*1024**2:
            skipped.append(name);continue
        text=p.read_text(errors='replace')
        possible=[(path,parent) for path,parent in pairs if path in text or (len(parent)>12 and parent in text)]
        for no,line in enumerate(text.splitlines(),1):
            for path,parent in possible:
                if path in line or (len(parent)>12 and parent in line):
                    matches.append(dict(asset=path,source=name,line=no,
                                        match='EXACT_PATH' if path in line else 'PARENT_TOKEN',text=line[:1000]))
    write('REFERENCE_AUDIT.json',dict(roots=roots,command=argv,skipped_oversize=skipped,matches=matches,
          limitation='Historical mentions differ from active dependencies; each proposed deletion requires manual classification'))
    print(json.dumps(dict(references=len(matches),matched_files=len(set(r['source'] for r in matches)),skipped=len(skipped)),indent=2))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('stage',choices=['inventory','hashes','references'])
    globals()[parser.parse_args().stage]()
