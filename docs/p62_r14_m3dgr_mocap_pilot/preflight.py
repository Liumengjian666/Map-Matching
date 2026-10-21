#!/usr/bin/env python3
"""One bounded official-metadata admission attempt; never downloads sensor payloads."""
import hashlib
import json
from pathlib import Path
import re
import runpy
import shutil
import subprocess
import time
import urllib.request

DOC=Path(__file__).resolve().parent
REPO=DOC.parents[1]
REV='e0cf7d59c9a5a3df515624034698d976abc26549'
BASE='cfae39344a17ac2ee5e598fd7875d33f8c109794'
RESERVE=20*1024**3
R13=Path('/home/jian/livox_ws/p62_r13_multihypothesis_ws')

def write(path,value):
    with Path(path).open('x') as f:json.dump(value,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n')
def identity(path):
    p=Path(path);return dict(path=str(p),bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest())
def git(root,*args):return subprocess.check_output(['git','--no-optional-locks','-C',str(root),*args],text=True)
def snapshot():
    # Read-only source/index/refs/status/file snapshot; no backup/recovery action.
    helper=Path('/home/jian/livox_ws/p62_r12_feasibility_pool_ws/docs/p62_r12_feasibility_preserving_pool/execute.py')
    stable=runpy.run_path(str(helper),run_name='R14_readonly_snapshot_helper')['stable_snapshot']()
    return dict(stable=stable,R13_HEAD=git(R13,'rev-parse','HEAD').strip(),R13_status=git(R13,'status','--porcelain=v1'),
        R13_manifest=identity(R13/'docs/p62_r13_training_guided_multihypothesis/checksums.sha256'))
def admission(free,download,unpacked,map_cache):
    if any(v is None for v in [download,unpacked,map_cache]):return 'BLOCKED_PEAK_SIZE_NOT_CERTIFIED'
    return 'PASS' if free-download-unpacked-map_cache>=RESERVE else 'BLOCKED_STORAGE'
def fetch_small(name):
    url=f'https://raw.githubusercontent.com/sjtuyinjie/M3DGR/{REV}/{name}'
    start=time.monotonic()
    try:
        with urllib.request.urlopen(urllib.request.Request(url,headers={'User-Agent':'P62-R14-readonly-preflight'}),timeout=15) as response:
            data=response.read(65537)
            if len(data)>65536:raise RuntimeError('metadata exceeds fixed64KiB limit')
            status=response.status
        target=DOC/'evidence'/name
        with target.open('xb') as f:f.write(data)
        return dict(url=url,status=status,identity=identity(target),seconds=time.monotonic()-start),data.decode('utf-8-sig')
    except Exception as exc:return dict(url=url,error=type(exc).__name__+': '+str(exc),seconds=time.monotonic()-start),None
def probe_head(url):
    start=time.monotonic();record=dict(url=url,method='HEAD',payload_bytes_read=0)
    try:
        request=urllib.request.Request(url,method='HEAD',headers={'User-Agent':'P62-R14-readonly-preflight'})
        with urllib.request.urlopen(request,timeout=15) as response:
            record.update(status=response.status,final_url=response.geturl(),content_type=response.headers.get('Content-Type'),content_length=response.headers.get('Content-Length'))
    except Exception as exc:
        record.update(error=type(exc).__name__+': '+str(exc),status=getattr(exc,'code',None))
    record['seconds']=time.monotonic()-start;return record

def main():
    if (DOC/'PREFLIGHT_RECEIPT.json').exists() or (DOC/'PREFLIGHT_STARTED.json').exists():raise RuntimeError('one admission attempt only')
    if git(REPO,'status','--porcelain=v1').strip():raise RuntimeError('commit preflight source first')
    head=git(REPO,'rev-parse','HEAD').strip();free=shutil.disk_usage(REPO).free
    before=snapshot()
    if before['R13_HEAD']!=BASE or before['R13_status']:raise RuntimeError('frozen baseline identity differs')
    write(DOC/'ASSET_IDENTITY_BEFORE.json',before)
    write(DOC/'PREFLIGHT_STARTED.json',dict(CODE_SHA=head,official_revision=REV,free_bytes=free,reserve_bytes=RESERVE))
    (DOC/'evidence').mkdir(exist_ok=False)
    sources=[];readme=None;checksums=None
    for name in ['README.md','checksums.txt']:
        record,text=fetch_small(name);sources.append(record)
        if text is None:break
        if name=='README.md':readme=text
        else:checksums=text
    line=next((s for s in (readme or '').splitlines() if s.startswith('| Wheel-float01') or s.startswith('Wheel-float01')),None)
    # Accommodate standard Markdown table spacing without searching another sequence.
    if line is None:line=next((s for s in (readme or '').splitlines() if re.match(r'^\s*\|?\s*Wheel-float01\s*\|',s)),None)
    links=re.findall(r'\[OneDrive\]\((https://[^)]+)\)',line or '')
    checksum_line=next((s for s in (checksums or '').splitlines() if 'Wheel-float01.bag' in s),None)
    expected=checksum_line.split()[0].lower() if checksum_line else None
    if expected is not None and not re.fullmatch('[0-9a-f]{64}',expected):raise RuntimeError('invalid official digest')
    bag_probe=probe_head(links[0]) if links else dict(status='NOT_RUN',reason='official frozen README file link unavailable')
    # No payload request, alternate-host retry, other sequence or GT download.
    status='BLOCKED_ASSET_ACQUISITION'
    known_bag_bytes=None
    if bag_probe.get('status')==200 and bag_probe.get('content_type') and not any(t in bag_probe['content_type'].lower() for t in ['text/html','text/plain']):
        try:known_bag_bytes=int(bag_probe['content_length'])
        except (ValueError,TypeError):pass
    storage=dict(initial_free_bytes=free,reserve_bytes=RESERVE,headroom_bytes=free-RESERVE,
        official_sequence_row=line,official_size_is_rounded=True,exact_bag_bytes=known_bag_bytes,
        unpacked_peak_bytes=None,map_and_parser_peak_bytes=None,
        admission=admission(free,known_bag_bytes,None,None),
        optimistic_1_5_decimal_GB_remaining_bytes=free-1500000000,
        note='1.5g is not an exact byte identity; no certified download/extraction/map peak exists, so do not download')
    write(DOC/'STORAGE_AUDIT.json',storage)
    after=snapshot();write(DOC/'ASSET_IDENTITY_AFTER.json',after)
    if after!=before:raise RuntimeError('protected production/baseline identity changed')
    write(DOC/'PREFLIGHT_RECEIPT.json',dict(result=status,CODE_SHA=head,official_revision=REV,sources=sources,
        official_sequence_row=line,official_expected_bag_sha256=expected,local_payload_verified=False,
        bag_HEAD_probe=bag_probe,GT_HEAD_probe=dict(status='NOT_RUN_AFTER_BAG_ADMISSION_BLOCK'),GT_link=links[1] if len(links)>1 else None,
        storage=storage,protected_assets_equal=True,payload_downloaded_bytes=0,
        map_builds=0,new_NDT_align=0,GT_pose_files_read=0,wheel_messages_read=0,stop='No further admission, map, localization or evaluation performed'))
    print(json.dumps(dict(result=status,bag_HEAD=bag_probe,storage=storage,official_expected_sha256=expected,protected_assets_equal=True),ensure_ascii=False,indent=2))

if __name__=='__main__':main()
