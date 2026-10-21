#!/usr/bin/env python3
"""One bounded official Alipan attempt; never repeats the prior OneDrive page."""
import hashlib
from contextlib import contextmanager
import json
import os
from pathlib import Path
import runpy
import resource
import shutil
import signal
import subprocess
import time
import urllib.parse
import urllib.error
import urllib.request

DOC=Path(__file__).resolve().parent
REPO=DOC.parents[2]
OLD=DOC.parent
DATA=Path('/home/jian/livox_ws/p62_r14_m3dgr_data/resume_20261021')
BASE='9125c4f9b313994b08572bec143e20b7b81d1d3b'
REV='e0cf7d59c9a5a3df515624034698d976abc26549'
EXPECTED='366171a5f1d0c1e7bd602e2b664d85dbd89dd08570c4165142ef9849029c1cf6'
URL='https://www.alipan.com/s/aVQ24Hm4Xcm'
GIB=1024**3
LIMITS={'bag':6*GIB,'gt_aux':GIB,'map_temp':20*GIB,'run_results':4*GIB}
RESERVE=20*GIB
NEAR=64*1024**2
MAGIC=b'#ROSBAG V2.0\n'

class RedirectRequiresUserAction(RuntimeError):pass
class TransferGuardError(RuntimeError):pass

@contextmanager
def request_deadline(seconds=1800):
    if signal.getitimer(signal.ITIMER_REAL)!=(0.0,0.0):raise RuntimeError('Do not replace an existing process timer')
    def expired(signum,frame):raise TimeoutError('Total HTTP wall deadline exceeded')
    previous=signal.signal(signal.SIGALRM,expired)
    signal.setitimer(signal.ITIMER_REAL,seconds)
    try:yield
    finally:
        signal.setitimer(signal.ITIMER_REAL,0)
        signal.signal(signal.SIGALRM,previous)

def write(name, value):
    with (DOC/name).open('x') as f:
        json.dump(value,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')

def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(1024**2),b''):h.update(block)
    return h.hexdigest()

def git(*args):
    return subprocess.check_output(['git','--no-optional-locks','-C',str(REPO),*args],text=True).strip()

def snapshot():
    # Read-only protection helper; no historical runner or preflight is executed.
    helper='/home/jian/livox_ws/p62_r12_feasibility_pool_ws/docs/p62_r12_feasibility_preserving_pool/execute.py'
    stable=runpy.run_path(helper,run_name='resume_protection_only')['stable_snapshot']()
    files=subprocess.check_output(['git','-C',str(REPO),'ls-tree','-r','--name-only',BASE,
                                  'docs/p62_r14_m3dgr_mocap_pilot'],text=True).splitlines()
    old_files=[dict(path=name,bytes=(REPO/name).stat().st_size,sha256=digest(REPO/name)) for name in files]
    return dict(stable=stable,old_R14_files=old_files)

def capacity(free):
    return free>=RESERVE+sum(LIMITS.values())

def write_allowed(free, written, next_bytes, limit):
    return next_bytes>=0 and written>=0 and written+next_bytes<limit-NEAR and free-next_bytes>=RESERVE

class OfficialRedirects(urllib.request.HTTPRedirectHandler):
    def __init__(self):
        super().__init__();self.visited={self.key(URL)};self.followed=[]
    @staticmethod
    def key(url):
        p=urllib.parse.urlsplit(url)
        return (p.scheme.lower(),(p.hostname or '').lower(),p.port or (443 if p.scheme.lower()=='https' else 80),
                urllib.parse.unquote(p.path).rstrip('/'))
    def redirect_request(self,req,fp,code,msg,headers,newurl):
        parsed=urllib.parse.urlsplit(newurl);host=(parsed.hostname or '').lower()
        if parsed.scheme.lower()!='https' or parsed.username or parsed.password:
            raise RedirectRequiresUserAction('Refuse insecure or credential-bearing redirect')
        if not any(host==domain or host.endswith('.'+domain) for domain in ['alipan.com','aliyundrive.com','aliyuncs.com']):
            raise RedirectRequiresUserAction('Redirect needs manual browser/auth inspection: '+host)
        key=self.key(newurl)
        if key in self.visited:raise RedirectRequiresUserAction('Repeated share/redirect URL refused')
        self.visited.add(key)
        redirected=super().redirect_request(req,fp,code,msg,headers,newurl)
        self.followed.append(newurl)
        return redirected

def receive(response, destination, evidence, free=lambda:shutil.disk_usage(DATA).free):
    first=response.read(64)
    if not first.startswith(MAGIC):
        # Only a bounded page sample, never write an HTML response as .bag.
        page=first+response.read(max(0,65536-len(first)))
        with evidence.open('xb') as f:f.write(page)
        return dict(status='ASSET_ACCESS_REQUIRES_USER_ACTION',sample_bytes=len(page),
                    sample_sha256=digest(evidence),sample_path=str(evidence),bag_payload_bytes=0,
                    reason='Official share response is not ROS bag bytes; no direct payload URL available')
    partial=destination.with_name(destination.name+'.part')
    h=hashlib.sha256();count=0;start=time.monotonic()
    with partial.open('xb') as f:
        block=first
        while block:
            if time.monotonic()-start>1800:raise TransferGuardError('Bounded30-minute transfer duration exceeded')
            if not write_allowed(free(),count,len(block),LIMITS['bag']):
                raise TransferGuardError('Bag/category or20GiB reserve approaching; partial preserved')
            f.write(block);h.update(block);count+=len(block)
            block=response.read(1024**2)
        f.flush();os.fsync(f.fileno())
    if h.hexdigest()!=EXPECTED:
        return dict(status='ASSET_IDENTITY_MISMATCH',bag_payload_bytes=count,sha256=h.hexdigest(),
                    expected_sha256=EXPECTED,partial=str(partial))
    os.link(partial,destination)  # Exclusive publication; never overwrite an existing asset.
    return dict(status='BAG_SHA_AND_MAGIC_VERIFIED',bag_payload_bytes=count,sha256=h.hexdigest(),
                path=str(destination),staging=str(partial),format_check='ROS bag V2.0 header only; traversal pending')

def main():
    if (DOC/'ACCESS_STARTED.json').exists():raise RuntimeError('Single attempt already started; do not repeat')
    if git('status','--porcelain=v1'):raise RuntimeError('Commit continuation source before access')
    free=shutil.disk_usage(REPO).free
    if not capacity(free):raise RuntimeError('Authorized31GiB budget plus20GiB reserve not available')
    if DATA.exists():raise RuntimeError('Inspect existing continuation data; do not overwrite')
    before=snapshot();write('PRESERVED_ASSETS_BEFORE.json',before)
    write('ACCESS_STARTED.json',dict(CODE_SHA=git('rev-parse','HEAD'),at_ns=time.time_ns(),
          base=BASE,official_revision=REV,free_bytes=free,reserve_bytes=RESERVE,limits_bytes=LIMITS,
          near_limit_stop_margin_bytes=NEAR,storage_status='PASS_FOR_DECLARED_BOUNDED_BUDGET',url=URL,
          expected_sha256=EXPECTED,old_preflight_invocations=0,old_OneDrive_requests=0))
    for name in ['bag','gt','map_temp','run_results']:(DATA/name).mkdir(parents=True,exist_ok=False)
    if DATA.resolve()!=DATA or DATA.stat().st_dev!=REPO.stat().st_dev or not capacity(shutil.disk_usage(DATA).free):
        raise RuntimeError('Actual target filesystem or capacity differs from admitted one')
    inventory=subprocess.run(['find','/home/jian/livox_ws','/home/jian/rosbag','/home/jian/geode','-xdev',
        '-type','f','-iname','*wheel*float*','!','-path','*/.git/*','-printf','%s\t%p\n'],capture_output=True,text=True,timeout=30)
    write('LOCAL_ASSET_SEARCH.json',dict(command='bounded find Wheel/float identifiers over3roots',
          returncode=inventory.returncode,matches=inventory.stdout.splitlines(),stderr=inventory.stderr,
          limitation='No claim about renamed/out-of-scope assets; no matching payload candidates found before network access'))
    if inventory.returncode or inventory.stderr:raise RuntimeError('Local asset search incomplete')
    if inventory.stdout.strip():raise RuntimeError('A local candidate exists; verify it before any access')
    record=dict(url=URL,method='GET',initial_GET_attempts=1,old_OneDrive_requests=0)
    redirects=OfficialRedirects();start=time.monotonic()
    try:
        opener=urllib.request.build_opener(redirects)
        request=urllib.request.Request(URL,headers={'User-Agent':'P62-R14-bounded-official-access','Accept-Encoding':'identity'})
        with request_deadline():
            with opener.open(request,timeout=30) as response:
                record.update(http_status=response.status,final_url=response.geturl(),
                    content_type=response.headers.get('Content-Type'),declared_length=response.headers.get('Content-Length'))
                record.update(receive(response,DATA/'bag/Wheel-float01.bag',DOC/'Alipan_bag_response.sample'))
    except (urllib.error.URLError,TimeoutError,RedirectRequiresUserAction) as exc:
        record.update(status='ASSET_ACCESS_REQUIRES_USER_ACTION',error=type(exc).__name__+': '+str(exc),
                      http_status=getattr(exc,'code',record.get('http_status')),
                      bag_payload_bytes=sum(p.stat().st_size for p in (DATA/'bag').iterdir() if p.is_file()))
    except Exception as exc:
        record.update(status='TRANSFER_OR_LOCAL_IO_BLOCKED',error=type(exc).__name__+': '+str(exc),
                      bag_payload_bytes=sum(p.stat().st_size for p in (DATA/'bag').iterdir() if p.is_file()))
    record.update(wall_seconds=time.monotonic()-start,redirects_followed=redirects.followed,
                  acquisition_process_peak_rss_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,
                  memory_scope='Whole Linux acquisition process incl protection snapshot; not NDT runtime')
    write('OFFICIAL_ACCESS_RECEIPT.json',record)
    after=snapshot();write('PRESERVED_ASSETS_AFTER.json',after)
    if before!=after:raise RuntimeError('Stable or historical R14 assets changed')
    write('RESUME_STAGE_RESULT.json',dict(status=record['status'],old_assets_unchanged=True,
          free_bytes_after=shutil.disk_usage(DATA).free,bag=record,GT_files_read=0,new_NDT_align=0,
          note='Stop after inaccessible bag; if bag is verified, subsequent authorized gates still required'))
    print(json.dumps(record,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
