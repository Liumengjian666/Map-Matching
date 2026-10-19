#!/usr/bin/python3
"""One-shot P62-R3 acquisition. No retries outside the registered chunk budget."""
import csv
import datetime
import hashlib
import json
import os
import signal
import shutil
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

from recovery_io import (BASE,CHUNK,IdentityError,StorageError,TransportError,DeadlineError,
                         HttpsOnly,check_deadline,internal,reserve,identity,request_bytes,
                         append_complete,publish)

ROOT=BASE/'p62_r3_exp18_recovery_data'
OLD=BASE/'p62_r2_hilti_exp18_g0_data'
DOC=Path(__file__).resolve().parent
REV='e62017f907007fdc5ab8c721842e4ae7359d7f49'
DEADLINE=datetime.datetime(2026,10,19,12,0,0,tzinfo=datetime.timezone.utc).timestamp()
FILES=[('rosbags/exp18_corridor_lower_gallery_2.bag',8657654860,
        '98373b52f207b9ace3914792b5604a6642f89441190cabca6acbaf1d17249280',b'#ROSBAG V2.0\n'),
       ('laser_scans/sheldonian_full.e57',5447128064,
        '46575d3eaf3169a24e24ba0013cc48610125ac842d6745e5e8156c6aecc59756',b'ASTM-E57')]
LINEAR=OLD/'staging/exp18_corridor_lower_gallery_2.bag.part'
SEED_SIZE=452984832
PREFIX_BYTES=1024**2
FROZEN_FILE=BASE/'p62_r2_hilti_exp18_g0_ws/docs/p62_r2_hilti_exp18_g0_admission/asset_gate_after.json'
FROZEN_SHA='565c3588896745400f5899b4d4273ef6b36dd6caa6e112ef74a06e1e5ad28a26'
FIELDS=['asset','kind','start','end_inclusive','attempt','http_status','content_range',
        'received_bytes','chunk_sha256','elapsed_s','status','error','valid_prefix_bytes']


def save(path,value):
    internal(path.parent,create=True)
    internal(path)
    encoded=(json.dumps(value,ensure_ascii=False,indent=2)+'\n').encode()
    reserve(path.parent,len(encoded))
    with path.open('w') as handle:handle.write(encoded.decode());handle.flush();os.fsync(handle.fileno())


def old_snapshot(deadline):
    internal(OLD/'staging')
    return [identity(path,deadline) for path in sorted((OLD/'staging').glob('*.part'))]


def alarm(signum,frame):raise DeadlineError('GLOBAL_TRANSFER_DEADLINE')


def run():
    if ROOT.exists() or ROOT.is_symlink() or (DOC/'transfer_manifest.json').exists():
        raise IdentityError('ONE_SHOT_OUTPUT_EXISTS_NO_OVERWRITE')
    began_at=time.time()
    hard_deadline=min(DEADLINE,began_at+180*60)
    transfer_deadline=hard_deadline-60
    before=None
    start_free=shutil.disk_usage(BASE).free
    manifest={'task':'P62-R3-BOUNDED-EXP18-ASSET-RECOVERY','revision':REV,
              'started_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'hard_deadline_utc':datetime.datetime.fromtimestamp(hard_deadline,datetime.timezone.utc).isoformat(),
              'transfer_deadline_utc':datetime.datetime.fromtimestamp(transfer_deadline,datetime.timezone.utc).isoformat(),
              'initial_free_bytes':start_free,'chunk_bytes':CHUNK,'socket_timeout_s':90,
              'max_attempts_per_chunk':2,'single_active_request':True,'assets':[]}
    logpath=DOC/'chunk_manifest.csv'
    opener=urllib.request.build_opener(HttpsOnly())

    def log(row):
        reserve(DOC,8192)
        with logpath.open('a',newline='') as handle:
            writer=csv.DictWriter(handle,fieldnames=FIELDS,lineterminator='\n')
            writer.writerow({key:row.get(key,'NA') for key in FIELDS});handle.flush();os.fsync(handle.fileno())

    def fetch(relative,start,end,total,kind='SEGMENT',attempt_limit=2):
        for attempt in range(1,attempt_limit+1):
            row={'asset':relative,'kind':kind,'start':start,'end_inclusive':end,'attempt':attempt,
                 'http_status':'NA','content_range':'NA','valid_prefix_bytes':start}
            began=time.monotonic()
            url='https://huggingface.co/datasets/Hilti-Research/hilti-slam-challenge-2022/resolve/'+REV+'/'+relative
            url+='?download=true&r3_range={}-{}&attempt={}'.format(start,end,attempt)
            try:
                body=request_bytes(url,start,end,total,transfer_deadline,row,opener)
            except (TimeoutError,OSError,urllib.error.URLError,TransportError) as error:
                if isinstance(error,urllib.error.HTTPError):
                    row.update(http_status=error.code,content_range=error.headers.get('Content-Range',''))
                row.update(status='TRANSPORT_FAILED',error=type(error).__name__+':'+str(error),
                           elapsed_s=time.monotonic()-began)
                log(row)
                if isinstance(error,urllib.error.HTTPError) and error.code in (401,403):
                    raise TransportError('OFFICIAL_ACCESS_CONTROL:'+str(error.code))
                if attempt==attempt_limit:raise TransportError('CHUNK_ATTEMPTS_EXHAUSTED:'+str(error))
                continue
            except BaseException as error:
                row.update(status='REJECTED',error=type(error).__name__+':'+str(error),elapsed_s=time.monotonic()-began)
                log(row);raise
            row.update(status='NETWORK_SEGMENT_VALIDATED',error='',elapsed_s=time.monotonic()-began)
            log(row)
            return body

    signal.signal(signal.SIGALRM,alarm)
    signal.setitimer(signal.ITIMER_REAL,max(.001,transfer_deadline-time.time()))
    try:
        check_deadline(transfer_deadline)
        if ROOT.exists() or ROOT.is_symlink():raise IdentityError('ONE_SHOT_ROOT_EXISTS')
        if logpath.exists():raise IdentityError('ONE_SHOT_RECEIPT_EXISTS')
        if start_free<48000000000:raise StorageError('START_FREE_BELOW_48000000000:'+str(start_free))
        before=old_snapshot(transfer_deadline)
        save(DOC/'asset_identity_before.json',{'old_partials':before})
        if identity(FROZEN_FILE,transfer_deadline)['sha256']!=FROZEN_SHA:
            raise IdentityError('R2_FROZEN_SNAPSHOT_CHANGED')
        frozen=json.loads(FROZEN_FILE.read_text())
        expected={r['path']:r for r in frozen['preserved_incomplete_files']}
        actual={r['path']:r for r in before}
        if str(LINEAR) not in expected or not expected.keys()<=actual.keys():
            raise IdentityError('FROZEN_PARTIAL_ENTRY_MISSING')
        for path,prior in expected.items():
            row=actual[path]
            if row['bytes']!=prior['bytes'] or row['sha256']!=prior['sha256']:
                raise IdentityError('OLD_PARTIAL_IDENTITY_CHANGED:'+path)
        if LINEAR.stat().st_size!=SEED_SIZE:raise IdentityError('LINEAR_SIZE_CHANGED')
        with LINEAR.open('rb') as handle:local_prefix=handle.read(PREFIX_BYTES)
        if len(local_prefix)!=PREFIX_BYTES or not local_prefix.startswith(FILES[0][3]):
            raise IdentityError('LINEAR_MAGIC_OR_PREFIX_SIZE')
        internal(ROOT,create=True);internal(ROOT/'staging',create=True)
        reserve(DOC,4096)
        with logpath.open('x',newline='') as handle:
            csv.DictWriter(handle,fieldnames=FIELDS,lineterminator='\n').writeheader()
        probe=fetch(FILES[0][0],0,PREFIX_BYTES-1,FILES[0][1],'PREFIX_PROBE',1)
        if probe!=local_prefix:raise IdentityError('PREFIX_BYTES_DIFFER')
        manifest['prefix_probe']={'bytes':len(probe),'sha256':hashlib.sha256(probe).hexdigest(),'exact_match':True}
        save(DOC/'transfer_manifest.json',manifest)
        for index,(relative,size,digest,magic) in enumerate(FILES):
            target=ROOT/'staging'/(Path(relative).name+'.recovering.part')
            asset={'relative':relative,'url':'https://huggingface.co/datasets/Hilti-Research/hilti-slam-challenge-2022/resolve/'+REV+'/'+relative,
                   'expected_bytes':size,'expected_sha256':digest,'staging_path':str(target),'valid_prefix_bytes':0}
            manifest['assets'].append(asset)
            reserve(target.parent,4096)
            with target.open('xb') as output:
                if index==0:
                    with LINEAR.open('rb') as source:
                        for block in iter(lambda:source.read(1024**2),b''):
                            check_deadline(transfer_deadline);reserve(target.parent,len(block))
                            if output.write(block)!=len(block):raise StorageError('SHORT_SEED_WRITE')
                output.flush();os.fsync(output.fileno())
            if index==0:
                copy_id=identity(target,transfer_deadline)
                original=next(row for row in before if row['path']==str(LINEAR))
                if copy_id['bytes']!=original['bytes'] or copy_id['sha256']!=original['sha256']:
                    raise IdentityError('SEED_COPY_IDENTITY')
            asset['valid_prefix_bytes']=target.stat().st_size
            while asset['valid_prefix_bytes']<size:
                start=asset['valid_prefix_bytes'];end=min(start+CHUNK,size)-1
                body=fetch(relative,start,end,size)
                append_complete(target,body,start,transfer_deadline)
                asset['valid_prefix_bytes']=end+1
                log({'asset':relative,'kind':'COMMIT','start':start,'end_inclusive':end,
                     'received_bytes':len(body),'chunk_sha256':hashlib.sha256(body).hexdigest(),
                     'status':'COMMITTED','valid_prefix_bytes':end+1})
                save(DOC/'transfer_manifest.json',manifest)
                print(json.dumps({'asset':relative,'valid_prefix_bytes':end+1,'total_bytes':size,
                                  'utc':datetime.datetime.now(datetime.timezone.utc).isoformat()}),flush=True)
            destination=ROOT/'assets/verified'/relative
            asset['identity']=publish(target,destination,size,digest,magic,transfer_deadline)
            asset['status']='VERIFIED_AND_PUBLISHED'
            save(DOC/'transfer_manifest.json',manifest)
        manifest['result']='ASSET_RECOVERY_PASS_BOTH_FIXED_REVISION_FILES'
    except StorageError as error:
        manifest.update(result='ASSET_RECOVERY_BLOCKED_STORAGE',failure_category='STORAGE',error=str(error))
    except IdentityError as error:
        manifest.update(result='ASSET_RECOVERY_BLOCKED_IDENTITY',failure_category='IDENTITY',error=str(error))
    except BaseException as error:
        bag_ok=bool(manifest['assets'] and manifest['assets'][0].get('status')=='VERIFIED_AND_PUBLISHED')
        manifest.update(result='ASSET_RECOVERY_PARTIAL_BAG_ONLY' if bag_ok else 'ASSET_RECOVERY_BLOCKED_TRANSPORT',
                        failure_category=('DEADLINE' if isinstance(error,DeadlineError) else
                          'INTERRUPTED' if isinstance(error,KeyboardInterrupt) else
                          'TRANSPORT' if isinstance(error,TransportError) else
                          'LOCAL_IO' if isinstance(error,OSError) else 'UNEXPECTED'),
                        error=type(error).__name__+':'+str(error))
    finally:
        signal.setitimer(signal.ITIMER_REAL,0)
        signal.setitimer(signal.ITIMER_REAL,max(.001,hard_deadline-time.time()))
        final_errors=[]
        after=None
        def final_failure(label,error):
            final_errors.append(label+':'+repr(error))
            manifest['finalization_errors']=list(final_errors)
            if manifest.get('result')=='ASSET_RECOVERY_PASS_BOTH_FIXED_REVISION_FILES':
                manifest.update(result='ASSET_RECOVERY_BLOCKED_IDENTITY',error='FINALIZATION_NOT_VERIFIED')
        try:
            after=old_snapshot(hard_deadline)
            check_deadline(hard_deadline)
            save(DOC/'asset_identity_after.json',{'old_partials':after})
            if before is not None and before!=after:
                manifest.update(result='ASSET_RECOVERY_BLOCKED_IDENTITY',
                                primary_failure=manifest.get('error'),error='OLD_PARTIALS_CHANGED')
        except BaseException as error:final_failure('AFTER_SNAPSHOT',error)
        manifest['finished_utc']=datetime.datetime.now(datetime.timezone.utc).isoformat()
        def storage_audit():
            return {'initial_free_bytes':start_free,'final_free_bytes':shutil.disk_usage(BASE).free,
             'minimum_start_bytes':48000000000,
             'minimum_reserve_bytes':20*1024**3,'conversion_performed':False,
             'new_dependencies_installed':False,'old_partials_unchanged':before is not None and before==after}
        for path,build in [(DOC/'storage_audit.json',storage_audit),
                           (DOC/'transfer_manifest.json',lambda:manifest)]:
            try:
                check_deadline(hard_deadline)
                save(path,build())
            except DeadlineError as error:
                final_failure(str(path),error)
                break  # no more filesystem operations after the hard deadline
            except BaseException as error:final_failure(str(path),error)
        signal.setitimer(signal.ITIMER_REAL,0)
    print(json.dumps(manifest),flush=True)
    return manifest


if __name__=='__main__':
    if sys.argv[1:]!=['--run']:raise SystemExit('Explicit --run required; one-shot absolute deadline applies')
    run()
