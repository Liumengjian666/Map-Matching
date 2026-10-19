#!/usr/bin/python3
"""Bounded parallel ranges of ONE pinned asset at a time; never retries."""
import concurrent.futures
import json
import os
import signal
import threading
import time
import urllib.request

import acquire_assets as fixed
from publish_small import publish


def require_range(status, header, start, stop, size):
    if status != 206 or header != 'bytes {}-{}/{}'.format(start, stop-1, size):
        raise RuntimeError('RANGE_CONTRACT:' + str((status, header, start, stop, size)))


def main():
    free = fixed.available()
    if free < 48000000000:
        raise RuntimeError('G0_BLOCKED_STORAGE:' + str(free))
    fixed.internal_directory(fixed.ROOT)
    receipt = fixed.ROOT/'range_acquisition.json'
    with receipt.open('x') as handle:
        handle.write('{}\n')
    record = {'initial_available_bytes': free, 'revision': fixed.REVISION,
              'transport_change': 'Previous single-stream deliberately interrupted; original partial preserved',
              'workers_per_asset': 8, 'attempts_per_range': 1, 'assets': []}
    lock = threading.Lock()
    cancelled = threading.Event()
    try:
        for relative, size, digest, magic, cache_name in fixed.ASSETS:
            destination = fixed.ROOT/'assets'/relative
            fixed.internal_directory(destination.parent)
            if destination.is_symlink():
                raise RuntimeError('SYMLINK_ASSET')
            row = {'relative': relative, 'path': str(destination), 'expected_bytes': size,
                   'expected_sha256': digest}
            record['assets'].append(row)
            if destination.exists():
                row.update(fixed.verify(destination, size, digest));row['status']='VERIFIED_EXISTING';continue
            if cache_name:
                source = fixed.CACHE/cache_name
                row.update(publish(source,destination,size,digest));row['status']='VERIFIED_CACHE_COPY';continue
            temporary = fixed.ROOT/'staging'/(destination.name+'.ranged.part')
            fixed.internal_directory(temporary.parent)
            url = 'https://huggingface.co/datasets/Hilti-Research/hilti-slam-challenge-2022/resolve/'+fixed.REVISION+'/'+relative
            row['url'] = url
            descriptor = os.open(str(temporary),os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
            began = time.monotonic(); row['ranges']=[];row['completed_bytes']=0
            # Original partial is NOT consumed/modified; avoids any uncertain-prefix mixing.
            try:
                signal.signal(signal.SIGALRM,fixed.deadline)
                signal.setitimer(signal.ITIMER_REAL,1800)

                def fetch(start, stop):
                    info = {'start':start,'stop_exclusive':stop,'attempts':1}
                    with lock:row['ranges'].append(info)
                    range_url=url+'?download=true&g0_range={}-{}'.format(start,stop-1)
                    request = urllib.request.Request(range_url,headers={'Range':'bytes={}-{}'.format(start,stop-1)})
                    with urllib.request.urlopen(request,timeout=30) as response:
                        require_range(response.status,response.headers.get('Content-Range'),start,stop,size)
                        info.update(http_status=response.status,content_range=response.headers.get('Content-Range'))
                        offset=start
                        while offset<stop:
                            if cancelled.is_set():raise RuntimeError('PEER_RANGE_FAILED')
                            chunk=response.read1(min(1024**2,stop-offset))
                            if not chunk:raise RuntimeError('SHORT_RANGE')
                            with lock:
                                if fixed.available(temporary.parent)-8*1024**2<20*1024**3:
                                    raise RuntimeError('SAFETY_RESERVE')
                                written=os.pwrite(descriptor,chunk,offset)
                                if written!=len(chunk):raise RuntimeError('SHORT_WRITE')
                                offset+=written;row['completed_bytes']+=written
                            if time.monotonic()-began>1800:raise TimeoutError('TRANSFER_DEADLINE')
                        if response.read(1):raise RuntimeError('EXCESS_RANGE')
                        info['received_bytes']=stop-start
                    print(json.dumps({'asset':relative,'range_complete':info,'completed_bytes':row['completed_bytes']}),flush=True)

                with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
                    jobs=[pool.submit(fetch,size*i//8,size*(i+1)//8) for i in range(8)]
                    try:
                        for job in concurrent.futures.as_completed(jobs):job.result()
                    except BaseException:
                        cancelled.set()
                        raise
                os.fsync(descriptor)
            finally:
                signal.setitimer(signal.ITIMER_REAL,0)
                os.close(descriptor)
                row['wall_s']=time.monotonic()-began
                row['temporary_preserved']=str(temporary)
            with temporary.open('rb') as check:
                if not check.read(len(magic)).startswith(magic):raise RuntimeError('MAGIC')
            row.update(fixed.verify(temporary,size,digest))
            temporary.chmod(0o444)
            os.link(str(temporary),str(destination))
            row['status']='VERIFIED_PUBLISHED'
            row['mean_bytes_per_second']=size/row['wall_s']
            print(json.dumps(row),flush=True)
        record['status']='ALL_FIVE_LOCAL_IDENTITIES_VERIFIED'
    except BaseException as error:
        record.update(status='STOPPED',error=type(error).__name__+':'+str(error))
        raise
    finally:
        record['final_available_bytes']=fixed.available()
        with receipt.open('w') as handle:json.dump(record,handle,indent=2);handle.write('\n')


if __name__=='__main__':main()
