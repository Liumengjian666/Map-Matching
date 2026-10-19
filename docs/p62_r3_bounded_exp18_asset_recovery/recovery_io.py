"""Standard-library-only IO guards. No ROS, point-cloud parser or optimizer."""
import hashlib
import math
import os
import shutil
import stat
import time
import urllib.error
import urllib.request
from pathlib import Path
from urllib.parse import urlsplit

BASE=Path('/home/jian/livox_ws')
RESERVE=20*1024**3
CHUNK=64*1024**2


class IdentityError(RuntimeError):pass
class StorageError(RuntimeError):pass
class TransportError(RuntimeError):pass
class DeadlineError(RuntimeError):pass


def check_deadline(deadline):
    if time.time()>=deadline:raise DeadlineError('TASK_DEADLINE')


def internal(path, create=False):
    path=Path(path)
    if BASE not in path.parents and path!=BASE:raise IdentityError('OUTSIDE_INTERNAL_BASE')
    ancestors=(path,)+tuple(path.parents)
    for node in ancestors:
        if node==BASE:break
        if node.is_symlink():raise IdentityError('SYMLINK_PATH:'+str(node))
        if node.exists() and node.stat().st_dev!=BASE.stat().st_dev:
            raise IdentityError('DIFFERENT_DEVICE:'+str(node))
    if create:
        path.mkdir(parents=True,exist_ok=True)
        internal(path)
    return path


def reserve(path, pending=0, minimum=RESERVE):
    free=shutil.disk_usage(str(path)).free
    # Round for allocation, and leave 1 MiB for directory/receipt bookkeeping.
    required=minimum+math.ceil(pending/4096)*4096+1024**2
    if free<required:raise StorageError('FREE={} REQUIRED={}'.format(free,required))
    return free


def identity(path, deadline=None):
    path=internal(path)
    info=path.lstat()
    if not stat.S_ISREG(info.st_mode):raise IdentityError('NOT_REGULAR_FILE')
    h=hashlib.sha256()
    with path.open('rb') as handle:
        while True:
            if deadline is not None:check_deadline(deadline)
            block=handle.read(4*1024**2)
            if not block:break
            h.update(block)
    return {'path':str(path),'bytes':info.st_size,'allocated_bytes':info.st_blocks*512,
            'st_blocks':info.st_blocks,'type':'REGULAR_FILE','sha256':h.hexdigest()}


class HttpsOnly(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        parsed=urlsplit(newurl)
        host=parsed.hostname or ''
        if parsed.scheme!='https' or not (host=='huggingface.co' or host.endswith('.huggingface.co') or host.endswith('.hf.co')):
            raise IdentityError('UNEXPECTED_REDIRECT_HOST')
        return super().redirect_request(req,fp,code,msg,headers,newurl)


def request_bytes(url, start, end, total, deadline, row, opener):
    """end is inclusive; buffer ONLY, no effective-prefix write on failure."""
    check_deadline(deadline)
    payload=bytearray()
    request=urllib.request.Request(url,headers={'Range':'bytes={}-{}'.format(start,end),
                                               'Cache-Control':'no-cache'})
    try:
        with opener.open(request,timeout=90) as response:
            row['http_status']=response.status
            row['content_range']=response.headers.get('Content-Range','')
            expected='bytes {}-{}/{}'.format(start,end,total)
            if response.status!=206 or row['content_range']!=expected:
                raise IdentityError('RANGE_RESPONSE_MISMATCH')
            while len(payload)<end-start+1:
                check_deadline(deadline)
                data=response.read1(min(1024**2,end-start+1-len(payload)))
                if not data:raise TransportError('SHORT_RANGE_BODY')
                payload.extend(data)
            if response.read1(1):raise IdentityError('EXCESS_RANGE_BODY')
            check_deadline(deadline)
    finally:
        row['received_bytes']=len(payload)
        row['chunk_sha256']=hashlib.sha256(payload).hexdigest()
    return payload


def append_complete(target, payload, expected_start, deadline):
    """Called only after the COMPLETE network segment was validated."""
    target=internal(target)
    if target.stat().st_size!=expected_start:raise IdentityError('PREFIX_LENGTH_CONFLICT')
    with target.open('ab',buffering=0) as handle:
        for offset in range(0,len(payload),1024**2):
            check_deadline(deadline)
            block=memoryview(payload)[offset:offset+1024**2]
            reserve(target.parent,len(block))
            if handle.write(block)!=len(block):raise StorageError('SHORT_LOCAL_WRITE')
        os.fsync(handle.fileno())


def publish(source,destination,size,digest,magic,deadline):
    source=internal(source);internal(destination.parent,create=True)
    verified=identity(source,deadline)
    if verified['bytes']!=size or verified['sha256']!=digest:raise IdentityError('FULL_FILE_IDENTITY')
    with source.open('rb') as handle:
        if handle.read(len(magic))!=magic:raise IdentityError('FULL_FILE_MAGIC')
    if destination.exists() or destination.is_symlink():raise IdentityError('PUBLISH_PATH_EXISTS')
    reserve(destination.parent,4096)
    source.chmod(0o444)
    os.link(str(source),str(destination))
    published=identity(destination,deadline)
    if published['bytes']!=size or published['sha256']!=digest:raise IdentityError('POST_PUBLISH_IDENTITY')
    if source.stat().st_ino!=destination.stat().st_ino:raise IdentityError('NOT_SAME_HARDLINK')
    return {'staging':verified,'published':published,'hardlink_verified':True}
