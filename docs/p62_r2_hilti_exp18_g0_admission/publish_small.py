#!/usr/bin/python3
"""Verify reused three small files and stage before exclusive publication."""
import os
from pathlib import Path
import acquire_assets as fixed


def require_internal_source(path):
    base=Path('/home/jian/livox_ws').resolve()
    if base not in path.parents:
        raise RuntimeError('NON_INTERNAL_SOURCE')
    for node in (path,)+tuple(path.parents):
        if node==base:break
        if node.is_symlink() or node.stat().st_dev!=base.stat().st_dev:
            raise RuntimeError('NON_INTERNAL_SOURCE')


def publish(source,destination,size,digest):
    require_internal_source(source)
    fixed.verify(source,size,digest)
    fixed.internal_directory(destination.parent)
    if destination.is_symlink():raise RuntimeError('SYMLINK_DESTINATION')
    if destination.exists():return fixed.verify(destination,size,digest)
    temporary=fixed.ROOT/'staging'/(destination.name+'.cached.part')
    fixed.internal_directory(temporary.parent)
    with source.open('rb') as src,temporary.open('xb') as dst:
        payload=src.read(size+1)
        if fixed.available(temporary.parent)-len(payload)<20*1024**3:
            raise RuntimeError('SAFETY_RESERVE')
        dst.write(payload);dst.flush();os.fsync(dst.fileno())
    actual=fixed.verify(temporary,size,digest)
    temporary.chmod(0o444)
    os.link(str(temporary),str(destination))
    return actual


def main():
    for relative,size,digest,magic,cache_name in fixed.ASSETS:
        if cache_name:
            print(relative,publish(fixed.CACHE/cache_name,fixed.ROOT/'assets'/relative,size,digest))


if __name__=='__main__':main()
