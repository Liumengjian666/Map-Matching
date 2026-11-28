"""Fetch the two public Hilti assets from the official hub with resumable ranges."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
from pathlib import Path
import time
import subprocess
import shutil
import re

BASE="https://huggingface.co/datasets/Hilti-Research/hilti-slam-challenge-2022/resolve/main/"
ASSETS={
 "bag":("rosbags/exp18_corridor_lower_gallery_2.bag",8657654860,"98373b52f207b9ace3914792b5604a6642f89441190cabca6acbaf1d17249280"),
 "map":("laser_scans/sheldonian_full.e57",5447128064,"46575d3eaf3169a24e24ba0013cc48610125ac842d6745e5e8156c6aecc59756"),
}
BLOCK=32*1024*1024


def valid_range(body,headers,expected):
    if not body.exists() or not body.stat().st_size or not headers.exists():
        return False
    actual=re.findall(r"^content-range:\s*([^\r\n]+)",headers.read_text(),re.M|re.I)
    return bool(actual and actual[-1].strip()==expected)


def interval(url,folder,lo,hi,total):
    p=folder/f"{lo}_{hi}.part"
    scratch=p.with_suffix(".next")
    headers=p.with_suffix(".headers")
    for attempt in range(5):
        start=lo+(p.stat().st_size if p.exists() else 0)
        if start==hi+1:
            return p
        if start>hi:
            raise ValueError("Bad local range size")
        try:
            expected=f"bytes {start}-{hi}/{total}"
            if not valid_range(scratch,headers,expected):
                process=subprocess.run(["curl","-4","--http1.1","--fail","--location","--silent","--show-error","--connect-timeout","15","--max-time","360",
                                        "--range",f"{start}-{hi}","--dump-header",str(headers),url,"-o",str(scratch)],
                                       capture_output=True)
                if not valid_range(scratch,headers,expected):
                    raise ValueError(f"Range {expected} unavailable: {process.stderr.decode(errors='replace').strip()}")
            if scratch.stat().st_size>hi-start+1:
                raise ValueError("oversized range")
            with p.open("ab") as f,scratch.open("rb") as data:
                shutil.copyfileobj(data,f,1024*1024)
            scratch.unlink()
        except (OSError,ValueError) as error:
            print(f"retry {lo}-{hi} {error}",flush=True)
            time.sleep(min(8,2**attempt))
    if p.exists() and p.stat().st_size==hi-lo+1:
        return p
    raise RuntimeError(f"incomplete {p}")


def parts_for(folder,key):
    asset,total,_=ASSETS[key]
    filename=asset.rsplit("/",1)[1]
    initial=folder/(filename+".part")
    start=initial.stat().st_size if initial.exists() else 0
    caches=folder/(key+"_ranges")
    return initial,total,caches,[(lo,min(lo+BLOCK,total)-1) for lo in range(start,total,BLOCK)]


def prefix(folder,key,output,limit=None):
    initial,total,caches,parts=parts_for(folder,key)
    options=([(initial,initial.stat().st_size)] if initial.exists() else [])+[(caches/f"{lo}_{hi}.part",hi-lo+1) for lo,hi in parts]
    have=0
    with output.open("wb") as w:
        for path,n in options:
            if not path.exists():
                break
            size=path.stat().st_size
            amount=min(n,size)
            if limit is not None:
                amount=min(amount,limit-have)
            with path.open("rb") as f:
                left=amount
                while left:
                    b=f.read(min(left,2**20))
                    if not b:
                        raise IOError("range truncated while reading")
                    w.write(b)
                    left-=len(b)
            have+=amount
            if amount!=n:
                break
    print(f"prefix {key}: {have} bytes",flush=True)
    return have


def main(a):
    if a.prefix:
        prefix(a.folder,a.asset,a.prefix,a.limit_bytes)
        return
    a.folder.mkdir(parents=True,exist_ok=True)
    initial,total,caches,parts=parts_for(a.folder,a.asset)
    caches.mkdir(exist_ok=True)
    dest,digest=ASSETS[a.asset][0],ASSETS[a.asset][2]
    url=BASE+dest
    with ThreadPoolExecutor(max_workers=a.workers) as pool:
        results=list(pool.map(lambda rng:interval(url,caches,*rng,total),parts))
    name=initial.name.removesuffix(".part") if hasattr(str,"removesuffix") else initial.name[:-5]
    p=a.folder/(name+".assembled")
    if prefix(a.folder,a.asset,p)!=total or len(results)!=len(parts):
        raise ValueError("incomplete assembly")
    h=hashlib.sha256()
    with p.open("rb") as f:
        for b in iter(lambda:f.read(4*1024*1024),b""):
            h.update(b)
    if h.hexdigest()!=digest:
        raise ValueError("official SHA mismatch")
    p.rename(a.folder/name)
    print("verified",name,digest)


if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("folder",type=Path)
    p.add_argument("asset",choices=ASSETS)
    p.add_argument("--prefix",type=Path)
    p.add_argument("--limit-bytes",type=int)
    p.add_argument("--workers",type=int,default=8)
    main(p.parse_args())
