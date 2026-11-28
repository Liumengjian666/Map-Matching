"""Resume the already public Corridor01 attachment using validated HTTP byte ranges."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from urllib.request import urlopen, Request
import argparse
import hashlib
import shutil
import time

URL = "https://drive.usercontent.google.com/download?confirm=t&id=1aIIqPiE10nX3IhidpxKhc4Psq-AMER1X&export=download"
TOTAL = 3903086488
DIGEST = "6f83a45868aa5fa7299690bab78b782642295074d14388dd37707a2cc693aa05"
FILENAME = "SubT_MRS_Hawkins_Long_Corridor_RC.zip"


def fetch(directory, lo, hi):
    target = directory/f"range_{lo}_{hi}.part"
    if target.exists() and target.stat().st_size>hi-lo+1:
        raise ValueError("Existing range exceeds expected size")
    for attempt in range(5):
        start=lo+(target.stat().st_size if target.exists() else 0)
        if start==hi+1:
            return target
        try:
            with urlopen(Request(URL, headers={"Range":f"bytes={start}-{hi}","User-Agent":"Mozilla/5.0"}),timeout=30) as reply:
                if reply.status!=206 or reply.headers.get("Content-Range")!=f"bytes {start}-{hi}/{TOTAL}":
                    raise RuntimeError(f"Not the public data interval. HTTP {reply.status} {reply.headers.get('Content-Range')}")
                with target.open("ab") as stream:
                    shutil.copyfileobj(reply,stream,1024*1024)
        except (OSError,RuntimeError) as error:
            print("Retry",lo,hi,str(error),flush=True)
            time.sleep(min(2**attempt,16))
    if target.exists() and target.stat().st_size==hi-lo+1:
        return target
    raise RuntimeError(f"Incomplete interval {target}")


def main(directory, prefix_size):
    initial=directory/(FILENAME+".part")
    assert initial.stat().st_size==prefix_size
    plan=directory/"range_plan.tsv"
    size=134217728
    ranges=[(i,min(i+size,TOTAL)-1) for i in range(prefix_size,TOTAL,size)]
    plan.write_text("".join(f"{a}\t{b}\n" for a,b in ranges))
    jobs=[]
    with ThreadPoolExecutor(max_workers=7) as pool:
        jobs=[pool.submit(fetch,directory,a,b) for a,b in ranges]
        parts=[j.result() for j in jobs]
    joined=directory/(FILENAME+".joined")
    digest=hashlib.sha256()
    with joined.open("xb") as output:
        for part in [initial,*parts]:
            with part.open("rb") as source:
                for piece in iter(lambda:source.read(4*1024*1024),b""):
                    output.write(piece)
                    digest.update(piece)
    if joined.stat().st_size!=TOTAL or digest.hexdigest()!=DIGEST:
        raise ValueError(f"Attachment identity mismatch {digest.hexdigest()}")
    joined.rename(directory/FILENAME)
    print("verified",digest.hexdigest(),flush=True)


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("directory", type=Path)
    parser.add_argument("prefix_size", type=int)
    args=parser.parse_args()
    main(args.directory,args.prefix_size)
