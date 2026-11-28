"""Place official prefix and XML pages in a separate sparse qualification file."""
import argparse
import shutil
import struct
from pathlib import Path


def make_preview(prefix,tail,offset,out):
    with prefix.open("rb") as data:
        fields=struct.unpack("<8sIIQQQQ",data.read(48))
        magic,_,_,length,xml,_,page=fields
        if magic!=b"ASTM-E57" or xml<offset or offset%page or offset+tail.stat().st_size!=length:
            raise ValueError("E57 header or independently downloaded XML-tail range mismatch")
        n=prefix.stat().st_size//page*page
        if not 0<n<offset:
            raise ValueError("invalid partial prefix")
        data.seek(0)
        with out.open("xb") as f:
            remaining=n
            while remaining:
                b=data.read(min(remaining,2**20))
                if not b:
                    raise EOFError("prefix read incomplete")
                f.write(b)
                remaining-=len(b)
            f.seek(offset)
            with tail.open("rb") as source:
                shutil.copyfileobj(source,f)
    print(f"verified range placement prefix_pages={n//page} prefix_bytes={n} tail_offset={offset} original_size={length}")


if __name__=="__main__":
    p=argparse.ArgumentParser()
    for name in ("prefix","tail","out"):
        p.add_argument(name,type=Path)
    p.add_argument("--offset",type=int,default=5447122944)
    a=p.parse_args()
    make_preview(a.prefix,a.tail,a.offset,a.out)
