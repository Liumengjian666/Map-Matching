"""Report verified-length ranges separately from in-flight data."""
import csv
from datetime import datetime
from pathlib import Path
import re
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"p64_r3_q10_public_mechanism_validation"))
from fetch_ranges import ASSETS, parts_for


def summary(folder,key):
    initial,total,usable,parts=parts_for(folder,key)
    final=folder/ASSETS[key][0].split("/")[-1]
    if final.is_file() and final.stat().st_size==total:
        return dict(asset=key,expected=total,received=total,inflight=0,remaining=0,all_bytes_present=True)
    verified=initial.stat().st_size if initial.is_file() else 0
    inflight=0
    for lo,hi in parts:
        path=usable/f"{lo}_{hi}.part"
        have=path.stat().st_size if path.is_file() else 0
        if have>hi-lo+1:
            raise ValueError(f"oversized {path}")
        verified+=have
        body=path.with_suffix(".next")
        hdr=path.with_suffix(".headers")
        try:
            staged=body.stat().st_size
            values=re.findall(r"^content-range:\s*([^\r\n]+)",hdr.read_text(),re.I|re.M)
            if values and values[-1].strip()==f"bytes {lo+have}-{hi}/{total}":
                inflight+=min(staged,hi-(lo+have)+1)
        except FileNotFoundError:
            pass
    return dict(asset=key,expected=total,received=verified,inflight=inflight,remaining=total-verified-inflight,all_bytes_present=False)


def main(folder, output):
    now=datetime.now().astimezone().isoformat()
    rows=[dict(at=now,**summary(folder,key)) for key in ("map","bag")]
    if output is not None:
        header=not output.exists()
        with output.open("a") as f:
            writer=csv.DictWriter(f,list(rows[0]),lineterminator="\n")
            if header:
                writer.writeheader()
            writer.writerows(rows)
    for r in rows:
        print(f"{r['asset']} {r['received']+r['inflight']:,}/{r['expected']:,} bytes remaining={r['remaining']:,} complete={r['all_bytes_present']}")


if __name__=="__main__":
    main(Path(sys.argv[1]),Path(sys.argv[2]) if len(sys.argv)>2 else None)
