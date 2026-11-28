"""Cross-check measured-point annotations against the public ROS connection index."""
import argparse
import csv
import io
import struct
import subprocess
from pathlib import Path

import roslz4
from rosbag import bag as fmt
from bag_assets import records
from evaluate_exp18 import read_reference
from fetch_ranges import ASSETS,BASE


def main(index,ref,folder,out):
    folder.mkdir(exist_ok=True,parents=True)
    with index.open("rb") as f:
        data=list(records(f))
    id_=[fmt._read_uint32_field(h,"conn") for h,_ in data if fmt._read_uint8_field(h,"op")==fmt._OP_CONNECTION
         and fmt._read_str_field(h,"topic")=="/Point_name"]
    if len(id_)!=1:
        raise ValueError("point event topic index missing")
    id_=id_[0]
    idx=[(fmt._read_uint64_field(h,"chunk_pos"),dict(struct.iter_unpack("<II",p))) for h,p in data
          if fmt._read_uint8_field(h,"op")==fmt._OP_CHUNK_INFO]
    asset,total,_=ASSETS["bag"]
    rt,_=read_reference(ref)
    found=[]
    for (lo,counts),(end,_) in zip(idx,idx[1:]+[(8656555208,{})]):
        if id_ not in counts:
            continue
        x=folder/f"event_{lo}.bytes"
        hdr=x.with_suffix(".headers")
        subprocess.run(["curl","-4","--http1.1","--fail","--location","--silent","--show-error","--max-time","30",
                       "--range",f"{lo}-{end-1}","--dump-header",str(hdr),BASE+asset,"-o",str(x)],check=True)
        if f"content-range: bytes {lo}-{end-1}/{total}" not in hdr.read_text().lower() or x.stat().st_size!=end-lo:
            raise ValueError("event chunk identity invalid")
        with x.open("rb") as f:
            h,p=next(records(f))
        assert fmt._read_str_field(h,"compression")=="lz4"
        for inner,body in records(io.BytesIO(roslz4.decompress(p))):
            if fmt._read_uint8_field(inner,"op")!=fmt._OP_MSG_DATA or fmt._read_uint32_field(inner,"conn")!=id_:
                continue
            corrected=fmt._read_time_field(inner,"time").to_nsec()
            n=struct.unpack("<I",body[:4])[0]
            nearest=int(rt[abs(rt-corrected).argmin()])
            found.append(dict(chunk_offset=lo,event_stamp_ns=corrected,marker=body[4:4+n].decode(),
                              nearest_survey_ref_ns=nearest,ref_minus_event_ns=nearest-corrected))
    if len(found)!=3:
        raise ValueError("expected 3 point events")
    with out.open("w") as stream:
        w=csv.DictWriter(stream,list(found[0]),lineterminator="\n")
        w.writeheader();w.writerows(found)


if __name__=="__main__":
    p=argparse.ArgumentParser()
    for name in ("index","ref","folder","out"):
        p.add_argument(name,type=Path)
    a=p.parse_args()
    main(a.index,a.ref,a.folder,a.out)
