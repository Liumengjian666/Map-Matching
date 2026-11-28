"""Read Hilti ROS1 container metadata and extract complete sensor messages."""
import argparse
import io
import json
import struct
from pathlib import Path

import rosbag
from rosbag import bag as fmt
import roslz4

SENSORS={"sensor_msgs/PointCloud2", "sensor_msgs/Imu"}


def records(raw,bounded=False):
    while True:
        header_len=raw.read(4)
        if not header_len:
            return
        if len(header_len)!=4:
            if bounded:
                return
            raise EOFError("truncated ROS header length")
        n=struct.unpack("<I",header_len)[0]
        header=raw.read(n)
        data_len=raw.read(4)
        if len(header)!=n or len(data_len)!=4:
            if bounded:
                return
            raise EOFError("truncated ROS record header")
        size=struct.unpack("<I",data_len)[0]
        data=raw.read(size)
        if len(data)!=size:
            if bounded:
                return
            raise EOFError("truncated ROS record body")
        yield fmt._build_header_from_str(header,None),data


def connections_at_index(path,output):
    cid={}
    chunks=[]
    with path.open("rb") as f:
        for h,payload in records(f):
            op=fmt._read_uint8_field(h,"op")
            if op==fmt._OP_CONNECTION:
                c=fmt._read_uint32_field(h,"conn")
                topic=fmt._read_str_field(h,"topic")
                info=fmt._build_header_from_str(payload,None)
                cid[c]=dict(topic=topic,type=info["type"].decode())
            if op==fmt._OP_CHUNK_INFO:
                x=dict(pos=fmt._read_uint64_field(h,"chunk_pos"),start=fmt._read_time_field(h,"start_time").to_nsec(),
                       end=fmt._read_time_field(h,"end_time").to_nsec(),
                       counts=dict(struct.iter_unpack("<II",payload)))
                chunks.append(x)
    ids={i for i,h in cid.items() if h["type"] in SENSORS}
    count={i:sum(chunk["counts"].get(i,0) for chunk in chunks) for i in cid}
    sizes=[b["pos"]-a["pos"] for a,b in zip(chunks,chunks[1:])]+[8656555208-chunks[-1]["pos"]]
    keep=[bool(ids&set(x["counts"])) for x in chunks]
    output.write_text(json.dumps(dict(connections=cid,count=count,total_chunks=len(chunks),
        sensor_chunks=sum(keep),bytes_of_sensor_chunks=sum(s for s,k in zip(sizes,keep) if k),
        first_record_ns=chunks[0]["start"],last_record_ns=chunks[-1]["end"]),indent=2)+"\n")


def extract_prefix(path,output,seconds):
    conn={}
    types={}
    t0=None
    completed=False
    count={}
    with path.open("rb") as f, rosbag.Bag(str(output),"w") as bag:
        if f.readline()!=b"#ROSBAG V2.0\n":
            raise ValueError("invalid bag magic")
        for h,chunk in records(f,bounded=True):
            if fmt._read_uint8_field(h,"op")!=fmt._OP_CHUNK:
                continue
            compression=fmt._read_str_field(h,"compression")
            if compression=="lz4":
                chunk=roslz4.decompress(chunk)
            elif compression!="none":
                raise ValueError(compression)
            for rh,p in records(io.BytesIO(chunk)):
                op=fmt._read_uint8_field(rh,"op")
                if op==fmt._OP_CONNECTION:
                    i=fmt._read_uint32_field(rh,"conn")
                    topic=fmt._read_str_field(rh,"topic")
                    info=fmt._build_header_from_str(p,None)
                    if info["type"].decode() in SENSORS:
                        cn=fmt._ConnectionInfo(i,topic,info)
                        conn[i]=cn
                        types[i]=fmt._get_message_type(cn)
                if op!=fmt._OP_MSG_DATA:
                    continue
                i=fmt._read_uint32_field(rh,"conn")
                if i not in conn:
                    continue
                t=fmt._read_time_field(rh,"time")
                if t0 is None:
                    t0=t.to_nsec()
                if t.to_nsec()-t0>round(seconds*1e9):
                    completed=True
                    break
                c=conn[i]
                bag.write(c.topic,(c.datatype,p,c.md5sum,types[i]),t=t,raw=True,connection_header=c.header)
                count[c.topic]=count.get(c.topic,0)+1
            if completed:
                break
    print(json.dumps(dict(completed=completed,seconds_requested=seconds,counts=count)))


if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("input",type=Path)
    p.add_argument("output",type=Path)
    p.add_argument("--prefix-seconds",type=float)
    a=p.parse_args()
    if a.prefix_seconds is None:
        connections_at_index(a.input,a.output)
    else:
        extract_prefix(a.input,a.output,a.prefix_seconds)
