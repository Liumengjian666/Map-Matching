"""Recover a bounded, complete set of sensor messages from a public ZIP prefix."""
import argparse
import bz2
import io
import struct
import time
import zlib
from pathlib import Path

import rosbag
from rosbag import bag as fmt
try:
    import roslz4
except ImportError:
    roslz4=None

from range_fetch import FILENAME

LIDAR="/velodyne_packets"
IMU="/imu/data"


def compressed_bytes(folder, wait):
    # Continue in byte order, never skip a hole or a partially downloaded range.
    segments=[(folder/(FILENAME+".part"),None)]
    plan=(folder/"range_plan.tsv").read_text().splitlines()
    prefix=int(plan[0].split()[0])
    segments[0]=(segments[0][0],prefix)
    for line in plan:
        lo,hi=(int(x) for x in line.split())
        segments.append((folder/f"range_{lo}_{hi}.part",hi-lo+1))
    for path,length in segments:
        while not path.exists():
            if not wait:
                return
            time.sleep(3)
        with path.open("rb") as file:
            remaining=length
            while remaining>0:
                data=file.read(min(remaining,512*1024))
                if data:
                    remaining-=len(data)
                    yield data
                elif not wait:
                    return
                else:
                    time.sleep(3)


def member_bytes(folder, wait):
    deflated=None
    for part in compressed_bytes(folder,wait):
        if deflated is None:
            # Historic archive identity uses one initial directory entry and one Zip64 bag member.
            f=io.BytesIO(part)
            for expect in (b"Long_Corridor_Rosbag/",b"Long_Corridor_Rosbag/raw_data_core_2023-07-25-03-01-44.bag"):
                h=struct.unpack("<4s5H3I2H",f.read(30))
                assert h[0]==b"PK\x03\x04" and f.read(h[-2])==expect
                f.read(h[-1])
            assert h[3]==8
            deflated=zlib.decompressobj(-15)
            part=f.read()
        yield deflated.decompress(part)
        if deflated.eof:
            return


def records(blob):
    while blob.tell()<len(blob.getbuffer()):
        header=fmt._read_header(blob)
        data=fmt._read_sized(blob)
        yield header,data


def extract(folder,out,seconds,wait):
    buffer=bytearray()
    first_record=True
    recognized=dict()
    classes=dict()
    first=None
    emitted=0
    latest_ns=0
    with rosbag.Bag(str(out),"w") as dst:
        for payload in member_bytes(folder,wait):
            buffer.extend(payload)
            index=0
            if first_record:
                if len(buffer)<13:
                    continue
                assert buffer[:13]==b"#ROSBAG V2.0\n"
                index=13
                first_record=False
            while len(buffer)-index>=8:
                header_len=struct.unpack_from("<I",buffer,index)[0]
                mid=index+4+header_len
                if len(buffer)<mid+4:
                    break
                data_length=struct.unpack_from("<I",buffer,mid)[0]
                end=mid+4+data_length
                if len(buffer)<end:
                    break
                hdr=fmt._build_header_from_str(bytes(buffer[index+4:mid]),None)
                opcode=fmt._read_uint8_field(hdr,"op")
                if opcode==fmt._OP_CHUNK:
                    tag=fmt._read_str_field(hdr,"compression")
                    data=bytes(buffer[mid+4:end])
                    if tag=="bz2":
                        data=bz2.decompress(data)
                    elif tag=="lz4":
                        data=roslz4.decompress(data)
                    elif tag!="none":
                        raise ValueError(f"Unsupported compression {tag}")
                    for rh,body in records(io.BytesIO(data)):
                        op=fmt._read_uint8_field(rh,"op")
                        if op==fmt._OP_CONNECTION:
                            conn=fmt._read_uint32_field(rh,"conn")
                            topic=fmt._read_str_field(rh,"topic")
                            if topic in (LIDAR,IMU):
                                connection=fmt._ConnectionInfo(conn,topic,fmt._build_header_from_str(body,None))
                                recognized[conn]=connection
                                classes[conn]=fmt._get_message_type(connection)
                        elif op==fmt._OP_MSG_DATA:
                            conn=fmt._read_uint32_field(rh,"conn")
                            if conn not in recognized:
                                continue
                            record=fmt._read_time_field(rh,"time")
                            time_offset=4 if recognized[conn].topic==LIDAR else 4
                            ns,low=struct.unpack_from("<II",body,time_offset)
                            stamp=ns*10**9+low
                            if first is None and recognized[conn].topic==LIDAR:
                                first=stamp
                            if first is not None and stamp>first+round(seconds*1e9):
                                print(f"complete_sensor_prefix elapsed_s={(latest_ns-first)*1e-9:.6f} messages={emitted}",flush=True)
                                return
                            conn_info=recognized[conn]
                            dst.write(conn_info.topic,(conn_info.datatype,body,conn_info.md5sum,classes[conn]),
                                      t=record,raw=True,connection_header=conn_info.header)
                            latest_ns=max(stamp,latest_ns)
                            emitted+=1
                index=end
            del buffer[:index]
    print(f"partial_sensor_prefix elapsed_s={(latest_ns-first)*1e-9 if first else 0:.6f} messages={emitted}",flush=True)


if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("asset_folder",type=Path)
    p.add_argument("bag",type=Path)
    p.add_argument("seconds",type=float)
    p.add_argument("--wait",action="store_true")
    a=p.parse_args()
    extract(a.asset_folder,a.bag,a.seconds,a.wait)
