"""Audit the downloaded original ROS1 bag and extract its untouched maiden scan."""
import argparse
import json
from pathlib import Path
import sys

import rosbag

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"p64_r3_q10_public_mechanism_validation"))
from prepare_exp18 import decode, pcd, read_anchor

BAG_BYTES=8_657_654_860
EXPECTED={"/hesai/pandar":1094,"/alphasense/imu":43656,"/Point_name":3}


def audit(full_bag, reference, out):
    if full_bag.stat().st_size!=BAG_BYTES:
        raise ValueError("expected full official bag")
    out.mkdir(exist_ok=True)
    epoch,_=read_anchor(reference)
    events=[]
    first,last,count,post_anchor=None,None,0,0
    max_query_ns=None
    last_imu=None
    with rosbag.Bag(str(full_bag)) as bag:
        topics=bag.get_type_and_topic_info().topics
        actual={k:topics[k].message_count for k in EXPECTED}
        if actual!=EXPECTED:
            raise ValueError(f"official index mismatch {actual}")
        for topic,msg,rt in bag.read_messages(topics=EXPECTED):
            if topic=="/hesai/pandar":
                ns=msg.header.stamp.to_nsec()
                if last is not None and not ns>last:
                    raise ValueError(f"non-monotonic scan timestamp {ns}")
                if first is None:
                    first=ns
                last=ns
                count+=1
                if ns>=epoch:
                    post_anchor+=1
                    xyz,stamp,_=decode(msg)
                    max_query_ns=int(stamp.max())
                    if ns==epoch:
                        pcd(out/"raw_scan_0000.pcd",xyz)
            elif topic=="/alphasense/imu":
                last_imu=msg.header.stamp.to_nsec()
            else:
                events.append(dict(name=msg.data,stamp_ns=rt.to_nsec()))
        record={
            "bag_bytes":full_bag.stat().st_size,
            "indexed_messages":actual,"lidar_records_decoded":count,
            "lidar_first_ns":first,"lidar_last_ns":last,
            "raw_first_scan_available":(out/"raw_scan_0000.pcd").exists(),
            "last_scan_final_point_ns":max_query_ns,
            "imu_last_ns":last_imu,"record_start_s":bag.get_start_time(),
            "record_end_s":bag.get_end_time(),
            "scans_after_anchor":post_anchor,"survey_events":events,
        }
    if count!=actual["/hesai/pandar"] or len(events)!=EXPECTED["/Point_name"]:
        raise ValueError("incomplete bag iteration")
    record["causally_deskewable_after_anchor"]=post_anchor-int(last_imu<max_query_ns)
    (out/"asset_info.json").write_text(json.dumps(record,indent=2)+"\n")


if __name__=="__main__":
    p=argparse.ArgumentParser()
    for name in ("bag","reference","output"):
        p.add_argument(name,type=Path)
    a=p.parse_args()
    audit(a.bag,a.reference,a.output)
