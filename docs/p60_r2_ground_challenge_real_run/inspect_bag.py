#!/usr/bin/env python3
"""Read-only ROS1 bag contract inspection; outputs only to a new internal file."""
import argparse
import collections
import hashlib
import json
import os
import time

import numpy as np
import rosbag


def inspect(path):
    result = {"path": os.path.realpath(path), "bytes": os.path.getsize(path)}
    with open(path, "rb") as stream:
        result["magic"] = stream.read(13).decode("ascii")
    with rosbag.Bag(path, "r") as bag:
        result["record_start_sec"] = bag.get_start_time()
        result["record_end_sec"] = bag.get_end_time()
        topics = bag.get_type_and_topic_info().topics
        result["topics"] = {name: {"type": t.msg_type, "count": t.message_count,
                                    "frequency_hz": t.frequency}
                            for name, t in topics.items()}
        sensor_topics = [t for t, v in topics.items()
                         if v.msg_type in ("sensor_msgs/PointCloud2", "sensor_msgs/Imu",
                                           "tf2_msgs/TFMessage", "tf/tfMessage")]
        stamps = collections.defaultdict(list)
        frames = collections.defaultdict(set)
        cloud_records, imu_summary, transforms = [], {}, {}
        for topic, msg, record in bag.read_messages(topics=sensor_topics):
            if hasattr(msg, "transforms"):
                for t in msg.transforms:
                    value = {"parent": t.header.frame_id, "child": t.child_frame_id,
                             "translation": [t.transform.translation.x, t.transform.translation.y,
                                             t.transform.translation.z],
                             "quaternion_xyzw": [t.transform.rotation.x, t.transform.rotation.y,
                                                 t.transform.rotation.z, t.transform.rotation.w]}
                    transforms[json.dumps(value, sort_keys=True)] = value
                continue
            stamps[topic].append(msg.header.stamp.to_nsec())
            frames[topic].add(msg.header.frame_id)
            if hasattr(msg, "fields"):
                fields = [{"name": f.name, "offset": f.offset, "datatype": f.datatype,
                           "count": f.count} for f in msg.fields]
                item = {"scan_index": len(cloud_records) + 1, "topic": topic,
                        "stamp_ns": msg.header.stamp.to_nsec(), "record_ns": record.to_nsec(),
                        "points": msg.width * msg.height, "point_step": msg.point_step,
                        "row_step": msg.row_step, "height": msg.height, "fields": fields,
                        "data_sha256": hashlib.sha256(msg.data).hexdigest()}
                for field in msg.fields:
                    if field.name in ("time", "t", "timestamp") and field.datatype in (6, 7, 8):
                        code = {6: "u4", 7: "f4", 8: "f8"}[field.datatype]
                        dtype = np.dtype({"names": ["v"], "formats": [(">" if msg.is_bigendian else "<") + code],
                                          "offsets": [field.offset], "itemsize": msg.point_step})
                        arr = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                                         strides=(msg.row_step, msg.point_step))["v"]
                        item["point_time"] = {"field": field.name, "min": float(np.min(arr)),
                                              "max": float(np.max(arr)),
                                              "finite": bool(np.isfinite(arr).all())}
                cloud_records.append(item)
            else:
                state = imu_summary.setdefault(topic, {"count": 0, "first_sample": None})
                sample = {"stamp_ns": msg.header.stamp.to_nsec(),
                          "acceleration": [msg.linear_acceleration.x, msg.linear_acceleration.y,
                                           msg.linear_acceleration.z],
                          "angular_velocity": [msg.angular_velocity.x, msg.angular_velocity.y,
                                               msg.angular_velocity.z],
                          "orientation_xyzw": [msg.orientation.x, msg.orientation.y,
                                               msg.orientation.z, msg.orientation.w]}
                state["count"] += 1
                if state["first_sample"] is None:
                    state["first_sample"] = sample
                state["last_sample"] = sample
        result["sensors"] = {}
        for topic, values in stamps.items():
            delta = np.diff(values)
            result["sensors"][topic] = {"count": len(values), "first_stamp_ns": values[0],
                "last_stamp_ns": values[-1], "frames": sorted(frames[topic]),
                "strictly_increasing": bool(np.all(delta > 0)),
                "median_dt_sec": float(np.median(delta) * 1e-9) if len(delta) else None,
                "max_dt_sec": float(np.max(delta) * 1e-9) if len(delta) else None}
        result.update(cloud_records=cloud_records, imu=imu_summary, transforms=list(transforms.values()))
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("bag")
    p.add_argument("output")
    args = p.parse_args()
    started = time.monotonic()
    data = inspect(args.bag)
    data["inspection_wall_sec"] = time.monotonic() - started
    with open(args.output, "x") as stream:
        json.dump(data, stream, indent=2)
    print(json.dumps({k: v for k, v in data.items()
                      if k not in ("cloud_records", "transforms")}, indent=2))
