#!/usr/bin/env python3
"""Bounded diagnostic recorder; NDT Source is captured by the existing C++ class.

These are the eight full, ordered *pre-filter* deskew messages. No poses are
reconstructed and nothing is republished into the localization system.
"""
import argparse
import json
from pathlib import Path
import rosbag
import rospy
from sensor_msgs.msg import PointCloud2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--requests', required=True)
    args = parser.parse_args()
    selected = {int(item.split(':')[1]): int(item.split(':')[0])
                for item in json.loads(args.requests)}
    if len(selected) != 8:
        raise ValueError('P29_EXACTLY_EIGHT_DESKEW_REQUESTS_REQUIRED')
    path = args.output / 'deskew_selected.bag'
    if path.exists():
        raise ValueError('P29_DESKEW_BAG_ALREADY_EXISTS')
    records = []
    seen = set()
    rospy.init_node('p29_selected_deskew_observer', anonymous=False)
    with rosbag.Bag(str(path), 'w') as bag:
        def callback(message):
            ns = message.header.stamp.to_nsec()
            if ns not in selected:
                return
            if ns in seen:
                rospy.logfatal('P29 duplicate selected deskew message')
                rospy.signal_shutdown('DUPLICATE_SELECTED_DESKEW')
                return
            seen.add(ns)
            bag.write('/p10/corridor01/deskewed_points', message, message.header.stamp)
            records.append(dict(frame_index=selected[ns], scan_ns=str(ns),
                                width=message.width, height=message.height,
                                bytes=len(message.data), is_dense=message.is_dense))
        subscriber = rospy.Subscriber('/p10/corridor01/deskewed_points', PointCloud2,
                                      callback, queue_size=8)
        (args.output / 'deskew_capture.ready').write_text('READY\n')
        try:
            rospy.spin()
        finally:
            subscriber.unregister()
    (args.output / 'deskew_capture.json').write_text(json.dumps(
        dict(captured=len(records), records=records, GT_LOADED=False), indent=2)+'\n')


if __name__ == '__main__':
    main()
