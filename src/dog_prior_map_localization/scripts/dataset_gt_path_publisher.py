#!/usr/bin/env python3
"""Clock-following display-only GT. No TF or estimator observation is published."""
import bisect
from decimal import Decimal
import hashlib
import json
from pathlib import Path
import threading

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from tf.transformations import quaternion_from_matrix, quaternion_matrix
from visualization_msgs.msg import Marker


def transform(config):
    result = np.eye(4)
    result[:3, :3] = np.asarray(config['rotation_row_major'], dtype=float).reshape(3, 3)
    result[:3, 3] = config['translation_m']
    rotation = result[:3, :3]
    if (not np.isfinite(result).all() or
            not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-9, rtol=0) or
            not np.isclose(np.linalg.det(rotation), 1, atol=1e-9, rtol=0)):
        raise ValueError('Invalid configured SE(3)')
    return result


def load_samples(filename, expected_sha, t_gt_map, t_imu_lidar):
    payload = Path(filename).read_bytes()
    if hashlib.sha256(payload).hexdigest() != expected_sha:
        raise ValueError('GT SHA256 mismatch')
    t_map_gt = np.linalg.inv(t_gt_map)
    samples = []
    for line in payload.decode('utf-8').splitlines():
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        fields = line.split()
        if len(fields) != 8:
            raise ValueError('Expected TUM timestamp tx ty tz qx qy qz qw')
        ns_decimal = Decimal(fields[0]) * 1000000000
        if not ns_decimal.is_finite() or ns_decimal != ns_decimal.to_integral_value():
            raise ValueError('GT timestamp is not integer nanoseconds')
        stamp_ns = int(ns_decimal)
        values = np.asarray([float(value) for value in fields[1:]])
        if (stamp_ns <= 0 or not np.isfinite(values).all() or
                np.linalg.norm(values[3:]) < 1e-12 or
                (samples and stamp_ns <= samples[-1][0])):
            raise ValueError('Invalid or unordered GT pose')
        t_gt_imu = quaternion_matrix(values[3:] / np.linalg.norm(values[3:]))
        t_gt_imu[:3, 3] = values[:3]
        t_map_lidar = t_map_gt @ t_gt_imu @ t_imu_lidar
        quaternion = quaternion_from_matrix(t_map_lidar)
        samples.append((stamp_ns, t_map_lidar[:3, 3].copy(), quaternion))
    if not samples:
        raise ValueError('Empty GT')
    return samples


class ClockCursor:
    """Track only past samples; reset even on a rewind within the same sample."""
    def __init__(self, stamps):
        self.stamps = stamps
        self.last_clock = None
        self.index = -1

    def advance(self, now_ns):
        rewound = self.last_clock is not None and now_ns < self.last_clock
        previous = self.index
        self.index = bisect.bisect_right(self.stamps, now_ns) - 1
        self.last_clock = now_ns
        return self.index, rewound, rewound or self.index != previous


def make_pose(sample, frame):
    stamp_ns, position, quaternion = sample
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = rospy.Time(stamp_ns // 1000000000, stamp_ns % 1000000000)
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
    (pose.pose.orientation.x, pose.pose.orientation.y,
     pose.pose.orientation.z, pose.pose.orientation.w) = quaternion
    return pose


class GroundTruthPublisher:
    def __init__(self):
        self.frame = rospy.get_param('~frame_id')
        mode = rospy.get_param('~alignment_mode')
        if mode == 'FROZEN_PREFIX_ALIGNED':
            if self.frame != 'camera_init':
                raise ValueError('Frozen Corridor01 alignment requires camera_init')
            alignment = transform(rospy.get_param('~T_gt_map'))
            extrinsic = transform(rospy.get_param('~T_imu_lidar'))
        elif mode == 'NATIVE_GT_WORLD':
            if self.frame == 'camera_init':
                raise ValueError('Unaligned native GT must not be labelled as the localization map')
            alignment, extrinsic = np.eye(4), np.eye(4)
        else:
            raise ValueError('Unsupported GT display alignment mode')
        self.samples = load_samples(rospy.get_param('~gt_file'), rospy.get_param('~gt_sha256'),
                                    alignment, extrinsic)
        self.poses = [make_pose(sample, self.frame) for sample in self.samples]
        self.cursor = ClockCursor([sample[0] for sample in self.samples])
        self.path = RosPath()
        self.path.header.frame_id = self.frame
        self.lock = threading.Lock()
        path_topic = rospy.get_param('~path_topic')
        marker_topic = rospy.get_param('~marker_topic')
        # An accidental remap to an estimator input must not turn GT into an observation.
        if (rospy.resolve_name(path_topic) != '/dog_livo/gt_path' or
                rospy.resolve_name(marker_topic) != '/dog_livo/gt_current_pose'):
            raise ValueError('GT publishers are restricted to display-only topics')
        self.path_pub = rospy.Publisher(path_topic, RosPath, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher(marker_topic, Marker, queue_size=1, latch=True)
        self.info_pub = rospy.Publisher('~info', String, queue_size=1, latch=True)
        self.info_pub.publish(String(data=json.dumps({
            'contract': rospy.get_param('~alignment_contract'),
            'alignment_source': rospy.get_param('~alignment_source'),
            'alignment_source_sha256': rospy.get_param('~alignment_source_sha256'),
            'reference_point': rospy.get_param('~reference_point'),
            'samples': len(self.samples), 'first_ns': self.samples[0][0],
            'last_ns': self.samples[-1][0], 'no_tf_or_estimator_output': True})))
        self.publish_empty()
        self.subscriber = rospy.Subscriber('/clock', Clock, self.on_clock, queue_size=1)
        rospy.logwarn('GT VISUALIZATION ONLY: %s, %d samples, no estimator input or TF.',
                      rospy.get_param('~alignment_contract'), len(self.samples))

    def publish_empty(self):
        self.path.header.stamp = rospy.Time(0)
        self.path.poses = []
        self.path_pub.publish(self.path)
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.ns = 'ground_truth_display'
        marker.id = 0
        marker.action = Marker.DELETE
        self.marker_pub.publish(marker)

    def on_clock(self, message):
        with self.lock:
            index, rewound, changed = self.cursor.advance(message.clock.to_nsec())
            if not changed:
                return
            if rewound:
                self.publish_empty()
            if index < 0:
                return
            # No future sample or interpolated/extrapolated pose is emitted.
            self.path.poses = self.poses[:index + 1]
            self.path.header.stamp = self.poses[index].header.stamp
            self.path_pub.publish(self.path)
            marker = Marker()
            marker.header = self.poses[index].header
            marker.ns = 'ground_truth_display'
            marker.id = 0
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = self.poses[index].pose
            marker.scale.x, marker.scale.y, marker.scale.z = 1.0, 0.15, 0.15
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0, 1, 1, 1
            self.marker_pub.publish(marker)


if __name__ == '__main__':
    rospy.init_node('dataset_gt_visualizer')
    try:
        if rospy.get_param('~enabled', False):
            node = GroundTruthPublisher()
        else:
            rospy.loginfo('GT display disabled in YAML: no GT file loaded or display topic advertised.')
        rospy.spin()
    except (OSError, ValueError, KeyError) as error:
        rospy.logfatal('GT visualization refused: %s', error)
        raise SystemExit(1)
