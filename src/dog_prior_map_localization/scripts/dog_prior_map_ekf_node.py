#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器狗端低算力先验地图定位节点。

设计目标
========
1. FAST-LIVO2 只负责离线建图，得到一张先验点云地图。
2. 机器狗实时运行本节点，不依赖“外部 FAST-LIVO2 location 后处理”。
3. IMU 高频传播状态，输出高频位姿。
4. LiDAR 当前帧低频与先验地图匹配，作为 EKF/IESKF 的观测更新。
5. 相机当前版本先作为视觉质量门控：曝光/纹理不好时降低视觉相关约束权重。

状态量说明
==========
为了保持机器狗端低算力，本原型使用 15 维误差状态：
  p  : 位置，世界/map 坐标系，3 维
  v  : 速度，世界/map 坐标系，3 维
  R  : 姿态，SO(3)，表示 body 到 map 的旋转
  ba : 加速度计零偏，3 维
  bg : 陀螺仪零偏，3 维
误差状态顺序为：
  [delta_p, delta_v, delta_theta, delta_ba, delta_bg]

注意
====
这是“机器狗部署方案”的清晰原型版，强调结构正确、参数可控、低算力。
后续若要上真实机器狗长时间运行，建议把 KDTree 匹配与 EKF 更新迁移到 C++。
"""
import math
import os
import struct
import threading
from collections import deque
from pathlib import Path as FsPath

import cv2
import numpy as np
import rospy
import tf
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import Imu, Image, PointCloud2
from sensor_msgs import point_cloud2
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as SciRot
from std_msgs.msg import Header

try:
    # livox_ros_driver2 的 CustomMsg 只有在工作空间 source 后才可导入。
    from livox_ros_driver2.msg import CustomMsg as LivoxCustomMsg
except Exception:  # pragma: no cover
    LivoxCustomMsg = None


def skew(vec):
    """把三维向量变成反对称矩阵，用于叉乘线性化。"""
    x, y, z = vec
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=np.float64)


def rpy_deg_to_rot(roll_deg, pitch_deg, yaw_deg):
    """欧拉角转旋转矩阵，输入单位为度，顺序 xyz。"""
    return SciRot.from_euler("xyz", [roll_deg, pitch_deg, yaw_deg], degrees=True).as_matrix()


def rot_to_quat_xyzw(rot):
    """旋转矩阵转四元数，返回 ROS 使用的 x/y/z/w 顺序。"""
    return SciRot.from_matrix(rot).as_quat()


def limit_vector(vec, max_norm):
    """限制向量模长，防止一次错误匹配把滤波器拉飞。"""
    norm = float(np.linalg.norm(vec))
    if max_norm > 0.0 and norm > max_norm:
        return vec * (max_norm / max(norm, 1e-12))
    return vec


class DogPriorMapEkfNode:
    """机器狗端先验地图定位主类。"""

    def __init__(self):
        """读取 ROS 参数，初始化滤波状态、地图、订阅器和发布器。"""
        # ------------------------- 参数读取 -------------------------
        self.map_frame = rospy.get_param("~frames/map_frame", rospy.get_param("frames/map_frame", "map"))
        self.odom_frame = rospy.get_param("~frames/odom_frame", rospy.get_param("frames/odom_frame", "odom"))
        self.base_frame = rospy.get_param("~frames/base_frame", rospy.get_param("frames/base_frame", "base_link"))

        self.imu_topic = rospy.get_param("topics/imu", "/livox/imu")
        self.lidar_topic = rospy.get_param("topics/lidar", "/livox/lidar")
        self.image_topic = rospy.get_param("topics/image", "/image_left/image_rect")
        self.initial_pose_topic = rospy.get_param("topics/initial_pose", "/initialpose")
        self.odom_high_rate_topic = rospy.get_param("topics/odom_high_rate", "/dog_livo/odom_high_rate")
        self.odom_corrected_topic = rospy.get_param("topics/odom_corrected", "/dog_livo/odom_corrected")
        self.path_high_rate_topic = rospy.get_param("topics/path_high_rate", "/dog_livo/path_high_rate")
        self.path_corrected_topic = rospy.get_param("topics/path_corrected", "/dog_livo/path_corrected")

        self.gravity_norm = float(rospy.get_param("imu/gravity", 9.80665))
        self.max_imu_dt = float(rospy.get_param("imu/max_dt", 0.05))
        self.publish_high_rate = bool(rospy.get_param("imu/publish_high_rate", True))

        init_p = rospy.get_param("filter/initial_position", [0.0, 0.0, 0.0])
        init_rpy = rospy.get_param("filter/initial_rpy_deg", [0.0, 0.0, 0.0])
        init_v = rospy.get_param("filter/initial_velocity", [0.0, 0.0, 0.0])

        self.p = np.array(init_p, dtype=np.float64)
        self.v = np.array(init_v, dtype=np.float64)
        self.R = rpy_deg_to_rot(init_rpy[0], init_rpy[1], init_rpy[2])
        self.ba = np.zeros(3, dtype=np.float64)
        self.bg = np.zeros(3, dtype=np.float64)
        self.g = np.array([0.0, 0.0, -self.gravity_norm], dtype=np.float64)

        init_pos_std = float(rospy.get_param("filter/init_pos_std", 0.5))
        init_rot_std = math.radians(float(rospy.get_param("filter/init_rot_std_deg", 5.0)))
        init_vel_std = float(rospy.get_param("filter/init_vel_std", 0.5))
        init_bias_std = float(rospy.get_param("filter/init_bias_std", 0.05))
        self.P = np.diag(
            [init_pos_std ** 2] * 3
            + [init_vel_std ** 2] * 3
            + [init_rot_std ** 2] * 3
            + [init_bias_std ** 2] * 6
        ).astype(np.float64)

        self.acc_noise = float(rospy.get_param("imu/acc_noise", 2.0))
        self.gyro_noise = float(rospy.get_param("imu/gyro_noise", 0.1))
        self.acc_bias_noise = float(rospy.get_param("imu/acc_bias_noise", 0.0001))
        self.gyro_bias_noise = float(rospy.get_param("imu/gyro_bias_noise", 0.0001))

        self.lidar_enable = bool(rospy.get_param("lidar_update/enable", True))
        self.update_every_n_scans = int(rospy.get_param("lidar_update/update_every_n_scans", 1))
        self.scan_voxel_size = float(rospy.get_param("lidar_update/scan_voxel_size", 0.25))
        self.max_scan_points = int(rospy.get_param("lidar_update/max_scan_points", 2500))
        self.max_match_distance = float(rospy.get_param("lidar_update/max_match_distance", 1.0))
        self.min_effective_points = int(rospy.get_param("lidar_update/min_effective_points", 80))
        self.point_noise = float(rospy.get_param("lidar_update/point_to_point_noise", 0.20))
        self.huber_threshold = float(rospy.get_param("lidar_update/huber_threshold", 0.6))
        self.max_iterations = int(rospy.get_param("lidar_update/max_iterations", 3))
        self.max_translation_update = float(rospy.get_param("lidar_update/max_translation_update", 0.25))
        self.max_rotation_update = math.radians(float(rospy.get_param("lidar_update/max_rotation_update_deg", 3.0)))

        self.camera_enable = bool(rospy.get_param("camera_update/enable", True))
        self.min_gradient_mean = float(rospy.get_param("camera_update/min_gradient_mean", 2.5))
        self.max_over_exposure_ratio = float(rospy.get_param("camera_update/max_over_exposure_ratio", 0.25))
        self.max_under_exposure_ratio = float(rospy.get_param("camera_update/max_under_exposure_ratio", 0.35))
        self.good_image_weight_scale = float(rospy.get_param("camera_update/good_image_weight_scale", 1.0))
        self.bad_image_weight_scale = float(rospy.get_param("camera_update/bad_image_weight_scale", 0.4))
        self.visual_weight_scale = 1.0

        self.path_max_length = int(rospy.get_param("output/path_max_length", 5000))
        self.publish_tf = bool(rospy.get_param("output/publish_tf", True))
        self.print_debug = bool(rospy.get_param("output/print_debug", True))

        self.T_base_lidar = np.array(rospy.get_param("extrinsic/T_base_lidar", [0, 0, 0]), dtype=np.float64)
        self.R_base_lidar = np.array(rospy.get_param("extrinsic/R_base_lidar", [1,0,0,0,1,0,0,0,1]), dtype=np.float64).reshape(3, 3)

        # ------------------------- 地图加载 -------------------------
        self.map_points = self.load_prior_map()
        self.map_tree = cKDTree(self.map_points)
        rospy.loginfo("[DogPriorMap] 先验地图加载完成: %d points", len(self.map_points))

        # ------------------------- ROS 通信 -------------------------
        self.lock = threading.RLock()
        self.last_imu_time = None
        self.scan_count = 0
        self.bridge = CvBridge()

        self.pub_high = rospy.Publisher(self.odom_high_rate_topic, Odometry, queue_size=50)
        self.pub_corr = rospy.Publisher(self.odom_corrected_topic, Odometry, queue_size=20)
        self.pub_path_high = rospy.Publisher(self.path_high_rate_topic, RosPath, queue_size=5)
        self.pub_path_corr = rospy.Publisher(self.path_corrected_topic, RosPath, queue_size=5)
        self.path_high = RosPath(header=Header(frame_id=self.map_frame))
        self.path_corr = RosPath(header=Header(frame_id=self.map_frame))
        self.tf_broadcaster = tf.TransformBroadcaster()

        rospy.Subscriber(self.imu_topic, Imu, self.imu_callback, queue_size=200)
        rospy.Subscriber(self.initial_pose_topic, PoseWithCovarianceStamped, self.initial_pose_callback, queue_size=2)
        if self.lidar_enable:
            if LivoxCustomMsg is not None:
                rospy.Subscriber(self.lidar_topic, LivoxCustomMsg, self.livox_callback, queue_size=5)
            rospy.Subscriber(self.lidar_topic, PointCloud2, self.pointcloud2_callback, queue_size=5)
        if self.camera_enable:
            rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=2)

    def load_prior_map(self):
        """加载先验地图，优先 NPZ，失败时提示用户先转换 PCD。"""
        npz_path = FsPath(rospy.get_param("map/npz_path", "")).expanduser()
        if npz_path.exists():
            data = np.load(npz_path)
            points = data["points"].astype(np.float32)
            return points[np.isfinite(points).all(axis=1)]
        fallback = rospy.get_param("map/pcd_fallback_path", "")
        raise RuntimeError(
            "找不到 NPZ 先验地图。请先运行 pcd_to_npz_map.py 转换地图。\n"
            f"期望 NPZ: {npz_path}\n"
            f"PCD 备选路径: {fallback}"
        )

    def imu_callback(self, msg):
        """IMU 高频传播：每来一帧 IMU，就预测一次状态并发布高频位姿。"""
        with self.lock:
            t = msg.header.stamp.to_sec()
            if self.last_imu_time is None:
                self.last_imu_time = t
                return
            dt = t - self.last_imu_time
            self.last_imu_time = t
            if dt <= 0.0 or dt > self.max_imu_dt:
                return

            acc_m = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z], dtype=np.float64)
            gyr_m = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z], dtype=np.float64)
            self.propagate_imu(acc_m, gyr_m, dt)
            if self.publish_high_rate:
                self.publish_state(msg.header.stamp, corrected=False)

    def propagate_imu(self, acc_m, gyr_m, dt):
        """误差状态 EKF 的 IMU 预测。

        名义状态：
          R <- R * Exp((gyro-bg)dt)
          v <- v + (R(acc-ba)+g)dt
          p <- p + vdt + 0.5*a*dt^2
        协方差：使用简化连续模型离散化，足够做机器狗端原型验证。
        """
        acc = acc_m - self.ba
        gyr = gyr_m - self.bg
        dR = SciRot.from_rotvec(gyr * dt).as_matrix()
        acc_world = self.R @ acc + self.g
        self.p = self.p + self.v * dt + 0.5 * acc_world * dt * dt
        self.v = self.v + acc_world * dt
        self.R = self.R @ dR

        F = np.eye(15)
        F[0:3, 3:6] = np.eye(3) * dt
        F[3:6, 6:9] = -self.R @ skew(acc) * dt
        F[3:6, 9:12] = -self.R * dt
        F[6:9, 12:15] = -np.eye(3) * dt

        Q = np.zeros((15, 15))
        Q[3:6, 3:6] = np.eye(3) * (self.acc_noise ** 2) * dt * dt
        Q[6:9, 6:9] = np.eye(3) * (self.gyro_noise ** 2) * dt * dt
        Q[9:12, 9:12] = np.eye(3) * (self.acc_bias_noise ** 2) * dt
        Q[12:15, 12:15] = np.eye(3) * (self.gyro_bias_noise ** 2) * dt
        self.P = F @ self.P @ F.T + Q

    def livox_callback(self, msg):
        """处理 Livox CustomMsg 点云。"""
        points = []
        for p in msg.points:
            points.append([p.x, p.y, p.z])
        self.handle_lidar_points(np.asarray(points, dtype=np.float32), msg.header.stamp)

    def pointcloud2_callback(self, msg):
        """处理标准 PointCloud2 点云。"""
        points = []
        for x, y, z in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([x, y, z])
        self.handle_lidar_points(np.asarray(points, dtype=np.float32), msg.header.stamp)

    def handle_lidar_points(self, lidar_points, stamp):
        """LiDAR 低频更新入口。"""
        if lidar_points.size == 0:
            return
        self.scan_count += 1
        if self.scan_count % max(1, self.update_every_n_scans) != 0:
            return
        with self.lock:
            scan_body = self.preprocess_scan(lidar_points)
            if len(scan_body) < self.min_effective_points:
                return
            ok, used, mean_res = self.lidar_map_update(scan_body)
            if ok:
                self.publish_state(stamp, corrected=True)
                if self.print_debug:
                    rospy.loginfo_throttle(1.0, "[DogPriorMap] LiDAR-map EKF update: used=%d mean_res=%.3f", used, mean_res)

    def preprocess_scan(self, lidar_points):
        """当前帧点云预处理：外参变到 base、去除近处点、体素降采样、限制点数。"""
        finite = lidar_points[np.isfinite(lidar_points).all(axis=1)]
        if len(finite) == 0:
            return finite
        # 雷达坐标转机体系。若外参为单位阵，则不改变点。
        body = (self.R_base_lidar @ finite.T).T + self.T_base_lidar
        dist = np.linalg.norm(body, axis=1)
        body = body[(dist > 0.5) & (dist < 80.0)]
        if len(body) == 0:
            return body
        body = self.voxel_downsample(body, self.scan_voxel_size)
        if len(body) > self.max_scan_points:
            idx = np.linspace(0, len(body) - 1, self.max_scan_points).astype(np.int64)
            body = body[idx]
        return body.astype(np.float64)

    @staticmethod
    def voxel_downsample(points, voxel):
        """体素降采样，每个体素保留第一个点。"""
        if voxel <= 0.0 or len(points) == 0:
            return points
        keys = np.floor(points / voxel).astype(np.int64)
        _, idx = np.unique(keys, axis=0, return_index=True)
        return points[np.sort(idx)]

    def lidar_map_update(self, scan_body):
        """用当前 LiDAR 帧和先验地图做低频 EKF 更新。

        当前实现采用点到点最近邻残差：
          r_i = map_nn_i - (R * p_body_i + p)
        线性化：
          r_i ≈ H_i * dx + noise
          H_i = [I, 0, -R*skew(p_body_i), 0, 0]
        为了低算力，使用少量迭代，每次只解 6 自由度误差并反馈到 15 维状态。
        """
        total_used = 0
        total_residual = 0.0
        for _ in range(max(1, self.max_iterations)):
            scan_map = (self.R @ scan_body.T).T + self.p
            # 只查询最近邻，先验地图 KDTree 是启动时一次性构建的。
            dist, idx = self.map_tree.query(scan_map, k=1, workers=1)
            mask = dist < self.max_match_distance
            if np.count_nonzero(mask) < self.min_effective_points:
                return False, int(np.count_nonzero(mask)), float(np.mean(dist)) if len(dist) else 0.0

            src = scan_body[mask]
            pred = scan_map[mask]
            target = self.map_points[idx[mask]].astype(np.float64)
            residual = target - pred
            residual_norm = np.linalg.norm(residual, axis=1)

            # Huber 权重：大残差降低权重，避免动态物体/错误匹配影响过大。
            weight = np.ones_like(residual_norm)
            large = residual_norm > self.huber_threshold
            weight[large] = self.huber_threshold / np.maximum(residual_norm[large], 1e-6)
            weight *= self.visual_weight_scale

            H = np.zeros((len(src) * 3, 15), dtype=np.float64)
            r = residual.reshape(-1, 1)
            for i, p_body in enumerate(src):
                row = slice(3 * i, 3 * i + 3)
                H[row, 0:3] = np.eye(3)
                H[row, 6:9] = -self.R @ skew(p_body)
                # 把鲁棒权重并入 H/r，相当于加权最小二乘。
                scale = math.sqrt(float(weight[i]))
                H[row, :] *= scale
                r[row, 0] *= scale

            R_meas = np.eye(H.shape[0]) * (self.point_noise ** 2)
            S = H @ self.P @ H.T + R_meas
            K = self.P @ H.T @ np.linalg.pinv(S)
            dx = (K @ r).reshape(-1)

            # 保护：限制单次平移和旋转修正，防止错误匹配把状态拉飞。
            dx[0:3] = limit_vector(dx[0:3], self.max_translation_update)
            dx[6:9] = limit_vector(dx[6:9], self.max_rotation_update)
            self.apply_error_state(dx)
            self.P = (np.eye(15) - K @ H) @ self.P

            total_used = int(np.count_nonzero(mask))
            total_residual = float(np.mean(residual_norm)) if len(residual_norm) else 0.0
            if np.linalg.norm(dx[0:3]) < 0.01 and np.linalg.norm(dx[6:9]) < math.radians(0.2):
                break
        return True, total_used, total_residual

    def apply_error_state(self, dx):
        """把误差状态反馈到名义状态。"""
        self.p += dx[0:3]
        self.v += dx[3:6]
        self.R = SciRot.from_rotvec(dx[6:9]).as_matrix() @ self.R
        self.ba += dx[9:12]
        self.bg += dx[12:15]

    def image_callback(self, msg):
        """相机质量门控。

        当前原型没有直接做特征点重投影更新，而是先判断图像是否可靠：
        - 过曝比例太高：白斑严重，直接法/特征法都不可靠。
        - 欠曝比例太高：暗部无纹理。
        - 梯度均值太低：纹理太弱。
        若图像不可靠，则降低视觉相关权重。后续可在这里接入特征重投影残差。
        """
        if not self.camera_enable:
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except Exception:
            return
        over = float(np.mean(img > 245))
        under = float(np.mean(img < 10))
        gx = cv2.Sobel(img, cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(img, cv2.CV_32F, 0, 1, ksize=3)
        grad_mean = float(np.mean(np.sqrt(gx * gx + gy * gy)))
        good = over < self.max_over_exposure_ratio and under < self.max_under_exposure_ratio and grad_mean > self.min_gradient_mean
        self.visual_weight_scale = self.good_image_weight_scale if good else self.bad_image_weight_scale

    def initial_pose_callback(self, msg):
        """外部初值：RViz 2D Pose Estimate 或老师给定初始位姿。"""
        with self.lock:
            self.p = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z], dtype=np.float64)
            q = msg.pose.pose.orientation
            self.R = SciRot.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            self.v[:] = 0.0
            rospy.loginfo("[DogPriorMap] 收到外部初始位姿，已重置位置和姿态。")

    def publish_state(self, stamp, corrected=False):
        """发布高频传播位姿或低频修正位姿。"""
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = float(self.p[0])
        msg.pose.pose.position.y = float(self.p[1])
        msg.pose.pose.position.z = float(self.p[2])
        q = rot_to_quat_xyzw(self.R)
        msg.pose.pose.orientation.x = float(q[0])
        msg.pose.pose.orientation.y = float(q[1])
        msg.pose.pose.orientation.z = float(q[2])
        msg.pose.pose.orientation.w = float(q[3])
        msg.twist.twist.linear.x = float(self.v[0])
        msg.twist.twist.linear.y = float(self.v[1])
        msg.twist.twist.linear.z = float(self.v[2])

        if corrected:
            self.pub_corr.publish(msg)
            self.append_path(self.path_corr, msg)
            self.pub_path_corr.publish(self.path_corr)
        else:
            self.pub_high.publish(msg)
            self.append_path(self.path_high, msg)
            self.pub_path_high.publish(self.path_high)

        if self.publish_tf:
            self.tf_broadcaster.sendTransform(
                tuple(self.p.tolist()),
                tuple(q.tolist()),
                stamp,
                self.base_frame,
                self.map_frame,
            )

    def append_path(self, path_msg, odom_msg):
        """维护 Path 消息，限制最大长度避免 RViz 越跑越卡。"""
        pose_stamped = type("PoseStampedLike", (), {})()
        from geometry_msgs.msg import PoseStamped
        ps = PoseStamped()
        ps.header = odom_msg.header
        ps.pose = odom_msg.pose.pose
        path_msg.header.stamp = odom_msg.header.stamp
        path_msg.poses.append(ps)
        if len(path_msg.poses) > self.path_max_length:
            path_msg.poses = path_msg.poses[-self.path_max_length:]


def main():
    """初始化 Python 原型节点并进入 ROS 回调循环。"""
    rospy.init_node("dog_prior_map_ekf")
    DogPriorMapEkfNode()
    rospy.loginfo("[DogPriorMap] 节点启动完成，等待 IMU/LiDAR/Camera 数据。")
    rospy.spin()


if __name__ == "__main__":
    main()
