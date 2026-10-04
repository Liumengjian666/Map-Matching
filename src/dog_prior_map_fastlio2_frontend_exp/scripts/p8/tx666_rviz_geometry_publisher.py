#!/usr/bin/env python3
"""Publish one frozen TX666 scan and both Corridor01 map frames for RViz."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np
import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import TransformStamped
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf.transformations import quaternion_from_matrix


PCD_TYPES = {
    ("F", 4): "<f4",
    ("F", 8): "<f8",
    ("U", 1): "u1",
    ("U", 2): "<u2",
    ("U", 4): "<u4",
    ("I", 1): "i1",
    ("I", 2): "<i2",
    ("I", 4): "<i4",
}


def read_binary_pcd_xyz(path):
    path = Path(path)
    with path.open("rb") as stream:
        header_lines = []
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f"pcd_header_truncated:{path}")
            header_lines.append(line.decode("ascii").strip())
            if header_lines[-1].startswith("DATA "):
                break
        header = {line.split()[0]: line.split()[1:] for line in header_lines if line.split()}
        if header["DATA"] != ["binary"]:
            raise ValueError(f"pcd_requires_uncompressed_binary:{path}")
        fields = header["FIELDS"]
        sizes = [int(value) for value in header["SIZE"]]
        types = header["TYPE"]
        counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
        if not (len(fields) == len(sizes) == len(types) == len(counts)):
            raise ValueError(f"pcd_field_header_mismatch:{path}")
        dtype_fields = []
        point_step = 0
        for name, size, field_type, count in zip(fields, sizes, types, counts):
            try:
                scalar_type = PCD_TYPES[(field_type, size)]
            except KeyError as error:
                raise ValueError(f"pcd_unsupported_field:{name}:{field_type}:{size}") from error
            dtype_fields.append((name, scalar_type, (count,)) if count > 1 else (name, scalar_type))
            point_step += size * count
        payload = stream.read()
    point_count = int(header["POINTS"][0])
    declared_payload_bytes = point_count * point_step
    if len(payload) < declared_payload_bytes:
        raise ValueError(f"pcd_payload_truncated:{path}")
    trailing = payload[declared_payload_bytes:]
    if any(trailing):
        raise ValueError(f"pcd_nonzero_trailing_payload:{path}")
    records = np.frombuffer(payload, dtype=np.dtype(dtype_fields), count=point_count)
    xyz = np.column_stack((records["x"], records["y"], records["z"])).astype(np.float64)
    if xyz.shape != (point_count, 3) or not np.isfinite(xyz).all():
        raise ValueError(f"pcd_nonfinite_xyz:{path}")
    return xyz, len(trailing)


def pose_matrix(row, prefix):
    from scipy.spatial.transform import Rotation

    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, 3] = [float(row[f"{prefix}_{axis}"]) for axis in "xyz"]
    quaternion = [float(row[f"{prefix}_q{axis}"]) for axis in ("x", "y", "z", "w")]
    matrix[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    return matrix


def transform_points(matrix, points):
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def validate_rigid(name, matrix):
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError(f"invalid_transform:{name}")
    rotation = matrix[:3, :3]
    if np.linalg.norm(rotation.T @ rotation - np.eye(3)) > 2e-5 or \
            abs(np.linalg.det(rotation) - 1.0) > 2e-5 or \
            not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-10):
        raise ValueError(f"nonrigid_transform:{name}")


def point_cloud_message(points, frame_id):
    xyz = np.asarray(points, dtype="<f4", order="C")
    if xyz.ndim != 2 or xyz.shape[1] != 3 or not np.isfinite(xyz).all():
        raise ValueError(f"invalid_published_cloud:{frame_id}")
    message = PointCloud2()
    message.header = Header(stamp=rospy.Time(0), frame_id=frame_id)
    message.height = 1
    message.width = int(xyz.shape[0])
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = message.point_step * message.width
    message.is_dense = True
    message.data = xyz.tobytes(order="C")
    return message


def transform_message(parent, child, matrix):
    message = TransformStamped()
    message.header.stamp = rospy.Time.now()
    message.header.frame_id = parent
    message.child_frame_id = child
    message.transform.translation.x = float(matrix[0, 3])
    message.transform.translation.y = float(matrix[1, 3])
    message.transform.translation.z = float(matrix[2, 3])
    quaternion = quaternion_from_matrix(matrix)
    message.transform.rotation.x = float(quaternion[0])
    message.transform.rotation.y = float(quaternion[1])
    message.transform.rotation.z = float(quaternion[2])
    message.transform.rotation.w = float(quaternion[3])
    return message


def load_inputs(args):
    config = yaml.safe_load(Path(args.dataset_config).read_text(encoding="utf-8"))
    manifest = json.loads(Path(args.startup_manifest).read_text(encoding="utf-8"))
    with Path(args.registration_csv).open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    tx_rows = [row for row in rows if row["transaction_id"] == "666"]
    if len(tx_rows) != 1:
        raise ValueError("registration_must_have_exactly_one_tx666")
    registration = tx_rows[0]
    if registration["status"] != "ITERATION_LIMIT_EXHAUSTED" or \
            registration["effective"] != "0" or registration["raw_source_points"] != "29063" or \
            registration["overlap_points_dropped"] != "0":
        raise ValueError("tx666_registration_provenance_unexpected")

    transforms = {
        name: np.asarray(value, dtype=np.float64).reshape(4, 4)
        for name, value in manifest["transforms"].items()
    }
    T_world_imu = transforms["T_WORLD_IMU"]
    T_imu_lidar = transforms["T_IMU_LIDAR"]
    T_world_lidar = transforms["T_WORLD_LIDAR"]
    T_normalized_world = transforms["T_NORMALIZED_WORLD"]
    T_normalized_lidar_contract = transforms["T_NORMALIZED_LIDAR"]
    expected_norm_world = np.asarray(
        config["frames"]["normalized_map_world_transform"], dtype=np.float64
    ).reshape(4, 4)
    if np.max(np.abs(T_normalized_world - expected_norm_world)) > 1e-10:
        raise ValueError("manifest_and_dataset_config_normalized_world_mismatch")
    if np.max(np.abs(T_world_lidar - T_world_imu @ T_imu_lidar)) > 2e-5:
        raise ValueError("world_lidar_composition_mismatch")
    if np.max(np.abs(T_normalized_lidar_contract -
                     T_normalized_world @ T_world_imu @ T_imu_lidar)) > 2e-5:
        raise ValueError("normalized_lidar_composition_mismatch")

    T_normalized_lidar_initial = pose_matrix(registration, "initial")
    T_normalized_lidar_terminal = pose_matrix(registration, "raw")
    T_world_from_normalized = np.linalg.inv(T_normalized_world)
    T_world_lidar_prediction = T_world_from_normalized @ T_normalized_lidar_initial
    T_world_lidar_terminal = T_world_from_normalized @ T_normalized_lidar_terminal
    T_world_imu_prediction = T_world_lidar_prediction @ np.linalg.inv(T_imu_lidar)
    T_world_imu_terminal = T_world_lidar_terminal @ np.linalg.inv(T_imu_lidar)

    raw_map, raw_map_trailing_zero_bytes = read_binary_pcd_xyz(args.raw_map)
    normalized_map, normalized_map_trailing_zero_bytes = read_binary_pcd_xyz(
        config["paths"]["normalized_map"])
    deskew_scan, deskew_scan_trailing_zero_bytes = read_binary_pcd_xyz(args.deskew_scan)
    if raw_map.shape[0] != 338210 or normalized_map.shape[0] != 338210:
        raise ValueError("map_point_count_mismatch")
    if deskew_scan.shape[0] != 29063:
        raise ValueError("tx666_deskewed_point_count_mismatch")

    transformed_raw_map = transform_points(T_normalized_world, raw_map)
    tree = cKDTree(normalized_map)
    map_residual, _ = tree.query(transformed_raw_map, k=1, workers=-1)
    map_alignment = {
        "mean_m": float(np.mean(map_residual)),
        "median_m": float(np.median(map_residual)),
        "p95_m": float(np.quantile(map_residual, 0.95)),
        "max_m": float(np.max(map_residual)),
    }

    clouds = {
        "/corridor01/raw_map": (raw_map, "world"),
        "/corridor01/normalized_map": (normalized_map, "normalized_map"),
        "/corridor01/tx666_official_raw":
            (transform_points(T_world_lidar, deskew_scan), "world"),
        "/corridor01/tx666_official_normalized":
            (transform_points(T_normalized_lidar_contract, deskew_scan), "normalized_map"),
        "/corridor01/tx666_ndt_raw":
            (transform_points(T_world_lidar_terminal, deskew_scan), "world"),
        "/corridor01/tx666_ndt_normalized":
            (transform_points(T_normalized_lidar_terminal, deskew_scan), "normalized_map"),
    }
    frame_transforms = [
        ("world", "imu", T_world_imu),
        ("imu", "lidar", T_imu_lidar),
        ("world", "tx666_prediction_imu", T_world_imu_prediction),
        ("tx666_prediction_imu", "tx666_prediction_lidar", T_imu_lidar),
        ("world", "tx666_ndt_imu", T_world_imu_terminal),
        ("tx666_ndt_imu", "tx666_ndt_lidar", T_imu_lidar),
        ("normalized_map", "world", T_normalized_world),
    ]
    result = {
        "transaction_id": 666,
        "raw_map_frame_assumption": config["frames"]["raw_map_world_relation_status"],
        "normalized_map_frame": config["frames"]["normalized_map"],
        "scan_points": int(deskew_scan.shape[0]),
        "pcd_trailing_zero_padding_bytes": {
            "raw_map": raw_map_trailing_zero_bytes,
            "normalized_map": normalized_map_trailing_zero_bytes,
            "deskew_scan": deskew_scan_trailing_zero_bytes,
        },
        "deskew_reference": "scan_end_lidar_frame; same ScanEndProcessor output as frozen P7 replay",
        "deskew_points_dropped": 0,
        "ndt_update_applied": False,
        "ndt_status": registration["status"],
        "ndt_iterations": int(registration["iterations"]),
        "ndt_correction_translation_m": float(registration["correction_translation_m"]),
        "ndt_correction_rotation_rad": float(registration["correction_rotation_rad"]),
        "map_alignment_raw_to_normalized_nn": map_alignment,
        "T_WORLD_IMU": T_world_imu.tolist(),
        "T_IMU_LIDAR": T_imu_lidar.tolist(),
        "T_WORLD_LIDAR_anchor": T_world_lidar.tolist(),
        "T_NORMALIZED_WORLD": T_normalized_world.tolist(),
        "T_NORMALIZED_LIDAR_anchor": T_normalized_lidar_contract.tolist(),
        "T_NORMALIZED_LIDAR_tx666_official_initial": T_normalized_lidar_contract.tolist(),
        "T_NORMALIZED_LIDAR_tx666_prediction": T_normalized_lidar_initial.tolist(),
        "T_NORMALIZED_LIDAR_tx666_ndt_terminal": T_normalized_lidar_terminal.tolist(),
        "T_WORLD_LIDAR_tx666_official_initial": T_world_lidar.tolist(),
        "T_WORLD_LIDAR_tx666_prediction": T_world_lidar_prediction.tolist(),
        "T_WORLD_LIDAR_tx666_ndt_terminal": T_world_lidar_terminal.tolist(),
        "GT_USED": False,
        "cloud_topics": {name: {"points": int(points.shape[0]), "frame": frame}
                         for name, (points, frame) in clouds.items()},
        "tf_edges": [
            {"parent": parent, "child": child, "matrix": matrix.tolist()}
            for parent, child, matrix in frame_transforms
        ],
    }
    for name, matrix in transforms.items():
        validate_rigid(name, matrix)
    for name, matrix in {
        "T_NORMALIZED_LIDAR_INITIAL": T_normalized_lidar_initial,
        "T_NORMALIZED_LIDAR_TERMINAL": T_normalized_lidar_terminal,
        "T_WORLD_LIDAR_PREDICTION": T_world_lidar_prediction,
        "T_WORLD_LIDAR_TERMINAL": T_world_lidar_terminal,
    }.items():
        validate_rigid(name, matrix)
    return result, clouds, frame_transforms


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-config", required=True)
    parser.add_argument("--startup-manifest", required=True)
    parser.add_argument("--registration-csv", required=True)
    parser.add_argument("--raw-map", required=True)
    parser.add_argument("--deskew-scan", required=True)
    parser.add_argument("--output-manifest", required=True)
    args = parser.parse_args()

    rospy.init_node("corridor01_tx666_rviz_geometry_publisher", anonymous=False)
    result, clouds, frame_transforms = load_inputs(args)
    publishers = {
        topic: rospy.Publisher(topic, PointCloud2, queue_size=1, latch=True)
        for topic in clouds
    }
    messages = {
        topic: point_cloud_message(points, frame)
        for topic, (points, frame) in clouds.items()
    }
    static_tf = tf2_ros.StaticTransformBroadcaster()
    tf_messages = [transform_message(parent, child, matrix)
                   for parent, child, matrix in frame_transforms]
    static_tf.sendTransform(tf_messages)
    for topic, message in messages.items():
        publishers[topic].publish(message)

    output_path = Path(args.output_manifest)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print("TX666_POINTS=29063/29063")
    print("NDT_UPDATE_APPLIED=false")
    print("RAW_TO_NORMALIZED_MAP_NN=" + json.dumps(
        result["map_alignment_raw_to_normalized_nn"], sort_keys=True))
    print("PUBLISHED_CLOUD_TOPICS=" + json.dumps(result["cloud_topics"], sort_keys=True))
    print("PUBLISHED_TF_EDGES=" + json.dumps(
        [{"parent": edge["parent"], "child": edge["child"]}
         for edge in result["tf_edges"]], sort_keys=True))
    print("VISUALIZATION_MANIFEST=" + str(output_path))
    rospy.loginfo("Published frozen TX666 clouds and frame transforms; Ctrl-C to stop.")
    rospy.spin()


if __name__ == "__main__":
    main()
