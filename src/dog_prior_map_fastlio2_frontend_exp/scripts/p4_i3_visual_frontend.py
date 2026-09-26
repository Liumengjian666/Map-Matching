"""OpenCV MEI rectification, KLT and LiDAR-depth PnP. No GT is accepted here."""

import sys
import time

import numpy as np

# Distro OpenCV has ccalib/omnidir; user-site OpenCV 5 lacks it. Keep the
# existing NumPy and restore search order immediately; no environment install.
sys.path.insert(0, "/usr/lib/python3/dist-packages")
import cv2

sys.path.pop(0)
import yaml
from scipy.spatial import cKDTree


def load_calibration(directory):
    with (directory / "floor01_intrinsics.yaml").open() as stream:
        intr = yaml.safe_load(stream)["rgb_camera"]
    with (directory / "floor01_extrinsics.yaml").open() as stream:
        extr = yaml.safe_load(stream)
    assert intr["model_type"] == "MEI"
    p = intr["projection_parameters"]
    k = np.array([[p["gamma1"], 0, p["u0"]], [0, p["gamma2"], p["v0"]], [0, 0, 1.0]])
    d = np.array(
        [intr["distortion_parameters"][name] for name in ("k1", "k2", "p1", "p2")]
    )
    xi = float(intr["mirror_parameters"]["xi"])
    # Fixed central angular scale of MEI model; no GT/image-accuracy tuning.
    rect_k = k.copy()
    rect_k[0, 0] /= 1 + xi
    rect_k[1, 1] /= 1 + xi
    size = (intr["image_width"], intr["image_height"])
    maps = cv2.omnidir.initUndistortRectifyMap(
        k,
        d,
        np.array([xi]),
        np.eye(3),
        rect_k,
        size,
        cv2.CV_32FC1,
        cv2.omnidir.RECTIFY_PERSPECTIVE,
    )
    t_ic = np.array(extr["rgb_camera_to_imu"]["data"]).reshape(4, 4)
    t_il = np.array(extr["laser_to_imu"]["data"]).reshape(4, 4)
    for transform in (t_ic, t_il):
        assert np.max(abs(transform[:3, :3].T @ transform[:3, :3] - np.eye(3))) < 1e-6
        assert abs(np.linalg.det(transform[:3, :3]) - 1) < 1e-6
    mask = (
        (maps[0] >= 0)
        & (maps[0] < size[0] - 1)
        & (maps[1] >= 0)
        & (maps[1] < size[1] - 1)
    ).astype("uint8") * 255
    return {
        "K": k,
        "D": d,
        "xi": xi,
        "Krect": rect_k,
        "size": size,
        "maps": maps,
        "mask": mask,
        "T_imu_camera": t_ic,
        "T_imu_lidar": t_il,
        "T_camera_lidar": np.linalg.inv(t_ic) @ t_il,
    }


def rectify(message, calibration):
    assert message.encoding in ("bgr8", "rgb8", "mono8")
    channels = 1 if message.encoding == "mono8" else 3
    image = np.frombuffer(message.data, dtype=np.uint8).reshape(
        message.height, message.step
    )
    image = image[:, : message.width * channels].reshape(
        message.height, message.width, channels
    )
    image = cv2.remap(image, *calibration["maps"], cv2.INTER_LINEAR)
    if channels == 1:
        return image.reshape(message.height, message.width)
    return cv2.cvtColor(
        image, cv2.COLOR_BGR2GRAY if message.encoding == "bgr8" else cv2.COLOR_RGB2GRAY
    )


def xyz_array(cloud):
    fields = {f.name: f for f in cloud.fields}
    assert all(fields[a].datatype == 7 and fields[a].count == 1 for a in "xyz")
    dtype = np.dtype(
        {
            "names": list("xyz"),
            "formats": [(">" if cloud.is_bigendian else "<") + "f4"] * 3,
            "offsets": [fields[a].offset for a in "xyz"],
            "itemsize": cloud.point_step,
        }
    )
    values = np.ndarray(
        (cloud.height, cloud.width),
        dtype=dtype,
        buffer=cloud.data,
        strides=(cloud.row_step, cloud.point_step),
    )
    return np.column_stack([values[a].reshape(-1) for a in "xyz"]).astype(float)


def associate_depth(cloud, features, calibration):
    tick = time.perf_counter()
    xyz = xyz_array(cloud)
    transform = calibration["T_camera_lidar"]
    xyz = xyz @ transform[:3, :3].T + transform[:3, 3]
    xyz = xyz[np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > 0)]
    k = calibration["Krect"]
    uvh = xyz @ k.T
    uv = uvh[:, :2] / uvh[:, 2:3]
    w, h = calibration["size"]
    valid = (uv[:, 0] >= 0) & (uv[:, 0] < w - 1) & (uv[:, 1] >= 0) & (uv[:, 1] < h - 1)
    uv, xyz = uv[valid], xyz[valid]
    if not len(uv):
        return (
            np.zeros(len(features), dtype=bool),
            np.empty((0, 3)),
            (time.perf_counter() - tick) * 1000,
            0.0,
        )
    pixels = np.rint(uv).astype(int)
    pixel_id = pixels[:, 1] * w + pixels[:, 0]
    order = np.lexsort((xyz[:, 2], pixel_id))
    _, first = np.unique(pixel_id[order], return_index=True)
    chosen = order[first]  # Nearest positive Z per pixel, then nearest projection.
    projection_ms = (time.perf_counter() - tick) * 1000
    tick = time.perf_counter()
    distance, index = cKDTree(uv[chosen]).query(features, distance_upper_bound=2.0)
    good = np.isfinite(distance)
    rays = np.column_stack([features[good], np.ones(good.sum())]) @ np.linalg.inv(k).T
    points = rays * xyz[chosen[index[good]], 2:3]
    return good, points, projection_ms, (time.perf_counter() - tick) * 1000


def pnp(points, pixels, k, seed):
    cv2.setRNGSeed(int(seed) % 2147483647)
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        points,
        pixels,
        k,
        None,
        iterationsCount=100,
        reprojectionError=2.0,
        confidence=0.99,
        flags=cv2.SOLVEPNP_EPNP,
    )
    if not ok or inliers is None or len(inliers) < 20:
        return None, 0 if inliers is None else len(inliers), float("nan")
    take = inliers.ravel()
    rvec, tvec = cv2.solvePnPRefineLM(points[take], pixels[take], k, None, rvec, tvec)
    transform = np.eye(4)
    transform[:3, :3] = cv2.Rodrigues(rvec)[0]
    transform[:3, 3] = tvec.ravel()
    projected = cv2.projectPoints(points[take], rvec, tvec, k, None)[0].reshape(-1, 2)
    rmse = float(np.sqrt(np.mean(np.sum((projected - pixels[take]) ** 2, axis=1))))
    positive = (points[take] @ transform[:3, :3].T + transform[:3, 3])[:, 2] > 0
    if not np.isfinite(transform).all() or not np.all(positive):
        return None, len(take), rmse
    return transform, len(take), rmse


def estimate(ref, cur, cloud, calibration, seed):
    start = time.perf_counter()
    row = {
        "status": "NO_FEATURES",
        "detected": 0,
        "klt_valid": 0,
        "klt_forward_valid": 0,
        "fb_valid": 0,
        "depth_associated": 0,
        "pnp_correspondences": 0,
        "pnp_inliers": 0,
        "inlier_ratio": 0.0,
        "reprojection_rmse_px": float("nan"),
        "feature_ms": 0.0,
        "klt_ms": 0.0,
        "depth_ms": 0.0,
        "projection_ms": 0.0,
        "association_ms": 0.0,
        "pnp_ms": 0.0,
    }
    features = cv2.goodFeaturesToTrack(ref, 500, 0.01, 10, mask=calibration["mask"])
    row["feature_ms"] = (time.perf_counter() - start) * 1000
    if features is None:
        return row, None
    row["detected"] = len(features)
    tick = time.perf_counter()
    options = {
        "winSize": (21, 21),
        "maxLevel": 3,
        "criteria": (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
    }
    forward, s1, _ = cv2.calcOpticalFlowPyrLK(ref, cur, features, None, **options)
    row["klt_forward_valid"] = int(np.count_nonzero(s1))
    backward, s2, _ = cv2.calcOpticalFlowPyrLK(cur, ref, forward, None, **options)
    a, b = features.reshape(-1, 2), forward.reshape(-1, 2)
    w, h = calibration["size"]
    valid = (
        (s1.ravel() != 0)
        & (s2.ravel() != 0)
        & (np.linalg.norm(backward.reshape(-1, 2) - a, axis=1) <= 1.0)
    )
    valid &= (
        np.isfinite(b).all(axis=1)
        & (b[:, 0] >= 0)
        & (b[:, 0] < w)
        & (b[:, 1] >= 0)
        & (b[:, 1] < h)
    )
    a, b = a[valid], b[valid]
    row["klt_valid"] = len(a)
    row["fb_valid"] = len(a)
    row["klt_ms"] = (time.perf_counter() - tick) * 1000
    tick = time.perf_counter()
    good, points, row["projection_ms"], row["association_ms"] = associate_depth(
        cloud, a, calibration
    )
    pixels = b[good].astype(float)
    row["depth_ms"] = (time.perf_counter() - tick) * 1000
    row["depth_associated"] = row["pnp_correspondences"] = len(points)
    row["status"] = "INSUFFICIENT_CORRESPONDENCES"
    if len(points) < 30:
        return row, None
    tick = time.perf_counter()
    transform, count, rmse = pnp(points, pixels, calibration["Krect"], seed)
    row.update(
        pnp_inliers=count,
        inlier_ratio=count / len(points),
        reprojection_rmse_px=rmse,
        pnp_ms=(time.perf_counter() - tick) * 1000,
        status="VALID" if transform is not None else "PNP_REJECT",
    )
    return row, transform


def sanity(calibration):
    rng = np.random.RandomState(0)
    points = rng.uniform([-2, -1, 3], [2, 1, 8], size=(100, 3))
    true = np.eye(4)
    true[:3, :3] = cv2.Rodrigues(np.array([0.02, -0.03, 0.01]))[0]
    true[:3, 3] = [0.12, -0.03, 0.02]
    pixels = cv2.projectPoints(
        points, cv2.Rodrigues(true[:3, :3])[0], true[:3, 3], calibration["Krect"], None
    )[0].reshape(-1, 2)
    measured, count, rmse = pnp(points, pixels, calibration["Krect"], 1)
    assert measured is not None and np.max(abs(measured - true)) < 1e-5 and count == 100
    t_ic = calibration["T_imu_camera"]
    imu_increment = t_ic @ np.linalg.inv(measured) @ np.linalg.inv(t_ic)
    reconstructed = np.linalg.inv(imu_increment @ t_ic) @ t_ic
    assert np.max(abs(reconstructed - true)) < 1e-5
    # Check the rectification map against mature OpenCV MEI projection.
    test_uv = np.array([[100.0, 100.0], [320.0, 240.0], [500.0, 350.0]])
    rays = (
        np.column_stack([test_uv, np.ones(3)]) @ np.linalg.inv(calibration["Krect"]).T
    )
    raw = cv2.omnidir.projectPoints(
        rays.reshape(1, -1, 3),
        np.zeros(3),
        np.zeros(3),
        calibration["K"],
        calibration["xi"],
        calibration["D"],
    )[0].reshape(-1, 2)
    mapped = np.array(
        [[m[int(v), int(u)] for m in calibration["maps"]] for u, v in test_uv]
    )
    assert np.max(abs(raw - mapped)) < 1e-3
    print("SYNTHETIC_PNP_DIRECTION_AND_MEI_RECTIFICATION_PASS", rmse, flush=True)
