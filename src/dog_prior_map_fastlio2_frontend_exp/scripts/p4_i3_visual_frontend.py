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


def load_calibration(directory, dataset_prefix="floor01"):
    """Load the same validated MEI/depth geometry for another calibrated set."""
    with (directory / f"{dataset_prefix}_intrinsics.yaml").open() as stream:
        intr = yaml.safe_load(stream)["rgb_camera"]
    with (directory / f"{dataset_prefix}_extrinsics.yaml").open() as stream:
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
    t_ic = np.array(extr["rgb_camera_to_imu"]["data"], dtype=float).reshape(4, 4)
    t_il = np.array(extr["laser_to_imu"]["data"], dtype=float).reshape(4, 4)
    rotation_projection_error = {}
    for name, transform, max_defect, project_to_so3 in (
        ("camera_to_imu", t_ic, 1e-6, False),
        # P2C explicitly projects Corridor01's rounded official laser matrix
        # before composing transforms. Preserve Floor01's historical raw
        # calibration semantics and reject non-orthogonal Floor01 matrices.
        ("laser_to_imu", t_il,
         2e-3 if dataset_prefix == "corridor01" else 1e-6,
         dataset_prefix == "corridor01"),
    ):
        raw_rotation = transform[:3, :3]
        defect = np.max(abs(raw_rotation.T @ raw_rotation - np.eye(3)))
        determinant = np.linalg.det(raw_rotation)
        if (not np.isfinite(transform).all() or defect > max_defect or
                determinant <= 0.0 or abs(determinant - 1.0) > max_defect):
            raise RuntimeError(f"invalid official {name} calibration matrix")
        if project_to_so3:
            u, _, vt = np.linalg.svd(raw_rotation)
            sign = np.eye(3)
            sign[2, 2] = np.linalg.det(u @ vt)
            transform[:3, :3] = u @ sign @ vt
        rotation_projection_error[name] = float(defect)
    if not np.allclose(t_ic[3], [0, 0, 0, 1], atol=1e-10) or \
       not np.allclose(t_il[3], [0, 0, 0, 1], atol=1e-10):
        raise RuntimeError("invalid official calibration homogeneous row")
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
        "rotation_projection_defect": rotation_projection_error,
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


def associate_depth(cloud, features, calibration, include_uncertainty=False):
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
        empty = (
            np.zeros(len(features), dtype=bool),
            np.empty((0, 3)),
            (time.perf_counter() - tick) * 1000,
            0.0,
        )
        return empty if not include_uncertainty else (empty[0], empty[1], np.empty((0, 3, 3)), empty[2], empty[3])
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
    depth_z = xyz[chosen[index[good]], 2]
    points = rays * depth_z[:, None]
    association_ms = (time.perf_counter() - tick) * 1000
    if not include_uncertainty:
        return good, points, projection_ms, association_ms

    # Conservative sensor-local point uncertainty used only by the optional
    # P25 covariance path.  The Corridor01 calibration does not publish a
    # LiDAR range-noise covariance, so use a declared 5 cm floor and 1% of
    # range; image feature coordinates use a 0.5 px quantization floor.
    inverse_k = np.linalg.inv(k)
    point_covariances = np.empty((len(points), 3, 3), dtype=np.float64)
    for i, (uv_feature, ray, z) in enumerate(zip(features[good], rays, depth_z)):
        jacobian = np.column_stack((z * inverse_k[:, 0], z * inverse_k[:, 1], ray))
        sigma_depth = max(0.05, 0.01 * float(z))
        input_covariance = np.diag([0.5 ** 2, 0.5 ** 2, sigma_depth ** 2])
        point_covariances[i] = jacobian @ input_covariance @ jacobian.T
    return good, points, point_covariances, projection_ms, association_ms


def _skew(vector):
    x, y, z = np.asarray(vector, dtype=float).reshape(3)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def _left_se3_log(transform):
    rotation = transform[:3, :3]
    cosine = np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0)
    angle = np.arccos(cosine)
    vee = np.array([rotation[2, 1] - rotation[1, 2],
                    rotation[0, 2] - rotation[2, 0],
                    rotation[1, 0] - rotation[0, 1]])
    rotation_vector = (0.5 * vee if angle < 1e-6 else
                       angle / (2.0 * np.sin(angle)) * vee)
    angle = np.linalg.norm(rotation_vector)
    omega = _skew(rotation_vector)
    if angle < 1e-8:
        left_jacobian = np.eye(3) + 0.5 * omega + (1.0 / 6.0) * omega @ omega
    else:
        left_jacobian = (np.eye(3) + (1.0 - np.cos(angle)) / angle ** 2 * omega +
                         (angle - np.sin(angle)) / angle ** 3 * omega @ omega)
    rho = np.linalg.solve(left_jacobian, transform[:3, 3])
    return np.r_[rho, rotation_vector]


def pnp_left_covariance(points, pixels, point_covariances, k, transform,
                        inlier_indices, reprojection_rmse_px):
    """Full 6x6 covariance from P4 inlier reprojection geometry.

    Tangent ordering is [translation (m), left rotation (rad)] for a
    left-multiplicative perturbation of T_Ccur_Cref.  Pixel residual noise is
    floored at 0.75 px and the resulting information inverse is inflated by
    four to cover unmodelled KLT/depth association correlation.
    """
    indices = np.asarray(inlier_indices, dtype=np.int64).reshape(-1)
    if len(indices) < 20 or point_covariances.shape != (len(points), 3, 3):
        return None
    fx, fy = float(k[0, 0]), float(k[1, 1])
    sigma_pixel = max(0.75, float(reprojection_rmse_px) / np.sqrt(2.0))
    information = np.zeros((6, 6), dtype=np.float64)
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    for index in indices:
        point = points[index]
        transformed = rotation @ point + translation
        x, y, z = transformed
        if not np.isfinite(transformed).all() or z <= 1e-6:
            return None
        projection_jacobian = np.array([
            [fx / z, 0.0, -fx * x / (z * z)],
            [0.0, fy / z, -fy * y / (z * z)],
        ])
        pose_jacobian = projection_jacobian @ np.column_stack((np.eye(3), -_skew(transformed)))
        point_jacobian = projection_jacobian @ rotation
        residual_covariance = (sigma_pixel ** 2 * np.eye(2) +
                               point_jacobian @ point_covariances[index] @ point_jacobian.T)
        residual_covariance = 0.5 * (residual_covariance + residual_covariance.T)
        try:
            weight = np.linalg.inv(residual_covariance)
        except np.linalg.LinAlgError:
            return None
        information += pose_jacobian.T @ weight @ pose_jacobian

    information = 0.5 * (information + information.T)
    eigenvalues = np.linalg.eigvalsh(information)
    if (not np.isfinite(eigenvalues).all() or eigenvalues[0] <= 1e-10 or
            eigenvalues[-1] / eigenvalues[0] > 1e12):
        return None
    covariance = 4.0 * np.linalg.inv(information)
    # Explicit conservative floors for unreported calibration/model error;
    # these are not fitted from GT or presented as sensor-calibrated values.
    covariance += np.diag([0.02 ** 2] * 3 + [np.deg2rad(0.5) ** 2] * 3)
    covariance = 0.5 * (covariance + covariance.T)
    if not np.isfinite(covariance).all() or np.linalg.eigvalsh(covariance)[0] <= 0.0:
        return None
    return covariance


def relative_imu_covariance(transform_camera, covariance_camera, t_imu_camera):
    """Propagate PnP left-tangent covariance through the calibrated conjugation."""
    def to_relative(transform):
        return t_imu_camera @ np.linalg.inv(transform) @ np.linalg.inv(t_imu_camera)

    base = to_relative(transform_camera)
    jacobian = np.zeros((6, 6), dtype=np.float64)
    epsilon = 1e-6
    for column in range(6):
        delta = np.zeros(6)
        delta[column] = epsilon
        angle = np.linalg.norm(delta[3:])
        perturbed = np.eye(4)
        if angle > 1e-12:
            perturbed[:3, :3] = cv2.Rodrigues(delta[3:])[0]
            omega = _skew(delta[3:])
            left_jacobian = np.eye(3) + (1.0 - np.cos(angle)) / angle ** 2 * omega + (angle - np.sin(angle)) / angle ** 3 * omega @ omega
            perturbed[:3, 3] = left_jacobian @ delta[:3]
        else:
            perturbed[:3, 3] = delta[:3]
        plus = to_relative(perturbed @ transform_camera)
        delta[column] = -epsilon
        angle = np.linalg.norm(delta[3:])
        perturbed = np.eye(4)
        if angle > 1e-12:
            perturbed[:3, :3] = cv2.Rodrigues(delta[3:])[0]
            omega = _skew(delta[3:])
            left_jacobian = np.eye(3) + (1.0 - np.cos(angle)) / angle ** 2 * omega + (angle - np.sin(angle)) / angle ** 3 * omega @ omega
            perturbed[:3, 3] = left_jacobian @ delta[:3]
        else:
            perturbed[:3, 3] = delta[:3]
        minus = to_relative(perturbed @ transform_camera)
        jacobian[:, column] = (_left_se3_log(plus @ np.linalg.inv(base)) -
                               _left_se3_log(minus @ np.linalg.inv(base))) / (2.0 * epsilon)
    covariance = jacobian @ covariance_camera @ jacobian.T
    covariance = 0.5 * (covariance + covariance.T)
    if not np.isfinite(covariance).all() or np.linalg.eigvalsh(covariance)[0] <= 0.0:
        return None
    return covariance


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
        return (None, 0 if inliers is None else len(inliers), float("nan"),
                np.empty(0, dtype=np.int64))
    take = inliers.ravel()
    rvec, tvec = cv2.solvePnPRefineLM(points[take], pixels[take], k, None, rvec, tvec)
    transform = np.eye(4)
    transform[:3, :3] = cv2.Rodrigues(rvec)[0]
    transform[:3, 3] = tvec.ravel()
    projected = cv2.projectPoints(points[take], rvec, tvec, k, None)[0].reshape(-1, 2)
    rmse = float(np.sqrt(np.mean(np.sum((projected - pixels[take]) ** 2, axis=1))))
    positive = (points[take] @ transform[:3, :3].T + transform[:3, 3])[:, 2] > 0
    if not np.isfinite(transform).all() or not np.all(positive):
        return None, len(take), rmse, take
    return transform, len(take), rmse, take


def feature_distribution(pixels, image_size):
    """Return dimensionless grid occupancy and convex-hull coverage."""
    width, height = image_size
    pixels = np.asarray(pixels, dtype=np.float64).reshape(-1, 2)
    if len(pixels) < 3 or width <= 0 or height <= 0:
        return float("nan"), float("nan")
    grid_x = np.clip((pixels[:, 0] * 4 / width).astype(int), 0, 3)
    grid_y = np.clip((pixels[:, 1] * 3 / height).astype(int), 0, 2)
    occupancy = len(np.unique(grid_y * 4 + grid_x)) / 12.0
    hull = cv2.convexHull(pixels.astype(np.float32))
    hull_fraction = float(cv2.contourArea(hull) / (width * height))
    return float(occupancy), hull_fraction


def estimate(ref, cur, cloud, calibration, seed, include_covariance=False):
    start = time.perf_counter()
    row = {
        "status": "NO_FEATURES",
        "detected": 0,
        "tracked_count": 0,
        "klt_valid": 0,
        "klt_forward_valid": 0,
        "fb_valid": 0,
        "depth_associated": 0,
        "pnp_correspondences": 0,
        "pnp_inliers": 0,
        "inlier_ratio": 0.0,
        "depth_fraction": float("nan"),
        "grid_occupancy": float("nan"),
        "hull_fraction": float("nan"),
        "median_parallax_px": float("nan"),
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
    row["tracked_count"] = len(a)
    row["klt_ms"] = (time.perf_counter() - tick) * 1000
    tick = time.perf_counter()
    if include_covariance:
        good, points, point_covariances, row["projection_ms"], row["association_ms"] = associate_depth(
            cloud, a, calibration, include_uncertainty=True)
    else:
        good, points, row["projection_ms"], row["association_ms"] = associate_depth(cloud, a, calibration)
    pixels = b[good].astype(float)
    row["depth_ms"] = (time.perf_counter() - tick) * 1000
    row["depth_associated"] = row["pnp_correspondences"] = len(points)
    row["depth_fraction"] = len(points) / max(1, len(a))
    row["status"] = "INSUFFICIENT_CORRESPONDENCES"
    if len(points) < 30:
        return row, None
    tick = time.perf_counter()
    transform, count, rmse, inlier_indices = pnp(
        points, pixels, calibration["Krect"], seed)
    inlier_pixels = pixels[inlier_indices] if len(inlier_indices) else np.empty((0, 2))
    grid_occupancy, hull_fraction = feature_distribution(
        inlier_pixels, calibration["size"])
    row["grid_occupancy"] = grid_occupancy
    row["hull_fraction"] = hull_fraction
    if len(inlier_indices):
        row["median_parallax_px"] = float(np.median(
            np.linalg.norm(b[good][inlier_indices] - a[good][inlier_indices], axis=1)))
    row.update(
        pnp_inliers=count,
        inlier_ratio=count / len(points),
        reprojection_rmse_px=rmse,
        pnp_ms=(time.perf_counter() - tick) * 1000,
        status="VALID" if transform is not None else "PNP_REJECT",
    )
    if include_covariance:
        covariance = None if transform is None else pnp_left_covariance(
            points, pixels, point_covariances, calibration["Krect"], transform,
            inlier_indices, rmse)
        row["pnp_covariance_valid"] = int(covariance is not None)
        if covariance is not None:
            for i in range(6):
                for j in range(6):
                    row[f"pnp_cov_{i}{j}"] = float(covariance[i, j])
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
    measured, count, rmse, _ = pnp(points, pixels, calibration["Krect"], 1)
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
