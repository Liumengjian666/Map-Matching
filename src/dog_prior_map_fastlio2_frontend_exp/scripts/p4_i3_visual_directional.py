"""Post-hoc body-frame errors from saved poses; never runs the estimator."""

import numpy as np
from p3_r10c_failure_mechanism import interpolate_gt, read_gt, read_pose_csv
from scipy.spatial.transform import Rotation


def nearest(stream, stamps, stamp):
    hi = int(np.searchsorted(stamps, stamp))
    candidates = [i for i in (hi - 1, hi) if 0 <= i < len(stream)]
    return stream[min(candidates, key=lambda i: (abs(stamps[i] - stamp), i))]


def evaluate_directional(rows, calibration, gt_path, runtime_path):
    """Nearest endpoint fallback authorized by the task; no interpolation across updates.

    Translation and rotation residuals are expressed in the common GT-current
    scan IMU basis. Image-time residuals are rotated into that basis using GT
    only after visual validity has been frozen. This is evaluation, not fusion.
    """
    directory = runtime_path.parent / "evaluation_inputs"
    corrected = read_pose_csv(directory / "corrected.csv")
    predictor = read_pose_csv(directory / "predictor.csv")
    cs = np.array([r["stamp_ns"] for r in corrected], dtype=np.int64)
    ps = np.array([r["stamp_ns"] for r in predictor], dtype=np.int64)
    assert np.all(np.diff(cs) > 0) and np.all(np.diff(ps) > 0)
    gt_times, gt = read_gt(gt_path)
    t_il = calibration["T_imu_lidar"].copy()
    # Exactly the same rotation normalization as the existing R10C evaluator.
    t_il[:3, :3] = Rotation.from_matrix(t_il[:3, :3]).as_matrix()
    t_li = np.linalg.inv(t_il)
    t_ic = calibration["T_imu_camera"]
    count = 0
    for row in rows:
        for method in ("visual", "ikfom"):
            for axis in ("x", "y", "z", "roll", "pitch", "yaw"):
                row[f"e_{method}_{axis}"] = float("nan")
        for key in (
            "ikfom_ref_image_offset_ms",
            "ikfom_cur_image_offset_ms",
            "image_minus_ikfom_dt_ms",
            "ikfom_matched_dt_s",
        ):
            row[key] = float("nan")
        row["directional_evaluable"] = 0
        if not row["attempted"]:
            continue
        c = nearest(corrected, cs, row["timestamp_ref_ns"])
        p = nearest(predictor, ps, row["timestamp_cur_ns"])
        assert c["frame_index"] == row["transaction_ref"]
        assert p["frame_index"] == row["transaction_cur"]
        offsets = (
            c["stamp_ns"] - row["timestamp_ref_ns"],
            p["stamp_ns"] - row["timestamp_cur_ns"],
        )
        assert max(abs(x) for x in offsets) <= 20_000_000
        row["ikfom_ref_image_offset_ms"], row["ikfom_cur_image_offset_ms"] = (
            x / 1e6 for x in offsets
        )
        row["ikfom_matched_dt_s"] = (p["stamp_ns"] - c["stamp_ns"]) * 1e-9
        row["image_minus_ikfom_dt_ms"] = (
            row["image_dt_s"] - row["ikfom_matched_dt_s"]
        ) * 1000
        if not row["paired_evaluable"]:
            continue
        scan_ref = interpolate_gt(gt_times, gt, c["stamp"])
        scan_cur = interpolate_gt(gt_times, gt, p["stamp"])
        image_ref = interpolate_gt(gt_times, gt, row["timestamp_ref_ns"] * 1e-9)
        image_cur = interpolate_gt(gt_times, gt, row["timestamp_cur_ns"] * 1e-9)
        assert all(x is not None for x in (scan_ref, scan_cur, image_ref, image_cur))
        predicted = np.linalg.inv(c["pose_l"] @ t_li) @ (p["pose_l"] @ t_li)
        pred_residual = np.linalg.inv(np.linalg.inv(scan_ref) @ scan_cur) @ predicted
        measured = np.eye(4)
        measured[:3] = np.array(
            [row[f"T_Ccur_Cref_{a}{b}"] for a in range(3) for b in range(4)]
        ).reshape(3, 4)
        visual = t_ic @ np.linalg.inv(measured) @ np.linalg.inv(t_ic)
        visual_residual = np.linalg.inv(np.linalg.inv(image_ref) @ image_cur) @ visual
        q = scan_cur[:3, :3].T @ image_cur[:3, :3]
        visual_residual[:3, 3] = q @ visual_residual[:3, 3]
        visual_residual[:3, :3] = q @ visual_residual[:3, :3] @ q.T
        for method, residual in (("visual", visual_residual), ("ikfom", pred_residual)):
            assert (
                abs(np.linalg.norm(residual[:3, 3]) - row[f"{method}_t_error_m"]) < 1e-5
            )
            assert (
                abs(
                    np.degrees(Rotation.from_matrix(residual[:3, :3]).magnitude())
                    - row[f"{method}_r_error_deg"]
                )
                < 1e-4
            )
            for axis, value in zip("xyz", residual[:3, 3]):
                row[f"e_{method}_{axis}"] = float(value)
            angles = Rotation.from_matrix(residual[:3, :3]).as_euler(
                "xyz", degrees=True
            )
            for axis, value in zip(("roll", "pitch", "yaw"), angles):
                row[f"e_{method}_{axis}"] = float(value)
        row["directional_evaluable"] = 1
        count += 1
    return count


def direction_stats(rows):
    groups = [("all", rows), ("150-end", [r for r in rows if r["eval_time_s"] >= 150])]
    groups += [
        (
            f"{lo}-{lo + 50 if lo < 350 else 'end'}",
            [
                r
                for r in rows
                if lo <= r["eval_time_s"] < (lo + 50 if lo < 350 else float("inf"))
            ],
        )
        for lo in range(0, 400, 50)
    ]
    result = []
    for label, group in groups:
        paired = [r for r in group if r["directional_evaluable"]]
        for method in ("visual", "ikfom"):
            entry = {"segment": label, "method": method, "count": len(paired)}
            for axis in ("x", "y", "z", "roll", "pitch", "yaw"):
                entry[f"{axis}_rmse"] = (
                    float(
                        np.sqrt(np.mean([r[f"e_{method}_{axis}"] ** 2 for r in paired]))
                    )
                    if paired
                    else float("nan")
                )
            result.append(entry)
    return result
