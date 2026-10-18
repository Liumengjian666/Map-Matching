"""IMU-only feasibility diagnostics, NOT a LiDAR-IMU calibration or Fisher matrix."""
import numpy as np


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def gram_summary(matrices):
    matrices = np.asarray(matrices)
    gram = np.einsum('nki,nkj->ij', matrices, matrices) / len(matrices)
    values, vectors = np.linalg.eigh(gram)
    return {"eigenvalues_ascending": values.tolist(),
            "min_max_ratio": float(values[0] / values[-1]) if values[-1] > 0 else None,
            "least_sensitive_axis": vectors[:, 0].tolist()}


def motion_summary(t, omega):
    """omega rad/s in IMU frame; alpha rad/s^2; t relative seconds.

    Rotation design -[omega]x is a *known gyro alignment* sensitivity only.
    Lever-arm design maps t_I_L in m to a_L^I-a_I in m/s^2.
    Neither includes LiDAR uncertainty, biases, gravity or clock-offset nuisance states.
    """
    if len(t) < 3 or np.any(np.diff(t) <= 0) or not np.isfinite(omega).all():
        raise ValueError('finite samples and strictly increasing times required')
    alpha = np.gradient(omega, t, axis=0)
    cross = np.array([skew(w) for w in omega])
    lever = np.array([skew(a) for a in alpha]) + cross @ cross
    moment = omega.T @ omega / len(omega)
    values, axes = np.linalg.eigh(moment)
    active = np.linalg.norm(omega, axis=1) > 0.05  # fixed pre-result diagnostic only
    return {"samples": len(t), "duration_sec": float(t[-1] - t[0]),
            "gyro_rms_xyz_rad_s": np.sqrt(np.mean(omega**2, axis=0)).tolist(),
            "gyro_peak_xyz_rad_s": np.max(np.abs(omega), axis=0).tolist(),
            "integral_abs_gyro_xyz_deg": np.rad2deg(np.trapz(np.abs(omega), t, axis=0)).tolist(),
            "active_samples_gt_0p05_rad_s": int(active.sum()),
            "gyro_second_moment_eigenvalues": values.tolist(),
            "dominant_gyro_axis": axes[:, -1].tolist(),
            "dominant_gyro_energy_fraction": float(values[-1] / values.sum()) if values.sum() > 0 else None,
            "rotation_sensitivity": gram_summary(cross),
            "lever_arm_sensitivity": gram_summary(lever)}


def absolute_point_interval(header_ns, offsets):
    """Conditional on verified standard driver semantics; never clip/rebase time."""
    offsets = np.asarray(offsets, dtype=float)
    if not len(offsets) or not np.isfinite(offsets).all():
        raise ValueError('finite nonempty offsets required')
    return (int(header_ns) + int(np.rint(offsets.min() * 1e9)),
            int(header_ns) + int(np.rint(offsets.max() * 1e9)))
