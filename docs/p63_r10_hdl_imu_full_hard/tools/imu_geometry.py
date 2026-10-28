"""Rigid-body input conversion only; no filter, GT, thresholds or timestamp edits."""
import numpy as np
from scipy.spatial.transform import Rotation
R_L_I = Rotation.from_quat([-.476, .474, .524, .525]).as_matrix()
P_L_I = np.array([.003, .004, -.051])  # IMU origin in cloud frame

def convert(force_i, omega_i, previous_omega_i, dt):
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError('Strictly increasing original IMU timestamps required')
    omega_l = R_L_I @ np.asarray(omega_i)
    alpha_l = R_L_I @ ((np.asarray(omega_i) - np.asarray(previous_omega_i)) / dt)
    # a_I^L = a_L^L + alpha x p_L_I + omega x (omega x p_L_I).
    # Same local gravitational field at both origins: force follows same relation.
    lever = np.cross(alpha_l, P_L_I) + np.cross(omega_l, np.cross(omega_l, P_L_I))
    force_l = R_L_I @ np.asarray(force_i) - lever
    if not np.isfinite(np.r_[force_l, omega_l, lever]).all():
        raise ValueError('Nonfinite conversion')
    return force_l, omega_l, lever
