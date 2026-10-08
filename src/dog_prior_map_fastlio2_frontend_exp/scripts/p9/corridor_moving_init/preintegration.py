"""Standard midpoint IMU kinematics for bootstrap diagnostics (not a new LIO)."""
import numpy as np
from scipy.spatial.transform import Rotation

def exp(w):
    return Rotation.from_rotvec(w).as_matrix()

def log(R):
    return Rotation.from_matrix(R).as_rotvec()

def integrate(stamps, measurements, begin, end, bg, max_gap=.02, endpoint_hold=.01):
    """Integer ns times. Samples already restricted to the causal stage horizon.

    Midpoint interpolation is only inside observed IMU coverage; a terminal
    hold uses the last real sample for <= frozen endpoint_hold. No backfill.
    Returns DR, Dv, Dp and exact linear accelerometer-bias integral coefficients.
    """
    # Never let interpolation consume a sample after the requested interval end,
    # even when the enclosing bootstrap stage already possesses that sample.
    causal = stamps <= end
    stamps, measurements = stamps[causal], measurements[causal]
    if not len(stamps) or end <= begin or begin < stamps[0] or end-stamps[-1] > int(endpoint_hold*1e9):
        raise ValueError("IMU interval not causally covered")
    first = max(0, int(np.searchsorted(stamps, begin, side="right"))-1)
    # The real pair bracketing begin is also consumed by interpolation. Checking
    # only artificial [begin, interior stamps, end] knots can hide a long gap.
    used_stamps = stamps[first:]
    if len(used_stamps)>1 and np.max(np.diff(used_stamps))*1e-9 > max_gap:
        raise ValueError("real IMU interpolation bracket exceeds frozen gap")
    ids = np.flatnonzero((stamps > begin) & (stamps < end))
    knots = np.concatenate(([begin], stamps[ids], [end])).astype(np.int64)
    if np.max(np.diff(knots))*1e-9 > max_gap:
        raise ValueError("IMU gap exceeds frozen gate")
    # Subtract epoch before converting to float; never float-cast absolute ns.
    local_stamps = (stamps-begin)*1e-9
    samples = np.column_stack([np.interp((knots-begin)*1e-9, local_stamps, measurements[:, i])
                               for i in range(6)])
    R = np.eye(3); v = np.zeros(3); p = np.zeros(3)
    Jv = np.zeros((3, 3)); Jp = np.zeros((3, 3))
    for i, dt_ns in enumerate(np.diff(knots)):
        dt = dt_ns*1e-9
        a = .5*(samples[i, :3]+samples[i+1, :3])
        w = .5*(samples[i, 3:]+samples[i+1, 3:])-bg
        mid = R @ exp(w*dt*.5)
        force = mid @ a
        p += v*dt + .5*force*dt*dt
        Jp += Jv*dt + .5*mid*dt*dt
        v += force*dt; Jv += mid*dt
        R = R @ exp(w*dt)
    return R, v, p, Jv, Jp

def tangent(g):
    u = g / np.linalg.norm(g)
    axis = np.eye(3)[np.argmin(np.abs(u))]
    a = np.cross(u, axis); a /= np.linalg.norm(a)
    return np.column_stack((a, np.cross(u, a)))

def motion_system(poses, stamps, integrals, sigmas):
    """Standard p/v preintegration linear system for unknown per-knot v,g,ba."""
    n = len(poses); A = np.zeros((6*(n-1), 3*n+6)); y = np.zeros(6*(n-1))
    for i, (_, dv, dp, Jv, Jp) in enumerate(integrals):
        dt = (stamps[i+1]-stamps[i])*1e-9
        Ri = poses[i, :3, :3]; start=6*i
        A[start:start+3, 3*i:3*i+3] = np.eye(3)*dt
        A[start:start+3, 3*n:3*n+3] = .5*dt*dt*np.eye(3)
        A[start:start+3, 3*n+3:] = -Ri @ Jp
        y[start:start+3] = poses[i+1, :3, 3]-poses[i, :3, 3]-Ri @ dp
        A[start+3:start+6, 3*i:3*i+3] = -np.eye(3)
        A[start+3:start+6, 3*(i+1):3*(i+1)+3] = np.eye(3)
        A[start+3:start+6, 3*n:3*n+3] = -dt*np.eye(3)
        A[start+3:start+6, 3*n+3:] = Ri @ Jv
        y[start+3:start+6] = Ri @ dv
    W = np.repeat(np.asarray(sigmas).reshape(-1), 3)
    return A/W[:, None], y/W, A, y

def profile_diagnostics(A, n, g, scales, rotation_uncertainty, intervals, sigmas, rank_tol=1e-6):
    """No inverse for uncertified directions. No covariance-as-certainty shortcut."""
    Uv, sv, _ = np.linalg.svd(A[:, :3*n], full_matrices=True)
    rank_v = np.count_nonzero(sv > sv[0]*rank_tol)
    Q = Uv[:, rank_v:]
    D = np.diag(scales)
    B = Q.T @ np.column_stack((A[:, 3*n:3*n+3] @ tangent(g), A[:, 3*n+3:])) @ D
    _, s, Vt = np.linalg.svd(B, full_matrices=False)
    rank = int(np.count_nonzero(s > s[0]*rank_tol))
    # Conditional on stated orientation uncertainty: ||delta R|| <= 2 sin(theta/2).
    # Frobenius upper bound, nuisance velocity columns independent of rotation.
    # This is an uncertainty rejection certificate, NOT proof physical rank is zero.
    error_squared = 0.
    for dt, angle, (sp, sv_) in zip(intervals, rotation_uncertainty, sigmas):
        bound = min(2., 2*np.sin(min(np.pi, 3*angle)/2))
        error_squared += 3*scales[2]**2*bound**2*((.5*dt*dt/sp)**2+(dt/sv_)**2)
    error_bound = np.sqrt(error_squared)
    lower = max(0., float(s[-1]-error_bound))
    condition = float(s[0]/s[-1]) if s[-1] > 0 else None
    robust_condition = float((s[0]+error_bound)/lower) if lower else None
    return {"profile_rank": rank, "profile_dimension": 5, "velocity_nuisance_rank": int(rank_v),
            "singular_values": s.tolist(), "condition": condition,
            "normal_condition": condition**2 if condition else None,
            "perturbation_frobenius_bound": float(error_bound), "sigma_min_lower_bound": lower,
            "robust_condition": robust_condition, "weakest_scaled_direction": Vt[-1].tolist(),
            "rank_interpretation": "DIAGNOSTIC_ONLY; untrusted raw-scan poses; lower-bound-zero is not proof of physical rank deficiency"}
