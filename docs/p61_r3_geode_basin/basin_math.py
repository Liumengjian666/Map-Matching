"""Frozen-chart Schur algebra and time-only selection; no optimizer or GT."""
import numpy as np


def canonical(v):
    v = np.asarray(v, dtype=float).copy()
    return v if v[np.argmax(abs(v))] >= 0 else -v


def effective(H):
    H = np.asarray(H, dtype=float)
    if H.shape != (6, 6) or not np.isfinite(H).all():
        raise ValueError('invalid Hessian')
    if not np.allclose(H, H.T, rtol=1e-12, atol=1e-12):
        raise ValueError('non-symmetric Hessian')
    H = (H + H.T) / 2
    A, B, C = H[:3, :3], H[:3, 3:], H[3:, 3:]
    L = np.linalg.cholesky(C)
    if np.linalg.cond(C) > 1e12:
        raise ValueError('ill-conditioned rotational block')
    X = np.linalg.solve(L.T, np.linalg.solve(L, B.T))
    independent = np.linalg.solve(C, B.T)
    residual = np.linalg.norm(C @ X - B.T) / max(1., np.linalg.norm(B))
    if residual > 1e-10 or not np.allclose(X, independent, rtol=1e-10, atol=1e-12):
        raise ValueError('independent Schur solve failed')
    S = A - B @ X
    S = (S + S.T) / 2
    values, vectors = np.linalg.eigh(S)
    vectors = np.column_stack([canonical(vectors[:, k]) for k in range(3)])
    eigen_error = np.linalg.norm(S @ vectors - vectors @ np.diag(values)) / max(1., np.linalg.norm(S))
    if eigen_error > 1e-10:
        raise ValueError('effective eigenvector residual failed')
    # Congruence independently reconstructs the profiled quadratic energy.
    Q = np.vstack([np.eye(3), -X])
    congruence_error = np.linalg.norm(Q.T @ H @ Q - S) / max(1., np.linalg.norm(S))
    if congruence_error > 1e-10:
        raise ValueError('profiled energy identity failed')
    return {'S': S, 'X': X, 'eigenvalues': values, 'vectors': vectors,
            'tt_eigenvalues': np.linalg.eigvalsh(A), 'rr_condition': np.linalg.cond(C),
            'solve_residual': residual, 'congruence_error': congruence_error}


def uniform_rows(rows, count=20):
    if len(rows) < count:
        raise ValueError('not enough frozen pairs')
    stamps = np.array([int(r['stamp_ns']) for r in rows], dtype=np.int64)
    if np.any(np.diff(stamps) <= 0):
        raise ValueError('time order invalid')
    relative = (stamps - stamps[0]).astype(float)
    ids = [int(np.argmin(abs(relative - t))) for t in np.linspace(0, relative[-1], count)]
    if len(set(ids)) != count:
        raise ValueError('time-uniform duplicate')
    return [rows[k] for k in ids]


def perturbations(item):
    for family, direction, coupled in [('WEAK', item['vectors'][:, 0], False),
                                       ('STRONG', item['vectors'][:, 2], False),
                                       ('COUPLED_INITIAL', item['vectors'][:, 0], True)]:
        for amplitude in (.30, .80):
            for sign in (-1, 1):
                eta_t = sign * amplitude * direction / .8
                eta_r = -item['X'] @ eta_t if coupled else np.zeros(3)
                angle = np.linalg.norm(eta_r) * 180 / np.pi
                yield {'family': family, 'amplitude_m': amplitude, 'sign': sign,
                       'eta': np.r_[eta_t, eta_r], 'rotation_deg': angle,
                       'status': 'OUT_OF_BOUNDS' if angle > 5 else 'READY'}
