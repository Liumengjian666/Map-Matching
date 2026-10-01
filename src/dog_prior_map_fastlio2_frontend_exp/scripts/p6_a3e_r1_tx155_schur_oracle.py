#!/usr/bin/env python3
"""A3E-R1 OFFLINE ONLY: exact-binary-entry arbitrary-precision Schur forensics.

No estimator/rosbag imports, production solver calls, or numerical repair.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess

import mpmath as mp
import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
CAPSULE = PACKAGE / "docs/p6_alg_integration_a3d_r1/TX155_FAILURE_CAPSULE.npz"
EXPECTED_SHA = "b44fe28654437ff1cd944945cb436f62368c7ff315e9e920686836950aed58a8"
TRACE = PACKAGE / "docs/p6_alg_integration_a3d_r1/RUN_P3_200/FIRST_FAILURE_marginalization.csv"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sym(matrix):
    return (matrix + matrix.T) * 0.5


def exact(value):
    numerator, denominator = float(value).as_integer_ratio()
    return mp.mpf(numerator) / denominator


def high(matrix):
    return mp.matrix([[exact(value) for value in row] for row in np.atleast_2d(matrix)])


def quadratic(matrix, vector):
    return (vector.T * matrix * vector)[0]


def mp_sym(matrix):
    return (matrix + matrix.T) / 2


def mp_schur(matrix, marginalized):
    """Small active-support oracle only; no production helper/inverse of full H."""
    mm = matrix[:marginalized, :marginalized]
    mr = matrix[:marginalized, marginalized:]
    rr = matrix[marginalized:, marginalized:]
    columns = [mp.lu_solve(mm, mr[:, column]) for column in range(mr.cols)]
    correction = mp.matrix(mm.rows, mr.cols)
    for column, vector in enumerate(columns):
        correction[:, column] = vector
    return mp_sym(rr - mr.T * correction)


def mp_min(matrix):
    return mp.eigsy(mp_sym(matrix), eigvals_only=True)[0]


def recovered_scaled_schur(matrix, marginalized):
    """D=diag(sqrt(Hii)), B=D^-1 H D^-1; S=D_r S_B D_r."""
    diagonal = np.diag(matrix)
    if np.any(diagonal <= 0) or not np.isfinite(diagonal).all():
        raise RuntimeError("NONPOSITIVE_ACTIVE_DIAGONAL: no floor permitted")
    d = np.sqrt(diagonal)
    balanced = (matrix / d[:, None]) / d[None, :]
    mm = balanced[:marginalized, :marginalized]
    mr = balanced[:marginalized, marginalized:]
    small = balanced[marginalized:, marginalized:] - mr.T @ np.linalg.solve(mm, mr)
    recovered = sym((small * d[marginalized:, None]) * d[None, marginalized:])
    return recovered, mm, d


def synthetic_stress():
    """Two fixed scales, not a tuned search. Exact-entry factor theory is PSD."""
    rows = []
    transform, _ = np.linalg.qr(np.random.default_rng(155).normal(size=(3, 3)))
    rotation, _ = np.linalg.qr(np.random.default_rng(208).normal(size=(3, 3)))
    jacobian = np.column_stack((np.eye(3), -transform))
    for strong in (1e6, 1e18):
        weights = np.array([strong, strong * 0.1, 1e7 if strong == 1e18 else 1e5])
        weighted = np.sqrt(weights)[:, None] * rotation.T @ jacobian
        prior = np.diag([1e5, 20.0, 5.0] * 2)
        rounded = prior + weighted.T @ weighted
        direct = sym(rounded[3:, 3:] - rounded[:3, 3:].T @
                     np.linalg.solve(rounded[:3, :3], rounded[:3, 3:]))
        scaled, _, _ = recovered_scaled_schur(rounded, 3)
        with mp.workdps(100):
            factor = high(weighted)
            ideal = high(prior) + factor.T * factor
            theory = mp_schur(ideal, 3)
            capsule_equivalent = mp_schur(high(rounded), 3)
            rows.append(dict(relative_factor_scale=strong,
                unscaled_double_min=stats(direct)["lambda_min"],
                scaled_double_min=stats(scaled)["lambda_min"],
                rounded_entries_hp_min=mp.nstr(mp_min(capsule_equivalent), 85),
                exact_factor_hp_min=mp.nstr(mp_min(theory), 85),
                unscaled_relative_error_to_factor=float(mp.norm(high(direct) - theory) / mp.norm(theory)),
                scaled_relative_error_to_factor=float(mp.norm(high(scaled) - theory) / mp.norm(theory))))
    return rows


def stats(matrix):
    values = np.linalg.eigvalsh(sym(matrix))
    scale = max(abs(values[0]), abs(values[-1]))
    return dict(lambda_min=float(values[0]), lambda_max=float(values[-1]),
        spectral_norm=float(scale), frobenius=float(np.linalg.norm(matrix)),
        symmetry_max=float(np.max(np.abs(matrix - matrix.T))),
        relative_negative=float(max(0.0, -values[0]) / max(scale, np.finfo(float).eps)),
        negative_count=int(np.count_nonzero(values < 0)))


def load():
    if sha(CAPSULE) != EXPECTED_SHA:
        raise RuntimeError("TX155_CAPSULE_SHA_MISMATCH")
    with np.load(CAPSULE, allow_pickle=False) as source:
        arrays = {name: source[name] for name in source.files}
    required = {"incoming_prior_information", "incoming_prior_gradient", "charted_prior_information",
        "charted_prior_gradient", "consumed_hessian", "consumed_gradient", "imu_hessian",
        "lidar_hessian", "visual_hessian", "correction_h", "correction_b"}
    if set(arrays) != required or any(value.dtype != np.float64 or not np.isfinite(value).all()
                                    for value in arrays.values()):
        raise RuntimeError("TX155_CAPSULE_RECONSTRUCTION_MISMATCH: schema/finite")
    h = arrays["consumed_hessian"]
    g = arrays["consumed_gradient"].reshape(-1)
    if h.shape != (615, 615) or g.shape != (615,):
        raise RuntimeError("TX155_CAPSULE_RECONSTRUCTION_MISMATCH: dimensions")
    hmm, hmr, hrr = h[:15, :15], h[:15, 15:], h[15:, 15:]
    raw = hrr - hmr.T @ arrays["correction_h"]
    gradient = g[15:] - hmr.T @ arrays["correction_b"].reshape(-1)
    with TRACE.open() as source:
        row = next(csv.DictReader(source))
    if (int(row["transaction_id"]) != 155 or int(row["event_stamp_ns"]) != 1517157234720485283
            or int(row["enforcement_index"]) != 208 or int(row["attempt_index"]) != 1
            or float(row["solve_jitter"]) != 0):
        raise RuntimeError("TX155_CAPSULE_RECONSTRUCTION_MISMATCH: event identity")
    trace_checks = {
        "new_prior_lambda_min": stats(raw)["lambda_min"],
        "new_prior_lambda_max": stats(raw)["lambda_max"],
        "raw_schur_symmetry_max_abs": stats(raw)["symmetry_max"],
        "new_gradient_norm": float(np.linalg.norm(gradient)),
        "new_gradient_max_abs": float(np.max(np.abs(gradient))),
    }
    for name, actual in trace_checks.items():
        if not np.isclose(actual, float(row[name]), rtol=1e-10, atol=1e-10):
            raise RuntimeError(f"TX155_CAPSULE_RECONSTRUCTION_MISMATCH: {name}")
    # Exact-zero inactive rows are removed only from forensic computation, not
    # by an eigenvalue/rank threshold. This is not a production rank policy.
    active = np.flatnonzero(np.any(h != 0.0, axis=1))
    if not np.array_equal(active, np.arange(30)):
        raise RuntimeError("unexpected capsule structural support")
    return arrays, h, g, hmm, hmr, hrr, raw, gradient, row


def directional(hmm, hmr, hrr, direction):
    v = high(direction.reshape(-1, 1))
    v /= mp.sqrt(quadratic(mp.eye(len(direction)), v))
    y = hmr * v
    z = mp.lu_solve(hmm, y)
    a = quadratic(hrr, v)
    b = (y.T * z)[0]
    lifted = mp.matrix(list(-z) + list(v))
    return v, z, lifted, a, b, a - b


def self_test():
    with mp.workdps(100):
        # Exact integer SPD full matrix; exact known Schur [[3,1],[1,4]].
        hmm = mp.matrix([[4, 0], [0, 9]])
        hmr = mp.matrix([[2, 0], [0, 3]])
        target = mp.matrix([[3, 1], [1, 4]])
        hrr = target + hmr.T * mp.inverse(hmm) * hmr
        h = mp.matrix([[4,0,2,0], [0,9,0,3], [2,0,4,1], [0,3,1,5]])
        assert mp.norm(mp_schur(h, 2) - target) < mp.mpf("1e-90")
        for direction in (np.array([1.0, 0.0]), np.array([0.0, 1.0]), np.array([1.0, 2.0])):
            v, _, x, _, _, q = directional(hmm, hmr, hrr, direction)
            assert abs(q - quadratic(target, v)) < mp.mpf("1e-90")
            assert abs(q - quadratic(h, x)) < mp.mpf("1e-90") and q > 0
        assert exact(np.nextafter(1.0, 2.0)) - 1 == mp.mpf(2) ** -52
        eigen = mp.eigsy(hmm, eigvals_only=True)
        assert eigen[eigen.rows - 1] == 9
        assert eigen[eigen.rows - 1] / eigen[0] == mp.mpf(9) / 4
        scaled, _, _ = recovered_scaled_schur(np.array(h.tolist(), dtype=float), 2)
        assert np.linalg.norm(scaled - np.array(target.tolist(), dtype=float)) < 1e-14
    stress = synthetic_stress()
    assert all(float(row["exact_factor_hp_min"]) > 0 for row in stress)
    print("A3E_R1_HIGH_PRECISION_ORACLE_SELF_TEST_PASS", flush=True)


def write_csv(path, rows):
    with path.open("w", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def run(output):
    arrays, h, g, hmm, hmr, hrr, raw, gradient, trace = load()
    output.mkdir(parents=True, exist_ok=False)
    print("TX155_CAPSULE_RECONSTRUCTION_PASS", flush=True)
    s = sym(raw)
    values, vectors = np.linalg.eigh(s[:15, :15])
    indices = list(np.flatnonzero(values < 0))
    indices += list(np.flatnonzero(values > 0)[:3])
    directions, components = [], []
    active_h = h[:30, :30]
    scaled_s, scaled_mm, scaling = recovered_scaled_schur(active_h, 15)
    independent_s = sym(hrr[:15, :15] - hmr[:, :15].T @ np.linalg.solve(hmm, hmr[:, :15]))
    cancellation = []
    with mp.workdps(100):
        mh = high(h[:30, :30]); mm = high(hmm); mr = high(hmr[:, :15]); rr = high(hrr[:15, :15])
        component_matrices = {name: high(arrays[key][:30, :30]) for name, key in (
            ("prior", "charted_prior_information"), ("imu", "imu_hessian"),
            ("lidar", "lidar_hessian"), ("visual", "visual_hessian"))}
        remainder = mh - sum(component_matrices.values(), mp.zeros(30))
        schur_hp = mp_schur(mh, 15)
        diagonal_mp = mp.diag([exact(value) for value in scaling])
        diagonal_inverse_mp = mp.diag([1 / exact(value) for value in scaling])
        balanced_mp = diagonal_inverse_mp * mh * diagonal_inverse_mp
        balanced_s_mp = mp_schur(balanced_mp, 15)
        retained_diagonal = diagonal_mp[15:, 15:]
        recovered_mp = retained_diagonal * balanced_s_mp * retained_diagonal
        scaling_equivalence_error = mp.norm(recovered_mp - schur_hp) / mp.norm(schur_hp)
        if scaling_equivalence_error > mp.mpf("1e-65"):
            raise RuntimeError("HIGH_PRECISION_SCALING_CONGRUENCE_MISMATCH")
        for index in indices:
            v, z, x, a, b, q = directional(mm, mr, rr, vectors[:, index])
            q_full = quadratic(mh, x)
            terms = {name: quadratic(matrix, x) for name, matrix in component_matrices.items()}
            remainder_q = quadratic(remainder, x)
            summed = sum(terms.values()) + remainder_q
            if abs(q - q_full) > mp.mpf("1e-65") or abs(summed - q_full) > mp.mpf("1e-65"):
                raise RuntimeError("HIGH_PRECISION_DIRECTIONAL_ACCOUNTING_MISMATCH")
            label = f"{'negative' if values[index] < 0 else 'positive'}_{index}"
            text = lambda value: mp.nstr(value, 85)
            directions.append(dict(mode=label, production_eigenvalue=values[index],
                precision_digits=100, a_hp=text(a), b_hp=text(b), schur_q_hp=text(q),
                lifted_Hc_q_hp=text(q_full), identity_error=text(q - q_full),
                scalar_cancellation_ratio=text(max(abs(a), abs(b)) / max(abs(q), mp.eps)),
                production_minus_hp=values[index] - float(q),
                scaled_double_q=float(vectors[:, index] @ scaled_s @ vectors[:, index]),
                scaled_double_minus_hp=float(vectors[:, index] @ scaled_s @ vectors[:, index]) - float(q)))
            components.append(dict(mode=label, **{name + "_q_hp": text(value) for name, value in terms.items()},
                assembly_rounding_remainder_q_hp=text(remainder_q),
                component_sum_without_remainder_hp=text(sum(terms.values())),
                component_sum_with_remainder_hp=text(summed), consumed_Hc_q_hp=text(q_full)))
            print(label, "production=", values[index], "HP=", text(q), "components=",
                  {name: float(value) for name, value in terms.items()}, "assembly=", float(remainder_q), flush=True)
            cancellation.append(dict(kind="directional", mode=label,
                Hrr_quantity=text(a), correction_quantity=text(b), Schur_quantity=text(q),
                cancellation_ratio=text(max(abs(a), abs(b)) / max(abs(q), mp.eps))))
        eigen_mm = mp.eigsy(mp_sym(mm), eigvals_only=True)
        condition_hp = eigen_mm[eigen_mm.rows - 1] / eigen_mm[0]
        if eigen_mm[0] <= 0 or condition_hp <= 1:
            raise RuntimeError("unexpected capsule Hmm SPD/condition")
        component_sum = sum((mp_sym(matrix) for matrix in component_matrices.values()), mp.zeros(30))
        sum_s = mp_schur(component_sum, 15)
        # Forensic-only high-precision Q diag(max(lambda,0)) Q^T. Only tiny
        # negative modes are replaced; positive modes are preserved at 100 dps.
        cleaned_sum = mp.zeros(30)
        cleanup_rows = []
        for name, matrix in component_matrices.items():
            eig, vec = mp.eigsy(mp_sym(matrix))
            scale = max(abs(eig[0]), abs(eig[eig.rows - 1]))
            band = exact(np.finfo(float).eps) * 30 * scale
            negatives = [value for value in eig if value < 0]
            if any(value < -band for value in negatives):
                raise RuntimeError("component has negative mode outside declared roundoff band")
            positive_weights = mp.diag([max(mp.mpf(0), value) for value in eig])
            cleaned = vec * positive_weights * vec.T
            cleaned_sum += cleaned
            cleanup_rows.append(dict(component=name, roundoff_band=text(band),
                negative_modes=len(negatives), original_min=text(eig[0]),
                relative_entry_perturbation=float(mp.norm(cleaned - matrix) / max(mp.norm(matrix), mp.eps)),
                relative_change_to_symmetric_component=text(mp.norm(cleaned - mp_sym(matrix)) /
                    max(mp.norm(mp_sym(matrix)), mp.eps))))
        cleaned_s = mp_schur(cleaned_sum, 15)
        summary_hp = dict(Hmm_lambda_min=text(eigen_mm[0]), Hmm_lambda_max=text(eigen_mm[eigen_mm.rows - 1]),
            Hmm_condition=text(condition_hp), condition_times_double_epsilon=text(condition_hp * exact(np.finfo(float).eps)),
            rounded_Hc_hp_S_min=text(mp_min(mp_schur(mh, 15))),
            exact_component_sum_hp_S_min=text(mp_min(sum_s)),
            PSD_component_oracle_hp_S_min=text(mp_min(cleaned_s)),
            assembly_remainder_frobenius=text(mp.norm(remainder)),
            assembly_remainder_symmetric_frobenius=text(mp.norm(mp_sym(remainder))),
            assembly_remainder_skew_frobenius=text(mp.norm((remainder - remainder.T) / 2)),
            assembly_remainder_relative=text(mp.norm(remainder) / mp.norm(mh)),
            scaling_congruence_relative_error=text(scaling_equivalence_error),
            production_schur_relative_error_to_hp=text(mp.norm(high(s[:15, :15]) - schur_hp) / mp.norm(schur_hp)),
            scaled_double_relative_error_to_hp=text(mp.norm(high(scaled_s) - schur_hp) / mp.norm(schur_hp)),
            cleanup_components=cleanup_rows,
            cleaned_plus_original_assembly_remainder_hp_S_min=text(mp_min(mp_schur(cleaned_sum + mp_sym(remainder), 15))))
        hc_eigen = mp.eigsy(mh, eigvals_only=True)
        summary_hp.update(dict(Hc_lambda_min=text(hc_eigen[0]),
            Hc_lambda_max=text(hc_eigen[hc_eigen.rows - 1]),
            Hc_relative_negative=text(max(mp.mpf(0), -hc_eigen[0]) / hc_eigen[hc_eigen.rows - 1])))
        summary_hp["rounded_Hc_hp_S_min_120_digit_check"] = None
        with mp.workdps(120):
            # Reconvert entries, not a precision increase of an already-rounded solution.
            check = mp_schur(high(active_h), 15)
            summary_hp["rounded_Hc_hp_S_min_120_digit_check"] = mp.nstr(mp_min(check), 85)
        if abs(mp.mpf(summary_hp["rounded_Hc_hp_S_min"]) -
               mp.mpf(summary_hp["rounded_Hc_hp_S_min_120_digit_check"])) > mp.mpf("1e-65"):
            raise RuntimeError("100_VS_120_DIGIT_STABILITY_MISMATCH")
        exact_correction = mp.matrix(15, 15)
        for column in range(15):
            exact_correction[:, column] = mp.lu_solve(mm, mr[:, column])
        gm = high(g[:15].reshape(-1, 1))
        exact_correction_b = mp.lu_solve(mm, gm)
        stored_correction = high(arrays["correction_h"][:, :15])
        stored_b = high(arrays["correction_b"].reshape(15, 1))
        exact_gradient = high(g[15:30].reshape(15, 1)) - mr.T * exact_correction_b
        summary_hp.update(dict(
            production_X_rhs_relative_backward_error=text(mp.norm(mm * stored_correction - mr) / mp.norm(mr)),
            production_y_rhs_relative_backward_error=text(mp.norm(mm * stored_b - gm) / mp.norm(gm)),
            production_X_relative_forward_error=text(mp.norm(stored_correction - exact_correction) / mp.norm(exact_correction)),
            production_y_relative_forward_error=text(mp.norm(stored_b - exact_correction_b) / mp.norm(exact_correction_b)),
            production_gradient_relative_error_to_hp=text(
                mp.norm(high(gradient[:15].reshape(15, 1)) - exact_gradient) / mp.norm(exact_gradient))))
        eig, vec = mp.eigsy(mp_sym(mm))
        maximum = max(abs(value) for value in eig)
        rank_rows = []
        for label, relative in (("machine_eps_times_dimension", exact(np.finfo(float).eps) * 15),
                                ("1e-14", mp.mpf("1e-14")), ("1e-12", mp.mpf("1e-12")),
                                ("1e-10", mp.mpf("1e-10"))):
            supported = [abs(value) > relative * maximum for value in eig]
            inverse = vec * mp.diag([1 / value if keep else 0 for value, keep in zip(eig, supported)]) * vec.T
            projector = vec * mp.diag([0 if keep else 1 for keep in supported]) * vec.T
            gm = high(g[:15].reshape(-1, 1))
            generalized = mp_sym(rr - mr.T * inverse * mr)
            generalized_eig = mp.eigsy(generalized, eigvals_only=True)
            rank_rows.append(dict(threshold=label, relative_threshold=text(relative),
                absolute_threshold=text(relative * maximum), rank=sum(supported), nullity=15-sum(supported),
                Hmr_range_residual=text(mp.norm(projector * mr) / mp.norm(mr)),
                gm_range_residual=text(mp.norm(projector * gm) / mp.norm(gm)),
                schur_lambda_min=text(generalized_eig[0]),
                schur_lambda_max=text(generalized_eig[generalized_eig.rows - 1]), forensic_only=True))
    write_csv(output / "TX155_HIGH_PRECISION_DIRECTIONAL_ORACLE.csv", directions)
    write_csv(output / "TX155_COMPONENT_DIRECTIONAL_AUDIT.csv", components)
    correction = hmr.T @ arrays["correction_h"]
    for norm, function in (("spectral", lambda a: np.linalg.norm(a, 2)),
                           ("frobenius", lambda a: np.linalg.norm(a))):
        a, b, c = function(hrr[:15, :15]), function(correction[:15, :15]), function(s[:15, :15])
        cancellation.append(dict(kind=norm, mode="whole_active_schur", Hrr_quantity=a,
            correction_quantity=b, Schur_quantity=c, cancellation_ratio=max(a, b) / c))
    write_csv(output / "TX155_CANCELLATION_AUDIT.csv", cancellation)
    scaling_rows = [dict(method=name, **stats(matrix),
        relative_error_to_production=np.linalg.norm(matrix - s[:15, :15]) / np.linalg.norm(s[:15, :15]))
        for name, matrix in (("captured_production", s[:15, :15]),
                             ("independent_unscaled_double", independent_s),
                             ("diagonally_equilibrated_double_recovered", scaled_s))]
    write_csv(output / "TX155_SCALING_ORACLE.csv", scaling_rows)
    write_csv(output / "TX155_RANK_SENSITIVITY.csv", rank_rows)
    write_csv(output / "SYNTHETIC_SCALE_STRESS.csv", synthetic_stress())
    (output / "reconstruction.json").write_text(json.dumps(dict(capsule_sha=sha(CAPSULE),
        script_sha=sha(Path(__file__)), precision_digits=100, structural_active_indices=list(range(30)),
        source_git_sha=subprocess.check_output(["git", "-C", str(PACKAGE), "rev-parse", "HEAD"], text=True).strip(),
        numpy_version=np.__version__, mpmath_version=mp.__version__,
        solver="mpmath 100-digit LU (120-digit Schur/eigen stability check); NumPy LAPACK double solve",
        production_schur_stats=stats(raw), trace_checks="PASS",
        production_gradient_norm=float(np.linalg.norm(gradient)),
        scaled_Hmm_stats=stats(scaled_mm), scaled_Hmm_condition=np.linalg.cond(scaled_mm),
        scaling_convention="D=diag(sqrt(Hii)), B=D^-1 H D^-1, S=D_r S_B D_r; exact disconnected zero rows use identity",
        scaling_min=float(np.min(scaling)), scaling_max=float(np.max(scaling)),
        high_precision=summary_hp, original_trace=trace,
        REAL_REPLAY_COUNT=0, PRODUCTION_CLAMP_USED=False), indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.self_test:
        self_test()
    elif args.output:
        run(args.output)
    else:
        parser.error("select --self-test or --output; offline only")
