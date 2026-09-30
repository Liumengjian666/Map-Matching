#!/usr/bin/env python3
"""Independent NumPy oracle for the A3C-R1 failure-matrix capsule."""

import argparse
import hashlib
import json
import math
import struct
from pathlib import Path

import numpy as np


MAGIC = b"P6A3CR1CAPSULE\0\0"


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_capsule(path):
    arrays = {}
    with open(path, "rb") as stream:
        if stream.read(16) != MAGIC:
            raise ValueError("unexpected A3C-R1 capsule magic")
        version, count = struct.unpack("<II", stream.read(8))
        if version != 1:
            raise ValueError("unsupported A3C-R1 capsule version")
        for _ in range(count):
            (name_size,) = struct.unpack("<I", stream.read(4))
            name = stream.read(name_size).decode("utf-8")
            rows, columns = struct.unpack("<QQ", stream.read(16))
            element_count = rows * columns
            payload = stream.read(element_count * 8)
            if len(payload) != element_count * 8:
                raise ValueError("truncated matrix capsule array: " + name)
            arrays[name] = np.frombuffer(payload, dtype="<f8").copy().reshape(
                (rows, columns))
        if stream.read(1):
            raise ValueError("unexpected trailing bytes in matrix capsule")
    return arrays


def pack_capsule(input_path, output_path, metadata_path):
    arrays = read_capsule(input_path)
    np.savez_compressed(output_path, **arrays)
    metadata = {
        "source_path": str(Path(input_path).resolve()),
        "source_sha256": sha256(input_path),
        "source_byte_size": Path(input_path).stat().st_size,
        "capsule_path": str(Path(output_path).resolve()),
        "capsule_sha256": sha256(output_path),
        "capsule_byte_size": Path(output_path).stat().st_size,
        "arrays": {
            name: {"shape": list(value.shape), "dtype": str(value.dtype)}
            for name, value in arrays.items()
        },
        "format": "P6A3CR1CAPSULE_v1 converted to compressed NPZ",
        "matrix_order_in_source": "row-major float64",
    }
    Path(metadata_path).write_text(json.dumps(metadata, indent=2) + "\n",
                                   encoding="utf-8")
    print(json.dumps(metadata, indent=2))


def matrix_stats(matrix, rank_threshold=None):
    matrix = np.asarray(matrix, dtype=np.float64)
    if matrix.ndim != 2 or matrix.shape[0] != matrix.shape[1]:
        return {"available": False, "shape": list(matrix.shape)}
    if matrix.shape[0] == 0:
        return {"available": False, "shape": list(matrix.shape)}
    finite = bool(np.isfinite(matrix).all())
    result = {
        "available": True,
        "finite": finite,
        "dimension": int(matrix.shape[0]),
        "symmetry_fro": float(np.linalg.norm(matrix - matrix.T, ord="fro")),
        "frobenius": float(np.linalg.norm(matrix, ord="fro")),
    }
    if not finite:
        return result
    values = np.linalg.eigvalsh(0.5 * (matrix + matrix.T))
    scale = float(np.max(np.abs(values)))
    result.update({
        "lambda_min": float(values[0]),
        "lambda_max": float(values[-1]),
        "min_abs_eigenvalue": float(np.min(np.abs(values))),
        "spectral_scale": scale,
        "relative_negative": float(max(0.0, -values[0]) /
                                    max(scale, np.finfo(float).eps)),
        "negative_eigenvalue_count": int(np.count_nonzero(values < 0.0)),
    })
    if rank_threshold is None:
        rank_threshold = np.finfo(float).eps * matrix.shape[0] * scale
    result["rank_threshold"] = float(rank_threshold)
    result["numerical_rank"] = int(np.count_nonzero(np.abs(values) >
                                                   rank_threshold))
    result["nullity"] = int(matrix.shape[0] - result["numerical_rank"])
    return result


def vector_stats(vector):
    vector = np.asarray(vector, dtype=np.float64).reshape(-1)
    return {
        "finite": bool(np.isfinite(vector).all()),
        "norm": float(np.linalg.norm(vector)),
        "max_abs": float(np.max(np.abs(vector))) if vector.size else 0.0,
        "size": int(vector.size),
    }


def relative_error(left, right):
    return float(np.linalg.norm(left - right, ord="fro") /
                 max(np.linalg.norm(right, ord="fro"),
                     np.finfo(float).eps))


def read_key_values(path):
    values = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key] = value
    return values


def generalized_schur(hmm, hmr, hrr, gm, gr, relative_tau=None):
    values, vectors = np.linalg.eigh(0.5 * (hmm + hmm.T))
    scale = float(np.max(np.abs(values))) if values.size else 0.0
    if relative_tau is None:
        tau = np.finfo(float).eps * hmm.shape[0] * scale
    else:
        tau = relative_tau * scale
    supported = np.abs(values) > tau
    null = ~supported
    inverse = np.zeros_like(values)
    inverse[supported] = 1.0 / values[supported]
    pinv = (vectors * inverse) @ vectors.T
    range_h = vectors[:, null].T @ hmr
    range_g = vectors[:, null].T @ gm
    return {
        "threshold": float(tau),
        "rank": int(np.count_nonzero(supported)),
        "nullity": int(np.count_nonzero(null)),
        "range_residual_hmr": float(np.linalg.norm(range_h, ord="fro") /
                                      max(np.linalg.norm(hmr, ord="fro"),
                                          np.finfo(float).eps)),
        "range_residual_gm": float(np.linalg.norm(range_g) /
                                    max(np.linalg.norm(gm),
                                        np.finfo(float).eps)),
        "schur": 0.5 * ((hrr - hmr.T @ pinv @ hmr) +
                        (hrr - hmr.T @ pinv @ hmr).T),
        "gradient": gr - hmr.T @ pinv @ gm,
    }


def analyze_capsule(npz_path, package_metadata_path, failure_summary_path,
                    summary_path):
    package_metadata = json.loads(Path(package_metadata_path).read_text(
        encoding="utf-8"))
    metadata = read_key_values(failure_summary_path)
    with np.load(npz_path) as archive:
        arrays = {name: archive[name] for name in archive.files}
    required = {"incoming_prior_information", "incoming_prior_gradient",
                "charted_prior_information", "charted_prior_gradient",
                "consumed_hessian", "consumed_gradient", "imu_hessian",
                "lidar_hessian", "visual_hessian", "correction_h",
                "correction_b"}
    missing = required - arrays.keys()
    if missing:
        raise ValueError("missing NPZ arrays: " + ",".join(sorted(missing)))

    hessian = arrays["consumed_hessian"]
    gradient = arrays["consumed_gradient"].reshape(-1)
    prior = arrays["incoming_prior_information"]
    charted_prior = arrays["charted_prior_information"]
    hdim = int(metadata.get("marginalized_dimension", "15"))
    if hessian.shape[0] <= hdim or hessian.shape[0] != hessian.shape[1]:
        raise ValueError("invalid consumed Hessian shape")
    hmm = hessian[:hdim, :hdim]
    hmr = hessian[:hdim, hdim:]
    hrr = hessian[hdim:, hdim:]
    gm = gradient[:hdim]
    gr = gradient[hdim:]
    correction_h = arrays["correction_h"]
    correction_b = arrays["correction_b"].reshape(-1)
    solve_jitter = float(metadata.get("solve_jitter", "0"))
    production_schur = 0.5 * ((hrr - hmr.T @ correction_h) +
                              (hrr - hmr.T @ correction_h).T)
    production_gradient = gr - hmr.T @ correction_b
    # Production explicitly symmetrizes H_mm before its LDLT solve.
    solve_matrix = 0.5 * (hmm + hmm.T) + solve_jitter * np.eye(hdim)
    x_error = float(np.linalg.norm(solve_matrix @ correction_h - hmr,
                                   ord="fro") /
                    max(np.linalg.norm(hmr, ord="fro"),
                        np.finfo(float).eps))
    y_error = float(np.linalg.norm(solve_matrix @ correction_b - gm) /
                    max(np.linalg.norm(gm), np.finfo(float).eps))

    primary = generalized_schur(hmm, hmr, hrr, gm, gr)
    sensitivity = {}
    for relative_tau in (1e-14, 1e-12, 1e-10):
        sensitivity[f"relative_{relative_tau:.0e}"] = generalized_schur(
            hmm, hmr, hrr, gm, gr, relative_tau)
    solve_schur = None
    solve_gradient = None
    try:
        solve_x = np.linalg.solve(solve_matrix, hmr)
        solve_y = np.linalg.solve(solve_matrix, gm)
        solve_schur = 0.5 * ((hrr - hmr.T @ solve_x) +
                            (hrr - hmr.T @ solve_x).T)
        solve_gradient = gr - hmr.T @ solve_y
    except np.linalg.LinAlgError:
        pass

    factor_hessian = (arrays["imu_hessian"] + arrays["lidar_hessian"] +
                      arrays["visual_hessian"])
    incoming_prior_gradient = arrays["incoming_prior_gradient"].reshape(-1)
    charted_prior_gradient = arrays["charted_prior_gradient"].reshape(-1)
    touching_gradient = gradient - charted_prior_gradient
    results = {
        "capsule_npz": str(Path(npz_path).resolve()),
        "capsule_sha256": sha256(npz_path),
        "capsule_byte_size": Path(npz_path).stat().st_size,
        "packed_capsule_metadata": package_metadata,
        "npz_arrays": {name: {"shape": list(value.shape),
                              "dtype": str(value.dtype)}
                       for name, value in arrays.items()},
        "incoming_prior": matrix_stats(prior),
        "prior_gradient_stages": {
            "incoming": vector_stats(incoming_prior_gradient),
            "charted": vector_stats(charted_prior_gradient),
            "touching_factors": vector_stats(touching_gradient),
            "consumed": vector_stats(gradient),
        },
        "charted_prior": matrix_stats(charted_prior),
        "touching_factors": matrix_stats(hessian - charted_prior),
        "factor_components": {
            "imu": matrix_stats(arrays["imu_hessian"]),
            "lidar": matrix_stats(arrays["lidar_hessian"]),
            "visual": matrix_stats(arrays["visual_hessian"]),
            "component_sum": matrix_stats(factor_hessian),
        },
        "H_consumed": matrix_stats(hessian),
        "H_mm": matrix_stats(hmm),
        "H_mm_rank_sensitivities": {
            key: {"rank": value["rank"], "nullity": value["nullity"],
                  "threshold": value["threshold"],
                  "range_residual_hmr": value["range_residual_hmr"],
                  "range_residual_gm": value["range_residual_gm"]}
            for key, value in {"primary_machine": primary,
                               **sensitivity}.items()
        },
        "solve": {"jitter": solve_jitter,
                  "H_mr_backward_error": x_error,
                  "g_m_backward_error": y_error,
                  "production_correction_h_shape": list(correction_h.shape),
                  "production_correction_b": vector_stats(correction_b)},
        "production_schur": matrix_stats(production_schur),
        "production_schur_gradient": vector_stats(production_gradient),
        "generalized_schur_primary": matrix_stats(primary["schur"]),
        "generalized_schur_gradient": vector_stats(primary["gradient"]),
        "production_vs_generalized_schur": {
            "H_relative_frobenius_error": relative_error(
                production_schur, primary["schur"]),
            "gradient_relative_error": float(
                np.linalg.norm(production_gradient - primary["gradient"]) /
                max(np.linalg.norm(primary["gradient"]),
                    np.finfo(float).eps)),
        },
        "production_vs_jittered_solve_schur": None,
    }
    if solve_schur is not None:
        results["production_vs_jittered_solve_schur"] = {
            "H_relative_frobenius_error": relative_error(
                production_schur, solve_schur),
            "gradient_relative_error": float(
                np.linalg.norm(production_gradient - solve_gradient) /
                max(np.linalg.norm(solve_gradient), np.finfo(float).eps)),
            "solve_schur": matrix_stats(solve_schur),
            "solve_gradient": vector_stats(solve_gradient),
        }
    summary = json.dumps(results, indent=2, allow_nan=False) + "\n"
    Path(summary_path).write_text(summary, encoding="utf-8")
    print(summary)


def self_test():
    # A: SPD Schur equals a direct solve and is PSD.
    hmm = np.diag([2.0, 3.0])
    hmr = np.array([[0.2], [0.3]])
    hrr = np.array([[1.0]])
    gm = np.array([0.1, -0.2])
    gr = np.array([0.4])
    spd = generalized_schur(hmm, hmr, hrr, gm, gr)
    expected = hrr - hmr.T @ np.linalg.solve(hmm, hmr)
    assert spd["rank"] == 2 and spd["nullity"] == 0
    assert np.allclose(spd["schur"], expected, rtol=1e-13, atol=1e-13)

    # B: PSD rank-deficient Hmm with consistent cross/gradient ranges.
    hmm = np.diag([2.0, 1.0, 0.0])
    hmr = np.array([[0.2], [0.3], [0.0]])
    hrr = np.array([[1.0]])
    gm = np.array([0.1, -0.2, 0.0])
    gr = np.array([0.4])
    deficient = generalized_schur(hmm, hmr, hrr, gm, gr)
    assert deficient["rank"] == 2 and deficient["nullity"] == 1
    assert deficient["range_residual_hmr"] == 0.0
    assert deficient["range_residual_gm"] == 0.0
    assert matrix_stats(deficient["schur"])["lambda_min"] >= -1e-14
    inconsistent = generalized_schur(
        hmm, np.array([[0.2], [0.3], [1e-5]]), hrr, gm, gr)
    assert inconsistent["range_residual_hmr"] > 1e-7

    # C: an absolute pivot gate can pass while relative conditioning is tiny.
    disparity = np.diag([1e8, 1e-10])
    values = np.linalg.eigvalsh(disparity)
    assert np.min(np.abs(values)) > 1e-12
    assert np.min(np.abs(values)) / np.max(np.abs(values)) < 1e-16
    assert matrix_stats(disparity)["numerical_rank"] == 1

    # D: signed negative direction/pivot is visible, not hidden by abs().
    indefinite = np.array([[1.0, 2.0], [2.0, 1.0]])
    assert matrix_stats(indefinite)["lambda_min"] < 0.0
    assert np.linalg.eigvalsh(indefinite)[0] == -1.0

    # E: repeated rank-deficient eliminations keep explicit prior history.
    hprior_0 = np.diag([1.0, 0.0, 0.0])
    hprior_1 = np.zeros((2, 2))
    hprior_2 = np.zeros((1, 1))
    assert matrix_stats(hprior_0)["numerical_rank"] == 1
    assert matrix_stats(hprior_1)["lambda_min"] == 0.0
    assert matrix_stats(hprior_2)["lambda_min"] == 0.0
    print("A3C_R1_ORACLE_MATH_SELF_TEST_PASS")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--input-bin")
    parser.add_argument("--output-npz")
    parser.add_argument("--metadata")
    parser.add_argument("--analyze-npz")
    parser.add_argument("--failure-summary")
    parser.add_argument("--summary-output")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if args.input_bin:
        if not args.output_npz or not args.metadata:
            parser.error("packing requires --output-npz and --metadata")
        pack_capsule(args.input_bin, args.output_npz, args.metadata)
        return
    if args.analyze_npz:
        if not args.metadata or not args.failure_summary or not args.summary_output:
            parser.error("analysis requires --metadata, --failure-summary, and --summary-output")
        analyze_capsule(args.analyze_npz, args.metadata,
                        args.failure_summary, args.summary_output)
        return
    parser.error("choose --self-test, --input-bin, or --analyze-npz")


if __name__ == "__main__":
    main()
