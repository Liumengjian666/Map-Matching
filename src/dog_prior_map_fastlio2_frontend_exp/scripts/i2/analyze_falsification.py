#!/usr/bin/env python3
"""Offline standard subspace metrics; no trajectory/GT or detector tuning."""
import argparse
import csv
import json
import time
import tempfile
from collections import Counter
from pathlib import Path

import numpy as np


def mat(row, name):
    return np.array([float(row[f"{name}{i}"]) for i in range(36)]).reshape(6, 6)


def orth(basis):
    if basis.shape[1] == 0:
        return basis
    q, s, _ = np.linalg.svd(basis, full_matrices=False)
    if np.min(s) <= 6 * np.finfo(float).eps * np.max(s):
        raise ValueError("rank-deficient transported basis")
    return q


def compare(a, b):
    a, b = orth(a), orth(b)
    overlap = np.linalg.svd(a.T @ b, compute_uv=False)
    angles = np.rad2deg(np.arccos(np.clip(overlap, 0, 1)))
    # Empty-vs-nonempty angles are undefined, NOT an artificial zero degree success.
    return dict(dimension=a.shape[1], reference_dimension=b.shape[1],
                dimension_match=a.shape[1] == b.shape[1],
                angles_deg=angles.tolist(), max_angle_deg=float(max(angles)) if angles.size else None,
                projection_distance=float(np.linalg.norm(a @ a.T - b @ b.T)),
                false_weak_dimension=max(a.shape[1] - b.shape[1], 0),
                missed_weak_dimension=max(b.shape[1] - a.shape[1], 0))


def weak(h, ratio=.05, inclusive=False):
    eigenvalues, vectors = np.linalg.eigh((h + h.T) / 2)
    floor = 100 * np.finfo(float).eps * max(1., abs(eigenvalues[-1]))
    if not np.isfinite(eigenvalues).all() or eigenvalues[0] < -floor:
        raise ValueError("indefinite spectrum")
    threshold = max(floor, ratio * eigenvalues[-1])
    mask = eigenvalues <= threshold if inclusive else eigenvalues < threshold
    return vectors[:, mask]


def adaptive(h, length, ratio, inclusive=False):
    """FMCW arXiv:2603.10248v1 Eqs32–38, reordered [rotation,translation].

    Equation reproduction, NOT official runtime. No new scaling is introduced.
    Schur solves and back-transport are explicit; common chart is [rot,t/L].
    """
    rr, rt, tt = h[:3, :3], h[:3, 3:], h[3:, 3:]
    sr = rr - rt @ np.linalg.solve(tt, rt.T)
    st = tt - rt.T @ np.linalg.solve(rr, rt)
    mr = np.linalg.eigvalsh((sr + sr.T) / 2)[-1]
    mt = np.linalg.eigvalsh((st + st.T) / 2)[-1]
    if mr <= 0 or mt <= 0:
        raise ValueError("adaptive length unavailable")
    ell = np.sqrt(mr / mt)
    inverse_s = np.diag([1 / ell] * 3 + [1.] * 3)
    w = weak(inverse_s.T @ h @ inverse_s, ratio, inclusive)
    common = np.diag([1.] * 3 + [1 / length] * 3)
    return orth(common @ inverse_s @ w)


def reference(coarse, fine):
    discrepancy = np.linalg.norm(coarse - fine) / max(np.linalg.norm(fine), 1e-30)
    if not np.isfinite(coarse).all() or not np.isfinite(fine).all():
        return None, "NONFINITE"
    if discrepancy > .1:
        return None, "STEP_UNSTABLE"
    bases = []
    for h in [coarse, fine]:
        ev, q = np.linalg.eigh((h + h.T) / 2)
        tol = 1e-6 * max(1., np.max(np.abs(ev)))
        if ev[0] < -tol:
            return None, "INDEFINITE_COST_CURVATURE"
        # Roundoff accepted only by explicit reference tolerance, never repair Uobs.
        ev = np.maximum(ev, 0)
        bases.append(q[:, ev < max(100 * np.finfo(float).eps * max(1., ev[-1]), .05 * ev[-1])])
    if bases[0].shape[1] != bases[1].shape[1]:
        return None, "DIMENSION_UNSTABLE"
    if compare(*bases)["projection_distance"] > .1:
        return None, "SUBSPACE_UNSTABLE"
    return bases[1], "VALID"


def stats(values):
    values = list(values)
    if not values:
        return None
    return dict(count=len(values), mean=float(np.mean(values)), median=float(np.median(values)),
                p95=float(np.percentile(values, 95)), max=float(max(values)))


def tests():
    a = np.eye(6)[:, :2]
    assert compare(a, a[:, ::-1] * [-1, 1])["projection_distance"] < 1e-14
    assert compare(a, np.eye(6)[:, 2:4])["projection_distance"] == 2
    assert compare(a[:, :0], a)["max_angle_deg"] is None
    h1 = np.diag([.049, .051, 1, 1, 1, 1])
    h2 = np.diag([.051, .049, 1, 1, 1, 1])
    assert reference(h1, h2)[1] == "SUBSPACE_UNSTABLE"
    assert reference(np.eye(6), -np.eye(6))[0] is None
    assert reference(h1, h1)[1] == "VALID"
    boundary = np.diag([.0125, 1., 1., 1., 1., 1.])
    assert weak(boundary, 1/80, inclusive=True).shape[1] == 1
    assert weak(boundary, 1/80, inclusive=False).shape[1] == 0
    # Independently calculated diagonal Schur scale ell=sqrt(100/4)=5.
    h = np.diag([100., 10., 1., 4., 2., .01])
    assert adaptive(h, .8, .05).shape[1] == 2
    scale = np.diag([1., 1., 1., 100., 100., 100.])
    h_cm = np.linalg.solve(scale, h) @ np.linalg.inv(scale)
    assert compare(adaptive(h, .8, .05), adaptive(h_cm, 80., .05))["projection_distance"] < 1e-12
    offline_rows = [dict(variant="real", case="1", source_hash="123")]
    def rejected(call):
        try:
            call()
        except ValueError:
            return
        raise AssertionError("corrupt/missing population was accepted")
    rejected(lambda: load_population(None, None, offline_rows, {"1": {}}))
    with tempfile.TemporaryDirectory(prefix="i2_population_test_") as tmp:
        reg, prod = Path(tmp)/"registration.csv", Path(tmp)/"uobs.csv"
        reg.write_text("transaction_id,stamp_ns,effective,status,source_hash_actual\n" +
                       "".join(f"{i},{i},{int(i==1)},TEST,123\n" for i in range(1,101)))
        full = "transaction_id,stamp_ns,ndt_effective\n" + "".join(f"{i},{i},{int(i==1)}\n" for i in range(1,101))
        prod.write_text(full)
        assert len(load_population(reg,prod,offline_rows,{"1":{}})[0]) == 100
        prod.write_text("\n".join(full.splitlines()[:-1])+"\n")
        rejected(lambda: load_population(reg,prod,offline_rows,{"1":{}}))
        prod.write_text(full)
        rejected(lambda: load_population(reg,prod,offline_rows,{}))


def load_population(registration, production_uobs, rows, curves):
    real = [r for r in rows if r["variant"] == "real"]
    if not real:
        return {}, {}
    if registration is None or production_uobs is None:
        raise ValueError("real evidence requires registration AND complete production Uobs")
    with open(registration) as stream:
        entries = list(csv.DictReader(stream))
    with open(production_uobs) as stream:
        uobs_entries = list(csv.DictReader(stream))
    expected = [str(i) for i in range(1, 101)]
    if [r["transaction_id"] for r in entries] != expected or [r["transaction_id"] for r in uobs_entries] != expected:
        raise ValueError("production evidence must cover tx1..100 exactly once in order")
    reg = {r["transaction_id"]: r for r in entries}
    production = {r["transaction_id"]: r for r in uobs_entries}
    effective = {tx for tx, r in reg.items() if r["effective"] == "1"}
    if set(r["case"] for r in real) != effective or len(real) != len(effective) or set(curves) != effective:
        raise ValueError("offline evidence does not exactly cover successful transactions")
    for tx in expected:
        if reg[tx]["stamp_ns"] != production[tx]["stamp_ns"] or reg[tx]["effective"] != production[tx]["ndt_effective"]:
            raise ValueError("production metadata mismatch")
    for row in real:
        if row["source_hash"] != reg[row["case"]]["source_hash_actual"]:
            raise ValueError("production hash mismatch")
    return reg, production


def analyze(prefix, output, production_uobs=None, registration=None):
    rows = list(csv.DictReader(open(str(prefix) + "_geometry.csv")))
    curves = {r["transaction_id"]: r for r in csv.DictReader(open(str(prefix) + "_curvature.csv"))}
    population, production = load_population(registration, production_uobs, rows, curves)
    results, cached, runtimes, unavailable = [], {}, [], Counter()
    for row in rows:
        name, variant = row["case"], row["variant"]
        h, t = mat(row, "H"), mat(row, "transport")
        length = float(row["length_scale"])
        bases = {}
        if row["uobs_valid"] == "1":
            bases["uobs"] = orth(mat(row, "W")[:, :int(row["uobs_dim"])])
        else:
            unavailable[f"{variant}:uobs"] += 1
        if row["dc_valid"] == "1":
            bases["dcreg_native"] = orth(mat(row, "DC")[:, :int(row["dc_dim"])])
        else:
            unavailable[f"{variant}:dcreg_native"] += 1
        for method, ratio in [("fmcw_native", 1 / 80), ("adaptive_paired", .05)]:
            start = time.perf_counter()
            try:
                bases[method] = adaptive(h, length, ratio, inclusive=method == "fmcw_native")
            except (ValueError, np.linalg.LinAlgError):
                unavailable[f"{variant}:{method}"] += 1
            elapsed = 1000 * (time.perf_counter() - start)
            if method == "fmcw_native" and variant == "real":
                runtimes.append(elapsed)
        item = dict(case=name, variant=variant, parameter=float(row["parameter"]),
                    dimensions={k: v.shape[1] for k, v in bases.items()})
        if variant == "real":
            curve = curves[name]
            coarse, fine = mat(curve, "coarse"), mat(curve, "fine")
            truth, status = reference(coarse, fine)
            item["reference_status"] = status
            item["reference_relative_step_error"] = float(np.linalg.norm(coarse-fine)/max(np.linalg.norm(fine), 1e-30))
            item["reference_coarse_eigenvalues"] = np.linalg.eigvalsh(coarse).tolist()
            item["reference_fine_eigenvalues"] = np.linalg.eigvalsh(fine).tolist()
            if truth is not None:
                item["comparisons"] = {k: compare(v, truth) for k, v in bases.items()}
            pr = production[name]
            # Active eigenbasis sign is irrelevant; H_phys must match the runner.
            item["production_observation_count_match"] = int(row["observations"]) == int(pr["valid_correspondences"])
            item["production_dimension_match"] = int(row["uobs_dim"]) == int(pr["weak_dimension"])
            production_h = np.array([[float(pr[f"Hphys_r{i}c{j}"]) for j in range(6)] for i in range(6)])
            h_match = np.allclose(h, production_h, rtol=1e-12, atol=1e-12, equal_nan=True)
            item["production_h_relative_error"] = float(np.linalg.norm(h-production_h)/max(1., np.linalg.norm(production_h))) if np.isfinite(h).all() else None
            if (row["uobs_valid"] != pr["classification_valid"] or not h_match
                    or not item["production_observation_count_match"] or not item["production_dimension_match"]):
                raise ValueError("offline geometry does not reproduce production Uobs")
            cached[(name, variant)] = bases
        else:
            d = np.diag([1.] * 3 + [length] * 3)
            item["uobs_relative_spectrum"] = (np.linalg.eigvalsh(d @ h @ d)/np.linalg.eigvalsh(d @ h @ d)[-1]).tolist() if np.isfinite(h).all() else None
            ref = orth(mat(row, "reference")[:, :int(row["reference_dim"])])
            if variant == "endwall":
                item["analytic_exact_dimension_not_scored"] = int(row["reference_dim"])
            else:
                item["comparisons"] = {k: compare(v, ref) for k, v in bases.items()}
            if variant in ["cm", "rotated", "reference_point"]:
                base = cached[(name, "base")]
                item["transport"] = {k: compare(t @ v, base[k]) for k, v in bases.items() if k in base}
            if variant == "base":
                cached[(name, variant)] = bases
        results.append(item)
    real = [r for r in results if r["variant"] == "real"]
    controlled = [r for r in results if r["variant"] == "base"]
    summary = dict(controlled={}, real={}, invariance={}, comparator_unavailable=dict(unavailable))
    for method in ["uobs", "dcreg_native", "fmcw_native", "adaptive_paired"]:
        cc = [r["comparisons"][method] for r in controlled if method in r["comparisons"]]
        rc = [r["comparisons"][method] for r in real if "comparisons" in r and method in r["comparisons"]]
        summary["controlled"][method] = dict(available=len(cc), correct_dimension=sum(c["dimension_match"] for c in cc),
            projection_distance=stats(c["projection_distance"] for c in cc),
            false_weak=sum(c["false_weak_dimension"] for c in cc), missed_weak=sum(c["missed_weak_dimension"] for c in cc))
        temporal, transported_temporal = [], []
        for prev, nxt in zip(real, real[1:]):
            if int(nxt["case"]) == int(prev["case"]) + 1:
                b1, b2 = cached[(prev["case"], "real")], cached[(nxt["case"], "real")]
                if method in b1 and method in b2:
                    temporal.append(compare(b1[method], b2[method])["projection_distance"])
                    # Compare at the preceding scan's physical increment origin.
                    p1, p2 = population[prev["case"]], population[nxt["case"]]
                    c = np.array([float(p2[f"raw_{a}"])-float(p1[f"raw_{a}"]) for a in "xyz"])
                    cross = np.array([[0.,-c[2],c[1]],[c[2],0.,-c[0]],[-c[1],c[0],0.]])
                    transport = np.eye(6)
                    transport[3:,:3] = cross/.8
                    transported_temporal.append(compare(b1[method], transport @ b2[method])["projection_distance"])
        summary["real"][method] = dict(detector_available_frames=sum(method in r["dimensions"] for r in real),
            reference_comparable_frames=len(rc), correct_dimension=sum(c["dimension_match"] for c in rc),
            projection_distance=stats(c["projection_distance"] for c in rc),
            max_angle_deg=stats(c["max_angle_deg"] for c in rc if c["max_angle_deg"] is not None),
            temporal_projector_delta=stats(temporal),
            temporal_reference_point_transported_delta=stats(transported_temporal),
            weak_dimension_histogram=dict(Counter(r["dimensions"][method] for r in real if method in r["dimensions"])))
        summary["invariance"][method] = {variant: stats(r["transport"][method]["projection_distance"] for r in results
            if r["variant"] == variant and method in r.get("transport", {})) for variant in ["cm", "rotated", "reference_point"]}
    summary["reference_statuses"] = dict(Counter(r["reference_status"] for r in real))
    summary["population"] = dict(total_frames=len(population), effective_frames=len(real),
        ndt_statuses=dict(Counter(r["status"] for r in population.values())),
        unavailable_frames=[dict(transaction_id=int(tx), ndt_status=r["status"], detector_status="NDT_INEFFECTIVE",
                                 reference_status="NOT_EVALUATED") for tx,r in population.items() if r["effective"] != "1"])
    summary["paired_scaling"] = {}
    for label, group in [("controlled", controlled), ("real", real)]:
        common = [r for r in group if all(k in r.get("comparisons", {}) for k in ("uobs", "adaptive_paired"))]
        summary["paired_scaling"][label] = dict(common_cases=[r["case"] for r in common],
            **{k: dict(projection_distance=stats(r["comparisons"][k]["projection_distance"] for r in common),
                       dimension_correct=sum(r["comparisons"][k]["dimension_match"] for r in common))
               for k in ("uobs", "adaptive_paired")})
    summary["runtime_ms"] = dict(uobs=stats(float(r["uobs_ms"]) for r in rows if r["variant"] == "real"),
        dcreg_characterization_only=stats(float(r["dc_ms"]) for r in rows if r["variant"] == "real"),
        fmcw_python_characterization_only=stats(runtimes))
    output.write_text(json.dumps(dict(summary=summary, rows=results), indent=2, allow_nan=False) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("prefix", type=Path, nargs="?")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--production-uobs", type=Path)
    parser.add_argument("--registration", type=Path)
    args = parser.parse_args()
    tests()
    if args.prefix:
        if args.output is None:
            parser.error("--output required")
        analyze(args.prefix, args.output, args.production_uobs, args.registration)
    else:
        print("offline analysis unit tests PASS")
