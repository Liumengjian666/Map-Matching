#!/usr/bin/env python3
"""Recompute descriptive P13 evidence from committed archives; run no algorithm."""
import csv
import hashlib
import json
import math
import statistics
from pathlib import Path


HERE = Path(__file__).resolve().parent
DOCS = HERE.parent
P11 = DOCS / "p11_mature_solid_prior_map_integration_r1"
P12 = DOCS / "p12_dcreg_solid_minimal_integration_r1"
P12B = DOCS / "p12b_reference_pose_candidate_retry_r1"
P10 = DOCS / "p10_corridor01_deterministic_oosm_replay_r1"
INPUTS = {}


def read(path, table=False):
    INPUTS[str(path.relative_to(DOCS.parent))] = hashlib.sha256(path.read_bytes()).hexdigest()
    with path.open() as stream:
        return list(csv.DictReader(stream)) if table else json.load(stream)


def percentile(values, q):
    ordered = sorted(values)
    if not ordered or not 0 <= q <= 1:
        raise ValueError("empty sample or invalid quantile")
    index = (len(ordered) - 1) * q
    lo, hi = math.floor(index), math.ceil(index)
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (index - lo)


def stats(values):
    values = list(values)
    if not values or not all(math.isfinite(x) for x in values):
        raise ValueError("empty/nonfinite timing or statistic")
    return dict(count=len(values), mean=statistics.mean(values),
                p95=percentile(values, .95), max=max(values))


def dump_json(name, value):
    (HERE / name).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def dump_csv(name, rows):
    with (HERE / name).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def main():
    # Direct regression checks for interpolation and invalid data handling.
    assert math.isclose(percentile([1, 2, 3, 4], .95), 3.85, abs_tol=1e-12)
    assert stats([2]) == dict(count=1, mean=2, p95=2, max=2)
    for values in ([], [float("nan")], [float("inf")]):
        try:
            stats(values)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid data silently accepted")

    all_reg = read(P12 / "registration.csv", True)
    candidate = [r for r in all_reg if int(r["rank"]) > 0]
    retry = read(P12B / "retry.csv", True)
    query = read(P11 / "query_results.csv", True)
    valid_query = [r for r in query if r["descriptor_status"] == "OK"]
    ndt = read(P12 / "ndt_baseline.csv", True)
    initial_positions = {int(r["frame_index"]):
                         [float(r[f"initial_guess_t{a}"]) for a in "xyz"] for r in ndt}
    assert (len(all_reg), len(candidate), len(retry), len(valid_query)) == (96, 72, 18, 5816)
    cost_rows, partition = [], {}
    for name, rows, flag in (("P12", candidate, "converged"), ("P12B", retry, "B_converged")):
        wall = [float(r["call_wall_ms"]) for r in rows]
        iteration = [float(r["author_iteration_ms"]) for r in rows]
        setup = [w - i for w, i in zip(wall, iteration)]
        assert min(setup) >= 0
        assert all(r[flag] in ("0", "1") for r in rows)
        failed = [r for r in rows if r[flag] == "0"]
        failed_wall = sum(float(r["call_wall_ms"]) for r in failed)
        partition[name] = dict(calls=len(rows), author_converged=len(rows) - len(failed),
                               call_wall_ms=stats(wall), author_iteration_ms=stats(iteration),
                               setup_proxy_ms=stats(setup), setup_share=sum(setup)/sum(wall),
                               nonconverged_calls=len(failed), nonconverged_wall_ms=failed_wall,
                               nonconverged_wall_share=failed_wall/sum(wall))
        for stage, values in (("API_wall", wall), ("author_iteration", iteration),
                              ("setup_proxy_NOT_isolated_KDtree", setup)):
            cost_rows.append(dict(experiment=name, stage=stage, unit="ms", **stats(values)))
    for name, file in (("P12", P12 / "timing.csv"), ("P12B", P12B / "timing.csv")):
        for field in ("descriptor_ms", "search_ms", "combination_ms"):
            rows = read(file, True)
            cost_rows.append(dict(experiment=name, stage=field, unit="ms",
                                  **stats(float(r[field]) for r in rows)))
    for field in ("descriptor_ms", "search_ms"):
        cost_rows.append(dict(experiment="P11", stage=field, unit="ms",
                              **stats(float(r[field]) for r in valid_query)))

    far = [r for r in candidate if math.dist(
        [float(r[f"reference_t{a}"]) for a in "xyz"],
        initial_positions[int(r["frame_index"])] ) > 5]
    assert len(far) == 30 and all(r["converged"] == "0" for r in far)
    cases = [dict(frame_index=r["frame_index"], reference_id=r["reference_id"],
                  similarity=r["similarity"], author_converged=r["converged"],
                  iterations=r["iterations"], native_preupdate_rmse_m=r["rmse_before_last_update_m"],
                  correspondence_fraction=r["fitness_correspondence_fraction"],
                  final_xyz=" ".join(r[f"final_t{a}"] for a in "xyz"),
                  independent_location_label="NOT_AVAILABLE")
             for r in candidate if (int(r["frame_index"]), int(r["reference_id"]))
             in {(16, 0), (16, 1), (1413, 53), (4105, 265)}]
    assert len(cases) == 4
    db_path = P11 / "solid_database.txt"
    INPUTS[str(db_path.relative_to(DOCS.parent))] = hashlib.sha256(db_path.read_bytes()).hexdigest()
    with db_path.open() as stream:
        header = next(stream).split()
        range_bins = int(header[6])
        assert range_bins == 40
        db = [list(map(float, line.split())) for line in stream]
    similar_far_pairs = []
    for i, left in enumerate(db):
        for right in db[i + 1:]:
            # Same RANGE block used by upstream loop_detection; not the angle block.
            a, b = left[9:9+range_bins], right[9:9+range_bins]
            cosine = sum(x*y for x, y in zip(a, b)) / math.sqrt(sum(x*x for x in a)*sum(y*y for y in b))
            distance = math.dist(left[2:5], right[2:5])
            if cosine > .99 and distance > 5:
                similar_far_pairs.append(dict(left_id=int(left[0]), right_id=int(right[0]),
                                              cosine=cosine, reference_state_distance_m=distance))
    assert len(similar_far_pairs) == 2
    p10 = read(P10 / "posthoc_metrics.json")
    corridor = {name: dict(primary_3_35s=entry["metrics"]["primary_3_35s"],
                           first_0_10s=entry["metrics"]["first_0_10s"],
                           persistent_translation_thresholds=entry["persistent_translation_thresholds"],
                           runtime_integrity=p10["runtime_integrity"][name])
                for name, entry in p10["corrected_imu_trajectory_metrics"].items()
                if name != "shadow_v2"}
    dump_json("metrics.json", dict(cost_partition=partition, p12_far_initial_hypotheses=len(far),
              p11_valid_queries=len(valid_query),
              p11_top5_spread_gt5m=sum(float(r["topk_reference_pose_spread_m"]) > 5 for r in valid_query),
              reference_similar_far_pairs=similar_far_pairs,
              corridor_evaluation_contract=p10["evaluation_contract"], corridor=corridor,
              new_registration_calls=0, new_ros_replays=0, independent_loop3_place_labels=False))
    dump_csv("cost_breakdown.csv", cost_rows)
    dump_csv("evidence_cases.csv", cases)
    dump_json("input_hashes.json", INPUTS)
    print("PASS: archived evidence recomputed; no algorithm, GT refit, registration or ROS replay")


if __name__ == "__main__":
    main()
