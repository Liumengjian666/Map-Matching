#!/usr/bin/env python3
"""Bounded fixed-u offline requests. Never loads GT or invokes weak-grid search."""
import argparse
from collections import Counter
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess

import numpy as np
from scipy.spatial.transform import Rotation


def read(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def write(path, rows):
    if not rows:
        raise RuntimeError(f"empty output {path}")
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def vec(text):
    return np.array([float(x) for x in text.split(";")])


def text(vector):
    return ";".join(format(float(x), ".17g") for x in vector)


def pose(row, field="endpoint_pose_matrix16"):
    return vec(row[field]).reshape(4, 4)


def distances(a, b):
    return (float(np.linalg.norm(a[:3, 3] - b[:3, 3])),
            float((Rotation.from_matrix(a[:3, :3]).inv() *
                   Rotation.from_matrix(b[:3, :3])).magnitude() * 180 / np.pi))


def near(a, b):
    dt, dr = distances(a, b)
    return dt <= .2 and dr <= 2


class EndpointGroups:
    """Greedy complete-link with a canonical anchor and immutable main groups."""
    def __init__(self, canonical):
        self.groups = {"CANONICAL": [canonical]}
        self.main_names = None

    def add_main(self, point):
        for name, members in self.groups.items():
            if all(near(point, member) for member in members):
                members.append(point)
                return name
        name = f"OTHER_{len(self.groups):02d}"
        self.groups[name] = [point]
        return name

    def freeze_main(self):
        self.main_names = set(self.groups)

    def supplemental(self, point):
        if self.main_names is None:
            raise RuntimeError("main groups must be frozen before supplements")
        for name, members in self.groups.items():
            if all(near(point, member) for member in members):
                if name not in self.main_names:
                    members.append(point)
                return name
        count = sum(name.startswith("NEW_") for name in self.groups)
        name = f"NEW_{count + 1:02d}"
        self.groups[name] = [point]
        return name


REQUEST_FIELDS = ["request_id", "transaction_id", "cluster_id", "stage", "solver",
                  "objective", "beta", "local_d", "axis", "sign", "initial_v"]


def request(basin, stage, suffix, initial, solver="NEWTON", objective="DYNAMIC",
            beta="", local_d="", axis="", sign=""):
    values = [f"{basin['transaction_id']}_{basin['cluster_id']}_{stage}_{suffix}",
              basin["transaction_id"], basin["cluster_id"], stage, solver, objective,
              beta, local_d, axis, sign, text(initial)]
    return dict(zip(REQUEST_FIELDS, values))


def canonical_inputs(path):
    expected = {"616/P02", "616/P03", "616/P10", "616/P12", "616/P13", "616/P17", "2226/P09"}
    rows = [r for r in read(path) if r["group_a"] == "1"]
    if {f"{r['transaction_id']}/{r['cluster_id']}" for r in rows} != expected or len(rows) != 7:
        raise RuntimeError("wrong frozen GROUP A")
    if hashlib.sha256(Path(path).read_bytes()).hexdigest() != "e3b955a700a6205eb48e473186daec22f44b711bb357a2d682f31cbaeb745458":
        raise RuntimeError("R1A canonical sidecar digest changed")
    return rows


def invoke(args, mode, input_path, output):
    archive = Path(args.archive)
    command = [args.runner, mode, args.map, str(archive / "frozen/cohort_frozen.csv"),
               str(archive / "dual_u.csv"), args.canonical]
    if input_path is not None:
        command.append(str(input_path))
    command.append(str(output))
    env = dict(os.environ, LD_LIBRARY_PATH="/lib/x86_64-linux-gnu")
    subprocess.run(command, check=True, env=env)


def bind_inputs(args, out):
    archive = Path(args.archive)
    paths = [Path(args.map), Path(args.canonical), archive / "dual_u.csv",
             archive / "frozen/cohort_frozen.csv"]
    used = {616, 2226}
    paths += [Path(r["raw_cloud_file"]) for r in read(paths[-1])
              if int(r["transaction_id"]) in used]
    digests = {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    provenance = out / "input_provenance.json"
    if args.stage == "initial":
        provenance.write_text(json.dumps(digests, indent=2)+"\n")
    elif not provenance.is_file() or json.loads(provenance.read_text()) != digests:
        raise RuntimeError("input provenance differs from the initial run")


def checked_initial(out):
    requests = read(out / "initial_requests.csv");samples = read(out / "initial_samples.csv")
    expected = {r["request_id"]: r for r in requests}
    if len(expected) != 350 or len(samples) != len(expected) or \
            len({r["request_id"] for r in samples}) != len(samples):
        raise RuntimeError("incomplete/duplicate initial request coverage")
    if Counter(r["stage"] for r in requests) != {"MAIN":77,"LOCAL":168,"CONTROL":105}:
        raise RuntimeError("wrong main/local/matched-control coverage")
    for row in samples:
        req = expected[row["request_id"]]
        for name in REQUEST_FIELDS[:-1]:
            if str(row[name]) != str(req[name]):
                raise RuntimeError("initial output does not match request metadata")
        if np.linalg.norm(vec(row["initial_v"]) - vec(req["initial_v"])) > 1e-12:
            raise RuntimeError("initial output seed differs from request")
    return samples


def main_groups(basins, samples):
    groups, rows = {}, []
    for b in basins:
        key = (b["transaction_id"], b["cluster_id"])
        g = EndpointGroups(pose(b, "closed_pose_matrix16"));groups[key] = g
        selected = sorted([r.copy() for r in samples if (r["transaction_id"], r["cluster_id"]) == key and r["stage"] == "MAIN"],
                          key=lambda r: float(r["beta"]))
        if len(selected) != 11 or [float(r["beta"]) for r in selected] != [i/10 for i in range(11)]:
            raise RuntimeError("main beta coverage differs from frozen path")
        for r in selected:
            r["strong_branch_id"] = g.add_main(pose(r))
            r["canonical_radius_capture"] = int(near(pose(r), pose(b, "closed_pose_matrix16")))
            rows.append(r)
        g.freeze_main()
    return groups, rows


def representatives(main):
    result = []
    for key in sorted({(r["transaction_id"], r["cluster_id"], r["strong_branch_id"]) for r in main},
                      key=lambda k: (int(k[0]), k[1], k[2])):
        candidates = [r for r in main if (r["transaction_id"], r["cluster_id"], r["strong_branch_id"]) == key]
        result.append(min(candidates, key=lambda r: (float(r["endpoint_dynamic_energy"]),
            0.0 if key[2].startswith("NEW_") else float(r["beta"]), r["request_id"])))
    return result


def checked_representatives(basins, out):
    _, main = main_groups(basins, checked_initial(out))
    expected = all_representatives(main,out);actual = read(out / "full_refine_requests.csv")
    if len(actual) != len(expected):
        raise RuntimeError("missing/extra main group refinement request")
    keys = ["request_id", "transaction_id", "cluster_id", "strong_branch_id",
            "endpoint_v", "endpoint_pose_matrix16", "endpoint_dynamic_energy"]
    for a, b in zip(actual, expected):
        if any(str(a[k]) != str(b[k]) for k in keys):
            raise RuntimeError("refinement request not the selected actual PART A representative")


def all_representatives(main, out):
    selected = representatives(main)
    supplemental = [r for r in read(out / "beta_attractor_map.csv") + read(out / "local_capture.csv")
                    if r["strong_branch_id"].startswith("NEW_")]
    return selected + representatives(supplemental)


def initial(args, basins, out):
    rows = []
    for b in basins:
        v = vec(b["v_b"])
        rows += [request(b, "MAIN", f"b{i:02d}", i/10*v, beta=i/10) for i in range(11)]
        for d in [.02, .05, .10]:
            for axis in range(len(v)):
                for sign in [1, -1]:
                    trial = v.copy(); trial[axis] += sign*d
                    rows.append(request(b, "LOCAL", f"d{d}_a{axis}_s{sign}", trial,
                                        local_d=d, axis=axis, sign=sign))
        for objective in ["DYNAMIC", "FROZEN_T0", "FROZEN_CANONICAL"]:
            for i, beta in enumerate([0, .25, .5, .75, 1]):
                rows.append(request(b, "CONTROL", f"{objective}_b{i}", beta*v,
                                    "PATTERN", objective, beta=beta))
    write(out / "initial_requests.csv", rows)
    invoke(args, "--sample", out / "initial_requests.csv", out / "initial_samples")
    invoke(args, "--path", None, out / "support_path_scan.csv")


def classify_and_bisect(args, basins, out):
    samples = checked_initial(out)
    groups, main = main_groups(basins, samples)
    queue = []
    for b in basins:
        key = (b["transaction_id"], b["cluster_id"])
        rows = sorted([r for r in main if (r["transaction_id"], r["cluster_id"]) == key],
                      key=lambda r: float(r["beta"]))
        for a, c in zip(rows, rows[1:]):
            if a["strong_branch_id"] != c["strong_branch_id"]:
                queue.append((b, a, c, 0))
    all_rows, boundaries = list(main), []
    round_number = 0
    while queue:
        next_queue, pending, work = [], [], []
        for b, a, c, depth in queue:
            lo, hi = float(a["beta"]), float(c["beta"])
            if hi-lo <= .01+1e-12 or depth >= 8:
                boundaries.append({"transaction_id": b["transaction_id"], "cluster_id": b["cluster_id"],
                                   "beta_low": lo, "beta_high": hi, "width": hi-lo,
                                   "low_branch": a["strong_branch_id"], "high_branch": c["strong_branch_id"],
                                   "depth": depth, "boundary_kind": "BUDGETED_ENDPOINT_GROUP_SECTION"})
                continue
            mid = (lo+hi)/2
            req = request(b, "BISECTION", f"b{mid:.9f}", mid*vec(b["v_b"]), beta=mid)
            pending.append(req); work.append((b, a, c, depth, req["request_id"]))
        if not pending:
            break
        path = out / f"bisection_{round_number}_requests.csv"
        prefix = out / f"bisection_{round_number}"
        write(path, pending); invoke(args, "--sample", path, prefix)
        results = {r["request_id"]: r for r in read(str(prefix)+".csv")}
        for b, a, c, depth, rid in work:
            r = results[rid]; key = (b["transaction_id"], b["cluster_id"])
            r["strong_branch_id"] = groups[key].supplemental(pose(r))
            r["canonical_radius_capture"] = int(near(pose(r), pose(b, "closed_pose_matrix16")))
            all_rows.append(r)
            for lo, hi in [(a, r), (r, c)]:
                if lo["strong_branch_id"] != hi["strong_branch_id"]:
                    next_queue.append((b, lo, hi, depth+1))
        queue = next_queue; round_number += 1
    write(out / "beta_attractor_map.csv", all_rows)
    # An explicitly headed empty table is valid when all coarse endpoints agree.
    fields = ["transaction_id", "cluster_id", "beta_low", "beta_high", "width", "low_branch", "high_branch", "depth", "boundary_kind"]
    with (out / "beta_capture_boundaries.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n");writer.writeheader();writer.writerows(boundaries)
    local, controls = [], []
    for b in basins:
        key = (b["transaction_id"], b["cluster_id"]); closed = pose(b, "closed_pose_matrix16")
        for r in [r for r in samples if (r["transaction_id"], r["cluster_id"]) == key and r["stage"] == "LOCAL"]:
            r["strong_branch_id"] = groups[key].supplemental(pose(r))
            r["canonical_radius_capture"] = int(near(pose(r), closed));local.append(r)
        for objective in ["DYNAMIC", "FROZEN_T0", "FROZEN_CANONICAL"]:
            g = EndpointGroups(closed)
            rows = sorted([r for r in samples if (r["transaction_id"], r["cluster_id"]) == key and
                           r["stage"] == "CONTROL" and r["objective"] == objective], key=lambda r: float(r["beta"]))
            for r in rows:
                r["strong_branch_id"] = g.add_main(pose(r))
                r["canonical_radius_capture"] = int(near(pose(r), closed));controls.append(r)
    write(out / "local_capture.csv", local)
    write(out / "frozen_support_control.csv", controls)
    write(out / "full_refine_requests.csv", all_representatives(main,out))


def self_test():
    origin = np.eye(4); group = EndpointGroups(origin)
    assert group.add_main(origin) == "CANONICAL"
    shifted = origin.copy(); shifted[0, 3] = .3
    assert group.add_main(shifted) == "OTHER_01"
    group.freeze_main()
    a = origin.copy();a[0, 3] = 1
    b = origin.copy();b[0, 3] = 2
    assert group.supplemental(a) == "NEW_01"
    assert group.supplemental(b) == "NEW_02"
    assert group.supplemental(a) == "NEW_01"
    assert len(group.groups["CANONICAL"]) == 2
    # Near one anchor is not enough for complete-link admission.
    group = EndpointGroups(origin)
    a = origin.copy();a[0, 3] = .15
    b = origin.copy();b[0, 3] = -.15
    assert group.add_main(a) == "CANONICAL"
    assert group.add_main(b) != "CANONICAL"
    supplemental = [{"transaction_id":"616", "cluster_id":"P02", "strong_branch_id":"NEW_01",
                     "endpoint_dynamic_energy":"-1", "beta":"", "request_id":name} for name in ["z", "a"]]
    assert representatives(supplemental)[0]["request_id"] == "a"
    print("R1B_ORCHESTRATION_SELF_TEST=PASS")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--stage", choices=["initial", "bisect", "prepare-refine", "refine", "refine-supplemental"], default="initial")
    for name in ["runner", "archive", "map", "canonical", "output"]:
        parser.add_argument("--"+name)
    args = parser.parse_args()
    if args.self_test:
        self_test();return
    if any(getattr(args, name) is None for name in ["runner", "archive", "map", "canonical", "output"]):
        parser.error("all experiment paths are required")
    basins = canonical_inputs(args.canonical);out = Path(args.output);out.mkdir(parents=True, exist_ok=True)
    if hashlib.sha256(Path(args.map).read_bytes()).hexdigest() != "2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570":
        raise RuntimeError("frozen map digest changed")
    bind_inputs(args, out)
    if args.stage == "initial":
        initial(args, basins, out)
    elif args.stage == "bisect":
        classify_and_bisect(args, basins, out)
    elif args.stage == "prepare-refine":
        _, main = main_groups(basins,checked_initial(out))
        write(out / "full_refine_requests.csv",all_representatives(main,out))
    elif args.stage == "refine-supplemental":
        checked_representatives(basins,out)
        selected = [r for r in read(out / "full_refine_requests.csv") if r["strong_branch_id"].startswith("NEW_")]
        main_refines = read(out / "full_refine_main_branch_map.csv")
        main_ids = {r["representative_request_id"] for r in main_refines}
        expected_main_ids = {r["request_id"] for r in read(out / "full_refine_requests.csv") if not r["strong_branch_id"].startswith("NEW_")}
        if main_ids != expected_main_ids or len(main_refines) != len(main_ids):
            raise RuntimeError("reused main-refine coverage mismatch")
        write(out / "full_refine_supplemental_requests.csv",selected)
        invoke(args,"--refine",out / "full_refine_supplemental_requests.csv",out / "full_refine_supplemental_branch_map.csv")
        write(out / "full_refine_branch_map.csv",main_refines+read(out / "full_refine_supplemental_branch_map.csv"))
    else:
        checked_representatives(basins, out)
        invoke(args, "--refine", out / "full_refine_requests.csv", out / "full_refine_branch_map.csv")


if __name__ == "__main__":
    main()
