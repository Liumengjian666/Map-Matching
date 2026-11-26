"""Expand Q2 retention and terminal collision receipts from sealed, evaluated tables."""
import argparse
import csv
from collections import defaultdict
from itertools import combinations
import json
from pathlib import Path

import numpy as np


def read(path):
    with path.open(newline="") as stream:
        yield from csv.DictReader(stream)


def write(path, rows):
    with path.open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def truth(value):
    return value in ("1", "True", True)


def pose(row):
    return np.array([float(row[f"T{i}{j}"]) for i in range(4) for j in range(4)]).reshape(4, 4)


def near(a, b):
    delta = a[:3, :3].T @ b[:3, :3]
    angle = np.rad2deg(np.arccos(np.clip((np.trace(delta)-1)/2, -1.0, 1.0)))
    return np.linalg.norm(a[:3, 3]-b[:3, 3]) <= 0.30 and angle <= 3.0


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run",type=Path)
    parser.add_argument("q1",type=Path)
    parser.add_argument("out",type=Path)
    args=parser.parse_args()
    assert json.loads((args.run/"online_seal.json").read_text())["before_reference"]
    official=args.run/"official"
    evaluations=args.run/"evaluation"
    selected=defaultdict(list)
    for row in read(official/"retained.csv"):
        if row["index"]!="-1":
            selected[tuple(row[k] for k in ("sample_id","method","metric","policy"))].append(row)
    for row in read(args.q1/"run/official/modes.csv"):
        if row["index"]:
            selected[row["sample_id"],"S0",row["metric"],"C0"].append(row)
    times={(r["sample_id"],r["method"]):r for r in read(official/"runtime_breakdown.csv")}
    receipt=[]
    for row in read(evaluations/"candidate_recall.csv"):
        if float(row["translation_tol_m"])!=0.15 or float(row["rotation_tol_deg"])!=2:
            continue
        key=tuple(row[k] for k in ("sample_id","method","metric","policy"))
        peaks=selected[key]
        retained=[r for r in peaks if truth(r["retained"])]
        count=int(row["unique_grid_n"])
        receipt.append({
            "sample_id":key[0],"method":key[1],"metric":key[2],"policy":key[3],
            "grid_count":count,"grid_duplicates_removed":times[key[:2]]["duplicates"],
            "all_local_peaks":len(peaks),"kept_modes":len(retained),
            "retained_grid_fraction":len(retained)/count,
            "boundary_peaks":sum(truth(r["boundary"]) for r in peaks),
            "boundary_kept":sum(truth(r["boundary"]) for r in retained),
            "correct_peak_lost":truth(row["raw_peak_any_correct"]) and not truth(row["retained_any_correct"]),
            "any_correct_retained":truth(row["retained_any_correct"]),
            "correct_boundary_kept":truth(row["boundary_retained_any_correct"]),
            "any_correct_qualified_refit_retained":truth(row["refined_retained_any_correct"]),
            "any_correct_accepted_refit_retained":truth(row["accepted_refined_any_correct"]),
            "final_distinct_modes":row["final_distinct_modes_n"],"final_top1_correct":row["final_top1_correct"],
        })
    write(args.out/"candidate_retention_analysis.csv",receipt)
    fit_rows=defaultdict(list)
    for r in read(evaluations/"refinement_offline_errors.csv"):
        fit_rows[r["sample_id"],r["method"],r["origin"]].append(r)
    required={(r["sample_id"],r["method"],r["index"]) for group in fit_rows.values() for r in group}
    orig={}
    for r in read(official/"grid.csv"):
        key=r["sample_id"],r["method"],r["index"]
        if key in required:
            orig[key]=pose(r)
    merges=[]
    for (sample,method,origin),group in sorted(fit_rows.items()):
        pairs=combined=split_start=0
        for a,b in combinations(group,2):
            if not (truth(a["qualified_native_fit"]) and truth(b["qualified_native_fit"])):
                continue
            pairs+=1
            if not near(orig[sample,method,a["index"]],orig[sample,method,b["index"]]):
                split_start+=1
                combined+=near(pose(a),pose(b))
        merges.append({"sample_id":sample,"method":method,"origin":origin,
                       "qualified_pair_count":pairs,"initially_separated_pairs":split_start,
                       "terminal_collisions":combined,
                       "collision_fraction_given_separation":combined/split_start if split_start else ""})
    write(args.out/"terminal_mode_merge.csv",merges)


if __name__=="__main__":
    main()
