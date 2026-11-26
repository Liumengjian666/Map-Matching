"""Validate and hash the online-only Q2 artifacts before offline evaluation."""
import argparse
import csv
from collections import Counter
from hashlib import sha256
import json
from pathlib import Path


def sha(path):
    digest = sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def rows(path):
    with path.open(newline="") as f:
        yield from csv.DictReader(f)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("q1", type=Path)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    baseline = args.q1 / "run"
    manifest = args.q1 / "docs/p64_r2_q1_candidate_search/qualification_sample_manifest.csv"
    check = json.loads((baseline / "online_seal.json").read_text())
    for name in ("input/online.tsv", "official/grid.csv", "official/fit.csv", "gate/native_score_consistency.csv"):
        assert sha(baseline / name) == check["files"][name], name
    perf = list(rows(args.run / "official/runtime_breakdown.csv"))
    checks = list(rows(args.run / "official_score_parity.csv"))
    assert len(checks) == 204 and all(x["pass"] == "1" for x in checks)
    sids = {row["sample_id"] for row in rows(manifest)}
    assert len(sids) == 12
    assert {(row["sample_id"], row["method"]) for row in perf} == {(sid, method) for sid in sids for method in ("S0","S1","S2","S3")}
    counts, fits, seen_poses = Counter(), Counter(), set()
    original = {}
    for row in rows(baseline / "official/grid.csv"):
        original[(row["sample_id"], row["index"])] = row
    fields = ("native", "park", "associated", "park_associated", "total", "dx", "dy", "dyaw") + tuple(f"T{i}{j}" for i in range(4) for j in range(4))
    for row in rows(args.run / "official/grid.csv"):
        group = row["sample_id"], row["method"]
        key = group + (row["index"],)
        assert key not in seen_poses, key
        seen_poses.add(key)
        counts[group] += 1
        if row["method"] == "S0":
            base = original.pop((row["sample_id"], row["index"]))
            assert all(float(row[k]) == float(base[k]) for k in fields), key
    assert not original
    for row in rows(args.run / "official/fit.csv"):
        key = row["sample_id"], row["method"]
        assert key + (row["index"],) in seen_poses
        fits[key] += 1
    for row in perf:
        method = row["method"]
        key = row["sample_id"], method
        n = int(row["unique_poses"])
        assert 0 < n == counts[key] <= 4851
        assert fits[key] == int(row["align_calls"]) <= (2 if method == "S0" else 4)
        for col in ("score_calls","native_calls","park_calls","transform_calls"):
            assert int(row[col]) == (0 if method == "S0" else n)
        if method == "S2":
            assert int(row["proposals"]) == 4846
        if method == "S3":
            assert int(row["proposals"]) <= 3590
    assert sha(args.run / "online.tsv") == check["files"]["input/online.tsv"]
    filenames = ("online.tsv", "official/grid.csv", "official/retained.csv", "official/fit.csv",
                 "official/runtime_breakdown.csv", "official_score_parity.csv")
    receipt = {
        "files": {f: sha(args.run / f) for f in filenames},
        "manifest_sha256": sha(manifest),
        "q1_online_sha256": sha(baseline / "input/online.tsv"),
        "original_s0_identical": True,
        "before_reference": True,
    }
    with (args.run / "online_seal.json").open("x") as f:
        json.dump(receipt, f, indent=2)
        f.write("\n")
    print("Q2 seal passed:", sum(counts.values()), "reused/new grid rows and", sum(fits.values()), "new native aligns")


if __name__ == "__main__":
    main()
