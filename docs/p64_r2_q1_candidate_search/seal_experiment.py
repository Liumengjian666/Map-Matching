"""Check input identity and freeze candidate-only outputs before offline evaluation."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


def filesha(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def rows(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def save(path, data):
    with path.open("x") as stream:
        json.dump(data, stream, indent=2)
        stream.write("\n")


def verify_gate(run, manifest):
    score = rows(run / "gate/native_score_consistency.csv")
    assert len(score) == 204 and all(row["pass"] == "1" for row in score)
    identity = {}
    for row in manifest:
        sid = row["sample_id"]
        identity[sid] = {}
        for role in ("source", "target"):
            blob = run / "gate" / f"{sid}_{role}_xyz.bin"
            observed = filesha(blob)
            assert observed == row[f"expected_{role}_xyz_sha256"], (sid, role)
            assert blob.stat().st_size == 12 * int(row[f"expected_{role}_points"])
            identity[sid][role] = observed
    return {
        "cases": len(score),
        "max_absolute": max(float(row["absolute"]) for row in score),
        "max_relative": max(float(row["relative"]) for row in score),
        "source_and_target_xyz": identity,
    }


def combine_grid(run, manifest):
    sources = [run / p for p in ("smoke/grid.csv", "rest/grid.csv")]
    grid = [row for p in sources for row in rows(p)]
    sids = {row["sample_id"] for row in manifest}
    assert {row["sample_id"] for row in grid} == sids
    result = []
    for sid in sorted(sids):
        subset = [row for row in grid if row["sample_id"] == sid]
        assert len(subset) == 4851
        assert [int(row["index"]) for row in subset] == list(range(4851))
        for row in subset:
            ix, iy, ia = (int(row[key]) for key in ("ix", "iy", "ia"))
            assert 0 <= ix <= 20 and 0 <= iy <= 20 and 0 <= ia <= 10
            assert int(row["index"]) == (ix * 21 + iy) * 11 + ia
            assert abs(float(row["dx"]) - (ix - 10) * 0.2) < 1e-9
            assert abs(float(row["dy"]) - (iy - 10) * 0.2) < 1e-9
            assert float(row["dyaw"]) == ia - 5
        mass = [float(row["park_mass"]) for row in subset]
        assert all(math.isfinite(x) and x >= 0 for x in mass) and abs(math.fsum(mass) - 1) < 1e-12
        result.extend(subset)
    (run / "official").mkdir(exist_ok=True)
    with (run / "official/grid.csv").open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(result[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(result)


def seal(run, manifest_path):
    paths = [
        "input/online.tsv", "official/grid.csv", "official/modes.csv", "official/fit.csv",
        "official/fit_seeds.tsv", "fit/curvature.csv", "gate/native_score_consistency.csv",
        "gate/identity.json", "gate/point_scores.csv", "gate/voxel_contributions.csv",
        "fit/point_scores.csv", "fit/voxel_contributions.csv",
    ]
    files = {name: filesha(run / name) for name in paths}
    save(run / "online_seal.json", {"files": files, "manifest_sha256": filesha(manifest_path)})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("gate", "grid", "seal"))
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    manifest = Path(__file__).with_name("qualification_sample_manifest.csv")
    if args.stage == "gate":
        save(args.run / "gate/identity.json", verify_gate(args.run, rows(manifest)))
    elif args.stage == "grid":
        combine_grid(args.run, rows(manifest))
    else:
        seal(args.run, manifest)


if __name__ == "__main__":
    main()
