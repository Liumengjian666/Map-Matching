"""Evaluate a separately sealed blind localizer run against Omni reference."""
import argparse
import csv
import hashlib
import json
import runpy
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

Q1 = Path(__file__).resolve().parents[1] / "p64_r2_q1_candidate_search/prepare_inputs.py"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def matrix(row, kind):
    return np.asarray([float(row[f"{kind}{r}{c}"]) for r in range(4) for c in range(4)]).reshape(4, 4)


def evaluate(directory, seal_path, output):
    seal = json.loads(seal_path.read_text())
    assert set(seal) >= {"output", "anchor", "imu", "scans", "input_receipt", "exe"}
    for item in seal.values():
        assert sha(Path(item["path"])) == item["sha256"], item["path"]
    metadata = json.loads((directory / "BLIND_INIT_RECEIPT.json").read_text())
    for scan in metadata["scans"]:
        assert sha(directory / scan["file"]) == scan["sha256"]
    assert metadata["qualifies_initialization"]
    with Path(seal["output"]["path"]).open(newline="") as stream:
        states = list(csv.DictReader(stream))
    match = [x["stamp_ns"] for x in metadata["scans"]]
    assert [int(x["stamp_ns"]) for x in states] == match and len(states) == 30
    assert all(not (int(x["latest_imu_ns"]) > int(x["stamp_ns"])) for x in states)
    original = runpy.run_path(str(Q1))
    track = original["read_omni_reference"](match[-1])
    rows = []
    columns = ["frame", "stamp_ns", "valid"]
    for name in ("pred", "ndt", "state"):
        columns.extend([f"{name}_translation_m", f"{name}_rotation_deg", f"{name}_correct"])
    for row in states:
        ns = int(row["stamp_ns"])
        ref = original["interpolate"](*track, ns, True)
        current = [row["frame"], ns, ref is not None]
        if ref is not None:
            for name in ("pred", "ndt", "state"):
                pose = matrix(row, name)
                translation = float(np.linalg.norm(ref[:3, 3] - pose[:3, 3]))
                rotation = float(np.rad2deg(Rotation.from_matrix(ref[:3, :3].T @ pose[:3, :3]).magnitude()))
                current.extend((translation, rotation, translation <= 0.15 and rotation <= 2))
        rows.append(current)
    with output.open("x", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(columns)
        writer.writerows(row + [""] * (len(columns) - len(row)) for row in rows)
    print("reference matched", sum(row[2] for row in rows), "/", len(rows))
    for j, name in enumerate(("pred", "ndt", "state")):
        values = [row[3+j*3:6+j*3] for row in rows if row[2]]
        if values:
            print(name, "first", values[0], "medians", np.median(values, axis=0).tolist(),
                  "correct count", sum(a[2] for a in values))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("frozen_input", type=Path)
    parser.add_argument("seal", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    evaluate(args.frozen_input, args.seal, args.output)


if __name__ == "__main__":
    main()
