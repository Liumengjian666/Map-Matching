"""Recheck immutable experiment data and hash the final small-file archive."""
import json
import pathlib

from evaluate_experiment import verify_pool
from run_experiment import ARCHIVE, HERE, ROOT, csv_read, json_write, sha, verified_inputs


def audit(write=False):
    verified_inputs()
    freeze = json.loads((ARCHIVE / "attempt_0/execution_freeze.json").read_text())
    if sha(freeze["command"][0]) != freeze["binary_sha256"]:
        raise RuntimeError("experiment binary no longer matches receipt")
    blind = json.loads((ARCHIVE / "attempt_0/blind_outputs_freeze.json").read_text())
    for name, expected in blind["output_sha256"].items():
        if sha(ARCHIVE / "attempt_0" / name) != expected:
            raise RuntimeError("blind evidence SHA changed")
    verify_pool(csv_read(ARCHIVE / "attempt_0/probes.csv"))
    results = json.loads((ARCHIVE / "results.json").read_text())
    if [m["recovered"] for m in results["methods"]] != [6, 7, 7, 7]:
        raise RuntimeError("reported recovery changed")
    if results["real_full_NDT_calls"] != 426 or results["scientific_attempts"] != 1:
        raise RuntimeError("unreported experiment budget")
    for path in ARCHIVE.rglob("*.json"):
        json.loads(path.read_text(), parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
    files = [p for p in ARCHIVE.rglob("*") if p.is_file() and p.name != "artifact_hashes.json"]
    files += [p for p in HERE.iterdir() if p.suffix in {".py", ".cpp", ".hpp"} or p.name == "CMakeLists.txt"]
    current = {str(p.relative_to(ROOT)): sha(p) for p in sorted(files)}
    path = ARCHIVE / "artifact_hashes.json"
    if write:
        json_write(path, {"algorithm_implementation_commit": freeze["implementation_commit"],
            "artifact_sha256": current, "input_sha256": freeze["input_sha256"],
            "binary_sha256": freeze["binary_sha256"], "blind_chain": "PASS", "GT_LOADED": False})
    else:
        recorded = json.loads(path.read_text())["artifact_sha256"]
        if current != recorded:
            raise RuntimeError("final artifact manifest differs")
    print(json.dumps({"audit": "PASS", "artifact_files": len(current), "real_NDT_calls": 426,
                      "scientific_attempts": 1, "GT_LOADED": False}))


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--write", action="store_true")
    audit(parser.parse_args().write)
