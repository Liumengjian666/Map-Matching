#!/usr/bin/env python3
"""Test-only source inventory and reconstruction of the committed tx90 fixture."""
import argparse
import hashlib
from pathlib import Path
import re
import struct
import subprocess
import tempfile

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
CAPSULE = PACKAGE / "docs/p6_alg_integration_a3c_r1/TX90_FAILURE_CAPSULE.npz"
CAPSULE_SHA = "2f597628160e75f41bc4c13d9de015727c17b1755535ad806263f272ebcf0280"


def inventory():
    """Inventory symmetry averages, not general C++ alias analysis."""
    entries = []
    # Semicolon-bounded expressions cover multiline assignments in this package.
    assignment = re.compile(r"(?P<dest>\*?[\w]+(?:->\w+|\.\w+)*)\s*=(?!=)\s*"
                            r"(?P<rhs>[^;]*transpose\(\)[^;]*);", re.M)
    for directory in (PACKAGE / "src", PACKAGE / "include"):
        for path in sorted(directory.rglob("*")):
            if path.suffix not in (".cpp", ".hpp", ".h"):
                continue
            source = re.sub(r"//[^\n]*|/\*[\s\S]*?\*/",
                            lambda m: "\n" * m.group().count("\n"), path.read_text())
            for match in assignment.finditer(source):
                dest, rhs = match.group("dest", "rhs")
                if "0.5" not in rhs:
                    continue
                transposed_dest = dest[1:] + "->transpose()" if dest.startswith("*") else dest + ".transpose()"
                # Pointer expressions already contain -> in dest when not dereferenced.
                self_transpose = re.search(
                    r"(?<![\w.>])" + re.escape(transposed_dest), rhs) is not None
                entries.append((str(path.relative_to(PACKAGE)),
                                source.count("\n", 0, match.start()) + 1,
                                dest, self_transpose, ".eval()" in rhs,
                                " ".join(rhs.split())))
    unsafe = [e for e in entries if e[3] and not e[4]]
    if unsafe:
        raise AssertionError("remaining self-transpose symmetry assignments: " + repr(unsafe))
    # Bind the capsule evaluator to both actual production destinations.
    for filename, required in (
        ("fixed_lag_window.cpp", "evaluateSymmetricInformation(*hessian)"),
        ("window_marginalization.cpp", "evaluateSymmetricInformation(new_information)"),
    ):
        assert required in (PACKAGE / "src" / filename).read_text(), filename
    for e in entries:
        print(" | ".join(map(str, e)))
    print(f"SOURCE_ALIAS_INVENTORY_PASS average_assignments={len(entries)} "
          f"remaining_self_assignments={sum(e[3] for e in entries)} unsafe=0 "
          "critical_independent_temporaries=2")


def capsule_regression(inspector):
    assert hashlib.sha256(CAPSULE.read_bytes()).hexdigest() == CAPSULE_SHA
    # Regenerate the old diagnostic binary in a temporary test directory only;
    # the sole durable fixture remains the 32 KiB committed NPZ.
    with np.load(CAPSULE, allow_pickle=False) as archive:
        arrays = [(name, archive[name]) for name in archive.files]
    assert len(arrays) == 11
    with tempfile.TemporaryDirectory(prefix="p6_a3c_r2_capsule_") as directory:
        path = Path(directory) / "capsule.bin"
        with path.open("wb") as output:
            output.write(b"P6A3CR1CAPSULE\0\0")
            output.write(struct.pack("<II", 1, len(arrays)))
            for name, value in arrays:
                encoded = name.encode("ascii")
                assert value.ndim == 2 and value.dtype == np.float64
                output.write(struct.pack("<I", len(encoded)))
                output.write(encoded)
                output.write(struct.pack("<QQ", *value.shape))
                output.write(np.asarray(value, dtype="<f8").tobytes(order="C"))
        subprocess.run([inspector, str(path)], check=True)
    print("TX90_COMMITTED_CAPSULE_REPAIR_PASS sha=" + CAPSULE_SHA)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--inventory", action="store_true")
    parser.add_argument("--capsule-inspector")
    args = parser.parse_args()
    if args.inventory:
        inventory()
    if args.capsule_inspector:
        capsule_regression(args.capsule_inspector)
    if not args.inventory and not args.capsule_inspector:
        parser.error("select inventory or capsule inspector")


if __name__ == "__main__":
    main()
