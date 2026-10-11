#!/usr/bin/env python3
"""Check own PCD/voxel adapter against original author's preprocessing body."""
import argparse
import ast
import json
import pathlib
import subprocess
import numpy as np


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--author", type=pathlib.Path, required=True)
    p.add_argument("--binary", type=pathlib.Path, required=True)
    p.add_argument("--out", type=pathlib.Path, required=True)
    args = p.parse_args()
    scope = {"np": np}
    source = args.author / "inlier/eval/encode.py"
    functions = [n for n in ast.parse(source.read_text()).body if isinstance(n, ast.FunctionDef)
                 and n.name in ("prepare_points", "voxel_downsample")]
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), "exec"), scope)
    raw = np.array([[0, 0, 0], [.1, .1, .1], [.2, .2, .2], [-.1, -.1, -.1],
                    [-.2, -.2, -.2], [.5, 0, 0], [1, 2, 3], [np.nan, 1, 2],
                    [1, np.inf, 2]], dtype=np.float32)
    pcdf = args.out / "adapter_boundary.pcd"
    with pcdf.open("w") as f:
        f.write("VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        f.write("WIDTH 9\nHEIGHT 1\nPOINTS 9\nDATA ascii\n")
        np.savetxt(f, raw, fmt="%.9g")
    output = args.out / "adapter_boundary.bin"
    subprocess.run([str(args.binary), "--preprocess", str(pcdf), str(output)], check=True)
    expected = scope["prepare_points"](raw, 0.5).astype(np.float64)
    actual = np.fromfile(output, dtype=np.float64).reshape(-1, 3)
    assert np.array_equal(expected, actual), (expected, actual)
    # Real reference ASCII XYZ input; no encoder is called in this input test.
    path = pathlib.Path("/home/jian/rosbag/p11_solid_r1_20261011/reference_scans/scans/key_000000.pcd")
    with path.open() as f:
        while f.readline().strip() != "DATA ascii":
            pass
        real = np.loadtxt(f, dtype=np.float32)
    actual_path = args.out / "adapter_reference0.bin"
    subprocess.run([str(args.binary), "--preprocess", str(path), str(actual_path)], check=True)
    actual_real = np.fromfile(actual_path, dtype=np.float64).reshape(-1, 3)
    expected_real = scope["prepare_points"](real[:, :3], 0.5).astype(np.float64)
    assert np.array_equal(actual_real, expected_real)
    (args.out / "adapter_test.json").write_text(json.dumps({
        "status": "PASS", "boundary_points": len(raw), "boundary_retained": len(actual),
        "real_points": len(real), "real_retained": len(actual_real),
        "exact_point_and_order_parity": True, "author_function": "prepare_points/voxel_downsample",
    }, indent=2) + "\n")
    print("ADAPTER_TEST_PASS boundary and real reference point/order exact")


if __name__ == "__main__":
    main()
