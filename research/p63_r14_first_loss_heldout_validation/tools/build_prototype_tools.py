#!/usr/bin/env python3
"""Rebuild the R14 registration interposer and post-seal scorer."""
import hashlib
import json
from pathlib import Path
import shlex
import subprocess

A = Path(__file__).resolve().parents[1]
BUILD = A / "build"


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def run(name, command):
    result = subprocess.run(command, text=True, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT)
    log = "$ " + shlex.join(command) + "\n" + result.stdout
    (BUILD / f"{name}.log").write_text(log + f"\nexit status: {result.returncode}\n")
    if result.returncode:
        raise SystemExit(f"{name} failed with {result.returncode}; see {BUILD}/{name}.log")


def replace_after(command, marker, value):
    index = command.index(marker)
    command[index + 1] = str(value)


BUILD.mkdir(exist_ok=True)
trace_manifest = json.loads((BUILD / "SPATIAL_TRACE_BUILD.json").read_text())
trace_source = A / "src/spatial_split_trace.cpp"
trace_object = BUILD / "spatial_split_trace.o"
trace_binary = BUILD / "libp63_r14_spatial_split_trace.so"
trace_compile = list(trace_manifest["compile_command"])
replace_after(trace_compile, "-c", trace_source)
replace_after(trace_compile, "-o", trace_object)
trace_link = list(trace_manifest["link_command"])
replace_after(trace_link, "-o", trace_binary)
trace_link[trace_link.index(str(BUILD / "spatial_split_trace.o"))] = str(trace_object)
run("SPATIAL_TRACE_REBUILD", trace_compile)
run("SPATIAL_TRACE_LINK", trace_link)
(BUILD / "SPATIAL_TRACE_BUILD.json").write_text(json.dumps({
    "compile_command": trace_compile,
    "link_command": trace_link,
    "source_sha256": sha(trace_source),
    "split_header_sha256": sha(A / "src/spatial_split.hpp"),
    "object_sha256": sha(trace_object),
    "binary_sha256": sha(trace_binary),
    "alignment_mode": "one original NDT align on registration subset only",
    "validation_cloud_to_ndt": "never passed to setInputSource or computeTransformation",
    "heldout_score_computed_online": False,
}, indent=2) + "\n")

preflight = json.loads((BUILD / "PARTITION_PREFLIGHT_BUILD.json").read_text())
score_source = A / "src/score_heldout.cpp"
score_object = BUILD / "score_heldout.o"
score_binary = BUILD / "score_heldout"
score_compile = list(preflight["compile_command"])
replace_after(score_compile, "-c", score_source)
replace_after(score_compile, "-o", score_object)
score_link = [
    "/usr/bin/c++", "-O3", "-DNDEBUG", "-o", str(score_binary),
    str(score_object), "-Wl,-rpath,/usr/lib/x86_64-linux-gnu",
    "-lpcl_io", "-lpcl_filters", "-lpcl_kdtree", "-lpcl_common",
    "-lcrypto", "-lflann_cpp", "-llz4",
]
run("SCORE_COMPILE", score_compile)
run("SCORE_LINK", score_link)
dynamic = subprocess.check_output(["ldd", str(score_binary)], text=True)
if "libndt_omp" in dynamic or "libhdl_localization" in dynamic:
    raise SystemExit("offline scorer unexpectedly links runtime NDT/HDL libraries")
(BUILD / "SCORE_BUILD.json").write_text(json.dumps({
    "compile_command": score_compile,
    "link_command": score_link,
    "source_sha256": sha(score_source),
    "split_header_sha256": sha(A / "src/spatial_split.hpp"),
    "object_sha256": sha(score_object),
    "binary": str(score_binary),
    "binary_sha256": sha(score_binary),
    "dynamic_dependencies": dynamic.splitlines(),
    "align_calls": 0,
    "ros_connection": False,
    "reference_or_GT_access": False,
}, indent=2) + "\n")
identity_path = BUILD / "RUN_IDENTITY.json"
identity = json.loads(identity_path.read_text())
identity["trace_binary_sha256"] = sha(trace_binary)
identity["trace_source_sha256"] = sha(trace_source)
identity["split_header_sha256"] = sha(A / "src/spatial_split.hpp")
identity["score_binary"] = str(score_binary)
identity["score_binary_sha256"] = sha(score_binary)
identity["raw_imu_sha256"] = sha(A / "data/RAW_IMU_VERIFIED.csv")
identity["input_freeze_sha256"] = sha(A / "data/INPUT_FREEZE_LOCAL.json")
identity["partition_support_sha256"] = sha(A / "data/preflight_output/PARTITION_SUPPORT.csv")
identity["source_files"] = {
    str(path): sha(path) for path in [
        trace_source, A / "src/spatial_split.hpp", score_source,
        A / "tools/run_split_once.py", A / "tools/run_shadow_score.py",
        A / "tools/build_prototype_tools.py", A / "tools/imu_geometry.py",
        A / "tools/data.py", A / "tools/contract.py",
        A / "data/INPUT_FREEZE_LOCAL.json", A / "data/RAW_IMU_VERIFIED.csv",
        A / "data/preflight_output/PARTITION_SUPPORT.csv",
        A / "data/preflight_output/SOURCES.csv",
        A.parents[1] / "research/p63_hdl_real_imu/package.xml",
        A.parents[1] / "research/p63_hdl_real_imu/launch/p63_r14_split_indoor_hard.launch",
        Path("/media/jian/HIKVISION/livox_ws/p63_r3_hdl_data/catkin_ws/src/hdl_localization/package.xml"),
        Path("/media/jian/HIKVISION/livox_ws/p63_r3_hdl_data/catkin_ws/src/hdl_localization/nodelet_plugins.xml"),
    ]
}
identity_path.write_text(json.dumps(identity, indent=2) + "\n")
print(json.dumps({
    "trace_binary": str(trace_binary),
    "trace_sha256": sha(trace_binary),
    "score_binary": str(score_binary),
    "score_sha256": sha(score_binary),
    "score_links_ndt_or_hdl": False,
}, indent=2))
