"""Generate auditable replay builds without modifying either upstream source tree."""
import argparse
from pathlib import Path


def replace(text, old, new):
    if text.count(old) != 1:
        raise ValueError(f"build input mismatch: {old!r}")
    return text.replace(old,new)


def main(docs, official, research):
    nodelet = official/"apps/hdl_localization_nodelet.cpp"
    native = nodelet.read_text()
    native = '#include "input_receipt.hpp"\n'+native
    native = replace(native,"    registration->setInputTarget(globalmap);",
        '    registration->setInputTarget(globalmap);\n    record_cloud("TARGET",0,*globalmap);')
    native = replace(native,"    auto filtered = downsample(cloud);",
        '    auto filtered = downsample(cloud);\n    record_cloud("SOURCE",stamp.toNSec(),*filtered);')
    (research/"official_nodelet.cpp").write_text(native)
    upstream = docs/"p64_r3_q6_fine_grained_geometry_assisted_ndt/pipeline.cpp"
    adapted = '#include "input_receipt.hpp"\n'+upstream.read_text()
    adapted = replace(adapted,"auto target=downsample(load(map));",
        'auto target=load(map);\n    record_cloud("TARGET",0,*target);')
    adapted = replace(adapted,"const auto scan=downsample(raw); ros::Time stamp;stamp.fromNSec(ns);",
        'const auto scan=downsample(raw); ros::Time stamp;stamp.fromNSec(ns);\n      record_cloud("SOURCE",ns,*scan);')
    adapted = replace(adapted,"frame<300 && ns>last_scan","frame<1200 && ns>last_scan")
    (research/"research_pipeline.cpp").write_text(adapted)
    replay = docs/"p64_r3_q9_public_benchmark_fine_geometry"
    script = (replay/"replay_hdl.sh").read_text().replace("export OMP_NUM_THREADS=4","export OMP_NUM_THREADS=1")
    script = script.replace('"$here/stream_hdl.py"',f'"{replay}/stream_hdl.py"')
    (research/"replay_hdl.sh").write_text(script)


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    for name in ("docs","official","research"):
        parser.add_argument(name,type=Path)
    args=parser.parse_args()
    main(*(p.resolve() for p in (args.docs,args.official,args.research)))
