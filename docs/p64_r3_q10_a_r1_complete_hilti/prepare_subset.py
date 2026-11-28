"""Materialize the frozen 10- and 100-frame replay windows from a full input."""
import argparse
import json
from pathlib import Path


def build(src, target_map, output, n):
    source=src.resolve()
    scans=(source/"scans.tsv").read_text().splitlines()
    if n<1 or len(scans)<n:
        raise ValueError("requested interval outside recorded scan set")
    output.mkdir()
    for line in scans[:n]:
        _,filename=line.split()
        (output/filename).symlink_to(source/filename)
    (output/"scans.tsv").write_text("\n".join(scans[:n])+"\n")
    last=int(scans[n-1].split()[0])
    with (source/"imu.tsv").open() as original,(output/"imu.tsv").open("w") as selected:
        for line in original:
            if int(line.split(maxsplit=1)[0])>last:
                break
            selected.write(line)
    (output/"map.pcd").symlink_to(target_map.resolve())
    (output/"anchor.tsv").symlink_to(source/"anchor.tsv")
    stats=json.loads((src/"input_summary.json").read_text())
    stats.update(frames=n,last_ns=last)
    (output/"input_summary.json").write_text(json.dumps(stats,indent=2)+"\n")


if __name__=="__main__":
    p=argparse.ArgumentParser()
    for n in ("input","map","output"):
        p.add_argument(n,type=Path)
    p.add_argument("count",type=int)
    a=p.parse_args()
    build(a.input,a.map,a.output,a.count)
