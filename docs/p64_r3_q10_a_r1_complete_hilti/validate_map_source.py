"""Audit E57 coordinate interpretation using a separate pye57 binding."""
import argparse
import json
from pathlib import Path

import numpy as np
import pye57
from pye57 import libe57


def audit(e57_path, csv_path, output):
    results=[]
    with pye57.E57(str(e57_path)) as data:
        for scan in range(data.scan_count):
            header=data.get_header(scan)
            record={
                "scan":scan,"count":header.point_count,
                "rot":header.rotation_matrix.tolist(),"trans":header.translation.tolist(),
                "samples_validated":0,"max_raw_coordinate_error_m":0.,
                "max_world_coordinate_error_m":0.,
            }
            refs={}
            for line in csv_path.read_text().splitlines():
                tag,s,n,*v=line.split()
                if tag=="SAMPLE" and int(s)==scan:
                    refs[int(n)-1]=np.array(v[1:],dtype=float)
            # Read first raw block via E57Foundation's compressed-vector buffers.
            # After the decoder checksum test, the first million values yield a sampled
            # raw record independently of the SimpleReader and Eigen transforms.
            n=min(header.point_count,1_000_000)
            if not n:
                continue
            arrays=[np.empty(n,dtype=np.float64) for _ in range(3)]
            buffers=libe57.VectorSourceDestBuffer()
            for name,array in zip(("cartesianX","cartesianY","cartesianZ"),arrays):
                buffers.append(libe57.SourceDestBuffer(data.image_file,name,array,n,True,True))
            read=header.points.reader(buffers)
            count=read.read()
            read.close()
            if count!=n:
                raise ValueError("unexpected count from secondary E57 reader")
            for index,v in refs.items():
                if index>=count:
                    continue
                raw=np.array([x[index] for x in arrays])
                p=header.rotation_matrix@raw+header.translation
                record["samples_validated"]+=1
                record["max_raw_coordinate_error_m"]=max(record["max_raw_coordinate_error_m"],np.max(abs(raw-v[:3])))
                record["max_world_coordinate_error_m"]=max(record["max_world_coordinate_error_m"],np.max(abs(p-v[3:])))
            if not record["samples_validated"] or record["max_world_coordinate_error_m"]>1e-8:
                raise ValueError(f"E57 secondary crosscheck failed: {record}")
            results.append(record)
    output.write_text(json.dumps(results,indent=2)+"\n")


if __name__=="__main__":
    p=argparse.ArgumentParser()
    for n in ("e57","cpp_audit","output"):
        p.add_argument(n,type=Path)
    a=p.parse_args()
    audit(a.e57,a.cpp_audit,a.output)
