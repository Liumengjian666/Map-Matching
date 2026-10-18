#!/usr/bin/env python3
"""Freeze R6 build identity and a reviewable minimal diff from the R5 author copy."""
import difflib
import json
from pathlib import Path
from prepare_bridge import sha,ROOT

HERE=Path(__file__).resolve().parent
R5=Path('/home/jian/livox_ws/p61_r5_geode_runs/author_ws/src/FAST_LIO')
R6=ROOT/'author_ws/src/FAST_LIO'


def main():
    patch=[];files=[]
    for p in sorted(R6.rglob('*')):
        if not p.is_file() or any(k in p.parts for k in ['Log','PCD']):continue
        name=p.relative_to(R6)
        files.append(dict(path=str(name),sha256=sha(p),bytes=p.stat().st_size))
        old=R5/name
        if old.exists() and sha(old)!=sha(p):
            patch.extend(difflib.unified_diff(old.read_text().splitlines(True),p.read_text().splitlines(True),
                fromfile='a/FAST_LIO/'+str(name),tofile='b/FAST_LIO/'+str(name)))
    (HERE/'r6_author.patch').write_text(''.join(patch))
    binary=ROOT/'author_ws/devel/lib/fast_lio/fastlio_mapping'
    result=dict(upstream_author_commit='1f008a7249e36393a1752622de50660b77b5b7f4',
        R5_source_archive_sha256='023586059f21dbecc1dd80c600fb0122b442e88891e52e2f66033e258c2c4b71',
        binary=str(binary),binary_sha256=sha(binary),source_files=files,
        patch_sha256=sha(HERE/'r6_author.patch'),production_math_changed=False,
        description='Shared unchanged native IMU integration; terminal no-LiDAR wrapper; exact raw arrival watermark; research export paths.')
    (HERE/'build_manifest.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='source_files'},indent=2))


if __name__=='__main__':main()
