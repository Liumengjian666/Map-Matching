#!/usr/bin/env python3
"""Record exact author blobs, local observation diff, binary and build identity."""
import difflib
import hashlib
import json
from pathlib import Path
import subprocess

HERE=Path(__file__).resolve().parent
ROOT=Path('/home/jian/livox_ws/p61_r5_geode_runs')
AUTHOR=ROOT/'author_ws/src/FAST_LIO'


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    files={
        'CMakeLists.txt':ROOT/'upstream/CMakeLists.txt',
        'include/ikd-Tree/ikd_Tree.h':ROOT/'upstream/ikd_Tree.h',
        'include/ikd-Tree/ikd_Tree.cpp':ROOT/'upstream/ikd_Tree.cpp',
        'src/laserMapping.cpp':HERE.parent/'p61_r4_geode_prior_map_feasibility/official/fastlio_laserMapping.cpp',
    }
    patch=[]
    for name, original in files.items():
        patch.extend(difflib.unified_diff(original.read_text().splitlines(True),
            (AUTHOR/name).read_text().splitlines(True),fromfile='a/FAST_LIO/'+name,tofile='b/FAST_LIO/'+name))
    (HERE/'author_observation.patch').write_text(''.join(patch))
    rows=[]
    tree=json.loads((HERE.parent/'p61_r4_geode_prior_map_feasibility/official/evaluation_master_tree.json').read_text())
    expected={t['path']:t['sha'] for t in tree['tree'] if t['type']=='blob'}
    for p in sorted(AUTHOR.rglob('*')):
        if not p.is_file() or 'Log' in p.parts or 'PCD' in p.parts:continue
        b=p.read_bytes();name='FAST_LIO/'+str(p.relative_to(AUTHOR))
        blob=hashlib.sha1(b'blob '+str(len(b)).encode()+b'\0'+b).hexdigest()
        rows.append(dict(path=name,bytes=len(b),sha256=sha(p),git_blob=blob,upstream_blob=expected.get(name)))
    binary=ROOT/'author_ws/devel/lib/fast_lio/fastlio_mapping'
    result=dict(author_commit='1f008a7249e36393a1752622de50660b77b5b7f4',
                repository='https://github.com/thisparticle/GEODE_Evaluation',license='GPL-2.0',files=rows,
                binary=str(binary),binary_sha256=sha(binary),binary_bytes=binary.stat().st_size,
                observation_header_sha256=sha(HERE/'p61_record.hpp'),
                patch_sha256=sha(HERE/'author_observation.patch'),
                compiler=subprocess.check_output(['g++','--version'],text=True).splitlines()[0],
                cmake=subprocess.check_output(['cmake','--version'],text=True).splitlines()[0])
    (HERE/'author_build_manifest.json').write_text(json.dumps(result,indent=2)+'\n')
    print(result['binary_sha256'])


if __name__=='__main__':main()
