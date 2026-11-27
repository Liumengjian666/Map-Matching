"""Summarize execution phases, export portable figures and index the raw local archive."""
from pathlib import Path
import argparse
import csv
import json
import shutil

from freeze import PAIRS, digest, read, write

def main(run,out):
    summary=[]
    raw=[]
    for frame in PAIRS:
        for phase in ['nominal','perturb']:
            rows=read(run/f'{frame}_{phase}'/'timing.csv')
            raw.extend(rows)
            for name in ['score_s','oracle_s','info_write_s','load_model_s']:
                summary.append(dict(frame=frame,stage=f'{phase}_{name}',wall_s=sum(float(r[name]) for r in rows),
                                    evaluations=len(rows) if name=='score_s' else '',max_rss_kb=''))
    for kind in ['observation','neighborhood','patch']:
        for r in read(out/f'runtime_{kind}.csv'):
            summary.append(dict(**r,evaluations='',max_rss_kb=''))
    for r in read(run/'blind/blind_runtime.csv'):
        for field in ['selection_s','cached_score_and_info_s']:
            summary.append(dict(frame=f"{r['frame']}/{r['pair']}",stage=field,wall_s=r[field],
                                evaluations=r['mask_count'],max_rss_kb=''))
    for path in sorted(run.glob('*_time.txt')):
        phase=path.stem[:-5]
        tokens=dict(x.split('=',1) for x in path.read_text().split() if '=' in x)
        if not tokens:
            continue
        summary.append(dict(frame='all or named above',stage=f'process_{phase}',wall_s=tokens['wall_s'],
                            evaluations='',max_rss_kb=tokens['max_rss_kb']))
    write(out/'runtime_breakdown.csv',summary)
    write(out/'native_fixed_score_audit.csv',raw)
    for name in ['geometry_difference_results.csv','H307_geometry_classification.png']:
        shutil.copyfile(run/'geometry'/name,out/name)
    assets=out/'point_clouds'
    assets.mkdir(exist_ok=False)
    for path in sorted((run/'geometry').glob('*.pcd')):
        shutil.copyfile(path,assets/path.name)
    shutil.copyfile(run/'H307_native_support_sign.pcd',assets/'H307_native_support_sign.pcd')
    files=[]
    for path in sorted(run.rglob('*')):
        if path.is_file() and path.name != '.gitignore':
            files.append(dict(relative_to_run=path.relative_to(run),bytes=path.stat().st_size,sha256=digest(path)))
    write(out/'artifact_sha256.csv',files)
    local=out/'blind_seal.json'
    shutil.copyfile(run/'blind/blind_seal.json',local)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path)
    p.add_argument('out',type=Path)
    a=p.parse_args()
    main(a.run,a.out)
