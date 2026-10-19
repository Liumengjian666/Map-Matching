#!/usr/bin/python3
"""Finite primary-source capture only; no sensor recordings or localization."""
import hashlib,json,time,urllib.request,urllib.error
from pathlib import Path
from urllib.parse import urlsplit,urlunsplit

ROOT=Path('/home/jian/livox_ws/p62_r1_source_cache')
SOURCES=[
 ('park2024.html','https://onlinelibrary.wiley.com/doi/full/10.1002/rob.22392'),
 ('chiang2023.html','https://researchoutput.ncku.edu.tw/en/publications/high-definition-map-based-lidar-localization-through-dynamic-time/'),
 ('ica2025.html','https://arxiv.org/html/2408.11809v2'),
 ('oce2026.html','https://link.springer.com/article/10.1007/s43684-026-00144-1'),
 ('dali_readme.md','https://raw.githubusercontent.com/DCSI2022/DALI_SLAM/main/README.md'),
 ('dali_parameters.md','https://raw.githubusercontent.com/DCSI2022/DALI_SLAM/main/Parameter_Descrip.md'),
 ('dali_tree.json','https://api.github.com/repos/DCSI2022/DALI_SLAM/git/trees/main?recursive=1'),
 ('multi_consensus2024.html','https://link.springer.com/article/10.1186/s13634-024-01153-z'),
 ('regscorer_readme.md','https://raw.githubusercontent.com/WHU-USI3DV/RegScorer/main/readme.md'),
 ('superloc.html','https://superodometry.com/superloc.html'),
 ('coral2022.pdf','https://arxiv.org/pdf/2205.05975'),
 ('xicp.pdf','https://arxiv.org/pdf/2211.16335'),
 ('genz.pdf','https://arxiv.org/pdf/2411.06766'),
 ('hilti2022.pdf','https://arxiv.org/pdf/2208.09825'),
 ('hilti_page.html','https://hilti-challenge.com/dataset-2022'),
 ('hilti_readme.md','https://raw.githubusercontent.com/Hilti-Research/hilti-slam-challenge-2022/main/README.md'),
 ('hf_info.json','https://huggingface.co/api/datasets/Hilti-Research/hilti-slam-challenge-2022'),
 ('hf_tree.json','https://huggingface.co/api/datasets/Hilti-Research/hilti-slam-challenge-2022/tree/main?recursive=true&expand=false'),
 ('hilti_lidar_calibration.yaml','https://huggingface.co/datasets/Hilti-Research/hilti-slam-challenge-2022/resolve/main/calibration/calibration_files/lidar_calibration.yaml'),
 ('nclt_page.html','https://robots.engin.umich.edu/nclt/index.html'),
 ('nclt.pdf','https://robots.engin.umich.edu/nclt/nclt.pdf'),
]


def capture(entries,receipt):
    ROOT.mkdir(exist_ok=True);records=[]
    for name,url in entries:
        target=ROOT/name
        if target.exists():raise RuntimeError('NEVER_OVERWRITE:'+str(target))
        row=dict(name=name,url=url,attempts=1,path=str(target));start=time.monotonic()
        try:
            request=urllib.request.Request(url,headers={'User-Agent':'ResearchSourceAudit/1.0'})
            with urllib.request.urlopen(request,timeout=25) as response:
                body=response.read(16*1024*1024+1)
                if len(body)>16*1024*1024:raise ValueError('SOURCE_SIZE_BOUND')
                if name.endswith('.pdf') and not body.startswith(b'%PDF-'):raise ValueError('NOT_PDF')
                if name.endswith('.json'):json.loads(body)
                with target.open('xb') as f:f.write(body)
                final=urlsplit(response.url)
                row.update(status=response.status,bytes=len(body),sha256=hashlib.sha256(body).hexdigest(),
                    content_type=response.headers.get('Content-Type'),final_url=urlunsplit((final.scheme,final.netloc,final.path,'','')))
        except (urllib.error.URLError,ValueError,TimeoutError) as e:
            row.update(status='ACCESS_FAILED',error=str(e),http_status=getattr(e,'code',None))
        row['wall_sec']=time.monotonic()-start;records.append(row);print(json.dumps(row),flush=True)
    with (ROOT/receipt).open('x') as f:json.dump(records,f,indent=2);f.write('\n')
    return records


if __name__=='__main__':capture(SOURCES,'capture_initial.json')
