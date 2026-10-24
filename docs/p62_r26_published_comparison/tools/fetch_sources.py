#!/usr/bin/env python3
"""Fetch public papers/documentation only, never code/data packages; bounded bytes."""
import urllib.request,json,hashlib,time
from pathlib import Path
D=Path('/home/jian/livox_ws/p62_r26_published_baseline_sources');D.mkdir(exist_ok=True)
SOURCES=[
 ('2fast_publisher.html','https://journals.sagepub.com/doi/10.1177/02783649261451003'),
 ('2fast_v2.pdf','https://arxiv.org/pdf/2410.05433v2'),
 ('steam_author.pdf','https://asrl.utias.utoronto.ca/~tdb/bib/burnett_tro24.pdf'),
 ('singer_publisher.pdf','https://www.cambridge.org/core/services/aop-cambridge-core/content/view/444D7B057346B86E789561DDA5FC305A/S0263574724002121a.pdf/imu-as-an-input-versus-a-measurement-of-the-state-in-inertial-aided-state-estimation.pdf'),
 ('fastlio2_author.pdf','https://jiaronglin.com/uploads/Fast_LIO_2.pdf'),
 ('superloc.pdf','https://arxiv.org/pdf/2412.02901v2'),
 ('range_localization_author.pdf','https://staff.aist.go.jp/k.koide/assets/pdf/icra2024_02.pdf'),
 ('dlio_readme.md','https://raw.githubusercontent.com/vectr-ucla/direct_lidar_inertial_odometry/main/README.md'),
 ('2fast_readme.md','https://raw.githubusercontent.com/clegenti/2fast2lamaa/main/README.md'),
 ('fastlio_loc_readme.md','https://raw.githubusercontent.com/HViktorTsoi/FAST_LIO_LOCALIZATION/main/README.md')]
manifest=[];total=0
for name,url in SOURCES:
 dest=D/name
 if dest.exists():raise SystemExit('Exclusive source path already exists: '+str(dest))
 begin=time.monotonic();m={'name':name,'requested_url':url,'retrieved_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())}
 try:
  with urllib.request.urlopen(urllib.request.Request(url,headers={'User-Agent':'Mozilla/5.0 research literature verification'}),timeout=25) as response:
   m.update(final_url=response.geturl(),http_status=response.status,content_type=response.headers.get('Content-Type'))
   with dest.open('xb') as f:
    n=0
    while True:
     block=response.read(65536)
     if not block:break
     n+=len(block);total+=len(block)
     if n>20*1024**2 or total>200*1024**2 or time.monotonic()-begin>60:raise RuntimeError('paper download byte/time budget exceeded')
     f.write(block)
  data=dest.read_bytes();m.update(bytes=len(data),sha256=hashlib.sha256(data).hexdigest(),pdf_magic=data.startswith(b'%PDF-'))
  if name.endswith('.pdf') and not m['pdf_magic']:raise RuntimeError('not a real PDF')
  m['status']='FETCHED'
 except Exception as e:m.update(status='FAILED',error=str(e))
 manifest.append(m);print(name,m['status'],m.get('bytes'),m.get('error'),flush=True)
 (D/'SOURCE_FETCH_MANIFEST.json').write_text(json.dumps(manifest,indent=2)+'\n')
