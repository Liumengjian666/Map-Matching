#!/usr/bin/env python3
"""Bounded official HTTP ranges, four connections, exact size/MD5, no auth workaround."""
from pathlib import Path
import concurrent.futures,hashlib,json,os,time,urllib.request,zipfile
P=Path(__file__).resolve().parents[1];D=Path('/home/jian/rosbag/public_datasets/koide_p62_r29')
record=json.loads((P/'sources/zenodo_record.json').read_text());files={x['key']:x for x in record['files']}
reserve=20*1024**3;chunk=16*1024**2
receipts=json.loads((P/'logs/download_receipts.json').read_text());requests=[]
for name in ['indoor_easy_01.zip','indoor_hard_01.zip']:
 x=files[name];size=x['size'];part=D/(name+'.part');target=D/name;start=time.monotonic()
 if not target.exists():
  old=part.stat().st_size if part.exists() else 0
  assert old<=size
  st=os.statvfs(D);assert st.f_bavail*st.f_frsize>reserve+(size-old)
  fd=os.open(str(part),os.O_RDWR|os.O_CREAT,0o664)
  def fetch(bounds):
   first,last=bounds;url='https://zenodo.org/records/10122133/files/'+name+'?download=1'
   for attempt in range(2):
    try:
     request=urllib.request.Request(url,headers={'Range':'bytes=%d-%d'%(first,last)})
     with urllib.request.urlopen(request,timeout=60) as r:
      assert r.status==206 and r.headers.get('Content-Range')=='bytes %d-%d/%d'%(first,last,size)
      b=r.read(last-first+2);assert len(b)==last-first+1
     assert os.pwrite(fd,b,first)==len(b)
     return {'first':first,'last':last,'bytes':len(b),'attempt':attempt+1}
    except Exception:
     if attempt:raise
   raise RuntimeError('unreachable')
  ranges=[(i,min(i+chunk,size)-1) for i in range(old,size,chunk)]
  with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
   for index,result in enumerate(pool.map(fetch,ranges)):
    requests.append({'file':name,**result})
    (P/'logs/range_download.json').write_text(json.dumps(requests,indent=2)+'\n')
    if (index+1)%4==0:print(name,'completed chunks',index+1,'/',len(ranges),flush=True)
    st=os.statvfs(D);assert st.f_bavail*st.f_frsize>reserve
  os.close(fd)
  assert part.stat().st_size==size
  part.rename(target)
 md5=hashlib.md5();sha=hashlib.sha256()
 with target.open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):md5.update(b);sha.update(b)
 assert 'md5:'+md5.hexdigest()==x['checksum'], 'publisher MD5 mismatch'
 receipt={'path':str(target),'bytes':size,'md5':md5.hexdigest(),'sha256':sha.hexdigest(),'publisher_md5_pass':True,'seconds':time.monotonic()-start}
 directory=D/name[:-4];assert not directory.exists()
 with zipfile.ZipFile(target) as z:
  total=sum(i.file_size for i in z.infolist());assert total<10*1024**3
  st=os.statvfs(D);assert st.f_bavail*st.f_frsize>reserve+total
  assert all(not i.filename.startswith('/') and '..' not in Path(i.filename).parts for i in z.infolist())
  receipt['uncompressed_bytes']=total;receipt['members']=[{'name':i.filename,'bytes':i.file_size,'crc':i.CRC} for i in z.infolist()]
  z.extractall(directory)
 receipts.append(receipt);(P/'logs/download_receipts.json').write_text(json.dumps(receipts,indent=2)+'\n')
 print('VERIFIED_AND_EXTRACTED',name,size,total,flush=True)
