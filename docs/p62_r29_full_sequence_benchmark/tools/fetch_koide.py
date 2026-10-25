#!/usr/bin/env python3
"""Only five approved Zenodo assets; exact size cap, reserve, publisher MD5."""
from pathlib import Path
import hashlib,json,os,time,urllib.request,zipfile
HERE=Path(__file__).resolve().parents[1]
DEST=Path('/home/jian/rosbag/public_datasets/koide_p62_r29')
DEST.mkdir(parents=True,exist_ok=True)
record=json.loads((HERE/'sources/zenodo_record.json').read_text())
wanted=['gt.zip','map_indoor_easy.ply','map_indoor_hard.ply','indoor_easy_01.zip','indoor_hard_01.zip']
files={x['key']:x for x in record['files']}
reserve=20*1024**3
receipts=[]
for name in wanted:
 x=files[name];target=DEST/name;part=DEST/(name+'.part');start=time.monotonic()
 if not target.exists():
  if part.exists():raise RuntimeError('partial asset exists; explicit review required: '+str(part))
  n=0
  with urllib.request.urlopen(x['links']['self'],timeout=45) as response,part.open('xb') as out:
   while True:
    b=response.read(1024*1024)
    if not b:break
    n+=len(b)
    if n>x['size']:raise RuntimeError('publisher size cap exceeded')
    st=os.statvfs(DEST)
    if st.f_bavail*st.f_frsize < reserve+len(b):raise RuntimeError('20GiB reserve reached')
    out.write(b)
  if n!=x['size']:raise RuntimeError('incomplete download')
  part.rename(target)
 md5=hashlib.md5();sha=hashlib.sha256()
 with target.open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):md5.update(b);sha.update(b)
 assert target.stat().st_size==x['size'] and 'md5:'+md5.hexdigest()==x['checksum']
 r={'path':str(target),'bytes':x['size'],'md5':md5.hexdigest(),'sha256':sha.hexdigest(),'publisher_md5_pass':True,'seconds':time.monotonic()-start}
 receipts.append(r);(HERE/'logs/download_receipts.json').write_text(json.dumps(receipts,indent=2)+'\n')
 print('VERIFIED',name,x['size'],r['seconds'],flush=True)
 if name.endswith('.zip'):
  directory=DEST/name[:-4]
  if directory.exists():raise RuntimeError('refuse existing extraction directory')
  with zipfile.ZipFile(target) as z:
   total=sum(a.file_size for a in z.infolist())
   assert total<10*1024**3
   st=os.statvfs(DEST);assert st.f_bavail*st.f_frsize>reserve+total
   assert all(not a.filename.startswith('/') and '..' not in Path(a.filename).parts for a in z.infolist())
   r['uncompressed_bytes']=total;r['members']=[{'name':a.filename,'bytes':a.file_size,'crc':a.CRC} for a in z.infolist()]
   z.extractall(directory)
  (HERE/'logs/download_receipts.json').write_text(json.dumps(receipts,indent=2)+'\n')
  print('EXTRACTED',name,total,flush=True)
