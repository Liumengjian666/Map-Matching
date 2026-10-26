#!/usr/bin/env python3
"""Read public metadata and bounded ZIP directory tails; never downloads sensor payloads."""
import hashlib, json, shutil, struct, time
from pathlib import Path
import requests
P=Path(__file__).resolve().parents[1]
URL='https://zenodo.org/api/records/10122133'
cap=2*1024**2
r=requests.get(URL,stream=True,timeout=(15,30)); r.raise_for_status()
raw=bytearray()
for block in r.iter_content(65536):
 raw.extend(block)
 if len(raw)>cap: r.close(); raise RuntimeError('metadata capacity exceeded')
r.close(); record=json.loads(raw)
(P/'sources/zenodo_record_current.json').write_bytes(raw)
selected=[]
for f in record['files']:
 if f['key'] not in ['outdoor_hard_01a.zip','outdoor_hard_01b.zip','outdoor_hard_02a.zip','outdoor_hard_02b.zip','map_outdoor_hard.ply','gt.zip']:continue
 item={'name':f['key'],'bytes':f['size'],'checksum':f['checksum'],'url':f['links']['self']}
 if f['key'].endswith('.zip') and f['key']!='gt.zip':
  size=f['size']; tail_size=131072; start=size-tail_size
  try:
   q=requests.get(f['links']['self'],headers={'Range':f'bytes={start}-{size-1}','Accept-Encoding':'identity'},stream=True,timeout=(15,30))
   if q.status_code!=206: q.close(); raise RuntimeError('server did not return HTTP206; refusing whole payload')
   if not q.headers.get('Content-Range','').startswith(f'bytes {start}-{size-1}/'):q.close();raise RuntimeError('range identity mismatch')
   data=bytearray(); began=time.monotonic()
   for block in q.iter_content(32768):
    data.extend(block)
    if len(data)>tail_size or time.monotonic()-began>45:q.close();raise RuntimeError('bounded range limit')
   q.close(); assert len(data)==tail_size
   e=data.rfind(b'PK\x05\x06'); assert e>=0
   sig,disk,diskcd,ndisk,n,cd_size,cd_offset,comment=struct.unpack_from('<4s4H2IH',data,e)
   assert disk==0 and diskcd==0 and n==ndisk and cd_size<tail_size and cd_offset>=start
   pos=cd_offset-start;members=[]
   for _ in range(n):
    v=struct.unpack_from('<4s6H3I5H2I',data,pos); assert v[0]==b'PK\x01\x02'
    name_len,extra_len,comment_len=v[10:13]
    name=bytes(data[pos+46:pos+46+name_len]).decode('utf-8')
    members.append({'name':name,'compressed_bytes':v[8],'uncompressed_bytes':v[9],'crc32':v[7]})
    pos+=46+name_len+extra_len+comment_len
   item['directory_only_members']=members
   item['uncompressed_bytes']=sum(m['uncompressed_bytes'] for m in members)
   item['range_bytes_read']=len(data)
   (P/'sources'/(f['key']+'.directory_tail.bin')).write_bytes(data)
  except Exception as error:item['directory_status']='UNAVAILABLE: '+str(error)
 selected.append(item)
usage=shutil.disk_usage('/home/jian/rosbag')
external=shutil.disk_usage('/media/jian/HIKVISION')
receipt={'retrieved_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),'metadata_url':URL,'metadata_sha256':hashlib.sha256(raw).hexdigest(),'selected_files':selected,'archive_total_bytes':sum(x['bytes'] for x in selected),'storage':{'root_free_bytes':usage.free,'external_free_bytes':external.free,'reserve_bytes':20*1024**3},'raw_sensor_payload_downloaded':False,'only_metadata_and_zip_directory_tails':True}
(P/'sources/OUTDOOR_ASSET_METADATA.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps(receipt,indent=2))
